// WhoDatESP — receives nearby-contact list from WhoDat phone app via BLE
//
// Key build settings (IMPORTANT):
//   PSRAM: DISABLED  — OPI PSRAM conflicts with BLE DMA buffers on ESP32-S3
//   Flash: QIO 80MHz, 32MB
//   Partition: 32M Flash (4.8MB APP/22MB FATFS)
//
// Libraries:
//   Arduino_GFX, Arduino_DriveBus, SensorLib



#include <Arduino.h>
#include "ESP_I2S.h"
extern "C" {
#include "es8311.h"
}
#include "weather_icons.h"
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include <Wire.h>
#include "HWCDC.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "TouchDrvFT6X36.hpp"
#include "SensorQMI8658.hpp"
#include <Preferences.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>

#define NUS_SERVICE_UUID  "6E400001-B5B3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5B3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5B3-F393-E0A9-E50E24DCCA9E"

// Colors (defined here in case the GFX library version omits them)
#ifndef BLACK
#define BLACK 0x0000
#endif
#ifndef WHITE
#define WHITE 0xFFFF
#endif
#define PINK  0xFB56   // hot pink — used for Shop button when items need buying

// Display layout constants — built-in glcdfont at textSize=2 (12x16 per char)
#define LINE_H       18    // 16px tall + 2px leading
#define CHAR_W       12    // 6px * textSize=2
#define CHAR_DESCENT 16    // full char height, cursor is top-left in built-in font
#define LIST_START_Y 73    // pushed down ~2 old rows from top
#define DIST_COL_W   72    // right-column pixels reserved for up to 6-char distance at textSize=2
#define MARGIN       10
#define CORNER_R     112   // panel corner radius in pixels (calibrated)
#define STATUS_Y     90    // safe y for top status row — clear of corners
#define WEATHER_ICON_W  62
#define WEATHER_ICON_H  54
#define WEATHER_GAP      1
#define WEATHER_Y        44  // top of hourly weather strip (10px below top row)
#define CAL_LINE_H       20  // pixels per calendar display line
#define FACE_INFO_TOP  218                                // top of info panel (below time)
#define FACE_INFO_MID  ((FACE_INFO_TOP + BTN_Y - 2) / 2) // midpoint divider ~342

HWCDC USBSerial;


// ── PMIC ───────────────────────────────────────────────────────────────────────
#define AXP2101_ADDR 0x34
uint8_t cachedBattPct = 0xFF;  // 0xFF = not yet read

// ── IMU ────────────────────────────────────────────────────────────────────────
SensorQMI8658  imu;
uint32_t       stepCount       = 0;
unsigned long  lastStepMs      = 0;
int            lastHour        = -1;
#define STEP_POLL_MS  1000   // battery + midnight-reset check interval

// Software pedometer
#define SW_PEDO_MS    40     // accelerometer poll interval (25Hz)
#define STEP_THRESH   0.15f  // dynamic magnitude threshold (g)
#define MIN_STEP_MS   300    // debounce — ignore peaks closer than this
#define MAX_STEP_MS   2000   // reset cadence if no step within this window
#define DC_ALPHA      0.05f  // low-pass coefficient for gravity removal
float         swDcMag        = 1.0f;
bool          swStepAbove    = false;
unsigned long swLastStepMs   = 0;
unsigned long swPedoLastMs   = 0;

// ── Touch ──────────────────────────────────────────────────────────────────────
TouchDrvFT6X36 touch;
volatile bool touchDetected = false;

void IRAM_ATTR onTouchInterrupt() {
  touchDetected = true;
}

// ── Display ────────────────────────────────────────────────────────────────────
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_GFX *gfx = new Arduino_CO5300(
  bus, LCD_RESET, 0 /* rotation */,
  LCD_WIDTH, LCD_HEIGHT,
  22 /* col_offset1 */, 0, 0, 0);

// fillCircle workaround: CO5300 QSPI driver drops h=1 writes.
// Draw circle as 2-row fillRect pairs — always h=2, never h=1.
void fillCircleFixed(int16_t cx, int16_t cy, int16_t r, uint16_t color) {
  float r2 = (float)r * r;
  for (int16_t dy = -r; dy <= r; dy += 2) {
    int16_t dx0 = (int16_t)sqrtf(max(0.0f, r2 - (float)dy * dy));
    int16_t dx1 = (dy + 1 <= r) ? (int16_t)sqrtf(max(0.0f, r2 - (float)(dy+1)*(dy+1))) : 0;
    int16_t dx = max(dx0, dx1);
    gfx->fillRect(cx - dx, cy + dy, 2 * dx + 1, 2, color);
  }
}

// ── BLE state ──────────────────────────────────────────────────────────────────
BLECharacteristic *pTxChar    = nullptr;
volatile bool deviceConnected = false;
volatile bool messageUpdated  = false;
int           lostSeconds     = 0;      // beep if disconnected for this many seconds (0=off)
unsigned long lostTimerStartMs = 0;     // millis() at disconnect; 0 = timer inactive

#define MAX_HOME_ITEMS  8
#define HOME_ITEM_LEN   64
char homeItems[MAX_HOME_ITEMS][HOME_ITEM_LEN + 1];  // "label|abbrev|compact|dist"
int  homeItemCount = 0;
String pendingMessage         = "";

// Receive buffer — accumulates chunks until "\nEND" is received
String bleBuffer    = "";
bool   listReady    = false;
String contactList  = "";   // parse buffer — replaced on every BLE message
String contactData  = "";   // display copy — only updated from full payloads
char   weatherData[25] = "";  // 24 hourly category digits (0–6), null-terminated
int8_t weatherTemp[24] = {};  // 24 hourly temperatures in °C

#define MAX_CAL_EVENTS  20
#define CAL_LINE_LEN    27
char   calLines[MAX_CAL_EVENTS][CAL_LINE_LEN + 1];
int8_t calDays [MAX_CAL_EVENTS];   // day offset: 0=today, 1=tomorrow, …
int    calLineCount = 0;

// ── Scroll state ───────────────────────────────────────────────────────────────
int            scrollOffset    = 0;
int            calScrollOffset = 0;
int            faceCalZoneBot  = 330;  // divider Y between cal/contact zones; updated each draw
bool           wasTouching   = false;
int16_t        touchStartX   = -1;
int16_t        touchStartY   = -1;
int16_t        touchLastX    = -1;
int16_t        touchLastY    = -1;
unsigned long  lastTouchMs   = 0;
unsigned long  findFeedbackMs = 0;
#define TIME_Y          150   // top of the HH:MM:SS block
#define BTN_Y           (LCD_HEIGHT - BTN_H - 4)  // flush to bottom edge
#define BTN_H            30   // button height
#define BTN_IGAP          6   // gap between adjacent buttons
#define BTN_W           ((LCD_WIDTH - 2*MARGIN - 2*BTN_IGAP) / 3)  // fills full row
#define SWIPE_THRESHOLD  40   // px movement to count as a swipe
#define TOUCH_RELEASE_MS 120   // ms silence = finger lifted
#define DOUBLE_TAP_MS    400   // max gap between taps to count as double-tap
#define SLEEP_MS         30000 // light sleep after this many ms with no touch
#define PWR_DOUBLETAP_MS 500   // max ms between button presses for double-press power-off
unsigned long  lastTapMs       = 0;
bool           screenOn        = true;
unsigned long  screenActivityMs = 0;
#define VISIBLE_ROWS     ((LCD_HEIGHT - LIST_START_Y) / LINE_H)
#define SCROLL_ROWS      (VISIBLE_ROWS * 2 / 3)

// ── Watch mode ─────────────────────────────────────────────────────────────────
enum WatchMode { MODE_WATCH_FACE, MODE_CONTACT_LIST, MODE_TAKE_LIST, MODE_BUY_LIST, MODE_CAL_LIST };
WatchMode     currentMode         = MODE_WATCH_FACE;
static uint32_t calAlertedMask    = 0;   // bit i = event[i] alerted today
bool          watchFaceFullRedraw = true;
int           lastWatchSecond     = -1;
unsigned long lastInteractionMs   = 0;
#define INACTIVITY_MS 10000

// ── PCF85063 RTC (I2C 0x51, shares bus with touch controller) ─────────────────
#define RTC_ADDR 0x51

static uint8_t rtcToBCD(int v)       { return ((v / 10) << 4) | (v % 10); }
static int     rtcFromBCD(uint8_t b) { return ((b >> 4) * 10) + (b & 0x0F); }

bool rtcCheckValid() {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x04);  // PCF85063A: Seconds register is at 0x04 (not 0x02)
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom((uint8_t)RTC_ADDR, (uint8_t)1);
  return Wire.available() && !(Wire.read() & 0x80);  // OS bit clear = time valid
}

// wday/day/mon/yr = -1 → don't update date registers
void rtcSet(int h, int m, int s, int wday = -1, int day = -1, int mon = -1, int yr = -1) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x00);
  Wire.write(0x20);  // STOP=1 — halt clock before writing time registers
  Wire.endTransmission();
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x04);  // PCF85063A: Seconds at 0x04
  Wire.write(rtcToBCD(s) & 0x7F);
  Wire.write(rtcToBCD(m) & 0x7F);
  Wire.write(rtcToBCD(h) & 0x3F);
  Wire.endTransmission();
  if (day >= 1 && mon >= 1) {
    Wire.beginTransmission(RTC_ADDR);
    Wire.write(0x07);  // PCF85063A: Days at 0x07
    Wire.write(rtcToBCD(day) & 0x3F);
    Wire.write((uint8_t)(wday & 0x07));
    Wire.write(rtcToBCD(mon) & 0x1F);
    Wire.write(rtcToBCD(yr % 100));
    Wire.endTransmission();
  }
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x00);
  Wire.write(0x00);  // STOP=0 — restart clock, 24h mode
  Wire.endTransmission();
}

// Read date registers (0x07-0x0A): day, weekday, month, year
void rtcGetDate(int &wday, int &day, int &mon, int &yr) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x07);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)RTC_ADDR, (uint8_t)4);
  if (Wire.available() < 4) { wday = day = mon = yr = 0; return; }
  day  = rtcFromBCD(Wire.read() & 0x3F);
  wday = Wire.read() & 0x07;
  mon  = rtcFromBCD(Wire.read() & 0x1F);
  yr   = rtcFromBCD(Wire.read()) + 2000;
}

void rtcGet(int &h, int &m, int &s) {
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x04);  // PCF85063A: Seconds at 0x04
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)RTC_ADDR, (uint8_t)3);
  s = rtcFromBCD(Wire.read() & 0x7F);
  m = rtcFromBCD(Wire.read() & 0x7F);
  h = rtcFromBCD(Wire.read() & 0x3F);
}

// ── Time tracking ──────────────────────────────────────────────────────────────
bool hasTime = false;

// ── Font diagnostic ───────────────────────────────────────────────────────
// Call debugFont() once from setup() to check GFXfont rendering.
// Put your font file in a subfolder fonts/ and #include "fonts/YourFont.h"
// so Arduino IDE doesn't auto-compile it. Then uncomment the three lines below.
void debugFont() {
  // #include won't work here — include the font at file top, then:
  // const GFXfont *testFont = &FreeSans12pt7b;  // ← your font
  // For now this tests the coordinate path with the built-in font,
  // confirming text IS visible at various Y positions.

  gfx->fillScreen(BLACK);

  // Draw reference bands so we can see where each row lives
  for (int y = 0; y < LCD_HEIGHT; y += 50) {
    gfx->fillRect(0, y, LCD_WIDTH, 24, (y / 50 % 2) ? 0x001F : 0x7800); // blue/red stripes
    gfx->setTextSize(1);
    gfx->setTextColor(WHITE);
    gfx->setCursor(4, y + 4);
    gfx->printf("y=%d", y);
  }

  delay(3000);   // pause so you can photograph / read the screen

  // Now test setTextSize(3) text at each band — confirm it appears inside
  gfx->fillScreen(BLACK);
  for (int row = 0; row < 5; row++) {
    int y = row * 50;
    gfx->fillRect(0, y, LCD_WIDTH, 48, (row % 2) ? 0x001F : 0x7800);
    gfx->setTextSize(3);
    gfx->setTextColor(WHITE);
    gfx->setCursor(4, y);
    gfx->printf("Row %d y=%d", row, y);
  }
  delay(3000);

  // Serial: report display dimensions so we can sanity-check constants
  USBSerial.printf("LCD %d x %d, LINE_H=%d VISIBLE_ROWS=%d\n",
    LCD_WIDTH, LCD_HEIGHT, LINE_H, VISIBLE_ROWS);

  gfx->fillScreen(BLACK);
  gfx->fillRect(0, 80, LCD_WIDTH, 80, 0x001F);   // blue reference box y=80..160
  gfx->setTextSize(2);
  gfx->setTextColor(WHITE);
  gfx->setCursor(10, 100);
  gfx->print("ABCabc 123");

  int16_t bx, by; uint16_t bw, bh;
  gfx->getTextBounds("ABCabc 123", 10, 100, &bx, &by, &bw, &bh);
  USBSerial.printf("Bounds: x=%d y=%d w=%d h=%d\n", bx, by, bw, bh);
  delay(5000);
}

// ── Corner-profile calibration ────────────────────────────────────────────────
// Shows 10px grid + colored edge strips (offset-coded) + predicted corner curve.
// Colored strips (3px wide) from each edge: red=0 orange=5 yellow=10 green=15
//   cyan=20 blue=25 magenta=30.  White 3×2 dashes mark the CORNER_R=95 prediction.
// Photograph the screen, then compare where strips disappear vs the white curve.
// Tap to dismiss.
// White fill makes AMOLED off-pixels appear black, revealing the physical corner
// curve directly.  Three colored predicted boundaries (R=75 red, R=95 green,
// R=115 blue) let you see which radius matches the actual white-black transition.
void drawCornerCalibration() {
  gfx->fillScreen(WHITE);
  gfx->setFont(nullptr);
  gfx->setTextWrap(false);

  // Dark grid every 10px so you can count pixels
  for (int x = 0; x < LCD_WIDTH;  x += 10) gfx->drawFastVLine(x, 0, LCD_HEIGHT, 0x39E7);
  for (int y = 0; y < LCD_HEIGHT; y += 10) gfx->drawFastHLine(0, y, LCD_WIDTH,  0x39E7);

  // Four predicted corner curves — 4px-wide dashes every 3 rows
  // Visible curve = radius > actual; invisible = inside clipped zone
  const int      testR[4]  = { 100,   105,   110,   115   };
  const uint16_t Rcols[4]  = {0xF800, 0xFFE0, 0x07E0, 0x001F}; // red, yellow, green, blue
  for (int ri = 0; ri < 4; ri++) {
    int      R = testR[ri];
    uint16_t c = Rcols[ri];
    for (int y = 0; y <= R; y += 3) {
      int dy   = R - y;
      int safe = R - (int)sqrtf((float)(R * R - dy * dy));
      // top-left and top-right
      gfx->fillRect(safe,                  y,                  4, 3, c);
      gfx->fillRect(LCD_WIDTH - safe - 4,  y,                  4, 3, c);
      // bottom-left and bottom-right
      gfx->fillRect(safe,                  LCD_HEIGHT - y - 3, 4, 3, c);
      gfx->fillRect(LCD_WIDTH - safe - 4,  LCD_HEIGHT - y - 3, 4, 3, c);
    }
    // Legend label
    char buf[8]; snprintf(buf, sizeof(buf), "R=%d", R);
    gfx->setTextSize(2);
    gfx->setTextColor(c, WHITE);
    gfx->setCursor(LCD_WIDTH / 2 - 75 + ri * 45, LCD_HEIGHT / 2 - 8);
    gfx->print(buf);
  }

  // Y-coordinate labels every 20px in safe centre column
  gfx->setTextSize(1);
  gfx->setTextColor(BLACK, WHITE);
  for (int y = 0; y < LCD_HEIGHT; y += 20) {
    gfx->setCursor(LCD_WIDTH / 2 + 20, y + 1);
    char buf[5]; snprintf(buf, sizeof(buf), "%-3d", y);
    gfx->print(buf);
  }
  // X-coordinate labels at mid-height
  for (int x = 50; x < LCD_WIDTH - 50; x += 50) {
    gfx->setCursor(x - 6, LCD_HEIGHT / 2 + 14);
    char buf[5]; snprintf(buf, sizeof(buf), "%d", x);
    gfx->print(buf);
  }
}

// ── Status screen ─────────────────────────────────────────────────────────────
void drawScreen(const String &msg, bool connected) {
  gfx->fillScreen(BLACK);
  gfx->setFont((const GFXfont*)nullptr);

  gfx->setTextSize(1);
  gfx->setTextColor(connected ? 0x07E0 : 0xFFE0);
  gfx->setCursor(MARGIN, 8);
  gfx->print(connected ? "BLE Connected" : "BLE Advertising...");

  gfx->drawFastHLine(0, 24, LCD_WIDTH, 0x4208);

  gfx->setTextWrap(true);
  gfx->setTextSize(3);
  gfx->setTextColor(WHITE);
  gfx->setCursor(MARGIN, 44);
  gfx->print(msg);
  gfx->setTextWrap(false);
}

// ── Top row: step count | date | battery — equal maximal spacing ──────────────
void drawTopRow() {
  static const char* const DOW_STR[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  static const char* const MON_STR[] = {"","Jan","Feb","Mar","Apr","May","Jun",
                                         "Jul","Aug","Sep","Oct","Nov","Dec"};

  // Build each string using actual width (no padding) so textSize=3 fits more often
  char stepBuf[8], battBuf[6];
  char dowBuf[4] = "", dayBuf[3] = "", monBuf[4] = "";
  snprintf(stepBuf, sizeof(stepBuf), "%lu", (unsigned long)stepCount);

  {
    int wday, day, mon, yr;
    rtcGetDate(wday, day, mon, yr);
    if (mon >= 1 && mon <= 12) {
      snprintf(dowBuf, sizeof(dowBuf), "%s", DOW_STR[wday&7]);
      snprintf(dayBuf, sizeof(dayBuf), "%d", day);
      snprintf(monBuf, sizeof(monBuf), "%s", MON_STR[mon]);
    }
  }

  battBuf[0] = '\0';
  if (cachedBattPct <= 100)
    snprintf(battBuf, sizeof(battBuf), "%d%%", cachedBattPct);

  const bool hasDate = dowBuf[0] != '\0';
  const bool hasBatt = battBuf[0] != '\0';
  // date pixel width: three parts separated by 2px gaps (no space characters)
  const int DATE_GAP = 2;
  auto dateChars = [&]() { return (int)(strlen(dowBuf) + strlen(dayBuf) + strlen(monBuf)); };
  auto datePx    = [&](int cw) { return hasDate ? dateChars()*cw + 2*DATE_GAP : 0; };

  // Corner-safe row bounds
  const int by  = MARGIN;
  const int dy  = CORNER_R - by;
  const int sqr = (int)sqrtf((float)(CORNER_R*CORNER_R - dy*dy));
  const int x0  = CORNER_R - sqr + 2;
  const int x1  = (LCD_WIDTH - CORNER_R) + sqr - 2;
  const int availW = x1 - x0;

  // Choose font size: try 3, fall back to 2 if any gap would be < 2px
  int nGaps = (hasDate ? 1 : 0) + (hasBatt ? 1 : 0);
  auto calcGap = [&](int cw) -> int {
    int tw = (int)strlen(stepBuf)*cw + datePx(cw)
           + (hasBatt ? (int)strlen(battBuf)*cw : 0);
    return nGaps > 0 ? (availW - tw) / nGaps : (availW - tw);
  };

  int ts = 3;
  if (calcGap(ts * 6) < 2) ts = 2;

  const int cw  = ts * 6;
  const int gap = max(0, calcGap(cw));

  // Clear the row only when a string changes length (positions shift) or font size changes.
  // Between those events, in-place background-colour redraw is flash-free.
  static char prevDow[4] = "", prevDay[3] = "", prevMon[4] = "";
  static char prevStep[8]  = "";
  static char prevBatt[6]  = "";
  static int  prevTs       = -1;
  if (ts != prevTs
      || strlen(stepBuf) != strlen(prevStep)
      || strlen(dayBuf)  != strlen(prevDay)
      || strlen(battBuf) != strlen(prevBatt)) {
    gfx->fillRect(x0, by, availW, 3*8, BLACK);
  }
  strcpy(prevStep, stepBuf);
  strcpy(prevDow,  dowBuf);
  strcpy(prevDay,  dayBuf);
  strcpy(prevMon,  monBuf);
  strcpy(prevBatt, battBuf);
  prevTs = ts;

  // Field widths
  const int sw = (int)strlen(stepBuf) * cw;
  const int dw = datePx(cw);
  const int bw = hasBatt ? (int)strlen(battBuf) * cw : 0;

  // X positions: left-to-right with equal gaps
  const int xStep = x0 + 2;
  const int xDate = xStep + sw + gap;
  const int xBatt = hasDate ? (xDate + dw + gap) : (xStep + sw + gap);

  // Battery colour
  uint16_t battCol = (cachedBattPct > 50) ? 0x07E0
                   : (cachedBattPct > 20) ? 0xFFE0
                                          : 0xF800;

  gfx->setFont(nullptr);
  gfx->setTextWrap(false);
  gfx->setTextSize(ts);

  gfx->setTextColor(0x07FF, BLACK);   // step: cyan
  gfx->setCursor(xStep, by);
  gfx->print(stepBuf);

  if (hasDate) {
    gfx->setTextColor(0xC618, BLACK);
    int xd = xDate;
    gfx->setCursor(xd, by); gfx->print(dowBuf); xd += (int)strlen(dowBuf)*cw + DATE_GAP;
    gfx->setCursor(xd, by); gfx->print(dayBuf); xd += (int)strlen(dayBuf)*cw + DATE_GAP;
    gfx->setCursor(xd, by); gfx->print(monBuf);
  }

  if (hasBatt) {
    gfx->setTextColor(battCol, BLACK);
    gfx->setCursor(xBatt, by);
    gfx->print(battBuf);
  }
}

// ── Weather icons (GFX primitives, 46×40 box) ─────────────────────────────────
// cat: 0=clear 1=partly-cloudy 2=cloudy 3=fog 4=rain 5=snow 6=thunder
// current=true → full brightness; false → dimmed to ~1/3
void drawWeatherIcon(int ix, int iy, int cat, bool current) {
  if (cat < 0 || cat > 6) cat = 0;
  const uint16_t* bmp = weather_icons[cat];
  if (current) {
    gfx->draw16bitRGBBitmap(ix, iy, (uint16_t*)bmp, WEATHER_ICON_W, WEATHER_ICON_H);
  } else {
    // dimmed — draw pixel by pixel at 1/3 brightness
    for (int dy = 0; dy < WEATHER_ICON_H; dy++) {
      for (int dx = 0; dx < WEATHER_ICON_W; dx++) {
        uint16_t c = pgm_read_word(bmp + dy * WEATHER_ICON_W + dx);
        if (c) {
          c = (uint16_t)((((c >> 11) & 0x1F) / 3) << 11 |
                         (((c >>  5) & 0x3F) / 3) <<  5 |
                         (( c        & 0x1F) / 3));
        }
        gfx->writePixel(ix + dx, iy + dy, c);
      }
    }
  }
}


// 6 icons for current hour + next 5, centred across the screen with hour labels
void drawWeatherStrip() {
  if (weatherData[0] == '\0') return;
  int h, m, s;
  getCurrentTime(h, m, s);

  const int N      = 6;
  const int totalW = N * WEATHER_ICON_W + (N-1) * WEATHER_GAP;
  const int startX = (LCD_WIDTH - totalW) / 2;
  const int labelY = WEATHER_Y + WEATHER_ICON_H + 2;

  for (int i = 0; i < N; i++) {
    int hour = (h + i) % 24;
    int cat  = (weatherData[hour] >= '0' && weatherData[hour] <= '6')
               ? (weatherData[hour] - '0') : 0;
    int ix   = startX + i * (WEATHER_ICON_W + WEATHER_GAP);

    gfx->fillRect(ix, WEATHER_Y, WEATHER_ICON_W, WEATHER_ICON_H + 40, BLACK);

    drawWeatherIcon(ix, WEATHER_Y, cat, true);

    gfx->setFont(nullptr);
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE, BLACK);
    // hour label — 12h clock with a/p suffix
    char label[5];
    int h12 = hour % 12; if (h12 == 0) h12 = 12;
    snprintf(label, sizeof(label), "%d%c", h12, hour < 12 ? 'a' : 'p');
    int lblW = strlen(label) * 12;
    gfx->setCursor(ix + (WEATHER_ICON_W - lblW) / 2, labelY);
    gfx->print(label);
    // temperature
    char tmpBuf[6];
    snprintf(tmpBuf, sizeof(tmpBuf), "%d", (int)weatherTemp[hour]);
    int tmpW = strlen(tmpBuf) * 12;
    gfx->setCursor(ix + (WEATHER_ICON_W - tmpW) / 2, labelY + 18);
    gfx->print(tmpBuf);
  }
}

// ── Take list ─────────────────────────────────────────────────────────────────
#define MAX_TAKE_ITEMS  24
#define TAKE_LINE_LEN   30
char takeItems[MAX_TAKE_ITEMS][TAKE_LINE_LEN + 1];
int  takeItemCount = 0;

static const char* const DEFAULT_TAKE_ITEMS[] = {
  "Keys", "Wallet", "Phone", "Headphones",
  "Charger", "Water", "Umbrella"
};

// ── Buy list (Shopping Kismet from Android Keep/Tasks) ────────────────────────
#define MAX_BUY_ITEMS   200
#define BUY_LINE_LEN    30
char buyItems[MAX_BUY_ITEMS][BUY_LINE_LEN + 1];
int  buyItemCount     = 0;
int  buyNeedCount     = 0;    // items 0..buyNeedCount-1 are "need"; rest are "bought"
int     buyScrollOffset    = 0;
int     buyHighlightIdx    = -1;   // index of item currently under finger (-1 = none)
int16_t buyDragBaseY       = -1;   // Y at touch-down for real-time scroll; -1 = not dragging
int     buyDragStartOffset = 0;    // buyScrollOffset at touch-down
bool buyItemBought[MAX_BUY_ITEMS];
bool buyLoading       = false;

// Sort: need items (bought=false) alphabetically first, bought items alphabetically second.
void repartitionBuyItems() {
  // Insertion sort by (bought ASC, name ASC) — list is tiny so O(n²) is fine
  for (int i = 1; i < buyItemCount; i++) {
    char keyName[BUY_LINE_LEN + 1];
    bool keyBought = buyItemBought[i];
    strncpy(keyName, buyItems[i], BUY_LINE_LEN);
    keyName[BUY_LINE_LEN] = '\0';
    int j = i - 1;
    while (j >= 0) {
      bool swap;
      if (buyItemBought[j] != keyBought)
        swap = buyItemBought[j] && !keyBought;       // bought before need → swap
      else
        swap = strcasecmp(buyItems[j], keyName) > 0; // same group, wrong alpha → swap
      if (!swap) break;
      strncpy(buyItems[j + 1], buyItems[j], BUY_LINE_LEN);
      buyItemBought[j + 1] = buyItemBought[j];
      j--;
    }
    strncpy(buyItems[j + 1], keyName, BUY_LINE_LEN);
    buyItemBought[j + 1] = keyBought;
  }
  buyNeedCount = 0;
  for (int i = 0; i < buyItemCount; i++)
    if (!buyItemBought[i]) buyNeedCount++;
}

// textSize(4) for buy list items: 24px/char wide, 32px tall, 36px per row
#define BUY_ITEM_H  36
#define CHKMARK_W   24   // px reserved left of item text for checkmark + gap

// 3px-thick checkmark drawn in a 16×20px box with top-left at (x, y)
void drawCheckmark(int x, int y, uint16_t color) {
  for (int t = 0; t < 3; t++) {
    gfx->drawLine(x,     y+8+t, x+6,  y+18+t, color);  // short left arm
    gfx->drawLine(x+6,   y+18+t, x+16, y+t,   color);  // long right arm
  }
}

void drawBuyList() {
  gfx->fillScreen(BLACK);
  gfx->setFont(nullptr);
  gfx->setTextWrap(false);

  // Header — textSize(2), green
  gfx->setTextSize(2);
  gfx->setTextColor(0x07E0, BLACK);
  const char* hdr = "-- SHOPPING --";
  gfx->setCursor((LCD_WIDTH - (int)strlen(hdr) * 12) / 2, MARGIN + 4);
  gfx->print(hdr);

  if (buyItemCount == 0) {
    gfx->setTextColor(0x8410, BLACK);
    gfx->setCursor(MARGIN + 4, LIST_START_Y);
    gfx->print(buyLoading ? "Fetching..." : "No items");
    return;
  }

  // Items — textSize(4)
  gfx->setTextSize(4);
  const int maxVisible = (LCD_HEIGHT - BTN_H - 8 - LIST_START_Y) / BUY_ITEM_H;
  int end = min(buyScrollOffset + maxVisible, buyItemCount);
  int y   = LIST_START_Y;

  for (int i = buyScrollOffset; i < end; i++) {
    int safeL = MARGIN + 2;
    if (y < CORNER_R) {
      int dy = CORNER_R - y;
      safeL = max(safeL, CORNER_R - (int)sqrtf((float)(CORNER_R*CORNER_R - dy*dy)) + 2);
    }
    {
      int yFromBot = LCD_HEIGHT - (y + BUY_ITEM_H);
      if (yFromBot < CORNER_R) {
        int dy2 = CORNER_R - yFromBot;
        safeL = max(safeL, CORNER_R - (int)sqrtf((float)(CORNER_R*CORNER_R - dy2*dy2)) + 2);
      }
    }
    bool hl     = (i == buyHighlightIdx);
    bool bought = buyItemBought[i];
    // Separator line at top of bought section
    if (i == buyNeedCount && buyNeedCount > 0 && buyNeedCount < buyItemCount)
      gfx->drawFastHLine(0, y, LCD_WIDTH, 0x4208);
    if (hl) {
      gfx->fillRect(0, y, LCD_WIDTH, BUY_ITEM_H, WHITE);
      gfx->setTextColor(BLACK, WHITE);
    } else {
      gfx->setTextColor(bought ? 0x8410 : WHITE, BLACK);
    }
    if (bought) drawCheckmark(safeL + 2, y + (BUY_ITEM_H - 20) / 2, hl ? 0x0320 : 0x07E0);
    gfx->setCursor(safeL + CHKMARK_W, y);
    gfx->print(buyItems[i]);
    y += BUY_ITEM_H;
  }

  // Scroll indicator — textSize(1), bottom-right of header row
  if (buyItemCount > maxVisible) {
    gfx->setTextSize(1);
    gfx->setTextColor(0x8410, BLACK);
    char ind[12];
    snprintf(ind, sizeof(ind), "%d/%d", buyScrollOffset / maxVisible + 1,
             (buyItemCount + maxVisible - 1) / maxVisible);
    gfx->setCursor(LCD_WIDTH - (int)strlen(ind) * 6 - MARGIN, MARGIN + 6);
    gfx->print(ind);
  }

  // Three buttons: Need | List | Back
  gfx->setFont(nullptr); gfx->setTextWrap(false); gfx->setTextSize(2);
  static const char* const BUY_BTN_LABELS[3] = { "Need", "List", "Back" };
  for (int i = 0; i < 3; i++) {
    int bx = menuBtnX(i);
    uint16_t fill = (i == 0 && buyNeedCount > 0) ? PINK : 0x4208;
    gfx->fillRoundRect(bx, BTN_Y, BTN_W, BTN_H, 8, fill);
    gfx->drawRoundRect(bx,     BTN_Y,     BTN_W,     BTN_H,     8, WHITE);
    gfx->drawRoundRect(bx + 1, BTN_Y + 1, BTN_W - 2, BTN_H - 2, 7, 0xC618);
    int tw   = strlen(BUY_BTN_LABELS[i]) * 12;
    int xOff = (i == 0) ? 12 : (i == 2) ? -12 : 0;
    gfx->setTextColor(WHITE, fill);
    gfx->setCursor(bx + (BTN_W - tw) / 2 + xOff, BTN_Y + (BTN_H - 16) / 2);
    gfx->print(BUY_BTN_LABELS[i]);
  }
}

// Repaint a single buy-list row without touching the rest of the screen.
void drawBuyItem(int idx, bool highlighted) {
  if (idx < 0 || idx < buyScrollOffset) return;
  const int maxVisible = (LCD_HEIGHT - BTN_H - 8 - LIST_START_Y) / BUY_ITEM_H;
  int pos = idx - buyScrollOffset;
  if (pos >= maxVisible) return;
  int y     = LIST_START_Y + pos * BUY_ITEM_H;
  int safeL = MARGIN + 2;
  if (y < CORNER_R) {
    int dy = CORNER_R - y;
    safeL = max(safeL, CORNER_R - (int)sqrtf((float)(CORNER_R*CORNER_R - dy*dy)) + 2);
  }
  {
    int yFromBot = LCD_HEIGHT - (y + BUY_ITEM_H);
    if (yFromBot < CORNER_R) {
      int dy2 = CORNER_R - yFromBot;
      safeL = max(safeL, CORNER_R - (int)sqrtf((float)(CORNER_R*CORNER_R - dy2*dy2)) + 2);
    }
  }
  gfx->setFont(nullptr);
  gfx->setTextWrap(false);
  gfx->setTextSize(4);
  bool bought = buyItemBought[idx];
  if (highlighted) {
    gfx->fillRect(0, y, LCD_WIDTH, BUY_ITEM_H, WHITE);
    gfx->setTextColor(BLACK, WHITE);
  } else {
    gfx->fillRect(0, y, LCD_WIDTH, BUY_ITEM_H, BLACK);
    gfx->setTextColor(bought ? 0x8410 : WHITE, BLACK);
  }
  if (bought) drawCheckmark(safeL + 2, y + (BUY_ITEM_H - 20) / 2, highlighted ? 0x0320 : 0x07E0);
  gfx->setCursor(safeL + CHKMARK_W, y);
  gfx->print(buyItems[idx]);
}

void loadTakeList() {
  Preferences p;
  p.begin("whodat", true);
  String stored = p.getString("takelist", "");
  p.end();

  if (stored.length() == 0) {
    takeItemCount = sizeof(DEFAULT_TAKE_ITEMS) / sizeof(DEFAULT_TAKE_ITEMS[0]);
    for (int i = 0; i < takeItemCount && i < MAX_TAKE_ITEMS; i++) {
      strncpy(takeItems[i], DEFAULT_TAKE_ITEMS[i], TAKE_LINE_LEN);
      takeItems[i][TAKE_LINE_LEN] = '\0';
    }
  } else {
    takeItemCount = 0;
    int start = 0;
    while (takeItemCount < MAX_TAKE_ITEMS) {
      int nl = stored.indexOf('\n', start);
      String line = (nl < 0) ? stored.substring(start) : stored.substring(start, nl);
      line.trim();
      if (line.length() > 0) {
        line.toCharArray(takeItems[takeItemCount], sizeof(takeItems[takeItemCount]));
        takeItemCount++;
      }
      if (nl < 0) break;
      start = nl + 1;
    }
  }
}

void drawTakeList() {
  gfx->fillScreen(BLACK);
  gfx->setFont(nullptr);
  gfx->setTextWrap(false);
  gfx->setTextSize(2);

  // Header
  gfx->setTextColor(0x07FF, BLACK);  // cyan
  const char* hdr = "-- TAKE --";
  gfx->setCursor((LCD_WIDTH - (int)strlen(hdr) * 12) / 2, MARGIN + 4);
  gfx->print(hdr);

  // Items
  gfx->setTextColor(WHITE, BLACK);
  int y = LIST_START_Y;
  for (int i = 0; i < takeItemCount; i++) {
    if (y + CHAR_DESCENT > LCD_HEIGHT - BTN_H - 12) break;
    int safeL = MARGIN + 4;
    if (y < CORNER_R) {
      int dy = CORNER_R - y;
      safeL = max(safeL, CORNER_R - (int)sqrtf((float)(CORNER_R*CORNER_R - dy*dy)) + 2);
    }
    gfx->setCursor(safeL, y);
    gfx->print("> ");
    gfx->print(takeItems[i]);
    y += LINE_H + 2;
  }

  // Exit button — centred at bottom
  gfx->setFont(nullptr); gfx->setTextWrap(false); gfx->setTextSize(2);
  int bx = menuBtnX(1);
  gfx->fillRoundRect(bx, BTN_Y, BTN_W, BTN_H, 8, 0x4208);
  gfx->drawRoundRect(bx,     BTN_Y,     BTN_W,     BTN_H,     8, WHITE);
  gfx->drawRoundRect(bx + 1, BTN_Y + 1, BTN_W - 2, BTN_H - 2, 7, 0xC618);
  const char* el = "Exit";
  gfx->setTextColor(WHITE, 0x4208);
  gfx->setCursor(bx + (BTN_W - (int)strlen(el) * 12) / 2, BTN_Y + (BTN_H - 16) / 2);
  gfx->print(el);
}

// Returns total virtual display lines for the calendar (day headers + event lines).
static int calTotalVLines() {
  int n = 0, lastDay = -1;
  for (int i = 0; i < calLineCount; i++) {
    if (calDays[i] != lastDay) { n++; lastDay = calDays[i]; }
    n++;
  }
  return n;
}

void drawCalList() {
  gfx->fillScreen(BLACK);
  gfx->setFont(nullptr);
  gfx->setTextWrap(false);

  gfx->setTextSize(2);
  gfx->setTextColor(0x07FF, BLACK);
  const char* hdr = "-- CALENDAR --";
  gfx->setCursor((LCD_WIDTH - (int)strlen(hdr) * 12) / 2, MARGIN + 4);
  gfx->print(hdr);

  if (calLineCount == 0) {
    gfx->setTextColor(0x8410, BLACK);
    gfx->setCursor(MARGIN + 4, LIST_START_Y);
    gfx->print("No events");
  } else {
    int wdCal, dCal, mCal, yCal;
    rtcGetDate(wdCal, dCal, mCal, yCal);

    static const char* const DOW[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
    static const char* const MON[] = {"","Jan","Feb","Mar","Apr","May","Jun",
                                       "Jul","Aug","Sep","Oct","Nov","Dec"};
    auto dowFn = [](int d, int m, int y) -> int {
      static const int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
      if (m < 3) y--;
      return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
    };

    const int safeL     = MARGIN + 4;
    const int calBottom = BTN_Y - 4;
    int       dispY     = LIST_START_Y;
    int       vLine     = 0;
    int       lastDay   = -1;

    for (int i = 0; i < calLineCount; i++) {
      if (calDays[i] != lastDay) {
        lastDay = calDays[i];
        if (vLine >= calScrollOffset) {
          if (dispY + CAL_LINE_H > calBottom) break;
          struct tm ev = {};
          ev.tm_year = yCal - 1900; ev.tm_mon = mCal - 1;
          ev.tm_mday = dCal + calDays[i]; ev.tm_hour = 12;
          mktime(&ev);
          int dow = dowFn(ev.tm_mday, ev.tm_mon + 1, ev.tm_year + 1900);
          char buf[14];
          snprintf(buf, sizeof(buf), "%s %d %s", DOW[dow & 7], ev.tm_mday, MON[ev.tm_mon + 1]);
          gfx->setTextColor(0x07E0, BLACK);
          gfx->setCursor(safeL, dispY);
          gfx->print(buf);
          gfx->drawFastHLine(safeL, dispY + 17, LCD_WIDTH - 2 * safeL, 0x4208);
          dispY += CAL_LINE_H;
        }
        vLine++;
      }
      if (vLine >= calScrollOffset) {
        if (dispY + CAL_LINE_H > calBottom) break;
        gfx->setTextColor(WHITE, BLACK);
        gfx->setCursor(safeL + 12, dispY);
        gfx->print(calLines[i]);
        dispY += CAL_LINE_H;
      }
      vLine++;
    }
  }

  // Back button — centred at bottom
  gfx->setFont(nullptr); gfx->setTextWrap(false); gfx->setTextSize(2);
  int bx = menuBtnX(1);
  gfx->fillRoundRect(bx, BTN_Y, BTN_W, BTN_H, 8, 0x4208);
  gfx->drawRoundRect(bx,     BTN_Y,     BTN_W,     BTN_H,     8, WHITE);
  gfx->drawRoundRect(bx + 1, BTN_Y + 1, BTN_W - 2, BTN_H - 2, 7, 0xC618);
  const char* el = "Back";
  gfx->setTextColor(WHITE, 0x4208);
  gfx->setCursor(bx + (BTN_W - (int)strlen(el) * 12) / 2, BTN_Y + (BTN_H - 16) / 2);
  gfx->print(el);
}

// Parses "H:MM[ap] title" prefix from a calLines[] entry (12-hour with a/p suffix).
// Returns true and fills h24 (0–23) and min (0–59); false for all-day or parse failure.
static bool parseCalTime(const char* line, int& h24, int& min) {
  int p = 0;
  if (!isdigit((unsigned char)line[p])) return false;
  int h = line[p++] - '0';
  if (isdigit((unsigned char)line[p])) h = h * 10 + (line[p++] - '0');
  if (line[p++] != ':') return false;
  if (!isdigit((unsigned char)line[p]) || !isdigit((unsigned char)line[p+1])) return false;
  min = (line[p] - '0') * 10 + (line[p+1] - '0');
  p += 2;
  char ap = line[p];
  if (ap != 'a' && ap != 'p') return false;
  h24 = (ap == 'a') ? (h == 12 ? 0 : h) : (h == 12 ? 12 : h + 12);
  return true;
}

// ── Three menu buttons: Take | Shop | tbd ────────────────────────────────────
static const char* const MENU_LABELS[2] = { "Take", "Shop" };

static int menuBtnX(int i) {
  return MARGIN + i * (BTN_W + BTN_IGAP);
}

void drawMenuButtons() {
  gfx->setFont(nullptr);
  gfx->setTextWrap(false);
  gfx->setTextSize(2);
  const int w = (LCD_WIDTH - 2 * MARGIN - BTN_IGAP) / 2;  // two buttons fill the row
  for (int i = 0; i < 2; i++) {
    int bx = MARGIN + i * (w + BTN_IGAP);
    bool shopAlert = (i == 1 && buyNeedCount > 0);
    uint16_t fill = shopAlert ? PINK : 0x4208;
    gfx->fillRoundRect(bx, BTN_Y, w, BTN_H, 8, fill);
    gfx->drawRoundRect(bx,     BTN_Y,     w,     BTN_H,     8, WHITE);
    gfx->drawRoundRect(bx + 1, BTN_Y + 1, w - 2, BTN_H - 2, 7, 0xC618);
    int tw  = strlen(MENU_LABELS[i]) * 12;
    int xOff = (i == 0) ? 12 : 0;  // Take nudged right to clear left corner curve
    gfx->setTextColor(WHITE, fill);
    gfx->setCursor(bx + (w - tw) / 2 + xOff, BTN_Y + (BTN_H - 16) / 2);
    gfx->print(MENU_LABELS[i]);
  }
}

// Returns 0/1 for Take/Shop, or -1 if miss.
// Excludes the bottom-corner shadow zones using the display's physical corner radius.
int hitMenuButton(int16_t tx, int16_t ty) {
  if (ty < BTN_Y || ty > BTN_Y + BTN_H) return -1;
  const int yFromBottom = LCD_HEIGHT - 1 - ty;
  if (yFromBottom < CORNER_R) {
    const int dy  = CORNER_R - yFromBottom;
    const int sqr = (int)sqrtf((float)(CORNER_R * CORNER_R - dy * dy));
    const int safeLeft  = CORNER_R - sqr;
    const int safeRight = LCD_WIDTH - safeLeft;
    if (tx < safeLeft || tx >= safeRight) return -1;
  }
  const int w = (LCD_WIDTH - 2 * MARGIN - BTN_IGAP) / 2;
  for (int i = 0; i < 2; i++) {
    int bx = MARGIN + i * (w + BTN_IGAP);
    if (tx >= bx && tx < bx + w) return i;
  }
  return -1;
}

// ── Time helpers ──────────────────────────────────────────────────────────────
void getCurrentTime(int &h, int &m, int &s) {
  rtcGet(h, m, s);
}

void drawWatchFace(bool fullRedraw) {
  if (fullRedraw) gfx->fillScreen(BLACK);
  gfx->setFont(nullptr);
  gfx->setTextWrap(false);

  drawTopRow();
  if (fullRedraw) { drawWeatherStrip(); drawMenuButtons(); }

  int h, m, s;
  getCurrentTime(h, m, s);

  // Large HH:MM + small :SS — centred as a unit
  char hhmm[6];
  snprintf(hhmm, sizeof(hhmm), "%02d:%02d", h, m);
  const int TS  = 7;
  const int SS  = 3;
  const int tw  = 5 * TS * 6;          // HH:MM width
  const int th  = TS * 8;
  const int ssW = 3 * SS * 6;          // :SS width (3 chars × 18px)
  const int tx  = (LCD_WIDTH - (tw + 2 + ssW)) / 2;  // centre combined string
  const int ty  = TIME_Y;
  gfx->setTextSize(TS);
  gfx->setTextColor(WHITE, BLACK);
  gfx->setCursor(tx, ty);
  gfx->print(hhmm);

  char secs[4];
  snprintf(secs, sizeof(secs), ":%02d", s);
  gfx->setTextSize(SS);
  gfx->setTextColor(0x8410, BLACK);
  gfx->setCursor(tx + tw + 2, ty + th - SS * 8);
  gfx->print(secs);

  // Connection indicator — green dot when BLE connected, red when not
  fillCircleFixed(tx / 2, ty + th / 2, 10, deviceConnected ? 0x07E0 : 0xF800);

  // Info panel — top half: cal preview; bottom half: WhoDat preview (full redraw only)
  if (fullRedraw) {
    gfx->setFont(nullptr);
    gfx->setTextWrap(false);
    gfx->setTextSize(2);

    const int infoTop = FACE_INFO_TOP;
    const int infoBot = BTN_Y - 2;

    // ── Calendar — today and tomorrow only ────────────────────────
    int calEndY = infoTop;
    if (calLineCount > 0) {
      static const char* const CAL_DOW[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
      int wdayCal, dayCal, monCal, yrCal;
      rtcGetDate(wdayCal, dayCal, monCal, yrCal);

      auto dowFromDate = [](int d, int m, int y) -> int {
        static const int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
        if (m < 3) y--;
        return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
      };

      int lastCalDay = -1;
      int calRow = 0;
      for (int i = 0; i < calLineCount; i++) {
        if (calRow >= 3) break;                               // max 3 event rows
        int lineY = infoTop + calRow * CAL_LINE_H;
        if (lineY + 16 > infoBot - 4 * LINE_H) break;       // always leave room for ≥4 contacts
        gfx->setTextColor(WHITE, BLACK);
        if (calDays[i] != lastCalDay) {
          struct tm ev = {};
          ev.tm_year = yrCal - 1900; ev.tm_mon = monCal - 1;
          ev.tm_mday = dayCal + calDays[i]; ev.tm_hour = 12;
          mktime(&ev);
          int eventWday = dowFromDate(ev.tm_mday, ev.tm_mon + 1, ev.tm_year + 1900);
          gfx->setCursor(MARGIN, lineY);
          gfx->print(CAL_DOW[eventWday]);
          gfx->print(' ');
          lastCalDay = calDays[i];
        } else {
          gfx->setCursor(MARGIN + 4 * CHAR_W, lineY);
        }
        gfx->print(calLines[i]);
        calEndY = lineY + CAL_LINE_H;
        calRow++;
      }
    }

    // Divider placed right after last calendar entry
    faceCalZoneBot = max(calEndY + 4, infoTop + CAL_LINE_H);
    gfx->drawFastHLine(MARGIN, faceCalZoneBot, LCD_WIDTH - 2 * MARGIN, 0x4208);

    // ── WhoDat — rest of space ─────────────────────────────────────
    if (contactData.length() > 0) {
      int y = faceCalZoneBot + 4;
      int start = 0;
      while (y + CHAR_DESCENT <= infoBot) {
        int nl = contactData.indexOf('\n', start);
        if (nl < 0) break;
        String line = contactData.substring(start, nl);
        line.trim();
        start = nl + 1;
        if (line.length() == 0) continue;
        int sep = line.indexOf('|');
        if (sep < 0) continue;
        String name = line.substring(0, sep);
        String dist = line.substring(line.lastIndexOf('|') + 1);
        dist.trim();

        int safeL = MARGIN;
        int yFromBot = LCD_HEIGHT - (y + CHAR_DESCENT);
        if (yFromBot < CORNER_R) {
          int dy2 = CORNER_R - yFromBot;
          safeL = max(safeL, CORNER_R - (int)sqrtf((float)(CORNER_R*CORNER_R - dy2*dy2)) + 2);
        }

        int availChars = (LCD_WIDTH - safeL - DIST_COL_W) / CHAR_W;
        if ((int)name.length() > availChars) name = name.substring(0, availChars - 1) + "~";

        gfx->setTextColor(0xFFE0, BLACK);
        gfx->setCursor(safeL, y);
        gfx->print(name);

        if (dist.length() > 0) {
          int16_t bx2, by2; uint16_t bw2, bh2;
          gfx->getTextBounds(dist, 0, y, &bx2, &by2, &bw2, &bh2);
          int distX = LCD_WIDTH - MARGIN - (int)bw2;
          if (distX > safeL + (int)name.length() * CHAR_W + CHAR_W) {
            gfx->setTextColor(0x07FF, BLACK);
            gfx->setCursor(distX, y);
            gfx->print(dist);
          }
        }
        y += LINE_H;
      }
    }
  }

  // Find-my-phone feedback (shown for 2s after swipe)
  if (findFeedbackMs > 0) {
    gfx->setTextSize(2);
    if (millis() - findFeedbackMs < 2000) {
      gfx->setTextColor(0x07E0, BLACK);
      const char* msg = deviceConnected ? "Finding phone..." : "Not connected";
      int w = strlen(msg) * 12;
      gfx->setCursor((LCD_WIDTH - w) / 2, LCD_HEIGHT - 52);
      gfx->print(msg);
    } else {
      gfx->fillRect(0, LCD_HEIGHT - 52, LCD_WIDTH, 20, BLACK);
      findFeedbackMs = 0;
    }
  }

}

// ── Contact list screen ───────────────────────────────────────────────────────
void drawContactList(const String &payload) {
  gfx->fillScreen(BLACK);
  gfx->setTextWrap(false);

  drawTopRow();

  gfx->setTextSize(2);

  int y = LIST_START_Y;
  int start = 0;
  int lineIndex = 0;
  int linesDrawn = 0;
  bool payloadExhausted = false;

  while (true) {
    int nl = payload.indexOf('\n', start);
    String line = (nl < 0) ? payload.substring(start)
                            : payload.substring(start, nl);
    line.trim();

    if (line.length() > 0) {
      if (lineIndex >= scrollOffset) {
        if (linesDrawn >= 20 || y + CHAR_DESCENT > BTN_Y - 4) break;

        String name, abbrev, compact, dist;
        int sep1 = line.indexOf('|');
        if (sep1 >= 0) {
          // New format: name|abbrev|compact|dist
          int sep2 = line.indexOf('|', sep1 + 1);
          int sep3 = (sep2 >= 0) ? line.indexOf('|', sep2 + 1) : -1;
          name    = line.substring(0, sep1);
          abbrev  = (sep2 >= 0) ? line.substring(sep1 + 1, sep2) : "";
          compact = (sep2 >= 0 && sep3 >= 0) ? line.substring(sep2 + 1, sep3) : "";
          dist    = (sep3 >= 0) ? line.substring(sep3 + 1) : "";
        } else {
          // Old format: name  dist
          int sep = line.indexOf("  ");
          name    = (sep >= 0) ? line.substring(0, sep) : line;
          abbrev  = "";
          compact = "";
          dist    = (sep >= 0) ? line.substring(sep + 2) : "";
        }
        dist.trim();

        int availChars = (LCD_WIDTH - MARGIN - DIST_COL_W) / CHAR_W;
        if ((int)name.length() > availChars)
          name = name.substring(0, availChars - 1) + "~";

        // Pick address tier: full abbrev → compact initials → blank
        String addrToShow = "";
        int remaining = availChars - (int)name.length() - 1;
        if (remaining > 0 && abbrev.length() > 0 && (int)abbrev.length() <= remaining)
          addrToShow = abbrev;
        else if (remaining > 0 && compact.length() > 0 && (int)compact.length() <= remaining)
          addrToShow = compact;

        gfx->setTextColor(0xFFE0, BLACK);
        gfx->setCursor(MARGIN, y);
        gfx->print(name);

        if (addrToShow.length() > 0) {
          int namePixels = (int)name.length() * CHAR_W;
          gfx->setTextColor(WHITE, BLACK);
          gfx->setCursor(MARGIN + namePixels + CHAR_W, y);
          gfx->print(addrToShow);
        }

        if (dist.length() > 0) {
          int16_t bx, by; uint16_t bw, bh;
          gfx->getTextBounds(dist, 0, y, &bx, &by, &bw, &bh);
          int distX = LCD_WIDTH - MARGIN - (int)bw;
          gfx->setTextColor(0x07FF, BLACK);
          gfx->setCursor(distX, y);
          gfx->print(dist);
        }

        y += LINE_H;
        linesDrawn++;
      }
      lineIndex++;
    }

    if (nl < 0) { payloadExhausted = true; break; }
    start = nl + 1;
  }

  // Home addresses — shown after all contacts when there is room
  if (payloadExhausted && homeItemCount > 0 && y + LINE_H <= BTN_Y - 4) {
    // Thin divider
    gfx->drawFastHLine(MARGIN + 4, y + LINE_H / 2, LCD_WIDTH - 2 * MARGIN - 8, 0x2945);
    y += LINE_H;

    gfx->setTextSize(2);
    for (int i = 0; i < homeItemCount && y + CHAR_DESCENT <= BTN_Y - 4; i++) {
      String line = String(homeItems[i]);
      String name, abbrev, compact, dist;
      int sep1 = line.indexOf('|');
      int sep2 = sep1 >= 0 ? line.indexOf('|', sep1 + 1) : -1;
      int sep3 = sep2 >= 0 ? line.indexOf('|', sep2 + 1) : -1;
      name    = sep1 >= 0 ? line.substring(0, sep1) : line;
      abbrev  = sep2 >= 0 ? line.substring(sep1 + 1, sep2) : "";
      compact = (sep2 >= 0 && sep3 >= 0) ? line.substring(sep2 + 1, sep3) : "";
      dist    = sep3 >= 0 ? line.substring(sep3 + 1) : "";
      dist.trim();

      int availChars = (LCD_WIDTH - MARGIN - DIST_COL_W) / CHAR_W;
      if ((int)name.length() > availChars)
        name = name.substring(0, availChars - 1) + "~";

      String addrToShow = "";
      int remaining = availChars - (int)name.length() - 1;
      if (remaining > 0 && abbrev.length() > 0 && (int)abbrev.length() <= remaining)
        addrToShow = abbrev;
      else if (remaining > 0 && compact.length() > 0 && (int)compact.length() <= remaining)
        addrToShow = compact;

      gfx->setTextColor(0x8410, BLACK);
      gfx->setCursor(MARGIN, y);
      gfx->print(name);

      if (addrToShow.length() > 0) {
        gfx->setTextColor(0x4208, BLACK);
        gfx->setCursor(MARGIN + (int)name.length() * CHAR_W + CHAR_W, y);
        gfx->print(addrToShow);
      }

      if (dist.length() > 0) {
        int16_t bx2, by2; uint16_t bw2, bh2;
        gfx->getTextBounds(dist, 0, y, &bx2, &by2, &bw2, &bh2);
        gfx->setTextColor(0x8410, BLACK);
        gfx->setCursor(LCD_WIDTH - MARGIN - (int)bw2, y);
        gfx->print(dist);
      }
      y += LINE_H;
    }
  }

  // Back button — centred at bottom
  gfx->setFont(nullptr); gfx->setTextWrap(false); gfx->setTextSize(2);
  int bx = menuBtnX(1);
  gfx->fillRoundRect(bx, BTN_Y, BTN_W, BTN_H, 8, 0x4208);
  gfx->drawRoundRect(bx,     BTN_Y,     BTN_W,     BTN_H,     8, WHITE);
  gfx->drawRoundRect(bx + 1, BTN_Y + 1, BTN_W - 2, BTN_H - 2, 7, 0xC618);
  const char* bl = "Back";
  gfx->setTextColor(WHITE, 0x4208);
  gfx->setCursor(bx + (BTN_W - (int)strlen(bl) * 12) / 2, BTN_Y + (BTN_H - 16) / 2);
  gfx->print(bl);
}

// ── BLE callbacks ──────────────────────────────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    USBSerial.println("BLE: connected");
    deviceConnected = true;
    messageUpdated  = true;
  }
  void onDisconnect(BLEServer *pSrv) override {
    USBSerial.println("BLE: disconnected");
    deviceConnected = false;
    messageUpdated  = true;
    bleBuffer       = "";   // discard any partial list
    pSrv->startAdvertising();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar, ble_gap_conn_desc *) override {
    String val = pChar->getValue();
    if (val.length() == 0) return;

    USBSerial.printf("RX %d bytes (buf=%d)\n", val.length(), bleBuffer.length());
    bleBuffer += val;

    int endPos = bleBuffer.indexOf("\nEND");
    if (endPos >= 0) {
      contactList = bleBuffer.substring(0, endPos);
      bleBuffer   = "";
      listReady   = true;
      USBSerial.printf("List complete: %d chars\n", contactList.length());
    }
  }
};

// ── Audio ─────────────────────────────────────────────────────────────────────
#define AUDIO_SR   16000
#define AUDIO_VOL  75

static I2SClass i2s;
static bool     audioReady = false;

void audioInit() {
  i2s.setPins(41, 45, 40, 42, 16);  // BCLK, WS, DOUT, DIN, MCLK
  if (!i2s.begin(I2S_MODE_STD, AUDIO_SR, I2S_DATA_BIT_WIDTH_16BIT,
                 I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    USBSerial.println("I2S init failed");
    return;
  }
  es8311_handle_t es = es8311_create(0, ES8311_ADDRRES_0);
  if (!es) { USBSerial.println("ES8311 create failed"); return; }
  const es8311_clock_config_t clk = {
    false, false, true,
    AUDIO_SR * 256,
    AUDIO_SR
  };
  es8311_init(es, &clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
  es8311_sample_frequency_config(es, clk.mclk_frequency, clk.sample_frequency);
  es8311_microphone_config(es, false);
  es8311_voice_volume_set(es, AUDIO_VOL, NULL);
  pinMode(46, OUTPUT);
  digitalWrite(46, HIGH);
  audioReady = true;
  USBSerial.println("Audio ready");
}

// Blocking sine-wave beep with 10ms fade in/out to avoid clicks.
void beep(int freqHz, int durationMs) {
  if (!audioReady) return;
  const int total = (AUDIO_SR * durationMs) / 1000;
  const int CHUNK = 128;
  const int fade  = AUDIO_SR / 100;  // 10ms
  int16_t buf[CHUNK * 2];

  for (int base = 0; base < total; base += CHUNK) {
    int n = min(CHUNK, total - base);
    for (int i = 0; i < n; i++) {
      int   s   = base + i;
      float env = 1.0f;
      if (s < fade)              env = (float)s / fade;
      else if (s > total - fade) env = (float)(total - s) / fade;
      int16_t sample = (int16_t)(sinf(2.0f * M_PI * freqHz * s / AUDIO_SR) * 28000 * env);
      buf[i * 2]     = sample;
      buf[i * 2 + 1] = sample;
    }
    i2s.write((uint8_t*)buf, n * 4);
  }
}

// Bell: sharp attack, exponential decay.
void bell(int freqHz, int durationMs) {
  if (!audioReady) return;
  const int   total   = (AUDIO_SR * durationMs) / 1000;
  const int   CHUNK   = 128;
  const int   attackS = AUDIO_SR / 200;   // 5ms attack
  const float decay   = 6.0f / total;
  int16_t buf[CHUNK * 2];

  for (int base = 0; base < total; base += CHUNK) {
    int n = min(CHUNK, total - base);
    for (int i = 0; i < n; i++) {
      int   s   = base + i;
      float env = expf(-decay * s);
      if (s < attackS) env *= (float)s / attackS;
      int16_t sample = (int16_t)(sinf(2.0f * M_PI * freqHz * s / AUDIO_SR) * 16000 * env);
      buf[i * 2]     = sample;
      buf[i * 2 + 1] = sample;
    }
    i2s.write((uint8_t*)buf, n * 4);
  }
}

// ── Setup ──────────────────────────────────────────────────────────────────────
void setup() {
  USBSerial.begin(115200);
  pinMode(0,  INPUT_PULLUP);   // BOOT button
  pinMode(10, INPUT_PULLUP);   // PWR  button
  esp_wifi_stop();          // Wi-Fi radio unused — saves ~30mA
  setCpuFrequencyMhz(80);   // 240→80MHz; BLE requires ≥80MHz; saves ~25mA
  USBSerial.printf("Free heap:  %u bytes\n", ESP.getFreeHeap());
  USBSerial.printf("PSRAM size: %u bytes  free: %u bytes\n", ESP.getPsramSize(), ESP.getFreePsram());

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  if (!gfx->begin()) {
    USBSerial.println("Display init failed!");
    while (1) delay(100);
  }
  gfx->setTextWrap(false);

  drawScreen("Starting...", false);

  // Touch + RTC share this I2C bus; audio init must come after Wire.begin
  Wire.begin(IIC_SDA, IIC_SCL);
  pinMode(TP_RESET, OUTPUT);
  digitalWrite(TP_RESET, LOW);
  delay(10);
  digitalWrite(TP_RESET, HIGH);
  delay(50);
  if (!touch.begin(Wire, FT6X36_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    USBSerial.println("Touch init failed");
  } else {
    USBSerial.println("Touch ready");
    pinMode(TP_INT, INPUT_PULLUP);
    attachInterrupt(TP_INT, onTouchInterrupt, FALLING);
  }

  audioInit();
  bell(2000, 1200);

  // Ensure PCF85063 clock is running (PMIC reset leaves STOP=1 after power cycle)
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0x00);
  Wire.write(0x00);  // STOP=0, 24h mode
  Wire.endTransmission();
  USBSerial.printf("RTC: %s\n", rtcCheckValid() ? "time valid" : "time unset");
  hasTime = true;  // always show watch face; phone corrects time when it connects

  // Restore last known time + display data from NVS
  {
    Preferences prefs;
    prefs.begin("whodat", true);

    int savedH = prefs.getInt("h", -1);
    int savedM = prefs.getInt("m", 0);
    int savedS = prefs.getInt("s", 0);
    if (savedH >= 0) {
      rtcSet(savedH, savedM, savedS);
      USBSerial.printf("Time restored: %02d:%02d:%02d\n", savedH, savedM, savedS);
    }

    // Weather
    String savedWeather = prefs.getString("weather", "");
    if (savedWeather.length() == 24) {
      savedWeather.toCharArray(weatherData, sizeof(weatherData));
      String tempsStr = prefs.getString("wtemps", "");
      int idx = 0, pos = 0;
      while (idx < 24 && pos < (int)tempsStr.length()) {
        int comma = tempsStr.indexOf(',', pos);
        String t = (comma < 0) ? tempsStr.substring(pos) : tempsStr.substring(pos, comma);
        weatherTemp[idx++] = (int8_t)t.toInt();
        if (comma < 0) break;
        pos = comma + 1;
      }
      USBSerial.println("Weather restored from NVS");
    }

    // Calendar
    int savedCalCnt = prefs.getInt("calcnt", 0);
    if (savedCalCnt > 0) {
      String calStr = prefs.getString("cal", "");
      calLineCount = 0;
      int start = 0;
      while (calLineCount < savedCalCnt && calLineCount < MAX_CAL_EVENTS) {
        int nl = calStr.indexOf('\n', start);
        String token = (nl < 0) ? calStr.substring(start) : calStr.substring(start, nl);
        if (token.length() > 0) {
          int8_t dayOff = 0;
          if (token.length() >= 2 && isDigit((unsigned char)token.charAt(0)) && token.charAt(1) == ':') {
            dayOff = (int8_t)(token.charAt(0) - '0');
            token  = token.substring(2);
          }
          calDays[calLineCount] = dayOff;
          token.toCharArray(calLines[calLineCount], sizeof(calLines[calLineCount]));
          calLineCount++;
        }
        if (nl < 0) break;
        start = nl + 1;
      }
      USBSerial.printf("Calendar restored: %d events\n", calLineCount);
    }

    // Buy list
    int savedBuyCnt = prefs.getInt("buycnt", 0);
    if (savedBuyCnt > 0) {
      String buyStr = prefs.getString("buylist", "");
      buyItemCount = 0; buyNeedCount = 0;
      int bstart = 0;
      while (buyItemCount < savedBuyCnt && buyItemCount < MAX_BUY_ITEMS) {
        int nl = buyStr.indexOf('\n', bstart);
        String token = (nl < 0) ? buyStr.substring(bstart) : buyStr.substring(bstart, nl);
        if (token.startsWith("BUY:") || token.startsWith("GOT:")) {
          bool isBought = token.startsWith("GOT:");
          String item = token.substring(4);
          item.trim();
          if (item.length() > 0) {
            item.toCharArray(buyItems[buyItemCount], sizeof(buyItems[buyItemCount]));
            buyItemBought[buyItemCount] = isBought;
            if (!isBought) buyNeedCount++;
            buyItemCount++;
          }
        }
        if (nl < 0) break;
        bstart = nl + 1;
      }
      USBSerial.printf("Buy list restored: %d items (%d needed)\n", buyItemCount, buyNeedCount);
    }

    // Contact list
    String savedContacts = prefs.getString("contacts", "");
    if (savedContacts.length() > 0) {
      contactData = savedContacts;
      USBSerial.printf("Contacts restored: %d chars\n", contactData.length());
    }

    // Lost-contact beep setting
    lostSeconds = prefs.getInt("lost", 0);
    if (lostSeconds > 0) USBSerial.printf("Lost beep: %ds\n", lostSeconds);

    // Home addresses
    int savedHomeCnt = prefs.getInt("homecnt", 0);
    if (savedHomeCnt > 0) {
      String homeStr = prefs.getString("homelist", "");
      homeItemCount = 0;
      int hstart = 0;
      while (homeItemCount < savedHomeCnt && homeItemCount < MAX_HOME_ITEMS) {
        int nl = homeStr.indexOf('\n', hstart);
        String token = (nl < 0) ? homeStr.substring(hstart) : homeStr.substring(hstart, nl);
        token.trim();
        if (token.length() > 0) {
          token.toCharArray(homeItems[homeItemCount], sizeof(homeItems[homeItemCount]));
          homeItemCount++;
        }
        if (nl < 0) break;
        hstart = nl + 1;
      }
      USBSerial.printf("Home addresses restored: %d\n", homeItemCount);
    }

    prefs.end();
  }

  loadTakeList();

  if (imu.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    imu.configAccelerometer(
      SensorQMI8658::ACC_RANGE_4G,
      SensorQMI8658::ACC_ODR_31_25Hz,  // 62.5Hz → 31.25Hz; sufficient for 25Hz poll
      SensorQMI8658::LPF_MODE_2);
    imu.enableAccelerometer();
    USBSerial.println("IMU ready");
  } else {
    USBSerial.println("IMU init failed");
  }

  // Initial battery read
  Wire.beginTransmission(AXP2101_ADDR);
  Wire.write(0xA4);
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom((uint8_t)AXP2101_ADDR, (uint8_t)1);
    if (Wire.available()) {
      uint8_t v = Wire.read();
      if (v <= 100) { cachedBattPct = v; USBSerial.printf("Battery: %d%%\n", v); }
    }
  }

  BLEDevice::init("Forget-me-not");
  BLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(NUS_SERVICE_UUID);

  pTxChar = pService->createCharacteristic(
    NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTxChar->addDescriptor(new BLE2902());

  BLECharacteristic *pRxChar = pService->createCharacteristic(
    NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE);
  pRxChar->setCallbacks(new RxCallbacks());

  pService->start();

  BLEAdvertising *pAdv = pServer->getAdvertising();
  pAdv->addServiceUUID(NUS_SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->start();

  drawScreen("Open WhoDat\non your phone", false);
  USBSerial.println("Advertising as 'Forget-me-not'");
  screenActivityMs = millis();  // start inactivity countdown from end of boot
}

// ── Loop ───────────────────────────────────────────────────────────────────────
void loop() {
  // Button monitoring — single press = restart, double press = power off
  { static bool         prev0    = (bool)digitalRead(0);
    static bool         prev10   = (bool)digitalRead(10);
    static int          pwrTaps  = 0;
    static unsigned long pwrTapMs = 0;
    bool b0  = digitalRead(0);
    bool b10 = digitalRead(10);
    if (b0  != prev0) { USBSerial.printf("BOOT(GPIO0)  %s\n", b0  ? "released" : "pressed"); prev0  = b0; }
    if (b10 != prev10) {
      USBSerial.printf("PWR(GPIO10)  %s\n", b10 ? "released" : "pressed");
      prev10 = b10;
      if (!b10) {  // falling edge = button pressed (active low)
        if (pwrTaps == 0) {
          pwrTaps  = 1;
          pwrTapMs = millis();
        } else {
          // Second press within window → full power off via AXP2101
          USBSerial.println("Double press — powering off");
          gfx->displayOff();
          delay(50);
          Wire.beginTransmission(AXP2101_ADDR);
          Wire.write(0x10);  // COMMON_CONFIG register
          Wire.write(0x01);  // bit 0 = PWROFF
          Wire.endTransmission();
          delay(500);  // wait for PMIC; fall through to deep sleep if it doesn't respond
          esp_deep_sleep_start();
        }
      }
    }
    // Single-press timeout → restart
    if (pwrTaps == 1 && millis() - pwrTapMs > PWR_DOUBLETAP_MS) {
      pwrTaps = 0;
      USBSerial.println("Single press — restarting");
      delay(50);
      ESP.restart();
    }
  }

  // Touch — tap watch face to open list; swipe list to scroll
  if (touchDetected) {
    touchDetected = false;
    // Screen-off wake: first touch just turns display on, not processed as gesture
    if (!screenOn) {
      screenOn = true;
      screenActivityMs = millis();
      gfx->displayOn();
      watchFaceFullRedraw = true;
    } else {
      screenActivityMs = millis();
      int16_t tx, ty;
      if (touch.getPoint(&tx, &ty, 1)) {
        if (!wasTouching) {
          touchStartX = tx; touchStartY = ty;
          wasTouching = true;
          // Highlight the touched item once at touch-down; leave it until release
          if (currentMode == MODE_BUY_LIST) {
            int newHL = (ty >= LIST_START_Y)
              ? buyScrollOffset + (ty - LIST_START_Y) / BUY_ITEM_H
              : -1;
            if (newHL >= buyItemCount) newHL = -1;
            buyHighlightIdx    = newHL;
            buyDragBaseY       = (ty >= LIST_START_Y) ? ty : -1;
            buyDragStartOffset = buyScrollOffset;
            if (buyHighlightIdx >= 0) drawBuyItem(buyHighlightIdx, true);
          }
        }
        touchLastX  = tx; touchLastY  = ty;
        lastTouchMs = millis();
      } else {
        // Quick tap: interrupt fired but finger already lifted before getPoint was called.
        // Still record it as a zero-delta tap so double-tap detection works.
        if (!wasTouching) {
          touchStartX = touchLastX >= 0 ? touchLastX : LCD_WIDTH / 2;
          touchStartY = touchLastY >= 0 ? touchLastY : LCD_HEIGHT / 2;
          wasTouching = true;
        }
        lastTouchMs = millis();
      }
    }
  }
  if (wasTouching && millis() - lastTouchMs > TOUCH_RELEASE_MS) {
    wasTouching   = false;
    int prevBuyHL = buyHighlightIdx;
    buyHighlightIdx = -1;
    int deltaX = touchLastX - touchStartX;
    int deltaY = touchLastY - touchStartY;

    if (currentMode == MODE_WATCH_FACE && hasTime) {
      if (abs(deltaX) > SWIPE_THRESHOLD && abs(deltaX) > abs(deltaY)) {
        // Horizontal swipe — Find My Phone
        if (deviceConnected && pTxChar) {
          pTxChar->setValue("FIND");
          pTxChar->notify();
        }
        findFeedbackMs    = millis();
        watchFaceFullRedraw = true;
      } else {
        // Tap on BLE connection dot → enter deep sleep
        {
          const int16_t dotX = (LCD_WIDTH - (5*7*6 + 2 + 3*3*6)) / 4;  // tx/2
          const int16_t dotY = TIME_Y + (7*8) / 2;                       // centre of time block
          int16_t ddx = touchLastX - dotX, ddy = touchLastY - dotY;
          if (ddx*ddx + ddy*ddy <= 25*25) {
            // Log battery state before sleeping — low voltage means FT6X36 may lose power
            {
              Wire.beginTransmission(AXP2101_ADDR);
              Wire.write(0xA4);
              if (Wire.endTransmission(false) == 0) {
                Wire.requestFrom((uint8_t)AXP2101_ADDR, (uint8_t)1);
                if (Wire.available()) {
                  uint8_t pct = Wire.read();
                  if (pct <= 100) cachedBattPct = pct;
                }
              }
              // Read VBAT ADC (12-bit, reg 0x78 high 8 bits, 0x79 low 4 bits)
              uint16_t vbatRaw = 0;
              Wire.beginTransmission(AXP2101_ADDR);
              Wire.write(0x78);
              if (Wire.endTransmission(false) == 0) {
                Wire.requestFrom((uint8_t)AXP2101_ADDR, (uint8_t)2);
                if (Wire.available() >= 2) {
                  uint8_t hi = Wire.read();
                  uint8_t lo = Wire.read();
                  vbatRaw = ((uint16_t)hi << 4) | (lo & 0x0F);
                }
              }
              USBSerial.printf("Pre-sleep: batt=%d%% vbat_raw=%u (~%umV)\n",
                cachedBattPct, vbatRaw, vbatRaw);
            }
            USBSerial.println("Sleep dot tapped — entering light sleep");
            gfx->displayOff();
            screenOn = false;
            gpio_wakeup_enable((gpio_num_t)TP_INT, GPIO_INTR_LOW_LEVEL);
            esp_sleep_enable_gpio_wakeup();
            esp_light_sleep_start();
            USBSerial.println("Woke from light sleep");
          }
        }
        // Check for menu button tap first
        int btn = hitMenuButton(touchLastX, touchLastY);
        if (btn >= 0) {
          USBSerial.printf("Menu button: %s\n", MENU_LABELS[btn]);
          if (btn == 0) {
            currentMode       = MODE_TAKE_LIST;
            lastInteractionMs = millis();
            drawTakeList();
          } else if (btn == 1) {
            buyScrollOffset   = 0;
            buyLoading        = deviceConnected;
            currentMode       = MODE_BUY_LIST;
            lastInteractionMs = millis();
            drawBuyList();
            if (deviceConnected && pTxChar) {
              pTxChar->setValue("BUY");
              pTxChar->notify();
            }
          }
        } else {
          // Single tap — zone navigation
          if (touchLastY >= FACE_INFO_TOP && touchLastY < faceCalZoneBot) {
            calScrollOffset   = 0;
            currentMode       = MODE_CAL_LIST;
            lastInteractionMs = millis();
            drawCalList();
          } else if (touchLastY >= faceCalZoneBot && touchLastY < BTN_Y) {
            scrollOffset      = 0;
            currentMode       = MODE_CONTACT_LIST;
            lastInteractionMs = millis();
            gfx->fillScreen(BLACK);
            if (contactData.length() > 0) drawContactList(contactData);
            else drawScreen(deviceConnected ? "Loading contacts..." : "Open WhoDat\non your phone", deviceConnected);
          }
        }
      }
    } else if (currentMode == MODE_TAKE_LIST) {
      lastInteractionMs = millis();
      int bx = menuBtnX(1);
      if (touchLastX >= bx && touchLastX < bx + BTN_W &&
          touchLastY >= BTN_Y && touchLastY < BTN_Y + BTN_H) {
        currentMode         = MODE_WATCH_FACE;
        watchFaceFullRedraw = true;
        lastWatchSecond     = -1;
      }
    } else if (currentMode == MODE_CAL_LIST) {
      lastInteractionMs = millis();
      if (abs(deltaY) > SWIPE_THRESHOLD && abs(deltaY) > abs(deltaX)) {
        lastTapMs = 0;
        const int calVis = (BTN_Y - 4 - LIST_START_Y) / CAL_LINE_H;
        if (deltaY < 0) calScrollOffset = min(calScrollOffset + calVis, max(0, calTotalVLines() - calVis));
        else            calScrollOffset = max(0, calScrollOffset - calVis);
        drawCalList();
      } else if (abs(deltaX) <= SWIPE_THRESHOLD && abs(deltaY) <= SWIPE_THRESHOLD) {
        int bx = menuBtnX(1);
        if (touchLastX >= bx && touchLastX < bx + BTN_W &&
            touchLastY >= BTN_Y && touchLastY < BTN_Y + BTN_H) {
          currentMode         = MODE_WATCH_FACE;
          watchFaceFullRedraw = true;
          lastWatchSecond     = -1;
        }
      }
    } else if (currentMode == MODE_BUY_LIST) {
      lastInteractionMs = millis();
      const int maxVisible = (LCD_HEIGHT - BTN_H - 8 - LIST_START_Y) / BUY_ITEM_H;
      // Gesture classification:
      //   vertical scroll  — deltaY dominant and > 15px
      //   tap              — all movement <= 15px
      //   (horizontal swipe kept for legacy; deltaX dominant and > SWIPE_THRESHOLD)
      if (abs(deltaY) > 15 && abs(deltaY) >= abs(deltaX)) {
        // Vertical gesture — always scroll 9 items in direction of swipe
        if (buyDragBaseY >= 0) {
          int jump = (deltaY < 0) ? 9 : -9;  // flick up (deltaY<0) → increase offset (lower items); flick down → decrease
          int newOff = constrain(buyDragStartOffset + jump, 0, max(0, buyItemCount - maxVisible));
          if (newOff != buyScrollOffset) {
            buyScrollOffset = newOff;
            drawBuyList();
          } else if (prevBuyHL >= 0) {
            drawBuyItem(prevBuyHL, false);
          }
        } else if (prevBuyHL >= 0) {
          drawBuyItem(prevBuyHL, false);
        }
      } else {
        // Tap — buttons, or toggle item
        int btn = -1;
        if (touchLastY >= BTN_Y && touchLastY < BTN_Y + BTN_H) {
          for (int i = 0; i < 3; i++) {
            int bx = menuBtnX(i);
            if (touchLastX >= bx && touchLastX < bx + BTN_W) { btn = i; break; }
          }
        }
        if (btn == 0) {
          buyScrollOffset = 0;
          drawBuyList();
        } else if (btn == 1) {
          buyScrollOffset = min(buyNeedCount, max(0, buyItemCount - maxVisible));
          drawBuyList();
        } else if (btn == 2) {
          currentMode         = MODE_WATCH_FACE;
          watchFaceFullRedraw = true;
          lastWatchSecond     = -1;
        } else if (prevBuyHL >= 0 && prevBuyHL < buyItemCount) {
          // Tap on item — toggle bought state
          if (deviceConnected && pTxChar) {
            String tickMsg = "TICK:";
            tickMsg += buyItems[prevBuyHL];
            pTxChar->setValue(tickMsg.c_str());
            pTxChar->notify();
          }
          buyItemBought[prevBuyHL] = !buyItemBought[prevBuyHL];
          repartitionBuyItems();
          if (buyScrollOffset > 0 && buyScrollOffset >= buyItemCount)
            buyScrollOffset = max(0, buyItemCount - maxVisible);
          drawBuyList();
        }
      }
      buyDragBaseY = -1;
    } else if (currentMode == MODE_CONTACT_LIST) {
      lastInteractionMs = millis();
      if (abs(deltaY) > SWIPE_THRESHOLD && contactData.length() > 0) {
        if (deltaY < 0) scrollOffset += SCROLL_ROWS;
        else            scrollOffset = max(0, scrollOffset - SCROLL_ROWS);
        drawContactList(contactData);
      } else if (abs(deltaX) <= SWIPE_THRESHOLD && abs(deltaY) <= SWIPE_THRESHOLD) {
        int bx = menuBtnX(1);
        if (touchLastX >= bx && touchLastX < bx + BTN_W &&
            touchLastY >= BTN_Y && touchLastY < BTN_Y + BTN_H) {
          currentMode         = MODE_WATCH_FACE;
          watchFaceFullRedraw = true;
          lastWatchSecond     = -1;
        }
      }
    }
    touchStartX = touchStartY = touchLastX = touchLastY = -1;
  }

  // Contact list received from phone
  if (listReady) {
    listReady = false;

    // Wake display whenever phone sends data
    if (!screenOn) {
      screenOn = true;
      screenActivityMs = millis();
      gfx->displayOn();
    }

    // ── BUYDATA response (on-demand, from Buy button) ──────────────────────────
    if (contactList.startsWith("BUYDATA\n")) {
      String buyData = contactList.substring(8);  // local copy — contactList preserved
      buyItemCount = 0;
      buyNeedCount = 0;
      buyLoading   = false;
      while (buyData.startsWith("BUY:") || buyData.startsWith("GOT:")) {
        bool isBought = buyData.startsWith("GOT:");
        int nl = buyData.indexOf('\n');
        if (nl < 0) break;
        String item = buyData.substring(4, nl);
        item.trim();
        if (item.length() > 0 && buyItemCount < MAX_BUY_ITEMS) {
          item.toCharArray(buyItems[buyItemCount], sizeof(buyItems[buyItemCount]));
          buyItemBought[buyItemCount] = isBought;
          if (!isBought) buyNeedCount++;
          buyItemCount++;
        }
        buyData = buyData.substring(nl + 1);
      }
      lastInteractionMs = millis();  // keep screen alive after slow Keep fetch
      if (buyItemCount > 0 && currentMode != MODE_BUY_LIST) {
        // Inactivity timed out before BUYDATA arrived — reopen the list
        currentMode = MODE_BUY_LIST;
      }
      if (currentMode == MODE_BUY_LIST) drawBuyList();
      // Cache to NVS (full-payload block is skipped by goto)
      if (buyItemCount > 0) {
        String buyStr = "";
        for (int i = 0; i < buyItemCount; i++) {
          buyStr += buyItemBought[i] ? "GOT:" : "BUY:";
          buyStr += buyItems[i];
          buyStr += '\n';
        }
        Preferences prefs;
        prefs.begin("whodat", false);
        prefs.putInt("buycnt", buyItemCount);
        prefs.putString("buylist", buyStr);
        prefs.end();
      }
      goto listReadyDone;  // skip full-payload parse
    }

    scrollOffset = 0;
    // Parse leading TIME:HH:MM:SS|WD|DD|MM|YYYY line and sync PCF85063
    if (contactList.startsWith("TIME:")) {
      int nl = contactList.indexOf('\n');
      if (nl >= 0) {
        String ts = contactList.substring(5, nl);
        int ph = ts.substring(0, 2).toInt();
        int pm = ts.substring(3, 5).toInt();
        int ps = ts.substring(6, 8).toInt();
        // Optional date fields: |WD|DD|MM|YYYY
        int pWday = -1, pDay = -1, pMon = -1, pYr = -1;
        int p1 = ts.indexOf('|');
        if (p1 >= 0) {
          int p2 = ts.indexOf('|', p1+1);
          int p3 = ts.indexOf('|', p2+1);
          int p4 = ts.indexOf('|', p3+1);
          if (p2 >= 0 && p3 >= 0 && p4 >= 0) {
            pWday = ts.substring(p1+1, p2).toInt();
            pDay  = ts.substring(p2+1, p3).toInt();
            pMon  = ts.substring(p3+1, p4).toInt();
            pYr   = ts.substring(p4+1).toInt();
          }
        }
        rtcSet(ph, pm, ps, pWday, pDay, pMon, pYr);
        watchFaceFullRedraw = true;
        lastWatchSecond = -1;
        {
          Preferences prefs;
          prefs.begin("whodat", false);
          prefs.putInt("h", ph);
          prefs.putInt("m", pm);
          prefs.putInt("s", ps);
          prefs.end();
        }
        USBSerial.printf("Time from phone: %02d:%02d:%02d\n", ph, pm, ps);
        contactList = contactList.substring(nl + 1);
      }
    }

    // Parse WEATHER:CCCC...:T0,T1,... line
    if (contactList.startsWith("WEATHER:")) {
      int nl = contactList.indexOf('\n');
      if (nl >= 0) {
        String w = contactList.substring(8, nl);
        int colon = w.indexOf(':');
        String cats  = (colon >= 0) ? w.substring(0, colon) : w;
        String temps = (colon >= 0) ? w.substring(colon + 1) : "";
        if (cats.length() == 24) {
          cats.toCharArray(weatherData, sizeof(weatherData));
          // parse comma-separated temperatures
          int idx = 0, pos = 0;
          while (idx < 24 && pos < (int)temps.length()) {
            int comma = temps.indexOf(',', pos);
            String t = (comma < 0) ? temps.substring(pos) : temps.substring(pos, comma);
            weatherTemp[idx++] = (int8_t)t.toInt();
            if (comma < 0) break;
            pos = comma + 1;
          }
          USBSerial.printf("Weather: %s\n", weatherData);
          watchFaceFullRedraw = true;
        }
        contactList = contactList.substring(nl + 1);
      }
    }

    // Parse CAL:D:content lines (D = day offset: 0=today, 1=tomorrow, …)
    calLineCount = 0;
    while (contactList.startsWith("CAL:")) {
      int nl = contactList.indexOf('\n');
      if (nl < 0) break;
      String token = contactList.substring(4, nl);  // strip "CAL:"
      int8_t dayOff = 0;
      // Optional "D:" prefix (single digit + colon) — backwards-compatible
      if (token.length() >= 2 && isDigit((unsigned char)token.charAt(0)) && token.charAt(1) == ':') {
        dayOff = (int8_t)(token.charAt(0) - '0');
        token  = token.substring(2);
      }
      if (calLineCount < MAX_CAL_EVENTS) {
        calDays[calLineCount] = dayOff;
        token.toCharArray(calLines[calLineCount], sizeof(calLines[calLineCount]));
        calLineCount++;
      }
      contactList = contactList.substring(nl + 1);
    }
    if (calLineCount > 0) watchFaceFullRedraw = true;

    // Parse LOST:N (lost-contact beep delay in seconds; 0 = off)
    if (contactList.startsWith("LOST:")) {
      int nl = contactList.indexOf('\n');
      if (nl >= 0) {
        lostSeconds = contactList.substring(5, nl).toInt();
        contactList = contactList.substring(nl + 1);
      }
    }

    // Parse BUY:item (need) and GOT:item (bought) lines from Android
    buyItemCount = 0;
    buyNeedCount = 0;
    while (contactList.startsWith("BUY:") || contactList.startsWith("GOT:")) {
      bool isBought = contactList.startsWith("GOT:");
      int nl = contactList.indexOf('\n');
      if (nl < 0) break;
      String item = contactList.substring(4, nl);
      item.trim();
      if (item.length() > 0 && buyItemCount < MAX_BUY_ITEMS) {
        item.toCharArray(buyItems[buyItemCount], sizeof(buyItems[buyItemCount]));
        buyItemBought[buyItemCount] = isBought;
        if (!isBought) buyNeedCount++;
        buyItemCount++;
      }
      contactList = contactList.substring(nl + 1);
    }
    if (buyItemCount > 0) watchFaceFullRedraw = true;

    // Parse HOME:label|abbrev|compact|dist lines (owner profile addresses)
    homeItemCount = 0;
    while (contactList.startsWith("HOME:")) {
      int nl = contactList.indexOf('\n');
      if (nl < 0) break;
      String home = contactList.substring(5, nl);
      if (home.length() > 0 && homeItemCount < MAX_HOME_ITEMS) {
        home.toCharArray(homeItems[homeItemCount], sizeof(homeItems[homeItemCount]));
        homeItemCount++;
      }
      contactList = contactList.substring(nl + 1);
    }

    // Save clean contact lines for display (separate from the parse buffer)
    contactData = contactList;
    watchFaceFullRedraw = true;  // contacts or cal may have changed

    // Persist everything to NVS so it survives power cycles
    {
      Preferences prefs;
      prefs.begin("whodat", false);
      if (weatherData[0] != '\0') {
        prefs.putString("weather", weatherData);
        String tempsStr = "";
        for (int i = 0; i < 24; i++) {
          if (i > 0) tempsStr += ',';
          tempsStr += (int)weatherTemp[i];
        }
        prefs.putString("wtemps", tempsStr);
      }
      String calStr = "";
      for (int i = 0; i < calLineCount; i++) {
        if (i > 0) calStr += '\n';
        calStr += (int)calDays[i];  // "D:content" so restore uses same parser as wire format
        calStr += ':';
        calStr += calLines[i];
      }
      prefs.putInt("calcnt", calLineCount);
      prefs.putString("cal", calStr);
      // Trim contact list to fit NVS entry limit (~4000 bytes)
      prefs.putString("contacts", contactData.substring(0, min((int)contactData.length(), 3800)));
      prefs.putInt("lost", lostSeconds);
      // Home addresses
      {
        String homeStr = "";
        for (int i = 0; i < homeItemCount; i++) { homeStr += homeItems[i]; homeStr += '\n'; }
        prefs.putInt("homecnt", homeItemCount);
        prefs.putString("homelist", homeStr);
      }
      // Buy list
      if (buyItemCount > 0) {
        String buyStr = "";
        for (int i = 0; i < buyItemCount; i++) {
          buyStr += buyItemBought[i] ? "GOT:" : "BUY:";
          buyStr += buyItems[i];
          buyStr += '\n';
        }
        prefs.putInt("buycnt", buyItemCount);
        prefs.putString("buylist", buyStr);
      }
      prefs.end();
    }

    if      (currentMode == MODE_CONTACT_LIST) drawContactList(contactData);
    else if (currentMode == MODE_BUY_LIST)    drawBuyList();
    else if (currentMode == MODE_CAL_LIST)    drawCalList();
    listReadyDone: ;  // BUYDATA fast-path jumps here
  }

  // Inactivity timeout — return to watch face (buy list uses left-swipe to exit)
  if ((currentMode == MODE_CONTACT_LIST || currentMode == MODE_TAKE_LIST || currentMode == MODE_CAL_LIST) &&
      millis() - lastInteractionMs > INACTIVITY_MS) {
    currentMode         = MODE_WATCH_FACE;
    watchFaceFullRedraw = true;
    lastWatchSecond     = -1;
  }

  if (screenOn && millis() - screenActivityMs > SLEEP_MS) {
    gfx->displayOff();
    screenOn = false;
    gpio_wakeup_enable((gpio_num_t)TP_INT, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    esp_light_sleep_start();  // returns on touch; ISR sets touchDetected
  }

  // Watch face — redraw once per second (millis-based; avoids stall if RTC I2C hiccups)
  if (currentMode == MODE_WATCH_FACE && hasTime && screenOn) {
    static unsigned long lastFaceMs = 0;
    unsigned long now = millis();
    if (watchFaceFullRedraw || now - lastFaceMs >= 950) {
      lastFaceMs          = now;
      drawWatchFace(watchFaceFullRedraw);
      watchFaceFullRedraw = false;
    }
  }

  // Software pedometer — poll accelerometer at 25Hz
  if (millis() - swPedoLastMs >= SW_PEDO_MS) {
    swPedoLastMs = millis();
    IMUdata acc;
    if (imu.getAccelerometer(acc.x, acc.y, acc.z)) {
      float mag = sqrtf(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);
      swDcMag += DC_ALPHA * (mag - swDcMag);          // track gravity DC
      float dyn = mag - swDcMag;                       // dynamic component
      unsigned long now = millis();
      if (!swStepAbove && dyn > STEP_THRESH) {
        swStepAbove = true;
        unsigned long dt = now - swLastStepMs;
        if (swLastStepMs == 0 || dt >= MIN_STEP_MS) {  // debounce
          stepCount++;
        }
        swLastStepMs = now;
      } else if (swStepAbove && dyn < STEP_THRESH * 0.5f) {
        swStepAbove = false;                            // hysteresis on release
      }
    }
  }

  // Once per second: midnight reset + battery refresh + NVS time save
  if (millis() - lastStepMs > STEP_POLL_MS) {
    lastStepMs = millis();
    int h, m, s;
    getCurrentTime(h, m, s);
    if (h == 0 && lastHour != 0) {
      stepCount           = 0;
      swLastStepMs        = 0;
      calAlertedMask      = 0;
      watchFaceFullRedraw = true;  // date string may change length at midnight
      USBSerial.println("Midnight: step count reset");
    }
    lastHour = h;
    static int lastSavedMinute = -1;
    if (m != lastSavedMinute) {
      lastSavedMinute = m;

      // Calendar alert — fire two pings ~1 minute before each timed event today
      int curMins = h * 60 + m;
      for (int i = 0; i < calLineCount; i++) {
        if (calAlertedMask & (1u << i)) continue;
        if (calDays[i] != 0) continue;          // today only
        int evH, evM;
        if (!parseCalTime(calLines[i], evH, evM)) continue;  // skip all-day
        if (evH * 60 + evM - curMins == 1) {
          USBSerial.printf("Cal alert: %s\n", calLines[i]);
          bell(880, 300);
          delay(500);
          bell(880, 300);
          calAlertedMask |= (1u << i);
        }
      }

      Preferences prefs;
      prefs.begin("whodat", false);
      prefs.putInt("h", h);
      prefs.putInt("m", m);
      prefs.putInt("s", 0);
      prefs.end();
      Wire.beginTransmission(AXP2101_ADDR);
      Wire.write(0xA4);
      if (Wire.endTransmission(false) == 0) {
        Wire.requestFrom((uint8_t)AXP2101_ADDR, (uint8_t)1);
        if (Wire.available()) {
          uint8_t v = Wire.read();
          if (v <= 100) cachedBattPct = v;
        }
      }
    }
  }

  // BLE connection status changed
  if (messageUpdated) {
    messageUpdated = false;
    static bool prevConnected = false;
    if (deviceConnected && !prevConnected) {
      lostTimerStartMs = 0;            // cancel pending beep — phone reconnected
    } else if (!deviceConnected && prevConnected && lostSeconds > 0) {
      lostTimerStartMs = millis();     // start countdown
    }
    prevConnected = deviceConnected;
    if (!hasTime) {
      drawScreen(
        deviceConnected ? "Loading contacts..." : "Open WhoDat\non your phone",
        deviceConnected
      );
    }
  }

  // Lost-contact beep
  if (lostTimerStartMs > 0 && lostSeconds > 0 &&
      millis() - lostTimerStartMs >= (unsigned long)lostSeconds * 1000UL) {
    lostTimerStartMs = 0;
    beep(880, 400);
    delay(250);
    beep(880, 400);
  }

  delay(5);
}
