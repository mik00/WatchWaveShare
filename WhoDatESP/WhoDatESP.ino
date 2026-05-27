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
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include <Wire.h>
#include "HWCDC.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "TouchDrvFT6X36.hpp"
#include "fonts/FreeSans9pt7b.h"

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

// Display layout constants — FreeSans9pt7b
#define LINE_H       30    // yAdvance=22 + 8px leading
#define CHAR_W        6    // average px per glyph (proportional estimate)
#define CHAR_DESCENT  4    // max descent below baseline
#define LIST_START_Y 17    // first row baseline: top margin(5) + ascent(12)
#define MARGIN       10

HWCDC USBSerial;

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

// ── BLE state ──────────────────────────────────────────────────────────────────
BLECharacteristic *pTxChar    = nullptr;
volatile bool deviceConnected = false;
volatile bool messageUpdated  = false;
String pendingMessage         = "";

// Receive buffer — accumulates chunks until "\nEND" is received
String bleBuffer    = "";
bool   listReady    = false;
String contactList  = "";

// ── Scroll state ───────────────────────────────────────────────────────────────
int            scrollOffset  = 0;
bool           wasTouching   = false;
int16_t        touchStartY   = -1;
int16_t        touchLastY    = -1;
unsigned long  lastTouchMs   = 0;
#define SWIPE_THRESHOLD  40    // px movement to count as a swipe
#define TOUCH_RELEASE_MS 120   // ms silence = finger lifted
#define VISIBLE_ROWS     ((LCD_HEIGHT - LIST_START_Y) / LINE_H)
#define SCROLL_ROWS      (VISIBLE_ROWS * 2 / 3)

// ── Font diagnostic ───────────────────────────────────────────────────────
// Call debugFont() once from setup() to check GFXfont rendering.
// Put your font file in a subfolder fonts/ and #include "fonts/YourFont.h"
// so Arduino IDE doesn't auto-compile it. Then uncomment the three lines below.
void debugFont() {
  // #include won't work here — include the font at file top, then:
  // const GFXfont *testFont = &FreeSans9pt7b;  // ← your font
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

  const GFXfont *f = &FreeSans9pt7b;
  USBSerial.printf("Font ptr: %p  first=%d last=%d yAdvance=%d\n",
    f, f->first, f->last, f->yAdvance);

  gfx->fillScreen(BLACK);
  gfx->fillRect(0, 80, LCD_WIDTH, 80, 0x001F);   // blue reference box y=80..160
  gfx->setFont(f);
  gfx->setTextColor(WHITE);
  gfx->setCursor(10, 150);    // baseline at 150 → glyphs top at ~y=125
  gfx->print("ABCabc 123");

  int16_t bx, by; uint16_t bw, bh;
  gfx->getTextBounds("ABCabc 123", 10, 150, &bx, &by, &bw, &bh);
  USBSerial.printf("Bounds: x=%d y=%d w=%d h=%d\n", bx, by, bw, bh);
  // w==0 or h==0 → font not loaded
  // by should be ~125 (inside the blue box)

  gfx->setFont((const GFXfont*)nullptr);   // explicit cast avoids overload ambiguity
  delay(5000);
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

// ── Contact list screen ───────────────────────────────────────────────────────
void drawContactList(const String &payload) {
  gfx->fillScreen(BLACK);
  gfx->setTextWrap(false);

  gfx->setFont(&FreeSans9pt7b);

  int y = LIST_START_Y;
  int start = 0;
  int lineIndex = 0;

  while (true) {
    int nl = payload.indexOf('\n', start);
    String line = (nl < 0) ? payload.substring(start)
                            : payload.substring(start, nl);
    line.trim();

    if (line.length() > 0) {
      if (lineIndex >= scrollOffset) {
        if (y + CHAR_DESCENT > LCD_HEIGHT) break;

        int sep = line.indexOf("  ");
        String name = (sep >= 0) ? line.substring(0, sep)  : line;
        String dist = (sep >= 0) ? line.substring(sep + 2) : "";
        dist.trim();

        int distPx  = dist.length() * CHAR_W + MARGIN;
        int nameMax = (LCD_WIDTH - MARGIN - distPx) / CHAR_W;
        if (nameMax > 0 && (int)name.length() > nameMax)
          name = name.substring(0, nameMax - 1) + "~";

        gfx->setTextColor(WHITE);
        gfx->setCursor(MARGIN, y);
        gfx->print(name);

        if (dist.length() > 0) {
          int distX = LCD_WIDTH - (int)dist.length() * CHAR_W - MARGIN;
          gfx->setTextColor(0x07FF);
          gfx->setCursor(distX, y);
          gfx->print(dist);
        }

        y += LINE_H;
      }
      lineIndex++;
    }

    if (nl < 0) break;
    start = nl + 1;
  }

  gfx->setFont((const GFXfont*)nullptr);
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

// ── Setup ──────────────────────────────────────────────────────────────────────
void setup() {
  USBSerial.begin(115200);

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  if (!gfx->begin()) {
    USBSerial.println("Display init failed!");
    while (1) delay(100);
  }
  gfx->setTextWrap(false);
  drawScreen("Starting...", false);

  // Touch init
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

  BLEDevice::init("WatchWave");
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
  USBSerial.println("Advertising as 'WatchWave'");
}

// ── Loop ───────────────────────────────────────────────────────────────────────
void loop() {
  // Touch — track swipe to scroll the contact list
  if (touchDetected) {
    touchDetected = false;
    int16_t tx, ty;
    if (touch.getPoint(&tx, &ty, 1)) {
      if (!wasTouching) { touchStartY = ty; wasTouching = true; }
      touchLastY  = ty;
      lastTouchMs = millis();
    }
  }
  if (wasTouching && millis() - lastTouchMs > TOUCH_RELEASE_MS) {
    wasTouching = false;
    int delta = touchLastY - touchStartY;
    if (abs(delta) > SWIPE_THRESHOLD && contactList.length() > 0) {
      if (delta < 0) scrollOffset += SCROLL_ROWS;          // swipe up → later contacts
      else           scrollOffset = max(0, scrollOffset - SCROLL_ROWS); // swipe down → earlier
      drawContactList(contactList);
    }
    touchStartY = touchLastY = -1;
  }

  // Contact list received — render it from the top
  if (listReady) {
    listReady    = false;
    scrollOffset = 0;
    drawContactList(contactList);
  }

  // Connection status changes
  if (messageUpdated) {
    messageUpdated = false;
    if (!listReady) {  // don't overwrite a freshly drawn list
      drawScreen(
        deviceConnected ? "Loading contacts..." : "Open WhoDat\non your phone",
        deviceConnected
      );
    }
  }

  delay(50);
}
