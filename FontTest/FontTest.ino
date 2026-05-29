// FontTest — prints full alphabet + digits repeatedly to identify corrupted glyphs.
// If the same characters are always distorted across repetitions, the font
// bitmap data for those specific glyphs is corrupt.

#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "HWCDC.h"
#include "../WhoDatESP/fonts/FreeSans9pt7b.h"

// ── Pins ───────────────────────────────────────────────────────────────────
#define LCD_CS     12
#define LCD_SCLK   11
#define LCD_SDIO0   4
#define LCD_SDIO1   5
#define LCD_SDIO2   6
#define LCD_SDIO3   7
#define LCD_RESET   8
#define LCD_WIDTH  410
#define LCD_HEIGHT 502

#define BLACK  0x0000
#define WHITE  0xFFFF
#define CYAN   0x07FF
#define GREEN  0x07E0
#define MARGIN  6
#define LINE_H 36   // comfortably spaced so rows don't interfere

HWCDC USBSerial;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_GFX *gfx = new Arduino_CO5300(
  bus, LCD_RESET, 0,
  LCD_WIDTH, LCD_HEIGHT,
  22, 0, 0, 0);

// Print a string one character at a time with a small gap between cells
void printRow(const char* str, int y, uint16_t color) {
  char ch[2] = {0, 0};
  int x = MARGIN;
  for (int i = 0; str[i] && x < LCD_WIDTH - MARGIN; i++) {
    ch[0] = str[i];
    int16_t bx, by; uint16_t bw, bh;
    gfx->getTextBounds(ch, x, y, &bx, &by, &bw, &bh);
    gfx->setTextColor(color, BLACK);
    gfx->setCursor(x, y);
    gfx->print(ch[0]);
    x += (int)bw + 2;   // 2 px gap between character cells
  }
}

void drawAlphabetTest() {
  gfx->fillScreen(BLACK);

  // Header
  gfx->setFont(nullptr);
  gfx->setTextSize(2);
  gfx->setTextColor(GREEN, BLACK);
  gfx->setCursor(MARGIN, 60);
  gfx->print("Alphabet test");
  gfx->drawFastHLine(0, 90, LCD_WIDTH, 0x4208);

  gfx->setFont(&FreeSans9pt7b);
  gfx->setTextSize(1);

  // Print upper, lower, digits — repeated 3 times to confirm consistency
  int y = 120;
  for (int rep = 0; rep < 3 && y + 6 <= LCD_HEIGHT; rep++) {
    printRow("ABCDEFGHIJKLMNOPQRSTUVWXYZ", y, WHITE);
    y += LINE_H;
    if (y + 6 > LCD_HEIGHT) break;

    printRow("abcdefghijklmnopqrstuvwxyz", y, WHITE);
    y += LINE_H;
    if (y + 6 > LCD_HEIGHT) break;

    printRow("0123456789", y, CYAN);
    y += LINE_H;
  }

  gfx->setFont(nullptr);
  USBSerial.println("Alphabet test drawn");
}

void setup() {
  USBSerial.begin(115200);
#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif
  if (!gfx->begin()) { while (1) delay(100); }
  gfx->setTextWrap(false);
  drawAlphabetTest();
}

void loop() {
  // Static display — just show it until you're done reading
  delay(1000);
}
