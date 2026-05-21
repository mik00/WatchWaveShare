// Hello World for Waveshare ESP32-S3-Touch-AMOLED-2.06
// Based on official Waveshare sample: examples/Arduino-v3.2.0/examples/01_HelloWorld
//
// Required libraries (install from Waveshare GitHub sample package):
//   https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-2.06
//   Copy contents of examples/Arduino-v3.2.0/libraries/ into your Arduino libraries folder:
//     - Arduino_GFX  (Waveshare fork with CO5300 driver)
//     - Arduino_DriveBus
//     - XPowersLib
//
// Board settings (Arduino IDE):
//   Board:            ESP32S3 Dev Module
//   PSRAM:            OPI PSRAM
//   Flash Mode:       QIO 80MHz
//   Flash Size:       32MB
//   Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)

#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include <Wire.h>
#include "HWCDC.h"

HWCDC USBSerial;

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_GFX *gfx = new Arduino_CO5300(
  bus, LCD_RESET, 0 /* rotation */,
  LCD_WIDTH, LCD_HEIGHT,
  22 /* col_offset1 */, 0 /* row_offset1 */,
   0 /* col_offset2 */, 0 /* row_offset2 */);

void setup() {
  USBSerial.begin(115200);
  USBSerial.println("HelloWatch starting...");

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  if (!gfx->begin()) {
    USBSerial.println("Display init failed!");
    while (1) delay(100);
  }

  gfx->fillScreen(BLACK);
  gfx->setCursor(10, 10);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(3);
  gfx->println("Hello World!");

  gfx->setCursor(10, 60);
  gfx->setTextColor(0x07E0); // green
  gfx->setTextSize(2);
  gfx->println("WatchWaveShare");

  USBSerial.println("Display ready.");
}

void loop() {
  // Scatter random Hello World messages across the screen
  gfx->setCursor(random(gfx->width()), random(gfx->height()));
  gfx->setTextColor(random(0xFFFF), random(0xFFFF));
  gfx->setTextSize(random(1, 5), random(1, 5));
  gfx->println("Hello!");
  delay(300);
}
