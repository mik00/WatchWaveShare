// HelloWatch — receives nearby-contact list from WhoDat phone app via BLE
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

// Display layout constants
#define LINE_H   22    // pixels per contact row
#define CHAR_W   12    // text size 2: 6px base × 2
#define CHAR_H   16    // text size 2: 8px base × 2
#define MARGIN   10

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

// ── Status screen ─────────────────────────────────────────────────────────────
void drawScreen(const String &msg, bool connected) {
  gfx->fillScreen(BLACK);

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

  // Header
  gfx->setTextSize(2);
  gfx->setTextColor(0x07E0);  // green
  gfx->setCursor(MARGIN, 8);
  gfx->print("Nearby Contacts");
  gfx->drawFastHLine(0, 28, LCD_WIDTH, 0x4208);

  int y = 36;
  int start = 0;

  while (y + CHAR_H <= LCD_HEIGHT) {
    int nl = payload.indexOf('\n', start);
    String line = (nl < 0) ? payload.substring(start)
                            : payload.substring(start, nl);
    line.trim();

    if (line.length() > 0) {
      // Each line is "Name  distance" — split at double-space
      int sep  = line.indexOf("  ");
      String name = (sep >= 0) ? line.substring(0, sep)      : line;
      String dist = (sep >= 0) ? line.substring(sep + 2)     : "";
      dist.trim();

      // Truncate name if it would overlap the distance column
      int distPx  = dist.length() * CHAR_W + MARGIN;
      int nameMax = (LCD_WIDTH - MARGIN - distPx) / CHAR_W;
      if (nameMax > 0 && (int)name.length() > nameMax)
        name = name.substring(0, nameMax - 1) + "~";

      // Name — white, left-aligned
      gfx->setTextSize(2);
      gfx->setTextColor(WHITE);
      gfx->setCursor(MARGIN, y);
      gfx->print(name);

      // Distance — cyan, right-aligned
      if (dist.length() > 0) {
        int distX = LCD_WIDTH - (int)dist.length() * CHAR_W - MARGIN;
        gfx->setTextColor(0x07FF);  // cyan
        gfx->setCursor(distX, y);
        gfx->print(dist);
      }

      y += LINE_H;
    }

    if (nl < 0) break;
    start = nl + 1;
  }
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

  drawScreen("Open WhoDat\non your phone\nthen tap here", false);
  USBSerial.println("Advertising as 'WatchWave'");
}

// ── Loop ───────────────────────────────────────────────────────────────────────
void loop() {
  // Touch — send request to phone
  if (touchDetected) {
    touchDetected = false;
    int16_t x, y;
    if (touch.getPoint(&x, &y, 1)) {
      USBSerial.println("Touch: sending request");
      drawScreen("Searching...", deviceConnected);
      if (deviceConnected && pTxChar) {
        pTxChar->setValue("request");
        pTxChar->notify();
      }
    }
  }

  // Contact list received — render it
  if (listReady) {
    listReady = false;
    drawContactList(contactList);
  }

  // Connection status changes
  if (messageUpdated) {
    messageUpdated = false;
    if (!listReady) {  // don't overwrite a freshly drawn list
      drawScreen(
        deviceConnected ? "Tap to search\nnearby contacts" : "Open WhoDat\non your phone\nthen tap here",
        deviceConnected
      );
    }
  }

  delay(50);
}
