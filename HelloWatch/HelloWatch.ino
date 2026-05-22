// HelloWatch — displays text sent from phone via BLE (Nordic UART Service)
// Android app: "Serial Bluetooth Terminal" by Kai Morich (free, Play Store)
//
// Libraries required (Waveshare sample package + ESP32 core built-ins):
//   Arduino_GFX, Arduino_DriveBus  — from Waveshare GitHub sample package
//   BLEDevice, BLEServer           — built into ESP32 Arduino core

#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include <Wire.h>
#include "HWCDC.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_log.h"

// Nordic UART Service — recognised by Serial Bluetooth Terminal automatically
#define NUS_SERVICE_UUID  "6E400001-B5B3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5B3-F393-E0A9-E50E24DCCA9E"  // phone → watch
#define NUS_TX_UUID       "6E400003-B5B3-F393-E0A9-E50E24DCCA9E"  // watch → phone

HWCDC USBSerial;

// ── Display ────────────────────────────────────────────────────────────────────
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_GFX *gfx = new Arduino_CO5300(
  bus, LCD_RESET, 0 /* rotation */,
  LCD_WIDTH, LCD_HEIGHT,
  22 /* col_offset1 */, 0, 0, 0);

// ── BLE state ──────────────────────────────────────────────────────────────────
BLECharacteristic *pTxCharacteristic;
volatile bool deviceConnected = false;
volatile bool messageUpdated  = false;
String pendingMessage = "";

// ── Display helpers ────────────────────────────────────────────────────────────
void drawScreen(const String &msg, bool connected) {
  gfx->fillScreen(BLACK);

  // Status bar
  gfx->setTextSize(1);
  gfx->setTextColor(connected ? 0x07E0 /* green */ : 0xFFE0 /* yellow */);
  gfx->setCursor(15, 8);
  gfx->print(connected ? "BLE Connected" : "BLE Advertising...");

  // Divider
  gfx->drawFastHLine(0, 24, LCD_WIDTH, 0x4208 /* dark grey */);

  // Message
  gfx->setTextWrap(true);
  gfx->setTextSize(3);
  gfx->setTextColor(WHITE);
  gfx->setCursor(15, 44);
  gfx->print(msg);
}

// ── BLE callbacks ──────────────────────────────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    USBSerial.println("BLE: phone connected");
    deviceConnected = true;
    // deliberately NOT triggering drawScreen here — isolating display/BLE timing
  }
  void onDisconnect(BLEServer *pSrv) override {
    USBSerial.println("BLE: phone disconnected");
    deviceConnected = false;
    messageUpdated  = true;
    pSrv->startAdvertising();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    String val = pChar->getValue();
    USBSerial.print("BLE: onWrite fired, length=");
    USBSerial.print(val.length());
    USBSerial.print(", value='");
    USBSerial.print(val);
    USBSerial.println("'");
    if (val.length() > 0) {
      pendingMessage = val;
      pendingMessage.trim();
      messageUpdated = true;
      USBSerial.println("BLE: messageUpdated set");
    }
  }
};

// ── Setup ──────────────────────────────────────────────────────────────────────
void setup() {
  USBSerial.begin(115200);
  esp_log_level_set("BLE_GAP", ESP_LOG_DEBUG);
  esp_log_level_set("BLE_GATTS", ESP_LOG_DEBUG);

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  if (!gfx->begin()) {
    USBSerial.println("Display init failed!");
    while (1) delay(100);
  }
  drawScreen("Starting...", false);

  // BLE init
  BLEDevice::init("WatchWave");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(NUS_SERVICE_UUID);

  // TX characteristic — watch notifies phone
  pTxCharacteristic = pService->createCharacteristic(
    NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  // RX characteristic — phone writes to watch
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    NUS_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxCharacteristic->setCallbacks(new RxCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(NUS_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  drawScreen("Send text from\nthe app!", false);
  USBSerial.println("BLE advertising as 'WatchWave'");
}

// ── Loop ───────────────────────────────────────────────────────────────────────
void loop() {
  if (messageUpdated) {
    messageUpdated = false;
    USBSerial.print("loop: redrawing, connected=");
    USBSerial.print(deviceConnected);
    USBSerial.print(", msg='");
    USBSerial.print(pendingMessage);
    USBSerial.println("'");
    drawScreen(pendingMessage.isEmpty() ? "Send text from\nthe app!" : pendingMessage,
               deviceConnected);
  }
  delay(50);
}
