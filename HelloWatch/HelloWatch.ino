// HelloWatch — displays text sent from phone via BLE (Web Bluetooth)
// Web app: https://mik00.github.io/WatchWaveShare/
//
// Key build settings (IMPORTANT):
//   PSRAM: DISABLED  — OPI PSRAM conflicts with BLE DMA buffers on ESP32-S3
//   Flash: QIO 80MHz, 32MB
//   Partition: 32M Flash (4.8MB APP/22MB FATFS)
//
// Libraries (from Waveshare sample package):
//   Arduino_GFX, Arduino_DriveBus

#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include <Wire.h>
#include "HWCDC.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define NUS_SERVICE_UUID  "6E400001-B5B3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5B3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5B3-F393-E0A9-E50E24DCCA9E"

HWCDC USBSerial;

// ── Display ────────────────────────────────────────────────────────────────────
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_GFX *gfx = new Arduino_CO5300(
  bus, LCD_RESET, 0 /* rotation */,
  LCD_WIDTH, LCD_HEIGHT,
  22 /* col_offset1 */, 0, 0, 0);

// ── BLE state ──────────────────────────────────────────────────────────────────
volatile bool deviceConnected = false;
volatile bool messageUpdated  = false;
String pendingMessage = "";

// ── Display helpers ────────────────────────────────────────────────────────────
void drawScreen(const String &msg, bool connected) {
  gfx->fillScreen(BLACK);

  gfx->setTextSize(1);
  gfx->setTextColor(connected ? 0x07E0 /* green */ : 0xFFE0 /* yellow */);
  gfx->setCursor(15, 8);
  gfx->print(connected ? "BLE Connected" : "BLE Advertising...");

  gfx->drawFastHLine(0, 24, LCD_WIDTH, 0x4208 /* dark grey */);

  gfx->setTextWrap(true);
  gfx->setTextSize(4);
  gfx->setTextColor(WHITE);
  gfx->setCursor(15, 44);
  gfx->print(msg);
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
    pSrv->startAdvertising();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar, ble_gap_conn_desc *) override {
    String val = pChar->getValue();
    USBSerial.print("Received: ");
    USBSerial.println(val);
    if (val.length() > 0) {
      pendingMessage = val;
      pendingMessage.trim();
      messageUpdated = true;
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
  drawScreen("Starting...", false);

  BLEDevice::init("WatchWave");
  // Random address forces Android GATT re-discovery on every connection,
  // avoiding stale cached characteristic handles from previous sessions.
  BLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(NUS_SERVICE_UUID);

  BLECharacteristic *pTxChar = pService->createCharacteristic(
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

  drawScreen("Open WatchWave\nin Chrome\non your phone", false);
  USBSerial.println("Advertising as 'WatchWave'");
}

// ── Loop ───────────────────────────────────────────────────────────────────────
void loop() {
  if (messageUpdated) {
    messageUpdated = false;
    drawScreen(pendingMessage.isEmpty() ? "Send a message\nfrom the app!" : pendingMessage,
               deviceConnected);
  }
  delay(50);
}
