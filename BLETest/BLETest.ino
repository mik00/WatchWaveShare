// Minimal BLE UART test — NO display, NO PSRAM
// Compiled with PSRAM=disabled

#include <Arduino.h>
#include "HWCDC.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define NUS_SERVICE_UUID  "6E400001-B5B3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5B3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5B3-F393-E0A9-E50E24DCCA9E"

HWCDC USBSerial;
volatile bool connected = false;
int writeCount = 0;

class SrvCB : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    USBSerial.println("CONNECTED");
    connected = true;
  }
  void onDisconnect(BLEServer *pSrv) override {
    USBSerial.println("DISCONNECTED");
    connected = false;
    pSrv->startAdvertising();
  }
};

class RxCB : public BLECharacteristicCallbacks {
  // Override the NimBLE path directly
  void onWrite(BLECharacteristic *pChar, ble_gap_conn_desc *desc) override {
    writeCount++;
    USBSerial.printf("onWrite #%d (NimBLE path)\n", writeCount);
    String val = pChar->getValue();
    USBSerial.printf("  value='%s' len=%u\n", val.c_str(), val.length());
  }
  // Also override the 1-param fallback
  void onWrite(BLECharacteristic *pChar) override {
    writeCount++;
    USBSerial.printf("onWrite #%d (1-param path)\n", writeCount);
    String val = pChar->getValue();
    USBSerial.printf("  value='%s' len=%u\n", val.c_str(), val.length());
  }
};

void setup() {
  USBSerial.begin(115200);
  delay(500);
  USBSerial.println("BLETest v2 starting...");

  BLEDevice::init("WatchWave");
  // Random address forces Android to re-discover GATT on every connection
  BLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);

  BLEServer  *pServer  = BLEDevice::createServer();
  pServer->setCallbacks(new SrvCB());

  BLEService *pService = pServer->createService(NUS_SERVICE_UUID);

  BLECharacteristic *pTxChar = pService->createCharacteristic(
    NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTxChar->addDescriptor(new BLE2902());

  BLECharacteristic *pRxChar = pService->createCharacteristic(
    NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE);
  pRxChar->setCallbacks(new RxCB());

  USBSerial.printf("RxChar UUID=%s\n", pRxChar->getUUID().toString().c_str());

  pService->start();

  BLEAdvertising *pAdv = pServer->getAdvertising();
  pAdv->addServiceUUID(NUS_SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->start();

  USBSerial.println("Advertising as 'WatchWave'");
}

void loop() {
  delay(1000);
  if (connected) {
    USBSerial.printf("tick — writes received: %d\n", writeCount);
  }
}
