// Minimal BLE UART test — NO display, NO PSRAM dependency
// Purpose: confirm BLE works on this hardware before adding complexity

#include <Arduino.h>
#include "HWCDC.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "host/ble_hs.h"

#define NUS_SERVICE_UUID  "6E400001-B5B3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID       "6E400002-B5B3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5B3-F393-E0A9-E50E24DCCA9E"

HWCDC USBSerial;

BLECharacteristic *pTxChar;
volatile bool connected = false;

class SrvCB : public BLEServerCallbacks {
  void onConnect(BLEServer *, ble_gap_conn_desc *desc) override {
    USBSerial.printf("CONNECTED itvl=%u timeout=%u\n", desc->conn_itvl, desc->supervision_timeout);
    connected = true;
  }
  void onDisconnect(BLEServer *pSrv) override {
    USBSerial.println("DISCONNECTED");
    connected = false;
    pSrv->startAdvertising();
  }
};

class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    String val = pChar->getValue();
    USBSerial.print("RECEIVED: ");
    USBSerial.println(val);
  }
};

void setup() {
  USBSerial.begin(115200);
  delay(1000);
  USBSerial.println("BLETest starting...");

  BLEDevice::init("WatchWave");
  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm    = 0;
  ble_hs_cfg.sm_sc      = 1;
  ble_hs_cfg.sm_io_cap  = BLE_HS_IO_NO_INPUT_OUTPUT;

  BLEServer  *pServer  = BLEDevice::createServer();
  pServer->setCallbacks(new SrvCB());

  BLEService *pService = pServer->createService(NUS_SERVICE_UUID);

  pTxChar = pService->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTxChar->addDescriptor(new BLE2902());

  BLECharacteristic *pRxChar = pService->createCharacteristic(
    NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE);  // WRITE WITH RESPONSE only
  pRxChar->setCallbacks(new RxCB());

  pService->start();

  BLEAdvertising *pAdv = pServer->getAdvertising();
  pAdv->addServiceUUID(NUS_SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->start();

  USBSerial.println("Advertising as 'WatchWave' — connect from your phone.");
}

void loop() {
  if (connected) {
    delay(1000);
    USBSerial.println("(connected, waiting for writes...)");
  } else {
    delay(500);
  }
}
