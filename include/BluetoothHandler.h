//////------------------------------------
////// Manage bluetooth

#include "Arduino.h"

#ifndef _BluetoothHandler_h

#define _BluetoothHandler_h

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "Settings.h"
#include "SharedData.h"

class BluetoothHandler
{
public:
    BluetoothHandler();
    static void init(Settings *);
    static void bleOnScanResults(BLEScanResults);
    static void notifyBleLock();
    static void notifyBleLogs(char *);
    static void setBleLock(bool);
    static void processBLE();
    static void notifyModeOrder(uint8_t);
    static void notifyBreakeSentOrder(uint8_t,uint8_t);
    static void notifyEcoOrder(uint8_t);
    static void notifyAccelOrder(uint8_t);
    static void notifyTemperatureStatus(uint32_t);
    static void notifyHumidityStatus(uint32_t);
    static void notifyAuxOrder(uint8_t);
    static void notifySpeedLimiterStatus(uint8_t);
    static void setSharedData(SharedData*);

    static BLEScan *pBLEScan;
    static BLEServer *pServer;
    static BLESecurity *pSecurity;
    static BLECharacteristic *pCharacteristicSpeed;
    static BLECharacteristic *pCharacteristicMode;
    static BLECharacteristic *pCharacteristicBrakeSentOrder;
    static BLECharacteristic *pCharacteristicVoltageStatus;
    static BLECharacteristic *pCharacteristicCurrentStatus;
    static BLECharacteristic *pCharacteristicPowerStatus;
    static BLECharacteristic *pCharacteristicBtlockStatus;
    static BLECharacteristic *pCharacteristicTemperatureStatus;
    static BLECharacteristic *pCharacteristicHumidityStatus;
    static BLECharacteristic *pCharacteristicSettings1;
    static BLECharacteristic *pCharacteristicSpeedLimiter;
    static BLECharacteristic *pCharacteristicEco;
    static BLECharacteristic *pCharacteristicAccel;
    static BLECharacteristic *pCharacteristicCurrentCalib;
    static BLECharacteristic *pCharacteristicOtaSwitch;
    static BLECharacteristic *pCharacteristicLogs;
    static BLECharacteristic *pCharacteristicFastUpdate;
    static BLECharacteristic *pCharacteristicSettings2;
    static BLECharacteristic *pCharacteristicSettings3;
    static BLECharacteristic *pCharacteristicAux;
    static BLECharacteristic *pCharacteristicSpeedPid;

    static int8_t bleLockStatus;
    static int8_t blePicclyVisible;
    static int8_t blePicclyRSSI;
    static int8_t bleLockForced;
    static int8_t fastUpdate;

    static bool deviceConnected;
    static bool oldDeviceConnected;

    static SharedData *shrd;
    static Settings *settings;

    static int8_t test2;

};


#endif