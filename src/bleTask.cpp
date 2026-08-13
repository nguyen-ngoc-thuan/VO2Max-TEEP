/*
 * bleTask.cpp - BLE: HR belt reader (client) + HR output for Zwift (server) + Golden Cheetah
 * Copyright (C) 2025 TEEP Project
 * Based on VO2Max-500pa by Lauri Peltonen and VO2max-main (GPL V3)
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <NimBLEDevice.h>

#include "esp_log.h"
#include "config.h"
#include "sensorTask.h"
#include "bleTask.h"

static const char *TAG = "BLE";

// ---- HR Belt Reader (Client) ----
static NimBLEUUID hrServiceUUID((uint16_t)0x180D);
static NimBLEUUID hrCharUUID((uint16_t)0x2A37);

static NimBLEClient *pClient = nullptr;
static bool hrConnected = false;
static bool hrScanRequested = false;
static bool hrConnectRequested = false;
static bool hrDisconnectRequested = false;
static char hrSensorName[32] = "Unknown";

// ---- HR Output (Server) for Zwift/Strava ----
static NimBLEServer *pServer = nullptr;
static NimBLECharacteristic *pHRMeasurement = nullptr;
static NimBLECharacteristic *pSensorPosition = nullptr;
static bool hrOutputActive = false;
static bool hrClientConnected = false;

// ---- Golden Cheetah (Server) ----
static NimBLEUUID gcServiceUUID("00001523-1212-EFDE-1523-785FEABCD123");
static NimBLECharacteristic *pGCChar = nullptr;
static bool gcActive = false;
static bool gcClientConnected = false;

struct __attribute__((packed)) GCData {
    int16_t freq;
    uint8_t temp;
    uint8_t hum;
    int16_t rmv;
    int16_t feo2;
    int16_t vo2;
};

// ---- BLE Event Group ----
static EventGroupHandle_t bleEvent = nullptr;
#define BLE_HR_CONNECT      0x0001
#define BLE_HR_DISCONNECT   0x0002
#define BLE_HR_SCAN         0x0004
#define BLE_GC_START        0x0010
#define BLE_GC_STOP         0x0020
#define BLE_HROUT_START     0x0040
#define BLE_HROUT_STOP      0x0080

// ---- HR Belt Notification Callback ----
static void hrNotifyCallback(NimBLERemoteCharacteristic *pChar, uint8_t *pData,
                             size_t length, bool isNotify) {
    if (length < 2) return;

    uint16_t hr;
    uint8_t idx = 1;

    if (pData[0] & 0x01) {
        // 16-bit HR
        hr = pData[1] | (pData[2] << 8);
        idx = 3;
    } else {
        hr = pData[1];
        idx = 2;
    }

    sensorAddExtraValue(SENSOR_EXTRA_HR, (float)hr, 0);

    // Check for RR interval
    if (pData[0] & 0x10) {
        if (idx + 1 < length) {
            int16_t rr = pData[idx] | (pData[idx + 1] << 8);
            float rr_ms = (float)rr * 1000.0f / 1024.0f;
            sensorAddExtraValue(SENSOR_EXTRA_RR, rr_ms, 0);
        }
    }
}

// ---- Server Callbacks ----
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *pServer) {
        hrClientConnected = true;
        gcClientConnected = true;
        ESP_LOGI(TAG, "BLE client connected");
    }
    void onDisconnect(NimBLEServer *pServer) {
        hrClientConnected = false;
        gcClientConnected = false;
        ESP_LOGI(TAG, "BLE client disconnected");
        // Restart advertising
        NimBLEDevice::startAdvertising();
    }
};

// ---- Initialize BLE Server (HR output + Golden Cheetah) ----
static void initBLEServer(void) {
    if (pServer) return;  // Already initialized

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // HR Service (0x180D)
    NimBLEService *hrService = pServer->createService((uint16_t)0x180D);
    pHRMeasurement = hrService->createCharacteristic(
        (uint16_t)0x2A37, NIMBLE_PROPERTY::NOTIFY);
    pSensorPosition = hrService->createCharacteristic(
        (uint16_t)0x2A38, NIMBLE_PROPERTY::READ);
    uint8_t bodyPos = 2;  // Wrist
    pSensorPosition->setValue(&bodyPos, 1);
    hrService->start();

    // Golden Cheetah Service
    NimBLEService *gcService = pServer->createService(gcServiceUUID);
    pGCChar = gcService->createCharacteristic(
        NimBLEUUID("00001524-1212-EFDE-1523-785FEABCD123"),
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    gcService->start();

    // Advertising
    NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID((uint16_t)0x180D);
    pAdv->addServiceUUID(gcServiceUUID);
    pAdv->setScanResponse(true);
}

// ---- Connect to HR Belt ----
static bool connectToHRBelt(void) {
    if (!pClient) {
        pClient = NimBLEDevice::createClient();
    }

    NimBLEAddress addr(global_settings.HRSensorAddress);
    ESP_LOGI(TAG, "Connecting to HR belt: %s", addr.toString().c_str());

    if (!pClient->connect(addr)) {
        ESP_LOGE(TAG, "HR belt connection failed");
        return false;
    }

    NimBLERemoteService *pRemoteService = pClient->getService(hrServiceUUID);
    if (!pRemoteService) {
        ESP_LOGE(TAG, "HR service not found");
        pClient->disconnect();
        return false;
    }

    NimBLERemoteCharacteristic *pRemoteChar = pRemoteService->getCharacteristic(hrCharUUID);
    if (!pRemoteChar || !pRemoteChar->canNotify()) {
        ESP_LOGE(TAG, "HR characteristic not found");
        pClient->disconnect();
        return false;
    }

    pRemoteChar->subscribe(true, hrNotifyCallback);
    hrConnected = true;

    // Get device name (use address as fallback)
    std::string name = pClient->getPeerAddress().toString();
    if (name.length() > 0) {
        strncpy(hrSensorName, name.c_str(), 31);
    }

    ESP_LOGI(TAG, "HR belt connected: %s", hrSensorName);
    return true;
}

// ---- Scan for HR Belts ----
static void scanForHRBelt(void) {
    ESP_LOGI(TAG, "Scanning for HR belts...");
    NimBLEScan *pScan = NimBLEDevice::getScan();
    pScan->setActiveScan(true);
    NimBLEScanResults results = pScan->start(10);

    for (int i = 0; i < results.getCount(); i++) {
        NimBLEAdvertisedDevice dev = results.getDevice(i);
        if (dev.isAdvertisingService(hrServiceUUID)) {
            ESP_LOGI(TAG, "Found HR device: %s [%s]",
                     dev.getName().c_str(), dev.getAddress().toString().c_str());

            // Store address
            memcpy(global_settings.HRSensorAddress,
                   dev.getAddress().getNative(), 6);

            std::string name = dev.getName();
            if (name.length() > 0) {
                strncpy(hrSensorName, name.c_str(), 31);
            }

            // Auto-connect
            hrConnectRequested = true;
            break;
        }
    }
    pScan->clearResults();
}

// ==========================
//      MAIN BLE TASK
// ==========================
void BLETask(void *params) {
    bleEvent = xEventGroupCreate();

    NimBLEDevice::init("VO2Max-TEEP");
    NimBLEDevice::setPower(ESP_PWR_LVL_P6);

    initBLEServer();

    sensorData_t data;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));

        // ---- Handle HR belt requests ----
        if (bleEvent) {
            EventBits_t bits = xEventGroupGetBits(bleEvent);

            if (bits & BLE_HR_SCAN) {
                xEventGroupClearBits(bleEvent, BLE_HR_SCAN);
                scanForHRBelt();
            }
            if (bits & BLE_HR_CONNECT) {
                xEventGroupClearBits(bleEvent, BLE_HR_CONNECT);
                connectToHRBelt();
            }
            if (bits & BLE_HR_DISCONNECT) {
                xEventGroupClearBits(bleEvent, BLE_HR_DISCONNECT);
                if (pClient && pClient->isConnected()) {
                    pClient->disconnect();
                }
                hrConnected = false;
            }
            if (bits & BLE_HROUT_START) {
                xEventGroupClearBits(bleEvent, BLE_HROUT_START);
                hrOutputActive = true;
                NimBLEDevice::startAdvertising();
                ESP_LOGI(TAG, "HR output started");
            }
            if (bits & BLE_HROUT_STOP) {
                xEventGroupClearBits(bleEvent, BLE_HROUT_STOP);
                hrOutputActive = false;
            }
            if (bits & BLE_GC_START) {
                xEventGroupClearBits(bleEvent, BLE_GC_START);
                gcActive = true;
                NimBLEDevice::startAdvertising();
                ESP_LOGI(TAG, "Golden Cheetah output started");
            }
            if (bits & BLE_GC_STOP) {
                xEventGroupClearBits(bleEvent, BLE_GC_STOP);
                gcActive = false;
            }
        }

        // ---- Update HR output (VO2 as BPM) ----
        if (hrOutputActive && hrClientConnected && pHRMeasurement) {
            sensorGetData(&data);
            uint8_t bpm = (uint8_t)(data.vo2 + 0.5f);
            if (bpm < 1) bpm = 1;
            if (bpm > 254) bpm = 254;

            uint8_t hrData[4] = {0x0E, bpm, 0, 0};  // Flags + BPM
            uint16_t energy = (uint16_t)(data.calories_total * 4.184f);  // kcal to kJ
            hrData[2] = energy & 0xFF;
            hrData[3] = (energy >> 8) & 0xFF;
            pHRMeasurement->setValue(hrData, 4);
            pHRMeasurement->notify();
        }

        // ---- Update Golden Cheetah ----
        if (gcActive && gcClientConnected && pGCChar) {
            sensorGetData(&data);
            GCData gc;
            gc.freq = (int16_t)data.resp_rate;
            gc.temp = (uint8_t)data.ambient_temperature;
            gc.hum = 0;
            gc.rmv = (int16_t)data.veMean;
            gc.feo2 = (int16_t)(data.o2 * 100);
            gc.vo2 = (int16_t)data.vo2;
            pGCChar->setValue((uint8_t *)&gc, sizeof(gc));
            pGCChar->notify();
        }

        // ---- Check HR belt connection ----
        if (hrConnected && pClient && !pClient->isConnected()) {
            hrConnected = false;
            ESP_LOGW(TAG, "HR belt disconnected");
        }
    }
}

// ---- Public API ----
void BLEHRConnect(void)    { if (bleEvent) xEventGroupSetBits(bleEvent, BLE_HR_CONNECT); }
void BLEHRDisconnect(void) { if (bleEvent) xEventGroupSetBits(bleEvent, BLE_HR_DISCONNECT); }
void BLEHRScan(void)       { if (bleEvent) xEventGroupSetBits(bleEvent, BLE_HR_SCAN); }
bool BLEHRIsConnected(void){ return hrConnected; }
const char *BLEHRGetName(void) { return hrSensorName; }

void BLEGCStart(void) { if (bleEvent) xEventGroupSetBits(bleEvent, BLE_GC_START); }
void BLEGCStop(void)  { if (bleEvent) xEventGroupSetBits(bleEvent, BLE_GC_STOP); }
bool BLEGCIsConnected(void) { return gcClientConnected && gcActive; }

void BLEHROutputStart(void) { if (bleEvent) xEventGroupSetBits(bleEvent, BLE_HROUT_START); }
void BLEHROutputStop(void)  { if (bleEvent) xEventGroupSetBits(bleEvent, BLE_HROUT_STOP); }
void BLEHROutputUpdate(uint8_t bpm, uint16_t energyKJ) {
    // Direct update handled in task loop
}
