/*
 * wifiTask.cpp - WiFi AP + UDP broadcast for real-time data streaming
 * Copyright (C) 2025 TEEP Project, based on VO2Max-500pa by Lauri Peltonen
 * GPL V3
 */

#include <WiFi.h>
#include <AsyncUDP.h>
#include "esp_wifi.h"
#include "esp_mac.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "config.h"
#include "wifiTask.h"
#include "sensorTask.h"
#include "files.h"

static const char *TAG = "WiFi";

static AsyncUDP udp;

static char wifiStationName[32] = "VO2MAX_";
static char wifiPassword[32]    = "4321asdf";
static int wifiChannel = 1;
static IPAddress wifiIP(192, 168, 30, 1);
static IPAddress wifiNetmask(255, 255, 255, 0);
static int wifiBcastPort = 8008;

static wifiUdpPacket_t packet;
EventGroupHandle_t wifiEvent = nullptr;
static QueueHandle_t wifiQueue = nullptr;
static wifiSetting_t wifiSettingBuf;
static sensorData_t sensor_data_buf;

static uint8_t mac_base[6] = {0};
static char SERIALNUMBER[5] = {0};

// Build UDP packet from sensor data
static void createPacket(void) {
    packet.status = WIFI_PACKET_REALTIME;
    packet.dummy1 = 0;
    packet.errors = sensor_data_buf.errors;
    packet.flow = sensor_data_buf.flow_value;
    packet.o2 = sensor_data_buf.o2;
    packet.ve = sensor_data_buf.veMean;
    packet.vo2 = sensor_data_buf.vo2;
    packet.vco2 = sensor_data_buf.vco2;
    packet.resp_rate = sensor_data_buf.resp_rate;
    packet.pressure = sensor_data_buf.ambient_pressure;
    packet.temperature = sensor_data_buf.ambient_temperature;
    packet.exhale_temperature = sensor_data_buf.exhale_temperature;
    packet.hr = sensor_data_buf.hr;
    packet.rr = sensor_data_buf.rr;
}

void wifiTask(void *params) {
    EventBits_t wifiEventCode = SENSOR_EVENT_FLOW;

    wifiEvent = xEventGroupCreate();
    xEventGroupSetBits(wifiEvent, WIFI_STATUS_STOPPED);

    wifiQueue = xQueueCreate(10, sizeof(wifiSetting_t));

    // Copy config
    strncpy(wifiStationName, global_settings.wifiStationName, 31);
    strncpy(wifiPassword, global_settings.wifiPassword, 31);

    if (esp_efuse_mac_get_default(mac_base) == ESP_OK) {
        snprintf(SERIALNUMBER, 5, "%02X%02X", mac_base[4], mac_base[5]);
    }
    strncat(wifiStationName, SERIALNUMBER, 31 - strlen(wifiStationName));

    ESP_LOGI(TAG, "AP Name: %s", wifiStationName);

    for (;;) {
        // Select data rate
        switch (global_settings.wifiDataRate) {
        case 2:  wifiEventCode = SENSOR_EVENT_AVE;  break;   // ~15s
        case 1:  wifiEventCode = SENSOR_EVENT_VOL;  break;   // ~300ms
        default: wifiEventCode = SENSOR_EVENT_FLOW; break;    // ~50ms
        }

        if (sensorWaitEvent(wifiEventCode, pdMS_TO_TICKS(1000))) {
            if (xEventGroupGetBits(wifiEvent) & WIFI_STATUS_STARTED) {
                sensorGetData(&sensor_data_buf);
                createPacket();
                udp.broadcastTo((uint8_t *)&packet, sizeof(packet), wifiBcastPort);
            }
        }

        // Track client connections
        if (WiFi.softAPgetStationNum()) {
            xEventGroupSetBits(wifiEvent, WIFI_HAS_CLIENTS);
        } else {
            xEventGroupClearBits(wifiEvent, WIFI_HAS_CLIENTS);
        }

        // ---- Start WiFi ----
        if ((xEventGroupGetBits(wifiEvent) & WIFI_REQUEST_START) &&
            !(xEventGroupGetBits(wifiEvent) & WIFI_STATUS_STARTED)) {
            xEventGroupClearBits(wifiEvent, WIFI_REQUEST_START);
            ESP_LOGI(TAG, "Starting WiFi AP...");

            WiFi.setHostname(wifiStationName);
            WiFi.softAP(wifiStationName, wifiPassword, wifiChannel);
            vTaskDelay(pdMS_TO_TICKS(500));
            WiFi.softAPConfig(wifiIP, wifiIP, wifiNetmask);

            ESP_LOGI(TAG, "WiFi AP [%s] started", wifiStationName);
            xEventGroupClearBits(wifiEvent, WIFI_STATUS_STOPPED | WIFI_HAS_CLIENTS);
            xEventGroupSetBits(wifiEvent, WIFI_STATUS_STARTED);
        }

        // ---- Stop WiFi ----
        if ((xEventGroupGetBits(wifiEvent) & WIFI_REQUEST_STOP) &&
            !(xEventGroupGetBits(wifiEvent) & WIFI_STATUS_STOPPED)) {
            xEventGroupClearBits(wifiEvent, WIFI_REQUEST_STOP);
            ESP_LOGI(TAG, "Stopping WiFi...");

            esp_wifi_deauth_sta(0);
            WiFi.softAPdisconnect(true);

            xEventGroupClearBits(wifiEvent, WIFI_STATUS_STARTED | WIFI_HAS_CLIENTS);
            xEventGroupSetBits(wifiEvent, WIFI_STATUS_STOPPED);
        }

        // ---- Handle config changes ----
        while (xQueueReceive(wifiQueue, &wifiSettingBuf, 0) == pdTRUE) {
            switch (wifiSettingBuf.type) {
            case WIFI_SETTING_NAME:
                strncpy(wifiStationName, (char *)wifiSettingBuf.value, 31);
                strncat(wifiStationName, SERIALNUMBER, 31 - strlen(wifiStationName));
                break;
            case WIFI_SETTING_PASS:
                strncpy(wifiPassword, (char *)wifiSettingBuf.value, 31);
                break;
            case WIFI_SEND_STORED: {
                // Send ring buffer over UDP
                const storeData_t *buf = getStorageBuffer();
                unsigned int pos = getStorageBufferPosition();
                ESP_LOGI(TAG, "Sending stored data (%d items)...", STORE_BUFFER_SIZE);
                for (int i = 0; i < STORE_BUFFER_SIZE; i++) {
                    int idx = (pos + i) % STORE_BUFFER_SIZE;
                    udp.broadcastTo((uint8_t *)&buf[idx], sizeof(storeData_t), wifiBcastPort);
                    vTaskDelay(pdMS_TO_TICKS(5));
                }
                ESP_LOGI(TAG, "Stored data sent");
                break;
            }
            case WIFI_SEND_FILE: {
                // Send file over UDP
                if (open_data_file()) {
                    uint8_t chunk[512];
                    while (read_data_file((char *)chunk, sizeof(chunk))) {
                        udp.broadcastTo(chunk, sizeof(chunk), wifiBcastPort);
                        vTaskDelay(pdMS_TO_TICKS(5));
                    }
                    close_data_file();
                    ESP_LOGI(TAG, "File data sent");
                }
                break;
            }
            }
        }
    }
}

void wifiRequestStart(void) {
    if (wifiEvent) xEventGroupSetBits(wifiEvent, WIFI_REQUEST_START);
}

void wifiRequestStop(void) {
    if (wifiEvent) xEventGroupSetBits(wifiEvent, WIFI_REQUEST_STOP);
}

bool wifiSetConfig(uint8_t type, void *param, TickType_t timeout) {
    wifiSetting_t s = {type, param};
    if (wifiQueue) return xQueueSendToBack(wifiQueue, &s, timeout) == pdTRUE;
    return false;
}

uint16_t wifiGetStatus(void) {
    if (wifiEvent) return xEventGroupGetBits(wifiEvent);
    return 0;
}
