/*
 * wifiTask.h - WiFi task interface
 * Copyright (C) 2025 TEEP Project, based on VO2Max-500pa by Lauri Peltonen
 * GPL V3
 */
#ifndef __WIFITASK_H__
#define __WIFITASK_H__

#include <freertos/event_groups.h>

extern EventGroupHandle_t wifiEvent;

#define WIFI_REQUEST_START      0x0004
#define WIFI_STATUS_STARTED     0x0008
#define WIFI_REQUEST_STOP       0x0010
#define WIFI_STATUS_STOPPED     0x0020
#define WIFI_HAS_CLIENTS        0x0100

// WiFi config commands
typedef struct _wifiSetting_t {
    uint8_t type;
    void *value;
} wifiSetting_t;

#define WIFI_SETTING_NAME     0x01
#define WIFI_SETTING_PASS     0x02
#define WIFI_SEND_STORED      0x10
#define WIFI_SEND_FILE        0x20

// UDP Packet status codes
#define WIFI_PACKET_REALTIME  0x01
#define WIFI_PACKET_STORED    0x02

// UDP data packet structure
typedef struct __attribute__((packed)) _wifiUdpPacket_t {
    uint8_t  status;
    uint8_t  dummy1;
    uint16_t errors;
    float flow;
    float o2;
    float ve;
    float vo2;
    float vco2;
    float resp_rate;
    float pressure;
    float temperature;
    float exhale_temperature;
    float hr;
    float rr;
} wifiUdpPacket_t;

void wifiTask(void *params);
void wifiRequestStart(void);
void wifiRequestStop(void);
bool wifiSetConfig(uint8_t type, void *param, TickType_t timeout);
uint16_t wifiGetStatus(void);

#endif
