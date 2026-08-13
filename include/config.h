/*
 * config.h - Global configuration for VO2Max-TEEP
 *
 * Copyright (C) 2025 TEEP Project
 * Based on VO2Max-500pa by Lauri Peltonen and VO2max-main by meteoscientific
 * GPL V3
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <stdint.h>

// ---- NVS Settings ----
#define SETTINGS_MAGIC     0xAC
#define SETTINGS_VERSION   1

// ---- Pin Definitions (TTGO T-Display) ----
#define PIN_SDA           21
#define PIN_SCL           22
#define BUTTON_1_PIN      0      // Lower button (GPIO0)
#define BUTTON_2_PIN      35     // Upper button (GPIO35)
#define PIN_ADC_EN        14     // ADC detection enable
#define PIN_BAT_VOLT      34     // Battery voltage ADC pin

// ---- Button Codes ----
#define BUTTON_LOWER      0x01
#define BUTTON_UPPER      0x02

// ---- I2C ----
#define I2C_PORT          I2C_NUM_0

// ---- I2C Sensor Addresses ----
#define OMRON_I2C_ADDR    0x6C   // Omron D6F-PH0025AMD2
#define O2_I2C_ADDR       0x73   // DFRobot SEN0322
#define BMP280_I2C_ADDR   0x76   // BMP280 (SDO→GND)

// ---- Venturi Tube Geometry ----
// Outer diameter is always 26mm, inner selectable
#define VENTURI_DIAMETER_OUTER  26   // mm (fixed)
// Inner diameter options: 16, 19, 20 mm (selected in settings)

// ---- Data Storage ----
#define STORE_BUFFER_SIZE   1024   // Ring buffer sample count
#define DATA_FILE_MAGIC     0xDA
#define DATA_FILE_VERSION   1

// ---- Settings Structure (stored in NVS) ----
typedef struct __attribute__((packed)) _settings_t {
    float flowCorrectionFactor;   // Correction for flow calculation (from 3L syringe)
    float userWeight;             // User weight in kg
    int   venturiDiameter;        // Inner venturi diameter in mm (16, 19, or 20)
    int   integrationTime;        // Integration time in ms (5000, 10000, 15000, 30000, 60000)
    int   storeDataRate;          // Store rate: 0=all, 1=1:2, 2=1:3, 4=1:5, 9=1:10
    int   wifiDataRate;           // WiFi data rate: 0=all, 1=calc, 2=average
    bool  hrsensor_enable;        // Enable BLE HR belt reader
    bool  hroutput_enable;        // Enable BLE HR output (VO2 as BPM for Zwift)
    bool  wifi_enable;            // Enable WiFi AP
    bool  cheetah_enable;         // Enable Golden Cheetah BLE output
    bool  btserial_enable;        // Enable Bluetooth Serial CSV output
    bool  co2sensor_enable;       // CO2 sensor (future use)
    uint8_t HRSensorAddress[6];   // BLE HR sensor MAC address
    char  wifiStationName[32];    // WiFi AP name
    char  wifiPassword[32];       // WiFi AP password
} settings_t;

extern settings_t global_settings;

// ---- ADC Calibration ----
#define ADC_CUSTOM_GAIN    2.89f
#define ADC_CUSTOM_OFFSET  0.546f

// ---- Function Declarations ----
void storeSettings(void);

#endif // __CONFIG_H__
