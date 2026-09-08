/*
 * main.cpp - VO2Max-TEEP: Improved DIY portable spirometer
 *
 * Main entry point: initializes hardware, creates FreeRTOS tasks,
 * runs the GUI loop (display + buttons + menu).
 *
 * Sensors: Omron D6F-PH0025AMD2, DFRobot SEN0322, BMP280
 * Board:   TTGO T-Display (ESP32 + 1.14" TFT)
 *
 * Copyright (C) 2025 TEEP Project
 * GPL V3
 */

#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

#include "config.h"
#include "sensorTask.h"
#include "buttonsTask.h"
#include "wifiTask.h"
#include "bleTask.h"
#include "menu.h"
#include "status.h"
#include "files.h"

// ---- Version ----
#define VERSION "TEEP v1.0"

// ---- TFT display ----
TFT_eSPI tft = TFT_eSPI();

// ---- Global settings ----
settings_t global_settings;
Preferences preferences;

// ---- Battery ----
float battery_voltage = 0.0f;
static uint16_t vref = 1100;

// ---- Timer ----
volatile uint32_t seconds_from_start = 0;
static uint32_t timer_start_ms = 0;

// ---- Task handles ----
static TaskHandle_t sensorTaskHandle = nullptr;
static TaskHandle_t wifiTaskHandle = nullptr;
static TaskHandle_t bleTaskHandle = nullptr;

// ---- Load/Store Settings via NVS ----
static void loadSettings(void) {
    preferences.begin("vo2max", true);  // read-only

    global_settings.flowCorrectionFactor = preferences.getFloat("flowCorr", 1.0f);
    global_settings.userWeight = preferences.getFloat("weight", 75.0f);
    global_settings.venturiDiameter = preferences.getInt("venturiD", 20);
    global_settings.integrationTime = preferences.getInt("intTime", 15000);
    global_settings.storeDataRate = preferences.getInt("storeRate", 0);
    global_settings.wifiDataRate = preferences.getInt("wifiRate", 0);
    global_settings.hrsensor_enable = preferences.getBool("hrSens", false);
    global_settings.hroutput_enable = preferences.getBool("hrOut", false);
    global_settings.wifi_enable = preferences.getBool("wifi", false);
    global_settings.cheetah_enable = preferences.getBool("cheetah", false);
    global_settings.btserial_enable = preferences.getBool("btserial", false);
    global_settings.co2sensor_enable = preferences.getBool("co2", false);

    String ssid = preferences.getString("wifiName", "VO2MAX_");
    strncpy(global_settings.wifiStationName, ssid.c_str(), 31);

    String pass = preferences.getString("wifiPass", "4321asdf");
    strncpy(global_settings.wifiPassword, pass.c_str(), 31);

    size_t len = preferences.getBytes("hrAddr", global_settings.HRSensorAddress, 6);
    if (len != 6) memset(global_settings.HRSensorAddress, 0, 6);

    preferences.end();
}

void storeSettings(void) {
    preferences.begin("vo2max", false);  // read-write

    preferences.putFloat("flowCorr", global_settings.flowCorrectionFactor);
    preferences.putFloat("weight", global_settings.userWeight);
    preferences.putInt("venturiD", global_settings.venturiDiameter);
    preferences.putInt("intTime", global_settings.integrationTime);
    preferences.putInt("storeRate", global_settings.storeDataRate);
    preferences.putInt("wifiRate", global_settings.wifiDataRate);
    preferences.putBool("hrSens", global_settings.hrsensor_enable);
    preferences.putBool("hrOut", global_settings.hroutput_enable);
    preferences.putBool("wifi", global_settings.wifi_enable);
    preferences.putBool("cheetah", global_settings.cheetah_enable);
    preferences.putBool("btserial", global_settings.btserial_enable);
    preferences.putBool("co2", global_settings.co2sensor_enable);
    preferences.putString("wifiName", global_settings.wifiStationName);
    preferences.putString("wifiPass", global_settings.wifiPassword);
    preferences.putBytes("hrAddr", global_settings.HRSensorAddress, 6);

    preferences.end();
}

// ---- Battery voltage ----
static void readBattery(void) {
    uint16_t v = analogRead(PIN_BAT_VOLT);
    battery_voltage = ((float)v / 4095.0f) * 2.0f * 3.3f * (vref / 1000.0f);
}

// ============================================================
//                         SETUP
// ============================================================
void setup() {
    // ---- Serial ----
    Serial.begin(115200);
    Serial.println();
    Serial.println(VERSION);

    // ---- Battery ADC ----
    pinMode(PIN_ADC_EN, OUTPUT);
    digitalWrite(PIN_ADC_EN, HIGH);
    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
        ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        vref = adc_chars.vref;
    }

    // ---- TFT ----
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("VO2Max-TEEP", 120, 10, 4);
    tft.drawString(VERSION, 120, 40, 4);
    tft.drawString("Initializing...", 120, 80, 4);
    readBattery();
    delay(1500);

    // ---- I2C ----
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(100000);  // 100kHz for Omron compatibility

    // ---- Buttons ----
    buttonInit();
    attachInterrupt(BUTTON_1_PIN, buttonInterrupt, CHANGE);
    attachInterrupt(BUTTON_2_PIN, buttonInterrupt, CHANGE);

    // ---- Load Settings ----
    loadSettings();

    // ---- Filesystem ----
    init_filesystem();

    // ---- Create FreeRTOS Tasks ----
    xTaskCreatePinnedToCore(sensorTask, "sensor", 8192, nullptr, 8, &sensorTaskHandle, 1);
    xTaskCreatePinnedToCore(wifiTask, "wifi", 8192, nullptr, 4, &wifiTaskHandle, 0);
    xTaskCreatePinnedToCore(BLETask, "ble", 8192, nullptr, 3, &bleTaskHandle, 0);

    // Wait for sensor init
    tft.drawString("Sensors...", 120, 100, 4);
    while (!(sensorGetStatus() & SENSOR_INIT_DONE)) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Show sensor status
    tft.fillScreen(TFT_BLACK);
    uint16_t status = sensorGetStatus();
    int y = 10;

    tft.setTextDatum(TL_DATUM);
    tft.setTextColor((status & SENSOR_HAS_FLOW) ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.drawString("Flow sensor", 5, y, 4);
    tft.drawString((status & SENSOR_HAS_FLOW) ? "OK" : "FAIL", 180, y, 4);
    y += 25;

    tft.setTextColor((status & SENSOR_HAS_O2) ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.drawString("O2 sensor", 5, y, 4);
    tft.drawString((status & SENSOR_HAS_O2) ? "OK" : "FAIL", 180, y, 4);
    y += 25;

    tft.setTextColor((status & SENSOR_HAS_PRESSURE) ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.drawString("BMP280", 5, y, 4);
    tft.drawString((status & SENSOR_HAS_PRESSURE) ? "OK" : "FAIL", 180, y, 4);
    y += 25;

    readBattery();
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    char batBuf[16];
    snprintf(batBuf, sizeof(batBuf), "Battery: %.2fV", battery_voltage);
    tft.drawString(batBuf, 5, y, 4);

    delay(2000);

    // ---- Menu (optional) ----
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("Press [>] for Menu", 120, 30, 4);
    tft.drawString("or wait to start", 120, 60, 4);

    uint8_t btn;
    if (waitButtonPress(&btn, pdMS_TO_TICKS(3000))) {
        if (btn == BUTTON_UPPER) {
            doMenu(nullptr);
        }
    }

    // ---- Start WiFi if enabled ----
    if (global_settings.wifi_enable) {
        wifiRequestStart();
    }

    // ---- Start BLE outputs if enabled ----
    if (global_settings.hroutput_enable) {
        BLEHROutputStart();
    }
    if (global_settings.cheetah_enable) {
        BLEGCStart();
    }
    if (global_settings.hrsensor_enable) {
        BLEHRConnect();
    }

    // ---- O2 Warm-up (3 min countdown, skippable) ----
    statusInitial();

    // ---- Ready ----
    initScreens();
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("Ready!", 120, 55, 4);
    delay(1000);
    tft.fillScreen(TFT_BLACK);

    timer_start_ms = millis();
}

// ============================================================
//                       MAIN LOOP
// ============================================================
void loop() {
    // Update timer
    seconds_from_start = (millis() - timer_start_ms) / 1000;

    // Update display
    showScreen();

    // Read battery every ~30 loops (~15s)
    static uint8_t batteryCounter = 0;
    batteryCounter++;
    if (batteryCounter > 30) {
        readBattery();
        batteryCounter = 0;
    }

    // Check buttons
    uint8_t btn;
    if (waitButtonPress(&btn, pdMS_TO_TICKS(500))) {
        if (btn == BUTTON_LOWER) {
            changeScreen(false);
        } else if (btn == BUTTON_UPPER) {
            changeScreen(true);
        }
    }

    // Check for long press (both buttons) → restart
    if (isButtonPressed(BUTTON_LOWER) && isButtonPressed(BUTTON_UPPER)) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        if (isButtonPressed(BUTTON_LOWER) && isButtonPressed(BUTTON_UPPER)) {
            ESP.restart();
        }
    }
}
