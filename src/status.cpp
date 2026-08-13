/*
 * status.cpp - Status display screens with gauge widgets
 * Copyright (C) 2025 TEEP Project, based on VO2Max-500pa by Lauri Peltonen
 * GPL V3
 */

#include <TFT_eSPI.h>
#include "config.h"
#include "status.h"
#include "sensorTask.h"
#include "buttonsTask.h"
#include "wifiTask.h"
#include "bleTask.h"

extern TFT_eSPI tft;
extern float battery_voltage;
extern volatile uint32_t seconds_from_start;

static sensorData_t sData;
static int currentScreen = 0;
static const int NUM_SCREENS = 5;

// ---- Draw status bar (top row) ----
static void drawStatusBar(void) {
    uint32_t s = seconds_from_start;
    uint16_t h = s / 3600; s %= 3600;
    uint16_t m = s / 60; s %= 60;

    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    char buf[20];
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", h, m, (int)s);
    tft.drawString(buf, 5, 2, 4);

    // Battery voltage with color
    uint16_t batColor = TFT_GREEN;
    if (battery_voltage < 3.3f) batColor = TFT_RED;
    else if (battery_voltage < 3.6f) batColor = TFT_YELLOW;
    else if (battery_voltage > 4.5f) batColor = TFT_CYAN;  // USB power

    tft.setTextColor(batColor, TFT_BLACK);
    snprintf(buf, sizeof(buf), "%.1fV", battery_voltage);
    tft.drawString(buf, 165, 2, 2);

    // Sensor status icons
    int x = 200;
    uint16_t status = sensorGetStatus();

    tft.setTextColor((status & SENSOR_HAS_FLOW) ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.drawString("F", x, 2, 2); x += 12;

    tft.setTextColor((status & SENSOR_HAS_O2) ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.drawString("O", x, 2, 2); x += 12;

    tft.setTextColor((status & SENSOR_HAS_PRESSURE) ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.drawString("P", x, 2, 2);

    // WiFi/BLE status on second line
    x = 200;
    uint16_t wStat = wifiGetStatus();
    if (wStat & WIFI_HAS_CLIENTS)
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
    else if (wStat & WIFI_STATUS_STARTED)
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    else
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString("W", x, 12, 2); x += 12;

    tft.setTextColor(BLEHRIsConnected() ? TFT_GREEN : TFT_DARKGREY, TFT_BLACK);
    tft.drawString("H", x, 12, 2); x += 12;

    tft.setTextColor(BLEGCIsConnected() ? TFT_GREEN : TFT_DARKGREY, TFT_BLACK);
    tft.drawString("G", x, 12, 2);

    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
}

// ---- Screen 1: VO2 + HR (large) ----
static void drawScreen1(void) {
    tft.setTextDatum(TL_DATUM);
    tft.setTextPadding(8);

    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString("VO2", 5, 28, 4);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawFloat(sData.vo2, 1, 70, 25, 6);

    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("HR", 5, 75, 4);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawFloat(sData.hr, 0, 70, 72, 6);

    tft.setTextPadding(0);
}

// ---- Screen 2: VO2/VE/VCO2 with max values ----
static void drawScreen2(void) {
    tft.setTextDatum(TL_DATUM);
    tft.setTextPadding(5);

    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString("Vo2", 5, 30, 4);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawFloat(sData.vo2, 1, 65, 30, 4);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString("/", 135, 30, 4);
    tft.drawFloat(sData.vo2Max, 1, 150, 30, 4);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("Ve", 5, 57, 4);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawFloat(sData.veMean, 1, 65, 57, 4);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString("/", 135, 57, 4);
    tft.drawFloat(sData.veMax, 1, 150, 57, 4);

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString("Vco2", 5, 84, 4);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawFloat(sData.vco2, 1, 65, 84, 4);

    tft.setTextPadding(0);
}

// ---- Screen 3: Breathing ----
static void drawScreen3(void) {
    tft.setTextDatum(TL_DATUM);
    tft.setTextPadding(5);

    tft.drawString("Rate", 5, 30, 4);
    tft.drawFloat(sData.resp_rate, 1, 80, 30, 4);
    tft.drawString("b/m", 170, 30, 4);

    tft.drawString("RQ", 5, 57, 4);
    tft.drawFloat(sData.rq, 2, 80, 57, 4);

    tft.drawString("VT", 5, 84, 4);
    tft.drawFloat(sData.ve, 2, 80, 84, 4);
    tft.drawString("L", 170, 84, 4);

    tft.setTextPadding(0);
}

// ---- Screen 4: Environment ----
static void drawScreen4(void) {
    tft.setTextDatum(TL_DATUM);
    tft.setTextPadding(5);

    tft.drawString("Pa", 5, 30, 4);
    tft.drawFloat(sData.ambient_pressure, 1, 60, 30, 4);
    tft.drawString("hPa", 175, 30, 4);

    tft.drawString("Ta", 5, 57, 4);
    tft.drawFloat(sData.ambient_temperature, 1, 60, 57, 4);
    tft.drawString("C", 175, 57, 4);

    tft.drawString("Cal", 5, 84, 4);
    tft.drawFloat(sData.calories_total, 0, 60, 84, 4);
    tft.drawString("kcal", 165, 84, 4);

    tft.setTextPadding(0);
}

// ---- Screen 5: Status/Config ----
static void drawScreen5(void) {
    tft.setTextDatum(TL_DATUM);
    tft.setTextPadding(5);

    tft.drawString("O2", 5, 25, 4);
    tft.drawFloat(sData.o2, 2, 60, 25, 4);
    tft.drawString("%", 170, 25, 4);

    tft.drawString("kg", 5, 50, 4);
    tft.drawFloat(global_settings.userWeight, 1, 60, 50, 4);

    tft.drawString("k", 5, 75, 4);
    tft.drawFloat(global_settings.flowCorrectionFactor, 3, 60, 75, 4);

    char buf[8];
    snprintf(buf, sizeof(buf), "D%d", global_settings.venturiDiameter);
    tft.drawString(buf, 5, 100, 4);

    tft.setTextPadding(0);
}

// ---- O2 warm-up screen ----
void statusInitial(void) {
    uint16_t status = sensorGetStatus();
    if (!(status & SENSOR_HAS_O2)) return;

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("O2 Sensor Warm-up", 120, 5, 4);

    for (int i = 180; i >= 0; i--) {
        sensorGetData(&sData);

        tft.setTextDatum(TC_DATUM);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        char buf[16];
        snprintf(buf, sizeof(buf), "%d s", i);
        tft.drawString(buf, 120, 40, 6);

        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        snprintf(buf, sizeof(buf), "O2: %.2f %%", sData.o2);
        tft.drawString(buf, 120, 90, 4);

        // Check for skip button
        uint8_t btn = 0;
        if (waitButtonPress(&btn, pdMS_TO_TICKS(1000))) {
            break;
        }
    }

    // Set initial O2 level
    sensorSetInitial();
    tft.fillScreen(TFT_BLACK);
}

// ---- Public API ----
void initScreens(void) {
    currentScreen = 0;
}

void changeScreen(bool forward) {
    if (forward) {
        currentScreen++;
        if (currentScreen >= NUM_SCREENS) currentScreen = 0;
    } else {
        currentScreen--;
        if (currentScreen < 0) currentScreen = NUM_SCREENS - 1;
    }
    tft.fillScreen(TFT_BLACK);
}

void showScreen(void) {
    sensorGetData(&sData);
    drawStatusBar();

    switch (currentScreen) {
    case 0: drawScreen1(); break;
    case 1: drawScreen2(); break;
    case 2: drawScreen3(); break;
    case 3: drawScreen4(); break;
    case 4: drawScreen5(); break;
    }
}
