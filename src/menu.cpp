/*
 * menu.cpp - Configuration menu system
 * Copyright (C) 2025 TEEP Project, based on VO2Max-500pa by Lauri Peltonen
 * GPL V3
 */

#include <TFT_eSPI.h>
#include "config.h"
#include "menu.h"
#include "buttonsTask.h"
#include "sensorTask.h"
#include "wifiTask.h"
#include "bleTask.h"
#include "files.h"

extern TFT_eSPI tft;

// ---- MenuItem base classes ----
class MenuItem {
public:
    MenuItem(const char *lbl) : label(lbl) {}
    virtual ~MenuItem() {}
    const char *getLabel() const { return label; }
    virtual void click() {}
    virtual void draw() { tft.print(label); }
protected:
    const char *label;
};

class CheckMenuItem : public MenuItem {
public:
    CheckMenuItem(const char *lbl, bool *val) : MenuItem(lbl), value(val) {}
    void click() override { *value = !(*value); }
    void draw() override { MenuItem::draw(); tft.print(*value ? " [Yes]" : " [No ]"); }
private:
    bool *value;
};

class FunctionMenuItem : public MenuItem {
public:
    FunctionMenuItem(const char *lbl, void (*fn)()) : MenuItem(lbl), fn(fn) {}
    void draw() override { MenuItem::draw(); tft.print("..."); }
    void click() override { if (fn) fn(); }
private:
    void (*fn)();
};

class SelectMenuItem : public MenuItem {
public:
    SelectMenuItem(const char *lbl, int *var, const int *vals, const char **names, int cnt)
        : MenuItem(lbl), variable(var), values(vals), choices(names), count(cnt), current(0) {
        for (int i = 0; i < count; i++) {
            if (*variable == values[i]) { current = i; break; }
        }
    }
    void click() override { current = (current + 1) % count; *variable = values[current]; }
    void draw() override { MenuItem::draw(); tft.print(" "); tft.print(choices[current]); }
private:
    int *variable;
    const int *values;
    const char **choices;
    int count, current;
};

// ---- Menu Functions ----
static void func_done() {}  // Handled by menu loop

static void func_calibrateO2() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("Calibrating O2...", 5, 55, 4);
    sensorO2Calibrate();
    vTaskDelay(pdMS_TO_TICKS(2000));
}

static void func_calibrateFlow() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("3L Calibration Pump", 5, 5, 4);
    tft.drawString("Pump NOW...", 5, 35, 4);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("Press to start >>>", 5, 105, 4);

    uint8_t btn;
    waitButtonPress(&btn, pdMS_TO_TICKS(30000));

    // Reset and measure for 10 seconds
    sensorResetCalculation();
    uint32_t startTime = millis();
    sensorData_t data;
    float totalVolume = 0;

    while (millis() - startTime < 10000) {
        sensorGetData(&data);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextPadding(10);
        tft.drawFloat(data.veMean, 1, 60, 60, 7);

        float elapsed = (millis() - startTime) / 1000.0f;
        tft.drawFloat(elapsed, 1, 120, 105, 4);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    tft.setTextPadding(0);

    sensorGetData(&data);
    totalVolume = data.veMean;  // This should be ~3L
    if (totalVolume > 0.1f) {
        float newCorr = 3.0f / totalVolume;
        if (newCorr > 0.5f && newCorr < 2.0f) {
            global_settings.flowCorrectionFactor = newCorr;
            sensorSetConfiguration();
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.drawString("Correction:", 5, 40, 4);
            tft.drawFloat(newCorr, 3, 5, 70, 4);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}

static void func_setWeight() {
    float w = global_settings.userWeight;

    tft.fillScreen(TFT_BLUE);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.drawString("Set Weight (kg)", 5, 5, 4);
    tft.drawString("+ (lower)  OK (upper)", 5, 105, 2);

    while (1) {
        tft.setTextColor(TFT_WHITE, TFT_BLUE);
        tft.setTextPadding(8);
        tft.drawFloat(w, 1, 60, 50, 7);
        tft.setTextPadding(0);

        uint8_t btn;
        if (waitButtonPress(&btn, pdMS_TO_TICKS(5000))) {
            if (btn == BUTTON_LOWER) {
                w += 0.5f;
                if (w > 200.0f) w = 30.0f;
            } else {
                break;
            }
        }
    }
    global_settings.userWeight = w;
    sensorSetConfiguration();
}

static void func_hrScan() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Scanning BLE...", 5, 55, 4);
    BLEHRScan();
    vTaskDelay(pdMS_TO_TICKS(500));
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Found:", 5, 30, 4);
    tft.drawString(BLEHRGetName(), 5, 60, 4);
    vTaskDelay(pdMS_TO_TICKS(3000));
}

static void func_saveToFile() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Saving to flash...", 5, 55, 4);

    const storeData_t *buf = getStorageBuffer();
    unsigned int pos = getStorageBufferPosition();
    write_log_buffer_to_file((const char *)buf, STORE_BUFFER_SIZE, sizeof(storeData_t), pos);

    tft.drawString("Done!", 5, 85, 4);
    vTaskDelay(pdMS_TO_TICKS(2000));
}

static void func_sendStored() {
    wifiSetConfig(WIFI_SEND_STORED, nullptr, pdMS_TO_TICKS(100));
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Sending...", 5, 55, 4);
    vTaskDelay(pdMS_TO_TICKS(3000));
}

static void func_resetValues() {
    sensorResetCalculation();
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("Values Reset!", 5, 55, 4);
    vTaskDelay(pdMS_TO_TICKS(2000));
}

// ---- Menu Item Arrays ----

// Integration time options
static const int integrationValues[] = {5000, 10000, 15000, 30000, 60000};
static const char *integrationNames[] = {"5s", "10s", "15s", "30s", "60s"};

// Venturi diameter options
static const int venturiValues[] = {16, 19, 20};
static const char *venturiNames[] = {"16mm", "19mm", "20mm"};

// Store rate options
static const int storeRateValues[] = {0, 1, 2, 4, 9};
static const char *storeRateNames[] = {"1:1", "1:2", "1:3", "1:5", "1:10"};

// All menu items
static MenuItem *menuItems[] = {
    new FunctionMenuItem("Done", func_done),
    new FunctionMenuItem("Save to file", func_saveToFile),
    new FunctionMenuItem("Send stored", func_sendStored),
    new FunctionMenuItem("Set weight", func_setWeight),
    new SelectMenuItem("Interval", &global_settings.integrationTime, integrationValues, integrationNames, 5),
    new SelectMenuItem("Venturi D", &global_settings.venturiDiameter, venturiValues, venturiNames, 3),
    new SelectMenuItem("Store rate", &global_settings.storeDataRate, storeRateValues, storeRateNames, 5),
    new FunctionMenuItem("Cal. O2", func_calibrateO2),
    new FunctionMenuItem("Cal. Flow", func_calibrateFlow),
    new CheckMenuItem("WiFi", &global_settings.wifi_enable),
    new CheckMenuItem("HR belt", &global_settings.hrsensor_enable),
    new FunctionMenuItem("Scan HR", func_hrScan),
    new CheckMenuItem("HR out", &global_settings.hroutput_enable),
    new CheckMenuItem("Cheetah", &global_settings.cheetah_enable),
    new FunctionMenuItem("Reset vals", func_resetValues),
};
static const int MENU_COUNT = sizeof(menuItems) / sizeof(menuItems[0]);

// ---- Main Menu Function ----
void doMenu(void *param) {
    int cur = 0;
    int first = 0;
    const int visible = 5;

    while (1) {
        // Draw menu
        tft.fillScreen(TFT_BLUE);
        tft.setTextColor(TFT_WHITE, TFT_BLUE);

        // Button labels
        tft.setTextDatum(TR_DATUM);
        tft.drawString(">", 235, 5, 4);
        tft.drawString("+", 235, 115, 4);
        tft.setTextDatum(TL_DATUM);

        for (int i = 0; i < visible && (first + i) < MENU_COUNT; i++) {
            int y = 5 + i * 25;
            int item = first + i;

            if (item == cur) {
                tft.setTextColor(TFT_BLUE, TFT_WHITE);
            } else {
                tft.setTextColor(TFT_WHITE, TFT_BLUE);
            }

            tft.setCursor(5, y, 4);
            tft.print(" ");
            menuItems[item]->draw();
        }

        // Wait for button
        uint8_t btn;
        if (!waitButtonPress(&btn, pdMS_TO_TICKS(30000))) {
            continue;
        }

        if (btn == BUTTON_LOWER) {
            // Navigate down
            cur++;
            if (cur >= MENU_COUNT) cur = 0;
            if (cur >= first + visible) first = cur - visible + 1;
            if (cur < first) first = cur;
        } else {
            // Select/toggle
            if (cur == 0) {
                // "Done" — save and exit
                storeSettings();
                sensorSetConfiguration();
                tft.fillScreen(TFT_BLACK);
                return;
            }
            menuItems[cur]->click();
        }
    }
}
