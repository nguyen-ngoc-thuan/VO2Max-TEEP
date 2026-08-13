/*
 * sensorTask.cpp - Main sensor task for VO2Max-TEEP
 *
 * Reads Omron D6F flow, DFRobot O2, BMP280 pressure/temperature.
 * Handles breath detection, volume integration, and VO2 calculation.
 * Runs as highest priority FreeRTOS task.
 *
 * Copyright (C) 2025 TEEP Project
 * Based on VO2Max-500pa by Lauri Peltonen (GPL V3)
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

#include "esp_log.h"
#include "config.h"
#include "sensorTask.h"
#include "vo2_calc.h"
#include "Omron_D6FPH.h"
#include "DFRobot_OxygenSensor.h"

static const char *TAG = "sensor";

// ---- Sensor Instances ----
static Omron_D6FPH flowSensor;
static DFRobot_OxygenSensor oxygenSensor;
static Adafruit_BMP280 bmp;

// ---- Timing ----
// Omron D6F has 33ms internal delay in getPressure(), so we use 50ms intervals
#define READINTERVAL    50    // ms, main task tick
#define FLOWINTERVAL    100   // ms, effective flow read interval (every other tick)
#define O2_READ_DELAY   2     // Read O2 on Nth iteration after trigger (~100ms gap)
#define NORMALIZATION_TIME 60000  // Normalize to 1 minute

// ---- Integration ----
static int INTEGRATION_TIME = 15000;  // Default 15s
static unsigned int STORE_RATE = 0;

// ---- Error tracking ----
static uint16_t errorCounter = 0;

// ---- Data Structures ----
sensorData_t sensorData;
storeData_t bufferData[STORE_BUFFER_SIZE] = {0};
uint16_t bufferPosition = 0;

// ---- FreeRTOS sync ----
static EventGroupHandle_t sensorStatus = nullptr;
static EventGroupHandle_t sensorEvent = nullptr;
static QueueHandle_t sensorQueue = nullptr;
static QueueHandle_t sensorExtraQueue = nullptr;
static sensorExtraData_t extraDataBuf;

// ---- Flow integration state ----
static float prevPressure[3] = {0};
static float currentPressure = 0.0f;
static const float pressureThreshold = 0.2f;  // Pa, start/stop exhale detection
static float integratedPressure = 0.0f;
static float integrationTotal = 0.0f;
static uint32_t integrationTime = 0;
static unsigned int storeIntervalTime = 0;
static bool integratePressure = false;
static bool newBreathData = false;
static uint32_t breathIntervalCount = 0;
static uint32_t lastBreathInterval = 0;

// ---- Venturi & correction ----
static float flowCorrectionFactor = 1.0f;

// ---- Temperature ----
static float exhale_temperature = 35.0f;
static volatile float ambient_pressure = 1013.25f;
static volatile float ambient_temperature = 20.0f;

// ---- O2 ----
float initialO2 = 20.93f;
static float currentO2 = 20.93f;
static uint8_t o2_read_step = 0;

// ---- CO2 (stub for future) ----
float initialCO2 = 0.04f;
static float currentCO2 = 0.04f;

// ---- Calculated values ----
static float lastBreathVolume = 0.0f;
static float veMax = 0.0f;
static float veMean = 0.0f;
static float vo2 = 0.0f, vo2Max = 0.0f;
static float vco2 = 0.0f, vco2Max = 0.0f;
static float averageO2 = 0.0f, averageCO2 = 0.0f;
static uint32_t averagingCount = 0;
static float weight = 75.0f;
static float caloriesPerMin = 0.0f;
static float caloriesTotal = 0.0f;
static uint32_t lastCalTime = 0;

// ---- Low-pass filter ----
static const float lpGain = 0.02f;  // ~3s time constant at 50ms intervals

// ---- Read flow sensor and integrate ----
static void readFlow(void) {
    // Shift pressure history
    prevPressure[0] = prevPressure[1];
    prevPressure[1] = prevPressure[2];
    prevPressure[2] = currentPressure;

    integrationTime += FLOWINTERVAL;
    breathIntervalCount += FLOWINTERVAL;

    float rawPressure = flowSensor.getPressure();
    if (isnan(rawPressure)) {
        errorCounter++;
        return;
    }

    currentPressure = rawPressure;
    if (currentPressure < 0.0f) currentPressure = 0.0f;

    if (integratePressure) {
        // Currently exhaling — integrate sqrt(P)
        integratedPressure += sqrt(currentPressure);
        integrationTotal += sqrt(currentPressure);

        // Detect end of exhale: 3 consecutive samples below threshold
        if (currentPressure <= pressureThreshold &&
            prevPressure[2] <= pressureThreshold &&
            prevPressure[1] <= pressureThreshold) {
            newBreathData = true;
            integratePressure = false;
        }
    } else if (currentPressure >= pressureThreshold &&
               prevPressure[2] >= pressureThreshold &&
               prevPressure[1] >= pressureThreshold) {
        // Start of new exhale detected
        integratedPressure = sqrt(currentPressure) + sqrt(prevPressure[2]) +
                             sqrt(prevPressure[1]) + sqrt(prevPressure[0]);
        integrationTotal += integratedPressure;

        lastBreathInterval = breathIntervalCount;
        breathIntervalCount = 0;
        integratePressure = true;
        newBreathData = false;
    }

    xEventGroupSetBits(sensorEvent, SENSOR_EVENT_FLOW);
}

// ---- Read ambient pressure/temperature ----
static void readPressure(void) {
    float t = bmp.readTemperature();
    float p = bmp.readPressure() / 100.0f;  // Convert Pa to hPa

    if (!isnan(t) && !isnan(p)) {
        ambient_temperature = lowPassFilter(t, ambient_temperature, lpGain);
        ambient_pressure = lowPassFilter(p, ambient_pressure, lpGain);
    } else {
        errorCounter++;
    }
}

// ---- Read O2 (two-step: trigger then read) ----
static void readO2(void) {
    if (o2_read_step == 0) {
        oxygenSensor.triggerSampling();
        o2_read_step = O2_READ_DELAY;
    } else if (o2_read_step == 1) {
        float val;
        if (oxygenSensor.readOxygenValue(val)) {
            currentO2 = val;
            if (currentO2 > initialO2) initialO2 = currentO2;  // Drift compensation
        }
    }
    if (o2_read_step > 0) o2_read_step--;
}

// ---- Calculate volumes ----
static void calculateVolumes(void) {
    if (newBreathData) {
        float vc = venturiGetConstant();
        float irho = getInverseRhoCoeff();
        float stpd = getATPStoSTDP();

        // Volume = corrFactor * venturiConst * sqrt(2/rho) * ∫sqrt(dP) * dt * STPD
        lastBreathVolume = flowCorrectionFactor * vc * irho * integratedPressure;
        lastBreathVolume *= (float)FLOWINTERVAL;  // ms → L (m³×1000)
        lastBreathVolume *= stpd;

        if (lastBreathVolume > veMax) veMax = lastBreathVolume;

        // Accumulate O2 averages
        averageO2 += currentO2;
        averageCO2 += currentCO2;
        averagingCount++;

        newBreathData = false;
        integratedPressure = 0.0f;

        xEventGroupSetBits(sensorEvent, SENSOR_EVENT_CALC);
    }

    // ---- Integration period complete ----
    if (integrationTime >= (uint32_t)INTEGRATION_TIME) {
        float vc = venturiGetConstant();
        float irho = getInverseRhoCoeff();
        float stpd = getATPStoSTDP();

        // Minute ventilation (L/min)
        veMean = flowCorrectionFactor * vc * irho * integrationTotal;
        veMean *= (float)FLOWINTERVAL;
        veMean *= (float)NORMALIZATION_TIME / (float)integrationTime;
        veMean *= stpd;

        if (averagingCount > 0) {
            float avgO2 = averageO2 / (float)averagingCount;
            float avgCO2 = averageCO2 / (float)averagingCount;

            vo2 = calcVO2(veMean, initialO2, avgO2);
            if (vo2 > vo2Max) vo2Max = vo2;

            // Calories
            float vo2Total = vo2 * weight / 1000.0f;  // Back to L/min total
            caloriesPerMin = calcCaloriesPerMin(vo2Total);
            uint32_t now = millis();
            if (lastCalTime > 0) {
                caloriesTotal += caloriesPerMin * (float)(now - lastCalTime) / 60000.0f;
            }
            lastCalTime = now;
        } else {
            vo2 = 0.0f;
            vco2 = 0.0f;
            lastBreathInterval = 0;
        }

        // Store to buffer
        storeIntervalTime++;
        if (storeIntervalTime > STORE_RATE) {
            bufferData[bufferPosition].vo2 = 1000.0f * vo2 / weight;
            bufferData[bufferPosition].ve = veMean;
            bufferData[bufferPosition].vco2 = 1000.0f * vco2 / weight;
            bufferData[bufferPosition].resp_rate = lastBreathInterval ? (60000.0f / (float)lastBreathInterval) : 0;
            bufferData[bufferPosition].hr = sensorData.hr;
            bufferData[bufferPosition].temperature = ambient_temperature;
            bufferData[bufferPosition].pressure = ambient_pressure;
            bufferData[bufferPosition].o2 = currentO2;

            bufferPosition++;
            if (bufferPosition >= STORE_BUFFER_SIZE) bufferPosition = 0;
            storeIntervalTime = 0;
        }

        // Reset accumulators
        integrationTime = 0;
        integrationTotal = 0.0f;
        averagingCount = 0;
        averageO2 = 0.0f;
        averageCO2 = 0.0f;

        xEventGroupSetBits(sensorEvent, SENSOR_EVENT_AVE);
    }

    xEventGroupSetBits(sensorEvent, SENSOR_EVENT_VOL);
}

// ---- Copy sensor data for other tasks ----
static void copySensorData(void) {
    sensorData.vo2 = 1000.0f * vo2 / weight;
    sensorData.vo2Max = 1000.0f * vo2Max / weight;
    sensorData.ve = lastBreathVolume;
    sensorData.veMax = veMax;
    sensorData.veMean = veMean;
    sensorData.vco2 = 1000.0f * vco2 / weight;
    sensorData.vco2Max = 1000.0f * vco2Max / weight;
    sensorData.rq = (vo2 > 0) ? (vco2 / vo2) : 0.0f;
    sensorData.resp_rate = (lastBreathInterval > 0) ? (60000.0f / (float)lastBreathInterval) : 0.0f;
    sensorData.calories_min = caloriesPerMin;
    sensorData.calories_total = caloriesTotal;
    sensorData.ambient_temperature = ambient_temperature;
    sensorData.ambient_pressure = ambient_pressure;
    sensorData.exhale_temperature = exhale_temperature;
    sensorData.flow_value = currentPressure;
    sensorData.o2 = currentO2;
    sensorData.co2 = currentCO2;
    sensorData.errors = errorCounter;
}

// ---- Reset calculations ----
static void resetCalculations(void) {
    memset(prevPressure, 0, sizeof(prevPressure));
    currentPressure = 0.0f;
    integrationTotal = 0.0f;
    integrationTime = 0;
    integratePressure = false;
    newBreathData = false;
    breathIntervalCount = 0;
    lastBreathInterval = 0;
    lastBreathVolume = 0.0f;
    veMax = 0.0f;
    veMean = 0.0f;
    vo2 = 0.0f; vo2Max = 0.0f;
    vco2 = 0.0f; vco2Max = 0.0f;
    averageO2 = 0.0f; averageCO2 = 0.0f;
    averagingCount = 0;
    caloriesPerMin = 0.0f;
    caloriesTotal = 0.0f;
    lastCalTime = 0;
    bufferPosition = 0;
    memset(bufferData, 0, sizeof(bufferData));
}

// ========================
//    MAIN SENSOR TASK
// ========================
void sensorTask(void *params) {
    sensorStatus = xEventGroupCreate();
    sensorEvent = xEventGroupCreate();
    sensorQueue = xQueueCreate(1, sizeof(sensorData_t));
    sensorExtraQueue = xQueueCreate(10, sizeof(sensorExtraData_t));

    copySensorData();
    xQueueOverwrite(sensorQueue, &sensorData);

    // ---- Initialize I2C (already started by Wire.begin in setup) ----

    // ---- Init Flow Sensor (Omron D6F-PH0025AMD2) ----
    ESP_LOGI(TAG, "Init: Flow sensor (Omron D6F)...");
    if (flowSensor.begin(MODEL_0025AD1)) {
        xEventGroupSetBits(sensorStatus, SENSOR_HAS_FLOW);
        ESP_LOGI(TAG, "Init: Flow sensor OK");
    } else {
        ESP_LOGE(TAG, "Init: Flow sensor FAILED");
    }

    // ---- Init BMP280 ----
    ESP_LOGI(TAG, "Init: BMP280...");
    if (bmp.begin(BMP280_I2C_ADDR)) {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X4,   // temperature
                        Adafruit_BMP280::SAMPLING_X4,   // pressure
                        Adafruit_BMP280::FILTER_X4,
                        Adafruit_BMP280::STANDBY_MS_125);
        xEventGroupSetBits(sensorStatus, SENSOR_HAS_PRESSURE);
        ESP_LOGI(TAG, "Init: BMP280 OK");
    } else {
        ESP_LOGE(TAG, "Init: BMP280 FAILED");
    }

    // ---- Init O2 Sensor ----
    ESP_LOGI(TAG, "Init: O2 sensor (DFRobot SEN0322)...");
    if (oxygenSensor.begin(O2_I2C_ADDR)) {
        xEventGroupSetBits(sensorStatus, SENSOR_HAS_O2);
        ESP_LOGI(TAG, "Init: O2 sensor OK");
    } else {
        ESP_LOGE(TAG, "Init: O2 sensor FAILED");
    }

    // ---- Load config ----
    INTEGRATION_TIME = global_settings.integrationTime;
    STORE_RATE = global_settings.storeDataRate;
    flowCorrectionFactor = global_settings.flowCorrectionFactor;
    weight = global_settings.userWeight;
    venturiInit(VENTURI_DIAMETER_OUTER, global_settings.venturiDiameter);

    resetCalculations();

    xEventGroupSetBits(sensorStatus, SENSOR_INIT_DONE);

    // ---- Main loop ----
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(READINTERVAL);
    uint8_t step = 0;

    while (1) {
        xTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Alternating sequence: FLOW, O2, FLOW, PRESSURE, FLOW, CALC
        switch (step) {
        case 0:  // Flow
            if (xEventGroupGetBits(sensorStatus) & SENSOR_HAS_FLOW)
                readFlow();
            break;
        case 1:  // O2
            if (xEventGroupGetBits(sensorStatus) & SENSOR_HAS_O2)
                readO2();
            break;
        case 2:  // Flow
            if (xEventGroupGetBits(sensorStatus) & SENSOR_HAS_FLOW)
                readFlow();
            break;
        case 3:  // Pressure + air density
            if (xEventGroupGetBits(sensorStatus) & SENSOR_HAS_PRESSURE) {
                readPressure();
                calcAirDensity(ambient_pressure, exhale_temperature);
            }
            break;
        case 4:  // Flow
            if (xEventGroupGetBits(sensorStatus) & SENSOR_HAS_FLOW)
                readFlow();
            break;
        case 5:  // Volume calculation
            calculateVolumes();
            break;
        }

        step++;
        if (step > 5) step = 0;

        // ---- Handle events from other tasks ----
        if (xEventGroupGetBits(sensorEvent) & SENSOR_EVENT_INIT) {
            initialO2 = currentO2;
            initialCO2 = currentCO2;
            xEventGroupClearBits(sensorEvent, SENSOR_EVENT_INIT);
        }
        if (xEventGroupGetBits(sensorEvent) & SENSOR_EVENT_RESET) {
            resetCalculations();
            xEventGroupClearBits(sensorEvent, SENSOR_EVENT_RESET);
        }
        if (xEventGroupGetBits(sensorEvent) & SENSOR_EVENT_O2_CAL) {
            oxygenSensor.calibrate(20.9f, 0.0f);
            xEventGroupClearBits(sensorEvent, SENSOR_EVENT_O2_CAL);
        }
        if (xEventGroupGetBits(sensorEvent) & SENSOR_EVENT_RECONF) {
            flowCorrectionFactor = global_settings.flowCorrectionFactor;
            weight = global_settings.userWeight;
            INTEGRATION_TIME = global_settings.integrationTime;
            STORE_RATE = global_settings.storeDataRate;
            venturiInit(VENTURI_DIAMETER_OUTER, global_settings.venturiDiameter);
            xEventGroupClearBits(sensorEvent, SENSOR_EVENT_RECONF);
        }

        // ---- Receive extra data (HR etc.) ----
        while (xQueueReceive(sensorExtraQueue, &extraDataBuf, 0) == pdTRUE) {
            switch (extraDataBuf.type) {
            case SENSOR_EXTRA_HR:
                sensorData.hr = extraDataBuf.value;
                if (extraDataBuf.value > sensorData.hr_max) sensorData.hr_max = extraDataBuf.value;
                break;
            case SENSOR_EXTRA_RR:
                sensorData.rr = extraDataBuf.value;
                break;
            }
        }

        // ---- Update shared data ----
        copySensorData();
        xQueueOverwrite(sensorQueue, &sensorData);
    }
}

// ---- Public API ----

bool sensorGetData(sensorData_t *copy_to) {
    return xQueuePeek(sensorQueue, copy_to, 0) == pdTRUE;
}

void sensorClearEvent(uint32_t event) {
    xEventGroupClearBits(sensorEvent, event);
}

bool sensorWaitEvent(uint32_t event, TickType_t timeout) {
    if (!sensorEvent) { vTaskDelay(timeout); return false; }
    return (xEventGroupWaitBits(sensorEvent, event, pdTRUE, pdFALSE, timeout) & event) ? true : false;
}

bool sensorAddExtraValue(uint8_t type, float value, TickType_t timeout) {
    sensorExtraData_t data = {type, value};
    return xQueueSendToBack(sensorExtraQueue, (void *)&data, timeout) == pdTRUE;
}

uint16_t sensorGetStatus() { return xEventGroupGetBits(sensorStatus); }
void sensorSetInitial(void) { xEventGroupSetBits(sensorEvent, SENSOR_EVENT_INIT); }
void sensorResetCalculation(void) { xEventGroupSetBits(sensorEvent, SENSOR_EVENT_RESET); }
void sensorO2Calibrate(void) { xEventGroupSetBits(sensorEvent, SENSOR_EVENT_O2_CAL); }
void sensorSetConfiguration(void) { xEventGroupSetBits(sensorEvent, SENSOR_EVENT_RECONF); }
bool sensorHasErrors(void) { return errorCounter > 0; }
const storeData_t *getStorageBuffer(void) { return bufferData; }
unsigned int getStorageBufferPosition(void) { return bufferPosition; }
