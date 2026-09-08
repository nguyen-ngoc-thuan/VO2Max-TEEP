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
static const float startThresholdPa = 0.25f;  // Hysteresis: start exhale detection
static const float stopThresholdPa = 0.12f;   // Hysteresis: end exhale detection
static float pressureZeroPa = 0.0f;           // Zero offset from calibration
static float integratedPressure = 0.0f;
static float integrationTotal = 0.0f;
static uint32_t integrationTime = 0;
static unsigned int storeIntervalTime = 0;
static bool integratePressure = false;
static bool newBreathData = false;
static bool hasPreviousBreath = false;    // Skip first breath interval
static uint32_t breathIntervalCount = 0;
static uint32_t lastBreathInterval = 0;
static uint32_t previousFlowTimeUs = 0;   // For real dt integration

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

// ---- Time-based O2 averaging ----
static float o2Sum = 0.0f;
static uint32_t o2SampleCount = 0;
static bool fiO2Locked = false;           // Lock FiO2 after warm-up

// ---- Ring buffer state ----
static bool bufferWrapped = false;
static uint32_t totalStoredSamples = 0;

// ---- Low-pass filter ----
static const float lpGain = 0.02f;  // ~3s time constant at 50ms intervals

// ---- Read flow sensor and integrate ----
static void readFlow(void) {
    // Shift pressure history
    prevPressure[0] = prevPressure[1];
    prevPressure[1] = prevPressure[2];
    prevPressure[2] = currentPressure;

    // Calculate real dt instead of assuming fixed FLOWINTERVAL
    uint32_t nowUs = micros();
    float dtMs = (previousFlowTimeUs > 0) ? (float)(nowUs - previousFlowTimeUs) / 1000.0f : (float)FLOWINTERVAL;
    previousFlowTimeUs = nowUs;

    integrationTime += (uint32_t)dtMs;
    breathIntervalCount += (uint32_t)dtMs;

    float rawPressure = flowSensor.getPressure();
    if (isnan(rawPressure)) {
        errorCounter++;
        return;
    }

    // Apply zero offset calibration
    currentPressure = rawPressure - pressureZeroPa;
    if (currentPressure < 0.0f) currentPressure = 0.0f;

    if (integratePressure) {
        // Currently exhaling — integrate sqrt(P) weighted by real dt
        float sqrtP = sqrt(currentPressure);
        integratedPressure += sqrtP * (dtMs / (float)FLOWINTERVAL);  // Normalize to equivalent samples
        integrationTotal += sqrtP * (dtMs / (float)FLOWINTERVAL);

        // Detect end of exhale: 3 consecutive samples below stop threshold (hysteresis)
        if (currentPressure <= stopThresholdPa &&
            prevPressure[2] <= stopThresholdPa &&
            prevPressure[1] <= stopThresholdPa) {
            newBreathData = true;
            integratePressure = false;
        }
    } else if (currentPressure >= startThresholdPa &&
               prevPressure[2] >= startThresholdPa &&
               prevPressure[1] >= startThresholdPa) {
        // Start of new exhale detected (hysteresis: higher threshold to start)
        integratedPressure = sqrt(currentPressure) + sqrt(prevPressure[2]) +
                             sqrt(prevPressure[1]) + sqrt(prevPressure[0]);
        integrationTotal += integratedPressure;

        // Only use breath interval from second breath onwards
        if (hasPreviousBreath) {
            lastBreathInterval = breathIntervalCount;
        }
        hasPreviousBreath = true;
        breathIntervalCount = 0;
        integratePressure = true;
        newBreathData = false;
    }

    // $FAST: High-rate flow waveform for breath visualization (every flow read ~10Hz)
    Serial.printf("$FAST,%.3f,%.3f,%.2f,%d\r\n",
        (float)millis() / 1000.0f,   // Time (s)
        currentPressure,              // ΔP zeroed (Pa)
        currentO2,                    // FeO2 (%)
        integratePressure ? 1 : 0     // Exhale state (1=exhaling)
    );

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
            // Sanity check: O2 in air should be 10-25%
            if (val >= 10.0f && val <= 25.0f) {
                currentO2 = val;
                // Accumulate for time-based averaging
                o2Sum += val;
                o2SampleCount++;
            }
            // FiO2 is locked after warm-up. Do NOT auto-update.
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

        // Use time-based O2 average instead of per-breath average
        if (o2SampleCount > 0) {
            float avgO2 = o2Sum / (float)o2SampleCount;

            vo2 = calcApproxVO2(veMean, initialO2, avgO2);
            if (vo2 > vo2Max) vo2Max = vo2;

            // Calories: vo2 is already in L/min absolute
            caloriesPerMin = calcCaloriesPerMin(vo2);
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
            totalStoredSamples++;
            if (bufferPosition >= STORE_BUFFER_SIZE) {
                bufferPosition = 0;
                bufferWrapped = true;
            }
            storeIntervalTime = 0;
        }

        // Reset accumulators
        integrationTime = 0;
        integrationTotal = 0.0f;
        averagingCount = 0;
        averageO2 = 0.0f;
        averageCO2 = 0.0f;
        o2Sum = 0.0f;
        o2SampleCount = 0;

        // ---- Serial diagnostic output (every integration period) ----
        float vo2rel = (weight > 0) ? (1000.0f * vo2 / weight) : 0.0f;
        float respRate = (lastBreathInterval > 0) ? (60000.0f / (float)lastBreathInterval) : 0.0f;
        Serial.printf("$DIAG,%.1f,%.3f,%.2f,%.2f,%.1f,%.1f,%.2f,%.4f,%.1f,%.2f,%.1f,%.3f,%d\r\n",
            (float)millis() / 1000.0f,  // Time (s)
            currentPressure,             // ΔP instant (Pa)
            currentO2,                   // FeO2 (%)
            initialO2,                   // FiO2 (%)
            ambient_pressure,            // P_amb (hPa)
            ambient_temperature,         // T_amb (°C)
            veMean,                      // VE (L/min)
            vo2,                         // VO2 absolute (L/min)
            vo2rel,                      // VO2 relative (ml/min/kg)
            caloriesPerMin,              // Cal (kcal/min)
            respRate,                    // Resp rate (b/min)
            pressureZeroPa,              // D6F zero offset (Pa)
            errorCounter                 // Error count
        );

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
    hasPreviousBreath = false;
    breathIntervalCount = 0;
    lastBreathInterval = 0;
    previousFlowTimeUs = 0;
    lastBreathVolume = 0.0f;
    veMax = 0.0f;
    veMean = 0.0f;
    vo2 = 0.0f; vo2Max = 0.0f;
    vco2 = 0.0f; vco2Max = 0.0f;
    averageO2 = 0.0f; averageCO2 = 0.0f;
    averagingCount = 0;
    o2Sum = 0.0f;
    o2SampleCount = 0;
    caloriesPerMin = 0.0f;
    caloriesTotal = 0.0f;
    lastCalTime = 0;
    bufferPosition = 0;
    bufferWrapped = false;
    totalStoredSamples = 0;
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

    // ---- Zero calibration for D6F flow sensor ----
    if (xEventGroupGetBits(sensorStatus) & SENSOR_HAS_FLOW) {
        ESP_LOGI(TAG, "Flow sensor zero calibration (100 samples)...");
        float zeroSum = 0.0f;
        int zeroCount = 0;
        for (int i = 0; i < 100; i++) {
            float p = flowSensor.getPressure();
            if (!isnan(p)) {
                zeroSum += p;
                zeroCount++;
            }
            vTaskDelay(pdMS_TO_TICKS(40));
        }
        if (zeroCount > 0) {
            pressureZeroPa = zeroSum / (float)zeroCount;
            ESP_LOGI(TAG, "Flow zero offset: %.3f Pa (%d samples)", pressureZeroPa, zeroCount);
        }
    }

    // ---- O2 warm-up baseline (60s) to calculate accurate FiO2 ----
    if (xEventGroupGetBits(sensorStatus) & SENSOR_HAS_O2) {
        ESP_LOGI(TAG, "O2 sensor warm-up baseline (60s)...");
        float fiO2Sum = 0.0f;
        int fiO2Count = 0;
        for (int i = 0; i < 120; i++) {  // 120 × 500ms = 60s
            oxygenSensor.triggerSampling();
            vTaskDelay(pdMS_TO_TICKS(150));
            float val;
            if (oxygenSensor.readOxygenValue(val)) {
                if (val >= 18.0f && val <= 23.0f) {
                    fiO2Sum += val;
                    fiO2Count++;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(350));
            // Print progress every 10s
            if (i % 20 == 0) {
                ESP_LOGI(TAG, "O2 warm-up: %ds, samples=%d, avg=%.2f%%",
                    (i * 500) / 1000, fiO2Count,
                    fiO2Count > 0 ? fiO2Sum / fiO2Count : 0.0f);
            }
        }
        if (fiO2Count >= 10) {
            initialO2 = fiO2Sum / (float)fiO2Count;
            ESP_LOGI(TAG, "FiO2 locked at %.2f%% (%d samples)", initialO2, fiO2Count);
        } else {
            ESP_LOGW(TAG, "O2 warm-up insufficient samples (%d), using default FiO2=%.2f%%", fiO2Count, initialO2);
        }
    }
    fiO2Locked = true;

    // ---- Initialize BMP280 from first real reading ----
    if (xEventGroupGetBits(sensorStatus) & SENSOR_HAS_PRESSURE) {
        float t = bmp.readTemperature();
        float p = bmp.readPressure() / 100.0f;
        if (!isnan(t) && !isnan(p) && p > 800.0f && p < 1200.0f) {
            ambient_temperature = t;
            ambient_pressure = p;
            ESP_LOGI(TAG, "BMP280 initial: T=%.1f°C, P=%.1f hPa", t, p);
        }
    }

    xEventGroupSetBits(sensorStatus, SENSOR_INIT_DONE);


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

uint16_t sensorGetStatus() {
    if (!sensorStatus) return 0;  // Safe: may be called before task creates event group
    return xEventGroupGetBits(sensorStatus);
}
void sensorSetInitial(void) { xEventGroupSetBits(sensorEvent, SENSOR_EVENT_INIT); }
void sensorResetCalculation(void) { xEventGroupSetBits(sensorEvent, SENSOR_EVENT_RESET); }
void sensorO2Calibrate(void) { xEventGroupSetBits(sensorEvent, SENSOR_EVENT_O2_CAL); }
void sensorSetConfiguration(void) { xEventGroupSetBits(sensorEvent, SENSOR_EVENT_RECONF); }
bool sensorHasErrors(void) { return errorCounter > 0; }
const storeData_t *getStorageBuffer(void) { return bufferData; }
unsigned int getStorageBufferPosition(void) { return bufferPosition; }
