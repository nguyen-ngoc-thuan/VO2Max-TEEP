/*
 * sensorTask.h - Sensor task interface and data structures
 *
 * Copyright (C) 2025 TEEP Project
 * GPL V3
 */

#ifndef __SENSORTASK_H__
#define __SENSORTASK_H__

#include <freertos/FreeRTOS.h>

// ---- Sensor Status Bits ----
#define SENSOR_HAS_FLOW     0x0001
#define SENSOR_HAS_O2       0x0002
#define SENSOR_HAS_CO2      0x0004
#define SENSOR_HAS_PRESSURE 0x0008
#define SENSOR_INIT_DONE    0x0080

// ---- Sensor Event Bits ----
#define SENSOR_EVENT_FLOW   0x0001   // Triggered on every flow measurement
#define SENSOR_EVENT_CALC   0x0002   // Triggered on every breath calculation
#define SENSOR_EVENT_AVE    0x0004   // Triggered when integration period complete
#define SENSOR_EVENT_VOL    0x0008   // Triggered after volume calc step
#define SENSOR_EVENT_INIT   0x1000   // Set initial O2 level
#define SENSOR_EVENT_RESET  0x2000   // Reset calculations
#define SENSOR_EVENT_O2_CAL 0x4000   // Calibrate O2 sensor
#define SENSOR_EVENT_RECONF 0x8000   // Reload config from global settings

// ---- Extra Sensor Types ----
#define SENSOR_EXTRA_HR     0x01
#define SENSOR_EXTRA_RR     0x02

// Extra data from external sensors (BLE HR etc.)
typedef struct _sensorExtraData_t {
    uint8_t type;
    float value;
} sensorExtraData_t;

// Main sensor data structure (shared between tasks)
typedef struct _sensorData_t {
    // Calculated values
    float vo2;                // VO2 [ml/min/kg]
    float vo2Max;             // Maximum VO2 [ml/min/kg]
    float ve;                 // Last breath volume [L]
    float veMax;              // Maximum single breath volume [L]
    float veMean;             // Minute ventilation [L/min]
    float vco2;               // VCO2 [ml/min/kg]
    float vco2Max;            // Maximum VCO2 [ml/min/kg]
    float rq;                 // Respiratory quotient (VCO2/VO2)
    float resp_rate;          // Respiratory rate [breaths/min]
    float calories_min;       // Calories per minute [kcal/min]
    float calories_total;     // Total calories [kcal]

    // Raw sensor values
    float ambient_pressure;   // [hPa]
    float ambient_temperature;// [°C]
    float exhale_temperature; // [°C] from pressure sensor
    float o2;                 // O2 sensor reading [%]
    float co2;                // CO2 sensor reading [%]
    float flow_value;         // Instant flow sensor value [Pa]

    // External sensors
    float hr;                 // Heart rate [bpm]
    float hr_max;             // Maximum heart rate [bpm]
    float rr;                 // HR R-R interval [ms]

    // Debug
    uint16_t errors;          // Error counter
} sensorData_t;

// Structure for buffered data storage
typedef struct _storeData_t {
    float vo2;          // ml/min/kg
    float ve;           // L/min
    float vco2;         // ml/min/kg
    float resp_rate;    // breaths/min
    float hr;           // bpm
    float temperature;  // °C
    float pressure;     // hPa
    float o2;           // %
} storeData_t;

// ---- Public Functions ----
void sensorTask(void *params);
uint16_t sensorGetStatus(void);

bool sensorGetData(sensorData_t *copy_to);

void sensorClearEvent(uint32_t event);
bool sensorWaitEvent(uint32_t event, TickType_t timeout);

bool sensorAddExtraValue(uint8_t type, float value, TickType_t timeout);

void sensorSetInitial(void);
void sensorResetCalculation(void);
void sensorO2Calibrate(void);
void sensorSetConfiguration(void);

bool sensorHasErrors(void);

const storeData_t *getStorageBuffer(void);
unsigned int getStorageBufferPosition(void);

#endif // __SENSORTASK_H__
