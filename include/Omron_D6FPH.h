/*
 * Omron_D6FPH.h - Driver for Omron D6F-PH differential pressure sensors
 *
 * Supported models:
 *   D6F-PH0025AD1 / D6F-PH0025AMD2  (0-25 Pa)
 *   D6F-PH0505AD3                     (±50 Pa)
 *   D6F-PH5050AD3                     (±500 Pa)
 *
 * Original library under LGPL v2.1
 * Modified for VO2Max-TEEP project with improved error handling
 */

#ifndef OMRON_D6FPH_H
#define OMRON_D6FPH_H

#include <Arduino.h>
#include <Wire.h>

// Default I2C address
#define D6FPH_ADDRESS     0x6C

// Register definitions
#define D6F_SENS_CTRL     0xD040
#define D6F_COMP_DATA1_H  0xD051
#define D6F_TMP_H         0xD061
#define D6F_BUFFER_0      0x07
#define D6F_CTRL_REG      0x0B
#define D6F_START_ADDRESS  0x00

// Control register values
#define D6F_SENS_CTRL_MS        2
#define D6F_SENS_CTRL_DV_PWR    1
#define D6F_SENS_CTRL_VAL       ((0x01 << D6F_SENS_CTRL_MS) | (0x01 << D6F_SENS_CTRL_DV_PWR))

#define D6F_SERIAL_CTRL_D_BYTE_CNT3  5
#define D6F_SERIAL_CTRL_REQ          3
#define D6F_SERIAL_CTRL_R_WZ         2
#define D6F_SERIAL_CTRL_VAL          ((0x01 << D6F_SERIAL_CTRL_R_WZ) | (0x01 << D6F_SERIAL_CTRL_REQ) | (0x01 << D6F_SERIAL_CTRL_D_BYTE_CNT3))

// I2C error compatibility
// Wire.endTransmission() returns 0 on success on all platforms
#ifndef I2C_ERROR_OK
#define I2C_ERROR_OK 0
#endif

// Sensor model enumeration
enum sensorModels {
    MODEL_0025AD1 = 0,   // 0-25 Pa (D6F-PH0025AD1 / AMD2)
    MODEL_0505AD3,       // ±50 Pa
    MODEL_5050AD3        // ±500 Pa
};

class Omron_D6FPH {
public:
    Omron_D6FPH(void);

    // Initialize with default Wire, address, and model
    boolean begin(sensorModels sensorModel);
    // Initialize with custom Wire port, address, and model
    boolean begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = D6FPH_ADDRESS,
                  sensorModels sensorModel = MODEL_0025AD1);

    void setSensorModel(sensorModels sensorModel = MODEL_0025AD1);
    boolean isConnected();

    // Read differential pressure in Pa
    float getPressure();

    // Read temperature in °C
    float getTemperature();

    // Get last error status
    boolean hasError() const { return _lastError; }

private:
    sensorModels _sensorModel;
    boolean init();
    boolean readRegister(uint8_t reg, uint16_t *value);
    boolean executeMcuMode();

    TwoWire *_i2cPort;
    uint8_t _i2cAddress;
    uint16_t _rangeMode;
    uint16_t _rangeModeSubVal;
    uint8_t _rangeModeMulVal;
    boolean _lastError;
};

#endif // OMRON_D6FPH_H
