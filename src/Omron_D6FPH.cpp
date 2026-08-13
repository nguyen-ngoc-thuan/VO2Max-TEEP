/*
 * Omron_D6FPH.cpp - Driver for Omron D6F-PH differential pressure sensors
 *
 * Original library under LGPL v2.1
 * Modified for VO2Max-TEEP with improved error handling and retry logic
 */

#include "Omron_D6FPH.h"

Omron_D6FPH::Omron_D6FPH(void) {
    _lastError = false;
}

boolean Omron_D6FPH::begin(sensorModels sensorModel) {
    return begin(Wire, D6FPH_ADDRESS, sensorModel);
}

boolean Omron_D6FPH::begin(TwoWire &wirePort, uint8_t deviceAddress, sensorModels sensorModel) {
    setSensorModel(sensorModel);
    _i2cPort = &wirePort;
    _i2cAddress = deviceAddress;
    _lastError = false;

    if (isConnected()) {
        return init();
    }
    return false;
}

void Omron_D6FPH::setSensorModel(sensorModels sensorModel) {
    _sensorModel = sensorModel;
    switch (_sensorModel) {
    case MODEL_0025AD1:
        _rangeMode = 250;        // Range scaling factor
        _rangeModeMulVal = 1;
        _rangeModeSubVal = 0;
        break;
    case MODEL_0505AD3:
        _rangeMode = 50;
        _rangeModeMulVal = 2;
        _rangeModeSubVal = 50;
        break;
    default: // MODEL_5050AD3
        _rangeMode = 500;
        _rangeModeMulVal = 2;
        _rangeModeSubVal = 500;
        break;
    }
}

/**
 * Initialization after power up.
 * Write 00h to the Control Register (0Bh) to load NVM trim values
 * while keeping MCU in non-reset state.
 */
boolean Omron_D6FPH::init() {
    _i2cPort->beginTransmission(_i2cAddress);
    _i2cPort->write(D6F_CTRL_REG);
    _i2cPort->write(0x00);
    _lastError = (_i2cPort->endTransmission() != I2C_ERROR_OK);
    return !_lastError;
}

boolean Omron_D6FPH::isConnected() {
    _i2cPort->beginTransmission(_i2cAddress);
    return _i2cPort->endTransmission() == I2C_ERROR_OK;
}

/**
 * Execute MCU mode after desired configurations are set.
 * Write 06h (MS=1 & MCU_on) to the SENS_CTRL Register (D040h).
 */
boolean Omron_D6FPH::executeMcuMode() {
    _i2cPort->beginTransmission(_i2cAddress);
    _i2cPort->write(D6F_START_ADDRESS);
    _i2cPort->write(highByte(D6F_SENS_CTRL));
    _i2cPort->write(lowByte(D6F_SENS_CTRL));
    _i2cPort->write(0x18);
    _i2cPort->write(D6F_SENS_CTRL_VAL);
    _lastError = (_i2cPort->endTransmission() != I2C_ERROR_OK);
    return !_lastError;
}

/**
 * Read differential pressure in Pa.
 * Note: This function has a 33ms internal delay for MCU processing.
 */
float Omron_D6FPH::getPressure() {
    if (executeMcuMode()) {
        delay(33);  // Wait for MCU to process
        _i2cPort->beginTransmission(_i2cAddress);
        _i2cPort->write(D6F_START_ADDRESS);
        _i2cPort->write(highByte(D6F_COMP_DATA1_H));
        _i2cPort->write(lowByte(D6F_COMP_DATA1_H));
        _i2cPort->write(D6F_SERIAL_CTRL_VAL);

        if (_i2cPort->endTransmission() == I2C_ERROR_OK) {
            uint16_t value;
            if (readRegister(D6F_BUFFER_0, &value)) {
                _lastError = false;
                return (float)((value - 1024.00) * _rangeMode * _rangeModeMulVal / 60000L) - _rangeModeSubVal;
            }
        }
    }
    _lastError = true;
    return NAN;
}

/**
 * Read temperature in °C.
 * Note: This function has a 33ms internal delay.
 */
float Omron_D6FPH::getTemperature() {
    if (executeMcuMode()) {
        delay(33);
        _i2cPort->beginTransmission(_i2cAddress);
        _i2cPort->write(D6F_START_ADDRESS);
        _i2cPort->write(highByte(D6F_TMP_H));
        _i2cPort->write(lowByte(D6F_TMP_H));
        _i2cPort->write(D6F_SERIAL_CTRL_VAL);

        if (_i2cPort->endTransmission() == I2C_ERROR_OK) {
            uint16_t value;
            if (readRegister(D6F_BUFFER_0, &value)) {
                _lastError = false;
                int temp = round((float)(value - 10214) / 3.739);
                return (temp / 10.0);
            }
        }
    }
    _lastError = true;
    return NAN;
}

boolean Omron_D6FPH::readRegister(uint8_t reg, uint16_t *value) {
    _i2cPort->beginTransmission(_i2cAddress);
    _i2cPort->write(reg);
    if (_i2cPort->endTransmission() == I2C_ERROR_OK) {
        _i2cPort->requestFrom(_i2cAddress, (uint8_t)2);
        if (_i2cPort->available() >= 2) {
            *value = ((_i2cPort->read() << 8) | _i2cPort->read());
            return true;
        }
    }
    return false;
}
