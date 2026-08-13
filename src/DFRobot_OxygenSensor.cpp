/*
 * DFRobot_OxygenSensor.cpp - DFRobot SEN0322 Gravity I2C Oxygen Sensor
 *
 * Original: MIT License, Copyright (C) 2019 @DFRobot ZhiXinLiu
 * Modified for VO2Max-TEEP: trigger/read separation, improved error handling
 */

#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_OxygenSensor.h"

DFRobot_OxygenSensor::DFRobot_OxygenSensor() : _addr(0), _key(0) {
    memset(_oxygenData, 0, sizeof(_oxygenData));
}

DFRobot_OxygenSensor::~DFRobot_OxygenSensor() {
}

bool DFRobot_OxygenSensor::begin(uint8_t addr) {
    _addr = addr;
    Wire.beginTransmission(_addr);
    if (Wire.endTransmission() == 0) {
        readFlash();  // Read calibration key
        return true;
    }
    return false;
}

void DFRobot_OxygenSensor::readFlash() {
    uint8_t value = 0;
    Wire.beginTransmission(_addr);
    Wire.write(O2_GET_KEY_REGISTER);
    Wire.endTransmission();
    delay(50);
    Wire.requestFrom(_addr, (uint8_t)1);
    while (Wire.available()) {
        value = Wire.read();
    }
    if (value == 0) {
        _key = 20.9 / 120.0;  // Default calibration
    } else {
        _key = (float)value / 1000.0;
    }
}

void DFRobot_OxygenSensor::i2cWrite(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void DFRobot_OxygenSensor::calibrate(float vol, float mv) {
    uint8_t keyValue = vol * 10;
    if (mv < 0.000001 && mv > (-0.000001)) {
        i2cWrite(O2_USER_SET_REGISTER, keyValue);
    } else {
        keyValue = (vol / mv) * 1000;
        i2cWrite(O2_AUTUAL_SET_REGISTER, keyValue);
    }
    delay(100);
    readFlash();  // Re-read calibration after setting
}

/**
 * Trigger a new sampling. Call readOxygenValue() ~100ms later.
 * Non-blocking step 1 of the two-step read process.
 */
void DFRobot_OxygenSensor::triggerSampling() {
    Wire.beginTransmission(_addr);
    Wire.write(O2_OXYGEN_DATA_REGISTER);
    Wire.endTransmission();
}

/**
 * Read oxygen value after trigger. Non-blocking step 2.
 * Must be called ~100ms after triggerSampling().
 */
bool DFRobot_OxygenSensor::readOxygenValue(float &result) {
    uint8_t rxbuf[3] = {0};
    uint8_t k = 0;

    Wire.requestFrom(_addr, (uint8_t)3);
    while (Wire.available() && k < 3) {
        rxbuf[k++] = Wire.read();
    }

    if (k < 3) return false;

    result = _key * (((float)rxbuf[0]) + ((float)rxbuf[1] / 10.0) + ((float)rxbuf[2] / 100.0));
    return true;
}

/**
 * Read oxygen concentration with internal averaging.
 * This is a blocking call that takes ~100ms per sample.
 */
float DFRobot_OxygenSensor::readOxygenData(uint8_t collectNum) {
    static uint8_t i = 0;

    readFlash();

    if (collectNum > O2_MAX_COLLECT_NUMBER) collectNum = O2_MAX_COLLECT_NUMBER;
    if (collectNum <= 0) return -1.0;

    // Shift old data
    for (int j = collectNum - 1; j > 0; j--) {
        _oxygenData[j] = _oxygenData[j - 1];
    }

    // Read new sample
    Wire.beginTransmission(_addr);
    Wire.write(O2_OXYGEN_DATA_REGISTER);
    Wire.endTransmission();
    delay(100);

    uint8_t rxbuf[3] = {0};
    uint8_t k = 0;
    Wire.requestFrom(_addr, (uint8_t)3);
    while (Wire.available() && k < 3) {
        rxbuf[k++] = Wire.read();
    }

    _oxygenData[0] = _key * (((float)rxbuf[0]) + ((float)rxbuf[1] / 10.0) + ((float)rxbuf[2] / 100.0));

    if (i < collectNum) i++;
    return getAverageNum(_oxygenData, i);
}

float DFRobot_OxygenSensor::getAverageNum(float bArray[], uint8_t iFilterLen) {
    double bTemp = 0;
    for (uint8_t i = 0; i < iFilterLen; i++) {
        bTemp += bArray[i];
    }
    return bTemp / (float)iFilterLen;
}
