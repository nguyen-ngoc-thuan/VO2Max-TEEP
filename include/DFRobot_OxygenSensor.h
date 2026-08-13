/*
 * DFRobot_OxygenSensor.h - Driver for DFRobot SEN0322 Gravity I2C Oxygen Sensor
 *
 * Original: MIT License, Copyright (C) 2019 @DFRobot ZhiXinLiu
 * Modified for VO2Max-TEEP: trigger/read separation for non-blocking operation
 */

#ifndef DFROBOT_OXYGENSENSOR_H
#define DFROBOT_OXYGENSENSOR_H

#include <Arduino.h>
#include <Wire.h>

// I2C addresses (selected by address pins on sensor board)
#define O2_ADDRESS_0  0x70
#define O2_ADDRESS_1  0x71
#define O2_ADDRESS_2  0x72
#define O2_ADDRESS_3  0x73  // Default (both pins low)

// Register definitions
#define O2_OXYGEN_DATA_REGISTER  0x03
#define O2_USER_SET_REGISTER     0x08
#define O2_AUTUAL_SET_REGISTER   0x09
#define O2_GET_KEY_REGISTER      0x0A

// Maximum samples for averaging
#define O2_MAX_COLLECT_NUMBER    100

class DFRobot_OxygenSensor {
public:
    DFRobot_OxygenSensor();
    ~DFRobot_OxygenSensor();

    /**
     * @brief Initialize the sensor
     * @param addr I2C address (default 0x73)
     * @return true if sensor is found on bus
     */
    bool begin(uint8_t addr = O2_ADDRESS_3);

    /**
     * @brief Read oxygen concentration with averaging
     * @param collectNum Number of samples to average (1-100)
     * @return Oxygen concentration in % (e.g., 20.9)
     */
    float readOxygenData(uint8_t collectNum);

    /**
     * @brief Trigger a new sampling (non-blocking step 1)
     *        Call readOxygenValue() ~100ms later
     */
    void triggerSampling();

    /**
     * @brief Read the oxygen value after trigger (non-blocking step 2)
     * @param result Reference to store the result in %
     * @return true if read succeeded
     */
    bool readOxygenValue(float &result);

    /**
     * @brief Calibrate the sensor
     * @param vol Known O2 volume % (20.9 for air calibration)
     * @param mv  Set to 0 for air calibration
     */
    void calibrate(float vol, float mv);

private:
    void readFlash();
    void i2cWrite(uint8_t reg, uint8_t data);
    float getAverageNum(float bArray[], uint8_t iFilterLen);

    uint8_t _addr;
    float _key;
    float _oxygenData[O2_MAX_COLLECT_NUMBER];
};

#endif // DFROBOT_OXYGENSENSOR_H
