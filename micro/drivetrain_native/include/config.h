#ifndef _CONFIG_H_
#define _CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <stdbool.h>


// Frequency of control loops
static const uint FREQ = 200;

// I2C config
#define BUFF_SIZE 128

static const uint I2C_SLAVE_ADDRESS = 0x41;
static const uint I2C_BAUDRATE = 400000; // 400 kHz

static const uint I2C_SLAVE_SDA_PIN = 4;
static const uint I2C_SLAVE_SCL_PIN = 5;

// IMU config
static const uint8_t IMU_SDA_PIN = 26;
static const uint8_t IMU_SCL_PIN = 27;
static i2c_inst_t *IMU_I2C_INST = i2c1;

// 
static const float MAX_VXY = 2.0f;
static const float MAX_OMEGA = 8.0f;

// MOTOR PIN CONFIG
// These should be constant and not configurable from host
// Front left encoder A channel pin
static const int FL_PIN_A = 15;
// Front left encoder B channel pin
static const int FL_PIN_B = 14;
// Front left motor forward pin
static const int FL_PIN_F = 6;
// Front left motor reverse pin
static const int FL_PIN_R = 7;
// Front left motor inverted
static const bool FL_INVERTED = false;

// Front right encoder A channel pin
static const int FR_PIN_A = 19;
// Front right encoder B channel pin
static const int FR_PIN_B = 18;
// Front right motor forward pin
static const int FR_PIN_F = 10;
// Front right motor reverse pin
static const int FR_PIN_R = 11;
// Front right motor reverse pin
static const bool FR_INVERTED = true;

// Back left encoder A channel pin
static const int BL_PIN_A = 17;
// Back left encoder B channel pin
static const int BL_PIN_B = 16;
// Back left motor forward pin
static const int BL_PIN_F = 9;
// Back left motor reverse pin
static const int BL_PIN_R = 8;
// Back left motor inverted
static const bool BL_INVERTED = false;

// Back right encoder A channel pin
static const int BR_PIN_A = 21;
// Back right encoder B channel pin
static const int BR_PIN_B = 20;
// Back right motor forward pin
static const int BR_PIN_F = 13;
// Back right motor reverse pin
static const int BR_PIN_R = 12;
// Back right motor inverted
static const bool BR_INVERTED = true;

// KINEMATICS CONFIG
// Radius of the wheel in meters
static const float WHEEL_RADIUS = 0.03f;
// Distance from center to wheel in x direction in meters
static const float WHEEL_DIST_X = 0.06f;
// Distance from center to wheel in y direction in meters
static const float WHEEL_DIST_Y = 0.1075f;

#ifdef __cplusplus
}
#endif
#endif // _CONFIG_H_