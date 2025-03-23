#ifndef _CONFIG_H_zA
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

// Address of this pico over I2C
static const uint I2C_SLAVE_ADDRESS = 0x41;
// Baud rate of I2C in hertz
static const uint I2C_BAUDRATE = 400*1000;

// GPIO number of SDA pin for slave I2C
static const uint I2C_SLAVE_SDA_PIN = 20;
// GPIO number of SCL pin for slave I2C
static const uint I2C_SLAVE_SCL_PIN = 21;

// IMU config
// GPIO number of I2C SDA pin for use with IMU
static const uint8_t IMU_SDA_PIN = 26;
// GPIO number of I2C SCL pin for use with IMU
static const uint8_t IMU_SCL_PIN = 27;
// I2C instance to use with IMU, must be different from slave I2C
static i2c_inst_t *IMU_I2C_INST = i2c1;

// Maximum speed possible for usage in I2C communication
static const float MAX_VXY = 2.0f;
// Maximum angular velocity possible for usage in I2C communication
static const float MAX_OMEGA = 8.0f;

// MOTOR PIN CONFIG
// These should be constant and not configurable from host
// Front left encoder A channel pin
static const int FL_PIN_A = 11;
// Front left encoder B channel pin
static const int FL_PIN_B = 10;
// Front left motor pwm pin
static const int FL_PIN_PWM = 6;
// Front left motor reverse pin
static const int FL_PIN_DIR = 2;
// Front left motor inverted
static const bool FL_INVERTED = false;

// Front right encoder A channel pin
static const int FR_PIN_A = 15;
// Front right encoder B channel pin
static const int FR_PIN_B = 14;
// Front right motor pwm pin
static const int FR_PIN_PWM = 8;
// Front right motor reverse pin
static const int FR_PIN_DIR = 4;
// Front right motor reverse pin
static const bool FR_INVERTED = true;

// Back left encoder A channel pin
static const int BL_PIN_A = 13;
// Back left encoder B channel pin
static const int BL_PIN_B = 12;
// Back left motor pwm pin
static const int BL_PIN_PWM = 7;
// Back left motor reverse pin
static const int BL_PIN_DIR = 3;
// Back left motor inverted
static const bool BL_INVERTED = false;

// Back right encoder A channel pin
static const int BR_PIN_A = 17;
// Back right encoder B channel pin
static const int BR_PIN_B = 16;
// Back right motor pwm pin
static const int BR_PIN_PWM = 9;
// Back right motor reverse pin
static const int BR_PIN_DIR = 5;
// Back right motor inverted
static const bool BR_INVERTED = true;

// KINEMATICS CONFIG
// Radius of the wheel in meters
static const float WHEEL_RADIUS = 0.03f;
// Distance from center to wheel in x direction in meters
static const float WHEEL_DIST_X = 0.07f;
// Distance from center to wheel in y direction in meters
static const float WHEEL_DIST_Y = 0.1075f;

#ifdef __cplusplus
}
#endif
#endif // _CONFIG_H_