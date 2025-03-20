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
#define BUFF_SIZE 256

// Address of this pico over I2C
static const uint I2C_SLAVE_ADDRESS = 0x24;
// Baud rate of I2C in hertz
static const uint I2C_BAUDRATE = 400*1000;

// GPIO number of SDA pin for slave I2C
static const uint I2C_SLAVE_SDA_PIN = 20;
// GPIO number of SCL pin for slave I2C
static const uint I2C_SLAVE_SCL_PIN = 21;

// Maximum speed possible for usage in I2C communication
static const float MAX_V = 8.0f;

// MOTOR PIN CONFIG
// These should be constant and not configurable from host
// Front left encoder A channel pin
static const int INTAKE_PIN_A = 14;
// Front left encoder B channel pin
static const int INTAKE_PIN_B = 15;
// Front left motor pwm pin
static const int INTAKE_PIN_F = 2;
// Front left motor reverse pin
static const int INTAKE_PIN_R = 3;
// Front left motor inverted
static const bool INTAKE_INVERTED = false;


// Beacon PWM pin
static const int BEACON_PIN_PWM = 4;

// Box mover PWM pin
static const int BOX_MOVER_PIN_PWM = 5;

// Dumper Stepper DIR pin
static const int DUMPER_PIN_DIR = 6;
// Dumper Stepper STEP pin
static const int DUMPER_PIN_STEP = 7;
// Dumper stepper EN pin
static const int DUMPER_PIN_EN = 8;

// Start light sense pin
static const int START_PIN_SENSE = 28;
// Arm switch pin
static const int PIN_ARMED = 27;


// DUMPER CONFIG
// Acceleration in steps per second per second 
static const int DUMPER_ACCEL = 8000;
// Max steps per second
static const int DUMPER_MAX_VEL = 6000;

// Target steps for dumping
static const int DUMPER_TARGET_POS = -3000;
// Shake steps
static const int DUMPER_SHAKE_STEPS = 15;


#ifdef __cplusplus
}
#endif
#endif // _CONFIG_H_