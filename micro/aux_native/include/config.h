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
static const float MAX_V = 20.0f;

// MOTOR PIN CONFIG
// These should be constant and not configurable from host
// Front left encoder A channel pin
static const int INTAKE_PIN_A = 15;
// Front left encoder B channel pin
static const int INTAKE_PIN_B = 14;
// Front left motor pwm pin
static const int INTAKE_PIN_F = 3;
// Front left motor reverse pin
static const int INTAKE_PIN_R = 2;
// Front left motor inverted
static const bool INTAKE_INVERTED = false;


// Beacon PWM pin
static const int BEACON_PIN_PWM = 5;

// Box mover PWM pin
static const int BOX_MOVER_PIN_PWM = 4;

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
// Buzzer Pin
static const int BUZZER_PIN = 10;


// DUMPER CONFIG
// Acceleration in steps per second per second 
static const int DUMPER_ACCEL = 20000;
// Max steps per second
static const int DUMPER_MAX_VEL = 20000;

// Target steps for dumping
static const int DUMPER_TARGET_POS = -3800;
// Shake steps
static const int DUMPER_SHAKE_STEPS = 15;

// BOX MOVER CONFIG
static const int BOX_MOVER_OFF_POS = (int)(0);
static const int BOX_MOVER_IDLE_POS = (int)(1.8 / 20.0 * 65536);
static const int BOX_MOVER_GRIP_POS = (int)(1.95 / 20.0 * 65536);
static const int BOX_MOVER_OPEN_POS = (int)(2.7 / 20.0 * 65536);

// BEACON MOVER CONFIG
static const int BEACON_STOWED_POS = (int)(2.2 / 20.0 * 65536);
static const int BEACON_TRAVEL_POS = (int)(1.8 / 20.0 * 65536);
static const int BEACON_DEPLOY_POS = (int)(1.3 / 20.0 * 65536);

// START LED CONFIG
static const int START_THRESH = 2000;
// Minimum amount of samples (2 ms apart) to signal that start LED is enabled
static const int N_MEASURES = 500;

#ifdef __cplusplus
}
#endif
#endif // _CONFIG_H_