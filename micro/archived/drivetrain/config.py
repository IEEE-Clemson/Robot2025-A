"""Constants for drivetrain
"""

FREQ = 200
"Frequency of control loops"

# MOTOR PIN CONFIG
# These should be constant and not configurable from host
# Motor 1:
FL_PIN_A = 10
"Front left encoder A channel pin"
FL_PIN_B = 11
"Front left encoder B channel pin"
FL_PWM_PIN = 6
"Front left motor PWM pin"
FL_DIR_PIN = 2
"Front left motor direction pin"
FL_INVERTED = False
"Front left motor inverted"

# Motor 2:
BL_PIN_A = 12
"Back left encoder A channel pin"
BL_PIN_B = 13
"Back left encoder B channel pin"
BL_PWM_PIN = 7
"Back left motor PWM pin"
BL_DIR_PIN = 3
"Back left motor direction pin"
BL_INVERTED = False
"Back left motor inverted"

# Motor 3:
FR_PIN_A = 14
"Front right encoder A channel pin"
FR_PIN_B = 15
"Front right encoder B channel pin"
FR_PWM_PIN = 8
"Front right motor PWM pin"
FR_DIR_PIN = 4
"Front right motor direction pin"
FR_INVERTED = True
"Front right motor inverted"

# Motor 4:
BR_PIN_A = 16 #18
"Back right encoder A channel pin"
BR_PIN_B = 17 #19
"Back right encoder B channel pin"
BR_PWM_PIN = 9 #10
"Back right motor PWM pin"
BR_DIR_PIN = 5 #11
"Back right motor direction pin"
BR_INVERTED = True
"Back right motor inverted"

# KINEMATICS CONFIG
WHEEL_RADIUS = 0.03
"Radius of the wheel in meters"
WHEEL_DIST_X = 0.06
"Distance from center to wheel in x direction in meters"
WHEEL_DIST_Y = 0.1075
"Distance from center to wheel in y direction in meters"
