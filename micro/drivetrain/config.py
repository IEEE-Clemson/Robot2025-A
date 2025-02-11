"""Constants for drivetrain
"""

FREQ = 200
"Frequency of control loops"

# MOTOR PIN CONFIG
# These should be constant and not configurable from host
FL_PIN_A = 15
"Front left encoder A channel pin"
FL_PIN_B = 14
"Front left encoder B channel pin"
FL_PIN_F = 6
"Front left motor forward pin"
FL_PIN_R = 7
"Front left motor reverse pin"
FL_INVERTED = False
"Front left motor inverted"

FR_PIN_A = 19
"Front right encoder A channel pin"
FR_PIN_B = 18
"Front right encoder B channel pin"
FR_PIN_F = 10
"Front right motor forward pin"
FR_PIN_R = 11
"Front right motor reverse pin"
FR_INVERTED = True
"Front right motor inverted"

BL_PIN_A = 17
"Back left encoder A channel pin"
BL_PIN_B = 16
"Back left encoder B channel pin"
BL_PIN_F = 9
"Back left motor forward pin"
BL_PIN_R = 8
"Back left motor reverse pin"
BL_INVERTED = False
"Back left motor inverted"

BR_PIN_A = 21 #18
"Back right encoder A channel pin"
BR_PIN_B = 20 #19
"Back right encoder B channel pin"
BR_PIN_F = 13 #10
"Back right motor forward pin"
BR_PIN_R = 12 #11
"Back right motor reverse pin"
BR_INVERTED = True
"Back right motor inverted"

# KINEMATICS CONFIG
WHEEL_RADIUS = 0.03
"Radius of the wheel in meters"
WHEEL_DIST_X = 0.052
"Distance from center to wheel in x direction in meters"
WHEEL_DIST_Y = 0.1175
"Distance from center to wheel in y direction in meters"