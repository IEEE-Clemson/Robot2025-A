# -------------------------------------------------
# REGISTERS
# -------------------------------------------------

# I2C
I2C_BUS         = 1
I2C_PRIM_ADDR   = 0x68
I2C_SEC_ADDR    = 0x69

# General
CHIP_ID_ADDRESS = 0x00
SENSORTIME_0    = 0x18
SENSORTIME_1    = 0x19
SENSORTIME_2    = 0x1A
INTERNAL_STATUS = 0x21
DATA_REG        = 0x0C
FIFO_CONFIG_0   = 0x48
FIFO_CONFIG_1   = 0x49
INIT_CTRL       = 0x59
INIT_ADDR_0     = 0x5B
INIT_ADDR_1     = 0x5C
INIT_DATA       = 0x5E
CMD             = 0x7E
PWR_CONF        = 0x7C
PWR_CTRL        = 0x7D

# Accelerometer
ACC_CONF        = 0x40
ACC_RANGE       = 0x41
ACC_X_7_0       = 0x0C
ACC_X_15_8      = 0x0D
ACC_Y_7_0       = 0x0E
ACC_Y_15_8      = 0x0F
ACC_Z_7_0       = 0x10
ACC_Z_15_8      = 0x11

# Gyroscope
GYR_CONF        = 0x42
GYR_RANGE       = 0x43
GYR_X_7_0       = 0x12
GYR_X_15_8      = 0x13
GYR_Y_7_0       = 0x14
GYR_Y_15_8      = 0x15
GYR_Z_7_0       = 0x16
GYR_Z_15_8      = 0x17

# Temperature
TEMP_7_0        = 0x22
TEMP_15_8       = 0x23






# -------------------------------------------------
# DEFINITIONS
# -------------------------------------------------

# General
GRAVITY         = 9.81288
DEG2RAD         = 3.141592653589793 / 180.0
HERTZ_100       = 0.01
HERTZ_200       = 0.005
BINARY          = 'bin'
HEXADECIMAL     = 'hex'
BIT_0           = 0b00000001
BIT_1           = 0b00000010
BIT_2           = 0b00000100
BIT_3           = 0b00001000
BIT_4           = 0b00010000
BIT_5           = 0b00100000
BIT_6           = 0b01000000
BIT_7           = 0b10000000
LSB_MASK_8BIT   = 0x0F      # 00001111
MSB_MASK_8BIT   = 0xF0      # 11110000
FULL_MASK_8BIT  = 0xFF      # 11111111
LSB_MASK_8BIT_5 = 0x1F      # 00011111
LSB_MASK_8BIT_8 = 0x8F      # 10001111
LAST_2_BITS     = 0xC0      # 11000000
LAST_3_BITS     = 0xE0      # 11100000
FIRST_3_BITS    = 0x07      # 00000111
FIRST_2_BITS    = 0x04      # 00000011


# Device Modes
LOW_POWER_MODE = 'low_power'
NORMAL_MODE = 'normal'
PERFORMANCE_MODE = 'performance'

# Accelerometer
ACC_RANGE_2G    = 0x00      # +/- 2g
ACC_RANGE_4G    = 0x01      # +/- 4g
ACC_RANGE_8G    = 0x02      # +/- 8g
ACC_RANGE_16G   = 0x03      # +/- 16g
ACC_ODR_1600    = 0x0C      # 1600Hz
ACC_ODR_800     = 0x0B      # 800Hz
ACC_ODR_400     = 0x0A      # 400Hz
ACC_ODR_200     = 0x09      # 200Hz
ACC_ODR_100     = 0x08      # 100Hz
ACC_ODR_50      = 0x07      # 50Hz
ACC_ODR_25      = 0x06      # 25Hz
ACC_ODR_12P5    = 0X05      # 12.5Hz
ACC_ODR_6P25    = 0x04      # 6.25Hz 
ACC_ODR_3P1     = 0X03      # 3.1Hz
ACC_ODR_1P5     = 0X02      # 1.5Hz
ACC_ODR_0P78    = 0X01      # 0.78Hz
ACC_BWP_OSR4    = 0x00      # OSR4
ACC_BWP_OSR2    = 0x01      # OSR2
ACC_BWP_NORMAL  = 0x02      # Normal
ACC_BWP_CIC     = 0x03      # CIC
ACC_BWP_RES16   = 0x04      # Reserved
ACC_BWP_RES32   = 0x05      # Reserved
ACC_BWP_RES64   = 0x06      # Reserved
ACC_BWP_RES128  = 0x07      # Reserved



# Gyroscope
GYR_RANGE_2000  = 0x00      # +/- 2000dps,  16.4 LSB/dps
GYR_RANGE_1000  = 0x01      # +/- 1000dps,  32.8 LSB/dps
GYR_RANGE_500   = 0x02      # +/- 500dps,   65.6 LSB/dps
GYR_RANGE_250   = 0x03      # +/- 250dps,  131.2 LSB/dps
GYR_RANGE_125   = 0x04      # +/- 125dps,  262.4 LSB/dps
GYR_ODR_3200    = 0x0D      # 3200Hz
GYR_ODR_1600    = 0x0C      # 1600Hz
GYR_ODR_800     = 0x0B      # 800Hz
GYR_ODR_400     = 0x0A      # 400Hz
GYR_ODR_200     = 0x09      # 200Hz
GYR_ODR_100     = 0x08      # 100Hz
GYR_ODR_50      = 0x07      # 50Hz
GYR_ODR_25      = 0x06      # 25Hz
GYR_BWP_OSR4    = 0x00      # OSR4
GYR_BWP_OSR2    = 0x01      # OSR2
GYR_BWP_NORMAL  = 0x02      # Normal
