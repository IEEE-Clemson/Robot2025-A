#ifndef _COMM_H_
#define _COMM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// BNO055 Registers
// https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf

static const uint8_t CHIP_ID              = 0x00;
static const uint8_t ACC_ID               = 0x01;
static const uint8_t MAG_ID               = 0x02;
static const uint8_t GYR_ID               = 0x03;
static const uint8_t SW_REV_ID_7_0        = 0x04;
static const uint8_t SW_REV_ID_15_8       = 0x05;
static const uint8_t BL_REV_ID            = 0x06;

static const uint8_t ACC_DATA_X_7_0       = 0x08;
static const uint8_t ACC_DATA_X_15_8      = 0x09;
static const uint8_t ACC_DATA_Y_7_0       = 0x0A;
static const uint8_t ACC_DATA_Y_15_8      = 0x0B;
static const uint8_t ACC_DATA_Z_7_0       = 0x0C;
static const uint8_t ACC_DATA_Z_15_8      = 0x0D;

static const uint8_t MAG_DATA_X_7_0       = 0x0E;
static const uint8_t MAG_DATA_X_15_8      = 0x0F;
static const uint8_t MAG_DATA_Y_7_0       = 0x10;
static const uint8_t MAG_DATA_Y_15_8      = 0x11;
static const uint8_t MAG_DATA_Z_7_0       = 0x12;
static const uint8_t MAG_DATA_Z_15_8      = 0x13;

static const uint8_t GYR_DATA_X_7_0       = 0x14;
static const uint8_t GYR_DATA_X_15_8      = 0x15;
static const uint8_t GYR_DATA_Y_7_0       = 0x16;
static const uint8_t GYR_DATA_Y_15_8      = 0x17;
static const uint8_t GYR_DATA_Z_7_0       = 0x18;
static const uint8_t GYR_DATA_Z_15_8      = 0x19;

static const uint8_t EUL_DATA_X_7_0       = 0x1A;
static const uint8_t EUL_DATA_X_15_8      = 0x1B;
static const uint8_t EUL_DATA_Y_7_0       = 0x1C;
static const uint8_t EUL_DATA_Y_15_8      = 0x1D;
static const uint8_t EUL_DATA_Z_7_0       = 0x1E;
static const uint8_t EUL_DATA_Z_15_8      = 0x1F;

static const uint8_t QUAT_DATA_W_7_0      = 0x20;
static const uint8_t QUAT_DATA_W_15_8     = 0x21;
static const uint8_t QUAT_DATA_X_7_0      = 0x22;
static const uint8_t QUAT_DATA_X_15_8     = 0x23;
static const uint8_t QUAT_DATA_Y_7_0      = 0x24;
static const uint8_t QUAT_DATA_Y_15_8     = 0x25;
static const uint8_t QUAT_DATA_Z_7_0      = 0x26;
static const uint8_t QUAT_DATA_Z_15_8     = 0x27;

static const uint8_t LIA_DATA_X_7_0       = 0x28;
static const uint8_t LIA_DATA_X_15_8      = 0x29;
static const uint8_t LIA_DATA_Y_7_0       = 0x2A;
static const uint8_t LIA_DATA_Y_15_8      = 0x2B;
static const uint8_t LIA_DATA_Z_7_0       = 0x2C;
static const uint8_t LIA_DATA_Z_15_8      = 0x2D;

static const uint8_t GRV_DATA_X_7_0       = 0x2E;
static const uint8_t GRV_DATA_X_15_8      = 0x2F;
static const uint8_t GRV_DATA_Y_7_0       = 0x30;
static const uint8_t GRV_DATA_Y_15_8      = 0x31;
static const uint8_t GRV_DATA_Z_7_0       = 0x32;
static const uint8_t GRV_DATA_Z_15_8      = 0x33;

static const uint8_t TEMP                 = 0x34;

static const uint8_t CALIB_STAT           = 0x35;
static const uint8_t ST_RESULT            = 0x36;
static const uint8_t INT_STA              = 0x37;
static const uint8_t SYS_CLK_STATUS       = 0x38;
static const uint8_t SYS_STATUS           = 0x39;
static const uint8_t SYS_ERR              = 0x3A;

static const uint8_t UNIT_SEL             = 0x3B;
static const uint8_t OPR_MODE             = 0x3D;
static const uint8_t PWR_MODE             = 0x3E;
static const uint8_t SYS_TRIGGER          = 0x3F;
static const uint8_t TEMP_SOURCE          = 0x40;
static const uint8_t AXIS_MAP_CONFIG      = 0x41;
static const uint8_t AXIS_MAP_SIGN        = 0x42;

static const uint8_t ACC_OFFSET_X_7_0     = 0x55;
static const uint8_t ACC_OFFSET_X_15_8    = 0x56;
static const uint8_t ACC_OFFSET_Y_7_0     = 0x57;
static const uint8_t ACC_OFFSET_Y_15_8    = 0x58;
static const uint8_t ACC_OFFSET_Z_7_0     = 0x59;
static const uint8_t ACC_OFFSET_Z_15_8    = 0x5A;

static const uint8_t MAG_OFFSET_X_7_0     = 0x5B;
static const uint8_t MAG_OFFSET_X_15_8    = 0x5C;
static const uint8_t MAG_OFFSET_Y_7_0     = 0x5D;
static const uint8_t MAG_OFFSET_Y_15_8    = 0x5E;
static const uint8_t MAG_OFFSET_Z_7_0     = 0x5F;
static const uint8_t MAG_OFFSET_Z_15_8    = 0x60;

static const uint8_t GYR_OFFSET_X_7_0     = 0x61;
static const uint8_t GYR_OFFSET_X_15_8    = 0x62;
static const uint8_t GYR_OFFSET_Y_7_0     = 0x63;
static const uint8_t GYR_OFFSET_Y_15_8    = 0x64;
static const uint8_t GYR_OFFSET_Z_7_0     = 0x65;
static const uint8_t GYR_OFFSET_Z_15_8    = 0x66;

static const uint8_t ACC_RADIUS_7_0       = 0x67;
static const uint8_t ACC_RADIUS_15_8      = 0x68;
static const uint8_t MAG_RADIUS_7_0       = 0x69;
static const uint8_t MAG_RADIUS_15_8      = 0x6A;

// Common Definitions
// OPR_MODE
static const uint8_t OPR_MODE_CONFIGMODE = 0x00;
static const uint8_t OPR_MODE_ACCONLY    = 0x01;
static const uint8_t OPR_MODE_MAGONLY    = 0x02;
static const uint8_t OPR_MODE_GYROONLY   = 0x03;
static const uint8_t OPR_MODE_ACCMAG     = 0x04;
static const uint8_t OPR_MODE_ACCGYRO    = 0x05;
static const uint8_t OPR_MODE_MAGGYRO    = 0x06;
static const uint8_t OPR_MODE_AMG        = 0x07;
static const uint8_t OPR_MODE_IMU        = 0x08;
static const uint8_t OPR_MODE_COMPASSS   = 0x09;
static const uint8_t OPR_MODE_M4G        = 0x0A;
static const uint8_t OPR_MODE_NDOF_FM    = 0x0B;
static const uint8_t OPR_MODE_NDOF       = 0x0C;

// UNIT_SEL
static const uint8_t UNIT_SEL_MPS2       = 0x0;
static const uint8_t UNIT_SEL_MG         = 0x01;

static const uint8_t UNIT_SEL_DPS        = 0x0;
static const uint8_t UNIT_SEL_RADS       = 0x02;

static const uint8_t UNIT_SEL_DEG        = 0x0;
static const uint8_t UNIT_SEL_RAD        = 0x04;

static const uint8_t UNIT_SEL_DEGC       = 0x0;
static const uint8_t UNIT_SEL_DEGF       = 0x10;

// SYS_TRIGGER
static const uint8_t SYS_TRIGGER_TEST    = 0x00;

static const uint8_t SYS_TRIGGER_RST_SYS = 0x20;
static const uint8_t SYS_TRIGGER_RST_INT = 0x40;
static const uint8_t SYS_TRIGGER_CLK_SEL = 0x80;


// General
static const float   GRAVITY         = 9.81288;
static const float   DEG2RAD         = 3.141592653589793 / 180.0;
static const float   HERTZ_100       = 0.01;
static const float   HERTZ_200       = 0.005;
static const uint8_t BIT_0           = 1 << 0;
static const uint8_t BIT_1           = 1 << 1;
static const uint8_t BIT_2           = 1 << 2;
static const uint8_t BIT_3           = 1 << 3;
static const uint8_t BIT_4           = 1 << 4;
static const uint8_t BIT_5           = 1 << 5;
static const uint8_t BIT_6           = 1 << 6;
static const uint8_t BIT_7           = 1 << 7;
static const uint8_t LSB_MASK_8BIT   = 0x0F;      // 00001111
static const uint8_t MSB_MASK_8BIT   = 0xF0;      // 11110000
static const uint8_t FULL_MASK_8BIT  = 0xFF;      // 11111111
static const uint8_t LSB_MASK_8BIT_5 = 0x1F;      // 00011111
static const uint8_t LSB_MASK_8BIT_8 = 0x8F;      // 10001111
static const uint8_t LAST_2_BITS     = 0xC0;      // 11000000
static const uint8_t LAST_3_BITS     = 0xE0;      // 11100000

// Accelerometer
static const uint8_t ACC_RANGE_2G    = 0x00;      // +/- 2g
static const uint8_t ACC_RANGE_4G    = 0x01;      // +/- 4g
static const uint8_t ACC_RANGE_8G    = 0x02;      // +/- 8g
static const uint8_t ACC_RANGE_16G   = 0x03;      // +/- 16g
static const uint8_t ACC_ODR_1600    = 0x0C;      // 1600Hz
static const uint8_t ACC_ODR_800     = 0x0B;      // 800Hz
static const uint8_t ACC_ODR_400     = 0x0A;      // 400Hz
static const uint8_t ACC_ODR_200     = 0x09;      // 200Hz
static const uint8_t ACC_ODR_100     = 0x08;      // 100Hz
static const uint8_t ACC_ODR_50      = 0x07;      // 50Hz
static const uint8_t ACC_ODR_25      = 0x06;      // 25Hz
static const uint8_t ACC_BWP_OSR4    = 0x00;      // OSR4
static const uint8_t ACC_BWP_OSR2    = 0x01;      // OSR2
static const uint8_t ACC_BWP_NORMAL  = 0x02;      // Normal
static const uint8_t ACC_BWP_CIC     = 0x03;      // CIC
static const uint8_t ACC_BWP_RES16   = 0x04;      // Reserved
static const uint8_t ACC_BWP_RES32   = 0x05;      // Reserved
static const uint8_t ACC_BWP_RES64   = 0x06;      // Reserved
static const uint8_t ACC_BWP_RES128  = 0x07;      // Reserved

// Gyroscope
static const uint8_t GYR_RANGE_2000  = 0x00;      // +/- 2000dps,  16.4 LSB/dps
static const uint8_t GYR_RANGE_1000  = 0x01;      // +/- 1000dps,  32.8 LSB/dps
static const uint8_t GYR_RANGE_500   = 0x02;      // +/- 500dps,   65.6 LSB/dps
static const uint8_t GYR_RANGE_250   = 0x03;      // +/- 250dps,  131.2 LSB/dps
static const uint8_t GYR_RANGE_125   = 0x04;      // +/- 125dps,  262.4 LSB/dps
static const uint8_t GYR_ODR_3200    = 0x0D;      // 3200Hz
static const uint8_t GYR_ODR_1600    = 0x0C;      // 1600Hz
static const uint8_t GYR_ODR_800     = 0x0B;      // 800Hz
static const uint8_t GYR_ODR_400     = 0x0A;      // 400Hz
static const uint8_t GYR_ODR_200     = 0x09;      // 200Hz
static const uint8_t GYR_ODR_100     = 0x08;      // 100Hz
static const uint8_t GYR_ODR_50      = 0x07;      // 50Hz
static const uint8_t GYR_ODR_25      = 0x06;      // 25Hz
static const uint8_t GYR_BWP_OSR4    = 0x00;      // OSR4
static const uint8_t GYR_BWP_OSR2    = 0x01;      // OSR2
static const uint8_t GYR_BWP_NORMAL  = 0x02;      // Normal

#ifdef __cplusplus
}
#endif

#endif // _COMM_H_