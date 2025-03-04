#ifndef _GYRO_H_
#define _GYRO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hardware/i2c.h"

#include "madgwick_filter.h"

// UNIT CONVERSION
static const float REG2DEG = 1.0f / 16.0f;
static const float REG2RAD = 1.0f / 900.0f;

struct BNO055 {
    i2c_inst_t *i2c;
    uint8_t addr;
    float acc_range;
    uint acc_odr;
    uint gyr_range;
    uint gyr_odr;
    struct quaternion q;
};

struct BNO055_GYRO {
    i2c_inst_t *i2c;
};

void bno_init(struct BNO055 *bno, i2c_inst_t *i2c, uint8_t sda, uint8_t scl);
void bno_get_raw_gyr_data(struct BNO055* bno, int16_t *gx, int16_t* gy, int16_t* gz);
void bno_get_raw_acc_data(struct BNO055* bmi, int16_t *ax, int16_t* ay, int16_t*az);

void bno_gyro_init(struct BNO055_GYRO *bno, i2c_inst_t *i2c, uint8_t sda, uint8_t scl);
void bno_gyro_get_euler_angles_raw(struct BNO055_GYRO* bno, int16_t *roll, int16_t* pitch, int16_t* yaw);
void bno_gyro_reset(struct BNO055_GYRO* bmi, int16_t roll, int16_t pitch, int16_t yaw);

#ifdef __cplusplus
}
#endif

#endif
