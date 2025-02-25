#ifndef _GYRO_H_
#define _GYRO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hardware/i2c.h"

#include "bmi270.h"
#include "madgwick_filter.h"

struct IMU {
    struct bmi2_dev bmi;
    i2c_inst_t *i2c;

    struct quaternion q;
};

void imu_init(struct IMU *imu, i2c_inst_t *i2c, uint8_t sda, uint8_t scl);
void imu_update(struct IMU *imu, float dt);
float imu_get_z_radians(struct IMU *imu);

#ifdef __cplusplus
}
#endif

#endif
