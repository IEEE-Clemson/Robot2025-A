#ifndef _GYRO_H_
#define _GYRO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hardware/i2c.h"

#include "madgwick_filter.h"

struct BNO055 {
    i2c_inst_t *i2c;
    uint8_t addr;
    float acc_range;
    uint acc_odr;
    uint gyr_range;
    uint gyr_odr;
    struct quaternion q;
};

void bno_init(struct BNO055 *bno, i2c_inst_t *i2c, uint8_t sda, uint8_t scl);
void bno_get_raw_gyr_data(struct BNO055* bno, int16_t *gx, int16_t* gy, int16_t* gz);

#ifdef __cplusplus
}
#endif

#endif
