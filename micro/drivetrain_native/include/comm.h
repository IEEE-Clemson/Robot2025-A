#ifndef _COMM_H_
#define _COMM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Layout of the I2C registers
struct __attribute__((__packed__)) I2CMemLayout {
    int16_t target_vx;
    int16_t target_vy;
    int16_t target_omega;

    int16_t cur_vx;
    int16_t cur_vy;
    int16_t cur_omega;

    int16_t theta;
};


#ifdef __cplusplus
}
#endif

#endif // _COMM_H_