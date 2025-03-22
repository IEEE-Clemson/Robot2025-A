#ifndef _COMM_H_
#define _COMM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Robust boolean variables to maximize difference between true and false
const int RTRUE = 0xAA;

// Layout of the I2C registers
struct __attribute__((__packed__)) I2CMemLayout {
    // START
    uint8_t armed;
    uint8_t light_on;

    // INTAKE
    int16_t target_intake_v;
    int16_t cur_intake_v;

    // DUMPER
    uint8_t dumper_deploy;
    uint8_t dumper_active;

    // BEACON
    uint8_t beacon_pos; // 0 - Up, 1 - Travel, 2 - Deploy 

    // BOX MOVER
    uint8_t box_mover_pos; // 0 - Down / Grabbing, 1 - Up / Released
};


#ifdef __cplusplus
}
#endif

#endif // _COMM_H_