#ifndef _ENCODER_H_
#define _ENCODER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <hardware/pio.h>

struct Encoder {
    pio_hw_t* pio;
    uint sm;
    uint low_pin;
    uint high_pin;
    bool pin_swapped;
};

/** \brief 
 * 
 */
void encoder_init_pio(pio_hw_t* pio);

int encoder_init(struct Encoder* encoder, pio_hw_t* pio, uint sm, uint a, uint b);
int32_t encoder_get_count(struct Encoder* encoder);
void encoder_reset_count(struct Encoder* encoder);   
void encoder_set_count(struct Encoder* encoder, int32_t count);

#ifdef __cplusplus
}
#endif
#endif // _ENCODER_H_