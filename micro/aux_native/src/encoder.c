#include "encoder.h"
#include "encoder.pio.h"

static int encoder_pio_offset = -1;

uint32_t encoder_get_count_raw(struct Encoder *encoder) {
    // HACK
    // PIO program repeatedly fills FIFO with encoder counts, regardless of whether its full
    // To read, clear the FIFO and wait for a value
    
    // Clear RX and TX FIFO
    pio_sm_clear_fifos(encoder->pio, encoder->sm);
    // Wait for and read first value from RX FIFO
    return pio_sm_get_blocking(encoder->pio, encoder->sm);
}

void encoder_set_count_raw(struct Encoder *encoder, int32_t count) {
    pio_sm_put(encoder->pio, encoder->sm, count);
}

void encoder_init_pio(pio_hw_t *pio)
{
    if(encoder_pio_offset == -1)
        encoder_pio_offset = pio_add_program(pio, &pio_encoder_program);
}

int encoder_init(struct Encoder *encoder, pio_hw_t *pio, uint sm, uint a, uint b)
{
    if(encoder == NULL)
        return -1;
    
    if(sm > 3)
        return -2;

    if(a > b) {
        if(a - b != 1)
            return -3; 
        encoder->high_pin = a;
        encoder->low_pin = b;
        encoder->pin_swapped = true;
    } else {
        if(b - a != 1)
            return -3; 
        encoder->high_pin = b;
        encoder->low_pin = a;
        encoder->pin_swapped = false;
    }
    encoder->pio = pio;
    encoder->sm = sm;

    pio_gpio_init(pio, a);
    pio_gpio_init(pio, b);

    // Pull up encoder pins
    gpio_set_pulls(a, true, false);
    gpio_set_pulls(b + 1, true, false);

    // Init PIO params
    encoder_program_init(pio, sm, encoder_pio_offset, encoder->low_pin);

    return 0;
}

int32_t encoder_get_count(struct Encoder *encoder)
{
    uint32_t count = encoder_get_count_raw(encoder);
    return encoder->pin_swapped?-count:count;
}

void encoder_reset_count(struct Encoder *encoder)
{
    encoder_set_count_raw(encoder, 0);
}

void encoder_set_count(struct Encoder *encoder, int32_t count)
{
    encoder_set_count_raw(encoder, count);
}
