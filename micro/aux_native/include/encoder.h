#ifndef _ENCODER_H_
#define _ENCODER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <hardware/pio.h>

/// @brief Struct representing an quadrature encoder that is processed
/// through the RP2040 Programmable IO (PIO) engine. This method will only track encoder clicks (position)
/// and velocity must be solved by computing the derivative of the clicks
///
/// To create an instance, use encoder_init
///
/// @sa encoder_init
/// @sa encoder_get_count
/// @sa encoder_reset_count
/// @sa encoder_set_count
struct Encoder {
    /// @brief Instance of PIO engine to use
    pio_hw_t* pio;
    /// @brief State machine index of PIO
    uint sm;
    /// @brief Lower gpio number pin of quadrature encoder
    uint low_pin;
    /// @brief Upper gpio number pin of quadrature encoder
    uint high_pin;
    /// @brief True if A channel is the high_pin
    bool pin_swapped;
};


void encoder_init_pio(pio_hw_t* pio);

/// @brief Initializes an Encoder struct with the given pins for the A and B channel.
/// GPIO Pin A and B must only differ in position in 1 e.g. A = 1, B = 2 is valid, but A = 3, B = 5 is invalid
///
/// @param encoder Pointer to encoder to initialized
/// @param pio Instance of PIO engine to use. This must only be used for encoders since each PIO engine can only run a single program
/// @param sm State machine index of PIO. Each PIO contains 4 state machines and must be unique across encoders
/// @param a GPIO of A channel of encoder
/// @param b GPIO of B channel of encoder
/// @return 0 if initialization was correct, negative value if error occurred
int encoder_init(struct Encoder* encoder, pio_hw_t* pio, uint sm, uint a, uint b);

/// @brief Gets the current pulse count of a given encoder where positive is counter-clockwise
/// To convert this to a rotation, the clicks per rotation of the physical encoder must be known
/// @param encoder Encoder to get pulse count of
/// @return Current pulse count of encoder
int32_t encoder_get_count(struct Encoder* encoder);

/// @brief Resets the encoders click count to zero
/// @param encoder Encoder to reset
void encoder_reset_count(struct Encoder* encoder);   

/// @brief Sets the encoder click count to a specified value
/// @param encoder Encoder to set count of
/// @param count Count of encoder to set
void encoder_set_count(struct Encoder* encoder, int32_t count);

#ifdef __cplusplus
}
#endif
#endif // _ENCODER_H_