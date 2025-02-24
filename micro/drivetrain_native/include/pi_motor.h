#ifndef _PI_MOTOR_H_
#define _PI_MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "encoder.h"
#include "pid_control.h"

#define MA_SIZE 10
static const float PI_MOTOR_DEFAULT_FEED_FORWARD = 0.045;

struct PIMotor {
    struct Encoder encoder;
    struct PIDController pid;
    uint pin_f;
    uint pin_r;
    bool invert_motor;

    float setpoint;
    float ff;
    float cprad;
    float target_vel;
    float cur_vel;
    float out;
    bool use_pi;

    int prev_count;
    float ma_buff[MA_SIZE];
    int ma_index;
};

int pi_motor_init(struct PIMotor *motor, 
                  struct PIDParams *pid_params,
                  pio_hw_t *pio,
                  uint sm,
                  uint a,
                  uint b, 
                  uint pin_f,
                  uint pin_r,
                  bool inverted);

void pi_motor_drive_raw(struct PIMotor* motor, float percent_out);
void pi_motor_update(struct PIMotor* motor, float dt);

#ifdef __cplusplus
}
#endif
#endif // _PI_MOTOR_H_