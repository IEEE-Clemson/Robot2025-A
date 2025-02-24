#include "pi_motor.h"

#include <hardware/pwm.h>
#include <math.h>

int pi_motor_init(struct PIMotor *motor, 
                  struct PIDParams *pid_params,
                  pio_hw_t* pio,
                  uint sm,
                  uint a,
                  uint b,
                  uint pin_f,
                  uint pin_r,
                  bool inverted)
{
    int res;
    int cur_count;
    int i;
    pwm_config config;
    uint slice_f = pwm_gpio_to_slice_num(pin_f);
    uint slice_r = pwm_gpio_to_slice_num(pin_r);

    res = encoder_init(&motor->encoder, pio, sm, a, b);
    if(res < 0)
        return res;
    pid_controller_init(&motor->pid, pid_params);

    // Init other settings
    motor->pin_f = pin_f;
    motor->pin_r = pin_r;
    motor->invert_motor = inverted;
    motor->ff = PI_MOTOR_DEFAULT_FEED_FORWARD;

    motor->target_vel = 0.0f;
    motor->out = 0.0f;
    
    // Init moving average filter
    cur_count = encoder_get_count(&motor->encoder);
    motor->prev_count = cur_count;
    motor->ma_index = 0;
    for(i = 0; i < MA_SIZE; i++) {
        motor->ma_buff[i] = 0.0f;
    }

    motor->cprad = 2800.0f / (2 * 3.141592);
    // Initialize pwm pins
    gpio_set_function(pin_f, GPIO_FUNC_PWM);
    gpio_set_function(pin_r, GPIO_FUNC_PWM);
    
    config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f);
    pwm_init(slice_f, &config, true);
    pwm_init(slice_r, &config, true);
    
    return 0;
}

void pi_motor_drive_raw(struct PIMotor *motor, float percent_out)
{
    uint16_t out_f, out_r;
    float deadband = 0.05f;

    if(motor->invert_motor)
        percent_out = -1.0f * percent_out;

    // SAFETY: don't invert motor direction without stopping first
    if(fabs(motor->cur_vel) > 0.5 &&
        (motor->cur_vel > 0 && percent_out < 0) &&
        (motor->cur_vel < 0 && percent_out > 0)) {
        out_f = 0;
        out_r = 0;
    } else if(percent_out > deadband) {
        out_f = (uint16_t)(percent_out * 65536);
        out_r = 0;
    } else if(percent_out < -deadband) {
        out_f = 0;
        out_r = (uint16_t)(percent_out * 65536);
    } else {
        out_f = 0;
        out_r = 0;
    }

    pwm_set_gpio_level(motor->pin_f, out_f);
    pwm_set_gpio_level(motor->pin_r, out_r);
}

void pi_motor_update(struct PIMotor *motor, float dt)
{
    int cur_count;
    int i;
    float raw_vel;
    float vel;

    motor->pid.setpoint = motor->setpoint;
    cur_count = encoder_get_count(&motor->encoder);
    if(motor->invert_motor)
        cur_count *= -1;
    
    raw_vel = (cur_count - motor->prev_count) / dt;
    motor->prev_count = cur_count;
    motor->ma_buff[motor->ma_index] = raw_vel;
    motor->ma_index = (motor->ma_index + 1) % MA_SIZE;
    for(i = 0; i < MA_SIZE; i++) {
        vel += motor->ma_buff[i];
    }
    vel /= MA_SIZE * motor->cprad;
    motor->cur_vel = vel;

    pid_controller_update(&motor->pid, vel, dt);
    motor->out = motor->pid.output + vel * motor->ff;
    if(motor->use_pi)
        pi_motor_drive_raw(motor, motor->out);
}
