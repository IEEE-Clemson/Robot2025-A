#ifndef _PID_CONTROL_H_
#define _PID_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

struct PIDParams {
    float p;
    float i;
    float d;
    float max_i_accum;
    float max_output;
};

struct PIDController {
    struct PIDParams* params;
    float setpoint;
    float output;

    float prev_x;
    float i_accum;
};

void pid_create_params(struct PIDParams *params, float p, float i, float d);

void pid_controller_init(struct PIDController* pid, struct PIDParams *params);
void pid_controller_update(struct PIDController* pid, float x, float dt);

#ifdef __cplusplus
}
#endif
#endif // _PID_CONTROL_H_