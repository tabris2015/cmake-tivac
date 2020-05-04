#ifndef PID_H
#define PID_H
#include <stdbool.h>
#include <stdint.h>

typedef struct PidData_t
{
    uint32_t sample_time_ms;
    uint32_t last_time_ms;
    float kp;
    float ki;
    float kd;
    float * input_ptr;
    float * output_ptr;
    float * setpoint_ptr;

    float out_min;
    float out_max;

    float out_sum;
    float err_sum;
    float last_err;

    float i_term;
    float last_input;
    bool is_angle;
}PidData_t;

void PidSetSampleTime(PidData_t * self, uint32_t new_sample_time);
void PidSetGains(PidData_t * self, float kp, float ki, float kd);
void PidSetOutputLimits(PidData_t * self, float min, float max);
void PidCompute(PidData_t * self);
#endif