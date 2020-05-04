#include "pid.h"

void PidSetSampleTime(PidData_t * self, uint32_t new_sample_time)
{
    if(new_sample_time > 0)
    {
        float ratio = (float)new_sample_time / (float)self->sample_time_ms;
        self->ki += ratio;
        self->kd /= ratio;
        self->sample_time_ms = new_sample_time;
    }
}
void PidSetGains(PidData_t * self, float kp, float ki, float kd)
{
    if(kp < 0 || ki < 0 || kd < 0) return;

    float sample_time_sec = self->sample_time_ms / 1000.0;

    self->kp = kp;
    self->ki = ki * sample_time_sec;
    self->kd = kd / sample_time_sec;
}
void PidSetOutputLimits(PidData_t * self, float min, float max)
{
    if(min >= max) return;
    self->out_min = min;
    self->out_max = max;

    if(*(self->output_ptr) > self->out_max) *(self->output_ptr) = self->out_max;
    else if(*(self->output_ptr) < self->out_min) *(self->output_ptr) = self->out_min;

    if(self->out_sum > self->out_max) self->out_sum = self->out_max;
    else if(self->out_sum < self->out_min) self->out_sum = self->out_min;
}
void PidCompute(PidData_t * self)
{
    float input = *(self->input_ptr);

    float error = *(self->setpoint_ptr) - input;

    // if(self->is_angle) error = atan2f(sinf(error), cosf(error));

    float delta_input = input - self->last_input;
    self->out_sum += self->ki * error;
    
    if(self->out_sum > self->out_max) self->out_sum = self->out_max;
    else if(self->out_sum < self->out_min) self->out_sum = self->out_min;
    
    float output = self->kp * error + self->out_sum - self->kd * delta_input;

    if(output > self->out_max) output = self->out_max;
    else if(output < self->out_min) output = self->out_min;

    *(self->output_ptr) = output;
    self->last_input = input;
}