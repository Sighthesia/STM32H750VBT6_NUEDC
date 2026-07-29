#include "PID.h"

void PIDController_Init(PIDController *pid)
{
    pid->integrator = 0.0f; pid->prevError = 0.0f; pid->differentiator = 0.0f; pid->prevMeasurement = 0.0f; pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;
    float denominator = 2.0f * pid->tau + pid->T;
    pid->integrator += 0.5f * pid->Ki * pid->T * (error + pid->prevError);
    if (pid->integrator > pid->limMaxInt) pid->integrator = pid->limMaxInt;
    if (pid->integrator < pid->limMinInt) pid->integrator = pid->limMinInt;
    if (denominator != 0.0f) pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) + (2.0f * pid->tau - pid->T) * pid->differentiator) / denominator;
    pid->out = pid->Kp * error + pid->integrator + pid->differentiator;
    if (pid->out > pid->limMax) pid->out = pid->limMax;
    if (pid->out < pid->limMin) pid->out = pid->limMin;
    pid->prevError = error; pid->prevMeasurement = measurement;
    return pid->out;
}
