#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct { float Kp, Ki, Kd, tau, limMin, limMax, limMinInt, limMaxInt, T; float integrator, prevError, differentiator, prevMeasurement, out; } PIDController;
void PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif /* PID_CONTROLLER_H */
