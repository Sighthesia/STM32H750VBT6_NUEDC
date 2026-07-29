#ifndef CAR_ATTITUDE_CONTROL_H
#define CAR_ATTITUDE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CAR_ATTITUDE_CONTROL_PID = 0,
    CAR_ATTITUDE_CONTROL_LADRC = 1
} car_attitude_control_mode_t;

typedef struct
{
    float mode;
    float base_speed;
    float heading_kp;
    float heading_ki;
    float heading_kd;
    float rate_kp;
    float rate_ki;
    float rate_kd;
    float ladrc_observer_bandwidth;
    float ladrc_controller_bandwidth;
    float ladrc_plant_gain;
    float max_rate;
    float max_differential;
    float invert_x;
    float invert_y;
    float invert_z;
} car_attitude_control_config_t;

void Car_Attitude_Control_Init(void);
void Car_Attitude_Control_Process(void);
void Car_Attitude_Control_Reset_Defaults(void);
bool Car_Attitude_Control_Apply_Config(const car_attitude_control_config_t *config);
bool Car_Attitude_Control_Set_Enabled(bool enabled);
bool Car_Attitude_Control_Is_Enabled(void);
car_attitude_control_config_t *Car_Attitude_Control_Get_Config(void);

#endif /* CAR_ATTITUDE_CONTROL_H */
