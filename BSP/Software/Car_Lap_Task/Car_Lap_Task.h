#ifndef CAR_LAP_TASK_H
#define CAR_LAP_TASK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CAR_LAP_STATE_IDLE = 0,
    CAR_LAP_STATE_WAIT_LEAVE,
    CAR_LAP_STATE_RUNNING,
    CAR_LAP_STATE_STOP_DELAY,
    CAR_LAP_STATE_FINISHED,
    CAR_LAP_STATE_ABORTED
} car_lap_state_t;

typedef struct
{
    float half_turn_angle_deg;
    float half_turn_window_deg;
    float half_turn_confirm_ms;
    float cross_confirm_ms;
} car_lap_config_t;

typedef struct
{
    car_lap_state_t state;
    uint32_t elapsed_ms;
    uint32_t stop_delay_ms;
    float distance_m;
    float average_speed_mps;
    float relative_yaw_deg;
    uint8_t gray_state;
    uint8_t half_turn_armed;
} car_lap_status_t;

void Car_Lap_Task_Init(void);
void Car_Lap_Task_Process(void);
bool Car_Lap_Task_Start(void);
bool Car_Lap_Task_Stop(void);
bool Car_Lap_Task_Clear(void);
bool Car_Lap_Task_Set_Stop_Delay(uint32_t delay_ms);
uint32_t Car_Lap_Task_Get_Stop_Delay(void);
bool Car_Lap_Task_Set_Counts_Per_Meter(float counts_per_meter);
float Car_Lap_Task_Get_Counts_Per_Meter(void);
car_lap_config_t *Car_Lap_Task_Get_Config(void);
bool Car_Lap_Task_Apply_Config(const car_lap_config_t *config);
void Car_Lap_Task_Get_Status(car_lap_status_t *status);

#endif /* CAR_LAP_TASK_H */
