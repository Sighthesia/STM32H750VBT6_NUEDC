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

typedef enum
{
    CAR_LAP_START_NOT_ATTEMPTED = 0,
    CAR_LAP_START_OK,
    CAR_LAP_START_WRONG_ROLE,
    CAR_LAP_START_BUSY,
    CAR_LAP_START_CONTROL_FAILED
} car_lap_start_result_t;

typedef enum
{
    CAR_LAP_SEGMENT_WAIT_START = 0,
    CAR_LAP_SEGMENT_STRAIGHT_AB,
    CAR_LAP_SEGMENT_ARC_BC,
    CAR_LAP_SEGMENT_STRAIGHT_CD,
    CAR_LAP_SEGMENT_ARC_DA,
    CAR_LAP_SEGMENT_FINISH
} car_lap_segment_t;

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
    uint8_t odometry_valid;
    uint8_t geometry_tracking_valid;
    car_lap_segment_t segment;
    car_lap_start_result_t last_start_result;
} car_lap_status_t;

void Car_Lap_Task_Init(void);
void Car_Lap_Task_Process(void);
bool Car_Lap_Task_Start(void);
bool Car_Lap_Task_Stop(void);
bool Car_Lap_Task_Set_Stop_Delay(uint32_t delay_ms);
uint32_t Car_Lap_Task_Get_Stop_Delay(void);
bool Car_Lap_Task_Set_Counts_Per_Meter(float counts_per_meter);
float Car_Lap_Task_Get_Counts_Per_Meter(void);
void Car_Lap_Task_Get_Status(car_lap_status_t *status);

#endif /* CAR_LAP_TASK_H */
