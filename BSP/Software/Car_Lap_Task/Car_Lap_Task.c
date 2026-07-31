#include "Car_Lap_Task.h"

#include "App_Role.h"
#include "Car_Attitude_Control.h"
#include "Car_Motor_USART.h"
#include "Gray_Sensor.h"
#include "main.h"

#include <math.h>

#define CAR_LAP_CROSS_PATTERN       0xFFU
#define CAR_LAP_LEAVE_CONFIRM_MS    100U
#define CAR_LAP_STOP_DELAY_MAX_MS   2000U
#define CAR_LAP_DEFAULT_COUNTS_PER_METER 285053.625f
#define CAR_LAP_MIN_COUNTS_PER_METER 1.0f
#define CAR_LAP_MAX_COUNTS_PER_METER 1000000000.0f
#define CAR_LAP_ENCODER_TIMEOUT_MS    200U
#define CAR_LAP_ARM_YAW_DEG           180.0f
#define CAR_LAP_CROSS_CONFIRM_MS      50U

static car_lap_state_t state;
static uint32_t started_ms;
static uint32_t elapsed_ms;
static uint32_t condition_started_ms;
static uint32_t stop_delay_started_ms;
static uint32_t stop_delay_ms;
static int32_t encoder_baseline[2];
static uint8_t encoder_baseline_valid;
static float counts_per_meter;
static float distance_m;
static float average_speed_mps;
static uint8_t half_turn_armed;
static car_lap_start_result_t last_start_result;

static void update_odometry(void)
{
    const car_motor_telemetry_t *telemetry = Car_Motor_Get_Telemetry();
    float total_counts = 0.0f;

    if (!telemetry->total_encoder_valid ||
        (uint32_t)(HAL_GetTick() - telemetry->total_encoder_ms) > CAR_LAP_ENCODER_TIMEOUT_MS)
    {
        encoder_baseline_valid = 0U;
        return;
    }
    if (encoder_baseline_valid == 0U)
    {
        for (uint8_t i = 0U; i < 2U; i++) encoder_baseline[i] = telemetry->total_encoder[i];
        encoder_baseline_valid = 1U;
        return;
    }
    for (uint8_t i = 0U; i < 2U; i++)
    {
        int64_t delta = (int32_t)((uint32_t)telemetry->total_encoder[i] -
                                  (uint32_t)encoder_baseline[i]);
        if (delta < 0) delta = -delta;
        total_counts += (float)delta;
    }
    distance_m = total_counts / (2.0f * counts_per_meter);
    average_speed_mps = elapsed_ms != 0U ? distance_m * 1000.0f / (float)elapsed_ms : 0.0f;
}

static bool is_cross(uint8_t pattern)
{
    return (Gray_Sensor_Get_Stable() & pattern) == pattern;
}

void Car_Lap_Task_Init(void)
{
    state = CAR_LAP_STATE_IDLE;
    started_ms = 0U;
    elapsed_ms = 0U;
    condition_started_ms = 0U;
    stop_delay_started_ms = 0U;
    stop_delay_ms = 0U;
    counts_per_meter = CAR_LAP_DEFAULT_COUNTS_PER_METER;
    distance_m = 0.0f;
    average_speed_mps = 0.0f;
    half_turn_armed = 0U;
    last_start_result = CAR_LAP_START_NOT_ATTEMPTED;
    encoder_baseline_valid = 0U;
    for (uint8_t i = 0U; i < 2U; i++) encoder_baseline[i] = 0;
}

bool Car_Lap_Task_Start(void)
{
    const car_motor_telemetry_t *telemetry = Car_Motor_Get_Telemetry();

    if (App_Role_Get() != APP_ROLE_CAR)
    {
        last_start_result = CAR_LAP_START_WRONG_ROLE;
        return false;
    }
    if (state != CAR_LAP_STATE_IDLE && state != CAR_LAP_STATE_FINISHED &&
        state != CAR_LAP_STATE_ABORTED)
    {
        last_start_result = CAR_LAP_START_BUSY;
        return false;
    }

    started_ms = HAL_GetTick();
    elapsed_ms = 0U;
    condition_started_ms = 0U;
    stop_delay_started_ms = 0U;
    distance_m = 0.0f;
    average_speed_mps = 0.0f;
    half_turn_armed = 0U;
    encoder_baseline_valid = telemetry->total_encoder_valid != 0U &&
                             (uint32_t)(started_ms - telemetry->total_encoder_ms) <=
                             CAR_LAP_ENCODER_TIMEOUT_MS;
    for (uint8_t i = 0U; i < 2U; i++) encoder_baseline[i] = telemetry->total_encoder[i];
    state = CAR_LAP_STATE_WAIT_LEAVE;
    if (!Car_Attitude_Control_Set_Enabled(true))
    {
        state = CAR_LAP_STATE_ABORTED;
        last_start_result = CAR_LAP_START_CONTROL_FAILED;
        return false;
    }
    last_start_result = CAR_LAP_START_OK;
    return true;
}

bool Car_Lap_Task_Stop(void)
{
    bool active;

    if (App_Role_Get() != APP_ROLE_CAR) return false;
    active = state == CAR_LAP_STATE_WAIT_LEAVE || state == CAR_LAP_STATE_RUNNING ||
             state == CAR_LAP_STATE_STOP_DELAY;
    if (active)
    {
        elapsed_ms = HAL_GetTick() - started_ms;
        update_odometry();
        state = CAR_LAP_STATE_ABORTED;
    }
    condition_started_ms = 0U;
    half_turn_armed = 0U;
    return Car_Attitude_Control_Set_Enabled(false);
}

bool Car_Lap_Task_Set_Stop_Delay(uint32_t delay_ms)
{
    if (delay_ms > CAR_LAP_STOP_DELAY_MAX_MS ||
        state == CAR_LAP_STATE_WAIT_LEAVE || state == CAR_LAP_STATE_RUNNING ||
        state == CAR_LAP_STATE_STOP_DELAY)
    {
        return false;
    }
    stop_delay_ms = delay_ms;
    return true;
}

uint32_t Car_Lap_Task_Get_Stop_Delay(void)
{
    return stop_delay_ms;
}

bool Car_Lap_Task_Set_Counts_Per_Meter(float value)
{
    if (!(value >= CAR_LAP_MIN_COUNTS_PER_METER && value <= CAR_LAP_MAX_COUNTS_PER_METER) ||
        state == CAR_LAP_STATE_WAIT_LEAVE || state == CAR_LAP_STATE_RUNNING ||
        state == CAR_LAP_STATE_STOP_DELAY)
    {
        return false;
    }
    counts_per_meter = value;
    return true;
}

float Car_Lap_Task_Get_Counts_Per_Meter(void)
{
    return counts_per_meter;
}

void Car_Lap_Task_Process(void)
{
    uint32_t now = HAL_GetTick();
    bool active = state == CAR_LAP_STATE_WAIT_LEAVE || state == CAR_LAP_STATE_RUNNING ||
                  state == CAR_LAP_STATE_STOP_DELAY;

    if (active)
    {
        elapsed_ms = now - started_ms;
        update_odometry();
    }
    if (active && !Car_Attitude_Control_Is_Enabled())
    {
        state = CAR_LAP_STATE_ABORTED;
        condition_started_ms = 0U;
        return;
    }

    if (state == CAR_LAP_STATE_WAIT_LEAVE)
    {
        if (is_cross(CAR_LAP_CROSS_PATTERN))
        {
            condition_started_ms = 0U;
        }
        else if (condition_started_ms == 0U)
        {
            condition_started_ms = now;
        }
        else if ((uint32_t)(now - condition_started_ms) >= CAR_LAP_LEAVE_CONFIRM_MS)
        {
            state = CAR_LAP_STATE_RUNNING;
            condition_started_ms = 0U;
        }
    }
    else if (state == CAR_LAP_STATE_RUNNING)
    {
        const car_attitude_control_debug_t *control = Car_Attitude_Control_Get_Debug();
        const float yaw_magnitude = control->tracking_angle < 0.0f ?
                                    -control->tracking_angle : control->tracking_angle;
        if (!half_turn_armed)
        {
            if (yaw_magnitude >= CAR_LAP_ARM_YAW_DEG)
            {
                half_turn_armed = 1U;
                condition_started_ms = 0U;
            }
        }
        if (!half_turn_armed || !is_cross(0x3CU))
        {
            condition_started_ms = 0U;
        }
        else if (condition_started_ms == 0U)
        {
            condition_started_ms = now;
        }
        else if ((uint32_t)(now - condition_started_ms) >= CAR_LAP_CROSS_CONFIRM_MS)
        {
            state = CAR_LAP_STATE_STOP_DELAY;
            stop_delay_started_ms = now;
            condition_started_ms = 0U;
        }
    }
    else if (state == CAR_LAP_STATE_STOP_DELAY &&
             (uint32_t)(now - stop_delay_started_ms) >= stop_delay_ms)
    {
        elapsed_ms = now - started_ms;
        (void)Car_Attitude_Control_Set_Enabled(false);
        state = CAR_LAP_STATE_FINISHED;
    }
}

void Car_Lap_Task_Get_Status(car_lap_status_t *status)
{
    if (status == NULL) return;
    status->state = state;
    status->elapsed_ms = elapsed_ms;
    status->stop_delay_ms = stop_delay_ms;
    status->distance_m = distance_m;
    status->average_speed_mps = average_speed_mps;
    status->relative_yaw_deg = Car_Attitude_Control_Get_Debug()->tracking_angle;
    status->gray_state = Gray_Sensor_Get_Stable() & CAR_LAP_CROSS_PATTERN;
    status->half_turn_armed = half_turn_armed;
    status->last_start_result = last_start_result;
}
