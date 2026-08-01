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
#define CAR_LAP_RAD_TO_DEG             57.2957795f
#define CAR_LAP_TEST4_STOP_SENSOR_MASK 0xC0U
#define CAR_LAP_DEFAULT_RAMP_MS        600U

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
static float encoder_baseline_distance_m;
static float segment_started_distance_m;
static float segment_started_yaw_deg;
static float previous_distance_m;
static uint32_t previous_distance_ms;
static float current_speed_mps;
static car_lap_segment_t segment;
static uint8_t geometry_tracking_valid;
static car_lap_arc_config_t arc_config;
static uint8_t test4_active;
static uint32_t test4_stop_started_ms;
static uint32_t test4_ramp_ms;
static float test4_cruise_speed;
static float test4_saved_tracking_method;

static void reset_test4_control(void)
{
    if (test4_active != 0U)
    {
        car_attitude_control_config_t *control = Car_Attitude_Control_Get_Config();

        control->base_speed = test4_cruise_speed;
        Car_Attitude_Control_Get_Tracking_Config()->method = test4_saved_tracking_method;
    }
    test4_active = 0U;
    test4_stop_started_ms = 0U;
}

void Car_Lap_Task_Reset_Arc_Config(void)
{
    arc_config.straight_length_m = 1.5f;
    arc_config.arc_radius_m = 0.5f;
    arc_config.arc_length_m = 1.5707963f;
    arc_config.arc_exit_yaw_deg = 165.0f;
    arc_config.optical_rate_limit_deg_s = 15.0f;
    arc_config.feedforward_rate_min_deg_s = 0.0f;
    arc_config.feedforward_rate_max_deg_s = 120.0f;
    arc_config.feedforward_rate_slew_deg_s2 = 300.0f;
    arc_config.differential_slew_s2 = 1500.0f;
    arc_config.wheel_track_m = 0.15f;
}

bool Car_Lap_Task_Apply_Arc_Config(const car_lap_arc_config_t *next)
{
    if (next == NULL || !isfinite(next->straight_length_m) ||
        !isfinite(next->arc_radius_m) || !isfinite(next->arc_length_m) ||
        !isfinite(next->arc_exit_yaw_deg) || !isfinite(next->optical_rate_limit_deg_s) ||
        !isfinite(next->feedforward_rate_min_deg_s) ||
        !isfinite(next->feedforward_rate_max_deg_s) ||
        !isfinite(next->feedforward_rate_slew_deg_s2) ||
        !isfinite(next->differential_slew_s2) || !isfinite(next->wheel_track_m) ||
        next->straight_length_m <= 0.1f || next->straight_length_m > 10.0f ||
        next->arc_radius_m <= 0.1f || next->arc_radius_m > 5.0f ||
        next->arc_length_m <= 0.1f || next->arc_length_m > 20.0f ||
        next->arc_exit_yaw_deg < 90.0f || next->arc_exit_yaw_deg > 360.0f ||
        next->optical_rate_limit_deg_s < 0.0f || next->optical_rate_limit_deg_s > 180.0f ||
        next->feedforward_rate_min_deg_s < 0.0f ||
        next->feedforward_rate_min_deg_s > 360.0f ||
        next->feedforward_rate_max_deg_s < next->feedforward_rate_min_deg_s ||
        next->feedforward_rate_max_deg_s > 720.0f ||
        next->feedforward_rate_slew_deg_s2 <= 0.0f ||
        next->feedforward_rate_slew_deg_s2 > 5000.0f ||
        next->differential_slew_s2 <= 0.0f || next->differential_slew_s2 > 20000.0f ||
        next->wheel_track_m <= 0.05f || next->wheel_track_m > 0.50f)
    {
        return false;
    }
    if (state == CAR_LAP_STATE_WAIT_LEAVE || state == CAR_LAP_STATE_RUNNING ||
        state == CAR_LAP_STATE_STOP_DELAY)
    {
        return false;
    }
    arc_config = *next;
    return true;
}

car_lap_arc_config_t *Car_Lap_Task_Get_Arc_Config(void)
{
    return &arc_config;
}

static float absolute_value(float value)
{
    return value < 0.0f ? -value : value;
}

static void set_segment(car_lap_segment_t next, float yaw_deg)
{
    segment = next;
    segment_started_distance_m = distance_m;
    segment_started_yaw_deg = yaw_deg;
    if (next == CAR_LAP_SEGMENT_ARC_BC || next == CAR_LAP_SEGMENT_ARC_DA)
    {
        float target_rate = current_speed_mps / arc_config.arc_radius_m * CAR_LAP_RAD_TO_DEG;
        if (target_rate < arc_config.feedforward_rate_min_deg_s) target_rate = arc_config.feedforward_rate_min_deg_s;
        if (target_rate > arc_config.feedforward_rate_max_deg_s) target_rate = arc_config.feedforward_rate_max_deg_s;
        (void)Car_Attitude_Control_Set_Arc_Rate(target_rate);
    }
    else
    {
        (void)Car_Attitude_Control_Set_Arc_Rate(0.0f);
    }
}

static void update_arc_feedforward(void)
{
    if (geometry_tracking_valid == 0U || encoder_baseline_valid == 0U ||
        (segment != CAR_LAP_SEGMENT_ARC_BC && segment != CAR_LAP_SEGMENT_ARC_DA))
    {
        (void)Car_Attitude_Control_Set_Arc_Rate(0.0f);
        return;
    }

    float target_rate = current_speed_mps / arc_config.arc_radius_m * CAR_LAP_RAD_TO_DEG;
    if (target_rate < arc_config.feedforward_rate_min_deg_s) target_rate = arc_config.feedforward_rate_min_deg_s;
    if (target_rate > arc_config.feedforward_rate_max_deg_s) target_rate = arc_config.feedforward_rate_max_deg_s;
    (void)Car_Attitude_Control_Set_Arc_Rate(target_rate);
}

static void update_odometry(void)
{
    const car_motor_telemetry_t *telemetry = Car_Motor_Get_Telemetry();
    float total_counts = 0.0f;

    if (!telemetry->total_encoder_valid ||
        (uint32_t)(HAL_GetTick() - telemetry->total_encoder_ms) > CAR_LAP_ENCODER_TIMEOUT_MS)
    {
        encoder_baseline_valid = 0U;
        geometry_tracking_valid = 0U;
        return;
    }
    if (encoder_baseline_valid == 0U)
    {
        for (uint8_t i = 0U; i < 2U; i++) encoder_baseline[i] = telemetry->total_encoder[i];
        encoder_baseline_distance_m = distance_m;
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
    distance_m = encoder_baseline_distance_m + total_counts / (2.0f * counts_per_meter);
    average_speed_mps = elapsed_ms != 0U ? distance_m * 1000.0f / (float)elapsed_ms : 0.0f;
    const uint32_t now = HAL_GetTick();
    if (distance_m > previous_distance_m && now != previous_distance_ms)
    {
        const float sample_speed = (distance_m - previous_distance_m) * 1000.0f /
                                   (float)(now - previous_distance_ms);
        current_speed_mps += 0.25f * (sample_speed - current_speed_mps);
        previous_distance_m = distance_m;
        previous_distance_ms = now;
    }
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
    encoder_baseline_distance_m = 0.0f;
    segment_started_distance_m = 0.0f;
    segment_started_yaw_deg = 0.0f;
    previous_distance_m = 0.0f;
    previous_distance_ms = 0U;
    current_speed_mps = 0.0f;
    segment = CAR_LAP_SEGMENT_WAIT_START;
    geometry_tracking_valid = 0U;
    test4_active = 0U;
    test4_stop_started_ms = 0U;
    test4_ramp_ms = 0U;
    test4_cruise_speed = 0.0f;
    test4_saved_tracking_method = 0.0f;
    Car_Lap_Task_Reset_Arc_Config();
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
    encoder_baseline_distance_m = 0.0f;
    segment_started_distance_m = 0.0f;
    segment_started_yaw_deg = 0.0f;
    previous_distance_m = 0.0f;
    previous_distance_ms = started_ms;
    current_speed_mps = 0.0f;
    segment = CAR_LAP_SEGMENT_WAIT_START;
    encoder_baseline_valid = telemetry->total_encoder_valid != 0U &&
                             (uint32_t)(started_ms - telemetry->total_encoder_ms) <=
                             CAR_LAP_ENCODER_TIMEOUT_MS;
    geometry_tracking_valid = encoder_baseline_valid;
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

bool Car_Lap_Task_Start_Test(uint8_t test_id)
{
    car_attitude_control_config_t *control;

    if (test_id != 4U) return Car_Lap_Task_Start();
    if (!Car_Lap_Task_Start()) return false;

    control = Car_Attitude_Control_Get_Config();
    test4_active = 1U;
    test4_stop_started_ms = 0U;
    test4_cruise_speed = control->base_speed;
    test4_saved_tracking_method = Car_Attitude_Control_Get_Tracking_Config()->method;
    test4_ramp_ms = control->start_ramp_ms > 0.0f ?
                    (uint32_t)control->start_ramp_ms : CAR_LAP_DEFAULT_RAMP_MS;
    Car_Attitude_Control_Get_Tracking_Config()->method =
        (float)CAR_TRACKING_METHOD_OPTICAL_IMU;
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
    segment = CAR_LAP_SEGMENT_WAIT_START;
    reset_test4_control();
    (void)Car_Attitude_Control_Set_Arc_Rate(0.0f);
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
        update_arc_feedforward();
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
            set_segment(CAR_LAP_SEGMENT_STRAIGHT_AB,
                        Car_Attitude_Control_Get_Debug()->tracking_angle);
            condition_started_ms = 0U;
        }
    }
    else if (state == CAR_LAP_STATE_RUNNING)
    {
        const car_attitude_control_debug_t *control = Car_Attitude_Control_Get_Debug();
        const float yaw_magnitude = control->tracking_angle < 0.0f ?
                                    -control->tracking_angle : control->tracking_angle;
        const float segment_distance = distance_m - segment_started_distance_m;
        const float segment_yaw = absolute_value(control->tracking_angle - segment_started_yaw_deg);

        if (test4_active != 0U)
        {
            if ((Gray_Sensor_Get_Stable() & CAR_LAP_TEST4_STOP_SENSOR_MASK) != 0U)
            {
                test4_stop_started_ms = now;
                state = CAR_LAP_STATE_STOP_DELAY;
            }
            return;
        }
        if (geometry_tracking_valid != 0U && encoder_baseline_valid != 0U)
        {
            if (segment == CAR_LAP_SEGMENT_STRAIGHT_AB &&
                segment_distance >= arc_config.straight_length_m)
            {
                set_segment(CAR_LAP_SEGMENT_ARC_BC, control->tracking_angle);
            }
            else if (segment == CAR_LAP_SEGMENT_ARC_BC &&
                     segment_distance >= arc_config.arc_length_m &&
                     segment_yaw >= arc_config.arc_exit_yaw_deg)
            {
                set_segment(CAR_LAP_SEGMENT_STRAIGHT_CD, control->tracking_angle);
            }
            else if (segment == CAR_LAP_SEGMENT_STRAIGHT_CD &&
                segment_distance >= arc_config.straight_length_m)
            {
                set_segment(CAR_LAP_SEGMENT_ARC_DA, control->tracking_angle);
            }
            else if (segment == CAR_LAP_SEGMENT_ARC_DA &&
                     segment_distance >= arc_config.arc_length_m &&
                     segment_yaw >= arc_config.arc_exit_yaw_deg)
            {
                set_segment(CAR_LAP_SEGMENT_FINISH, control->tracking_angle);
            }
        }
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
    else if (state == CAR_LAP_STATE_STOP_DELAY && test4_active != 0U)
    {
        const uint32_t elapsed_stop_ms = now - test4_stop_started_ms;
        car_attitude_control_config_t *control = Car_Attitude_Control_Get_Config();

        if (elapsed_stop_ms < test4_ramp_ms)
        {
            control->base_speed = test4_cruise_speed *
                                  (float)(test4_ramp_ms - elapsed_stop_ms) /
                                  (float)test4_ramp_ms;
            return;
        }
        elapsed_ms = now - started_ms;
        segment = CAR_LAP_SEGMENT_FINISH;
        reset_test4_control();
        (void)Car_Attitude_Control_Set_Arc_Rate(0.0f);
        (void)Car_Attitude_Control_Set_Enabled(false);
        state = CAR_LAP_STATE_FINISHED;
    }
    else if (state == CAR_LAP_STATE_STOP_DELAY &&
             (uint32_t)(now - stop_delay_started_ms) >= stop_delay_ms)
    {
        elapsed_ms = now - started_ms;
        segment = CAR_LAP_SEGMENT_FINISH;
        (void)Car_Attitude_Control_Set_Arc_Rate(0.0f);
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
    status->current_speed_mps = current_speed_mps;
    status->relative_yaw_deg = Car_Attitude_Control_Get_Debug()->tracking_angle;
    status->gray_state = Gray_Sensor_Get_Stable() & CAR_LAP_CROSS_PATTERN;
    status->half_turn_armed = half_turn_armed;
    status->odometry_valid = encoder_baseline_valid;
    status->geometry_tracking_valid = geometry_tracking_valid;
    status->segment = segment;
    status->last_start_result = last_start_result;
}
