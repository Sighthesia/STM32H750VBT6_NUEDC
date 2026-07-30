#include "Car_Attitude_Control.h"

#include "Car_Motor_USART.h"
#include "Car_IMU.h"
#include "Gray_Sensor.h"
#include "PID.h"
#include "main.h"

#include <math.h>
#include <string.h>

#define CAR_CONTROL_PERIOD_S             0.01f
#define CAR_CONTROL_IMU_TIMEOUT_MS       50U
#define CAR_CONTROL_RAD_TO_DEG            57.2957795f
#define CAR_CONTROL_LINE_PERIOD_S         0.01f
#define CAR_CONTROL_LINE_D_FILTER         0.35f
#define CAR_CONTROL_CURVE_SPEED_REDUCTION 0.55f
#define CAR_CONTROL_MIN_SPEED_SCALE       0.35f
#define CAR_CONTROL_SEARCH_RATE_SCALE     0.50f
#define CAR_CONTROL_SEARCH_SPEED_SCALE    0.35f
#define CAR_CONTROL_LOST_STOP_MS          300U

typedef struct
{
    float z1;
    float z2;
    float previous_output;
    uint8_t initialized;
} car_ladrc_t;

static car_attitude_control_config_t config;
static car_tracking_config_t tracking_config;
static car_attitude_control_debug_t debug;
static PIDController heading_pid;
static PIDController rate_pid;
static car_ladrc_t rate_ladrc;
static uint32_t last_sequence;
static uint32_t last_sample_ms;
static float line_error_previous;
static float line_error_filtered;
static float line_derivative_filtered;
static float tracking_target_rate;
static float tracking_base_speed;
static float tracking_previous_heading;
static float tracking_unwrapped_heading;
static float tracking_target_heading;
static float line_heading_rate;
static uint32_t line_direction_started_ms;
static int8_t line_direction;
static uint8_t tracking_heading_initialized;
static int8_t last_line_sign;
static uint8_t last_line_sign_valid;
static uint32_t line_lost_started_ms;
static uint8_t line_lost_active;
static uint8_t line_stop_latched;
static uint8_t enabled;
static uint8_t stopped;
static uint8_t heading_pid_initialized;
static uint8_t rate_pid_initialized;
static car_attitude_step_mode_t step_mode;
static uint8_t step_return_phase;
static uint32_t step_phase_started_ms;
static float step_start_heading;

static void clear_debug_output(void)
{
    uint32_t status = debug.status & (CAR_ATTITUDE_STATUS_ENABLED |
                                     CAR_ATTITUDE_STATUS_IMU_VALID |
                                     CAR_ATTITUDE_STATUS_TRACKING_ONLINE |
                                     CAR_ATTITUDE_STATUS_STEP_ACTIVE |
                                     CAR_ATTITUDE_STATUS_ANGLE_STEP |
                                     CAR_ATTITUDE_STATUS_LINE_SEARCH |
                                     CAR_ATTITUDE_STATUS_PATTERN_INVALID |
                                     CAR_ATTITUDE_STATUS_HEADING_TRACKING);
    memset(&debug, 0, sizeof(debug));
    debug.status = status;
}

static float clampf(float value, float minimum, float maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

static float wrap_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float apply_axis_direction(float value, float inverted)
{
    return inverted != 0.0f ? -value : value;
}

static void reset_controllers(void)
{
    PIDController_Init(&heading_pid);
    PIDController_Init(&rate_pid);
    memset(&rate_ladrc, 0, sizeof(rate_ladrc));
    heading_pid_initialized = 0U;
    rate_pid_initialized = 0U;
}

static void reset_tracking_state(void)
{
    line_error_previous = 0.0f;
    line_error_filtered = 0.0f;
    line_derivative_filtered = 0.0f;
    tracking_target_rate = 0.0f;
    tracking_base_speed = 0.0f;
    tracking_previous_heading = 0.0f;
    tracking_unwrapped_heading = 0.0f;
    tracking_target_heading = 0.0f;
    line_heading_rate = 0.0f;
    line_direction_started_ms = 0U;
    line_direction = 0;
    tracking_heading_initialized = 0U;
    last_line_sign = 0;
    last_line_sign_valid = 0U;
    line_lost_started_ms = 0U;
    line_lost_active = 0U;
    line_stop_latched = 0U;
}

static float slew_target_rate(float target)
{
    const float maximum_step = tracking_config.target_rate_slew_deg_s2 * CAR_CONTROL_PERIOD_S;
    tracking_target_rate += clampf(target - tracking_target_rate, -maximum_step, maximum_step);
    return tracking_target_rate;
}

static void configure_pid(void)
{
    heading_pid.Kp = config.heading_kp;
    heading_pid.Ki = config.heading_ki;
    heading_pid.Kd = config.heading_kd;
    heading_pid.tau = 0.02f;
    heading_pid.limMin = -config.max_rate;
    heading_pid.limMax = config.max_rate;
    heading_pid.limMinInt = -config.max_rate;
    heading_pid.limMaxInt = config.max_rate;
    heading_pid.T = CAR_CONTROL_PERIOD_S;

    rate_pid.Kp = config.rate_kp;
    rate_pid.Ki = config.rate_ki;
    rate_pid.Kd = config.rate_kd;
    rate_pid.tau = 0.02f;
    rate_pid.limMin = -config.max_differential;
    rate_pid.limMax = config.max_differential;
    rate_pid.limMinInt = -config.max_differential;
    rate_pid.limMaxInt = config.max_differential;
    rate_pid.T = CAR_CONTROL_PERIOD_S;
}

static void stop_output(void)
{
    if (!stopped)
    {
        if (Car_Motor_Set_Speed(0, 0, 0, 0)) stopped = 1U;
    }
}

static float update_ladrc(float target_rate, float measured_rate)
{
    const float error = measured_rate - rate_ladrc.z1;
    const float observer = config.ladrc_observer_bandwidth;
    const float gain = config.ladrc_plant_gain;

    if (!rate_ladrc.initialized)
    {
        rate_ladrc.z1 = measured_rate;
        rate_ladrc.initialized = 1U;
    }

    rate_ladrc.z1 += CAR_CONTROL_PERIOD_S *
                     (rate_ladrc.z2 + gain * rate_ladrc.previous_output + 2.0f * observer * error);
    rate_ladrc.z2 += CAR_CONTROL_PERIOD_S * observer * observer * error;
    rate_ladrc.previous_output =
        (config.ladrc_controller_bandwidth * (target_rate - rate_ladrc.z1) - rate_ladrc.z2) / gain;
    if (rate_ladrc.previous_output > config.max_differential ||
        rate_ladrc.previous_output < -config.max_differential)
    {
        debug.status |= CAR_ATTITUDE_STATUS_DIFFERENTIAL_LIMITED;
    }
    rate_ladrc.previous_output = clampf(rate_ladrc.previous_output,
                                       -config.max_differential, config.max_differential);
    return rate_ladrc.previous_output;
}

void Car_Attitude_Control_Reset_Defaults(void)
{
    config.mode = (float)CAR_ATTITUDE_CONTROL_PID;
    config.base_speed = 150.0f;
    config.heading_kp = 3.0f;
    config.heading_ki = 0.0f;
    config.heading_kd = 0.05f;
    config.rate_kp = 1.2f;
    config.rate_ki = 0.2f;
    config.rate_kd = 0.0f;
    config.ladrc_observer_bandwidth = 20.0f;
    config.ladrc_controller_bandwidth = 8.0f;
    config.ladrc_plant_gain = 1.0f;
    config.max_rate = 180.0f;
    config.max_differential = 300.0f;
    config.invert_x = 0.0f;
    config.invert_y = 0.0f;
    config.invert_z = 0.0f;
    config.step_rate_amplitude = 90.0f;
    config.step_angle_amplitude = 45.0f;
    config.step_hold_ms = 2000.0f;
    config.step_angle_feedback_inverted = 1.0f;
    config.rate_feedback_inverted = 1.0f;
    tracking_config.direction_inverted = 0.0f;
    tracking_config.error_filter_time_s = 0.10f;
    tracking_config.target_rate_slew_deg_s2 = 600.0f;
    tracking_config.line_initial_weight = 0.15f;
    tracking_config.line_weight_ramp_time_s = 0.50f;
    tracking_config.line_release_time_s = 0.15f;
    tracking_config.line_heading_rate_deg_s = 100.0f;
    tracking_config.heading_error_limit_deg = 25.0f;
    tracking_config.line_inner_position_weight = 0.35f;
    tracking_config.line_outer_position_weight = 1.0f;
    tracking_config.line_time_curve_exponent = 2.0f;
    tracking_config.optical_fusion_weight = 0.35f;
    tracking_config.imu_fusion_weight = 1.0f;
    configure_pid();
    reset_controllers();
    reset_tracking_state();
}

bool Car_Attitude_Control_Apply_Tracking_Config(const car_tracking_config_t *next)
{
    if (next == NULL || !isfinite(next->direction_inverted) ||
        !isfinite(next->error_filter_time_s) ||
        !isfinite(next->target_rate_slew_deg_s2) ||
        !isfinite(next->line_initial_weight) ||
        !isfinite(next->line_weight_ramp_time_s) ||
        !isfinite(next->line_release_time_s) ||
        !isfinite(next->line_heading_rate_deg_s) ||
        !isfinite(next->heading_error_limit_deg) ||
        !isfinite(next->line_inner_position_weight) ||
        !isfinite(next->line_outer_position_weight) ||
        !isfinite(next->line_time_curve_exponent) ||
        !isfinite(next->optical_fusion_weight) ||
        !isfinite(next->imu_fusion_weight) ||
        (next->direction_inverted != 0.0f && next->direction_inverted != 1.0f) ||
        next->error_filter_time_s < 0.0f || next->error_filter_time_s > 2.0f ||
        next->target_rate_slew_deg_s2 <= 0.0f || next->target_rate_slew_deg_s2 > 20000.0f ||
        next->line_initial_weight < 0.0f || next->line_initial_weight > 1.0f ||
        next->line_weight_ramp_time_s < 0.01f || next->line_weight_ramp_time_s > 5.0f ||
        next->line_release_time_s < 0.0f || next->line_release_time_s > 2.0f ||
        next->line_heading_rate_deg_s <= 0.0f || next->line_heading_rate_deg_s > 720.0f ||
        next->heading_error_limit_deg <= 0.0f || next->heading_error_limit_deg > 90.0f ||
        next->line_inner_position_weight < 0.0f || next->line_inner_position_weight > 2.0f ||
        next->line_outer_position_weight < 0.01f || next->line_outer_position_weight > 2.0f ||
        next->line_inner_position_weight > next->line_outer_position_weight ||
        next->line_time_curve_exponent < 0.25f || next->line_time_curve_exponent > 8.0f ||
        next->optical_fusion_weight < 0.0f || next->optical_fusion_weight > 2.0f ||
        next->imu_fusion_weight < 0.0f || next->imu_fusion_weight > 2.0f)
    {
        return false;
    }
    tracking_config = *next;
    reset_tracking_state();
    return true;
}

bool Car_Attitude_Control_Apply_Config(const car_attitude_control_config_t *next)
{
    if (next == NULL || !isfinite(next->mode) || !isfinite(next->base_speed) ||
        !isfinite(next->heading_kp) || !isfinite(next->heading_ki) || !isfinite(next->heading_kd) ||
        !isfinite(next->rate_kp) || !isfinite(next->rate_ki) || !isfinite(next->rate_kd) ||
        !isfinite(next->ladrc_observer_bandwidth) || !isfinite(next->ladrc_controller_bandwidth) ||
        !isfinite(next->ladrc_plant_gain) || !isfinite(next->max_rate) || !isfinite(next->max_differential) ||
        !isfinite(next->step_rate_amplitude) || !isfinite(next->step_angle_amplitude) ||
        !isfinite(next->step_hold_ms) || !isfinite(next->step_angle_feedback_inverted) ||
        !isfinite(next->rate_feedback_inverted) ||
        (next->mode != 0.0f && next->mode != 1.0f) || next->base_speed < 0.0f || next->base_speed > 1000.0f ||
        next->heading_kp < 0.0f || next->heading_ki < 0.0f || next->heading_kd < 0.0f ||
        next->rate_kp < 0.0f || next->rate_ki < 0.0f || next->rate_kd < 0.0f ||
        next->ladrc_observer_bandwidth <= 0.0f || next->ladrc_controller_bandwidth <= 0.0f ||
        next->ladrc_plant_gain <= 0.0f || next->max_rate <= 0.0f || next->max_rate > 2000.0f ||
        next->max_differential <= 0.0f || next->max_differential > 1000.0f ||
        (next->invert_x != 0.0f && next->invert_x != 1.0f) ||
        (next->invert_y != 0.0f && next->invert_y != 1.0f) ||
        (next->invert_z != 0.0f && next->invert_z != 1.0f) ||
        next->step_rate_amplitude < -2000.0f || next->step_rate_amplitude > 2000.0f ||
        next->step_angle_amplitude < -180.0f || next->step_angle_amplitude > 180.0f ||
        next->step_hold_ms < 100.0f || next->step_hold_ms > 30000.0f ||
        next->step_hold_ms != floorf(next->step_hold_ms) ||
        (next->step_angle_feedback_inverted != 0.0f && next->step_angle_feedback_inverted != 1.0f) ||
        (next->rate_feedback_inverted != 0.0f && next->rate_feedback_inverted != 1.0f))
    {
        return false;
    }

    config = *next;
    configure_pid();
    reset_controllers();
    reset_tracking_state();
    return true;
}

void Car_Attitude_Control_Init(void)
{
    enabled = 0U;
    stopped = 0U;
    last_sequence = 0U;
    last_sample_ms = HAL_GetTick();
    step_mode = CAR_ATTITUDE_STEP_NONE;
    last_line_sign = -1;
    reset_tracking_state();
    memset(&debug, 0, sizeof(debug));
    Car_Attitude_Control_Reset_Defaults();
    stop_output();
}

bool Car_Attitude_Control_Set_Enabled(bool state)
{
    step_mode = CAR_ATTITUDE_STEP_NONE;
    enabled = state ? 1U : 0U;
    stopped = 0U;
    reset_controllers();
    reset_tracking_state();
    debug.status = enabled ? CAR_ATTITUDE_STATUS_ENABLED : 0U;
    clear_debug_output();
    if (!enabled) stop_output();
    return true;
}

static bool update_tracking_target(uint32_t now, float measured_heading)
{
    const uint8_t sensor_state = Gray_Sensor_Get_Stable() & 0x0FU;
    const float position_weight[4] = {
        -tracking_config.line_outer_position_weight,
        -tracking_config.line_inner_position_weight,
        tracking_config.line_inner_position_weight,
        tracking_config.line_outer_position_weight
    };
    float weighted_position = 0.0f;
    uint8_t active_count = 0U;

    if (tracking_heading_initialized == 0U)
    {
        tracking_previous_heading = measured_heading;
        tracking_unwrapped_heading = 0.0f;
        tracking_target_heading = 0.0f;
        tracking_heading_initialized = 1U;
    }
    else
    {
        tracking_unwrapped_heading += wrap_angle(measured_heading - tracking_previous_heading);
        tracking_previous_heading = measured_heading;
    }

    debug.status |= CAR_ATTITUDE_STATUS_TRACKING_ONLINE | CAR_ATTITUDE_STATUS_HEADING_TRACKING;
    if (sensor_state == 0x0FU)
    {
        line_lost_active = 0U;
        line_stop_latched = 0U;
        line_error_filtered += CAR_CONTROL_PERIOD_S /
                               (tracking_config.error_filter_time_s + CAR_CONTROL_PERIOD_S) *
                               -line_error_filtered;
        line_error_previous = line_error_filtered;
        line_derivative_filtered = 0.0f;
        line_direction = 0;
        line_direction_started_ms = 0U;
        line_heading_rate += CAR_CONTROL_PERIOD_S /
                             (tracking_config.line_release_time_s + CAR_CONTROL_PERIOD_S) *
                             -line_heading_rate;
        tracking_target_heading += line_heading_rate * CAR_CONTROL_PERIOD_S;
        tracking_target_heading = clampf(tracking_target_heading,
                                         tracking_unwrapped_heading - tracking_config.heading_error_limit_deg,
                                         tracking_unwrapped_heading + tracking_config.heading_error_limit_deg);
        tracking_base_speed = config.base_speed;
        debug.line_offset = line_error_filtered;
        debug.line_weight = 0.0f;
        debug.line_heading_rate = line_heading_rate;
        debug.line_position_weight = 0.0f;
        debug.line_time_weight = 0.0f;
        debug.imu_fusion_weight = tracking_config.imu_fusion_weight;
        return true;
    }
    if (sensor_state == 0U)
    {
        if (line_stop_latched != 0U) return false;
        if (line_lost_active == 0U)
        {
            line_lost_active = 1U;
            line_lost_started_ms = now;
            line_derivative_filtered = 0.0f;
            PIDController_Init(&rate_pid);
            rate_pid_initialized = 0U;
        }
        if (last_line_sign_valid != 0U &&
            (uint32_t)(now - line_lost_started_ms) <= CAR_CONTROL_LOST_STOP_MS)
        {
            const float direction = tracking_config.direction_inverted != 0.0f ? -1.0f : 1.0f;
            const float desired_rate = direction * (float)last_line_sign *
                                       tracking_config.line_heading_rate_deg_s *
                                       CAR_CONTROL_SEARCH_RATE_SCALE *
                                       tracking_config.optical_fusion_weight;
            line_heading_rate += CAR_CONTROL_PERIOD_S /
                                 (tracking_config.line_release_time_s + CAR_CONTROL_PERIOD_S) *
                                 (desired_rate - line_heading_rate);
            tracking_target_heading += line_heading_rate * CAR_CONTROL_PERIOD_S;
            tracking_base_speed = config.base_speed * CAR_CONTROL_SEARCH_SPEED_SCALE;
            debug.line_offset = line_error_filtered;
            debug.line_weight = CAR_CONTROL_SEARCH_RATE_SCALE *
                                tracking_config.optical_fusion_weight;
            debug.line_heading_rate = line_heading_rate;
            debug.line_position_weight = (float)last_line_sign;
            debug.line_time_weight = CAR_CONTROL_SEARCH_RATE_SCALE;
            debug.imu_fusion_weight = tracking_config.imu_fusion_weight;
            debug.status |= CAR_ATTITUDE_STATUS_LINE_SEARCH;
            return true;
        }
        if (last_line_sign_valid != 0U)
        {
            line_stop_latched = 1U;
            return false;
        }
        tracking_base_speed = config.base_speed;
        debug.line_offset = 0.0f;
        debug.line_weight = 0.0f;
        debug.line_heading_rate = 0.0f;
        debug.line_position_weight = 0.0f;
        debug.line_time_weight = 0.0f;
        debug.imu_fusion_weight = tracking_config.imu_fusion_weight;
        return true;
    }

    for (uint8_t bit = 0U; bit < 4U; bit++)
    {
        if ((sensor_state & (1U << bit)) != 0U)
        {
            weighted_position += position_weight[bit];
            active_count++;
        }
    }
    const float raw_error = weighted_position /
                            (tracking_config.line_outer_position_weight * (float)active_count);
    const float filter_alpha = CAR_CONTROL_PERIOD_S /
                               (tracking_config.error_filter_time_s + CAR_CONTROL_PERIOD_S);
    line_error_filtered += filter_alpha * (raw_error - line_error_filtered);
    const float error = line_error_filtered;
    const float derivative = line_lost_active != 0U ? 0.0f :
                             (error - line_error_previous) / CAR_CONTROL_LINE_PERIOD_S;
    const float magnitude = fabsf(error);
    const int8_t current_direction = magnitude > 0.02f ? (error < 0.0f ? -1 : 1) : 0;

    if (line_lost_active != 0U)
    {
        line_derivative_filtered = 0.0f;
        rate_pid_initialized = 0U;
        PIDController_Init(&rate_pid);
    }
    line_lost_active = 0U;
    line_stop_latched = 0U;
    if (magnitude > 0.02f)
    {
        last_line_sign = error < 0.0f ? -1 : 1;
        last_line_sign_valid = 1U;
    }
    line_derivative_filtered += CAR_CONTROL_LINE_D_FILTER *
                                (derivative - line_derivative_filtered);
    line_error_previous = error;
    const float direction = tracking_config.direction_inverted != 0.0f ? -1.0f : 1.0f;
    if (current_direction != 0 && current_direction != line_direction)
    {
        line_direction = current_direction;
        line_direction_started_ms = now;
    }
    else if (current_direction == 0)
    {
        line_direction = 0;
        line_direction_started_ms = 0U;
    }
    float line_weight = 0.0f;
    float time_weight = 0.0f;
    if (current_direction != 0)
    {
        const float held_s = (float)(now - line_direction_started_ms) * 0.001f;
        const float ramp = clampf(held_s / tracking_config.line_weight_ramp_time_s, 0.0f, 1.0f);
        time_weight = powf(ramp, tracking_config.line_time_curve_exponent);
        line_weight = tracking_config.line_initial_weight +
                      (1.0f - tracking_config.line_initial_weight) * time_weight;
    }
    const float fused_line_weight = line_weight * tracking_config.optical_fusion_weight;
    const float desired_heading_rate = direction * error * fused_line_weight *
                                       tracking_config.line_heading_rate_deg_s;
    line_heading_rate += CAR_CONTROL_PERIOD_S /
                         (tracking_config.line_release_time_s + CAR_CONTROL_PERIOD_S) *
                         (desired_heading_rate - line_heading_rate);
    tracking_target_heading += line_heading_rate * CAR_CONTROL_PERIOD_S;
    tracking_target_heading = clampf(tracking_target_heading,
                                     tracking_unwrapped_heading - tracking_config.heading_error_limit_deg,
                                     tracking_unwrapped_heading + tracking_config.heading_error_limit_deg);
    tracking_base_speed = config.base_speed * clampf(
        1.0f - CAR_CONTROL_CURVE_SPEED_REDUCTION * magnitude -
        0.20f * fabsf(line_heading_rate) / config.max_rate,
        CAR_CONTROL_MIN_SPEED_SCALE, 1.0f);
    debug.line_offset = error;
    debug.line_weight = fused_line_weight;
    debug.line_heading_rate = line_heading_rate;
    debug.line_position_weight = raw_error;
    debug.line_time_weight = time_weight;
    debug.imu_fusion_weight = tracking_config.imu_fusion_weight;
    return true;
}

bool Car_Attitude_Control_Start_Step(car_attitude_step_mode_t mode)
{
    car_imu_sample_t imu;

    if ((mode != CAR_ATTITUDE_STEP_RATE && mode != CAR_ATTITUDE_STEP_ANGLE) ||
        !Car_IMU_Get_Sample(&imu))
    {
        return false;
    }
    step_mode = mode;
    step_return_phase = 0U;
    step_phase_started_ms = HAL_GetTick();
    step_start_heading = apply_axis_direction(imu.angle_rad.z, config.invert_z) * CAR_CONTROL_RAD_TO_DEG;
    last_sequence = imu.sequence;
    last_sample_ms = step_phase_started_ms;
    enabled = 1U;
    stopped = 0U;
    reset_controllers();
    debug.status = CAR_ATTITUDE_STATUS_ENABLED | CAR_ATTITUDE_STATUS_STEP_ACTIVE;
    if (mode == CAR_ATTITUDE_STEP_ANGLE) debug.status |= CAR_ATTITUDE_STATUS_ANGLE_STEP;
    clear_debug_output();
    stop_output();
    return true;
}

void Car_Attitude_Control_Cancel_Step(void)
{
    step_mode = CAR_ATTITUDE_STEP_NONE;
    enabled = 0U;
    stopped = 0U;
    reset_controllers();
    debug.status = 0U;
    clear_debug_output();
    stop_output();
}

bool Car_Attitude_Control_Is_Enabled(void)
{
    return enabled != 0U;
}

car_attitude_control_config_t *Car_Attitude_Control_Get_Config(void)
{
    return &config;
}

car_tracking_config_t *Car_Attitude_Control_Get_Tracking_Config(void)
{
    return &tracking_config;
}

const car_attitude_control_debug_t *Car_Attitude_Control_Get_Debug(void)
{
    return &debug;
}

void Car_Attitude_Control_Process(void)
{
    car_imu_sample_t imu;
    uint32_t now = HAL_GetTick();

    bool sample_valid = Car_IMU_Get_Sample(&imu);
    bool new_sample = sample_valid && imu.sequence != last_sequence;

    debug.status = 0U;
    if (enabled) debug.status |= CAR_ATTITUDE_STATUS_ENABLED;
    if (sample_valid && (uint32_t)(now - last_sample_ms) <= CAR_CONTROL_IMU_TIMEOUT_MS)
    {
        debug.status |= CAR_ATTITUDE_STATUS_IMU_VALID;
    }
    if (Gray_Sensor_Get_Stable() != 0U) debug.status |= CAR_ATTITUDE_STATUS_TRACKING_ONLINE;
    if (step_mode != CAR_ATTITUDE_STEP_NONE) debug.status |= CAR_ATTITUDE_STATUS_STEP_ACTIVE;
    if (step_mode == CAR_ATTITUDE_STEP_ANGLE) debug.status |= CAR_ATTITUDE_STATUS_ANGLE_STEP;

    if (!enabled)
    {
        clear_debug_output();
        stop_output();
        return;
    }
    if (new_sample)
    {
        last_sequence = imu.sequence;
        last_sample_ms = now;
        debug.status |= CAR_ATTITUDE_STATUS_IMU_VALID;
    }
    else if ((uint32_t)(now - last_sample_ms) > CAR_CONTROL_IMU_TIMEOUT_MS)
    {
        if (step_mode != CAR_ATTITUDE_STEP_NONE) Car_Attitude_Control_Cancel_Step();
        else
        {
            clear_debug_output();
            stop_output();
            reset_controllers();
        }
        return;
    }
    else
    {
        return;
    }
    clear_debug_output();
    const float measured_heading = apply_axis_direction(imu.angle_rad.z, config.invert_z) * CAR_CONTROL_RAD_TO_DEG;
    const float measured_rate = apply_axis_direction(imu.angular_velocity_rad_s.z,
                                                     config.rate_feedback_inverted) * CAR_CONTROL_RAD_TO_DEG;
    if (step_mode == CAR_ATTITUDE_STEP_NONE && !update_tracking_target(now, measured_heading))
    {
        stop_output();
        reset_controllers();
        return;
    }
    float target_heading = 0.0f;
    float heading_error = 0.0f;

    if (step_mode != CAR_ATTITUDE_STEP_NONE &&
        (uint32_t)(now - step_phase_started_ms) >= (uint32_t)config.step_hold_ms)
    {
        if (!step_return_phase)
        {
            step_return_phase = 1U;
            step_phase_started_ms = now;
            reset_controllers();
        }
        else
        {
            Car_Attitude_Control_Cancel_Step();
            clear_debug_output();
            return;
        }
    }
    if (step_mode == CAR_ATTITUDE_STEP_ANGLE)
    {
        target_heading = step_return_phase ? 0.0f : config.step_angle_amplitude;
        float relative_heading = wrap_angle(measured_heading - step_start_heading);
        if (config.step_angle_feedback_inverted != 0.0f) relative_heading = -relative_heading;
        heading_error = wrap_angle(relative_heading - target_heading);
        debug.tracking_angle = relative_heading;
    }
    else if (step_mode == CAR_ATTITUDE_STEP_NONE)
    {
        target_heading = tracking_target_heading;
        heading_error = tracking_unwrapped_heading - target_heading;
        debug.tracking_angle = tracking_unwrapped_heading;
    }
    else if (step_mode != CAR_ATTITUDE_STEP_NONE)
    {
        debug.tracking_angle = heading_error;
    }
    debug.target_heading = target_heading;
    debug.measured_rate = measured_rate;
    if (!heading_pid_initialized)
    {
        heading_pid.prevMeasurement = heading_error;
        heading_pid_initialized = 1U;
    }
    float target_rate;
    if (step_mode == CAR_ATTITUDE_STEP_NONE)
    {
        const float fused_heading_error = tracking_config.imu_fusion_weight * heading_error;
        target_rate = slew_target_rate(PIDController_Update(&heading_pid, 0.0f,
                                                            fused_heading_error));
        debug.heading_p = heading_pid.Kp * -fused_heading_error;
        debug.heading_i = heading_pid.integrator;
        debug.heading_d = heading_pid.differentiator;
    }
    else if (step_mode == CAR_ATTITUDE_STEP_RATE)
    {
        target_rate = step_return_phase ? 0.0f : config.step_rate_amplitude;
    }
    else
    {
        target_rate = PIDController_Update(&heading_pid, 0.0f, heading_error);
        debug.heading_p = heading_pid.Kp * -heading_error;
        debug.heading_i = heading_pid.integrator;
        debug.heading_d = heading_pid.differentiator;
    }
    if (target_rate > config.max_rate || target_rate < -config.max_rate)
    {
        debug.status |= CAR_ATTITUDE_STATUS_TARGET_RATE_LIMITED;
    }
    target_rate = clampf(target_rate, -config.max_rate, config.max_rate);
    debug.target_rate = target_rate;
    if (!rate_pid_initialized)
    {
        rate_pid.prevMeasurement = measured_rate;
        rate_pid_initialized = 1U;
    }
    float differential;
    if (config.mode == (float)CAR_ATTITUDE_CONTROL_LADRC)
    {
        differential = update_ladrc(target_rate, measured_rate);
        debug.ladrc_z1 = rate_ladrc.z1;
        debug.ladrc_z2 = rate_ladrc.z2;
        debug.ladrc_output = differential;
    }
    else
    {
        differential = PIDController_Update(&rate_pid, target_rate, measured_rate);
        debug.rate_p = rate_pid.Kp * (target_rate - measured_rate);
        debug.rate_i = rate_pid.integrator;
        debug.rate_d = rate_pid.differentiator;
        if (debug.rate_p + debug.rate_i + debug.rate_d > config.max_differential ||
            debug.rate_p + debug.rate_i + debug.rate_d < -config.max_differential)
        {
            debug.status |= CAR_ATTITUDE_STATUS_DIFFERENTIAL_LIMITED;
        }
    }
    differential = clampf(differential, -config.max_differential, config.max_differential);

    const float base_speed = step_mode == CAR_ATTITUDE_STEP_NONE ? tracking_base_speed : 0.0f;
    debug.commanded_base_speed = base_speed;
    const float left_unclamped = base_speed - differential;
    const float right_unclamped = base_speed + differential;
    const int16_t left = (int16_t)clampf(left_unclamped, -1000.0f, 1000.0f);
    const int16_t right = (int16_t)clampf(right_unclamped, -1000.0f, 1000.0f);
    if (left_unclamped > 1000.0f || left_unclamped < -1000.0f)
    {
        debug.status |= CAR_ATTITUDE_STATUS_LEFT_SPEED_LIMITED;
    }
    if (right_unclamped > 1000.0f || right_unclamped < -1000.0f)
    {
        debug.status |= CAR_ATTITUDE_STATUS_RIGHT_SPEED_LIMITED;
    }
    debug.differential = differential;
    debug.left_target = (float)left;
    debug.right_target = (float)right;
    if (Car_Motor_Set_Speed(left, right, 0, 0)) stopped = 0U;
}
