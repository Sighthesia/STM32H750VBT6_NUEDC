#include "Car_Attitude_Control.h"

#include "Car_IMU.h"
#include "Car_Lap_Task.h"
#include "Car_Motor_USART.h"
#include "Car_Tracking_Model.h"
#include "Gray_Sensor.h"
#include "PID.h"
#include "main.h"

#include <math.h>
#include <string.h>

#define CAR_CONTROL_PERIOD_S 0.01f
#define CAR_CONTROL_TRACKING_LOOKAHEAD_S 0.05f
#define CAR_CONTROL_IMU_TIMEOUT_MS 50U
#define CAR_CONTROL_RAD_TO_DEG 57.2957795f

static car_attitude_control_config_t config;
static car_tracking_config_t tracking_config;
static car_attitude_control_debug_t debug;
static PIDController heading_pid;
static PIDController rate_pid;
static uint32_t last_sequence;
static uint32_t last_sample_ms;
static uint32_t last_open_loop_ms;
static uint32_t enabled_ms;
static uint32_t tracking_lost_since_ms;
static float previous_heading;
static float unwrapped_heading;
static float target_heading;
static float arc_target_rate;
static float arc_commanded_rate;
static float commanded_differential;
static float last_tracking_command;
static uint8_t heading_locked;
static uint8_t enabled;
static uint8_t stopped;
static uint8_t heading_pid_initialized;
static uint8_t rate_pid_initialized;

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
    heading_pid_initialized = 0U;
    rate_pid_initialized = 0U;
    commanded_differential = 0.0f;
}

static void reset_tracking_state(void)
{
    previous_heading = 0.0f;
    unwrapped_heading = 0.0f;
    target_heading = 0.0f;
    arc_target_rate = 0.0f;
    arc_commanded_rate = 0.0f;
    last_tracking_command = 0.0f;
    tracking_lost_since_ms = 0U;
    heading_locked = 0U;
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
    if (!stopped && Car_Motor_Set_Speed(0, 0, 0, 0)) stopped = 1U;
}

void Car_Attitude_Control_Reset_Defaults(void)
{
    config.base_speed = 150.0f;
    config.start_ramp_ms = 600.0f;
    config.heading_kp = 3.0f;
    config.heading_ki = 0.0f;
    config.heading_kd = 0.05f;
    config.rate_kp = 1.2f;
    config.rate_ki = 0.2f;
    config.rate_kd = 0.0f;
    config.max_rate = 180.0f;
    config.max_differential = 300.0f;
    config.invert_x = 0.0f;
    config.invert_y = 0.0f;
    config.invert_z = 0.0f;
    config.rate_feedback_inverted = 1.0f;
    tracking_config.method = (float)CAR_TRACKING_METHOD_OPTICAL_IMU;
    tracking_config.layer1_weight_deg_s = 7.0f;
    tracking_config.layer2_weight_deg_s = 25.0f;
    tracking_config.layer3_weight_deg_s = 55.0f;
    tracking_config.layer4_weight_deg_s = 101.0f;
    tracking_config.angle_layer1_deg = 0.35f;
    tracking_config.angle_layer2_deg = 1.25f;
    tracking_config.angle_layer3_deg = 2.75f;
    tracking_config.angle_layer4_deg = 5.05f;
    tracking_config.rate_layer1_deg_s = 7.0f;
    tracking_config.rate_layer2_deg_s = 25.0f;
    tracking_config.rate_layer3_deg_s = 55.0f;
    tracking_config.rate_layer4_deg_s = 101.0f;
    tracking_config.open_loop_layer1 = 7.0f;
    tracking_config.open_loop_layer2 = 25.0f;
    tracking_config.open_loop_layer3 = 60.0f;
    tracking_config.open_loop_layer4 = 101.0f;
    tracking_config.lost_strategy = (float)CAR_TRACKING_LOST_HOLD_THEN_SLOW;
    tracking_config.lost_hold_ms = 150.0f;
    tracking_config.lost_speed_ratio = 0.5f;
    configure_pid();
    reset_controllers();
    reset_tracking_state();
}

bool Car_Attitude_Control_Apply_Tracking_Config(const car_tracking_config_t *next)
{
    if (next == NULL || !isfinite(next->method) || !isfinite(next->layer1_weight_deg_s) || !isfinite(next->layer2_weight_deg_s) ||
        !isfinite(next->layer3_weight_deg_s) || !isfinite(next->layer4_weight_deg_s) ||
        next->method < (float)CAR_TRACKING_METHOD_OPTICAL_IMU ||
        !isfinite(next->angle_layer1_deg) || !isfinite(next->angle_layer2_deg) ||
        !isfinite(next->angle_layer3_deg) || !isfinite(next->angle_layer4_deg) ||
        !isfinite(next->rate_layer1_deg_s) || !isfinite(next->rate_layer2_deg_s) ||
        !isfinite(next->rate_layer3_deg_s) || !isfinite(next->rate_layer4_deg_s) ||
        !isfinite(next->open_loop_layer1) || !isfinite(next->open_loop_layer2) ||
        !isfinite(next->open_loop_layer3) || !isfinite(next->open_loop_layer4) ||
        !isfinite(next->lost_strategy) || !isfinite(next->lost_hold_ms) ||
        !isfinite(next->lost_speed_ratio) ||
        next->method > (float)CAR_TRACKING_METHOD_OPEN_LOOP || next->method != floorf(next->method) ||
        next->layer1_weight_deg_s < 0.0f || next->layer1_weight_deg_s > 720.0f ||
        next->layer2_weight_deg_s < 0.0f || next->layer2_weight_deg_s > 720.0f ||
        next->layer3_weight_deg_s < 0.0f || next->layer3_weight_deg_s > 720.0f ||
        next->layer4_weight_deg_s < 0.0f || next->layer4_weight_deg_s > 720.0f ||
        next->angle_layer1_deg < 0.0f || next->angle_layer1_deg > 180.0f ||
        next->angle_layer2_deg < 0.0f || next->angle_layer2_deg > 180.0f ||
        next->angle_layer3_deg < 0.0f || next->angle_layer3_deg > 180.0f ||
        next->angle_layer4_deg < 0.0f || next->angle_layer4_deg > 180.0f ||
        next->rate_layer1_deg_s < 0.0f || next->rate_layer1_deg_s > 720.0f ||
        next->rate_layer2_deg_s < 0.0f || next->rate_layer2_deg_s > 720.0f ||
        next->rate_layer3_deg_s < 0.0f || next->rate_layer3_deg_s > 720.0f ||
        next->rate_layer4_deg_s < 0.0f || next->rate_layer4_deg_s > 720.0f ||
        next->open_loop_layer1 < 0.0f || next->open_loop_layer1 > 1000.0f ||
        next->open_loop_layer2 < 0.0f || next->open_loop_layer2 > 1000.0f ||
        next->open_loop_layer3 < 0.0f || next->open_loop_layer3 > 1000.0f ||
        next->open_loop_layer4 < 0.0f || next->open_loop_layer4 > 1000.0f ||
        next->lost_strategy < (float)CAR_TRACKING_LOST_HOLD_THEN_SLOW ||
        next->lost_strategy > (float)CAR_TRACKING_LOST_STRAIGHT_SLOW ||
        next->lost_strategy != floorf(next->lost_strategy) ||
        next->lost_hold_ms < 0.0f || next->lost_hold_ms > 5000.0f ||
        next->lost_hold_ms != floorf(next->lost_hold_ms) ||
        next->lost_speed_ratio < 0.0f || next->lost_speed_ratio > 1.0f)
    {
        return false;
    }
    tracking_config = *next;
    reset_controllers();
    reset_tracking_state();
    return true;
}

bool Car_Attitude_Control_Apply_Config(const car_attitude_control_config_t *next)
{
    if (next == NULL || !isfinite(next->base_speed) || !isfinite(next->start_ramp_ms) ||
        !isfinite(next->heading_kp) || !isfinite(next->heading_ki) || !isfinite(next->heading_kd) ||
        !isfinite(next->rate_kp) || !isfinite(next->rate_ki) || !isfinite(next->rate_kd) ||
        !isfinite(next->max_rate) || !isfinite(next->max_differential) ||
        !isfinite(next->invert_x) || !isfinite(next->invert_y) || !isfinite(next->invert_z) ||
        !isfinite(next->rate_feedback_inverted) || next->base_speed < 0.0f || next->base_speed > 1000.0f ||
        next->start_ramp_ms < 0.0f || next->start_ramp_ms > 10000.0f ||
        next->heading_kp < 0.0f || next->heading_ki < 0.0f || next->heading_kd < 0.0f ||
        next->rate_kp < 0.0f || next->rate_ki < 0.0f || next->rate_kd < 0.0f || next->max_rate <= 0.0f ||
        next->max_rate > 2000.0f || next->max_differential <= 0.0f || next->max_differential > 1000.0f ||
        (next->invert_x != 0.0f && next->invert_x != 1.0f) ||
        (next->invert_y != 0.0f && next->invert_y != 1.0f) ||
        (next->invert_z != 0.0f && next->invert_z != 1.0f) ||
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
    last_open_loop_ms = last_sample_ms;
    enabled_ms = last_sample_ms;
    memset(&debug, 0, sizeof(debug));
    Car_Attitude_Control_Reset_Defaults();
    stop_output();
}

bool Car_Attitude_Control_Set_Enabled(bool state)
{
    enabled = state ? 1U : 0U;
    stopped = 0U;
    enabled_ms = HAL_GetTick();
    last_open_loop_ms = enabled_ms;
    reset_controllers();
    reset_tracking_state();
    memset(&debug, 0, sizeof(debug));
    if (!enabled) stop_output();
    return true;
}

bool Car_Attitude_Control_Set_Arc_Rate(float target_rate_deg_s)
{
    if ((car_tracking_method_t)(uint8_t)tracking_config.method != CAR_TRACKING_METHOD_GEOMETRY)
    {
        arc_target_rate = 0.0f;
        arc_commanded_rate = 0.0f;
        return true;
    }
    if (!isfinite(target_rate_deg_s) || target_rate_deg_s < -config.max_rate || target_rate_deg_s > config.max_rate)
    {
        return false;
    }
    arc_target_rate = target_rate_deg_s;
    return true;
}

bool Car_Attitude_Control_Start_Step(car_attitude_step_mode_t mode)
{
    (void)mode;
    return false;
}

bool Car_Attitude_Control_Start_Rotation(float relative_angle_deg)
{
    (void)relative_angle_deg;
    return false;
}

void Car_Attitude_Control_Cancel_Step(void)
{
    (void)Car_Attitude_Control_Set_Enabled(false);
}

bool Car_Attitude_Control_Is_Enabled(void) { return enabled != 0U; }
car_attitude_control_config_t *Car_Attitude_Control_Get_Config(void) { return &config; }
car_tracking_config_t *Car_Attitude_Control_Get_Tracking_Config(void) { return &tracking_config; }
const car_attitude_control_debug_t *Car_Attitude_Control_Get_Debug(void) { return &debug; }

static float tracking_direct_command(uint8_t sensor_state, uint8_t method)
{
    if (method == (uint8_t)CAR_TRACKING_METHOD_DIRECT_ANGLE)
        return Car_Tracking_Direct_Command(sensor_state, tracking_config.angle_layer1_deg,
            tracking_config.angle_layer2_deg, tracking_config.angle_layer3_deg, tracking_config.angle_layer4_deg);
    if (method == (uint8_t)CAR_TRACKING_METHOD_DIRECT_RATE)
        return Car_Tracking_Direct_Command(sensor_state, tracking_config.rate_layer1_deg_s,
            tracking_config.rate_layer2_deg_s, tracking_config.rate_layer3_deg_s, tracking_config.rate_layer4_deg_s);
    return Car_Tracking_Direct_Command(sensor_state, tracking_config.open_loop_layer1,
        tracking_config.open_loop_layer2, tracking_config.open_loop_layer3, tracking_config.open_loop_layer4);
}

static float update_direct_tracking(uint8_t sensor_state, uint8_t method, uint32_t now, float *speed_scale)
{
    if (sensor_state != 0U)
    {
        last_tracking_command = tracking_direct_command(sensor_state, method);
        tracking_lost_since_ms = 0U;
        *speed_scale = 1.0f;
        return last_tracking_command;
    }

    if (tracking_lost_since_ms == 0U) tracking_lost_since_ms = now;
    const uint32_t elapsed_ms = now - tracking_lost_since_ms;
    const uint8_t strategy = (uint8_t)tracking_config.lost_strategy;
    *speed_scale = Car_Tracking_Lost_Speed_Scale(strategy, elapsed_ms,
        (uint32_t)tracking_config.lost_hold_ms, tracking_config.lost_speed_ratio);
    return Car_Tracking_Lost_Command(last_tracking_command, strategy, elapsed_ms,
                                     (uint32_t)tracking_config.lost_hold_ms);
}

void Car_Attitude_Control_Process(void)
{
    car_imu_sample_t imu;
    const uint32_t now = HAL_GetTick();
    const uint8_t tracking_method = (uint8_t)tracking_config.method;
    const uint8_t sensor_state = Gray_Sensor_Get_Stable();
    const bool sample_valid = Car_IMU_Get_Sample(&imu);
    const bool new_sample = sample_valid && imu.sequence != last_sequence;

    if (!enabled)
    {
        memset(&debug, 0, sizeof(debug));
        stop_output();
        return;
    }
    if (tracking_method == (uint8_t)CAR_TRACKING_METHOD_OPEN_LOOP)
    {
        if ((uint32_t)(now - last_open_loop_ms) < 10U) return;
        last_open_loop_ms = now;
        memset(&debug, 0, sizeof(debug));
        debug.status = CAR_ATTITUDE_STATUS_ENABLED | CAR_ATTITUDE_STATUS_OPEN_LOOP;
        if (sensor_state != 0U) debug.status |= CAR_ATTITUDE_STATUS_TRACKING_ONLINE;
        if (new_sample)
        {
            last_sequence = imu.sequence;
            last_sample_ms = now;
            const float heading = apply_axis_direction(imu.angle_rad.z, config.invert_z) * CAR_CONTROL_RAD_TO_DEG;
            debug.measured_rate = apply_axis_direction(imu.angular_velocity_rad_s.z,
                                                       config.rate_feedback_inverted) * CAR_CONTROL_RAD_TO_DEG;
            if (!heading_locked)
            {
                previous_heading = heading;
                heading_locked = 1U;
            }
            else
            {
                unwrapped_heading += wrap_angle(heading - previous_heading);
                previous_heading = heading;
            }
        }
        if (sample_valid && (uint32_t)(now - last_sample_ms) <= CAR_CONTROL_IMU_TIMEOUT_MS)
            debug.status |= CAR_ATTITUDE_STATUS_IMU_VALID;
        if (heading_locked) debug.status |= CAR_ATTITUDE_STATUS_HEADING_LOCKED;
        float speed_scale;
        const float tracking_command = update_direct_tracking(sensor_state, tracking_method, now, &speed_scale);
        const float differential = clampf(tracking_command, -config.max_differential, config.max_differential);
        const float step = Car_Lap_Task_Get_Arc_Config()->differential_slew_s2 * CAR_CONTROL_PERIOD_S;
        const float limited = Car_Tracking_Approach(commanded_differential, differential, step);
        if (limited != differential) debug.status |= CAR_ATTITUDE_STATUS_DIFFERENTIAL_SLEW_LIMITED;
        if (sensor_state == 0U) debug.status |= CAR_ATTITUDE_STATUS_TRACKING_LOST;
        commanded_differential = limited;
        const float ramp = config.start_ramp_ms <= 0.0f ? 1.0f :
            clampf((float)(now - enabled_ms) / config.start_ramp_ms, 0.0f, 1.0f);
        const float base_speed = config.base_speed * ramp * speed_scale;
        const int16_t left = (int16_t)clampf(base_speed - limited, -1000.0f, 1000.0f);
        const int16_t right = (int16_t)clampf(base_speed + limited, -1000.0f, 1000.0f);
        debug.commanded_base_speed = base_speed;
        debug.tracking_angle = unwrapped_heading;
        debug.target_heading = unwrapped_heading;
        debug.differential = limited;
        debug.left_target = (float)left;
        debug.right_target = (float)right;
        debug.tracking_command = tracking_command;
        debug.tracking_speed_scale = speed_scale;
        debug.tracking_lost_elapsed_ms = tracking_lost_since_ms == 0U ? 0.0f : (float)(now - tracking_lost_since_ms);
        if (Car_Motor_Set_Speed(left, right, 0, 0)) stopped = 0U;
        return;
    }
    if (!new_sample)
    {
        if ((uint32_t)(now - last_sample_ms) > CAR_CONTROL_IMU_TIMEOUT_MS)
        {
            memset(&debug, 0, sizeof(debug));
            debug.status = CAR_ATTITUDE_STATUS_ENABLED;
            reset_controllers();
            reset_tracking_state();
            stop_output();
        }
        return;
    }

    memset(&debug, 0, sizeof(debug));
    debug.status = CAR_ATTITUDE_STATUS_ENABLED | CAR_ATTITUDE_STATUS_IMU_VALID;
    if (sensor_state != 0U) debug.status |= CAR_ATTITUDE_STATUS_TRACKING_ONLINE;
    if (heading_locked) debug.status |= CAR_ATTITUDE_STATUS_HEADING_LOCKED;
    last_sequence = imu.sequence;
    last_sample_ms = now;
    const float heading = apply_axis_direction(imu.angle_rad.z, config.invert_z) * CAR_CONTROL_RAD_TO_DEG;
    const float measured_rate = apply_axis_direction(imu.angular_velocity_rad_s.z, config.rate_feedback_inverted) * CAR_CONTROL_RAD_TO_DEG;
    if (!heading_locked)
    {
        previous_heading = heading;
        heading_locked = 1U;
        debug.status |= CAR_ATTITUDE_STATUS_HEADING_LOCKED;
        stop_output();
        return;
    }
    unwrapped_heading += wrap_angle(heading - previous_heading);
    previous_heading = heading;
    float speed_scale = 1.0f;
    float tracking_command;
    const float optical = Car_Tracking_Method_Target_Offset(sensor_state, tracking_method,
        tracking_config.layer1_weight_deg_s,
        tracking_config.layer2_weight_deg_s, tracking_config.layer3_weight_deg_s, tracking_config.layer4_weight_deg_s,
        CAR_CONTROL_TRACKING_LOOKAHEAD_S);
    if (tracking_method >= (uint8_t)CAR_TRACKING_METHOD_DIRECT_ANGLE)
    {
        tracking_command = update_direct_tracking(sensor_state, tracking_method, now, &speed_scale);
        if (sensor_state == 0U) debug.status |= CAR_ATTITUDE_STATUS_TRACKING_LOST;
    }
    else tracking_command = optical;
    if (tracking_method == (uint8_t)CAR_TRACKING_METHOD_DIRECT_RATE) target_heading = unwrapped_heading;
    else target_heading = unwrapped_heading +
        (tracking_method == (uint8_t)CAR_TRACKING_METHOD_DIRECT_ANGLE ? tracking_command : optical);
    const float heading_error = unwrapped_heading - target_heading;
    float target_rate = tracking_method == (uint8_t)CAR_TRACKING_METHOD_DIRECT_RATE ? tracking_command :
                        PIDController_Update(&heading_pid, 0.0f, heading_error);
    if (tracking_method == (uint8_t)CAR_TRACKING_METHOD_GEOMETRY &&
        (arc_target_rate != 0.0f || arc_commanded_rate != 0.0f))
    {
        const car_lap_arc_config_t *arc = Car_Lap_Task_Get_Arc_Config();
        arc_commanded_rate = Car_Tracking_Approach(arc_commanded_rate, arc_target_rate,
                                                    arc->feedforward_rate_slew_deg_s2 * CAR_CONTROL_PERIOD_S);
        target_rate = Car_Tracking_Arc_Target_Rate(arc_commanded_rate, optical / CAR_CONTROL_TRACKING_LOOKAHEAD_S,
                                                    arc->optical_rate_limit_deg_s);
        debug.status |= CAR_ATTITUDE_STATUS_ARC_ACTIVE;
        debug.arc_feedforward_rate = arc_commanded_rate;
        debug.arc_optical_rate = target_rate - arc_commanded_rate;
    }
    target_rate = clampf(target_rate, -config.max_rate, config.max_rate);
    float differential = PIDController_Update(&rate_pid, target_rate, measured_rate);
    differential = clampf(differential, -config.max_differential, config.max_differential);
    const float step = Car_Lap_Task_Get_Arc_Config()->differential_slew_s2 * CAR_CONTROL_PERIOD_S;
    const float limited = Car_Tracking_Approach(commanded_differential, differential, step);
    if (limited != differential) debug.status |= CAR_ATTITUDE_STATUS_DIFFERENTIAL_SLEW_LIMITED;
    commanded_differential = limited;
    const float ramp = config.start_ramp_ms <= 0.0f ? 1.0f :
        clampf((float)(now - enabled_ms) / config.start_ramp_ms, 0.0f, 1.0f);
    const float base_speed = config.base_speed * ramp * speed_scale;
    const float left_unclamped = base_speed - limited;
    const float right_unclamped = base_speed + limited;
    const int16_t left = (int16_t)clampf(left_unclamped, -1000.0f, 1000.0f);
    const int16_t right = (int16_t)clampf(right_unclamped, -1000.0f, 1000.0f);
    debug.target_heading = target_heading;
    debug.tracking_angle = unwrapped_heading;
    debug.optical_heading_correction_deg_s = optical;
    debug.commanded_base_speed = base_speed;
    debug.target_rate = target_rate;
    debug.measured_rate = measured_rate;
    debug.heading_p = heading_pid.Kp * -heading_error;
    debug.heading_i = heading_pid.integrator;
    debug.heading_d = heading_pid.differentiator;
    debug.rate_p = rate_pid.Kp * (target_rate - measured_rate);
    debug.rate_i = rate_pid.integrator;
    debug.rate_d = rate_pid.differentiator;
    debug.differential = limited;
    debug.left_target = (float)left;
    debug.right_target = (float)right;
    debug.tracking_command = tracking_command;
    debug.tracking_speed_scale = speed_scale;
    debug.tracking_lost_elapsed_ms = tracking_lost_since_ms == 0U ? 0.0f : (float)(now - tracking_lost_since_ms);
    if (Car_Motor_Set_Speed(left, right, 0, 0)) stopped = 0U;
}
