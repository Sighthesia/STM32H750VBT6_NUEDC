#include "Arm_Angle_Control.h"

#include "JY61P.h"
#include "PID.h"
#include "ZDT_Stepper.h"
#include "Vision_Link.h"

#include "main.h"

#include <math.h>
#include <string.h>

#define JY61P_ANGLE_RAW_TO_DEG (180.0f / 32768.0f)
#define ARM_ANGLE_IMU_TIMEOUT_MS 100U
#define ARM_ANGLE_MOTOR_STATUS_TIMEOUT_MS 500U
#define ARM_ANGLE_TARGET_LIMIT_DEG 30.0f
#define ARM_BALL_TARGET_LIMIT_CM 12.5f
#define ARM_BALL_GRAVITY_CM_S2 981.0f
#define ARM_BALL_ROLLING_ACCELERATION_RATIO (5.0f / 7.0f)
#define ARM_VISION_PREDICTION_MAX_MS 200.0f
#define ARM_MOTOR_POSITION_FEEDBACK_TIMEOUT_MS 200U
#define DEG_TO_RAD (0.01745329251994329577f)

static arm_angle_control_config_t config;
static arm_angle_control_debug_t debug;
static PIDController ball_position_controller;
static PIDController ball_velocity_controller;
static uint32_t last_sequence;
static uint32_t last_sample_ms;
static uint32_t last_vision_sample_ms;
static uint32_t last_position_command_ms;
static float motor_position_min_deg;
static float motor_position_balance_deg;
static float motor_position_max_deg;
static float motor_pitch_min_deg;
static float motor_pitch_balance_deg;
static float motor_pitch_max_deg;
static float last_motor_target_deg;
static float desired_motor_target_deg;
static float trajectory_position_deg;
static float trajectory_velocity_deg_s;
static uint32_t last_trajectory_update_ms;
static uint8_t trajectory_initialized;
static uint8_t motor_calibration_valid;
static uint8_t position_command_active;
static uint8_t pitch_inverted;
static float previous_velocity_target;
static float previous_velocity_loop_enabled;

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float interpolate(float input, float input_a, float input_b,
                         float output_a, float output_b)
{
    float ratio = (input - input_a) / (input_b - input_a);
    return output_a + ratio * (output_b - output_a);
}

static float beam_angle_from_motor_position(float motor_position_deg)
{
    float clamped = clampf(motor_position_deg, motor_position_min_deg,
                           motor_position_max_deg);
    float pitch;

    if (clamped <= motor_position_balance_deg)
        pitch = interpolate(clamped, motor_position_min_deg,
                            motor_position_balance_deg, motor_pitch_min_deg,
                            motor_pitch_balance_deg);
    else
        pitch = interpolate(clamped, motor_position_balance_deg,
                            motor_position_max_deg, motor_pitch_balance_deg,
                            motor_pitch_max_deg);
    return pitch - motor_pitch_balance_deg;
}

static float move_towards(float value, float target, float maximum_delta)
{
    if (value < target) return fminf(value + maximum_delta, target);
    return fmaxf(value - maximum_delta, target);
}

static void reset_motor_trajectory(float position_deg)
{
    trajectory_position_deg = position_deg;
    trajectory_velocity_deg_s = 0.0f;
    last_trajectory_update_ms = HAL_GetTick();
    trajectory_initialized = 1U;
    debug.motor_trajectory_position_deg = position_deg;
    debug.motor_trajectory_velocity_deg_s = 0.0f;
}

static void update_motor_trajectory(ZDTStepperMotor *motor, uint32_t now)
{
    float command_target = desired_motor_target_deg;
    float command_rpm = config.motor_position_speed_rpm;
    uint32_t dt_ms;
    uint8_t direction;

    if ((uint32_t)(now - last_position_command_ms) <
        (uint32_t)config.motor_position_command_period_ms) return;
    if (trajectory_initialized == 0U)
    {
        float initial = motor->data.current_position_ms != 0U ?
                        motor->data.current_position : last_motor_target_deg;
        reset_motor_trajectory(clampf(initial, motor_position_min_deg,
                                      motor_position_max_deg));
    }

    if (config.motor_trajectory_enabled != 0.0f)
    {
        float error;
        float target_velocity;
        float maximum_speed;
        float dt;
        float next_position;

        dt_ms = now - last_trajectory_update_ms;
        if (dt_ms == 0U) dt_ms = 1U;
        if (dt_ms > 100U) dt_ms = 100U;
        dt = (float)dt_ms * 0.001f;
        error = desired_motor_target_deg - trajectory_position_deg;
        maximum_speed = fminf(config.motor_trajectory_max_speed_deg_s,
                              config.motor_position_speed_rpm * 6.0f);
        target_velocity = sqrtf(2.0f * config.motor_trajectory_max_acceleration_deg_s2 *
                                fabsf(error));
        target_velocity = fminf(target_velocity, maximum_speed);
        if (error < 0.0f) target_velocity = -target_velocity;
        trajectory_velocity_deg_s = move_towards(
            trajectory_velocity_deg_s, target_velocity,
            config.motor_trajectory_max_acceleration_deg_s2 * dt);
        next_position = trajectory_position_deg + trajectory_velocity_deg_s * dt;
        if ((error > 0.0f && next_position >= desired_motor_target_deg) ||
            (error < 0.0f && next_position <= desired_motor_target_deg) ||
            fabsf(error) < config.motor_position_command_threshold_deg)
        {
            next_position = desired_motor_target_deg;
            trajectory_velocity_deg_s = 0.0f;
        }
        trajectory_position_deg = clampf(next_position, motor_position_min_deg,
                                         motor_position_max_deg);
        command_target = trajectory_position_deg;
        if (trajectory_velocity_deg_s == 0.0f &&
            fabsf(command_target - last_motor_target_deg) <
                config.motor_position_command_threshold_deg)
        {
            last_trajectory_update_ms = now;
            return;
        }
        if (fabsf(trajectory_velocity_deg_s) > 0.0f)
            command_rpm = clampf(fabsf(trajectory_velocity_deg_s) / 6.0f,
                                 1.0f, config.motor_position_speed_rpm);
    }
    else
    {
        trajectory_position_deg = desired_motor_target_deg;
        trajectory_velocity_deg_s = 0.0f;
        if (fabsf(command_target - last_motor_target_deg) <
            config.motor_position_command_threshold_deg) return;
    }

    debug.motor_trajectory_position_deg = trajectory_position_deg;
    debug.motor_trajectory_velocity_deg_s = trajectory_velocity_deg_s;
    debug.output_rpm = command_rpm;
    direction = command_target >= 0.0f ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW;
    ZDT_Stepper_Set_position(motor, direction, command_rpm,
                             fabsf(command_target), ZDT_STEPPER_ABS_POS_MODE,
                             ZDT_STEPPER_SYNC_DISABLE);
    last_motor_target_deg = command_target;
    last_position_command_ms = now;
    last_trajectory_update_ms = now;
    position_command_active = 1U;
}

static void stop_output(arm_angle_status_t status)
{
    debug.ball_target_velocity_cm_s = 0.0f;
    debug.ball_velocity_error_cm_s = 0.0f;
    debug.ball_position_p_output_cm_s = 0.0f;
    debug.ball_velocity_p_output_deg = 0.0f;
    debug.ball_velocity_i_output_deg = 0.0f;
    debug.ball_velocity_d_output_deg = 0.0f;
    debug.ball_velocity_raw_output_deg = 0.0f;
    debug.motor_position_offset_deg = 0.0f;
    debug.ball_velocity_output_limited = 0U;
    debug.status = (uint8_t)status;
    PIDController_Init(&ball_position_controller);
    PIDController_Init(&ball_velocity_controller);
    if (position_command_active != 0U && zdt_stepper_motors[0] != NULL)
    {
        ZDT_Stepper_stop(zdt_stepper_motors[0], ZDT_STEPPER_SYNC_DISABLE);
    }
    position_command_active = 0U;
    trajectory_initialized = 0U;
    trajectory_velocity_deg_s = 0.0f;
}

static void fault_output(arm_angle_status_t status)
{
    debug.enabled = 0U;
    stop_output(status);
}

static void apply_controller_config(void)
{
    ball_position_controller.Kp = config.ball_position_kp;
    ball_position_controller.Ki = config.ball_position_ki;
    ball_position_controller.Kd = config.ball_position_kd;
    ball_position_controller.tau = config.derivative_filter_s;
    ball_position_controller.limMin = -config.ball_velocity_limit_cm_s;
    ball_position_controller.limMax = config.ball_velocity_limit_cm_s;
    ball_position_controller.limMinInt = -config.ball_velocity_limit_cm_s;
    ball_position_controller.limMaxInt = config.ball_velocity_limit_cm_s;
    ball_position_controller.T = 0.01f;
    PIDController_Init(&ball_position_controller);
    ball_velocity_controller.Kp = config.ball_velocity_kp;
    ball_velocity_controller.Ki = config.ball_velocity_ki;
    ball_velocity_controller.Kd = config.ball_velocity_kd;
    ball_velocity_controller.tau = config.derivative_filter_s;
    ball_velocity_controller.limMin = -config.motor_position_offset_limit_deg;
    ball_velocity_controller.limMax = config.motor_position_offset_limit_deg;
    ball_velocity_controller.limMinInt = -config.motor_position_offset_limit_deg;
    ball_velocity_controller.limMaxInt = config.motor_position_offset_limit_deg;
    ball_velocity_controller.T = 0.01f;
    PIDController_Init(&ball_velocity_controller);
}

void Arm_Angle_Control_Reset_Defaults(void)
{
    memset(&config, 0, sizeof(config));
    config.ball_position_loop_enabled = 1.0f;
    config.ball_velocity_loop_enabled = 1.0f;
    config.ball_target_position_cm = 0.0f;
    config.ball_target_velocity_cm_s = 0.0f;
    config.ball_position_kp = 1.0f;
    config.ball_velocity_kp = 1.0f;
    config.derivative_filter_s = 0.03f;
    config.ball_velocity_limit_cm_s = 20.0f;
    config.motor_position_offset_limit_deg = 10.0f;
    config.motor_position_speed_rpm = 10.0f;
    config.motor_position_command_threshold_deg = 0.1f;
    config.motor_position_command_period_ms = 20.0f;
    config.vision_kalman_enabled = 1.0f;
    config.vision_prediction_enabled = 1.0f;
    config.vision_fixed_delay_ms = 60.0f;
    config.vision_gravity_feedforward_gain = 0.7f;
    config.vision_beam_angle_deadzone_deg = 0.8f;
    config.vision_prediction_max_ms = 200.0f;
    config.motor_trajectory_enabled = 1.0f;
    config.motor_trajectory_max_speed_deg_s = 60.0f;
    config.motor_trajectory_max_acceleration_deg_s2 = 300.0f;
    config.vision_kalman_measurement_noise = 0.02f;
    config.vision_kalman_process_noise_position = 0.02f;
    config.vision_kalman_process_noise_velocity = 0.3f;
    config.ball_velocity_output_inverted = 0.0f;
    apply_controller_config();
}

void Arm_Angle_Control_Init(void)
{
    memset(&debug, 0, sizeof(debug));
    last_sequence = 0U;
    last_sample_ms = 0U;
    last_vision_sample_ms = 0U;
    last_position_command_ms = 0U;
    last_trajectory_update_ms = 0U;
    motor_calibration_valid = 0U;
    position_command_active = 0U;
    trajectory_initialized = 0U;
    pitch_inverted = 0U;
    Arm_Angle_Control_Reset_Defaults();
    debug.enabled = 1U;
    debug.status = ARM_ANGLE_STATUS_WAITING_STARTUP;
}

bool Arm_Angle_Control_Apply_Config(const arm_angle_control_config_t *next)
{
    if (next == NULL ||
        (next->ball_position_loop_enabled != 0.0f && next->ball_position_loop_enabled != 1.0f) ||
        (next->ball_velocity_loop_enabled != 0.0f && next->ball_velocity_loop_enabled != 1.0f) ||
        !isfinite(next->ball_target_position_cm) || fabsf(next->ball_target_position_cm) > ARM_BALL_TARGET_LIMIT_CM ||
        !isfinite(next->ball_target_velocity_cm_s) || fabsf(next->ball_target_velocity_cm_s) > 100.0f ||
        !isfinite(next->ball_position_kp) || next->ball_position_kp < 0.0f || next->ball_position_kp > 100.0f ||
        !isfinite(next->ball_position_ki) || next->ball_position_ki < 0.0f || next->ball_position_ki > 100.0f ||
        !isfinite(next->ball_position_kd) || next->ball_position_kd < 0.0f || next->ball_position_kd > 100.0f ||
        !isfinite(next->ball_velocity_kp) || next->ball_velocity_kp < 0.0f || next->ball_velocity_kp > 100.0f ||
        !isfinite(next->ball_velocity_ki) || next->ball_velocity_ki < 0.0f || next->ball_velocity_ki > 100.0f ||
        !isfinite(next->ball_velocity_kd) || next->ball_velocity_kd < 0.0f || next->ball_velocity_kd > 100.0f ||
        !isfinite(next->derivative_filter_s) || next->derivative_filter_s < 0.001f ||
        next->derivative_filter_s > 1.0f ||
        !isfinite(next->ball_velocity_limit_cm_s) || next->ball_velocity_limit_cm_s <= 0.0f || next->ball_velocity_limit_cm_s > 100.0f ||
        !isfinite(next->motor_position_offset_limit_deg) || next->motor_position_offset_limit_deg <= 0.0f || next->motor_position_offset_limit_deg > 180.0f ||
        !isfinite(next->motor_position_speed_rpm) || next->motor_position_speed_rpm <= 0.0f || next->motor_position_speed_rpm > 100.0f ||
        !isfinite(next->motor_position_command_threshold_deg) || next->motor_position_command_threshold_deg < 0.01f || next->motor_position_command_threshold_deg > 10.0f ||
        !isfinite(next->motor_position_command_period_ms) || next->motor_position_command_period_ms < 10.0f || next->motor_position_command_period_ms > 1000.0f ||
        (next->direction_inverted != 0.0f && next->direction_inverted != 1.0f) ||
        (next->vision_kalman_enabled != 0.0f && next->vision_kalman_enabled != 1.0f) ||
        (next->vision_prediction_enabled != 0.0f && next->vision_prediction_enabled != 1.0f) ||
        !isfinite(next->vision_fixed_delay_ms) || next->vision_fixed_delay_ms < 0.0f || next->vision_fixed_delay_ms > 200.0f ||
        !isfinite(next->vision_gravity_feedforward_gain) || next->vision_gravity_feedforward_gain < 0.0f || next->vision_gravity_feedforward_gain > 2.0f ||
        !isfinite(next->vision_beam_angle_deadzone_deg) || next->vision_beam_angle_deadzone_deg < 0.0f || next->vision_beam_angle_deadzone_deg > 5.0f ||
        !isfinite(next->vision_prediction_max_ms) || next->vision_prediction_max_ms < 10.0f || next->vision_prediction_max_ms > 500.0f ||
        (next->motor_trajectory_enabled != 0.0f && next->motor_trajectory_enabled != 1.0f) ||
        !isfinite(next->motor_trajectory_max_speed_deg_s) || next->motor_trajectory_max_speed_deg_s < 1.0f || next->motor_trajectory_max_speed_deg_s > 600.0f ||
        !isfinite(next->motor_trajectory_max_acceleration_deg_s2) || next->motor_trajectory_max_acceleration_deg_s2 < 1.0f || next->motor_trajectory_max_acceleration_deg_s2 > 5000.0f ||
        !isfinite(next->vision_kalman_measurement_noise) || next->vision_kalman_measurement_noise < 0.001f || next->vision_kalman_measurement_noise > 0.2f ||
        !isfinite(next->vision_kalman_process_noise_position) || next->vision_kalman_process_noise_position < 0.001f || next->vision_kalman_process_noise_position > 0.2f ||
        !isfinite(next->vision_kalman_process_noise_velocity) || next->vision_kalman_process_noise_velocity < 0.01f || next->vision_kalman_process_noise_velocity > 5.0f ||
        (next->ball_velocity_output_inverted != 0.0f && next->ball_velocity_output_inverted != 1.0f))
    {
        return false;
    }
    config = *next;
    Vision_Link_Set_Kalman_Enabled(config.vision_kalman_enabled != 0.0f);
    Vision_Link_Set_Kalman_Parameters(config.vision_kalman_measurement_noise,
                                      config.vision_kalman_process_noise_position,
                                      config.vision_kalman_process_noise_velocity);
    apply_controller_config();
    return true;
}

arm_angle_control_config_t *Arm_Angle_Control_Get_Config(void)
{
    return &config;
}

const arm_angle_control_debug_t *Arm_Angle_Control_Get_Debug(void)
{
    return &debug;
}

void Arm_Angle_Control_Set_Authorized(bool authorized)
{
    debug.authorized = authorized ? 1U : 0U;
    if (!authorized)
    {
        stop_output(debug.enabled != 0U ? ARM_ANGLE_STATUS_WAITING_STARTUP :
                                         ARM_ANGLE_STATUS_DISABLED);
    }
}

void Arm_Angle_Control_Set_Pitch_Inverted(bool inverted)
{
    pitch_inverted = inverted ? 1U : 0U;
}

bool Arm_Angle_Control_Set_Enabled(bool enabled)
{
    debug.enabled = enabled ? 1U : 0U;
    if (!enabled) stop_output(ARM_ANGLE_STATUS_DISABLED);
    else if (debug.authorized == 0U) debug.status = ARM_ANGLE_STATUS_WAITING_STARTUP;
    return true;
}

bool Arm_Angle_Control_Set_Target(float target_deg)
{
    if (!isfinite(target_deg) || fabsf(target_deg) > ARM_ANGLE_TARGET_LIMIT_DEG) return false;
    debug.target_deg = target_deg;
    PIDController_Init(&ball_position_controller);
    return true;
}

bool Arm_Angle_Control_Set_Ball_Velocity_Target(float target_cm_s)
{
    if (!isfinite(target_cm_s) || fabsf(target_cm_s) > config.ball_velocity_limit_cm_s) return false;
    if ((target_cm_s > 0.0f) != (previous_velocity_target > 0.0f) ||
        (target_cm_s < 0.0f) != (previous_velocity_target < 0.0f))
        PIDController_Init(&ball_velocity_controller);
    config.ball_target_velocity_cm_s = target_cm_s;
    previous_velocity_target = target_cm_s;
    return true;
}

bool Arm_Angle_Control_Set_Motor_Calibration(float high_position_deg,
                                             float high_pitch_deg,
                                             float balance_position_deg,
                                             float balance_pitch_deg,
                                             float low_position_deg,
                                             float low_pitch_deg)
{
    float minimum;
    float maximum;

    if (!isfinite(high_position_deg) || !isfinite(high_pitch_deg) ||
        !isfinite(balance_position_deg) || !isfinite(balance_pitch_deg) ||
        !isfinite(low_position_deg) || !isfinite(low_pitch_deg)) return false;
    minimum = fminf(high_position_deg, low_position_deg);
    maximum = fmaxf(high_position_deg, low_position_deg);
    if (!(minimum < balance_position_deg && balance_position_deg < maximum)) return false;
    motor_position_min_deg = minimum;
    motor_position_balance_deg = balance_position_deg;
    motor_position_max_deg = maximum;
    if (high_position_deg < low_position_deg)
    {
        motor_pitch_min_deg = high_pitch_deg;
        motor_pitch_max_deg = low_pitch_deg;
    }
    else
    {
        motor_pitch_min_deg = low_pitch_deg;
        motor_pitch_max_deg = high_pitch_deg;
    }
    motor_pitch_balance_deg = balance_pitch_deg;
    last_motor_target_deg = balance_position_deg;
    desired_motor_target_deg = balance_position_deg;
    reset_motor_trajectory(balance_position_deg);
    debug.motor_target_position_deg = balance_position_deg;
    motor_calibration_valid = 1U;
    return true;
}

void Arm_Angle_Control_Process(void)
{
    const jy61p_status_t *imu_status = JY61P_Get_Status();
    vision_link_sample_t vision_sample;
    ZDTStepperMotor *motor = zdt_stepper_motors[0];
    uint32_t now = HAL_GetTick();
    float target_velocity;
    float position_offset;
    float motor_target;
    float prediction_s;

    Vision_Link_Process();
    debug.vision_valid = Vision_Link_Get_Sample(&vision_sample) ? 1U : 0U;
    if (debug.vision_valid != 0U)
    {
        debug.vision_position_cm = vision_sample.position_cm;
        debug.vision_velocity_cm_s = vision_sample.velocity_cm_s;
        debug.vision_beam_angle_deg = 0.0f;
        debug.vision_feedforward_acceleration_cm_s2 = 0.0f;
        debug.vision_prediction_ms = 0.0f;
        if (config.vision_prediction_enabled != 0.0f)
        {
            debug.vision_prediction_ms = clampf(config.vision_fixed_delay_ms +
                                                 (float)(now - vision_sample.received_ms),
                                                 0.0f, config.vision_prediction_max_ms);
            if (motor != NULL && motor_calibration_valid != 0U &&
                motor->data.current_position_ms != 0U &&
                (uint32_t)(now - motor->data.current_position_ms) <=
                    ARM_MOTOR_POSITION_FEEDBACK_TIMEOUT_MS)
            {
                debug.vision_beam_angle_deg =
                    beam_angle_from_motor_position(motor->data.current_position);
                if (fabsf(debug.vision_beam_angle_deg) >=
                    config.vision_beam_angle_deadzone_deg)
                {
                    debug.vision_feedforward_acceleration_cm_s2 =
                        config.vision_gravity_feedforward_gain *
                        ARM_BALL_ROLLING_ACCELERATION_RATIO *
                        ARM_BALL_GRAVITY_CM_S2 *
                        sinf(debug.vision_beam_angle_deg * DEG_TO_RAD);
                }
            }
            prediction_s = debug.vision_prediction_ms * 0.001f;
            debug.vision_position_cm += vision_sample.velocity_cm_s * prediction_s +
                0.5f * debug.vision_feedforward_acceleration_cm_s2 *
                prediction_s * prediction_s;
            debug.vision_velocity_cm_s +=
                debug.vision_feedforward_acceleration_cm_s2 * prediction_s;
        }
    }

    if (debug.enabled == 0U || motor == NULL)
    {
        return;
    }
    if (config.ball_position_loop_enabled == 0.0f && config.ball_velocity_loop_enabled == 0.0f)
    {
        stop_output(ARM_ANGLE_STATUS_DISABLED);
        return;
    }
    if (motor_calibration_valid == 0U)
    {
        stop_output(ARM_ANGLE_STATUS_WAITING_STARTUP);
        return;
    }
    if (motor->data.motor_status_ms != 0U &&
        now - motor->data.motor_status_ms <= ARM_ANGLE_MOTOR_STATUS_TIMEOUT_MS &&
        (motor->data.motor_stall != 0U || motor->data.motor_stall_protection != 0U))
    {
        fault_output(ARM_ANGLE_STATUS_STALL);
        return;
    }
    if (imu_status->sample_valid != 0U && imu_status->sequence != last_sequence)
    {
        const jy61p_sample_t *sample = JY61P_Get_Sample();

        last_sequence = imu_status->sequence;
        last_sample_ms = now;
        debug.imu_valid = 1U;
        debug.measured_deg = (float)sample->angle.y * JY61P_ANGLE_RAW_TO_DEG;
        if (pitch_inverted != 0U) debug.measured_deg = -debug.measured_deg;
        debug.error_deg = debug.target_deg - debug.measured_deg;
    }
    else if (last_sample_ms == 0U || (uint32_t)(now - last_sample_ms) > ARM_ANGLE_IMU_TIMEOUT_MS)
    {
        debug.imu_valid = 0U;
    }
    debug.sample_ms = now;
    if (debug.vision_valid == 0U)
    {
        stop_output(ARM_ANGLE_STATUS_VISION_TIMEOUT);
        return;
    }
    if (vision_sample.received_ms == last_vision_sample_ms)
    {
        update_motor_trajectory(motor, now);
        return;
    }
    uint32_t dt_ms = last_vision_sample_ms == 0U ? 10U :
                     vision_sample.received_ms - last_vision_sample_ms;
    if (dt_ms == 0U) dt_ms = 1U;
    if (dt_ms > 100U) dt_ms = 100U;
    last_vision_sample_ms = vision_sample.received_ms;
    ball_position_controller.T = (float)dt_ms * 0.001f;
    ball_velocity_controller.T = ball_position_controller.T;

    debug.ball_position_error_cm = debug.vision_position_cm - config.ball_target_position_cm;
    debug.ball_target_position_cm = config.ball_target_position_cm;
    debug.ball_position_p_output_cm_s = config.ball_position_kp * debug.ball_position_error_cm;
    if (config.ball_position_loop_enabled != 0.0f)
        target_velocity = PIDController_Update(&ball_position_controller,
                                               config.ball_target_position_cm,
                                               debug.vision_position_cm);
    else
        target_velocity = config.ball_target_velocity_cm_s;
    target_velocity = clampf(target_velocity, -config.ball_velocity_limit_cm_s,
                             config.ball_velocity_limit_cm_s);
    debug.ball_target_velocity_cm_s = target_velocity;
    debug.ball_velocity_error_cm_s = target_velocity - debug.vision_velocity_cm_s;
    debug.ball_velocity_p_output_deg = config.ball_velocity_kp * debug.ball_velocity_error_cm_s;
    if (previous_velocity_loop_enabled != config.ball_velocity_loop_enabled)
    {
        PIDController_Init(&ball_velocity_controller);
        previous_velocity_loop_enabled = config.ball_velocity_loop_enabled;
    }

    if (config.ball_velocity_loop_enabled != 0.0f)
        position_offset = PIDController_Update(&ball_velocity_controller,
                                               target_velocity,
                                               debug.vision_velocity_cm_s);
    else
        position_offset = 0.0f;
    debug.ball_velocity_i_output_deg = ball_velocity_controller.integrator;
    debug.ball_velocity_d_output_deg = ball_velocity_controller.differentiator;
    debug.ball_velocity_raw_output_deg = debug.ball_velocity_p_output_deg +
                                         debug.ball_velocity_i_output_deg +
                                         debug.ball_velocity_d_output_deg;
    debug.ball_velocity_output_limited =
        fabsf(debug.ball_velocity_raw_output_deg - position_offset) > 0.001f ? 1U : 0U;
    if (config.ball_velocity_output_inverted == 0.0f) position_offset = -position_offset;
    debug.motor_position_offset_deg = position_offset;
    motor_target = clampf(motor_position_balance_deg + position_offset,
                          motor_position_min_deg, motor_position_max_deg);
    if (fabsf(motor_target - (motor_position_balance_deg + position_offset)) > 0.001f)
        debug.ball_velocity_output_limited |= 0x02U;
    desired_motor_target_deg = motor_target;
    debug.motor_target_position_deg = motor_target;
    debug.status = ARM_ANGLE_STATUS_RUNNING;

    update_motor_trajectory(motor, now);
}
