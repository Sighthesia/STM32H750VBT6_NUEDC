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
static float last_motor_target_deg;
static uint8_t motor_calibration_valid;
static uint8_t position_command_active;
static uint8_t pitch_inverted;

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void stop_output(arm_angle_status_t status)
{
    debug.ball_target_velocity_cm_s = 0.0f;
    debug.ball_velocity_error_cm_s = 0.0f;
    debug.status = (uint8_t)status;
    PIDController_Init(&ball_position_controller);
    PIDController_Init(&ball_velocity_controller);
    if (position_command_active != 0U && zdt_stepper_motors[0] != NULL)
    {
        ZDT_Stepper_stop(zdt_stepper_motors[0], ZDT_STEPPER_SYNC_DISABLE);
    }
    position_command_active = 0U;
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
    config.ball_target_position_cm = 0.0f;
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
    apply_controller_config();
}

void Arm_Angle_Control_Init(void)
{
    memset(&debug, 0, sizeof(debug));
    last_sequence = 0U;
    last_sample_ms = 0U;
    last_vision_sample_ms = 0U;
    last_position_command_ms = 0U;
    motor_calibration_valid = 0U;
    position_command_active = 0U;
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
        (next->vision_prediction_enabled != 0.0f && next->vision_prediction_enabled != 1.0f))
    {
        return false;
    }
    config = *next;
    Vision_Link_Set_Kalman_Enabled(config.vision_kalman_enabled != 0.0f);
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

bool Arm_Angle_Control_Set_Motor_Calibration(float high_position_deg,
                                             float balance_position_deg,
                                             float low_position_deg)
{
    float minimum;
    float maximum;

    if (!isfinite(high_position_deg) || !isfinite(balance_position_deg) ||
        !isfinite(low_position_deg)) return false;
    minimum = fminf(high_position_deg, low_position_deg);
    maximum = fmaxf(high_position_deg, low_position_deg);
    if (!(minimum < balance_position_deg && balance_position_deg < maximum)) return false;
    motor_position_min_deg = minimum;
    motor_position_balance_deg = balance_position_deg;
    motor_position_max_deg = maximum;
    last_motor_target_deg = balance_position_deg;
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
    uint8_t direction;

    Vision_Link_Process();
    debug.vision_valid = Vision_Link_Get_Sample(&vision_sample) ? 1U : 0U;
    if (debug.vision_valid != 0U)
    {
        debug.vision_position_cm = vision_sample.position_cm;
        debug.vision_velocity_cm_s = vision_sample.velocity_cm_s;
        if (config.vision_prediction_enabled != 0.0f)
        {
            uint32_t prediction_ms = now - vision_sample.received_ms;
            if (prediction_ms > ARM_ANGLE_IMU_TIMEOUT_MS) prediction_ms = ARM_ANGLE_IMU_TIMEOUT_MS;
            debug.vision_position_cm += debug.vision_velocity_cm_s * ((float)prediction_ms * 0.001f);
        }
    }

    if (debug.enabled == 0U || debug.authorized == 0U || motor == NULL)
    {
        if (debug.enabled != 0U && debug.authorized == 0U) debug.status = ARM_ANGLE_STATUS_WAITING_STARTUP;
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
    if (vision_sample.received_ms == last_vision_sample_ms) return;
    uint32_t dt_ms = last_vision_sample_ms == 0U ? 10U :
                     vision_sample.received_ms - last_vision_sample_ms;
    if (dt_ms == 0U) dt_ms = 1U;
    if (dt_ms > 100U) dt_ms = 100U;
    last_vision_sample_ms = vision_sample.received_ms;
    ball_position_controller.T = (float)dt_ms * 0.001f;
    ball_velocity_controller.T = ball_position_controller.T;

    debug.ball_position_error_cm = config.ball_target_position_cm - debug.vision_position_cm;
    if (config.ball_position_loop_enabled != 0.0f)
        target_velocity = PIDController_Update(&ball_position_controller,
                                               config.ball_target_position_cm,
                                               debug.vision_position_cm);
    else
        target_velocity = 0.0f;
    target_velocity = clampf(target_velocity, -config.ball_velocity_limit_cm_s,
                             config.ball_velocity_limit_cm_s);
    debug.ball_target_velocity_cm_s = target_velocity;
    debug.ball_velocity_error_cm_s = target_velocity - debug.vision_velocity_cm_s;

    if (config.ball_velocity_loop_enabled != 0.0f)
        position_offset = PIDController_Update(&ball_velocity_controller,
                                               target_velocity,
                                               debug.vision_velocity_cm_s);
    else
        position_offset = 0.0f;
    if (config.direction_inverted != 0.0f) position_offset = -position_offset;
    if (pitch_inverted != 0U) position_offset = -position_offset;
    motor_target = clampf(motor_position_balance_deg + position_offset,
                          motor_position_min_deg, motor_position_max_deg);
    debug.motor_target_position_deg = motor_target;
    debug.output_rpm = config.motor_position_speed_rpm;
    debug.status = ARM_ANGLE_STATUS_RUNNING;

    if ((uint32_t)(now - last_position_command_ms) < (uint32_t)config.motor_position_command_period_ms ||
        fabsf(motor_target - last_motor_target_deg) < config.motor_position_command_threshold_deg)
        return;
    direction = motor_target >= 0.0f ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW;
    ZDT_Stepper_Set_position(motor, direction, config.motor_position_speed_rpm,
                             fabsf(motor_target), ZDT_STEPPER_ABS_POS_MODE,
                             ZDT_STEPPER_SYNC_DISABLE);
    last_motor_target_deg = motor_target;
    last_position_command_ms = now;
    position_command_active = 1U;
}
