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

static arm_angle_control_config_t config;
static arm_angle_control_debug_t debug;
static PIDController controller;
static uint32_t last_sequence;
static uint32_t last_sample_ms;
static uint8_t output_active;

static void stop_output(arm_angle_status_t status)
{
    debug.output_rpm = 0.0f;
    debug.status = (uint8_t)status;
    PIDController_Init(&controller);
    if (output_active != 0U && zdt_stepper_motors[0] != NULL)
    {
        ZDT_Stepper_stop(zdt_stepper_motors[0], ZDT_STEPPER_SYNC_DISABLE);
    }
    output_active = 0U;
}

static void fault_output(arm_angle_status_t status)
{
    debug.enabled = 0U;
    stop_output(status);
}

static void apply_controller_config(void)
{
    controller.Kp = config.kp;
    controller.Ki = config.ki;
    controller.Kd = config.kd;
    controller.tau = config.derivative_filter_s;
    controller.limMin = -config.maximum_speed_rpm;
    controller.limMax = config.maximum_speed_rpm;
    controller.limMinInt = -config.maximum_speed_rpm;
    controller.limMaxInt = config.maximum_speed_rpm;
    controller.T = 0.01f;
    PIDController_Init(&controller);
}

void Arm_Angle_Control_Reset_Defaults(void)
{
    config.kp = 1.0f;
    config.ki = 0.0f;
    config.kd = 0.05f;
    config.derivative_filter_s = 0.03f;
    config.maximum_speed_rpm = 10.0f;
    config.deadzone_deg = 0.2f;
    config.direction_inverted = 0.0f;
    apply_controller_config();
}

void Arm_Angle_Control_Init(void)
{
    memset(&debug, 0, sizeof(debug));
    last_sequence = 0U;
    last_sample_ms = 0U;
    output_active = 0U;
    Arm_Angle_Control_Reset_Defaults();
    debug.enabled = 1U;
    debug.status = ARM_ANGLE_STATUS_WAITING_STARTUP;
}

bool Arm_Angle_Control_Apply_Config(const arm_angle_control_config_t *next)
{
    if (next == NULL || !isfinite(next->kp) || next->kp < 0.0f || next->kp > 100.0f ||
        !isfinite(next->ki) || next->ki < 0.0f || next->ki > 100.0f ||
        !isfinite(next->kd) || next->kd < 0.0f || next->kd > 100.0f ||
        !isfinite(next->derivative_filter_s) || next->derivative_filter_s < 0.001f ||
        next->derivative_filter_s > 1.0f || !isfinite(next->maximum_speed_rpm) ||
        next->maximum_speed_rpm <= 0.0f || next->maximum_speed_rpm > 100.0f ||
        !isfinite(next->deadzone_deg) || next->deadzone_deg < 0.0f || next->deadzone_deg > 5.0f ||
        (next->direction_inverted != 0.0f && next->direction_inverted != 1.0f))
    {
        return false;
    }
    config = *next;
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
    PIDController_Init(&controller);
    return true;
}

void Arm_Angle_Control_Process(void)
{
    const jy61p_status_t *imu_status = JY61P_Get_Status();
    vision_link_sample_t vision_sample;
    ZDTStepperMotor *motor = zdt_stepper_motors[0];
    uint32_t now = HAL_GetTick();
    float output;

    Vision_Link_Process();
    debug.vision_valid = Vision_Link_Get_Sample(&vision_sample) ? 1U : 0U;
    if (debug.vision_valid != 0U)
    {
        debug.vision_position_cm = vision_sample.position_cm;
        debug.vision_velocity_cm_s = vision_sample.velocity_cm_s;
    }

    if (debug.enabled == 0U || debug.authorized == 0U || motor == NULL)
    {
        if (debug.enabled != 0U && debug.authorized == 0U) debug.status = ARM_ANGLE_STATUS_WAITING_STARTUP;
        return;
    }
    if (motor->data.motor_status_ms != 0U &&
        now - motor->data.motor_status_ms <= ARM_ANGLE_MOTOR_STATUS_TIMEOUT_MS &&
        (motor->data.motor_stall != 0U || motor->data.motor_stall_protection != 0U))
    {
        fault_output(ARM_ANGLE_STATUS_STALL);
        return;
    }
    if (imu_status->sample_valid == 0U ||
        (debug.imu_valid == 0U && imu_status->consecutive_failures != 0U) ||
        (debug.imu_valid != 0U && now - last_sample_ms > ARM_ANGLE_IMU_TIMEOUT_MS))
    {
        debug.imu_valid = 0U;
        fault_output(ARM_ANGLE_STATUS_IMU_TIMEOUT);
        return;
    }
    if (imu_status->sequence == last_sequence) return;

    const jy61p_sample_t *sample = JY61P_Get_Sample();
    uint32_t dt_ms = last_sample_ms == 0U ? 10U : now - last_sample_ms;
    if (dt_ms == 0U) dt_ms = 1U;
    if (dt_ms > ARM_ANGLE_IMU_TIMEOUT_MS) dt_ms = ARM_ANGLE_IMU_TIMEOUT_MS;
    controller.T = (float)dt_ms * 0.001f;
    last_sequence = imu_status->sequence;
    last_sample_ms = now;
    debug.sample_ms = now;
    debug.imu_valid = 1U;
    debug.measured_deg = (float)sample->angle.y * JY61P_ANGLE_RAW_TO_DEG;
    debug.error_deg = debug.target_deg - debug.measured_deg;

    output = PIDController_Update(&controller, debug.target_deg, debug.measured_deg);
    if (fabsf(debug.error_deg) <= config.deadzone_deg)
    {
        output = 0.0f;
        PIDController_Init(&controller);
    }
    if (config.direction_inverted != 0.0f) output = -output;
    if (motor->data.limits_enabled != 0U &&
        ((motor->data.current_position <= motor->data.limit_min && output < 0.0f) ||
         (motor->data.current_position >= motor->data.limit_max && output > 0.0f)))
    {
        fault_output(ARM_ANGLE_STATUS_LIMIT);
        return;
    }
    debug.output_rpm = output;
    debug.status = ARM_ANGLE_STATUS_RUNNING;

    if (output == 0.0f)
    {
        if (output_active != 0U) ZDT_Stepper_stop(motor, ZDT_STEPPER_SYNC_DISABLE);
        output_active = 0U;
        return;
    }
    ZDT_Stepper_Set_Speed(motor, output > 0.0f ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW,
                          fabsf(output), ZDT_STEPPER_SPEED_ACCEL,
                          ZDT_STEPPER_SYNC_DISABLE);
    output_active = 1U;
}
