#ifndef ARM_ANGLE_CONTROL_H
#define ARM_ANGLE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ARM_ANGLE_STATUS_DISABLED = 0,
    ARM_ANGLE_STATUS_WAITING_STARTUP,
    ARM_ANGLE_STATUS_RUNNING,
    ARM_ANGLE_STATUS_IMU_TIMEOUT,
    ARM_ANGLE_STATUS_LIMIT,
    ARM_ANGLE_STATUS_STALL,
    ARM_ANGLE_STATUS_MANUAL_OVERRIDE
} arm_angle_status_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float derivative_filter_s;
    float maximum_speed_rpm;
    float deadzone_deg;
    float direction_inverted;
    float motor_control_mode;
    float motor_position_limit;
    float motor_speed_limit;
    float motor_speed_kp;
    float motor_speed_ki;
    float motor_speed_kd;
} arm_angle_control_config_t;

typedef struct
{
    float target_deg;
    float measured_deg;
    float error_deg;
    float output_rpm;
    uint32_t sample_ms;
    uint8_t enabled;
    uint8_t authorized;
    uint8_t imu_valid;
    float vision_position_cm;
    float vision_velocity_cm_s;
    uint8_t vision_valid;
    uint8_t status;
} arm_angle_control_debug_t;

void Arm_Angle_Control_Init(void);
void Arm_Angle_Control_Process(void);
void Arm_Angle_Control_Set_Authorized(bool authorized);
bool Arm_Angle_Control_Set_Enabled(bool enabled);
bool Arm_Angle_Control_Set_Target(float target_deg);
bool Arm_Angle_Control_Apply_Config(const arm_angle_control_config_t *config);
void Arm_Angle_Control_Reset_Defaults(void);
arm_angle_control_config_t *Arm_Angle_Control_Get_Config(void);
const arm_angle_control_debug_t *Arm_Angle_Control_Get_Debug(void);

#endif /* ARM_ANGLE_CONTROL_H */
