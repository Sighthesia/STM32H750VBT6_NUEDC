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
    ARM_ANGLE_STATUS_VISION_TIMEOUT,
    ARM_ANGLE_STATUS_LIMIT,
    ARM_ANGLE_STATUS_STALL,
    ARM_ANGLE_STATUS_MANUAL_OVERRIDE
} arm_angle_status_t;

typedef struct
{
    float ball_position_loop_enabled;
    float ball_velocity_loop_enabled;
    float ball_target_position_cm;
    float ball_target_velocity_cm_s;
    float ball_position_kp;
    float ball_position_ki;
    float ball_position_kd;
    float ball_velocity_kp;
    float ball_velocity_ki;
    float ball_velocity_kd;
    float derivative_filter_s;
    float ball_velocity_limit_cm_s;
    float motor_position_offset_limit_deg;
    float motor_position_speed_rpm;
    float motor_position_command_threshold_deg;
    float motor_position_command_period_ms;
    float direction_inverted;
    float vision_kalman_enabled;
    float vision_prediction_enabled;
    float vision_fixed_delay_ms;
    float vision_gravity_feedforward_gain;
    float vision_beam_angle_deadzone_deg;
    float vision_prediction_max_ms;
    float motor_trajectory_enabled;
    float motor_trajectory_max_speed_deg_s;
    float motor_trajectory_max_acceleration_deg_s2;
    float vision_kalman_measurement_noise;
    float vision_kalman_process_noise_position;
    float vision_kalman_process_noise_velocity;
    float ball_velocity_output_inverted;
    float ball_brake_feedforward_enabled;
    float ball_brake_stop_time_s;
    float ball_brake_gain;
    float ball_brake_max_acceleration_cm_s2;
    float ball_brake_angle_limit_deg;
    float ball_brake_speed_deadzone_cm_s;
    float ball_brake_output_inverted;
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
    float vision_raw_position_cm;
    float vision_raw_velocity_cm_s;
    float vision_beam_angle_deg;
    float vision_feedforward_acceleration_cm_s2;
    float vision_prediction_ms;
    float ball_target_position_cm;
    float ball_position_error_cm;
    float ball_position_p_output_cm_s;
    float ball_target_velocity_cm_s;
    float ball_velocity_error_cm_s;
    float ball_velocity_p_output_deg;
    float ball_velocity_i_output_deg;
    float ball_velocity_d_output_deg;
    float ball_velocity_raw_output_deg;
    float motor_position_offset_deg;
    uint8_t ball_velocity_output_limited;
    float motor_target_position_deg;
    float motor_trajectory_position_deg;
    float motor_trajectory_velocity_deg_s;
    float ball_brake_target_acceleration_cm_s2;
    float ball_brake_raw_angle_deg;
    float ball_brake_angle_deg;
    float ball_combined_position_offset_deg;
    uint8_t vision_valid;
    uint8_t status;
} arm_angle_control_debug_t;

void Arm_Angle_Control_Init(void);
void Arm_Angle_Control_Process(void);
void Arm_Angle_Control_Set_Authorized(bool authorized);
void Arm_Angle_Control_Set_Pitch_Inverted(bool inverted);
bool Arm_Angle_Control_Set_Enabled(bool enabled);
bool Arm_Angle_Control_Set_Target(float target_deg);
bool Arm_Angle_Control_Return_To_Balance(void);
bool Arm_Angle_Control_Set_Ball_Velocity_Target(float target_cm_s);
bool Arm_Angle_Control_Set_Motor_Calibration(float high_position_deg,
                                             float high_pitch_deg,
                                             float balance_position_deg,
                                             float balance_pitch_deg,
                                             float low_position_deg,
                                             float low_pitch_deg);
bool Arm_Angle_Control_Apply_Config(const arm_angle_control_config_t *config);
void Arm_Angle_Control_Reset_Defaults(void);
arm_angle_control_config_t *Arm_Angle_Control_Get_Config(void);
const arm_angle_control_debug_t *Arm_Angle_Control_Get_Debug(void);

#endif /* ARM_ANGLE_CONTROL_H */
