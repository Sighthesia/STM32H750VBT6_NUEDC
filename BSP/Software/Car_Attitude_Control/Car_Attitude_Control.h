#ifndef CAR_ATTITUDE_CONTROL_H
#define CAR_ATTITUDE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CAR_ATTITUDE_CONTROL_PID = 0,
    CAR_ATTITUDE_CONTROL_LADRC = 1
} car_attitude_control_mode_t;

typedef enum
{
    CAR_ATTITUDE_STEP_NONE = 0,
    CAR_ATTITUDE_STEP_RATE,
    CAR_ATTITUDE_STEP_ANGLE
} car_attitude_step_mode_t;

typedef struct
{
    float mode;
    float base_speed;
    float heading_kp;
    float heading_ki;
    float heading_kd;
    float rate_kp;
    float rate_ki;
    float rate_kd;
    float ladrc_observer_bandwidth;
    float ladrc_controller_bandwidth;
    float ladrc_plant_gain;
    float max_rate;
    float max_differential;
    float invert_x;
    float invert_y;
    float invert_z;
    float step_rate_amplitude;
    float step_angle_amplitude;
    float step_hold_ms;
    float step_angle_feedback_inverted;
} car_attitude_control_config_t;

typedef struct
{
    float direction_inverted;
    float error_filter_time_s;
    float target_rate_slew_deg_s2;
    float line_initial_weight;
    float line_weight_ramp_time_s;
    float line_release_time_s;
    float line_heading_rate_deg_s;
    float heading_error_limit_deg;
} car_tracking_config_t;

enum
{
    CAR_ATTITUDE_STATUS_ENABLED = 1U << 0,
    CAR_ATTITUDE_STATUS_IMU_VALID = 1U << 1,
    CAR_ATTITUDE_STATUS_TRACKING_ONLINE = 1U << 2,
    CAR_ATTITUDE_STATUS_TARGET_RATE_LIMITED = 1U << 3,
    CAR_ATTITUDE_STATUS_DIFFERENTIAL_LIMITED = 1U << 4,
    CAR_ATTITUDE_STATUS_LEFT_SPEED_LIMITED = 1U << 5,
    CAR_ATTITUDE_STATUS_RIGHT_SPEED_LIMITED = 1U << 6,
    CAR_ATTITUDE_STATUS_STEP_ACTIVE = 1U << 7,
    CAR_ATTITUDE_STATUS_ANGLE_STEP = 1U << 8,
    CAR_ATTITUDE_STATUS_LINE_SEARCH = 1U << 9,
    CAR_ATTITUDE_STATUS_PATTERN_INVALID = 1U << 10,
    CAR_ATTITUDE_STATUS_HEADING_TRACKING = 1U << 11
};

typedef struct
{
    float target_heading;
    float tracking_angle;
    float line_offset;
    float line_weight;
    float line_heading_rate;
    float commanded_base_speed;
    float target_rate;
    float measured_rate;
    float heading_p;
    float heading_i;
    float heading_d;
    float rate_p;
    float rate_i;
    float rate_d;
    float differential;
    float left_target;
    float right_target;
    float ladrc_z1;
    float ladrc_z2;
    float ladrc_output;
    uint32_t status;
} car_attitude_control_debug_t;

void Car_Attitude_Control_Init(void);
void Car_Attitude_Control_Process(void);
void Car_Attitude_Control_Reset_Defaults(void);
bool Car_Attitude_Control_Apply_Config(const car_attitude_control_config_t *config);
bool Car_Attitude_Control_Set_Enabled(bool enabled);
bool Car_Attitude_Control_Start_Step(car_attitude_step_mode_t mode);
void Car_Attitude_Control_Cancel_Step(void);
bool Car_Attitude_Control_Is_Enabled(void);
car_attitude_control_config_t *Car_Attitude_Control_Get_Config(void);
car_tracking_config_t *Car_Attitude_Control_Get_Tracking_Config(void);
bool Car_Attitude_Control_Apply_Tracking_Config(const car_tracking_config_t *config);
const car_attitude_control_debug_t *Car_Attitude_Control_Get_Debug(void);

#endif /* CAR_ATTITUDE_CONTROL_H */
