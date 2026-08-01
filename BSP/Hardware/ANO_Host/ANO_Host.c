#include "ANO_Host.h"

#include "Flash_Layout.h"

#include "Car_Motor_USART.h"
#include "Car_Attitude_Control.h"
#include "Car_Lap_Task.h"
#include "Car_IMU.h"
#include "Gray_Sensor.h"
#include "JY61P.h"
#include "Stepper_PID.h"
#include "Arm_Angle_Control.h"
#include "Arm_Competition_Task.h"
#include "Tracking_Link.h"
#include "Vision_Link.h"
#include "W25Qxx.h"
#include "ZDT_Stepper.h"
#include "App_Role.h"
#include "usart.h"

#include <math.h>
#include <string.h>

#define ANO_HEAD 0xABU
#define ANO_HOST_ADDRESS 0xFEU
#define ANO_BROADCAST_ADDRESS 0xFFU
#define ANO_ID_CHECK 0x00U
#define ANO_ID_SENSOR_BASIC 0x01U
#define ANO_ID_SENSOR_EXTENDED 0x02U
#define ANO_ID_ATTITUDE_EULER 0x03U
#define ANO_ID_ATTITUDE_QUATERNION 0x04U
#define ANO_ID_COMMAND 0xC0U
#define ANO_ID_COMMAND_QUERY 0xC1U
#define ANO_ID_COMMAND_INFO 0xC2U
#define ANO_ID_PARAM_COMMAND 0xE0U
#define ANO_ID_PARAM_VALUE 0xE1U
#define ANO_ID_PARAM_INFO 0xE2U
#define ANO_ID_DEVICE_INFO 0xE3U
#define ANO_ID_TELEMETRY 0xF1U
#define ANO_ID_CONTROL_PID 0xF2U
#define ANO_ID_CONTROL_OUTPUT 0xF3U
#define ANO_ID_TRACKING_DIAGNOSTIC 0xF4U
#define ANO_ID_DIAGNOSTIC 0xF5U
#define ANO_ID_ARM_DEBUG_DIAGNOSTIC 0xF7U
#define ANO_ID_B0_CALIBRATION 0xF8U
#define ANO_RX_QUEUE_SIZE 128U
#define ANO_DATA_MAX 96U
#define ANO_TX_QUEUE_DEPTH 8U
#define ANO_RESPONSE_QUEUE_DEPTH 8U
#define ANO_IMU_PERIOD_MS 10U
#define ANO_DEFAULT_BAUD_RATE 921600U
#define ANO_BAUD_SWITCH_DELAY_MS 50U
#define ANO_PARAMETER_SESSION_MS 500U
#define ANO_PARAMETER_SAVE_DELAY_MS 100U
#define ANO_STORAGE_ADDRESS FLASH_ANO_STORAGE_ADDRESS
#define ANO_ARM_ANGLE_STORAGE_ADDRESS FLASH_ARM_ANGLE_STORAGE_ADDRESS
#define ANO_ARM_ANGLE_STORAGE_LEGACY_ADDRESS 512U
#define ANO_ARM_CALIBRATION_STORAGE_A FLASH_ARM_CALIBRATION_STORAGE_A
#define ANO_ARM_CALIBRATION_STORAGE_B FLASH_ARM_CALIBRATION_STORAGE_B
#define ANO_STORAGE_MAGIC 0x414E4F57UL
#define ANO_ARM_ANGLE_STORAGE_MAGIC 0x41414331UL
#define ANO_ARM_ANGLE_STORAGE_VERSION 8U
#define ANO_ARM_CALIBRATION_STORAGE_MAGIC 0x41433350UL
#define ANO_ARM_CALIBRATION_STORAGE_VERSION 2U
#define ANO_STORAGE_VERSION 18U
#define ARM_HOME_STATUS_PERIOD_MS 50U
#define ARM_HOME_SETTLE_MS 100U
#define ARM_STARTUP_HOME_TIMEOUT_MS 22000U
#define ARM_STARTUP_MOVE_TIMEOUT_MS 10000U
#define ARM_BALANCE_TOLERANCE_DEG 0.5f
#define ARM_MOTOR_QUERY_DEFAULT_HZ 100.0f
#define ARM_MOTOR_QUERY_MIN_HZ 20.0f
#define ARM_MOTOR_QUERY_MAX_HZ 200.0f
#define ARM_MOTOR_TELEMETRY_DEFAULT_HZ 20.0f
#define ARM_MOTOR_TELEMETRY_MIN_HZ 1.0f
#define ARM_MOTOR_TELEMETRY_MAX_HZ 50.0f
#define ARM_MOTOR_FEEDBACK_TIMEOUT_MS 200U
#define ARM_MOTOR_QUERY_RESPONSE_TIMEOUT_MS 20U
#define ARM_BALL_POSITION_STEP_TOLERANCE_CM 1.0f
#define ARM_BALL_POSITION_STEP_SETTLE_MS 200U
#define ARM_BALL_POSITION_STEP_PHASE_TIMEOUT_MS 2000U
#define ARM_CALIBRATION_SAMPLE_COUNT 10U
#define ARM_CALIBRATION_SAMPLE_TIMEOUT_MS 100U
#define ARM_CALIBRATION_CAPTURE_TIMEOUT_MS 100U
enum {
    CAR_KP,
    CAR_KI,
    CAR_KD,
    YAW_KP,
    YAW_KI,
    YAW_KD,
    PITCH_KP,
    PITCH_KI,
    PITCH_KD,
    CAR_TYPE,
    CAR_DEADZONE,
    CAR_LINE,
    CAR_WHEEL_DIAMETER,
    CAR_REDUCTION_RATIO,
    HOST_BAUD,
    CAR_DEBUG_CONTROL_MODE,
    CAR_DEBUG_M1_VALUE,
    CAR_DEBUG_M1_DIRECTION,
    CAR_DEBUG_M2_VALUE,
    CAR_DEBUG_M2_DIRECTION,
    CAR_M1_DIRECTION,
    CAR_M2_DIRECTION,
    ATTITUDE_MODE,
    ATTITUDE_BASE_SPEED,
    ATTITUDE_HEADING_KP,
    ATTITUDE_HEADING_KI,
    ATTITUDE_HEADING_KD,
    ATTITUDE_RATE_KP,
    ATTITUDE_RATE_KI,
    ATTITUDE_RATE_KD,
    ATTITUDE_LADRC_WO,
    ATTITUDE_LADRC_WC,
    ATTITUDE_LADRC_B0,
    ATTITUDE_MAX_RATE,
    ATTITUDE_MAX_DIFF,
    IMU_INVERT_X,
    IMU_INVERT_Y,
    IMU_INVERT_Z,
    STEP_RATE_AMPLITUDE,
    STEP_ANGLE_AMPLITUDE,
    STEP_HOLD_MS,
    STEP_ANGLE_FEEDBACK_INVERTED,
    STEPPER_FF_ENABLE,
    STEPPER_FF_PITCH_GAIN,
    STEPPER_FF_YAW_GAIN,
    STEPPER_FF_FILTER_TAU,
    STEPPER_FF_DEADZONE,
    STEPPER_FF_OUTPUT_LIMIT,
    BOARD_ROLE,
    ARM_LIMIT_MIN,
    ARM_LIMIT_MAX,
    ARM_DEBUG_SPEED,
    ARM_TARGET_POSITION,
    ARM_DEBUG_DIRECTION,
    ARM_DEBUG_ACCEL,
    ARM_DRIVER_MICROSTEP,
    ARM_DEBUG_TIMEOUT_MS,
    ARM_HOME_MODE,
    CAR_LAP_STOP_DELAY_MS,
    ARM_BALANCE_POSITION,
    CAR_LAP_COUNTS_PER_METER,
    TRACKING_LAYER1_WEIGHT,
    TRACKING_LAYER2_WEIGHT,
    TRACKING_LAYER3_WEIGHT,
    TRACKING_LAYER4_WEIGHT,
    IMU_RATE_Z_INVERTED,
    GRAY_SENSOR_ACTIVE_LOW,
    CAR_B0_START_DIFFERENTIAL,
    CAR_B0_DIFFERENTIAL_STEP,
    CAR_B0_LEVEL_COUNT,
    CAR_B0_PULSE_DURATION_MS,
    CAR_B0_SETTLE_DURATION_MS,
    CAR_B0_REPEAT_COUNT,
    ARM_MOTOR_QUERY_HZ,
    ARM_MOTOR_TELEMETRY_HZ,
    ARM_ANGLE_TARGET,
    ARM_ANGLE_KP,
    ARM_ANGLE_KI,
    ARM_ANGLE_KD,
    ARM_ANGLE_DERIVATIVE_FILTER,
    ARM_ANGLE_MAX_SPEED,
    ARM_ANGLE_DEADZONE,
    ARM_ANGLE_DIRECTION_INVERTED,
    ARM_CALIBRATION_LEFT_MOTOR,
    ARM_CALIBRATION_LEFT_PITCH,
    ARM_CALIBRATION_BALANCE_MOTOR,
    ARM_CALIBRATION_BALANCE_PITCH,
    ARM_CALIBRATION_RIGHT_MOTOR,
    ARM_CALIBRATION_RIGHT_PITCH,
    ARM_BALL_POSITION_LOOP_ENABLE,
    ARM_BALL_VELOCITY_LOOP_ENABLE,
    ARM_BALL_TARGET_POSITION,
    ARM_MOTOR_SPEED_KP,
    ARM_MOTOR_SPEED_KI,
    ARM_MOTOR_SPEED_KD,
    CAR_ARC_STRAIGHT_LENGTH,
    CAR_ARC_RADIUS,
    CAR_ARC_LENGTH,
    CAR_ARC_EXIT_YAW,
    CAR_ARC_OPTICAL_LIMIT,
    CAR_ARC_FEEDFORWARD_MIN,
    CAR_ARC_FEEDFORWARD_MAX,
    CAR_ARC_FEEDFORWARD_SLEW,
    CAR_ARC_DIFFERENTIAL_SLEW,
    ATTITUDE_LADRC_TARGET_RATE_SLEW,
    ATTITUDE_LADRC_DIFFERENTIAL_SLEW,
    ARM_PITCH_INVERTED,
    ARM_VISION_KALMAN_ENABLED,
    ARM_VISION_PREDICTION_ENABLED,
    ARM_BALL_TARGET_VELOCITY,
    ARM_MOTOR_POSITION_SPEED_RPM,
    ARM_MOTOR_POSITION_OFFSET_LIMIT_DEG,
    ARM_MOTOR_POSITION_COMMAND_PERIOD_MS,
    ARM_VISION_FIXED_DELAY_MS,
    ARM_VISION_GRAVITY_FEEDFORWARD_GAIN,
    ARM_VISION_BEAM_ANGLE_DEADZONE_DEG,
    ARM_VISION_PREDICTION_MAX_MS,
    ARM_MOTOR_TRAJECTORY_ENABLED,
    ARM_MOTOR_TRAJECTORY_MAX_SPEED_DEG_S,
    ARM_MOTOR_TRAJECTORY_MAX_ACCELERATION_DEG_S2,
    ARM_VISION_KALMAN_MEASUREMENT_NOISE,
    ARM_VISION_KALMAN_PROCESS_NOISE_POSITION,
    ARM_VISION_KALMAN_PROCESS_NOISE_VELOCITY,
    ARM_BALL_VELOCITY_OUTPUT_INVERTED,
    ARM_BALL_BRAKE_FEEDFORWARD_ENABLED,
    ARM_BALL_BRAKE_STOP_TIME_S,
    ARM_BALL_BRAKE_GAIN,
    ARM_BALL_BRAKE_MAX_ACCELERATION_CM_S2,
    ARM_BALL_BRAKE_ANGLE_LIMIT_DEG,
    ARM_BALL_BRAKE_SPEED_DEADZONE_CM_S,
    ARM_BALL_BRAKE_OUTPUT_INVERTED,
    ANO_PARAM_COUNT
};

typedef struct { uint8_t state, source, destination, id, sum, checksum; uint16_t length, index; uint8_t data[ANO_DATA_MAX]; } ano_parser_t;
typedef struct { uint8_t cid[3]; const char *name; const char *info; } ano_command_desc_t;

static const uint16_t arm_parameter_ids[] = {
    ARM_BALL_POSITION_LOOP_ENABLE, ARM_BALL_VELOCITY_LOOP_ENABLE,
    ARM_BALL_TARGET_POSITION, ARM_BALL_TARGET_VELOCITY,
    ARM_ANGLE_KP, ARM_ANGLE_KI, ARM_ANGLE_KD,
    ARM_MOTOR_SPEED_KP, ARM_MOTOR_SPEED_KI, ARM_MOTOR_SPEED_KD,
    ARM_ANGLE_MAX_SPEED, ARM_ANGLE_DERIVATIVE_FILTER,
    ARM_CALIBRATION_LEFT_MOTOR, ARM_CALIBRATION_LEFT_PITCH,
    ARM_CALIBRATION_BALANCE_MOTOR, ARM_CALIBRATION_BALANCE_PITCH,
    ARM_CALIBRATION_RIGHT_MOTOR, ARM_CALIBRATION_RIGHT_PITCH,
    ARM_PITCH_INVERTED, ARM_VISION_KALMAN_ENABLED, ARM_VISION_PREDICTION_ENABLED,
    ARM_MOTOR_POSITION_SPEED_RPM, ARM_MOTOR_POSITION_OFFSET_LIMIT_DEG,
    ARM_MOTOR_POSITION_COMMAND_PERIOD_MS, ARM_VISION_FIXED_DELAY_MS,
    ARM_VISION_GRAVITY_FEEDFORWARD_GAIN, ARM_VISION_BEAM_ANGLE_DEADZONE_DEG,
    ARM_VISION_PREDICTION_MAX_MS, ARM_MOTOR_TRAJECTORY_ENABLED,
    ARM_MOTOR_TRAJECTORY_MAX_SPEED_DEG_S,
    ARM_MOTOR_TRAJECTORY_MAX_ACCELERATION_DEG_S2,
    ARM_VISION_KALMAN_MEASUREMENT_NOISE,
    ARM_VISION_KALMAN_PROCESS_NOISE_POSITION,
    ARM_VISION_KALMAN_PROCESS_NOISE_VELOCITY,
    ARM_BALL_VELOCITY_OUTPUT_INVERTED
    , ARM_BALL_BRAKE_FEEDFORWARD_ENABLED, ARM_BALL_BRAKE_STOP_TIME_S,
    ARM_BALL_BRAKE_GAIN, ARM_BALL_BRAKE_MAX_ACCELERATION_CM_S2,
    ARM_BALL_BRAKE_ANGLE_LIMIT_DEG, ARM_BALL_BRAKE_SPEED_DEADZONE_CM_S,
    ARM_BALL_BRAKE_OUTPUT_INVERTED
};

static const ano_command_desc_t arm_commands[] = {
    {{0x03U, ANO_COMMAND_ARM_CAPTURE_HIGH, 0x00U}, "设置当前为最高点", "采集当前电机绝对位置和有效IMU Pitch并写入Flash"},
    {{0x03U, ANO_COMMAND_ARM_BALL_LOOP_START, 0x00U}, "启动小球闭环", "三点标定、电机位置和视觉有效且双环开启后立即进入闭环"},
    {{0x03U, ANO_COMMAND_ARM_BALL_LOOP_STOP, 0x00U}, "停止小球闭环", "立即撤销闭环授权、停止电机并清零PID"},
    {{0x03U, ANO_COMMAND_ARM_BALL_SPEED_STEP, 0x00U}, "小球速度阶跃", "按当前目标速度幅值执行0、正、0、负、0阶跃"},
    {{0x03U, ANO_COMMAND_ARM_BALL_POSITION_STEP, 0x00U}, "小球位置阶跃", "按位置到达判定执行0、+5cm、-5cm阶跃并最终保持"},
    {{0x03U, ANO_COMMAND_ARM_COMPETITION_SELECT, 0x03U}, "选择竞赛题3", "选择题3后等待竞赛启动命令"},
    {{0x03U, ANO_COMMAND_ARM_COMPETITION_SELECT, 0x04U}, "选择竞赛题4", "选择题4后等待竞赛启动命令"},
    {{0x03U, ANO_COMMAND_ARM_COMPETITION_SELECT, 0x05U}, "选择竞赛题5", "选择题5后等待竞赛启动命令"},
    {{0x03U, ANO_COMMAND_ARM_COMPETITION_SELECT, 0x06U}, "选择竞赛题6", "选择题6后等待竞赛启动命令"},
    {{0x03U, ANO_COMMAND_ARM_COMPETITION_START, 0x00U}, "启动竞赛摆杆", "按已选择题目启动小球闭环；题6锁存启动瞬间位置"},
    {{0x03U, ANO_COMMAND_ARM_COMPETITION_CANCEL, 0x00U}, "停止竞赛摆杆", "撤销小球闭环并使摆杆返回平衡原点"},
    {{0x03U, ANO_COMMAND_ARM_DISABLE, 0x00U}, "关闭保持力矩", "停止并关闭臂杆电机保持力矩"},
    {{0x03U, ANO_COMMAND_ARM_ENABLE, 0x00U}, "开启保持力矩", "开启臂杆电机保持力矩"},
    {{0x03U, ANO_COMMAND_ARM_CAPTURE_LOW, 0x00U}, "设置当前为最低点", "采集当前电机绝对位置和有效IMU Pitch并写入Flash"},
    {{0x03U, ANO_COMMAND_ARM_CAPTURE_BALANCE, 0x00U}, "设置当前为平衡点", "采集当前电机绝对位置和有效IMU Pitch并写入Flash"},
    {{0x03U, ANO_COMMAND_ARM_TELEMETRY_ENABLE, 0x00U}, "开启遥测", "上传臂杆电机实际转速、实际位置和反馈有效状态"},
    {{0x03U, ANO_COMMAND_ARM_TELEMETRY_DISABLE, 0x00U}, "关闭遥测", "停止上传臂杆电机状态"}
};

static const ano_command_desc_t car_commands[] = {
    {{0x03U, 0x03U, 0x00U}, "四驱停止", "将四路电机速度设为零"},
    {{0x03U, ANO_COMMAND_CAR_READ_FLASH, 0x00U}, "读取四驱配置", "请求四路电机驱动读取保存的配置"},
    {{0x03U, ANO_COMMAND_CAR_FACTORY_RESET, 0x00U}, "四驱恢复出厂", "请求四路电机驱动恢复出厂配置"},
    {{0x03U, ANO_COMMAND_CAR_APPLY_CONFIG, 0x00U}, "下发四驱配置", "按类型、死区、相位线、减速比、轮径和PID下发当前配置"},
    {{0x03U, ANO_COMMAND_CAR_DEBUG_ENTER, 0x00U}, "进入M1M2调试", "四路PWM置零后允许M1和M2调试"},
    {{0x03U, ANO_COMMAND_CAR_DEBUG_START, 0x00U}, "启动M1M2调试", "按调试参数仅输出M1和M2，M3和M4为零"},
    {{0x03U, ANO_COMMAND_CAR_DEBUG_STOP, 0x00U}, "停止M1M2调试", "四路PWM置零并退出M1和M2调试模式"},
    {{0x03U, ANO_COMMAND_CAR_RESTORE_DEFAULTS, 0x00U}, "完整恢复默认", "清除本机配置并恢复电机驱动默认设置"},
    {{0x03U, ANO_COMMAND_ATTITUDE_ENABLE, 0x00U}, "启动陀螺仪控制", "启动航向和角速度双环控制"},
    {{0x03U, ANO_COMMAND_ATTITUDE_DISABLE, 0x00U}, "停止陀螺仪控制", "停止双环控制并将M1和M2速度置零"},
    {{0x03U, ANO_COMMAND_RATE_STEP_TEST, 0x00U}, "角速度诊断", "唯一绕过航向角外环的诊断命令，按配置幅度执行角速度阶跃和回零"},
    {{0x03U, ANO_COMMAND_ANGLE_STEP_TEST, 0x00U}, "航向角阶跃测试", "按配置幅度执行相对航向角正阶跃和回零响应"},
    {{0x03U, ANO_COMMAND_STEPPER_FF_CALIBRATE, 0x00U}, "前馈零偏校准", "保持小车静止并重新采集一秒IMU加速度零偏"},
    {{0x03U, ANO_COMMAND_CAR_LAP_START, 0x00U}, "启动一圈停车", "启用循迹并计时，返回A点十字后按补偿停车"},
    {{0x03U, ANO_COMMAND_CAR_LAP_STOP, 0x00U}, "停止一圈任务", "立即停止循迹并冻结本圈时间、里程和平均速度"},
    {{0x03U, ANO_COMMAND_CAR_B0_CALIBRATE, 0x00U}, "b0开环标定", "按b0标定参数执行正反差速脉冲并上传标定遥测"}
};

static uint8_t rx_byte;
static volatile uint8_t rx_queue[ANO_RX_QUEUE_SIZE], rx_head, rx_tail;
static uint8_t tx_queue[ANO_TX_QUEUE_DEPTH][ANO_DATA_MAX + 8U], tx_length[ANO_TX_QUEUE_DEPTH];
static uint8_t response_queue[ANO_RESPONSE_QUEUE_DEPTH][ANO_DATA_MAX + 8U];
static uint8_t response_length[ANO_RESPONSE_QUEUE_DEPTH];
static volatile uint8_t tx_head, tx_tail, tx_count, tx_active, tx_active_response;
static volatile uint8_t response_head, response_tail, response_count;
static ano_parser_t parser;
static float car_pid[3] = {0.8f, 0.06f, 0.5f};
static float car_type = (float)CAR_MOTOR_TYPE_310;
static float car_deadzone = 1600.0f;
static float car_line = 500.0f;
static float car_wheel_diameter = 67.0f;
static float car_reduction_ratio = 30.0f;
static float car_debug_control_mode;
static float car_debug_m1_value;
static float car_debug_m1_direction;
static float car_debug_m2_value;
static float car_debug_m2_direction;
static float car_m1_direction;
static float car_m2_direction = 1.0f;
static float car_b0_start_differential = 100.0f;
static float car_b0_differential_step = 50.0f;
static float car_b0_level_count = 3.0f;
static float car_b0_pulse_duration_ms = 300.0f;
static float car_b0_settle_duration_ms = 1000.0f;
static float car_b0_repeat_count = 3.0f;
static float arm_motor_query_hz = ARM_MOTOR_QUERY_DEFAULT_HZ;
static float arm_motor_telemetry_hz = ARM_MOTOR_TELEMETRY_DEFAULT_HZ;
static float arm_angle_target;
static float arm_calibration_values[6];
static float arm_pitch_inverted;
static uint8_t car_debug_active;
static uint8_t car_b0_calibration_active;
static uint8_t car_b0_calibration_phase;
static uint8_t car_b0_calibration_index;
static uint8_t car_b0_calibration_command_status;
static int16_t car_b0_requested_differential;
static int16_t car_b0_applied_differential;
static uint32_t car_b0_phase_started_ms;
static uint32_t last_telemetry_ms;
static uint32_t last_control_telemetry_ms;
static uint32_t last_vision_telemetry_ms;
static uint32_t last_diagnostic_ms;
static uint32_t last_imu_ms;
static uint32_t last_imu_sequence;
static uint32_t host_baud_rate = ANO_DEFAULT_BAUD_RATE;
static uint32_t pending_baud_rate;
static uint32_t requested_baud_rate;
static uint32_t baud_switch_at_ms;
static uint32_t parameter_session_until_ms;
static uint32_t parameter_save_at_ms;
static uint8_t storage_ready;
static uint8_t storage_operation;
static uint8_t parameter_save_pending;
static float arm_limit_min;
static float arm_limit_max;
static float arm_debug_speed = 10.0f;
static float arm_target_position;
static float arm_debug_direction;
static float arm_debug_accel;
static float arm_driver_microstep = 16.0f;
static float arm_debug_timeout_ms = 500.0f;
static float arm_home_mode;
static float car_lap_stop_delay_ms;
static float arm_balance_position;
static float car_lap_counts_per_meter = 285053.625f;
static float gray_sensor_active_low = 1.0f;
static uint8_t arm_debug_active;
static uint8_t arm_zero_valid;
static uint8_t arm_motion_active;
static uint8_t arm_zero_confirm_pending;
static uint8_t arm_home_active;
static uint8_t arm_limit_configured;
static uint8_t arm_motion_positive;
static uint8_t arm_speed_motion;
static uint32_t arm_motion_deadline_ms;
static uint32_t arm_last_position_request_ms;
static uint32_t arm_zero_confirm_issued_ms;
static uint32_t arm_last_speed_request_ms;
static uint32_t arm_last_home_status_request_ms;
static uint32_t arm_startup_deadline_ms;
static uint32_t arm_startup_move_issued_ms;
static uint8_t arm_point_configured;
static uint8_t arm_home_complete;
static uint8_t arm_home_seen_active;
static uint8_t arm_home_idle_samples;
static uint32_t arm_home_status_processed_ms;
static uint32_t arm_last_motor_query_ms;
static uint32_t arm_last_motor_telemetry_ms;
static uint32_t arm_motor_query_response_baseline_ms;
static uint8_t arm_motor_query_position;
static uint8_t arm_motor_query_pending;
static uint8_t arm_motor_query_pending_position;
static uint8_t arm_motor_telemetry_enabled;
static uint32_t arm_last_motor_status_request_ms;
static uint8_t arm_calibration_status;
static uint8_t arm_calibration_stage;
static uint8_t arm_calibration_captured_mask;
static uint8_t arm_calibration_valid;
static uint16_t arm_calibration_sample_count;
static uint8_t arm_calibration_sample_index;
static uint32_t arm_calibration_last_sequence;
static uint32_t arm_calibration_last_sample_ms;
static uint32_t arm_calibration_sequence;
static float arm_calibration_pitch_sum;
static float arm_calibration_pitch_min;
static float arm_calibration_pitch_max;
static float arm_calibration_pitch_samples[ARM_CALIBRATION_SAMPLE_COUNT];
static float arm_calibration_pending[6];
static uint8_t arm_calibration_pending_mask;
static uint8_t arm_capture_pending;
static uint8_t arm_capture_index;
static uint32_t arm_capture_position_baseline_ms;
static uint32_t arm_capture_requested_ms;
static uint8_t arm_ball_speed_step_active;
static uint8_t arm_ball_speed_step_phase;
static uint32_t arm_ball_speed_step_started_ms;
static float arm_ball_speed_step_amplitude;
static uint8_t arm_ball_position_step_active;
static uint8_t arm_ball_position_step_phase;
static uint32_t arm_ball_position_step_started_ms;
static uint32_t arm_ball_position_step_settled_ms;

static float arm_coordinate_to_motor(float position)
{
    return arm_pitch_inverted != 0.0f ? -position : position;
}

static float arm_coordinate_from_motor(float position)
{
    return arm_coordinate_to_motor(position);
}

static void sync_arm_control_calibration(void)
{
    if (arm_calibration_valid == 0U) return;
    (void)Arm_Angle_Control_Set_Motor_Calibration(arm_calibration_values[0],
                                                  arm_calibration_values[1],
                                                  arm_calibration_values[2],
                                                  arm_calibration_values[3],
                                                  arm_calibration_values[4],
                                                  arm_calibration_values[5]);
}

static void arm_motor_limits(float *minimum, float *maximum)
{
    if (arm_pitch_inverted != 0.0f)
    {
        *minimum = -arm_limit_max;
        *maximum = -arm_limit_min;
    }
    else
    {
        *minimum = arm_limit_min;
        *maximum = arm_limit_max;
    }
}

typedef enum
{
    ARM_STARTUP_IDLE,
    ARM_STARTUP_WAIT_HOME_ACK,
    ARM_STARTUP_WAIT_HOME_DONE,
    ARM_STARTUP_WAIT_ZERO_ACK,
    ARM_STARTUP_MOVE_BALANCE,
    ARM_STARTUP_DONE,
    ARM_STARTUP_FAILED
} arm_startup_state_t;

static arm_startup_state_t arm_startup_state;

typedef struct
{
    float limit_min;
    float limit_max;
    float speed;
    float target_position;
    float direction;
    float acceleration;
    float microstepping;
    float timeout_ms;
    float home_mode;
    float balance_position;
} ano_arm_debug_config_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint32_t baud_rate;
    uint8_t reserved[3];
    float values[HOST_BAUD];
    float directions[2];
    car_attitude_control_config_t attitude;
    stepper_feedforward_config_t feedforward;
    ano_arm_debug_config_t arm_debug;
    float car_lap_counts_per_meter;
    car_tracking_config_t tracking;
    car_lap_arc_config_t arc;
    uint8_t reserved_2[4];
    uint32_t checksum;
} ano_storage_record_t;

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
    float rate_feedback_inverted;
} ano_attitude_config_v17_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint32_t baud_rate;
    uint8_t reserved[3];
    float values[HOST_BAUD];
    float directions[2];
    ano_attitude_config_v17_t attitude;
    stepper_feedforward_config_t feedforward;
    ano_arm_debug_config_t arm_debug;
    float car_lap_counts_per_meter;
    car_tracking_config_t tracking;
    car_lap_arc_config_t arc;
    uint8_t reserved_2[4];
    uint32_t checksum;
} ano_storage_record_v17_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint8_t reserved[3];
    arm_angle_control_config_t config;
    uint32_t checksum;
} ano_arm_angle_storage_record_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint8_t reserved[3];
    struct
    {
        float values[30];
    } config;
    uint32_t checksum;
} ano_arm_angle_storage_record_v7_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint8_t reserved[3];
    struct
    {
        float values[29];
    } config;
    uint32_t checksum;
} ano_arm_angle_storage_record_v6_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint8_t reserved[3];
    struct
    {
        float values[23];
    } config;
    uint32_t checksum;
} ano_arm_angle_storage_record_v5_t;

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
} ano_arm_angle_control_config_v4_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint8_t reserved[3];
    ano_arm_angle_control_config_v4_t config;
    uint32_t checksum;
} ano_arm_angle_storage_record_v4_t;

typedef struct
{
    float ball_position_loop_enabled;
    float ball_velocity_loop_enabled;
    float ball_target_position_cm;
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
} ano_arm_angle_control_config_v3_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint8_t reserved[3];
    ano_arm_angle_control_config_v3_t config;
    uint32_t checksum;
} ano_arm_angle_storage_record_v3_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float derivative_filter_s;
    float maximum_speed_rpm;
    float deadzone_deg;
    float direction_inverted;
    float legacy_control_mode;
    float motor_position_limit;
    float motor_speed_limit;
    float motor_speed_kp;
    float motor_speed_ki;
    float motor_speed_kd;
} ano_arm_angle_control_config_v1_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint8_t reserved[3];
    ano_arm_angle_control_config_v1_t config;
    uint32_t checksum;
} ano_arm_angle_storage_record_v1_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint8_t valid;
    uint8_t reserved[2];
    uint32_t sequence;
    float values[6];
    uint32_t checksum;
} ano_arm_calibration_storage_record_t;

_Static_assert(sizeof(ano_storage_record_t) <= ANO_ARM_ANGLE_STORAGE_ADDRESS,
               "ANO storage records overlap");
_Static_assert(ANO_ARM_CALIBRATION_STORAGE_A % W25QXX_FLASH_SECTOR_SIZE == 0U &&
               ANO_ARM_CALIBRATION_STORAGE_B % W25QXX_FLASH_SECTOR_SIZE == 0U,
               "Arm calibration slots must be sector aligned");

static void put_u16(uint8_t *d, uint16_t *i, uint16_t v) { d[(*i)++] = (uint8_t)v; d[(*i)++] = (uint8_t)(v >> 8U); }
static void put_i16(uint8_t *d, uint16_t *i, int16_t v) { put_u16(d, i, (uint16_t)v); }
static void put_i32(uint8_t *d, uint16_t *i, int32_t v) { memcpy(&d[*i], &v, 4U); *i += 4U; }
static void put_float(uint8_t *d, uint16_t *i, float v) { memcpy(&d[*i], &v, 4U); *i += 4U; }
static uint16_t get_u16(const uint8_t *d) { return (uint16_t)d[0] | ((uint16_t)d[1] << 8U); }
static int16_t get_i16(const uint8_t *d) { return (int16_t)get_u16(d); }
static float get_float(const uint8_t *d) { float v; memcpy(&v, d, 4U); return v; }

static bool baud_rate_is_valid(uint32_t baud_rate)
{
    return baud_rate == 115200U || baud_rate == 230400U || baud_rate == 460800U || baud_rate == 921600U;
}

static void reset_local_defaults(void)
{
    requested_baud_rate = 0U;
    car_pid[0] = 0.8f;
    car_pid[1] = 0.06f;
    car_pid[2] = 0.5f;
    car_type = (float)CAR_MOTOR_TYPE_310;
    car_deadzone = 1600.0f;
    car_line = 500.0f;
    car_wheel_diameter = 67.0f;
    car_reduction_ratio = 30.0f;
    car_debug_control_mode = 0.0f;
    car_debug_m1_value = 0.0f;
    car_debug_m1_direction = 0.0f;
    car_debug_m2_value = 0.0f;
    car_debug_m2_direction = 0.0f;
    car_m1_direction = 0.0f;
    car_m2_direction = 1.0f;
    car_b0_start_differential = 100.0f;
    car_b0_differential_step = 50.0f;
    car_b0_level_count = 3.0f;
    car_b0_pulse_duration_ms = 300.0f;
    car_b0_settle_duration_ms = 1000.0f;
    car_b0_repeat_count = 3.0f;
    arm_motor_query_hz = ARM_MOTOR_QUERY_DEFAULT_HZ;
    arm_motor_telemetry_hz = ARM_MOTOR_TELEMETRY_DEFAULT_HZ;
    arm_angle_target = 0.0f;
    arm_pitch_inverted = 0.0f;
    Arm_Angle_Control_Set_Pitch_Inverted(false);
    (void)Arm_Angle_Control_Set_Target(arm_angle_target);
    (void)Car_Motor_Set_Direction(0U, false);
    (void)Car_Motor_Set_Direction(1U, true);
    car_debug_active = 0U;
    arm_limit_min = 0.0f;
    arm_limit_max = 0.0f;
    arm_debug_speed = 10.0f;
    arm_target_position = 0.0f;
    arm_debug_direction = 0.0f;
    arm_debug_accel = 0.0f;
    arm_driver_microstep = 16.0f;
    arm_debug_timeout_ms = 500.0f;
    arm_home_mode = 0.0f;
    arm_balance_position = 0.0f;
    car_lap_stop_delay_ms = 0.0f;
    car_lap_counts_per_meter = 285053.625f;
    gray_sensor_active_low = 1.0f;
    Gray_Sensor_Set_Active_Low(1U);
    (void)Car_Lap_Task_Set_Stop_Delay(0U);
    (void)Car_Lap_Task_Set_Counts_Per_Meter(car_lap_counts_per_meter);
    arm_debug_active = 0U;
    arm_zero_valid = 0U;
    arm_motion_active = 0U;
    arm_zero_confirm_pending = 0U;
    arm_home_active = 0U;
    arm_limit_configured = 0U;
    arm_motion_positive = 0U;
    arm_speed_motion = 0U;
    arm_point_configured = 0U;
    arm_home_complete = 0U;
    arm_home_seen_active = 0U;
    arm_home_idle_samples = 0U;
    arm_home_status_processed_ms = 0U;
    arm_last_motor_query_ms = 0U;
    arm_last_motor_telemetry_ms = 0U;
    arm_motor_query_response_baseline_ms = 0U;
    arm_motor_query_position = 1U;
    arm_motor_query_pending = 0U;
    arm_motor_query_pending_position = 0U;
    arm_motor_telemetry_enabled = 0U;
    arm_last_motor_status_request_ms = 0U;
    arm_home_status_processed_ms = 0U;
    arm_startup_state = ARM_STARTUP_IDLE;
    arm_startup_move_issued_ms = 0U;
    Car_Attitude_Control_Reset_Defaults();
    Car_Lap_Task_Reset_Arc_Config();
    Stepper_Feedforward_Reset_Defaults();
    if (App_Role_Get() == APP_ROLE_ARM) Arm_Angle_Control_Reset_Defaults();

    pid_kp = 3.0f;
    pid_ki = 1.0f;
    pid_kd = 0.0f;
    pid_tau = 0.05f;
    pid_lim_min = -4.0f;
    pid_lim_max = 4.0f;
    pid_lim_min_int = -5.0f;
    pid_lim_max_int = 5.0f;
    sample_time_s = 0.01f;
    Stepper_PID_Parameters_Init(&yaw_pid);
    Stepper_PID_Parameters_Init(&pitch_pid);
    PIDController_Init(&yaw_pid);
    PIDController_Init(&pitch_pid);
}

static uint32_t storage_record_checksum(const ano_storage_record_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x6D2B79F5U;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++) checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t storage_record_v17_checksum(const ano_storage_record_v17_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x6D2B79F5U;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++)
        checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t arm_angle_storage_checksum(const ano_arm_angle_storage_record_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x13579BDFU;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++)
        checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t arm_angle_storage_v1_checksum(const ano_arm_angle_storage_record_v1_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x13579BDFU;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++)
        checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t arm_angle_storage_v3_checksum(const ano_arm_angle_storage_record_v3_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x13579BDFU;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++)
        checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t arm_angle_storage_v4_checksum(const ano_arm_angle_storage_record_v4_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x13579BDFU;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++)
        checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t arm_angle_storage_v5_checksum(const ano_arm_angle_storage_record_v5_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x13579BDFU;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++)
        checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t arm_angle_storage_v6_checksum(const ano_arm_angle_storage_record_v6_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x13579BDFU;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++)
        checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t arm_angle_storage_v7_checksum(const ano_arm_angle_storage_record_v7_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x13579BDFU;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++)
        checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t arm_calibration_storage_checksum(const ano_arm_calibration_storage_record_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x2468ACE1U;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++)
        checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static bool arm_calibration_record_valid(const ano_arm_calibration_storage_record_t *record)
{
    if (record->magic != ANO_ARM_CALIBRATION_STORAGE_MAGIC ||
        (record->version != 1U && record->version != ANO_ARM_CALIBRATION_STORAGE_VERSION) ||
        record->valid == 0U ||
        record->checksum != arm_calibration_storage_checksum(record)) return false;
    for (uint8_t i = 0U; i < 6U; i++)
        if (!isfinite(record->values[i])) return false;
    return true;
}

static void load_arm_calibration(void)
{
    ano_arm_calibration_storage_record_t a = {0};
    ano_arm_calibration_storage_record_t b = {0};
    bool a_valid = W25Qxx_Read(ANO_ARM_CALIBRATION_STORAGE_A, (uint8_t *)&a, sizeof(a)) &&
                   arm_calibration_record_valid(&a);
    bool b_valid = W25Qxx_Read(ANO_ARM_CALIBRATION_STORAGE_B, (uint8_t *)&b, sizeof(b)) &&
                   arm_calibration_record_valid(&b);
    const ano_arm_calibration_storage_record_t *selected;

    if (!a_valid && !b_valid) return;
    selected = !b_valid || (a_valid && (int32_t)(a.sequence - b.sequence) > 0) ? &a : &b;
    memcpy(arm_calibration_values, selected->values, sizeof(arm_calibration_values));
    memcpy(arm_calibration_pending, selected->values, sizeof(arm_calibration_pending));
    arm_calibration_pending_mask = selected->version == 1U ? 0x3FU : selected->valid;
    arm_calibration_sequence = selected->sequence;
    arm_calibration_valid = arm_calibration_pending_mask == 0x3FU ? 1U : 0U;
    sync_arm_control_calibration();
}

static bool save_arm_calibration(void)
{
    ano_arm_calibration_storage_record_t record = {0};
    ano_arm_calibration_storage_record_t verify = {0};
    uint32_t address;

    if (!storage_ready || arm_calibration_pending_mask == 0U) return false;
    record.magic = ANO_ARM_CALIBRATION_STORAGE_MAGIC;
    record.version = ANO_ARM_CALIBRATION_STORAGE_VERSION;
    record.valid = arm_calibration_pending_mask;
    record.sequence = arm_calibration_sequence + 1U;
    memcpy(record.values, arm_calibration_values, sizeof(record.values));
    record.checksum = arm_calibration_storage_checksum(&record);
    address = (record.sequence & 1U) == 0U ? ANO_ARM_CALIBRATION_STORAGE_A :
                                            ANO_ARM_CALIBRATION_STORAGE_B;
    if (!W25Qxx_EraseSector(address) ||
        !W25Qxx_Write(address, (const uint8_t *)&record, sizeof(record)) ||
        !W25Qxx_Read(address, (uint8_t *)&verify, sizeof(verify)) ||
        memcmp(&record, &verify, sizeof(record)) != 0)
    {
        return false;
    }
    arm_calibration_sequence = record.sequence;
    return true;
}

static bool save_arm_angle_config(void)
{
    ano_arm_angle_storage_record_t record = {0};
    ano_arm_angle_storage_record_t verify = {0};

    record.magic = ANO_ARM_ANGLE_STORAGE_MAGIC;
    record.version = ANO_ARM_ANGLE_STORAGE_VERSION;
    record.reserved[0] = (uint8_t)arm_pitch_inverted;
    record.config = *Arm_Angle_Control_Get_Config();
    record.checksum = arm_angle_storage_checksum(&record);
    return W25Qxx_EraseSector(ANO_ARM_ANGLE_STORAGE_ADDRESS) &&
           W25Qxx_Write(ANO_ARM_ANGLE_STORAGE_ADDRESS, (const uint8_t *)&record, sizeof(record)) &&
           W25Qxx_Read(ANO_ARM_ANGLE_STORAGE_ADDRESS, (uint8_t *)&verify, sizeof(verify)) &&
           memcmp(&record, &verify, sizeof(record)) == 0;
}

static void load_arm_angle_config(void)
{
    ano_arm_angle_storage_record_t record = {0};
    ano_arm_angle_storage_record_v7_t v7 = {0};
    ano_arm_angle_storage_record_v6_t v6 = {0};
    ano_arm_angle_storage_record_v5_t v5 = {0};
    ano_arm_angle_storage_record_v4_t v4 = {0};
    ano_arm_angle_storage_record_v3_t v3 = {0};
    ano_arm_angle_storage_record_v1_t legacy = {0};

    if (W25Qxx_Read(ANO_ARM_ANGLE_STORAGE_ADDRESS, (uint8_t *)&record, sizeof(record)) &&
        record.magic == ANO_ARM_ANGLE_STORAGE_MAGIC &&
        record.version == ANO_ARM_ANGLE_STORAGE_VERSION &&
        record.checksum == arm_angle_storage_checksum(&record) && record.reserved[0] <= 1U)
    {
        arm_pitch_inverted = (float)record.reserved[0];
        Arm_Angle_Control_Set_Pitch_Inverted(arm_pitch_inverted != 0.0f);
        (void)Arm_Angle_Control_Apply_Config(&record.config);
        return;
    }
    if (W25Qxx_Read(ANO_ARM_ANGLE_STORAGE_ADDRESS, (uint8_t *)&v7, sizeof(v7)) &&
        v7.magic == ANO_ARM_ANGLE_STORAGE_MAGIC && v7.version == 7U &&
        v7.checksum == arm_angle_storage_v7_checksum(&v7) && v7.reserved[0] <= 1U)
    {
        arm_angle_control_config_t next = *Arm_Angle_Control_Get_Config();
        memcpy(&next, &v7.config, sizeof(v7.config));
        arm_pitch_inverted = (float)v7.reserved[0];
        Arm_Angle_Control_Set_Pitch_Inverted(arm_pitch_inverted != 0.0f);
        (void)Arm_Angle_Control_Apply_Config(&next);
        return;
    }
    if (W25Qxx_Read(ANO_ARM_ANGLE_STORAGE_LEGACY_ADDRESS, (uint8_t *)&record, sizeof(record)) &&
        record.magic == ANO_ARM_ANGLE_STORAGE_MAGIC &&
        record.version == ANO_ARM_ANGLE_STORAGE_VERSION &&
        record.checksum == arm_angle_storage_checksum(&record) && record.reserved[0] <= 1U)
    {
        arm_pitch_inverted = (float)record.reserved[0];
        Arm_Angle_Control_Set_Pitch_Inverted(arm_pitch_inverted != 0.0f);
        (void)Arm_Angle_Control_Apply_Config(&record.config);
        return;
    }
    if (W25Qxx_Read(ANO_ARM_ANGLE_STORAGE_ADDRESS, (uint8_t *)&v6, sizeof(v6)) &&
        v6.magic == ANO_ARM_ANGLE_STORAGE_MAGIC && v6.version == 6U &&
        v6.checksum == arm_angle_storage_v6_checksum(&v6) && v6.reserved[0] <= 1U)
    {
        arm_angle_control_config_t next = *Arm_Angle_Control_Get_Config();
        memcpy(&next, &v6.config, sizeof(v6.config));
        next.ball_velocity_output_inverted = 0.0f;
        arm_pitch_inverted = (float)v6.reserved[0];
        Arm_Angle_Control_Set_Pitch_Inverted(arm_pitch_inverted != 0.0f);
        (void)Arm_Angle_Control_Apply_Config(&next);
        return;
    }
    if (W25Qxx_Read(ANO_ARM_ANGLE_STORAGE_ADDRESS, (uint8_t *)&v5, sizeof(v5)) &&
        v5.magic == ANO_ARM_ANGLE_STORAGE_MAGIC && v5.version == 5U &&
        v5.checksum == arm_angle_storage_v5_checksum(&v5) && v5.reserved[0] <= 1U)
    {
        arm_angle_control_config_t next = *Arm_Angle_Control_Get_Config();
        memcpy(&next, &v5.config, sizeof(v5.config));
        arm_pitch_inverted = (float)v5.reserved[0];
        Arm_Angle_Control_Set_Pitch_Inverted(arm_pitch_inverted != 0.0f);
        (void)Arm_Angle_Control_Apply_Config(&next);
        return;
    }
    if (W25Qxx_Read(ANO_ARM_ANGLE_STORAGE_ADDRESS, (uint8_t *)&v4, sizeof(v4)) &&
        v4.magic == ANO_ARM_ANGLE_STORAGE_MAGIC && v4.version == 4U &&
        v4.checksum == arm_angle_storage_v4_checksum(&v4) && v4.reserved[0] <= 1U)
    {
        arm_angle_control_config_t next = *Arm_Angle_Control_Get_Config();
        memcpy(&next, &v4.config, sizeof(v4.config));
        arm_pitch_inverted = (float)v4.reserved[0];
        Arm_Angle_Control_Set_Pitch_Inverted(arm_pitch_inverted != 0.0f);
        (void)Arm_Angle_Control_Apply_Config(&next);
        return;
    }
    if (W25Qxx_Read(ANO_ARM_ANGLE_STORAGE_ADDRESS, (uint8_t *)&v3, sizeof(v3)) &&
        v3.magic == ANO_ARM_ANGLE_STORAGE_MAGIC && v3.version == 3U &&
        v3.checksum == arm_angle_storage_v3_checksum(&v3) && v3.reserved[0] <= 1U)
    {
        arm_angle_control_config_t next = *Arm_Angle_Control_Get_Config();
        next.ball_position_loop_enabled = v3.config.ball_position_loop_enabled;
        next.ball_velocity_loop_enabled = v3.config.ball_velocity_loop_enabled;
        next.ball_target_position_cm = v3.config.ball_target_position_cm;
        next.ball_target_velocity_cm_s = 0.0f;
        next.ball_position_kp = v3.config.ball_position_kp;
        next.ball_position_ki = v3.config.ball_position_ki;
        next.ball_position_kd = v3.config.ball_position_kd;
        next.ball_velocity_kp = v3.config.ball_velocity_kp;
        next.ball_velocity_ki = v3.config.ball_velocity_ki;
        next.ball_velocity_kd = v3.config.ball_velocity_kd;
        next.derivative_filter_s = v3.config.derivative_filter_s;
        next.ball_velocity_limit_cm_s = v3.config.ball_velocity_limit_cm_s;
        next.motor_position_offset_limit_deg = v3.config.motor_position_offset_limit_deg;
        next.motor_position_speed_rpm = v3.config.motor_position_speed_rpm;
        next.motor_position_command_threshold_deg = v3.config.motor_position_command_threshold_deg;
        next.motor_position_command_period_ms = v3.config.motor_position_command_period_ms;
        next.direction_inverted = v3.config.direction_inverted;
        next.vision_kalman_enabled = v3.config.vision_kalman_enabled;
        next.vision_prediction_enabled = v3.config.vision_prediction_enabled;
        arm_pitch_inverted = (float)v3.reserved[0];
        Arm_Angle_Control_Set_Pitch_Inverted(arm_pitch_inverted != 0.0f);
        (void)Arm_Angle_Control_Apply_Config(&next);
        return;
    }
    if (!W25Qxx_Read(ANO_ARM_ANGLE_STORAGE_LEGACY_ADDRESS, (uint8_t *)&legacy, sizeof(legacy)) ||
        legacy.magic != ANO_ARM_ANGLE_STORAGE_MAGIC || legacy.version != 1U ||
        legacy.checksum != arm_angle_storage_v1_checksum(&legacy) || legacy.reserved[0] > 1U)
    {
        return;
    }

    arm_angle_control_config_t next = *Arm_Angle_Control_Get_Config();
    next.ball_position_kp = legacy.config.kp;
    next.ball_position_ki = legacy.config.ki;
    next.ball_position_kd = legacy.config.kd;
    next.ball_velocity_kp = legacy.config.motor_speed_kp;
    next.ball_velocity_ki = legacy.config.motor_speed_ki;
    next.ball_velocity_kd = legacy.config.motor_speed_kd;
    next.derivative_filter_s = legacy.config.derivative_filter_s;
    next.ball_velocity_limit_cm_s = legacy.config.maximum_speed_rpm;
    arm_pitch_inverted = (float)legacy.reserved[0];
    Arm_Angle_Control_Set_Pitch_Inverted(arm_pitch_inverted != 0.0f);
    (void)Arm_Angle_Control_Apply_Config(&next);
}

static float *parameter(uint16_t id);
static void send_param(uint16_t protocol_index);
static bool write_car_parameter(uint16_t parameter_id, float value);
static bool write_car_b0_calibration_parameter(uint16_t parameter_id, float value);
static bool apply_car_config(void);
static bool stop_arm_debug(bool exit_debug);
static bool arm_points_valid(void);
static bool parameter_session_active(void);

static bool save_storage(uint32_t baud_rate)
{
    ano_storage_record_t record = {0};
    ano_storage_record_t verify = {0};
    storage_operation = 3U;
    record.magic = ANO_STORAGE_MAGIC;
    record.version = ANO_STORAGE_VERSION;
    record.baud_rate = baud_rate;
    for (uint16_t id = 0U; id < HOST_BAUD; id++)
    {
        float *value = parameter(id);
        if (value != NULL && !isfinite(*value))
        {
            storage_operation = 3U;
            return false;
        }
        if (value != NULL) record.values[id] = *value;
    }
    record.directions[0] = car_m1_direction;
    record.directions[1] = car_m2_direction;
    record.attitude = *Car_Attitude_Control_Get_Config();
    record.feedforward = *Stepper_Feedforward_Get_Config();
    record.tracking = *Car_Attitude_Control_Get_Tracking_Config();
    record.arc = *Car_Lap_Task_Get_Arc_Config();
    record.arm_debug.limit_min = arm_limit_min;
    record.arm_debug.limit_max = arm_limit_max;
    record.arm_debug.speed = arm_debug_speed;
    record.arm_debug.target_position = arm_target_position;
    record.arm_debug.direction = arm_debug_direction;
    record.arm_debug.acceleration = arm_debug_accel;
    record.arm_debug.microstepping = arm_driver_microstep;
    record.arm_debug.timeout_ms = arm_debug_timeout_ms;
    record.arm_debug.home_mode = arm_home_mode;
    record.arm_debug.balance_position = arm_balance_position;
    record.car_lap_counts_per_meter = car_lap_counts_per_meter;
    record.reserved[0] = (uint8_t)gray_sensor_active_low;
    record.reserved[1] = 0xA5U;
    record.checksum = storage_record_checksum(&record);
    if (!W25Qxx_EraseSector(ANO_STORAGE_ADDRESS) ||
        !W25Qxx_Write(ANO_STORAGE_ADDRESS, (const uint8_t *)&record, sizeof(record)) ||
        !W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&verify, sizeof(verify)) ||
        memcmp(&record, &verify, sizeof(record)) != 0)
    {
        return false;
    }
    storage_operation = 0U;
    return true;
}

static void load_storage(void)
{
    ano_storage_record_t record = {0};
    storage_operation = 2U;
    if (!W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&record, sizeof(record)) ||
        record.magic != ANO_STORAGE_MAGIC)
    {
        return;
    }
    if (record.version == 17U)
    {
        ano_storage_record_v17_t legacy = {0};
        if (!W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&legacy, sizeof(legacy)) ||
            !baud_rate_is_valid(legacy.baud_rate) ||
            legacy.checksum != storage_record_v17_checksum(&legacy))
        {
            return;
        }
        memset(&record, 0, sizeof(record));
        record.magic = legacy.magic;
        record.version = ANO_STORAGE_VERSION;
        record.baud_rate = legacy.baud_rate;
        memcpy(record.reserved, legacy.reserved, sizeof(record.reserved));
        memcpy(record.values, legacy.values, sizeof(record.values));
        memcpy(record.directions, legacy.directions, sizeof(record.directions));
        memcpy(&record.attitude, &legacy.attitude, sizeof(legacy.attitude));
        record.attitude.ladrc_target_rate_slew_deg_s2 = 60.0f;
        record.attitude.ladrc_differential_slew_s2 = 250.0f;
        record.feedforward = legacy.feedforward;
        record.arm_debug = legacy.arm_debug;
        record.car_lap_counts_per_meter = legacy.car_lap_counts_per_meter;
        record.tracking = legacy.tracking;
        record.arc = legacy.arc;
        memcpy(record.reserved_2, legacy.reserved_2, sizeof(record.reserved_2));
    }
    else if (record.version != ANO_STORAGE_VERSION ||
             !baud_rate_is_valid(record.baud_rate) ||
             record.checksum != storage_record_checksum(&record))
    {
        return;
    }
    for (uint16_t id = 0U; id < HOST_BAUD; id++)
        {
            float value = record.values[id];
            float *target = parameter(id);
            if (target == NULL || !isfinite(value)) continue;
            *target = value;
        }
        if (record.directions[0] == 0.0f || record.directions[0] == 1.0f) car_m1_direction = record.directions[0];
        if (record.directions[1] == 0.0f || record.directions[1] == 1.0f) car_m2_direction = record.directions[1];
        if (isfinite(record.arm_debug.limit_min) && isfinite(record.arm_debug.balance_position) &&
            isfinite(record.arm_debug.limit_max) && record.arm_debug.limit_min < record.arm_debug.balance_position &&
            record.arm_debug.balance_position < record.arm_debug.limit_max)
        {
            arm_limit_min = record.arm_debug.limit_min;
            arm_limit_max = record.arm_debug.limit_max;
            arm_balance_position = record.arm_debug.balance_position;
            arm_limit_configured = 0x03U;
            arm_point_configured = 0x07U;
        }
        if (isfinite(record.arm_debug.speed) && record.arm_debug.speed > 0.0f && record.arm_debug.speed <= 1000.0f) arm_debug_speed = record.arm_debug.speed;
        if (isfinite(record.arm_debug.target_position)) arm_target_position = record.arm_debug.target_position;
        if (record.arm_debug.direction == 0.0f || record.arm_debug.direction == 1.0f) arm_debug_direction = record.arm_debug.direction;
        if (isfinite(record.arm_debug.acceleration) && record.arm_debug.acceleration >= 0.0f && record.arm_debug.acceleration <= 255.0f && record.arm_debug.acceleration == floorf(record.arm_debug.acceleration)) arm_debug_accel = record.arm_debug.acceleration;
        if (isfinite(record.arm_debug.microstepping) && record.arm_debug.microstepping >= 1.0f && record.arm_debug.microstepping <= 255.0f && record.arm_debug.microstepping == floorf(record.arm_debug.microstepping)) arm_driver_microstep = record.arm_debug.microstepping;
        if (isfinite(record.arm_debug.timeout_ms) && record.arm_debug.timeout_ms >= 100.0f && record.arm_debug.timeout_ms <= 5000.0f && record.arm_debug.timeout_ms == floorf(record.arm_debug.timeout_ms)) arm_debug_timeout_ms = record.arm_debug.timeout_ms;
        if (isfinite(record.arm_debug.home_mode) && record.arm_debug.home_mode >= 0.0f && record.arm_debug.home_mode <= 4.0f && record.arm_debug.home_mode == floorf(record.arm_debug.home_mode)) arm_home_mode = record.arm_debug.home_mode;
        if (isfinite(record.car_lap_counts_per_meter) &&
            Car_Lap_Task_Set_Counts_Per_Meter(record.car_lap_counts_per_meter))
        {
            car_lap_counts_per_meter = record.car_lap_counts_per_meter;
        }
        (void)Car_Attitude_Control_Apply_Config(&record.attitude);
        (void)Car_Attitude_Control_Apply_Tracking_Config(&record.tracking);
        (void)Car_Lap_Task_Apply_Arc_Config(&record.arc);
        if (record.reserved[1] == 0xA5U && record.reserved[0] <= 1U)
        {
            gray_sensor_active_low = (float)record.reserved[0];
            Gray_Sensor_Set_Active_Low(record.reserved[0]);
        }
#if 0
    else if (record.version == 13U)
    {
        ano_storage_record_v13_t legacy = {0};
        if (!W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&legacy, sizeof(legacy)) ||
            legacy.checksum != storage_record_v13_checksum(&legacy))
        {
            return;
        }
        host_baud_rate = legacy.baud_rate;
        for (uint16_t id = 0U; id < HOST_BAUD; id++)
        {
            float *target = parameter(id);
            if (target != NULL && isfinite(legacy.values[id])) *target = legacy.values[id];
        }
        if (legacy.directions[0] == 0.0f || legacy.directions[0] == 1.0f) car_m1_direction = legacy.directions[0];
        if (legacy.directions[1] == 0.0f || legacy.directions[1] == 1.0f) car_m2_direction = legacy.directions[1];
        if (isfinite(legacy.arm_debug.limit_min) && isfinite(legacy.arm_debug.balance_position) &&
            isfinite(legacy.arm_debug.limit_max) && legacy.arm_debug.limit_min < legacy.arm_debug.balance_position &&
            legacy.arm_debug.balance_position < legacy.arm_debug.limit_max)
        {
            arm_limit_min = legacy.arm_debug.limit_min;
            arm_limit_max = legacy.arm_debug.limit_max;
            arm_balance_position = legacy.arm_debug.balance_position;
            arm_limit_configured = 0x03U;
            arm_point_configured = 0x07U;
        }
        if (isfinite(legacy.arm_debug.speed) && legacy.arm_debug.speed > 0.0f && legacy.arm_debug.speed <= 1000.0f) arm_debug_speed = legacy.arm_debug.speed;
        if (isfinite(legacy.arm_debug.target_position)) arm_target_position = legacy.arm_debug.target_position;
        if (legacy.arm_debug.direction == 0.0f || legacy.arm_debug.direction == 1.0f) arm_debug_direction = legacy.arm_debug.direction;
        if (isfinite(legacy.arm_debug.acceleration) && legacy.arm_debug.acceleration >= 0.0f && legacy.arm_debug.acceleration <= 255.0f && legacy.arm_debug.acceleration == floorf(legacy.arm_debug.acceleration)) arm_debug_accel = legacy.arm_debug.acceleration;
        if (isfinite(legacy.arm_debug.microstepping) && legacy.arm_debug.microstepping >= 1.0f && legacy.arm_debug.microstepping <= 255.0f && legacy.arm_debug.microstepping == floorf(legacy.arm_debug.microstepping)) arm_driver_microstep = legacy.arm_debug.microstepping;
        if (isfinite(legacy.arm_debug.timeout_ms) && legacy.arm_debug.timeout_ms >= 100.0f && legacy.arm_debug.timeout_ms <= 5000.0f && legacy.arm_debug.timeout_ms == floorf(legacy.arm_debug.timeout_ms)) arm_debug_timeout_ms = legacy.arm_debug.timeout_ms;
        if (isfinite(legacy.arm_debug.home_mode) && legacy.arm_debug.home_mode >= 0.0f && legacy.arm_debug.home_mode <= 4.0f && legacy.arm_debug.home_mode == floorf(legacy.arm_debug.home_mode)) arm_home_mode = legacy.arm_debug.home_mode;
        if (isfinite(legacy.car_lap_counts_per_meter) &&
            Car_Lap_Task_Set_Counts_Per_Meter(legacy.car_lap_counts_per_meter))
        {
            car_lap_counts_per_meter = legacy.car_lap_counts_per_meter;
        }
        apply_legacy_attitude_config(&legacy.attitude);
        (void)Car_Attitude_Control_Apply_Tracking_Config(&legacy.tracking);
        (void)Car_Lap_Task_Apply_Config(&legacy.lap);
        (void)Stepper_Feedforward_Apply_Config(&legacy.feedforward);
        if (legacy.imu_source <= (uint8_t)CAR_IMU_SOURCE_WHEELTEC)
        {
            imu_source = (float)legacy.imu_source;
            (void)Car_IMU_Set_Source((car_imu_source_t)legacy.imu_source);
        }
    }
    else if (record.version == 12U)
    {
        ano_storage_record_v12_t legacy = {0};
        if (!W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&legacy, sizeof(legacy)) ||
            legacy.checksum != storage_record_v12_checksum(&legacy))
        {
            return;
        }
        host_baud_rate = legacy.baud_rate;
        for (uint16_t id = 0U; id < HOST_BAUD; id++)
        {
            float *target = parameter(id);
            if (target != NULL && isfinite(legacy.values[id])) *target = legacy.values[id];
        }
        if (legacy.directions[0] == 0.0f || legacy.directions[0] == 1.0f) car_m1_direction = legacy.directions[0];
        if (legacy.directions[1] == 0.0f || legacy.directions[1] == 1.0f) car_m2_direction = legacy.directions[1];
        if (isfinite(legacy.car_lap_counts_per_meter) &&
            Car_Lap_Task_Set_Counts_Per_Meter(legacy.car_lap_counts_per_meter))
        {
            car_lap_counts_per_meter = legacy.car_lap_counts_per_meter;
        }
        car_tracking_config_t tracking = *Car_Attitude_Control_Get_Tracking_Config();
        tracking.direction_inverted = legacy.tracking.direction_inverted;
        tracking.error_filter_time_s = legacy.tracking.error_filter_time_s;
        tracking.target_rate_slew_deg_s2 = legacy.tracking.target_rate_slew_deg_s2;
        tracking.line_initial_weight = legacy.tracking.line_initial_weight;
        tracking.line_weight_ramp_time_s = legacy.tracking.line_weight_ramp_time_s;
        tracking.line_release_time_s = legacy.tracking.line_release_time_s;
        tracking.line_heading_rate_deg_s = legacy.tracking.line_heading_rate_deg_s;
        tracking.heading_error_limit_deg = legacy.tracking.heading_error_limit_deg;
        apply_legacy_attitude_config(&legacy.attitude);
        (void)Car_Attitude_Control_Apply_Tracking_Config(&tracking);
        (void)Car_Lap_Task_Apply_Config(&legacy.lap);
        (void)Stepper_Feedforward_Apply_Config(&legacy.feedforward);
        if (legacy.imu_source <= (uint8_t)CAR_IMU_SOURCE_WHEELTEC)
        {
            imu_source = (float)legacy.imu_source;
            (void)Car_IMU_Set_Source((car_imu_source_t)legacy.imu_source);
        }
    }
    else if (record.version == 11U)
    {
        ano_storage_record_v11_t legacy = {0};
        if (!W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&legacy, sizeof(legacy)) ||
            legacy.checksum != storage_record_v11_checksum(&legacy))
        {
            return;
        }
        for (uint16_t id = 0U; id < HOST_BAUD; id++)
        {
            float *target = parameter(id);
            if (target != NULL && isfinite(legacy.values[id])) *target = legacy.values[id];
        }
        if (legacy.directions[0] == 0.0f || legacy.directions[0] == 1.0f) car_m1_direction = legacy.directions[0];
        if (legacy.directions[1] == 0.0f || legacy.directions[1] == 1.0f) car_m2_direction = legacy.directions[1];
        if (isfinite(legacy.car_lap_counts_per_meter) &&
            Car_Lap_Task_Set_Counts_Per_Meter(legacy.car_lap_counts_per_meter))
        {
            car_lap_counts_per_meter = legacy.car_lap_counts_per_meter;
        }
        car_tracking_config_t tracking = *Car_Attitude_Control_Get_Tracking_Config();
        tracking.direction_inverted = legacy.tracking.direction_inverted;
        tracking.error_filter_time_s = legacy.tracking.error_filter_time_s;
        tracking.target_rate_slew_deg_s2 = legacy.tracking.target_rate_slew_deg_s2;
        apply_legacy_attitude_config(&legacy.attitude);
        (void)Car_Attitude_Control_Apply_Tracking_Config(&tracking);
        (void)Stepper_Feedforward_Apply_Config(&legacy.feedforward);
        if (legacy.imu_source <= (uint8_t)CAR_IMU_SOURCE_WHEELTEC)
        {
            imu_source = (float)legacy.imu_source;
            (void)Car_IMU_Set_Source((car_imu_source_t)legacy.imu_source);
        }
    }
    else if (record.version == 10U)
    {
        ano_storage_record_v10_t legacy = {0};
        if (!W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&legacy, sizeof(legacy)) ||
            legacy.checksum != storage_record_v10_checksum(&legacy))
        {
            return;
        }
        for (uint16_t id = 0U; id < HOST_BAUD; id++)
        {
            float *target = parameter(id);
            if (target != NULL && isfinite(legacy.values[id])) *target = legacy.values[id];
        }
        if (legacy.directions[0] == 0.0f || legacy.directions[0] == 1.0f) car_m1_direction = legacy.directions[0];
        if (legacy.directions[1] == 0.0f || legacy.directions[1] == 1.0f) car_m2_direction = legacy.directions[1];
        if (isfinite(legacy.car_lap_counts_per_meter) &&
            Car_Lap_Task_Set_Counts_Per_Meter(legacy.car_lap_counts_per_meter))
        {
            car_lap_counts_per_meter = legacy.car_lap_counts_per_meter;
        }
        car_tracking_config_t tracking = *Car_Attitude_Control_Get_Tracking_Config();
        tracking.direction_inverted = legacy.tracking.direction_inverted;
        tracking.error_filter_time_s = legacy.tracking.error_filter_time_s;
        tracking.target_rate_slew_deg_s2 = legacy.tracking.target_rate_slew_deg_s2;
        apply_legacy_attitude_config(&legacy.attitude);
        (void)Car_Attitude_Control_Apply_Tracking_Config(&tracking);
        (void)Stepper_Feedforward_Apply_Config(&legacy.feedforward);
    }
    else if (record.version == 9U)
    {
        ano_storage_record_v9_t legacy = {0};
        if (!W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&legacy, sizeof(legacy)) ||
            legacy.checksum != storage_record_v9_checksum(&legacy))
        {
            return;
        }
        for (uint16_t id = 0U; id < HOST_BAUD; id++)
        {
            float *target = parameter(id);
            if (target != NULL && isfinite(legacy.values[id])) *target = legacy.values[id];
        }
        if (legacy.directions[0] == 0.0f || legacy.directions[0] == 1.0f) car_m1_direction = legacy.directions[0];
        if (legacy.directions[1] == 0.0f || legacy.directions[1] == 1.0f) car_m2_direction = legacy.directions[1];
        if (isfinite(legacy.arm_debug.limit_min) && isfinite(legacy.arm_debug.balance_position) &&
            isfinite(legacy.arm_debug.limit_max) && legacy.arm_debug.limit_min < legacy.arm_debug.balance_position &&
            legacy.arm_debug.balance_position < legacy.arm_debug.limit_max)
        {
            arm_limit_min = legacy.arm_debug.limit_min;
            arm_limit_max = legacy.arm_debug.limit_max;
            arm_balance_position = legacy.arm_debug.balance_position;
            arm_limit_configured = 0x03U;
            arm_point_configured = 0x07U;
        }
        if (isfinite(legacy.arm_debug.speed) && legacy.arm_debug.speed > 0.0f && legacy.arm_debug.speed <= 1000.0f) arm_debug_speed = legacy.arm_debug.speed;
        if (isfinite(legacy.arm_debug.target_position)) arm_target_position = legacy.arm_debug.target_position;
        if (legacy.arm_debug.direction == 0.0f || legacy.arm_debug.direction == 1.0f) arm_debug_direction = legacy.arm_debug.direction;
        if (isfinite(legacy.arm_debug.acceleration) && legacy.arm_debug.acceleration >= 0.0f && legacy.arm_debug.acceleration <= 255.0f) arm_debug_accel = legacy.arm_debug.acceleration;
        if (isfinite(legacy.arm_debug.microstepping) && legacy.arm_debug.microstepping >= 1.0f && legacy.arm_debug.microstepping <= 255.0f) arm_driver_microstep = legacy.arm_debug.microstepping;
        if (isfinite(legacy.arm_debug.timeout_ms) && legacy.arm_debug.timeout_ms >= 100.0f && legacy.arm_debug.timeout_ms <= 5000.0f) arm_debug_timeout_ms = legacy.arm_debug.timeout_ms;
        if (isfinite(legacy.arm_debug.home_mode) && legacy.arm_debug.home_mode >= 0.0f && legacy.arm_debug.home_mode <= 4.0f) arm_home_mode = legacy.arm_debug.home_mode;
        if (isfinite(legacy.car_lap_counts_per_meter) &&
            Car_Lap_Task_Set_Counts_Per_Meter(legacy.car_lap_counts_per_meter))
        {
            car_lap_counts_per_meter = legacy.car_lap_counts_per_meter;
        }
        apply_legacy_attitude_config(&legacy.attitude);
        (void)Stepper_Feedforward_Apply_Config(&legacy.feedforward);
    }
    else if (record.version == 8U)
    {
        ano_storage_record_v8_t legacy = {0};
        if (!W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&legacy, sizeof(legacy)) ||
            legacy.checksum != storage_record_v8_checksum(&legacy))
        {
            return;
        }
        for (uint16_t id = 0U; id < HOST_BAUD; id++)
        {
            float *target = parameter(id);
            if (target != NULL && isfinite(legacy.values[id])) *target = legacy.values[id];
        }
        if (legacy.directions[0] == 0.0f || legacy.directions[0] == 1.0f) car_m1_direction = legacy.directions[0];
        if (legacy.directions[1] == 0.0f || legacy.directions[1] == 1.0f) car_m2_direction = legacy.directions[1];
        if (isfinite(legacy.arm_debug.limit_min) && isfinite(legacy.arm_debug.balance_position) &&
            isfinite(legacy.arm_debug.limit_max) && legacy.arm_debug.limit_min < legacy.arm_debug.balance_position &&
            legacy.arm_debug.balance_position < legacy.arm_debug.limit_max)
        {
            arm_limit_min = legacy.arm_debug.limit_min;
            arm_limit_max = legacy.arm_debug.limit_max;
            arm_balance_position = legacy.arm_debug.balance_position;
            arm_limit_configured = 0x03U;
            arm_point_configured = 0x07U;
        }
        if (isfinite(legacy.arm_debug.speed) && legacy.arm_debug.speed > 0.0f && legacy.arm_debug.speed <= 1000.0f) arm_debug_speed = legacy.arm_debug.speed;
        if (isfinite(legacy.arm_debug.target_position)) arm_target_position = legacy.arm_debug.target_position;
        if (legacy.arm_debug.direction == 0.0f || legacy.arm_debug.direction == 1.0f) arm_debug_direction = legacy.arm_debug.direction;
        if (isfinite(legacy.arm_debug.acceleration) && legacy.arm_debug.acceleration >= 0.0f && legacy.arm_debug.acceleration <= 255.0f) arm_debug_accel = legacy.arm_debug.acceleration;
        if (isfinite(legacy.arm_debug.microstepping) && legacy.arm_debug.microstepping >= 1.0f && legacy.arm_debug.microstepping <= 255.0f) arm_driver_microstep = legacy.arm_debug.microstepping;
        if (isfinite(legacy.arm_debug.timeout_ms) && legacy.arm_debug.timeout_ms >= 100.0f && legacy.arm_debug.timeout_ms <= 5000.0f) arm_debug_timeout_ms = legacy.arm_debug.timeout_ms;
        if (isfinite(legacy.arm_debug.home_mode) && legacy.arm_debug.home_mode >= 0.0f && legacy.arm_debug.home_mode <= 4.0f) arm_home_mode = legacy.arm_debug.home_mode;
        apply_legacy_attitude_config(&legacy.attitude);
        (void)Stepper_Feedforward_Apply_Config(&legacy.feedforward);
    }
    else if (record.version == 7U)
    {
        ano_storage_record_v7_t legacy = {0};
        if (!W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&legacy, sizeof(legacy)) ||
            legacy.checksum != storage_record_v7_checksum(&legacy))
        {
            return;
        }
        for (uint16_t id = 0U; id < HOST_BAUD; id++)
        {
            float *target = parameter(id);
            if (target != NULL && isfinite(legacy.values[id])) *target = legacy.values[id];
        }
        if (legacy.directions[0] == 0.0f || legacy.directions[0] == 1.0f) car_m1_direction = legacy.directions[0];
        if (legacy.directions[1] == 0.0f || legacy.directions[1] == 1.0f) car_m2_direction = legacy.directions[1];
        if (isfinite(legacy.arm_debug.speed) && legacy.arm_debug.speed > 0.0f && legacy.arm_debug.speed <= 1000.0f) arm_debug_speed = legacy.arm_debug.speed;
        if (isfinite(legacy.arm_debug.acceleration) && legacy.arm_debug.acceleration >= 0.0f && legacy.arm_debug.acceleration <= 255.0f) arm_debug_accel = legacy.arm_debug.acceleration;
        if (isfinite(legacy.arm_debug.timeout_ms) && legacy.arm_debug.timeout_ms >= 100.0f && legacy.arm_debug.timeout_ms <= 5000.0f) arm_debug_timeout_ms = legacy.arm_debug.timeout_ms;
        if (isfinite(legacy.arm_debug.home_mode) && legacy.arm_debug.home_mode >= 0.0f && legacy.arm_debug.home_mode <= 3.0f) arm_home_mode = legacy.arm_debug.home_mode;
        apply_legacy_attitude_config(&legacy.attitude);
        (void)Stepper_Feedforward_Apply_Config(&legacy.feedforward);
    }
    else if (record.version == 6U)
    {
        ano_storage_record_v6_t legacy = {0};
        if (!W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&legacy, sizeof(legacy)) ||
            legacy.checksum != storage_record_v6_checksum(&legacy))
        {
            return;
        }
        for (uint16_t id = 0U; id < HOST_BAUD; id++)
        {
            float *target = parameter(id);
            if (target != NULL && isfinite(legacy.values[id])) *target = legacy.values[id];
        }
        if (legacy.directions[0] == 0.0f || legacy.directions[0] == 1.0f) car_m1_direction = legacy.directions[0];
        if (legacy.directions[1] == 0.0f || legacy.directions[1] == 1.0f) car_m2_direction = legacy.directions[1];
        apply_legacy_attitude_config(&legacy.attitude);
        (void)Stepper_Feedforward_Apply_Config(&legacy.feedforward);
    }
    else if (record.version == 5U)
    {
        ano_storage_record_v5_t legacy = {0};
        if (!W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&legacy, sizeof(legacy)) ||
            legacy.checksum != storage_record_v5_checksum(&legacy))
        {
            return;
        }
        for (uint16_t id = 0U; id < HOST_BAUD; id++)
        {
            float *target = parameter(id);
            if (target != NULL && isfinite(legacy.values[id])) *target = legacy.values[id];
        }
        if (legacy.directions[0] == 0.0f || legacy.directions[0] == 1.0f) car_m1_direction = legacy.directions[0];
        if (legacy.directions[1] == 0.0f || legacy.directions[1] == 1.0f) car_m2_direction = legacy.directions[1];
        apply_legacy_attitude_config(&legacy.attitude);
    }
    else if (record.version == 4U)
    {
        ano_storage_record_v4_t legacy = {0};
        car_attitude_control_config_t attitude = *Car_Attitude_Control_Get_Config();
        if (!W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&legacy, sizeof(legacy)) ||
            legacy.checksum != storage_record_v4_checksum(&legacy))
        {
            return;
        }
        for (uint16_t id = 0U; id < HOST_BAUD; id++)
        {
            float *target = parameter(id);
            if (target != NULL && isfinite(legacy.values[id])) *target = legacy.values[id];
        }
        if (legacy.directions[0] == 0.0f || legacy.directions[0] == 1.0f) car_m1_direction = legacy.directions[0];
        if (legacy.directions[1] == 0.0f || legacy.directions[1] == 1.0f) car_m2_direction = legacy.directions[1];
        memcpy(&attitude, &legacy.attitude, sizeof(legacy.attitude));
        attitude.step_rate_amplitude = legacy.step_rate_amplitude;
        attitude.step_angle_amplitude = legacy.step_angle_amplitude;
        attitude.step_hold_ms = legacy.step_hold_ms;
        (void)Car_Attitude_Control_Apply_Config(&attitude);
    }
    else if (record.version == 3U)
    {
        ano_storage_record_v3_t legacy = {0};
        car_attitude_control_config_t attitude = *Car_Attitude_Control_Get_Config();
        if (!W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&legacy, sizeof(legacy)) ||
            legacy.checksum != storage_record_v3_checksum(&legacy))
        {
            return;
        }
        for (uint16_t id = 0U; id < HOST_BAUD; id++)
        {
            float *target = parameter(id);
            if (target != NULL && isfinite(legacy.values[id])) *target = legacy.values[id];
        }
        if (legacy.directions[0] == 0.0f || legacy.directions[0] == 1.0f) car_m1_direction = legacy.directions[0];
        if (legacy.directions[1] == 0.0f || legacy.directions[1] == 1.0f) car_m2_direction = legacy.directions[1];
        memcpy(&attitude, &legacy.attitude, sizeof(legacy.attitude));
        (void)Car_Attitude_Control_Apply_Config(&attitude);
    }
    else if (record.version == 2U)
    {
        ano_storage_record_v2_t legacy = {0};
        if (!W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&legacy, sizeof(legacy)) ||
            legacy.checksum != storage_record_v2_checksum(&legacy))
        {
            return;
        }
        for (uint16_t id = 0U; id < HOST_BAUD; id++)
        {
            float *target = parameter(id);
            if (target != NULL && isfinite(legacy.values[id])) *target = legacy.values[id];
        }
        if (legacy.directions[0] == 0.0f || legacy.directions[0] == 1.0f) car_m1_direction = legacy.directions[0];
        if (legacy.directions[1] == 0.0f || legacy.directions[1] == 1.0f) car_m2_direction = legacy.directions[1];
    }
    else
    {
        ano_storage_record_v1_t legacy = {0};
        if (record.version != 1U || !W25Qxx_Read(ANO_STORAGE_ADDRESS, (uint8_t *)&legacy, sizeof(legacy)) ||
            legacy.checksum != storage_record_v1_checksum(&legacy))
        {
            return;
        }
        for (uint16_t id = 0U; id < HOST_BAUD; id++)
        {
            float value = legacy.values[id];
            float *target = parameter(id);
            if (target == NULL || !isfinite(value)) continue;
            *target = value;
        }
    }
#endif
    host_baud_rate = record.baud_rate;
    (void)Car_Motor_Set_Direction(0U, car_m1_direction != 0.0f);
    (void)Car_Motor_Set_Direction(1U, car_m2_direction != 0.0f);
    if (record.version == ANO_STORAGE_VERSION)
    {
        (void)Car_Attitude_Control_Apply_Config(&record.attitude);
        (void)Stepper_Feedforward_Apply_Config(&record.feedforward);
    }
    storage_operation = 0U;
}

static void apply_baud_rate(uint32_t baud_rate)
{
    if (HAL_UART_DeInit(&huart1) != HAL_OK) return;
    huart1.Init.BaudRate = baud_rate;
    if (HAL_UART_Init(&huart1) != HAL_OK || HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK || HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    host_baud_rate = baud_rate;
    (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

static void start_tx(void)
{
    uint8_t *frame;
    uint8_t length;

    if (tx_active) return;
    if (response_count != 0U)
    {
        tx_active_response = 1U;
        frame = response_queue[response_head];
        length = response_length[response_head];
    }
    else if (tx_count != 0U)
    {
        if (parameter_session_active()) return;
        tx_active_response = 0U;
        frame = tx_queue[tx_head];
        length = tx_length[tx_head];
    }
    else
    {
        return;
    }
    tx_active = 1U;
    if (HAL_UART_Transmit_IT(&huart1, frame, length) != HAL_OK) tx_active = 0U;
}

static bool tx_has_capacity(uint8_t slots)
{
    uint32_t primask = __get_PRIMASK();
    bool available;

    __disable_irq();
    available = slots <= (uint8_t)(ANO_TX_QUEUE_DEPTH - tx_count);
    __set_PRIMASK(primask);
    return available;
}

static void send_frame_to(uint8_t destination, uint8_t id, const uint8_t *data, uint16_t length)
{
    bool response = id == ANO_ID_CHECK ||
                    (id >= ANO_ID_COMMAND_QUERY && id <= ANO_ID_DEVICE_INFO) ||
                    id == ANO_ID_PARAM_COMMAND || id == ANO_ID_PARAM_VALUE ||
                    id == ANO_ID_PARAM_INFO;
    uint8_t sum = 0U, add = 0U; uint16_t index = 0U; uint32_t primask;
    uint8_t *frame;
    if (length > ANO_DATA_MAX) return;
    primask = __get_PRIMASK(); __disable_irq();
    if ((response && response_count >= ANO_RESPONSE_QUEUE_DEPTH) ||
        (!response && tx_count >= ANO_TX_QUEUE_DEPTH))
    {
        __set_PRIMASK(primask);
        return;
    }
    frame = response ? response_queue[response_tail] : tx_queue[tx_tail];
    frame[index++] = ANO_HEAD; frame[index++] = App_Role_Get_ANO_Address(); frame[index++] = destination; frame[index++] = id; frame[index++] = (uint8_t)length; frame[index++] = (uint8_t)(length >> 8U);
    if (length != 0U)
    {
        memcpy(&frame[index], data, length);
    }
    index += length;
    for (uint16_t i = 0U; i < index; i++) { sum += frame[i]; add += sum; }
    frame[index++] = sum;
    frame[index++] = add;
    if (response)
    {
        response_length[response_tail] = (uint8_t)index;
        response_tail = (response_tail + 1U) % ANO_RESPONSE_QUEUE_DEPTH;
        response_count++;
    }
    else
    {
        tx_length[tx_tail] = (uint8_t)index;
        tx_tail = (tx_tail + 1U) % ANO_TX_QUEUE_DEPTH;
        tx_count++;
    }
    start_tx();
    __set_PRIMASK(primask);
}

static void send_frame(uint8_t id, const uint8_t *data, uint16_t length)
{
    send_frame_to(ANO_HOST_ADDRESS, id, data, length);
}

static float *parameter(uint16_t id)
{
    if (id <= CAR_REDUCTION_RATIO ||
        (id > HOST_BAUD && id < CAR_M1_DIRECTION) ||
        (id >= ARM_DEBUG_SPEED && id <= ARM_HOME_MODE) ||
        id == ARM_MOTOR_QUERY_HZ || id == ARM_MOTOR_TELEMETRY_HZ ||
        id == ARM_ANGLE_TARGET)
    {
        return NULL;
    }
    car_attitude_control_config_t *attitude = Car_Attitude_Control_Get_Config();
    stepper_feedforward_config_t *feedforward = Stepper_Feedforward_Get_Config();
    arm_angle_control_config_t *arm_angle = Arm_Angle_Control_Get_Config();
    static float *const values[ANO_PARAM_COUNT] = {
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL,
        NULL, NULL, NULL, NULL, NULL,
        NULL,
        &car_m1_direction, &car_m2_direction,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL,
         &arm_limit_min, &arm_limit_max, &arm_debug_speed, &arm_target_position,
         &arm_debug_direction, &arm_debug_accel, &arm_driver_microstep,
         &arm_debug_timeout_ms, &arm_home_mode, &car_lap_stop_delay_ms,
        &arm_balance_position, &car_lap_counts_per_meter,
        NULL, NULL, NULL, NULL, NULL, &gray_sensor_active_low,
        &car_b0_start_differential, &car_b0_differential_step, &car_b0_level_count,
        &car_b0_pulse_duration_ms, &car_b0_settle_duration_ms, &car_b0_repeat_count,
         NULL, NULL, &arm_angle_target, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
         &arm_calibration_values[0], &arm_calibration_values[1],
         &arm_calibration_values[2], &arm_calibration_values[3],
         &arm_calibration_values[4], &arm_calibration_values[5],
         NULL, NULL, NULL, NULL, NULL, NULL
    };
    if (id >= ATTITUDE_MODE && id <= STEP_ANGLE_FEEDBACK_INVERTED)
    {
        float *const attitude_values[] = {
            &attitude->mode, &attitude->base_speed,
            &attitude->heading_kp, &attitude->heading_ki, &attitude->heading_kd,
            &attitude->rate_kp, &attitude->rate_ki, &attitude->rate_kd,
            &attitude->ladrc_observer_bandwidth, &attitude->ladrc_controller_bandwidth,
            &attitude->ladrc_plant_gain, &attitude->max_rate, &attitude->max_differential,
            &attitude->invert_x, &attitude->invert_y, &attitude->invert_z,
            &attitude->step_rate_amplitude, &attitude->step_angle_amplitude,
            &attitude->step_hold_ms, &attitude->step_angle_feedback_inverted
        };
        return attitude_values[id - ATTITUDE_MODE];
    }
    if (id >= STEPPER_FF_ENABLE && id <= STEPPER_FF_OUTPUT_LIMIT)
    {
        float *const feedforward_values[] = {
            &feedforward->enabled, &feedforward->pitch_gain, &feedforward->yaw_gain,
            &feedforward->filter_time_constant_s, &feedforward->deadzone_g,
            &feedforward->output_limit_rpm
        };
        return feedforward_values[id - STEPPER_FF_ENABLE];
    }
    if (id >= TRACKING_LAYER1_WEIGHT && id <= TRACKING_LAYER4_WEIGHT)
    {
        car_tracking_config_t *tracking = Car_Attitude_Control_Get_Tracking_Config();
        float *const tracking_values[] = {
            &tracking->layer1_weight_deg_s, &tracking->layer2_weight_deg_s,
            &tracking->layer3_weight_deg_s, &tracking->layer4_weight_deg_s
        };
        return tracking_values[id - TRACKING_LAYER1_WEIGHT];
    }
    if (id == IMU_RATE_Z_INVERTED) return &attitude->rate_feedback_inverted;
    if (id == ATTITUDE_LADRC_TARGET_RATE_SLEW)
        return &attitude->ladrc_target_rate_slew_deg_s2;
    if (id == ATTITUDE_LADRC_DIFFERENTIAL_SLEW)
        return &attitude->ladrc_differential_slew_s2;
    if (id == ARM_BALL_POSITION_LOOP_ENABLE) return &arm_angle->ball_position_loop_enabled;
    if (id == ARM_BALL_VELOCITY_LOOP_ENABLE) return &arm_angle->ball_velocity_loop_enabled;
    if (id == ARM_BALL_TARGET_POSITION) return &arm_angle->ball_target_position_cm;
    if (id == ARM_BALL_TARGET_VELOCITY) return &arm_angle->ball_target_velocity_cm_s;
    if (id == ARM_MOTOR_POSITION_SPEED_RPM) return &arm_angle->motor_position_speed_rpm;
    if (id == ARM_MOTOR_POSITION_OFFSET_LIMIT_DEG) return &arm_angle->motor_position_offset_limit_deg;
    if (id == ARM_MOTOR_POSITION_COMMAND_PERIOD_MS) return &arm_angle->motor_position_command_period_ms;
    if (id == ARM_VISION_FIXED_DELAY_MS) return &arm_angle->vision_fixed_delay_ms;
    if (id == ARM_VISION_GRAVITY_FEEDFORWARD_GAIN) return &arm_angle->vision_gravity_feedforward_gain;
    if (id == ARM_VISION_BEAM_ANGLE_DEADZONE_DEG) return &arm_angle->vision_beam_angle_deadzone_deg;
    if (id == ARM_VISION_PREDICTION_MAX_MS) return &arm_angle->vision_prediction_max_ms;
    if (id == ARM_MOTOR_TRAJECTORY_ENABLED) return &arm_angle->motor_trajectory_enabled;
    if (id == ARM_MOTOR_TRAJECTORY_MAX_SPEED_DEG_S) return &arm_angle->motor_trajectory_max_speed_deg_s;
    if (id == ARM_MOTOR_TRAJECTORY_MAX_ACCELERATION_DEG_S2) return &arm_angle->motor_trajectory_max_acceleration_deg_s2;
    if (id == ARM_VISION_KALMAN_MEASUREMENT_NOISE) return &arm_angle->vision_kalman_measurement_noise;
    if (id == ARM_VISION_KALMAN_PROCESS_NOISE_POSITION) return &arm_angle->vision_kalman_process_noise_position;
    if (id == ARM_VISION_KALMAN_PROCESS_NOISE_VELOCITY) return &arm_angle->vision_kalman_process_noise_velocity;
    if (id == ARM_BALL_VELOCITY_OUTPUT_INVERTED) return &arm_angle->ball_velocity_output_inverted;
    if (id == ARM_BALL_BRAKE_FEEDFORWARD_ENABLED) return &arm_angle->ball_brake_feedforward_enabled;
    if (id == ARM_BALL_BRAKE_STOP_TIME_S) return &arm_angle->ball_brake_stop_time_s;
    if (id == ARM_BALL_BRAKE_GAIN) return &arm_angle->ball_brake_gain;
    if (id == ARM_BALL_BRAKE_MAX_ACCELERATION_CM_S2) return &arm_angle->ball_brake_max_acceleration_cm_s2;
    if (id == ARM_BALL_BRAKE_ANGLE_LIMIT_DEG) return &arm_angle->ball_brake_angle_limit_deg;
    if (id == ARM_BALL_BRAKE_SPEED_DEADZONE_CM_S) return &arm_angle->ball_brake_speed_deadzone_cm_s;
    if (id == ARM_BALL_BRAKE_OUTPUT_INVERTED) return &arm_angle->ball_brake_output_inverted;
    if (id == ARM_MOTOR_SPEED_KP) return &arm_angle->ball_velocity_kp;
    if (id == ARM_MOTOR_SPEED_KI) return &arm_angle->ball_velocity_ki;
    if (id == ARM_MOTOR_SPEED_KD) return &arm_angle->ball_velocity_kd;
    if (id >= ARM_CALIBRATION_LEFT_MOTOR && id <= ARM_CALIBRATION_RIGHT_PITCH)
        return &arm_calibration_values[id - ARM_CALIBRATION_LEFT_MOTOR];
    if (id == ARM_PITCH_INVERTED) return &arm_pitch_inverted;
    if (id == ARM_VISION_KALMAN_ENABLED) return &arm_angle->vision_kalman_enabled;
    if (id == ARM_VISION_PREDICTION_ENABLED) return &arm_angle->vision_prediction_enabled;
    if (id >= ARM_ANGLE_KP && id <= ARM_ANGLE_DIRECTION_INVERTED)
    {
        float *const arm_angle_values[] = {
            &arm_angle->ball_position_kp, &arm_angle->ball_position_ki,
            &arm_angle->ball_position_kd, &arm_angle->derivative_filter_s,
            &arm_angle->ball_velocity_limit_cm_s,
            &arm_angle->motor_position_offset_limit_deg,
            &arm_angle->direction_inverted
        };
        return arm_angle_values[id - ARM_ANGLE_KP];
    }
    return id < ANO_PARAM_COUNT ? values[id] : NULL;
}

static bool write_feedforward_parameter(uint16_t parameter_id, float value)
{
    stepper_feedforward_config_t next = *Stepper_Feedforward_Get_Config();

    if (!isfinite(value)) return false;
    switch (parameter_id)
    {
    case STEPPER_FF_ENABLE: next.enabled = value; break;
    case STEPPER_FF_PITCH_GAIN: next.pitch_gain = value; break;
    case STEPPER_FF_YAW_GAIN: next.yaw_gain = value; break;
    case STEPPER_FF_FILTER_TAU: next.filter_time_constant_s = value; break;
    case STEPPER_FF_DEADZONE: next.deadzone_g = value; break;
    case STEPPER_FF_OUTPUT_LIMIT: next.output_limit_rpm = value; break;
    default: return false;
    }
    return Stepper_Feedforward_Apply_Config(&next);
}

static bool arm_points_valid(void)
{
    return arm_point_configured == 0x07U && isfinite(arm_limit_min) &&
           isfinite(arm_balance_position) && isfinite(arm_limit_max) &&
           arm_limit_min < 0.0f && arm_limit_max > 0.0f &&
           arm_limit_min < arm_balance_position && arm_balance_position < arm_limit_max;
}

static uint32_t arm_motor_query_period_ms(void)
{
    uint32_t period_ms = (uint32_t)(1000.0f / arm_motor_query_hz);
    return period_ms == 0U ? 1U : period_ms;
}

static uint32_t arm_motor_feedback_timeout_ms(void)
{
    uint32_t timeout_ms = arm_motor_query_period_ms() * 4U;
    return timeout_ms < 100U ? 100U : timeout_ms;
}

static void invalidate_arm_reference(void)
{
    float motor_limit_min;
    float motor_limit_max;

    Arm_Angle_Control_Set_Authorized(false);
    menu_stepper_tracking_enable = 0;
    arm_zero_valid = 0U;
    arm_home_complete = 0U;
    arm_home_seen_active = 0U;
    arm_home_idle_samples = 0U;
    if (zdt_stepper_motors[0] != NULL)
    {
        ZDT_Stepper_stop(zdt_stepper_motors[0], ZDT_STEPPER_SYNC_DISABLE);
        arm_motor_limits(&motor_limit_min, &motor_limit_max);
        ZDT_Stepper_Set_Software_Limits(zdt_stepper_motors[0], motor_limit_min, motor_limit_max, false);
    }
}

static bool write_arm_parameter(uint16_t parameter_id, float value)
{
    float *target = parameter(parameter_id);
    float previous;

    if (target == NULL || !isfinite(value)) return false;
    previous = *target;
    switch (parameter_id)
    {
    case ARM_LIMIT_MIN:
        if (value < -3600.0f || value > 3600.0f) return false;
        arm_limit_configured |= 0x01U;
        arm_point_configured |= 0x01U;
        break;
    case ARM_LIMIT_MAX:
        if (value < -3600.0f || value > 3600.0f) return false;
        arm_limit_configured |= 0x02U;
        arm_point_configured |= 0x04U;
        break;
    case ARM_BALANCE_POSITION:
        if (value < -3600.0f || value > 3600.0f) return false;
        arm_point_configured |= 0x02U;
        break;
    case ARM_BALL_POSITION_LOOP_ENABLE:
    case ARM_BALL_VELOCITY_LOOP_ENABLE:
        if (value != 0.0f && value != 1.0f) return false;
        break;
    case ARM_BALL_TARGET_POSITION:
        if (value < -12.5f || value > 12.5f) return false;
        break;
    case ARM_BALL_TARGET_VELOCITY:
        if (value < -100.0f || value > 100.0f) return false;
        break;
    case ARM_MOTOR_POSITION_SPEED_RPM:
        if (value <= 0.0f || value > 300.0f) return false;
        break;
    case ARM_MOTOR_POSITION_OFFSET_LIMIT_DEG:
        if (value <= 0.0f || value > 180.0f) return false;
        break;
    case ARM_MOTOR_POSITION_COMMAND_PERIOD_MS:
        if (value < 10.0f || value > 1000.0f) return false;
        break;
    case ARM_VISION_FIXED_DELAY_MS:
        if (value < 0.0f || value > 200.0f) return false;
        break;
    case ARM_VISION_GRAVITY_FEEDFORWARD_GAIN:
        if (value < 0.0f || value > 2.0f) return false;
        break;
    case ARM_VISION_BEAM_ANGLE_DEADZONE_DEG:
        if (value < 0.0f || value > 5.0f) return false;
        break;
    case ARM_VISION_PREDICTION_MAX_MS:
        if (value < 10.0f || value > 500.0f) return false;
        break;
    case ARM_MOTOR_TRAJECTORY_ENABLED:
        if (value != 0.0f && value != 1.0f) return false;
        break;
    case ARM_MOTOR_TRAJECTORY_MAX_SPEED_DEG_S:
        if (value < 1.0f || value > 600.0f) return false;
        break;
    case ARM_MOTOR_TRAJECTORY_MAX_ACCELERATION_DEG_S2:
        if (value < 1.0f || value > 5000.0f) return false;
        break;
    case ARM_VISION_KALMAN_MEASUREMENT_NOISE:
        if (value < 0.001f || value > 0.2f) return false;
        break;
    case ARM_VISION_KALMAN_PROCESS_NOISE_POSITION:
        if (value < 0.001f || value > 0.2f) return false;
        break;
    case ARM_VISION_KALMAN_PROCESS_NOISE_VELOCITY:
        if (value < 0.01f || value > 5.0f) return false;
        break;
    case ARM_BALL_VELOCITY_OUTPUT_INVERTED:
        if (value != 0.0f && value != 1.0f) return false;
        break;
    case ARM_BALL_BRAKE_FEEDFORWARD_ENABLED:
    case ARM_BALL_BRAKE_OUTPUT_INVERTED:
        if (value != 0.0f && value != 1.0f) return false;
        break;
    case ARM_BALL_BRAKE_STOP_TIME_S:
        if (value < 0.05f || value > 5.0f) return false;
        break;
    case ARM_BALL_BRAKE_GAIN:
        if (value < 0.0f || value > 5.0f) return false;
        break;
    case ARM_BALL_BRAKE_MAX_ACCELERATION_CM_S2:
        if (value < 1.0f || value > 500.0f) return false;
        break;
    case ARM_BALL_BRAKE_ANGLE_LIMIT_DEG:
        if (value < 0.1f || value > 15.0f) return false;
        break;
    case ARM_BALL_BRAKE_SPEED_DEADZONE_CM_S:
        if (value < 0.0f || value > 10.0f) return false;
        break;
    case ARM_MOTOR_SPEED_KP:
    case ARM_MOTOR_SPEED_KI:
    case ARM_MOTOR_SPEED_KD:
        if (value < 0.0f || value > 100.0f) return false;
        break;
    case ARM_PITCH_INVERTED:
        if (value != 0.0f && value != 1.0f) return false;
        break;
    case ARM_VISION_KALMAN_ENABLED:
    case ARM_VISION_PREDICTION_ENABLED:
        if (value != 0.0f && value != 1.0f) return false;
        break;
    default:
        return false;
    }
    *target = value;
    if (parameter_id == ARM_BALL_TARGET_VELOCITY &&
        !Arm_Angle_Control_Set_Ball_Velocity_Target(value))
    {
        *target = previous;
        return false;
    }
    if ((parameter_id >= ARM_BALL_POSITION_LOOP_ENABLE &&
         parameter_id <= ARM_MOTOR_SPEED_KD) ||
        parameter_id == ARM_VISION_KALMAN_ENABLED ||
        parameter_id == ARM_VISION_PREDICTION_ENABLED ||
         (parameter_id >= ARM_MOTOR_POSITION_SPEED_RPM &&
          parameter_id <= ARM_BALL_BRAKE_OUTPUT_INVERTED))
    {
        if (!Arm_Angle_Control_Apply_Config(Arm_Angle_Control_Get_Config()))
        {
            *target = previous;
            return false;
        }
    }
    if ((parameter_id == ARM_PITCH_INVERTED ||
         parameter_id == ARM_VISION_KALMAN_ENABLED ||
         parameter_id == ARM_VISION_PREDICTION_ENABLED ||
         (parameter_id >= ARM_VISION_FIXED_DELAY_MS &&
          parameter_id <= ARM_BALL_BRAKE_OUTPUT_INVERTED)) &&
        !save_arm_angle_config())
    {
        *target = previous;
        return false;
    }
    return true;
}

static bool write_arm_calibration_parameter(uint16_t parameter_id, float value)
{
    float next[6];
    float previous[6];
    uint8_t index;

    if (!isfinite(value) || parameter_id < ARM_CALIBRATION_LEFT_MOTOR ||
        parameter_id > ARM_CALIBRATION_RIGHT_PITCH || arm_calibration_status == ANO_ARM_CALIBRATION_ACTIVE)
    {
        return false;
    }
    index = (uint8_t)(parameter_id - ARM_CALIBRATION_LEFT_MOTOR);
    if ((index & 1U) == 0U)
    {
        if (value < -3600.0f || value > 3600.0f) return false;
    }
    else if (value < -180.0f || value > 180.0f)
    {
        return false;
    }

    memcpy(next, arm_calibration_pending, sizeof(next));
    next[index] = value;
    arm_calibration_pending[index] = value;
    arm_calibration_pending_mask |= (uint8_t)(1U << index);
    memcpy(previous, arm_calibration_values, sizeof(previous));
    memcpy(arm_calibration_values, next, sizeof(arm_calibration_values));
    if (!save_arm_calibration())
    {
        memcpy(arm_calibration_values, previous, sizeof(arm_calibration_values));
        return false;
    }
    arm_calibration_valid = arm_calibration_pending_mask == 0x3FU ? 1U : 0U;
    sync_arm_control_calibration();
    return true;
}

static bool write_arm_angle_parameter(uint16_t parameter_id, float value)
{
    arm_angle_control_config_t next = *Arm_Angle_Control_Get_Config();

    if (!isfinite(value)) return false;
    switch (parameter_id)
    {
    case ARM_ANGLE_KP: next.ball_position_kp = value; break;
    case ARM_ANGLE_KI: next.ball_position_ki = value; break;
    case ARM_ANGLE_KD: next.ball_position_kd = value; break;
    case ARM_ANGLE_DERIVATIVE_FILTER: next.derivative_filter_s = value; break;
    case ARM_ANGLE_MAX_SPEED: next.ball_velocity_limit_cm_s = value; break;
    default: return false;
    }
    return Arm_Angle_Control_Apply_Config(&next);
}

static bool write_attitude_parameter(uint16_t parameter_id, float value)
{
    car_attitude_control_config_t next = *Car_Attitude_Control_Get_Config();

    if (!isfinite(value)) return false;
    switch (parameter_id)
    {
    case ATTITUDE_MODE: next.mode = value; break;
    case ATTITUDE_BASE_SPEED: next.base_speed = value; break;
    case ATTITUDE_HEADING_KP: next.heading_kp = value; break;
    case ATTITUDE_HEADING_KI: next.heading_ki = value; break;
    case ATTITUDE_HEADING_KD: next.heading_kd = value; break;
    case ATTITUDE_RATE_KP: next.rate_kp = value; break;
    case ATTITUDE_RATE_KI: next.rate_ki = value; break;
    case ATTITUDE_RATE_KD: next.rate_kd = value; break;
    case ATTITUDE_LADRC_WO: next.ladrc_observer_bandwidth = value; break;
    case ATTITUDE_LADRC_WC: next.ladrc_controller_bandwidth = value; break;
    case ATTITUDE_LADRC_B0: next.ladrc_plant_gain = value; break;
    case ATTITUDE_MAX_RATE: next.max_rate = value; break;
    case ATTITUDE_MAX_DIFF: next.max_differential = value; break;
    case IMU_INVERT_X: next.invert_x = value; break;
    case IMU_INVERT_Y: next.invert_y = value; break;
    case IMU_INVERT_Z: next.invert_z = value; break;
    case STEP_RATE_AMPLITUDE: next.step_rate_amplitude = value; break;
    case STEP_ANGLE_AMPLITUDE: next.step_angle_amplitude = value; break;
    case STEP_HOLD_MS: next.step_hold_ms = value; break;
    case STEP_ANGLE_FEEDBACK_INVERTED: next.step_angle_feedback_inverted = value; break;
    case IMU_RATE_Z_INVERTED: next.rate_feedback_inverted = value; break;
    case ATTITUDE_LADRC_TARGET_RATE_SLEW: next.ladrc_target_rate_slew_deg_s2 = value; break;
    case ATTITUDE_LADRC_DIFFERENTIAL_SLEW: next.ladrc_differential_slew_s2 = value; break;
    default: return false;
    }
    return Car_Attitude_Control_Apply_Config(&next);
}

static bool write_tracking_parameter(uint16_t parameter_id, float value)
{
    car_tracking_config_t next = *Car_Attitude_Control_Get_Tracking_Config();

    if (!isfinite(value)) return false;
    switch (parameter_id)
    {
    case TRACKING_LAYER1_WEIGHT: next.layer1_weight_deg_s = value; break;
    case TRACKING_LAYER2_WEIGHT: next.layer2_weight_deg_s = value; break;
    case TRACKING_LAYER3_WEIGHT: next.layer3_weight_deg_s = value; break;
    case TRACKING_LAYER4_WEIGHT: next.layer4_weight_deg_s = value; break;
    default: return false;
    }
    return Car_Attitude_Control_Apply_Tracking_Config(&next);
}

static bool write_arc_parameter(uint16_t parameter_id, float value)
{
    car_lap_arc_config_t next = *Car_Lap_Task_Get_Arc_Config();

    if (!isfinite(value)) return false;
    switch (parameter_id)
    {
    case CAR_ARC_STRAIGHT_LENGTH: next.straight_length_m = value; break;
    case CAR_ARC_RADIUS: next.arc_radius_m = value; break;
    case CAR_ARC_LENGTH: next.arc_length_m = value; break;
    case CAR_ARC_EXIT_YAW: next.arc_exit_yaw_deg = value; break;
    case CAR_ARC_OPTICAL_LIMIT: next.optical_rate_limit_deg_s = value; break;
    case CAR_ARC_FEEDFORWARD_MIN: next.feedforward_rate_min_deg_s = value; break;
    case CAR_ARC_FEEDFORWARD_MAX: next.feedforward_rate_max_deg_s = value; break;
    case CAR_ARC_FEEDFORWARD_SLEW: next.feedforward_rate_slew_deg_s2 = value; break;
    case CAR_ARC_DIFFERENTIAL_SLEW: next.differential_slew_s2 = value; break;
    default: return false;
    }
    return Car_Lap_Task_Apply_Arc_Config(&next);
}

static bool write_gray_sensor_parameter(float value)
{
    if (!isfinite(value) || (value != 0.0f && value != 1.0f)) return false;
    gray_sensor_active_low = value;
    Gray_Sensor_Set_Active_Low((uint8_t)value);
    return true;
}

static bool write_car_parameter(uint16_t parameter_id, float value)
{
    if (parameter_id <= CAR_REDUCTION_RATIO ||
        (parameter_id > HOST_BAUD && parameter_id < CAR_M1_DIRECTION)) return false;
    float *target = parameter(parameter_id);
    if (target == NULL || !isfinite(value))
    {
        return false;
    }

    if (parameter_id <= CAR_KD)
    {
        if (value < -1000.0f || value > 1000.0f)
        {
            return false;
        }

        *target = value;
        return true;
    }
    else if (parameter_id == CAR_TYPE && value >= (float)CAR_MOTOR_TYPE_520 &&
             value <= (float)CAR_MOTOR_TYPE_TT && value == floorf(value))
    {
        *target = value;
        return true;
    }
    else if (parameter_id == CAR_DEADZONE && value >= 0.0f && value <= 3600.0f && value == floorf(value))
    {
        *target = value;
        return true;
    }
    else if (parameter_id == CAR_LINE && value >= 0.0f && value <= 65535.0f && value == floorf(value))
    {
        *target = value;
        return true;
    }
    else if (parameter_id == CAR_WHEEL_DIAMETER && value > 0.0f)
    {
        *target = value;
        return true;
    }
    else if (parameter_id == CAR_REDUCTION_RATIO && value > 0.0f && value <= 65535.0f && value == floorf(value))
    {
        *target = value;
        return true;
    }
    else if ((parameter_id == CAR_M1_DIRECTION || parameter_id == CAR_M2_DIRECTION) &&
             (value == 0.0f || value == 1.0f))
    {
        *target = value;
        return true;
    }

    return false;
}

static bool write_car_b0_calibration_parameter(uint16_t parameter_id, float value)
{
    float *target = parameter(parameter_id);

    if (App_Role_Get() != APP_ROLE_CAR || car_b0_calibration_active != 0U ||
        target == NULL || !isfinite(value) || value != floorf(value))
    {
        return false;
    }

    switch (parameter_id)
    {
    case CAR_B0_START_DIFFERENTIAL:
        if (value < 20.0f || value > 500.0f) return false;
        break;
    case CAR_B0_DIFFERENTIAL_STEP:
        if (value < 0.0f || value > 100.0f) return false;
        break;
    case CAR_B0_LEVEL_COUNT:
        if (value < 1.0f || value > 5.0f) return false;
        break;
    case CAR_B0_PULSE_DURATION_MS:
        if (value < 100.0f || value > 1000.0f) return false;
        break;
    case CAR_B0_SETTLE_DURATION_MS:
        if (value < 200.0f || value > 3000.0f) return false;
        break;
    case CAR_B0_REPEAT_COUNT:
        if (value < 1.0f || value > 5.0f) return false;
        break;
    default:
        return false;
    }

    if ((parameter_id == CAR_B0_START_DIFFERENTIAL ? value : car_b0_start_differential) +
        ((parameter_id == CAR_B0_LEVEL_COUNT ? value : car_b0_level_count) - 1.0f) *
        (parameter_id == CAR_B0_DIFFERENTIAL_STEP ? value : car_b0_differential_step) > 1000.0f)
    {
        return false;
    }

    *target = value;
    return true;
}

static bool parameter_is_visible(uint16_t parameter_id)
{
    if (App_Role_Get() == APP_ROLE_ARM)
    {
        for (size_t i = 0U; i < sizeof(arm_parameter_ids) / sizeof(arm_parameter_ids[0]); i++)
        {
            if (parameter_id == arm_parameter_ids[i]) return true;
        }
        return false;
    }

    if (parameter_id == BOARD_ROLE ||
        (parameter_id >= ARM_LIMIT_MIN && parameter_id <= ARM_HOME_MODE) ||
        parameter_id == ARM_BALANCE_POSITION ||
        (parameter_id >= ARM_MOTOR_QUERY_HZ && parameter_id <= ARM_MOTOR_SPEED_KD))
    {
        return false;
    }
    return parameter_id == HOST_BAUD || parameter(parameter_id) != NULL;
}

static uint16_t parameter_id_from_protocol_index(uint16_t index)
{
    if (App_Role_Get() == APP_ROLE_ARM)
    {
        return index < sizeof(arm_parameter_ids) / sizeof(arm_parameter_ids[0]) ?
               arm_parameter_ids[index] : ANO_PARAM_COUNT;
    }

    for (uint16_t id = 0U; id < ANO_PARAM_COUNT; id++)
    {
        if (!parameter_is_visible(id)) continue;
        if (index == 0U) return id;
        index--;
    }
    return ANO_PARAM_COUNT;
}

static uint16_t parameter_protocol_count(void)
{
    if (App_Role_Get() == APP_ROLE_ARM)
    {
        return (uint16_t)(sizeof(arm_parameter_ids) / sizeof(arm_parameter_ids[0]));
    }

    uint16_t count = 0U;
    for (uint16_t id = 0U; id < ANO_PARAM_COUNT; id++)
    {
        if (parameter_is_visible(id)) count++;
    }
    return count;
}

static void apply_parameter_after_ack(uint16_t parameter_id)
{
    if (App_Role_Get() == APP_ROLE_CAR)
    {
        if (parameter_id == CAR_M1_DIRECTION || parameter_id == CAR_M2_DIRECTION)
        {
            float direction = parameter_id == CAR_M1_DIRECTION ? car_m1_direction : car_m2_direction;
            (void)Car_Motor_Set_Direction((uint8_t)(parameter_id - CAR_M1_DIRECTION), direction != 0.0f);
        }
    }
    else
    {
        if (parameter_id >= YAW_KP && parameter_id <= YAW_KD) PIDController_Init(&yaw_pid);
        else if (parameter_id >= PITCH_KP && parameter_id <= PITCH_KD) PIDController_Init(&pitch_pid);
        else if (parameter_id == ARM_LIMIT_MIN || parameter_id == ARM_LIMIT_MAX ||
                 parameter_id == ARM_BALANCE_POSITION) invalidate_arm_reference();
        else if ((parameter_id >= ARM_BALL_POSITION_LOOP_ENABLE && parameter_id <= ARM_MOTOR_SPEED_KD) ||
                 parameter_id == ARM_VISION_KALMAN_ENABLED || parameter_id == ARM_VISION_PREDICTION_ENABLED ||
                 parameter_id == ARM_BALL_TARGET_VELOCITY ||
                 (parameter_id >= ARM_MOTOR_POSITION_SPEED_RPM && parameter_id <= ARM_BALL_BRAKE_OUTPUT_INVERTED))
            (void)Arm_Angle_Control_Apply_Config(Arm_Angle_Control_Get_Config());
        else if (parameter_id == ARM_PITCH_INVERTED)
            Arm_Angle_Control_Set_Pitch_Inverted(arm_pitch_inverted != 0.0f);
    }
}

static void send_check(uint8_t destination, uint8_t id, uint8_t sum, uint8_t add)
{
    uint8_t d[3] = {id, sum, add};
    send_frame_to(destination, ANO_ID_CHECK, d, sizeof(d));
}

static bool save_all_parameters(void)
{
    uint32_t baud_rate = requested_baud_rate != 0U ? requested_baud_rate : host_baud_rate;

    if (!storage_ready || !save_storage(baud_rate)) return false;
    if (App_Role_Get() == APP_ROLE_ARM && !save_arm_angle_config())
    {
        storage_operation = 3U;
        return false;
    }
    if (baud_rate != host_baud_rate)
    {
        pending_baud_rate = baud_rate;
        baud_switch_at_ms = HAL_GetTick() + ANO_BAUD_SWITCH_DELAY_MS;
    }
    return true;
}

static void schedule_parameter_save(void)
{
    parameter_save_pending = 1U;
    parameter_save_at_ms = HAL_GetTick() + ANO_PARAMETER_SAVE_DELAY_MS;
}

static void process_parameter_save(void)
{
    if (parameter_save_pending == 0U ||
        (int32_t)(HAL_GetTick() - parameter_save_at_ms) < 0 || tx_active ||
        response_count != 0U || rx_head != rx_tail)
    {
        return;
    }
    parameter_save_pending = 0U;
    (void)save_all_parameters();
}

static bool parameter_session_active(void)
{
    return parameter_save_pending != 0U ||
           (int32_t)(HAL_GetTick() - parameter_session_until_ms) < 0;
}

static void restore_parameter_defaults(void)
{
    reset_local_defaults();
    requested_baud_rate = ANO_DEFAULT_BAUD_RATE;
    if (host_baud_rate != ANO_DEFAULT_BAUD_RATE)
    {
        pending_baud_rate = ANO_DEFAULT_BAUD_RATE;
        baud_switch_at_ms = HAL_GetTick() + ANO_BAUD_SWITCH_DELAY_MS;
    }
    if (App_Role_Get() == APP_ROLE_CAR) (void)apply_car_config();
    else if (zdt_stepper_motors[0] != NULL) invalidate_arm_reference();
}
static void send_param(uint16_t protocol_index)
{
    uint16_t id = parameter_id_from_protocol_index(protocol_index);
    uint8_t d[6]; uint16_t i = 0U; float *v;
    if (id >= ANO_PARAM_COUNT) return;
    v = parameter(id);
    if (id == HOST_BAUD) { put_u16(d, &i, protocol_index); put_float(d, &i, (float)(requested_baud_rate != 0U ? requested_baud_rate : host_baud_rate)); send_frame(ANO_ID_PARAM_VALUE, d, i); return; }
    if (id >= ARM_CALIBRATION_LEFT_MOTOR && id <= ARM_CALIBRATION_RIGHT_PITCH)
    {
        put_u16(d, &i, protocol_index);
        put_float(d, &i, arm_calibration_pending[id - ARM_CALIBRATION_LEFT_MOTOR]);
        send_frame(ANO_ID_PARAM_VALUE, d, i);
        return;
    }
    if (v == NULL) return;
    put_u16(d, &i, protocol_index); put_float(d, &i, *v); send_frame(ANO_ID_PARAM_VALUE, d, i);
}
static void send_device_info(void)
{
    const char *name = App_Role_Get_Name();
    uint8_t d[32] = {0}; uint16_t i = 0U;
    size_t name_length = strlen(name);
    if (name_length > sizeof(d) - i - 9U) name_length = sizeof(d) - i - 9U;
    d[i++] = App_Role_Get_ANO_Address(); put_u16(d, &i, 1U); put_u16(d, &i, 1U); put_u16(d, &i, 0U); put_u16(d, &i, 810U);
    memcpy(&d[i], name, name_length); i += (uint16_t)name_length;
    send_frame(ANO_ID_DEVICE_INFO, d, i);
}
static void send_param_info(uint16_t protocol_index)
{
    uint16_t id = parameter_id_from_protocol_index(protocol_index);
    if (id >= ANO_PARAM_COUNT) return;
    if (App_Role_Get() == APP_ROLE_ARM)
    {
        static const char *const names[] = {
             "小球位置环开关", "小球速度环开关", "小球目标位置", "小球目标速度",
            "小球位置环Kp", "小球位置环Ki", "小球位置环Kd",
            "小球速度环Kp", "小球速度环Ki", "小球速度环Kd",
            "小球速度限幅", "PID微分滤波",
            "最高点电机位置", "最高点Pitch角度", "平衡点电机位置",
            "平衡点Pitch角度", "最低点电机位置", "最低点Pitch角度",             "Pitch反转",
            "视觉卡尔曼开关", "视觉帧间预测开关",
             "电机位置速度", "角度修正限幅", "电机命令周期", "视觉固定延迟",
             "重力前馈增益", "杆角死区", "预测时长上限", "位置轨迹开关",
             "位置跟随速度", "位置跟随加速度",
             "卡尔曼测量噪声", "卡尔曼位置噪声", "卡尔曼速度噪声",
             "速度环输出反向",
             "刹车前馈开关", "刹车目标时间", "刹车前馈增益", "刹车最大加速度",
             "刹车角度限幅", "刹车速度死区", "刹车输出反向"
        };
        static const char *const infos[] = {
             "0关闭，1启用位置外环", "0关闭，1启用速度内环", "位置环目标，单位厘米", "位置环关闭时的速度目标，单位厘米每秒",
            "位置外环比例系数", "位置外环积分系数", "位置外环微分系数",
            "速度内环比例系数", "速度内环积分系数", "速度内环微分系数",
            "位置环输出速度限幅，单位厘米每秒", "两个小球环共用微分滤波时间，单位秒",
            "设置最高点时自动采集，单位度，只读", "设置最高点时自动采集，单位度，只读", "设置平衡点时自动采集，单位度，只读",
            "设置平衡点时自动采集，单位度，只读", "设置最低点时自动采集，单位度，只读", "设置最低点时自动采集，单位度，只读",
            "0正常，1仅反转臂杆控制和标定的Pitch角度",
            "0关闭，1使用MCU二维卡尔曼估计位置和速度",
            "0关闭，1使用视觉帧间位置预测",
            "闭环电机位置命令转速，单位RPM，1至300",
            "速度环输出到电机角度的最大修正量，单位度，1至180",
             "电机位置命令最小间隔，单位毫秒，10至1000",
             "视觉固定延迟，单位毫秒，0至200",
             "重力前馈增益，0至2",
             "杆角死区，单位度，0至5",
             "预测时长上限，单位毫秒，10至500",
             "0关闭，1启用位置速度加速度平滑",
             "位置轨迹最大速度，单位度每秒，1至600",
             "位置轨迹最大加速度，单位度每秒平方，1至5000",
             "卡尔曼测量噪声，0.001至0.2，过大回正慢，过小跟随跳变，静止时减小",
             "卡尔曼位置过程噪声，0.001至0.2，值越大跟随越快",
             "卡尔曼速度过程噪声，0.01至5，过大静止时速度估计大，过小运动跟踪滞后",
             "0速度与电机修正同号，1反向",
             "0关闭，1启用速度误差刹车前馈",
             "期望刹停时间，单位秒，0.05至5，越小刹车越猛",
             "刹车前馈增益，0至5",
             "刹车最大加速度，单位厘米每秒平方，1至500",
             "刹车角度限幅，单位度，0.1至15",
             "刹车速度误差死区，单位厘米每秒，0至10，避免小误差抖动",
             "0原方向，1反向刹车角度"
        };
        uint8_t d[ANO_DATA_MAX] = {0};
        uint16_t i = 0U;
        size_t index;
        for (index = 0U; index < sizeof(arm_parameter_ids) / sizeof(arm_parameter_ids[0]); index++)
            if (arm_parameter_ids[index] == id) break;
        if (index == sizeof(arm_parameter_ids) / sizeof(arm_parameter_ids[0])) return;
        put_u16(d, &i, protocol_index); d[i++] = 8U;
        strncpy((char *)&d[i], names[index], 20U); i += 20U;
        strncpy((char *)&d[i], infos[index], sizeof(d) - i - 1U);
        i += (uint16_t)strlen((const char *)&d[i]);
        send_frame(ANO_ID_PARAM_INFO, d, i);
        return;
    }
    if (id == ATTITUDE_LADRC_TARGET_RATE_SLEW ||
        id == ATTITUDE_LADRC_DIFFERENTIAL_SLEW)
    {
        const char *name = id == ATTITUDE_LADRC_TARGET_RATE_SLEW ?
                           "LADRC目标斜率" : "LADRC差速斜率";
        const char *info = id == ATTITUDE_LADRC_TARGET_RATE_SLEW ?
                           "LADRC目标角速度变化率，单位度每秒平方" :
                           "LADRC差速输出变化率，单位速度单位每秒";
        uint8_t d[ANO_DATA_MAX] = {0};
        uint16_t i = 0U;
        put_u16(d, &i, protocol_index);
        d[i++] = 8U;
        strncpy((char *)&d[i], name, 20U); i += 20U;
        strncpy((char *)&d[i], info, sizeof(d) - i - 1U);
        i += (uint16_t)strlen((const char *)&d[i]);
        send_frame(ANO_ID_PARAM_INFO, d, i);
        return;
    }
    static const char *const arc_names[] = {
        "直道长度", "圆弧半径", "圆弧长度", "圆弧退出航向",
        "光电纠偏上限", "前馈最小角速度", "前馈最大角速度",
        "前馈变化率", "差速变化率"
    };
    static const char *const arc_infos[] = {
        "AB和CD直道长度，单位米",
        "BC和DA圆弧半径，单位米",
        "BC和DA圆弧长度，单位米",
        "圆弧退出所需的最小IMU相对航向，单位度",
        "圆弧中光电纠偏角速度上限，单位度每秒",
        "圆弧前馈角速度下限，单位度每秒",
        "圆弧前馈角速度上限，单位度每秒",
        "前馈角速度变化率，单位度每秒平方",
        "左右轮差速变化率，单位速度单位每秒"
    };
    if (id >= CAR_ARC_STRAIGHT_LENGTH && id <= CAR_ARC_DIFFERENTIAL_SLEW)
    {
        uint8_t d[ANO_DATA_MAX] = {0};
        uint16_t i = 0U;
        uint16_t index = (uint16_t)(id - CAR_ARC_STRAIGHT_LENGTH);
        put_u16(d, &i, protocol_index);
        d[i++] = 8U;
        strncpy((char *)&d[i], arc_names[index], 20U); i += 20U;
        strncpy((char *)&d[i], arc_infos[index], sizeof(d) - i - 1U);
        i += (uint16_t)strlen((const char *)&d[i]);
        send_frame(ANO_ID_PARAM_INFO, d, i);
        return;
    }
    if (id <= CAR_REDUCTION_RATIO ||
        (id > HOST_BAUD && id < CAR_M1_DIRECTION)) return;
#if 0
    static const char *const names[ANO_PARAM_COUNT] = {
         "", "", "", "", "", "", "", "", "", "", "", "", "", "", "上位机波特率",
         "", "", "", "", "", "M1方向", "M2方向",
        "双环控制模式", "基础轮速", "航向Kp", "航向Ki", "航向Kd", "角速度Kp", "角速度Ki", "角速度Kd",
        "LADRC观测带宽", "LADRC控制带宽", "LADRC对象增益", "最大角速度", "最大差速", "IMU X轴倒置", "IMU Y轴倒置", "IMU Z轴倒置",
        "角速度阶跃幅度", "指定旋转角度", "阶跃保持时间", "航向角反馈倒置",
        "滚球前馈使能", "前后X轴增益", "左右Y轴增益", "前馈滤波常数", "前馈加速度死区", "前馈转速限幅", "控制板角色",
        "臂杆最左点", "臂杆最右点", "臂杆调试速度", "臂杆目标位置", "臂杆调试方向",
        "臂杆加速度", "驱动细分", "调试超时时间", "驱动回零模式", "一圈停车补偿", "臂杆平衡点", "里程每米计数",
        "停用循迹方向", "停用偏差滤波", "停用转向斜率", "小车惯导来源",
        "停用初始权重", "停用增权时间", "停用释放时间", "停用推进速度", "停用航向限幅",
        "固定半圈角度", "停用半圈窗口", "停用半圈确认", "固定横线确认",
        "内侧光电权重", "外侧光电权重", "停用时间曲线", "停用光电融合", "停用惯导融合",
        "Z轴角速度反馈倒置", "光电低电平有效",
        "b0起始差速", "b0差速步进", "b0档位数量", "b0脉冲时长", "b0停稳时长", "b0重复次数",
        "电机查询频率", "电机遥测频率", "臂杆目标角度", "臂杆角度Kp",
        "臂杆角度Ki", "臂杆角度Kd", "角度微分滤波", "角度最大转速",
        "角度死区", "角度方向倒置", "标定左点电机", "标定左点Pitch",
         "标定平衡电机", "标定平衡Pitch", "标定右点电机", "标定右点Pitch",
         "直道长度", "圆弧半径", "圆弧长度", "圆弧退出航向",
         "光电纠偏上限", "前馈最小角速度", "前馈最大角速度", "前馈变化率", "差速变化率"
    };
    static const char *const infos[ANO_PARAM_COUNT] = {
         "", "", "", "", "", "", "", "", "", "", "", "", "", "", "匿名上位机波特率",
         "", "", "", "", "", "0为正向，1为反向，作用于速度和PWM", "0为正向，1为反向，作用于速度和PWM",
        "0为PID，1为LADRC", "M1和M2共同前进速度", "航向外环比例", "航向外环积分", "航向外环微分",
        "角速度内环比例", "角速度内环积分", "角速度内环微分", "线性扩张状态观测器带宽", "角速度控制器带宽",
        "被控对象输入增益，必须大于零", "航向外环目标角速度限幅", "左右轮速度差限幅", "0正常，1倒置", "0正常，1倒置", "0正常，1倒置",
        "角速度阶跃幅度，单位度每秒", "指定旋转和航向角阶跃共用的相对角度，单位度", "正阶跃和回零各自保持毫秒数", "0原方向，1倒置，作用于指定旋转和航向角阶跃",
        "0关闭，1启用；启用后先静止校准", "JY61P X轴到Pitch转速增益，可为负", "JY61P Y轴到Yaw转速增益，可为负",
        "一阶低通时间常数，单位秒", "滤波加速度死区，单位g", "单轴前馈最大转速，单位RPM", "0小车控制板，1臂杆电机板；保存后重启生效",
        "臂杆最向左倾斜绝对角度，修改后须重新回零", "臂杆最向右倾斜绝对角度，修改后须重新回零",
        "速度和定位命令使用的转速，单位RPM", "绝对定位目标角度，必须位于上下限内", "0顺时针，1逆时针",
        "驱动器加速度档位，0至255", "驱动器细分值，1至255，应用配置后生效", "运动看门狗超时，100至5000毫秒", "驱动器回零模式，0至4",
        "检测到A点十字后继续行驶时间，单位毫秒，范围0至2000", "上电回零后自动到达的绝对角度，必须严格位于左右点之间",
        "单轮前进一米对应的累计编码器计数，用于里程标定",
        "兼容保留，固定为0且不参与控制", "兼容保留，固定为0且不参与控制",
        "兼容保留，固定为0且不参与控制", "0使用JY61P，1使用WheelTec；保存后保持",
        "兼容保留，固定为0且不参与控制", "兼容保留，固定为0且不参与控制",
        "兼容保留，固定为0且不参与控制", "兼容保留，固定为0且不参与控制",
        "兼容保留，固定为0且不参与控制", "固定累计Yaw达到180度后武装终点检测",
        "兼容保留，固定为0且不参与控制", "兼容保留，固定为0且不参与控制",
        "终点横线固定连续确认50毫秒", "左二和右二改变目标航向的速度，单位度每秒",
        "左一和右一改变目标航向的速度，单位度每秒", "兼容保留，固定为0且不参与控制",
        "兼容保留，固定为0且不参与控制", "兼容保留，惯导航向保持始终启用",
        "0原方向，1倒置，仅作用于Z轴角速度反馈", "0为高电平黑线有效，1为低电平黑线有效",
        "第一档逻辑差速，范围20至500", "相邻档逻辑差速增量，范围0至100", "正幅值档位数量，范围1至5",
        "每个激励脉冲时长，单位毫秒，范围100至1000", "相邻脉冲停稳时长，单位毫秒，范围200至3000",
        "每个档位的正反激励重复次数，范围1至5",
        "0x35和0x36交替查询总频率，20至200Hz，复位恢复100Hz",
        "上位机电机状态帧频率，1至50Hz，复位恢复20Hz",
        "JY61P Pitch目标角度，负30至30度，复位恢复0度", "独立臂杆角度环比例系数",
        "独立臂杆角度环积分系数", "独立臂杆角度环微分系数",
        "微分低通时间常数，0.001至1秒", "角度环输出转速限幅，0至100RPM",
        "目标角死区，0至5度", "0原方向，1倒置电机输出方向",
        "三点联合标定最左点电机绝对位置，只读", "三点联合标定最左点JY61P Pitch，只读",
        "三点联合标定平衡点电机绝对位置，只读", "三点联合标定平衡点JY61P Pitch，只读",
         "三点联合标定最右点电机绝对位置，只读", "三点联合标定最右点JY61P Pitch，只读",
         "AB和CD直道长度，单位米", "BC和DA圆弧半径，单位米", "BC和DA圆弧长度，单位米",
         "圆弧退出所需的最小IMU相对航向，单位度", "圆弧中光电纠偏角速度上限，单位度每秒",
         "圆弧前馈角速度下限，单位度每秒", "圆弧前馈角速度上限，单位度每秒",
         "前馈角速度变化率，单位度每秒平方", "左右轮差速变化率，单位速度单位每秒"
    };
    uint8_t d[ANO_DATA_MAX] = {0}; uint16_t i = 0U;
     if (id >= ANO_PARAM_COUNT) return;
    put_u16(d, &i, id); d[i++] = 8U; strncpy((char *)&d[i], names[id], 20U); i += 20U;
    strncpy((char *)&d[i], infos[id], sizeof(d) - i - 1U); i += (uint16_t)strlen((const char *)&d[i]);
    send_frame(ANO_ID_PARAM_INFO, d, i);
 #else
    {
        uint8_t d[ANO_DATA_MAX] = {0};
        uint16_t i = 0U;
        if (id >= ANO_PARAM_COUNT) return;
        put_u16(d, &i, protocol_index);
        d[i++] = 8U;
        i += 20U;
        send_frame(ANO_ID_PARAM_INFO, d, i);
    }
#endif
}

static void send_command_count(void)
{
    size_t count = App_Role_Get() == APP_ROLE_ARM ?
                   sizeof(arm_commands) / sizeof(arm_commands[0]) :
                   sizeof(car_commands) / sizeof(car_commands[0]);
    uint8_t d[3] = {0x00U, (uint8_t)count, 0U};
    send_frame(ANO_ID_COMMAND_QUERY, d, sizeof(d));
}

static void send_command_info(uint16_t command_id)
{
    uint8_t d[ANO_DATA_MAX] = {0}; uint16_t i = 0U;
    const ano_command_desc_t *table = App_Role_Get() == APP_ROLE_ARM ? arm_commands : car_commands;
    size_t count = App_Role_Get() == APP_ROLE_ARM ?
                   sizeof(arm_commands) / sizeof(arm_commands[0]) :
                   sizeof(car_commands) / sizeof(car_commands[0]);
    if (command_id >= (uint16_t)count) return;
    put_u16(d, &i, command_id);
    memcpy(&d[i], table[command_id].cid, sizeof(table[command_id].cid)); i += sizeof(table[command_id].cid);
    i += 8U;
    strncpy((char *)&d[i], table[command_id].name, 20U); i += 20U;
    strncpy((char *)&d[i], table[command_id].info, sizeof(d) - i - 1U); i += (uint16_t)strlen((const char *)&d[i]);
    send_frame(ANO_ID_COMMAND_INFO, d, i);
}

static void handle_command_query(const uint8_t *d, uint16_t n)
{
    if (n < 3U) return;
    if (d[0] == 0U) send_command_count();
    else if (d[0] == 1U) send_command_info(get_u16(&d[1]));
}

static bool apply_car_config(void)
{
    car_motor_config_t config = {
        .type = (car_motor_type_t)(uint8_t)car_type,
        .deadzone = (uint16_t)car_deadzone,
        .line = (uint16_t)car_line,
        .reduction_ratio = (uint16_t)car_reduction_ratio,
        .wheel_diameter_mm = car_wheel_diameter,
        .pid_kp = car_pid[0],
        .pid_ki = car_pid[1],
        .pid_kd = car_pid[2]
    };
    return Car_Motor_Apply_Config(&config);
}

static bool restore_defaults(void)
{
    storage_operation = 3U;
    if (App_Role_Get() == APP_ROLE_CAR)
    {
        (void)Car_Lap_Task_Stop();
    }
    else
    {
        (void)stop_arm_debug(true);
    }
    if (!storage_ready || !W25Qxx_EraseSector(ANO_STORAGE_ADDRESS) ||
        (App_Role_Get() == APP_ROLE_CAR && !Car_Motor_Restore_Defaults()))
    {
        return false;
    }

    reset_local_defaults();
    if (host_baud_rate != ANO_DEFAULT_BAUD_RATE)
    {
        pending_baud_rate = ANO_DEFAULT_BAUD_RATE;
        baud_switch_at_ms = HAL_GetTick() + ANO_BAUD_SWITCH_DELAY_MS;
    }
    storage_operation = 0U;
    return true;
}

static bool stop_car_debug(void)
{
    car_debug_active = 0U;
    return Car_Motor_Set_PWM(0, 0, 0, 0);
}

static bool enter_car_debug(void)
{
    (void)Car_Lap_Task_Stop();
    if (!Car_Motor_Set_PWM(0, 0, 0, 0))
    {
        return false;
    }
    car_debug_active = 1U;
    return true;
}

static bool start_car_debug(void)
{
    int16_t m1;
    int16_t m2;

    if (!car_debug_active)
    {
        return false;
    }

    m1 = (int16_t)car_debug_m1_value;
    m2 = (int16_t)car_debug_m2_value;
    if (car_debug_m1_direction != 0.0f) m1 = (int16_t)-m1;
    if (car_debug_m2_direction != 0.0f) m2 = (int16_t)-m2;
    if (car_debug_control_mode == 0.0f)
    {
        return Car_Motor_Set_Speed(m1, m2, 0, 0);
    }
    return Car_Motor_Set_PWM(m1, m2, 0, 0);
}

static bool start_attitude_step(car_attitude_step_mode_t mode)
{
    (void)Car_Lap_Task_Stop();
    if (!Car_Attitude_Control_Start_Step(mode)) return false;
    car_debug_active = 0U;
    return true;
}

static bool start_attitude_rotation(void)
{
    const car_attitude_control_config_t *config = Car_Attitude_Control_Get_Config();

    (void)Car_Lap_Task_Stop();
    if (!Car_Attitude_Control_Start_Rotation(config->step_angle_amplitude)) return false;
    car_debug_active = 0U;
    return true;
}

static bool arm_limits_valid(void)
{
    return arm_points_valid();
}

static bool arm_angle_ready(void)
{
    return arm_zero_valid != 0U && arm_limits_valid() && arm_home_active == 0U &&
           arm_motion_active == 0U && arm_debug_active == 0U &&
           arm_calibration_status != ANO_ARM_CALIBRATION_ACTIVE;
}

static void reset_arm_calibration_samples(void)
{
    arm_calibration_sample_count = 0U;
    arm_calibration_sample_index = 0U;
    arm_calibration_last_sample_ms = 0U;
    arm_calibration_pitch_sum = 0.0f;
    arm_calibration_pitch_min = 0.0f;
    arm_calibration_pitch_max = 0.0f;
}

static void cancel_arm_calibration(uint8_t status)
{
    if (zdt_stepper_motors[0] != NULL)
        ZDT_Stepper_stop(zdt_stepper_motors[0], ZDT_STEPPER_SYNC_DISABLE);
    arm_motion_active = 0U;
    arm_speed_motion = 0U;
    arm_calibration_status = status;
    arm_calibration_stage = 0U;
    arm_calibration_captured_mask = 0U;
    memset(arm_calibration_pending, 0, sizeof(arm_calibration_pending));
    arm_calibration_pending_mask = 0U;
    reset_arm_calibration_samples();
}

static void process_arm_calibration(ZDTStepperMotor *motor, uint32_t now)
{
    const jy61p_status_t *status;
    const jy61p_sample_t *sample;
    float pitch;

    if (arm_calibration_status != ANO_ARM_CALIBRATION_ACTIVE) return;
    status = JY61P_Get_Status();
    if (arm_calibration_sample_count != 0U && arm_calibration_last_sample_ms != 0U &&
        now - arm_calibration_last_sample_ms > ARM_CALIBRATION_SAMPLE_TIMEOUT_MS)
    {
        reset_arm_calibration_samples();
    }
    if (arm_motion_active || fabsf(motor->data.current_speed) > 0.5f ||
        motor->data.current_speed_ms == 0U ||
        now - motor->data.current_speed_ms > arm_motor_feedback_timeout_ms() ||
        motor->data.current_position_ms == 0U ||
        now - motor->data.current_position_ms > arm_motor_feedback_timeout_ms() ||
        status->available == 0U || status->sample_valid == 0U ||
        status->consecutive_failures != 0U)
    {
        reset_arm_calibration_samples();
        arm_calibration_last_sequence = status->sequence;
        return;
    }
    if (status->sequence == arm_calibration_last_sequence) return;
    sample = JY61P_Get_Sample();
    pitch = (float)sample->angle.y * (180.0f / 32768.0f);
    if (arm_pitch_inverted != 0.0f) pitch = -pitch;
    arm_calibration_last_sequence = status->sequence;
    if (arm_calibration_sample_count < ARM_CALIBRATION_SAMPLE_COUNT)
    {
        arm_calibration_pitch_samples[arm_calibration_sample_index] = pitch;
        arm_calibration_pitch_sum += pitch;
        arm_calibration_sample_count++;
    }
    else
    {
        arm_calibration_pitch_sum -= arm_calibration_pitch_samples[arm_calibration_sample_index];
        arm_calibration_pitch_samples[arm_calibration_sample_index] = pitch;
        arm_calibration_pitch_sum += pitch;
    }
    arm_calibration_sample_index = (uint8_t)((arm_calibration_sample_index + 1U) %
                                             ARM_CALIBRATION_SAMPLE_COUNT);
    arm_calibration_pitch_min = arm_calibration_pitch_samples[0];
    arm_calibration_pitch_max = arm_calibration_pitch_samples[0];
    for (uint8_t i = 1U; i < arm_calibration_sample_count; i++)
    {
        if (arm_calibration_pitch_samples[i] < arm_calibration_pitch_min)
            arm_calibration_pitch_min = arm_calibration_pitch_samples[i];
        if (arm_calibration_pitch_samples[i] > arm_calibration_pitch_max)
            arm_calibration_pitch_max = arm_calibration_pitch_samples[i];
    }
    arm_calibration_last_sample_ms = now;
}

static bool stop_arm_debug(bool exit_debug)
{
    if (zdt_stepper_motors[0] == NULL) return false;
    if (arm_home_active) ZDT_Stepper_abort_zero_return(zdt_stepper_motors[0]);
    ZDT_Stepper_stop(zdt_stepper_motors[0], ZDT_STEPPER_SYNC_DISABLE);
    arm_motion_active = 0U;
    arm_speed_motion = 0U;
    arm_home_active = 0U;
    arm_zero_confirm_pending = 0U;
    if (arm_startup_state >= ARM_STARTUP_WAIT_HOME_ACK && arm_startup_state <= ARM_STARTUP_MOVE_BALANCE)
    {
        arm_startup_state = ARM_STARTUP_FAILED;
    }
    if (exit_debug && arm_calibration_status == ANO_ARM_CALIBRATION_ACTIVE)
        cancel_arm_calibration(ANO_ARM_CALIBRATION_CANCELLED);
    if (exit_debug) arm_debug_active = 0U;
    if (exit_debug) Arm_Competition_Task_Set_Ready(arm_angle_ready());
    return true;
}

static bool capture_arm_calibration_point(uint8_t index)
{
    ZDTStepperMotor *motor = zdt_stepper_motors[0];
    const jy61p_status_t *status = JY61P_Get_Status();
    const jy61p_sample_t *sample;
    float previous[6];
    float pitch;
    uint8_t previous_mask;

    if (motor == NULL || motor->data.current_position_ms == 0U ||
        status->sample_valid == 0U || index > 4U) return false;
    sample = JY61P_Get_Sample();
    pitch = (float)sample->angle.y * (180.0f / 32768.0f);
    if (arm_pitch_inverted != 0.0f) pitch = -pitch;

    memcpy(previous, arm_calibration_values, sizeof(previous));
    previous_mask = arm_calibration_pending_mask;
    arm_calibration_values[index] = motor->data.current_position;
    arm_calibration_values[index + 1U] = pitch;
    memcpy(arm_calibration_pending, arm_calibration_values, sizeof(arm_calibration_pending));
    arm_calibration_pending_mask |= (uint8_t)(0x03U << index);
    if (!save_arm_calibration())
    {
        memcpy(arm_calibration_values, previous, sizeof(previous));
        arm_calibration_pending_mask = previous_mask;
        return false;
    }
    arm_calibration_valid = arm_calibration_pending_mask == 0x3FU ? 1U : 0U;
    sync_arm_control_calibration();
    return true;
}

static void send_arm_calibration_parameters(uint8_t index)
{
    uint16_t first_id = ARM_CALIBRATION_LEFT_MOTOR + index;

    for (uint16_t protocol_index = 0U;
         protocol_index < sizeof(arm_parameter_ids) / sizeof(arm_parameter_ids[0]);
         protocol_index++)
    {
        if (arm_parameter_ids[protocol_index] == first_id ||
            arm_parameter_ids[protocol_index] == first_id + 1U)
        {
            send_param(protocol_index);
        }
    }
}

static bool request_arm_calibration_capture(uint8_t index)
{
    ZDTStepperMotor *motor = zdt_stepper_motors[0];

    if (motor == NULL || index > 4U || JY61P_Get_Status()->sample_valid == 0U) return false;
    arm_capture_pending = 1U;
    arm_capture_index = index;
    arm_capture_position_baseline_ms = motor->data.current_position_ms;
    arm_capture_requested_ms = HAL_GetTick();
    ZDT_Stepper_Read_current_position(motor);
    return true;
}

static void fail_arm_startup(void)
{
    float motor_limit_min;
    float motor_limit_max;

    Arm_Competition_Task_Set_Ready(false);
    if (zdt_stepper_motors[0] != NULL)
    {
        if (arm_home_active) ZDT_Stepper_abort_zero_return(zdt_stepper_motors[0]);
        ZDT_Stepper_stop(zdt_stepper_motors[0], ZDT_STEPPER_SYNC_DISABLE);
    }
    arm_motion_active = 0U;
    arm_speed_motion = 0U;
    arm_home_active = 0U;
    arm_zero_confirm_pending = 0U;
    arm_zero_valid = 0U;
    arm_motor_limits(&motor_limit_min, &motor_limit_max);
    ZDT_Stepper_Set_Software_Limits(zdt_stepper_motors[0], motor_limit_min, motor_limit_max, false);
    arm_debug_active = 0U;
    arm_startup_state = ARM_STARTUP_FAILED;
}

static void start_arm_startup(void)
{
    ZDTStepperMotor *motor = zdt_stepper_motors[0];
    uint32_t now = HAL_GetTick();

    if (App_Role_Get() != APP_ROLE_ARM || motor == NULL || !arm_points_valid()) return;
    menu_stepper_tracking_enable = 0;
    Arm_Angle_Control_Set_Authorized(false);
    arm_debug_active = 0U;
    arm_zero_valid = 0U;
    arm_motion_active = 0U;
    arm_home_active = 1U;
    arm_home_complete = 0U;
    arm_home_seen_active = 0U;
    arm_home_idle_samples = 0U;
    arm_home_status_processed_ms = 0U;
    arm_zero_confirm_pending = 0U;
    ZDT_Stepper_stop(motor, ZDT_STEPPER_SYNC_DISABLE);
    ZDT_Stepper_Enable(motor, ZDT_STEPPER_ENABLE, ZDT_STEPPER_SYNC_DISABLE);
    ZDT_Stepper_trigger_zero_return(motor, (uint8_t)arm_home_mode, ZDT_STEPPER_SYNC_DISABLE);
    arm_zero_confirm_issued_ms = now;
    arm_last_home_status_request_ms = now;
    arm_startup_deadline_ms = now + ARM_STARTUP_HOME_TIMEOUT_MS;
    arm_startup_state = ARM_STARTUP_WAIT_HOME_ACK;
}

static void move_arm_to_balance(void)
{
    ZDTStepperMotor *motor = zdt_stepper_motors[0];
    float motor_target = arm_coordinate_to_motor(arm_balance_position);
    uint8_t direction = motor_target >= 0.0f ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW;
    float travel_seconds = fabsf(arm_balance_position) / (arm_debug_speed * 6.0f);
    uint32_t timeout_ms = (uint32_t)(travel_seconds * 1000.0f) + 3000U;
    uint32_t now = HAL_GetTick();

    if (timeout_ms < ARM_STARTUP_MOVE_TIMEOUT_MS) timeout_ms = ARM_STARTUP_MOVE_TIMEOUT_MS;
    if (timeout_ms > 120000U) timeout_ms = 120000U;

    arm_motion_positive = motor_target >= motor->data.current_position ? 1U : 0U;
    arm_startup_move_issued_ms = now;
    ZDT_Stepper_Set_position_accel(motor, direction, arm_debug_speed, (uint8_t)arm_debug_accel,
                                    fabsf(motor_target), ZDT_STEPPER_ABS_POS_MODE,
                                   ZDT_STEPPER_SYNC_DISABLE);
    arm_motion_active = 1U;
    arm_speed_motion = 0U;
    arm_startup_deadline_ms = now + timeout_ms;
    arm_startup_state = ARM_STARTUP_MOVE_BALANCE;
}

static void process_arm_startup(ZDTStepperMotor *motor, uint32_t now)
{
    if (arm_startup_state < ARM_STARTUP_WAIT_HOME_ACK || arm_startup_state > ARM_STARTUP_MOVE_BALANCE) return;
    if ((int32_t)(now - arm_startup_deadline_ms) >= 0)
    {
        fail_arm_startup();
        return;
    }
    if ((arm_startup_state == ARM_STARTUP_WAIT_HOME_ACK || arm_startup_state == ARM_STARTUP_WAIT_HOME_DONE) &&
        now - arm_last_home_status_request_ms >= ARM_HOME_STATUS_PERIOD_MS)
    {
        arm_last_home_status_request_ms = now;
        ZDT_Stepper_Read_homing_status_flags(motor);
    }
    if (arm_startup_state == ARM_STARTUP_WAIT_HOME_ACK)
    {
        if (motor->data.last_command == 0x9AU &&
            (int32_t)(motor->data.last_command_ms - arm_zero_confirm_issued_ms) >= 0)
        {
            if (motor->data.last_command_status != 0x02U) fail_arm_startup();
            else arm_startup_state = ARM_STARTUP_WAIT_HOME_DONE;
        }
        return;
    }
    if (arm_startup_state == ARM_STARTUP_WAIT_HOME_DONE)
    {
        if ((int32_t)(motor->data.homing_status_ms - arm_zero_confirm_issued_ms) < 0) return;
        if (motor->data.homing_status_ms == arm_home_status_processed_ms) return;
        arm_home_status_processed_ms = motor->data.homing_status_ms;
        if ((int32_t)(motor->data.homing_status_ms - arm_zero_confirm_issued_ms) >= 0 &&
            (motor->data.homing_status_flags & 0x08U) != 0U)
        {
            fail_arm_startup();
            return;
        }
        if ((motor->data.homing_status_flags & 0x04U) != 0U)
        {
            arm_home_seen_active = 1U;
            arm_home_idle_samples = 0U;
            return;
        }
        if (arm_home_idle_samples < 0xFFU) arm_home_idle_samples++;
        if ((arm_home_seen_active != 0U || arm_home_idle_samples >= 3U) &&
            (motor->data.homing_status_flags & 0x03U) == 0x03U &&
            now - arm_zero_confirm_issued_ms >= ARM_HOME_SETTLE_MS)
        {
            ZDT_Stepper_clear_current_position(motor);
            arm_zero_confirm_issued_ms = now;
            arm_startup_deadline_ms = now + 1000U;
            arm_startup_state = ARM_STARTUP_WAIT_ZERO_ACK;
        }
        return;
    }
    if (arm_startup_state == ARM_STARTUP_WAIT_ZERO_ACK)
    {
        if (motor->data.last_command == 0x0AU &&
            (int32_t)(motor->data.last_command_ms - arm_zero_confirm_issued_ms) >= 0)
        {
            if (motor->data.last_command_status != 0x02U)
            {
                fail_arm_startup();
                return;
            }
            arm_zero_valid = 1U;
            float motor_limit_min;
            float motor_limit_max;
            arm_motor_limits(&motor_limit_min, &motor_limit_max);
            ZDT_Stepper_Set_Software_Limits(motor, motor_limit_min, motor_limit_max, true);
            arm_home_active = 0U;
            arm_home_complete = 1U;
            motor->data.current_position = 0.0f;
            motor->data.current_position_ms = now;
            move_arm_to_balance();
        }
        return;
    }
    if (arm_startup_state == ARM_STARTUP_MOVE_BALANCE)
    {
        if ((int32_t)(motor->data.current_position_ms - arm_startup_move_issued_ms) <= 0 ||
            (int32_t)(motor->data.current_speed_ms - arm_startup_move_issued_ms) <= 0)
        {
            return;
        }
        if (now - motor->data.current_position_ms > arm_motor_feedback_timeout_ms() ||
            now - motor->data.current_speed_ms > arm_motor_feedback_timeout_ms())
        {
            fail_arm_startup();
            return;
        }
    }
    if (fabsf(arm_coordinate_from_motor(motor->data.current_position) - arm_balance_position) <= ARM_BALANCE_TOLERANCE_DEG &&
        fabsf(motor->data.current_speed) <= 1.0f)
    {
        arm_motion_active = 0U;
        arm_startup_state = ARM_STARTUP_DONE;
        Arm_Competition_Task_Set_Ready(true);
    }
}

static bool handle_arm_command(uint8_t command_id)
{
    switch (command_id)
    {
    case ANO_COMMAND_ARM_CAPTURE_HIGH: return request_arm_calibration_capture(0U);
    case ANO_COMMAND_ARM_CAPTURE_LOW: return request_arm_calibration_capture(4U);
    case ANO_COMMAND_ARM_ENABLE:
        if (zdt_stepper_motors[0] == NULL) return false;
        ZDT_Stepper_Enable(zdt_stepper_motors[0], ZDT_STEPPER_ENABLE, ZDT_STEPPER_SYNC_DISABLE);
        return true;
    case ANO_COMMAND_ARM_DISABLE:
        if (zdt_stepper_motors[0] == NULL) return false;
        Arm_Competition_Task_Set_Ready(false);
        arm_ball_position_step_active = 0U;
        arm_ball_speed_step_active = 0U;
        (void)stop_arm_debug(false);
        ZDT_Stepper_Enable(zdt_stepper_motors[0], ZDT_STEPPER_DISABLE, ZDT_STEPPER_SYNC_DISABLE);
        return true;
    case ANO_COMMAND_ARM_TELEMETRY_ENABLE:
        arm_motor_telemetry_enabled = 1U;
        arm_last_motor_telemetry_ms = 0U;
        return true;
    case ANO_COMMAND_ARM_TELEMETRY_DISABLE:
        arm_motor_telemetry_enabled = 0U;
        return true;
    case ANO_COMMAND_ARM_CAPTURE_BALANCE: return request_arm_calibration_capture(2U);
    case ANO_COMMAND_ARM_BALL_LOOP_START:
        {
            arm_angle_control_config_t next = *Arm_Angle_Control_Get_Config();
            next.ball_position_loop_enabled = 1.0f;
            next.ball_velocity_loop_enabled = 1.0f;
            return Arm_Angle_Control_Apply_Config(&next);
        }
    case ANO_COMMAND_ARM_BALL_LOOP_STOP:
        {
            arm_angle_control_config_t next = *Arm_Angle_Control_Get_Config();
            arm_ball_position_step_active = 0U;
            arm_ball_speed_step_active = 0U;
            next.ball_position_loop_enabled = 0.0f;
            next.ball_velocity_loop_enabled = 0.0f;
            return Arm_Angle_Control_Apply_Config(&next);
        }
    case ANO_COMMAND_ARM_BALL_SPEED_STEP:
        {
            arm_angle_control_config_t next = *Arm_Angle_Control_Get_Config();
            arm_ball_position_step_active = 0U;
            arm_ball_speed_step_amplitude = fabsf(next.ball_target_velocity_cm_s);
            if (arm_ball_speed_step_amplitude < 0.1f) arm_ball_speed_step_amplitude = 2.0f;
            next.ball_position_loop_enabled = 0.0f;
            next.ball_velocity_loop_enabled = 1.0f;
            if (!Arm_Angle_Control_Apply_Config(&next) ||
                !Arm_Angle_Control_Set_Ball_Velocity_Target(0.0f)) return false;
            arm_ball_speed_step_phase = 0U;
            arm_ball_speed_step_started_ms = HAL_GetTick();
            arm_ball_speed_step_active = 1U;
            return true;
        }
    case ANO_COMMAND_ARM_BALL_POSITION_STEP:
        {
            arm_angle_control_config_t next = *Arm_Angle_Control_Get_Config();
            arm_ball_speed_step_active = 0U;
            next.ball_position_loop_enabled = 1.0f;
            next.ball_velocity_loop_enabled = 1.0f;
            next.ball_target_position_cm = 5.0f;
            if (!Arm_Angle_Control_Apply_Config(&next)) return false;
            arm_ball_position_step_phase = 0U;
            arm_ball_position_step_started_ms = HAL_GetTick();
            arm_ball_position_step_settled_ms = 0U;
            arm_ball_position_step_active = 1U;
            return true;
        }
    case ANO_COMMAND_ARM_COMPETITION_START:
        return Arm_Competition_Task_Start();
    case ANO_COMMAND_ARM_COMPETITION_CANCEL:
        Arm_Competition_Task_Cancel();
        return true;
    default: return false;
    }
}

static void process_arm_ball_speed_step(void)
{
    static const int8_t multipliers[5] = {0, 1, 0, -1, 0};
    uint32_t now;

    if (arm_ball_speed_step_active == 0U) return;
    now = HAL_GetTick();
    if ((uint32_t)(now - arm_ball_speed_step_started_ms) < 750U) return;
    arm_ball_speed_step_started_ms = now;
    arm_ball_speed_step_phase++;
    if (arm_ball_speed_step_phase >= 5U)
    {
        arm_ball_speed_step_active = 0U;
        (void)Arm_Angle_Control_Set_Ball_Velocity_Target(0.0f);
        return;
    }
    (void)Arm_Angle_Control_Set_Ball_Velocity_Target(
        arm_ball_speed_step_amplitude * (float)multipliers[arm_ball_speed_step_phase]);
}

static void process_arm_ball_position_step(void)
{
    const arm_angle_control_debug_t *debug = Arm_Angle_Control_Get_Debug();
    static const float targets[2] = {5.0f, -5.0f};
    const uint32_t now = HAL_GetTick();

    if (arm_ball_position_step_active == 0U) return;
    if (debug->vision_valid == 0U)
    {
        arm_ball_position_step_active = 0U;
        return;
    }
    if ((uint32_t)(now - arm_ball_position_step_started_ms) >=
        ARM_BALL_POSITION_STEP_PHASE_TIMEOUT_MS)
    {
        arm_ball_position_step_active = 0U;
        (void)Arm_Angle_Control_Set_Ball_Velocity_Target(0.0f);
        return;
    }

    if (fabsf(debug->vision_position_cm - targets[arm_ball_position_step_phase]) <=
            ARM_BALL_POSITION_STEP_TOLERANCE_CM &&
        fabsf(debug->vision_velocity_cm_s) <= 1.0f)
    {
        if (arm_ball_position_step_settled_ms == 0U)
            arm_ball_position_step_settled_ms = now;
        if ((uint32_t)(now - arm_ball_position_step_settled_ms) <
            ARM_BALL_POSITION_STEP_SETTLE_MS) return;
        if (arm_ball_position_step_phase >= 1U)
        {
            arm_ball_position_step_active = 0U;
            return;
        }
        arm_ball_position_step_phase++;
        arm_ball_position_step_started_ms = now;
        arm_ball_position_step_settled_ms = 0U;
        arm_angle_control_config_t next = *Arm_Angle_Control_Get_Config();
        next.ball_target_position_cm = targets[arm_ball_position_step_phase];
        (void)Arm_Angle_Control_Apply_Config(&next);
    }
    else
    {
        arm_ball_position_step_settled_ms = 0U;
    }
}

static void process_arm_debug(void)
{
    ZDTStepperMotor *motor = zdt_stepper_motors[0];
    uint32_t now = HAL_GetTick();
    uint32_t response_ms;

    if (motor == NULL) return;
    uint32_t query_period_ms = arm_motor_query_period_ms();
    if (arm_motor_query_pending != 0U)
    {
        response_ms = arm_motor_query_pending_position != 0U ?
                      motor->data.current_position_ms : motor->data.current_speed_ms;
        if (response_ms != arm_motor_query_response_baseline_ms ||
            now - arm_last_motor_query_ms >= ARM_MOTOR_QUERY_RESPONSE_TIMEOUT_MS)
        {
            arm_motor_query_pending = 0U;
        }
    }
    if (now - arm_last_motor_query_ms >= query_period_ms)
    {
        if (arm_motor_query_pending != 0U) goto process_feedback;
        arm_last_motor_query_ms = now;
        arm_motor_query_pending = 1U;
        arm_motor_query_pending_position = arm_motor_query_position;
        arm_motor_query_response_baseline_ms = arm_motor_query_position != 0U ?
                                               motor->data.current_position_ms :
                                               motor->data.current_speed_ms;
        if (arm_motor_query_position != 0U)
        {
            arm_last_position_request_ms = now;
            ZDT_Stepper_Read_current_position(motor);
        }
        else
        {
            arm_last_speed_request_ms = now;
            ZDT_Stepper_Read_current_speed(motor);
        }
        arm_motor_query_position ^= 1U;
    }
    if (now - arm_last_motor_status_request_ms >= 200U)
    {
        arm_last_motor_status_request_ms = now;
        ZDT_Stepper_Read_motor_status_flags(motor);
    }
    if (arm_capture_pending != 0U)
    {
        if (motor->data.current_position_ms != arm_capture_position_baseline_ms)
        {
            uint8_t index = arm_capture_index;

            arm_capture_pending = 0U;
            if (capture_arm_calibration_point(index))
            {
                send_arm_calibration_parameters(index);
            }
        }
        else if ((uint32_t)(now - arm_capture_requested_ms) >= ARM_CALIBRATION_CAPTURE_TIMEOUT_MS)
        {
            arm_capture_pending = 0U;
            arm_calibration_status = ANO_ARM_CALIBRATION_POSITION_TIMEOUT;
        }
    }
process_feedback:
    process_arm_calibration(motor, now);
    if (arm_zero_valid && motor->data.current_position_ms != 0U &&
        ((arm_coordinate_from_motor(motor->data.current_position) <= arm_limit_min &&
          arm_coordinate_from_motor(motor->data.current_speed) < -0.5f) ||
         (arm_coordinate_from_motor(motor->data.current_position) >= arm_limit_max &&
          arm_coordinate_from_motor(motor->data.current_speed) > 0.5f)))
    {
        ZDT_Stepper_stop(motor, ZDT_STEPPER_SYNC_DISABLE);
        arm_motion_active = 0U;
        if (arm_startup_state == ARM_STARTUP_MOVE_BALANCE) fail_arm_startup();
    }
    process_arm_startup(motor, now);
    if (arm_startup_state >= ARM_STARTUP_WAIT_HOME_ACK && arm_startup_state <= ARM_STARTUP_MOVE_BALANCE) return;
    if (arm_zero_confirm_pending && motor->data.last_command == 0x0AU &&
        motor->data.last_command_status == 0x02U &&
        (int32_t)(motor->data.last_command_ms - arm_zero_confirm_issued_ms) >= 0)
    {
        arm_zero_confirm_pending = 0U;
        arm_zero_valid = 1U;
        float motor_limit_min;
        float motor_limit_max;
        arm_motor_limits(&motor_limit_min, &motor_limit_max);
        ZDT_Stepper_Set_Software_Limits(motor, motor_limit_min, motor_limit_max, true);
        arm_home_active = 0U;
        motor->data.current_position = 0.0f;
        motor->data.current_position_ms = now;
    }
    if (arm_home_active)
    {
        if (now - arm_last_home_status_request_ms >= ARM_HOME_STATUS_PERIOD_MS)
        {
            arm_last_home_status_request_ms = now;
            ZDT_Stepper_Read_homing_status_flags(motor);
        }
        if ((motor->data.homing_status_flags & 0x08U) != 0U)
        {
            (void)stop_arm_debug(true);
            return;
        }
        if ((int32_t)(motor->data.homing_status_ms - arm_zero_confirm_issued_ms) >= 0 &&
            (motor->data.homing_status_flags & 0x04U) == 0U &&
            (motor->data.homing_status_flags & 0x03U) == 0x03U &&
            now - arm_zero_confirm_issued_ms >= ARM_HOME_SETTLE_MS)
        {
            arm_home_active = 0U;
            arm_home_complete = 1U;
            return;
        }
        if ((int32_t)(now - (arm_zero_confirm_issued_ms + (uint32_t)arm_debug_timeout_ms)) >= 0)
        {
            (void)stop_arm_debug(true);
        }
        return;
    }
    if (!arm_motion_active) return;
    float stopping_margin = arm_speed_motion ?
                            arm_debug_speed * 0.006f * (float)(arm_motor_query_period_ms() * 2U) : 0.0f;
    float arm_position = arm_coordinate_from_motor(motor->data.current_position);
    bool limit_reached = arm_motion_positive != 0U ?
                         arm_position + stopping_margin >= arm_limit_max :
                         arm_position - stopping_margin <= arm_limit_min;
    if ((int32_t)(now - arm_motion_deadline_ms) >= 0 ||
        motor->data.current_position_ms == 0U || now - motor->data.current_position_ms > arm_motor_feedback_timeout_ms() ||
        limit_reached)
    {
        if (arm_calibration_status == ANO_ARM_CALIBRATION_ACTIVE)
        {
            (void)stop_arm_debug(false);
            reset_arm_calibration_samples();
        }
        else
        {
            (void)stop_arm_debug(true);
        }
    }
}

static void send_arm_debug_diagnostic(void)
{
    ZDTStepperMotor *motor = zdt_stepper_motors[0];
    uint8_t d[8];
    uint16_t i = 0U;

    if (motor == NULL || !tx_has_capacity(1U)) return;
    put_float(d, &i, arm_coordinate_from_motor(motor->data.current_position));
    put_float(d, &i, motor->data.current_speed);
    send_frame(ANO_ID_ARM_DEBUG_DIAGNOSTIC, d, i);
}

static uint16_t feedback_age_ms(uint32_t now, uint32_t updated_ms)
{
    uint32_t age;

    if (updated_ms == 0U) return UINT16_MAX;
    age = now - updated_ms;
    return age > UINT16_MAX ? UINT16_MAX : (uint16_t)age;
}

static void send_arm_motor_telemetry(void)
{
    ZDTStepperMotor *motor = zdt_stepper_motors[0];
    const arm_angle_control_debug_t *angle = Arm_Angle_Control_Get_Debug();
    uint32_t now = HAL_GetTick();
    uint16_t speed_age;
    uint16_t position_age;
    uint8_t valid = 0U;
    uint8_t d[96];
    uint16_t i = 0U;

    if (motor == NULL || !tx_has_capacity(1U)) return;
    speed_age = feedback_age_ms(now, motor->data.current_speed_ms);
    position_age = feedback_age_ms(now, motor->data.current_position_ms);
    if (speed_age <= ARM_MOTOR_FEEDBACK_TIMEOUT_MS) valid |= ANO_ARM_MOTOR_SPEED_VALID;
    if (position_age <= ARM_MOTOR_FEEDBACK_TIMEOUT_MS) valid |= ANO_ARM_MOTOR_POSITION_VALID;

    d[i++] = 7U;
    d[i++] = motor->id;
    d[i++] = valid;
    d[i++] = motor->data.motor_status_flags;
    put_float(d, &i, motor->data.current_speed);
    put_float(d, &i, arm_coordinate_from_motor(motor->data.current_position));
    put_u16(d, &i, speed_age);
    put_u16(d, &i, position_age);
    put_u16(d, &i, (uint16_t)arm_motor_query_hz);
    put_u16(d, &i, (uint16_t)arm_motor_telemetry_hz);
    put_float(d, &i, angle->target_deg);
    /* measured_deg already uses the arm Pitch coordinate convention. */
    put_float(d, &i, angle->measured_deg);
    put_float(d, &i, angle->error_deg);
    put_float(d, &i, angle->output_rpm);
    d[i++] = angle->status;
    d[i++] = angle->enabled;
    d[i++] = angle->authorized;
    d[i++] = angle->imu_valid;
    d[i++] = arm_calibration_status;
    d[i++] = arm_calibration_stage;
    d[i++] = arm_calibration_captured_mask;
    d[i++] = arm_calibration_valid;
    d[i++] = motor->data.last_control_error_status;
    send_frame(ANO_ID_ARM_MOTOR_TELEMETRY, d, i);
}

static void send_arm_ball_telemetry(void)
{
    const arm_angle_control_debug_t *angle = Arm_Angle_Control_Get_Debug();
    uint8_t d[96];
    uint16_t i = 0U;

    if (!tx_has_capacity(1U)) return;
    put_float(d, &i, angle->ball_target_position_cm);
    put_float(d, &i, angle->ball_position_p_output_cm_s);
    put_float(d, &i, angle->ball_target_velocity_cm_s);
    put_float(d, &i, angle->ball_velocity_p_output_deg);
    put_float(d, &i, angle->vision_velocity_cm_s);
    put_float(d, &i, angle->vision_beam_angle_deg);
    put_float(d, &i, angle->vision_feedforward_acceleration_cm_s2);
    put_float(d, &i, angle->vision_prediction_ms);
    put_float(d, &i, angle->ball_velocity_error_cm_s);
    put_float(d, &i, angle->ball_velocity_i_output_deg);
    put_float(d, &i, angle->ball_velocity_d_output_deg);
    put_float(d, &i, angle->ball_velocity_raw_output_deg);
    put_float(d, &i, angle->motor_position_offset_deg);
    put_float(d, &i, angle->motor_trajectory_position_deg);
    put_float(d, &i, angle->motor_trajectory_velocity_deg_s);
    put_float(d, &i, angle->vision_raw_position_cm);
    put_float(d, &i, angle->vision_raw_velocity_cm_s);
    put_float(d, &i, angle->ball_brake_target_acceleration_cm_s2);
    put_float(d, &i, angle->ball_brake_raw_angle_deg);
    put_float(d, &i, angle->ball_brake_angle_deg);
    put_float(d, &i, angle->ball_combined_position_offset_deg);
    put_float(d, &i, angle->motor_target_position_deg);
    d[i++] = angle->ball_velocity_output_limited;
    d[i++] = arm_ball_speed_step_active;
    d[i++] = arm_ball_speed_step_phase;
    d[i++] = arm_ball_position_step_active;
    d[i++] = arm_ball_position_step_phase;
    send_frame(ANO_ID_ARM_BALL_TELEMETRY, d, i);
}

static void send_vision_telemetry(void)
{
    vision_link_sample_t vision = {0};
    vision_link_diagnostic_t diagnostic = {0};
    uint32_t now = HAL_GetTick();
    uint16_t age = UINT16_MAX;
    uint8_t d[39];
    uint16_t i = 0U;
    bool valid = Vision_Link_Get_Last_Sample(&vision);

    if (valid)
    {
        age = feedback_age_ms(now, vision.received_ms);
        valid = Vision_Link_Is_Online();
    }
    Vision_Link_Get_Diagnostic(&diagnostic);
    put_float(d, &i, vision.position_cm);
    put_float(d, &i, vision.velocity_cm_s);
    d[i++] = valid ? 1U : 0U;
    put_u16(d, &i, age);
    put_i32(d, &i, (int32_t)diagnostic.received_bytes);
    put_i32(d, &i, (int32_t)diagnostic.valid_frames);
    put_i32(d, &i, (int32_t)diagnostic.parse_errors);
    put_i32(d, &i, (int32_t)diagnostic.uart_errors);
    put_i32(d, &i, (int32_t)diagnostic.last_uart_error);
    put_i32(d, &i, (int32_t)diagnostic.last_uart_isr);
    put_i32(d, &i, (int32_t)diagnostic.restart_failures);
    send_frame(ANO_ID_VISION_TELEMETRY, d, i);
}

static void stop_car_b0_calibration(uint8_t status)
{
    car_b0_calibration_active = 0U;
    car_b0_calibration_phase = 0U;
    car_b0_requested_differential = 0;
    car_b0_applied_differential = 0;
    car_b0_calibration_command_status = status;
    (void)Car_Motor_Set_Speed(0, 0, 0, 0);
}

static bool start_car_b0_calibration(void)
{
    if (App_Role_Get() != APP_ROLE_CAR || car_b0_calibration_active != 0U)
    {
        return false;
    }

    (void)Car_Lap_Task_Stop();
    Car_Attitude_Control_Cancel_Step();
    if (car_debug_active != 0U) (void)stop_car_debug();
    car_b0_calibration_active = 1U;
    car_b0_calibration_phase = 0U;
    car_b0_calibration_index = 0U;
    car_b0_requested_differential = 0;
    car_b0_applied_differential = 0;
    car_b0_calibration_command_status = 1U;
    car_b0_phase_started_ms = HAL_GetTick();
    if (!Car_Motor_Set_Speed(0, 0, 0, 0))
    {
        car_b0_calibration_active = 0U;
        car_b0_calibration_command_status = 3U;
        return false;
    }
    return true;
}

static void process_car_b0_calibration(void)
{
    const uint32_t now = HAL_GetTick();

    if (car_b0_calibration_active == 0U) return;
    if (car_b0_calibration_phase == 0U)
    {
        int16_t differential;
        const uint8_t level_count = (uint8_t)car_b0_level_count;
        const uint8_t repeat_count = (uint8_t)car_b0_repeat_count;
        const uint8_t pulse_count = (uint8_t)(level_count * repeat_count * 2U);

        if ((uint32_t)(now - car_b0_phase_started_ms) < (uint32_t)car_b0_settle_duration_ms) return;
        if (car_b0_calibration_index >= pulse_count)
        {
            stop_car_b0_calibration(2U);
            return;
        }
        differential = (int16_t)(car_b0_start_differential +
                                 (float)(car_b0_calibration_index / (repeat_count * 2U)) *
                                 car_b0_differential_step);
        if ((car_b0_calibration_index & 1U) != 0U) differential = (int16_t)-differential;
        if (!Car_Motor_Set_Speed((int16_t)-differential, differential, 0, 0))
        {
            stop_car_b0_calibration(3U);
            return;
        }
        car_b0_requested_differential = differential;
        car_b0_applied_differential = differential;
        car_b0_calibration_phase = 1U;
        car_b0_phase_started_ms = now;
        return;
    }

    if ((uint32_t)(now - car_b0_phase_started_ms) < (uint32_t)car_b0_pulse_duration_ms) return;
    if (!Car_Motor_Set_Speed(0, 0, 0, 0))
    {
        stop_car_b0_calibration(3U);
        return;
    }
    car_b0_requested_differential = 0;
    car_b0_applied_differential = 0;
    car_b0_calibration_index++;
    car_b0_calibration_phase = 0U;
    car_b0_phase_started_ms = now;
}

static void send_car_b0_calibration_telemetry(void)
{
    car_imu_sample_t imu;
    const car_motor_telemetry_t *motor = Car_Motor_Get_Telemetry();
    uint8_t d[56];
    uint16_t i = 0U;
    const bool imu_valid = Car_IMU_Get_Sample(&imu);

    if (!tx_has_capacity(1U)) return;
    put_float(d, &i, (float)car_b0_calibration_active);
    put_float(d, &i, (float)car_b0_calibration_phase);
    put_float(d, &i, (float)car_b0_calibration_index);
    put_float(d, &i, (float)car_b0_requested_differential);
    put_float(d, &i, (float)car_b0_applied_differential);
    put_float(d, &i, (float)motor->applied_speed[0]);
    put_float(d, &i, (float)motor->applied_speed[1]);
    put_float(d, &i, motor->speed[0]);
    put_float(d, &i, motor->speed[1]);
    put_float(d, &i, imu_valid ? imu.angular_velocity_rad_s.z * 57.2957795f : 0.0f);
    put_float(d, &i, imu_valid ? imu.angle_rad.z * 57.2957795f : 0.0f);
    put_float(d, &i, motor->voltage);
    put_float(d, &i, (float)motor->command_status);
    put_float(d, &i, (float)car_b0_calibration_command_status);
    send_frame(ANO_ID_B0_CALIBRATION, d, i);
}

static void send_telemetry(void)
{
    const car_motor_telemetry_t *car = Car_Motor_Get_Telemetry(); uint8_t d[ANO_DATA_MAX]; uint16_t i = 0U;
    for (uint8_t n = 0U; n < 2U; n++) put_float(d, &i, car->encoder_upload_active ? (float)car->total_encoder[n] : 0.0f);
    for (uint8_t n = 0U; n < 2U; n++) put_float(d, &i, car->encoder_upload_active ? (float)car->encoder_delta_10ms[n] : 0.0f);
    for (uint8_t n = 0U; n < 2U; n++) put_float(d, &i, car->encoder_upload_active ? car->speed[n] : 0.0f);
    put_float(d, &i, car->voltage);
    put_float(d, &i, (float)car->voltage_valid);
    send_frame(ANO_ID_TELEMETRY, d, i);
}

static void send_control_telemetry(void)
{
    const car_attitude_control_debug_t *control = Car_Attitude_Control_Get_Debug();
    const car_attitude_control_config_t *control_config = Car_Attitude_Control_Get_Config();
    uint8_t d[40];
    uint16_t i = 0U;

    if (!tx_has_capacity(3U)) return;

    put_float(d, &i, control->target_heading);
    put_float(d, &i, control->tracking_angle);
    put_float(d, &i, control->target_rate);
    put_float(d, &i, control->measured_rate);
    put_float(d, &i, control->heading_p);
    put_float(d, &i, control->heading_i);
    put_float(d, &i, control->heading_d);
    put_float(d, &i, control->rate_p);
    put_float(d, &i, control->rate_i);
    put_float(d, &i, control->rate_d);
    send_frame(ANO_ID_CONTROL_PID, d, i);

    i = 0U;
    put_float(d, &i, control->differential);
    put_float(d, &i, control->left_target);
    put_float(d, &i, control->right_target);
    put_float(d, &i, control->ladrc_z1);
    put_float(d, &i, control->ladrc_z2);
    put_float(d, &i, control->ladrc_output);
    put_float(d, &i, control_config->mode);
    put_float(d, &i, (float)control->status);
    send_frame(ANO_ID_CONTROL_OUTPUT, d, i);
}

static void send_tracking_diagnostic(void)
{
    const car_attitude_control_debug_t *control = Car_Attitude_Control_Get_Debug();
    car_lap_status_t lap;
    const uint8_t gray = Gray_Sensor_Get_Stable();
    uint8_t d[72];
    uint16_t i = 0U;

    if (!tx_has_capacity(1U)) return;

    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        put_float(d, &i, (float)((gray >> bit) & 0x01U));
    }
    put_float(d, &i, control->optical_heading_correction_deg_s);
    Car_Lap_Task_Get_Status(&lap);
    put_float(d, &i, (float)lap.elapsed_ms * 0.001f);
    put_float(d, &i, control->tracking_angle);
    put_float(d, &i, control->target_heading);
    put_float(d, &i, (float)lap.half_turn_armed);
    put_float(d, &i, (float)lap.state);
    put_float(d, &i, (float)lap.segment);
    put_float(d, &i, (float)lap.odometry_valid);
    put_float(d, &i, (float)lap.geometry_tracking_valid);
    put_float(d, &i, control->arc_feedforward_rate);
    put_float(d, &i, control->arc_optical_rate);
    send_frame(ANO_ID_TRACKING_DIAGNOSTIC, d, i);
}

static int16_t float_to_i16(float value)
{
    if (value > 32767.0f) return INT16_MAX;
    if (value < -32768.0f) return INT16_MIN;
    return (int16_t)value;
}

static void send_imu(void)
{
    car_imu_sample_t imu;
    uint8_t d[14];
    uint16_t i;

    if (App_Role_Get() == APP_ROLE_ARM)
    {
        const jy61p_status_t *status = JY61P_Get_Status();
        const jy61p_sample_t *sample;

        if (!status->available || !status->sample_valid)
        {
            return;
        }
        sample = JY61P_Get_Sample();
        imu.acceleration_mps2.x = (float)sample->acceleration.x * (16.0f * 9.80665f / 32768.0f);
        imu.acceleration_mps2.y = (float)sample->acceleration.y * (16.0f * 9.80665f / 32768.0f);
        imu.acceleration_mps2.z = (float)sample->acceleration.z * (16.0f * 9.80665f / 32768.0f);
        imu.angular_velocity_rad_s.x = (float)sample->angular_velocity.x * (34.906585f / 32768.0f);
        imu.angular_velocity_rad_s.y = (float)sample->angular_velocity.y * (34.906585f / 32768.0f);
        imu.angular_velocity_rad_s.z = (float)sample->angular_velocity.z * (34.906585f / 32768.0f);
        imu.magnetic_field_mg.x = (float)sample->magnetic_field.x;
        imu.magnetic_field_mg.y = (float)sample->magnetic_field.y;
        imu.magnetic_field_mg.z = (float)sample->magnetic_field.z;
        imu.angle_rad.x = (float)sample->angle.x * (3.14159265f / 32768.0f);
        imu.angle_rad.y = (float)sample->angle.y * (3.14159265f / 32768.0f);
        imu.angle_rad.z = (float)sample->angle.z * (3.14159265f / 32768.0f);
        imu.quaternion_w = (float)sample->quaternion.w / 32768.0f;
        imu.quaternion_x = (float)sample->quaternion.x / 32768.0f;
        imu.quaternion_y = (float)sample->quaternion.y / 32768.0f;
        imu.quaternion_z = (float)sample->quaternion.z / 32768.0f;
        imu.temperature_c = (float)sample->temperature * 0.01f;
        imu.pressure_pa = 0.0f;
        imu.sequence = status->sequence;
        if (arm_pitch_inverted != 0.0f)
        {
            /* Rotate every exported IMU field into the arm's right-handed frame. */
            imu.acceleration_mps2.y = -imu.acceleration_mps2.y;
            imu.acceleration_mps2.z = -imu.acceleration_mps2.z;
            imu.angular_velocity_rad_s.y = -imu.angular_velocity_rad_s.y;
            imu.angular_velocity_rad_s.z = -imu.angular_velocity_rad_s.z;
            imu.magnetic_field_mg.y = -imu.magnetic_field_mg.y;
            imu.magnetic_field_mg.z = -imu.magnetic_field_mg.z;
            imu.angle_rad.y = -imu.angle_rad.y;
            imu.angle_rad.z = -imu.angle_rad.z;
            imu.quaternion_y = -imu.quaternion_y;
            imu.quaternion_z = -imu.quaternion_z;
        }
    }
    else if (!Car_IMU_Get_Sample(&imu))
    {
        return;
    }

    if (imu.sequence == last_imu_sequence || !tx_has_capacity(4U))
    {
        return;
    }

    i = 0U;
    put_i16(d, &i, float_to_i16(imu.acceleration_mps2.x * (32768.0f / (16.0f * 9.80665f))));
    put_i16(d, &i, float_to_i16(imu.acceleration_mps2.y * (32768.0f / (16.0f * 9.80665f))));
    put_i16(d, &i, float_to_i16(imu.acceleration_mps2.z * (32768.0f / (16.0f * 9.80665f))));
    put_i16(d, &i, float_to_i16(imu.angular_velocity_rad_s.x * (32768.0f / 34.906585f)));
    put_i16(d, &i, float_to_i16(imu.angular_velocity_rad_s.y * (32768.0f / 34.906585f)));
    put_i16(d, &i, float_to_i16(imu.angular_velocity_rad_s.z * (32768.0f / 34.906585f)));
    d[i++] = 0U;
    send_frame(ANO_ID_SENSOR_BASIC, d, i);

    i = 0U;
    put_i16(d, &i, float_to_i16(imu.magnetic_field_mg.x));
    put_i16(d, &i, float_to_i16(imu.magnetic_field_mg.y));
    put_i16(d, &i, float_to_i16(imu.magnetic_field_mg.z));
    put_i32(d, &i, (int32_t)imu.pressure_pa);
    put_i16(d, &i, float_to_i16(imu.temperature_c * 10.0f));
    d[i++] = 0U;
    d[i++] = 1U;
    send_frame(ANO_ID_SENSOR_EXTENDED, d, i);

    i = 0U;
    put_i16(d, &i, float_to_i16(imu.angle_rad.x * 5729.57795f));
    put_i16(d, &i, float_to_i16(imu.angle_rad.y * 5729.57795f));
    put_i16(d, &i, float_to_i16(imu.angle_rad.z * 5729.57795f));
    d[i++] = 1U;
    send_frame(ANO_ID_ATTITUDE_EULER, d, i);

    i = 0U;
    put_i16(d, &i, float_to_i16(imu.quaternion_w * 10000.0f));
    put_i16(d, &i, float_to_i16(imu.quaternion_x * 10000.0f));
    put_i16(d, &i, float_to_i16(imu.quaternion_y * 10000.0f));
    put_i16(d, &i, float_to_i16(imu.quaternion_z * 10000.0f));
    d[i++] = 1U;
    send_frame(ANO_ID_ATTITUDE_QUATERNION, d, i);

    last_imu_sequence = imu.sequence;
}

static void send_storage_diagnostic(void)
{
    uint8_t d[8];
    uint16_t i = 0U;
    d[i++] = storage_ready;
    d[i++] = storage_operation;
    d[i++] = (uint8_t)W25Qxx_GetLastStatus();
    d[i++] = 0U;
    put_i32(d, &i, (int32_t)W25Qxx_GetDeviceId());
    send_frame(ANO_ID_DIAGNOSTIC, d, i);
}

static bool command(const uint8_t *d, uint16_t n)
{
    if (App_Role_Get() == APP_ROLE_ARM)
    {
        if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_ARM_COMPETITION_SELECT)
            return Arm_Competition_Task_Select((arm_competition_task_id_t)d[2]);
        if (n == 3U && d[0] == 0x03U && d[2] == 0x00U) {
            return d[1] == ANO_COMMAND_ARM_CAPTURE_HIGH ||
                   d[1] == ANO_COMMAND_ARM_DISABLE ||
                   d[1] == ANO_COMMAND_ARM_ENABLE ||
                   d[1] == ANO_COMMAND_ARM_CAPTURE_LOW ||
                   d[1] == ANO_COMMAND_ARM_CAPTURE_BALANCE ||
                   d[1] == ANO_COMMAND_ARM_BALL_LOOP_START ||
                   d[1] == ANO_COMMAND_ARM_BALL_LOOP_STOP ||
                   d[1] == ANO_COMMAND_ARM_BALL_SPEED_STEP ||
                    d[1] == ANO_COMMAND_ARM_BALL_POSITION_STEP ||
                    d[1] == ANO_COMMAND_ARM_COMPETITION_START ||
                    d[1] == ANO_COMMAND_ARM_COMPETITION_CANCEL ||
                   d[1] == ANO_COMMAND_ARM_TELEMETRY_ENABLE ||
                   d[1] == ANO_COMMAND_ARM_TELEMETRY_DISABLE ? handle_arm_command(d[1]) : false;
        }
        if (n == 1U) {
            return d[0] == ANO_COMMAND_ARM_CAPTURE_HIGH ||
                   d[0] == ANO_COMMAND_ARM_DISABLE ||
                   d[0] == ANO_COMMAND_ARM_ENABLE ||
                   d[0] == ANO_COMMAND_ARM_CAPTURE_LOW ||
                   d[0] == ANO_COMMAND_ARM_CAPTURE_BALANCE ||
                   d[0] == ANO_COMMAND_ARM_BALL_LOOP_START ||
                   d[0] == ANO_COMMAND_ARM_BALL_LOOP_STOP ||
                   d[0] == ANO_COMMAND_ARM_BALL_SPEED_STEP ||
                    d[0] == ANO_COMMAND_ARM_BALL_POSITION_STEP ||
                    d[0] == ANO_COMMAND_ARM_COMPETITION_START ||
                    d[0] == ANO_COMMAND_ARM_COMPETITION_CANCEL ||
                   d[0] == ANO_COMMAND_ARM_TELEMETRY_ENABLE ||
                   d[0] == ANO_COMMAND_ARM_TELEMETRY_DISABLE ? handle_arm_command(d[0]) : false;
        }
        return false;
    }
    if (n == 3U && d[0] == 0x03U && d[1] == 0x03U && d[2] == 0x00U) { (void)Car_Lap_Task_Stop(); if (car_b0_calibration_active != 0U) { stop_car_b0_calibration(4U); return true; } return car_debug_active ? stop_car_debug() : Car_Motor_Set_Speed(0, 0, 0, 0); }
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_READ_FLASH && d[2] == 0x00U) return Car_Motor_Read_Flash();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_FACTORY_RESET && d[2] == 0x00U) return Car_Motor_Factory_Reset();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_APPLY_CONFIG && d[2] == 0x00U) return apply_car_config();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_DEBUG_ENTER && d[2] == 0x00U && car_b0_calibration_active == 0U) return enter_car_debug();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_DEBUG_START && d[2] == 0x00U && car_b0_calibration_active == 0U) return start_car_debug();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_DEBUG_STOP && d[2] == 0x00U) { if (car_b0_calibration_active != 0U) { stop_car_b0_calibration(4U); return true; } return stop_car_debug(); }
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_RESTORE_DEFAULTS && d[2] == 0x00U) return restore_defaults();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_ATTITUDE_ENABLE && d[2] == 0x00U && car_b0_calibration_active == 0U) { car_debug_active = 0U; return Car_Attitude_Control_Set_Enabled(true); }
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_ATTITUDE_DISABLE && d[2] == 0x00U) { if (car_b0_calibration_active != 0U) { stop_car_b0_calibration(4U); return true; } return Car_Lap_Task_Stop(); }
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_RATE_STEP_TEST && d[2] == 0x00U && car_b0_calibration_active == 0U) return start_attitude_step(CAR_ATTITUDE_STEP_RATE);
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_ANGLE_STEP_TEST && d[2] == 0x00U && car_b0_calibration_active == 0U) return start_attitude_step(CAR_ATTITUDE_STEP_ANGLE);
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_STEPPER_FF_CALIBRATE && d[2] == 0x00U) { Stepper_Feedforward_Reset_Bias(); return true; }
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_LAP_START && d[2] == 0x00U && car_b0_calibration_active == 0U) { if (car_debug_active) (void)stop_car_debug(); return Car_Lap_Task_Start(); }
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_LAP_STOP && d[2] == 0x00U) { if (car_b0_calibration_active != 0U) { stop_car_b0_calibration(4U); return true; } return Car_Lap_Task_Stop(); }
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_B0_CALIBRATE && d[2] == 0x00U) return start_car_b0_calibration();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_ROTATE_ANGLE && d[2] == 0x00U && car_b0_calibration_active == 0U) return start_attitude_rotation();
    if (n == 9U && d[0] == 1U && !car_debug_active && !Car_Attitude_Control_Is_Enabled() && car_b0_calibration_active == 0U) return Car_Motor_Set_Speed(get_i16(&d[1]), get_i16(&d[3]), get_i16(&d[5]), get_i16(&d[7]));
    if (n == 9U && d[0] == 2U && !car_debug_active && !Car_Attitude_Control_Is_Enabled() && car_b0_calibration_active == 0U) return Car_Motor_Set_PWM(get_i16(&d[1]), get_i16(&d[3]), get_i16(&d[5]), get_i16(&d[7]));
    return false;
}

static void handle(uint8_t source, uint8_t id, const uint8_t *d, uint16_t n, uint8_t sum, uint8_t add)
{
    if (id == ANO_ID_PARAM_COMMAND && n == 3U) {
        uint16_t v = get_u16(&d[1]);
        parameter_session_until_ms = HAL_GetTick() + ANO_PARAMETER_SESSION_MS;
        if (d[0] == 0U) send_device_info();
        else if (d[0] == 1U) {
            uint8_t count[3] = {1U, (uint8_t)parameter_protocol_count(), 0U};
            send_frame(ANO_ID_PARAM_COMMAND, count, 3U);
        }
        else if (d[0] == 2U) send_param(v);
        else if (d[0] == 3U) send_param_info(v);
        else if (d[0] == 0x10U && v == 0x00AAU) {
            send_check(source, id, sum, add);
            restore_parameter_defaults();
        }
        else if (d[0] == 0x10U && v == 0x00ABU) {
            schedule_parameter_save();
            send_check(source, id, sum, add);
        }
    }
    else if (id == ANO_ID_PARAM_VALUE && n == 6U) {
        bool accepted = false;
        uint16_t protocol_index = get_u16(d);
        uint16_t parameter_id = parameter_id_from_protocol_index(protocol_index);
        float value = get_float(&d[2]); float *v = parameter(parameter_id);
        parameter_session_until_ms = HAL_GetTick() + ANO_PARAMETER_SESSION_MS;
        if (parameter_id >= ANO_PARAM_COUNT) return;
        if (parameter_id == HOST_BAUD && isfinite(value) && value == (float)(uint32_t)value &&
            baud_rate_is_valid((uint32_t)value)) {
            requested_baud_rate = (uint32_t)value;
            accepted = true;
        } else if (parameter_id >= CAR_M1_DIRECTION && parameter_id <= CAR_M2_DIRECTION) {
            accepted = write_car_parameter(parameter_id, value);
        } else if (parameter_id > HOST_BAUD && parameter_id < CAR_M1_DIRECTION) {
            accepted = false;
        } else if ((parameter_id == CAR_M1_DIRECTION || parameter_id == CAR_M2_DIRECTION) &&
                   isfinite(value) && (value == 0.0f || value == 1.0f)) {
            accepted = write_car_parameter(parameter_id, value);
        } else if (parameter_id >= CAR_B0_START_DIFFERENTIAL && parameter_id <= CAR_B0_REPEAT_COUNT) {
            accepted = write_car_b0_calibration_parameter(parameter_id, value);
        } else if ((parameter_id >= ATTITUDE_MODE && parameter_id <= STEP_ANGLE_FEEDBACK_INVERTED) ||
                   parameter_id == IMU_RATE_Z_INVERTED ||
                   parameter_id == ATTITUDE_LADRC_TARGET_RATE_SLEW ||
                   parameter_id == ATTITUDE_LADRC_DIFFERENTIAL_SLEW) {
            accepted = write_attitude_parameter(parameter_id, value);
        } else if (parameter_id >= STEPPER_FF_ENABLE && parameter_id <= STEPPER_FF_OUTPUT_LIMIT) {
            accepted = write_feedforward_parameter(parameter_id, value);
        } else if (parameter_id >= TRACKING_LAYER1_WEIGHT && parameter_id <= TRACKING_LAYER4_WEIGHT) {
            accepted = write_tracking_parameter(parameter_id, value);
        } else if (parameter_id >= CAR_ARC_STRAIGHT_LENGTH && parameter_id <= CAR_ARC_DIFFERENTIAL_SLEW) {
            accepted = write_arc_parameter(parameter_id, value);
        } else if (parameter_id >= ARM_CALIBRATION_LEFT_MOTOR &&
                   parameter_id <= ARM_CALIBRATION_RIGHT_PITCH) {
            accepted = write_arm_calibration_parameter(parameter_id, value);
        } else if (parameter_id >= ARM_LIMIT_MIN && parameter_id <= ARM_BALANCE_POSITION) {
            accepted = write_arm_parameter(parameter_id, value);
        } else if ((parameter_id >= ARM_ANGLE_KP && parameter_id <= ARM_ANGLE_KD) ||
                   parameter_id == ARM_ANGLE_DERIVATIVE_FILTER ||
                   parameter_id == ARM_ANGLE_MAX_SPEED) {
            accepted = write_arm_angle_parameter(parameter_id, value);
        } else if ((parameter_id >= ARM_BALL_POSITION_LOOP_ENABLE && parameter_id <= ARM_MOTOR_SPEED_KD) ||
                   parameter_id == ARM_VISION_KALMAN_ENABLED || parameter_id == ARM_VISION_PREDICTION_ENABLED ||
                   parameter_id == ARM_BALL_TARGET_VELOCITY ||
                    (parameter_id >= ARM_MOTOR_POSITION_SPEED_RPM && parameter_id <= ARM_BALL_BRAKE_OUTPUT_INVERTED)) {
            accepted = write_arm_parameter(parameter_id, value);
        } else if (parameter_id == ARM_PITCH_INVERTED) {
            accepted = write_arm_parameter(parameter_id, value);
        } else if (parameter_id == CAR_LAP_STOP_DELAY_MS && isfinite(value) && value == floorf(value) &&
                   value >= 0.0f && value <= 2000.0f) {
            accepted = Car_Lap_Task_Set_Stop_Delay((uint32_t)value);
            if (accepted) car_lap_stop_delay_ms = value;
        } else if (parameter_id == CAR_LAP_COUNTS_PER_METER && isfinite(value)) {
            accepted = Car_Lap_Task_Set_Counts_Per_Meter(value);
            if (accepted) car_lap_counts_per_meter = value;
        } else if (parameter_id == GRAY_SENSOR_ACTIVE_LOW) {
            accepted = write_gray_sensor_parameter(value);
        } else if (v != NULL && isfinite(value) && value >= -1000.0f && value <= 1000.0f) {
            *v = value;
            accepted = true;
        }
        if (accepted) {
            send_check(source, id, sum, add);
            if (parameter_id != HOST_BAUD)
                apply_parameter_after_ack(parameter_id);
        }
    }
    else if (id == ANO_ID_COMMAND && command(d, n)) send_check(source, id, sum, add);
    else if (id == ANO_ID_COMMAND_QUERY) handle_command_query(d, n);
    else return;
}

static void feed(uint8_t b)
{
    switch (parser.state) {
    case 0U: if (b == ANO_HEAD) { parser.sum = b; parser.state = 1U; } break;
    case 1U: parser.source = b; parser.sum += b; parser.state++; break;
    case 2U: parser.destination = b; parser.sum += b; parser.state++; break;
    case 3U: parser.id = b; parser.sum += b; parser.state++; break;
    case 4U: parser.length = b; parser.sum += b; parser.state++; break;
    case 5U: parser.length |= (uint16_t)b << 8U; parser.sum += b; parser.index = 0U; parser.state = parser.length > ANO_DATA_MAX ? 0U : (parser.length ? 6U : 7U); break;
    case 6U: parser.data[parser.index++] = b; parser.sum += b; if (parser.index == parser.length) parser.state++; break;
    case 7U: if (b == parser.sum) { parser.checksum = b; parser.state = 8U; } else parser.state = 0U; break;
    case 8U:
        if (parser.destination == App_Role_Get_ANO_Address() || parser.destination == ANO_BROADCAST_ADDRESS) {
            uint8_t rolling = 0U, add = 0U;
            uint8_t header[6] = {ANO_HEAD, parser.source, parser.destination, parser.id, (uint8_t)parser.length, (uint8_t)(parser.length >> 8U)};
            for (uint8_t i = 0U; i < sizeof(header); i++) { rolling += header[i]; add += rolling; }
            for (uint16_t i = 0U; i < parser.length; i++) { rolling += parser.data[i]; add += rolling; }
            if (b == add) handle(parser.source, parser.id, parser.data, parser.length, parser.checksum, b);
        }
        parser.state = 0U;
        break;
    default: parser.state = 0U; break; }
}

void ANO_Host_Init(void)
{
    memset(arm_calibration_values, 0, sizeof(arm_calibration_values));
    memset(arm_calibration_pending, 0, sizeof(arm_calibration_pending));
    arm_calibration_status = ANO_ARM_CALIBRATION_IDLE;
    arm_calibration_stage = 0U;
    arm_calibration_captured_mask = 0U;
    arm_calibration_valid = 0U;
    arm_calibration_sample_count = 0U;
    arm_calibration_sample_index = 0U;
    arm_calibration_last_sequence = 0U;
    arm_calibration_last_sample_ms = 0U;
    arm_calibration_sequence = 0U;
    arm_calibration_pending_mask = 0U;
    arm_calibration_pitch_sum = 0.0f;
    arm_calibration_pitch_min = 0.0f;
    arm_calibration_pitch_max = 0.0f;
    memset(arm_calibration_pitch_samples, 0, sizeof(arm_calibration_pitch_samples));
    reset_local_defaults();
    last_control_telemetry_ms = HAL_GetTick();
    last_vision_telemetry_ms = HAL_GetTick();
    storage_ready = W25Qxx_Init() ? 1U : 0U;
    if (storage_ready)
    {
        load_storage();
        if (App_Role_Get() == APP_ROLE_ARM)
        {
            load_arm_angle_config();
            load_arm_calibration();
        }
    }
    if (App_Role_Get() == APP_ROLE_ARM) start_arm_startup();
    if (host_baud_rate != huart1.Init.BaudRate) apply_baud_rate(host_baud_rate);
    else HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}
void ANO_Host_Process(void)
{
    while (rx_head != rx_tail)
    {
        uint8_t b = rx_queue[rx_head];
        rx_head = (rx_head + 1U) % ANO_RX_QUEUE_SIZE;
        feed(b);
    }
    if (App_Role_Get() == APP_ROLE_ARM) { process_arm_debug(); process_arm_ball_speed_step(); process_arm_ball_position_step(); }
    else process_car_b0_calibration();
    start_tx();
    process_parameter_save();
    if (pending_baud_rate != 0U && (int32_t)(HAL_GetTick() - baud_switch_at_ms) >= 0 &&
        !tx_active && response_count == 0U && tx_count == 0U)
    {
        uint32_t baud_rate = pending_baud_rate;
        pending_baud_rate = 0U;
        apply_baud_rate(baud_rate);
    }
    if (parameter_session_active()) return;
    if (App_Role_Get() == APP_ROLE_ARM && arm_motor_telemetry_enabled != 0U)
    {
        uint32_t telemetry_period_ms = (uint32_t)(1000.0f / arm_motor_telemetry_hz);
        if (telemetry_period_ms == 0U) telemetry_period_ms = 1U;
        if (HAL_GetTick() - arm_last_motor_telemetry_ms >= telemetry_period_ms)
        {
            arm_last_motor_telemetry_ms = HAL_GetTick();
            send_arm_motor_telemetry();
            send_arm_ball_telemetry();
        }
    }
    if (App_Role_Get() == APP_ROLE_ARM)
    {
        uint32_t vision_period_ms = (uint32_t)(1000.0f / ARM_MOTOR_TELEMETRY_DEFAULT_HZ);
        if (vision_period_ms == 0U) vision_period_ms = 1U;
        if (HAL_GetTick() - last_vision_telemetry_ms >= vision_period_ms)
        {
            last_vision_telemetry_ms = HAL_GetTick();
            send_vision_telemetry();
        }
    }
    if (HAL_GetTick() - last_imu_ms >= ANO_IMU_PERIOD_MS)
    {
        last_imu_ms = HAL_GetTick();
        send_imu();
    }
    if (HAL_GetTick() - last_control_telemetry_ms >= 20U)
    {
        last_control_telemetry_ms = HAL_GetTick();
        if (App_Role_Get() == APP_ROLE_CAR)
        {
            send_control_telemetry();
            send_tracking_diagnostic();
            send_car_b0_calibration_telemetry();
        }
        else
        {
            send_arm_debug_diagnostic();
        }
    }
    if (App_Role_Get() == APP_ROLE_CAR && HAL_GetTick() - last_telemetry_ms >= 100U)
    {
        last_telemetry_ms = HAL_GetTick();
        send_telemetry();
    }
    if (HAL_GetTick() - last_diagnostic_ms >= 500U)
    {
        last_diagnostic_ms = HAL_GetTick();
        send_storage_diagnostic();
    }
}
void ANO_Host_UART_RxCplt_Callback(void) { uint8_t next = (rx_tail + 1U) % ANO_RX_QUEUE_SIZE; if (next != rx_head) { rx_queue[rx_tail] = rx_byte; rx_tail = next; } HAL_UART_Receive_IT(&huart1, &rx_byte, 1U); }
void ANO_Host_UART_TxCplt_Callback(void)
{
    if (tx_active)
    {
        if (tx_active_response && response_count != 0U)
        {
            response_head = (response_head + 1U) % ANO_RESPONSE_QUEUE_DEPTH;
            response_count--;
        }
        else if (!tx_active_response && tx_count != 0U)
        {
            tx_head = (tx_head + 1U) % ANO_TX_QUEUE_DEPTH;
            tx_count--;
        }
    }
    tx_active = 0U;
    start_tx();
}
void ANO_Host_UART_Error_Callback(void)
{
    tx_active = 0U;
    start_tx();
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}
uint32_t ANO_Host_Get_BaudRate(void) { return host_baud_rate; }
