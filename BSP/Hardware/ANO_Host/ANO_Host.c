#include "ANO_Host.h"

#include "Car_Motor_USART.h"
#include "Car_Attitude_Control.h"
#include "Car_Lap_Task.h"
#include "Car_IMU.h"
#include "Gray_Sensor.h"
#include "JY61P.h"
#include "Stepper_PID.h"
#include "Tracking_Link.h"
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
#define ANO_ID_STEPPER_FF_DIAGNOSTIC 0xF6U
#define ANO_ID_ARM_DEBUG_DIAGNOSTIC 0xF7U
#define ANO_RX_QUEUE_SIZE 128U
#define ANO_DATA_MAX 96U
#define ANO_TX_QUEUE_DEPTH 8U
#define ANO_IMU_PERIOD_MS 10U
#define ANO_DEFAULT_BAUD_RATE 460800U
#define ANO_BAUD_SWITCH_DELAY_MS 50U
#define ANO_STORAGE_ADDRESS 0U
#define ANO_STORAGE_MAGIC 0x414E4F57UL
#define ANO_STORAGE_VERSION 13U
#define ARM_DEBUG_POSITION_PERIOD_MS 20U
#define ARM_DEBUG_POSITION_STALE_MS 100U
#define ARM_HOME_STATUS_PERIOD_MS 50U
#define ARM_HOME_SETTLE_MS 100U
#define ARM_STARTUP_HOME_TIMEOUT_MS 22000U
#define ARM_STARTUP_MOVE_TIMEOUT_MS 10000U
#define ARM_BALANCE_TOLERANCE_DEG 0.5f

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
    TRACKING_DIRECTION_INVERTED,
    TRACKING_ERROR_FILTER_TIME,
    TRACKING_TARGET_RATE_SLEW,
    IMU_SOURCE,
    TRACKING_LINE_INITIAL_WEIGHT,
    TRACKING_LINE_WEIGHT_RAMP_TIME,
    TRACKING_LINE_RELEASE_TIME,
    TRACKING_LINE_HEADING_RATE,
    TRACKING_HEADING_ERROR_LIMIT,
    LAP_HALF_TURN_ANGLE,
    LAP_HALF_TURN_WINDOW,
    LAP_HALF_TURN_CONFIRM_MS,
    LAP_CROSS_CONFIRM_MS,
    TRACKING_INNER_POSITION_WEIGHT,
    TRACKING_OUTER_POSITION_WEIGHT,
    TRACKING_TIME_CURVE_EXPONENT,
    TRACKING_OPTICAL_FUSION_WEIGHT,
    TRACKING_IMU_FUSION_WEIGHT,
    ANO_PARAM_COUNT
};

typedef struct { uint8_t state, source, destination, id, sum, checksum; uint16_t length, index; uint8_t data[ANO_DATA_MAX]; } ano_parser_t;
typedef struct { uint8_t cid[3]; const char *name; const char *info; } ano_command_desc_t;

static const ano_command_desc_t commands[] = {
    {{0x03U, 0x03U, 0x00U}, "四驱停止", "将四路电机速度设为零"},
    {{0x02U, 0x12U, 0xFFU}, "ZDT电机停止", "立即停止臂杆PITCH轴ZDT闭环步进电机"},
    {{0x03U, ANO_COMMAND_CAR_READ_FLASH, 0x00U}, "读取四驱配置", "请求四路电机驱动读取保存的配置"},
    {{0x03U, ANO_COMMAND_CAR_FACTORY_RESET, 0x00U}, "四驱恢复出厂", "请求四路电机驱动恢复出厂配置"},
    {{0x03U, ANO_COMMAND_CAR_APPLY_CONFIG, 0x00U}, "下发四驱配置", "按类型、死区、相位线、减速比、轮径和PID下发当前配置"},
    {{0x03U, ANO_COMMAND_CAR_DEBUG_ENTER, 0x00U}, "进入M1M2调试", "四路PWM置零后允许M1和M2调试"},
    {{0x03U, ANO_COMMAND_CAR_DEBUG_START, 0x00U}, "启动M1M2调试", "按调试参数仅输出M1和M2，M3和M4为零"},
    {{0x03U, ANO_COMMAND_CAR_DEBUG_STOP, 0x00U}, "停止M1M2调试", "四路PWM置零并退出M1和M2调试模式"},
    {{0x03U, ANO_COMMAND_CAR_RESTORE_DEFAULTS, 0x00U}, "完整恢复默认", "清除本机配置并恢复电机驱动默认设置"},
    {{0x03U, ANO_COMMAND_ATTITUDE_ENABLE, 0x00U}, "启动陀螺仪控制", "启动航向和角速度双环控制"},
    {{0x03U, ANO_COMMAND_ATTITUDE_DISABLE, 0x00U}, "停止陀螺仪控制", "停止双环控制并将M1和M2速度置零"},
    {{0x03U, ANO_COMMAND_RATE_STEP_TEST, 0x00U}, "角速度阶跃测试", "按配置幅度执行角速度正阶跃和回零响应"},
    {{0x03U, ANO_COMMAND_ANGLE_STEP_TEST, 0x00U}, "航向角阶跃测试", "按配置幅度执行相对航向角正阶跃和回零响应"},
    {{0x03U, ANO_COMMAND_STEPPER_FF_CALIBRATE, 0x00U}, "前馈零偏校准", "保持小车静止并重新采集一秒IMU加速度零偏"},
    {{0x03U, ANO_COMMAND_CAR_LAP_START, 0x00U}, "启动一圈停车", "启用循迹并计时，返回A点十字后按补偿停车"},
    {{0x03U, ANO_COMMAND_CAR_LAP_STOP, 0x00U}, "停止一圈任务", "立即停止循迹并冻结本圈时间、里程和平均速度"},
    {{0x03U, ANO_COMMAND_CAR_LAP_CLEAR, 0x00U}, "清除一圈记录", "清除本圈状态、时间、里程和平均速度"},
    {{0x03U, ANO_COMMAND_ARM_DEBUG_ENTER, 0x00U}, "进入臂杆调试", "停止闭环输出并开放受限的臂杆调试命令"},
    {{0x03U, ANO_COMMAND_ARM_DEBUG_EXIT, 0x00U}, "退出臂杆调试", "立即停止电机并退出臂杆调试模式"},
    {{0x03U, ANO_COMMAND_ARM_HOME, 0x00U}, "臂杆电机回零", "按回零模式触发驱动器回零，完成后需确认零点"},
    {{0x03U, ANO_COMMAND_ARM_ZERO_CONFIRM, 0x00U}, "确认臂杆零点", "当前位置确认在机械零点后清零，驱动应答成功后开放运动"},
    {{0x03U, ANO_COMMAND_ARM_RUN_SPEED, 0x00U}, "臂杆速度转动", "按方向和速度转动，达到限位或超时自动停止"},
    {{0x03U, ANO_COMMAND_ARM_MOVE_ABSOLUTE, 0x00U}, "臂杆绝对定位", "移动到目标绝对角度，越过软件限位时拒绝"},
    {{0x03U, ANO_COMMAND_ARM_STOP, 0x00U}, "臂杆电机停止", "立即停止臂杆步进电机，重复执行安全"},
    {{0x03U, ANO_COMMAND_ARM_ENABLE, 0x00U}, "臂杆电机使能", "使能臂杆步进电机驱动"},
    {{0x03U, ANO_COMMAND_ARM_DISABLE, 0x00U}, "臂杆电机失能", "先停止再关闭臂杆步进电机驱动"},
    {{0x03U, ANO_COMMAND_ARM_APPLY_DRIVER, 0x00U}, "应用驱动配置", "将当前细分配置写入步进电机驱动器"}
};

static uint8_t rx_byte;
static volatile uint8_t rx_queue[ANO_RX_QUEUE_SIZE], rx_head, rx_tail;
static uint8_t tx_queue[ANO_TX_QUEUE_DEPTH][ANO_DATA_MAX + 8U], tx_length[ANO_TX_QUEUE_DEPTH];
static volatile uint8_t tx_head, tx_tail, tx_count, tx_active;
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
static uint8_t car_debug_active;
static uint32_t last_telemetry_ms;
static uint32_t last_control_telemetry_ms;
static uint32_t last_diagnostic_ms;
static uint32_t last_imu_ms;
static uint32_t last_imu_sequence;
static uint32_t host_baud_rate = ANO_DEFAULT_BAUD_RATE;
static uint32_t pending_baud_rate;
static uint32_t baud_switch_at_ms;
static uint8_t storage_ready;
static uint8_t storage_operation;
static float board_role;
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
static float imu_source;
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
    float direction_inverted;
    float error_filter_time_s;
    float target_rate_slew_deg_s2;
} ano_tracking_config_v11_t;

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
} ano_tracking_config_v12_t;

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
    car_lap_config_t lap;
    uint8_t imu_source;
    uint8_t reserved_2[3];
    uint32_t checksum;
} ano_storage_record_t;

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
    ano_tracking_config_v12_t tracking;
    car_lap_config_t lap;
    uint8_t imu_source;
    uint8_t reserved_2[3];
    uint32_t checksum;
} ano_storage_record_v12_t;

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
    ano_tracking_config_v11_t tracking;
    uint8_t imu_source;
    uint8_t reserved_2[3];
    uint32_t checksum;
} ano_storage_record_v11_t;

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
    ano_tracking_config_v11_t tracking;
    uint32_t checksum;
} ano_storage_record_v10_t;

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
    uint32_t checksum;
} ano_storage_record_v9_t;

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
    uint32_t checksum;
} ano_storage_record_v8_t;

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
    struct
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
    } arm_debug;
    uint32_t checksum;
} ano_storage_record_v7_t;

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
    uint32_t checksum;
} ano_storage_record_v6_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint32_t baud_rate;
    uint8_t reserved[3];
    float values[HOST_BAUD];
    float directions[2];
    car_attitude_control_config_t attitude;
    uint32_t checksum;
} ano_storage_record_v5_t;

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
} ano_attitude_config_v3_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint32_t baud_rate;
    uint8_t reserved[3];
    float values[HOST_BAUD];
    float directions[2];
    ano_attitude_config_v3_t attitude;
    float step_rate_amplitude;
    float step_angle_amplitude;
    float step_hold_ms;
    uint32_t checksum;
} ano_storage_record_v4_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint32_t baud_rate;
    uint8_t reserved[3];
    float values[HOST_BAUD];
    float directions[2];
    ano_attitude_config_v3_t attitude;
    uint32_t checksum;
} ano_storage_record_v3_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint32_t baud_rate;
    uint8_t reserved[3];
    float values[HOST_BAUD];
    float directions[2];
    uint32_t checksum;
} ano_storage_record_v2_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint32_t baud_rate;
    uint8_t reserved[3];
    float values[HOST_BAUD];
    uint32_t checksum;
} ano_storage_record_v1_t;

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
    const car_lap_config_t lap_defaults = {180.0f, 30.0f, 100.0f, 50.0f};
    board_role = (float)App_Role_Get();
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
    imu_source = (float)CAR_IMU_SOURCE_JY61P;
    (void)Car_IMU_Set_Source(CAR_IMU_SOURCE_JY61P);
    (void)Car_Lap_Task_Set_Stop_Delay(0U);
    (void)Car_Lap_Task_Set_Counts_Per_Meter(car_lap_counts_per_meter);
    (void)Car_Lap_Task_Apply_Config(&lap_defaults);
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
    arm_home_status_processed_ms = 0U;
    arm_startup_state = ARM_STARTUP_IDLE;
    arm_startup_move_issued_ms = 0U;
    Car_Attitude_Control_Reset_Defaults();
    Stepper_Feedforward_Reset_Defaults();

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

static uint32_t storage_record_v1_checksum(const ano_storage_record_v1_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x6D2B79F5U;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++) checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t storage_record_v2_checksum(const ano_storage_record_v2_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x6D2B79F5U;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++) checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t storage_record_v3_checksum(const ano_storage_record_v3_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x6D2B79F5U;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++) checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t storage_record_v4_checksum(const ano_storage_record_v4_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x6D2B79F5U;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++) checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t storage_record_v5_checksum(const ano_storage_record_v5_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x6D2B79F5U;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++) checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t storage_record_v6_checksum(const ano_storage_record_v6_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x6D2B79F5U;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++) checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t storage_record_v7_checksum(const ano_storage_record_v7_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x6D2B79F5U;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++) checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t storage_record_v8_checksum(const ano_storage_record_v8_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x6D2B79F5U;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++) checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t storage_record_v9_checksum(const ano_storage_record_v9_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x6D2B79F5U;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++) checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t storage_record_v10_checksum(const ano_storage_record_v10_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x6D2B79F5U;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++) checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t storage_record_v11_checksum(const ano_storage_record_v11_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x6D2B79F5U;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++) checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static uint32_t storage_record_v12_checksum(const ano_storage_record_v12_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x6D2B79F5U;
    for (uint16_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++) checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    return checksum;
}

static float *parameter(uint16_t id);
static bool write_car_parameter(uint16_t parameter_id, float value);
static bool stop_arm_debug(bool exit_debug);
static bool arm_points_valid(void);

static bool save_storage(uint32_t baud_rate)
{
    ano_storage_record_t record = {0};
    ano_storage_record_t verify = {0};
    if (App_Role_Get() == APP_ROLE_ARM && !arm_points_valid()) return false;
    storage_operation = 3U;
    record.magic = ANO_STORAGE_MAGIC;
    record.version = ANO_STORAGE_VERSION;
    record.baud_rate = baud_rate;
    for (uint16_t id = 0U; id < HOST_BAUD; id++)
    {
        float *value = parameter(id);
        if (value == NULL || !isfinite(*value))
        {
            storage_operation = 3U;
            return false;
        }
        record.values[id] = *value;
    }
    record.directions[0] = car_m1_direction;
    record.directions[1] = car_m2_direction;
    record.attitude = *Car_Attitude_Control_Get_Config();
    record.feedforward = *Stepper_Feedforward_Get_Config();
    record.tracking = *Car_Attitude_Control_Get_Tracking_Config();
    record.lap = *Car_Lap_Task_Get_Config();
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
    record.imu_source = (uint8_t)Car_IMU_Get_Source();
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
        record.magic != ANO_STORAGE_MAGIC || !baud_rate_is_valid(record.baud_rate))
    {
        return;
    }
    if (record.version == ANO_STORAGE_VERSION && record.checksum == storage_record_checksum(&record))
    {
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
        (void)Car_Attitude_Control_Apply_Tracking_Config(&record.tracking);
        (void)Car_Lap_Task_Apply_Config(&record.lap);
        if (record.imu_source <= (uint8_t)CAR_IMU_SOURCE_WHEELTEC)
        {
            imu_source = (float)record.imu_source;
            (void)Car_IMU_Set_Source((car_imu_source_t)record.imu_source);
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
        (void)Car_Attitude_Control_Apply_Config(&legacy.attitude);
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
        (void)Car_Attitude_Control_Apply_Config(&legacy.attitude);
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
        (void)Car_Attitude_Control_Apply_Config(&legacy.attitude);
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
        (void)Car_Attitude_Control_Apply_Config(&legacy.attitude);
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
        (void)Car_Attitude_Control_Apply_Config(&legacy.attitude);
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
        (void)Car_Attitude_Control_Apply_Config(&legacy.attitude);
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
        (void)Car_Attitude_Control_Apply_Config(&legacy.attitude);
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
        (void)Car_Attitude_Control_Apply_Config(&legacy.attitude);
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
    if (!tx_active && tx_count != 0U) { tx_active = 1U; if (HAL_UART_Transmit_IT(&huart1, tx_queue[tx_head], tx_length[tx_head]) != HAL_OK) tx_active = 0U; }
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
    uint8_t sum = 0U, add = 0U; uint16_t index = 0U; uint32_t primask;
    if (length > ANO_DATA_MAX) return;
    primask = __get_PRIMASK(); __disable_irq();
    if (tx_count >= ANO_TX_QUEUE_DEPTH) { __set_PRIMASK(primask); return; }
    uint8_t *frame = tx_queue[tx_tail];
    frame[index++] = ANO_HEAD; frame[index++] = App_Role_Get_ANO_Address(); frame[index++] = destination; frame[index++] = id; frame[index++] = (uint8_t)length; frame[index++] = (uint8_t)(length >> 8U);
    if (length != 0U)
    {
        memcpy(&frame[index], data, length);
    }
    index += length;
    for (uint16_t i = 0U; i < index; i++) { sum += frame[i]; add += sum; }
    frame[index++] = sum; frame[index++] = add; tx_length[tx_tail] = (uint8_t)index; tx_tail = (tx_tail + 1U) % ANO_TX_QUEUE_DEPTH; tx_count++; start_tx(); __set_PRIMASK(primask);
}

static void send_frame(uint8_t id, const uint8_t *data, uint16_t length)
{
    send_frame_to(ANO_HOST_ADDRESS, id, data, length);
}

static float *parameter(uint16_t id)
{
    car_attitude_control_config_t *attitude = Car_Attitude_Control_Get_Config();
    stepper_feedforward_config_t *feedforward = Stepper_Feedforward_Get_Config();
    car_tracking_config_t *tracking = Car_Attitude_Control_Get_Tracking_Config();
    car_lap_config_t *lap = Car_Lap_Task_Get_Config();
    static float *const values[ANO_PARAM_COUNT] = {
        &car_pid[0], &car_pid[1], &car_pid[2],
        &yaw_pid.Kp, &yaw_pid.Ki, &yaw_pid.Kd,
        &pitch_pid.Kp, &pitch_pid.Ki, &pitch_pid.Kd,
        &car_type, &car_deadzone, &car_line, &car_wheel_diameter, &car_reduction_ratio, NULL,
        &car_debug_control_mode, &car_debug_m1_value, &car_debug_m1_direction,
        &car_debug_m2_value, &car_debug_m2_direction, &car_m1_direction, &car_m2_direction,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL, NULL, NULL, &board_role,
        &arm_limit_min, &arm_limit_max, &arm_debug_speed, &arm_target_position,
        &arm_debug_direction, &arm_debug_accel, &arm_driver_microstep,
        &arm_debug_timeout_ms, &arm_home_mode, &car_lap_stop_delay_ms,
        &arm_balance_position, &car_lap_counts_per_meter,
        NULL, NULL, NULL, &imu_source
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
    if (id >= TRACKING_DIRECTION_INVERTED && id <= TRACKING_TARGET_RATE_SLEW)
    {
        float *const tracking_values[] = {
            &tracking->direction_inverted, &tracking->error_filter_time_s,
            &tracking->target_rate_slew_deg_s2
        };
        return tracking_values[id - TRACKING_DIRECTION_INVERTED];
    }
    if (id >= TRACKING_LINE_INITIAL_WEIGHT && id <= TRACKING_HEADING_ERROR_LIMIT)
    {
        float *const tracking_values[] = {
            &tracking->line_initial_weight, &tracking->line_weight_ramp_time_s,
            &tracking->line_release_time_s, &tracking->line_heading_rate_deg_s,
            &tracking->heading_error_limit_deg
        };
        return tracking_values[id - TRACKING_LINE_INITIAL_WEIGHT];
    }
    if (id >= LAP_HALF_TURN_ANGLE && id <= LAP_CROSS_CONFIRM_MS)
    {
        float *const lap_values[] = {
            &lap->half_turn_angle_deg, &lap->half_turn_window_deg,
            &lap->half_turn_confirm_ms, &lap->cross_confirm_ms
        };
        return lap_values[id - LAP_HALF_TURN_ANGLE];
    }
    if (id >= TRACKING_INNER_POSITION_WEIGHT && id <= TRACKING_IMU_FUSION_WEIGHT)
    {
        float *const fusion_values[] = {
            &tracking->line_inner_position_weight, &tracking->line_outer_position_weight,
            &tracking->line_time_curve_exponent, &tracking->optical_fusion_weight,
            &tracking->imu_fusion_weight
        };
        return fusion_values[id - TRACKING_INNER_POSITION_WEIGHT];
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

static void invalidate_arm_reference(void)
{
    menu_stepper_tracking_enable = 0;
    arm_zero_valid = 0U;
    arm_home_complete = 0U;
    arm_home_seen_active = 0U;
    arm_home_idle_samples = 0U;
    if (zdt_stepper_motors[0] != NULL)
    {
        ZDT_Stepper_stop(zdt_stepper_motors[0], ZDT_STEPPER_SYNC_DISABLE);
        ZDT_Stepper_Set_Software_Limits(zdt_stepper_motors[0], arm_limit_min, arm_limit_max, false);
    }
}

static bool write_arm_parameter(uint16_t parameter_id, float value)
{
    float *target = parameter(parameter_id);

    if (target == NULL || !isfinite(value) || arm_motion_active || arm_home_active ||
        (arm_startup_state >= ARM_STARTUP_WAIT_HOME_ACK && arm_startup_state <= ARM_STARTUP_MOVE_BALANCE)) return false;
    switch (parameter_id)
    {
    case ARM_LIMIT_MIN:
        if (value < -3600.0f || value > 3600.0f ||
            ((arm_point_configured & 0x02U) != 0U && value >= arm_balance_position) ||
            ((arm_point_configured & 0x04U) != 0U && value >= arm_limit_max)) return false;
        arm_limit_configured |= 0x01U;
        arm_point_configured |= 0x01U;
        invalidate_arm_reference();
        break;
    case ARM_LIMIT_MAX:
        if (value < -3600.0f || value > 3600.0f ||
            ((arm_point_configured & 0x01U) != 0U && value <= arm_limit_min) ||
            ((arm_point_configured & 0x02U) != 0U && value <= arm_balance_position)) return false;
        arm_limit_configured |= 0x02U;
        arm_point_configured |= 0x04U;
        invalidate_arm_reference();
        break;
    case ARM_BALANCE_POSITION:
        if (value < -3600.0f || value > 3600.0f ||
            ((arm_point_configured & 0x01U) != 0U && value <= arm_limit_min) ||
            ((arm_point_configured & 0x04U) != 0U && value >= arm_limit_max)) return false;
        arm_point_configured |= 0x02U;
        invalidate_arm_reference();
        break;
    case ARM_DEBUG_SPEED:
        if (value <= 0.0f || value > 1000.0f) return false;
        break;
    case ARM_TARGET_POSITION:
        if (!arm_points_valid() || value < arm_limit_min || value > arm_limit_max) return false;
        break;
    case ARM_DEBUG_DIRECTION:
        if (value != 0.0f && value != 1.0f) return false;
        break;
    case ARM_DEBUG_ACCEL:
        if (value < 0.0f || value > 255.0f || value != floorf(value)) return false;
        break;
    case ARM_DRIVER_MICROSTEP:
        if (value < 1.0f || value > 255.0f || value != floorf(value)) return false;
        break;
    case ARM_DEBUG_TIMEOUT_MS:
        if (value < 100.0f || value > 5000.0f || value != floorf(value)) return false;
        break;
    case ARM_HOME_MODE:
        if (value < 0.0f || value > 3.0f || value != floorf(value)) return false;
        break;
    default:
        return false;
    }
    *target = value;
    return true;
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
    case TRACKING_DIRECTION_INVERTED: next.direction_inverted = value; break;
    case TRACKING_ERROR_FILTER_TIME: next.error_filter_time_s = value; break;
    case TRACKING_TARGET_RATE_SLEW: next.target_rate_slew_deg_s2 = value; break;
    case TRACKING_LINE_INITIAL_WEIGHT: next.line_initial_weight = value; break;
    case TRACKING_LINE_WEIGHT_RAMP_TIME: next.line_weight_ramp_time_s = value; break;
    case TRACKING_LINE_RELEASE_TIME: next.line_release_time_s = value; break;
    case TRACKING_LINE_HEADING_RATE: next.line_heading_rate_deg_s = value; break;
    case TRACKING_HEADING_ERROR_LIMIT: next.heading_error_limit_deg = value; break;
    case TRACKING_INNER_POSITION_WEIGHT: next.line_inner_position_weight = value; break;
    case TRACKING_OUTER_POSITION_WEIGHT: next.line_outer_position_weight = value; break;
    case TRACKING_TIME_CURVE_EXPONENT: next.line_time_curve_exponent = value; break;
    case TRACKING_OPTICAL_FUSION_WEIGHT: next.optical_fusion_weight = value; break;
    case TRACKING_IMU_FUSION_WEIGHT: next.imu_fusion_weight = value; break;
    default: return false;
    }
    return Car_Attitude_Control_Apply_Tracking_Config(&next);
}

static bool write_lap_parameter(uint16_t parameter_id, float value)
{
    car_lap_config_t next = *Car_Lap_Task_Get_Config();

    if (!isfinite(value)) return false;
    switch (parameter_id)
    {
    case LAP_HALF_TURN_ANGLE: next.half_turn_angle_deg = value; break;
    case LAP_HALF_TURN_WINDOW: next.half_turn_window_deg = value; break;
    case LAP_HALF_TURN_CONFIRM_MS: next.half_turn_confirm_ms = value; break;
    case LAP_CROSS_CONFIRM_MS: next.cross_confirm_ms = value; break;
    default: return false;
    }
    return Car_Lap_Task_Apply_Config(&next);
}

static bool write_car_parameter(uint16_t parameter_id, float value)
{
    float *target = parameter(parameter_id);
    if (target == NULL || !isfinite(value))
    {
        return false;
    }

    bool accepted = false;
    if (parameter_id <= CAR_KD)
    {
        if (value < -1000.0f || value > 1000.0f)
        {
            return false;
        }

        float previous = *target;
        *target = value;
        accepted = Car_Motor_Set_PID(car_pid[0], car_pid[1], car_pid[2]);
        if (!accepted)
        {
            *target = previous;
        }
    }
    else if (parameter_id == CAR_TYPE && value >= (float)CAR_MOTOR_TYPE_520 &&
             value <= (float)CAR_MOTOR_TYPE_TT && value == floorf(value))
    {
        accepted = Car_Motor_Set_Type((car_motor_type_t)(uint8_t)value);
        if (accepted) *target = value;
    }
    else if (parameter_id == CAR_DEADZONE && value >= 0.0f && value <= 3600.0f && value == floorf(value))
    {
        accepted = Car_Motor_Set_Deadzone((uint16_t)value);
        if (accepted) *target = value;
    }
    else if (parameter_id == CAR_LINE && value >= 0.0f && value <= 65535.0f && value == floorf(value))
    {
        accepted = Car_Motor_Set_Line((uint16_t)value);
        if (accepted) *target = value;
    }
    else if (parameter_id == CAR_WHEEL_DIAMETER && value > 0.0f)
    {
        accepted = Car_Motor_Set_Wheel_Diameter(value);
        if (accepted) *target = value;
    }
    else if (parameter_id == CAR_REDUCTION_RATIO && value > 0.0f && value <= 65535.0f && value == floorf(value))
    {
        accepted = Car_Motor_Set_Reduction_Ratio((uint16_t)value);
        if (accepted) *target = value;
    }
    else if ((parameter_id == CAR_M1_DIRECTION || parameter_id == CAR_M2_DIRECTION) &&
             (value == 0.0f || value == 1.0f))
    {
        accepted = Car_Motor_Set_Direction((uint8_t)(parameter_id - CAR_M1_DIRECTION), value != 0.0f);
        if (accepted) *target = value;
    }

    return accepted;
}

static void send_check(uint8_t destination, uint8_t id, uint8_t sum, uint8_t add)
{
    uint8_t d[3] = {id, sum, add};
    send_frame_to(destination, ANO_ID_CHECK, d, sizeof(d));
}
static void send_param(uint16_t id)
{
    uint8_t d[6]; uint16_t i = 0U; float *v = parameter(id);
    if (id == HOST_BAUD) { put_u16(d, &i, id); put_float(d, &i, (float)host_baud_rate); send_frame(ANO_ID_PARAM_VALUE, d, i); return; }
    if (v == NULL) return;
    put_u16(d, &i, id); put_float(d, &i, *v); send_frame(ANO_ID_PARAM_VALUE, d, i);
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
static void send_param_info(uint16_t id)
{
    static const char *const names[ANO_PARAM_COUNT] = {
        "四驱Kp", "四驱Ki", "四驱Kd",
        "YAW轴Kp", "YAW轴Ki", "YAW轴Kd",
        "PITCH轴Kp", "PITCH轴Ki", "PITCH轴Kd",
        "四驱型号", "四驱死区", "四驱相位线", "车轮直径", "四驱减速比", "上位机波特率",
        "M1M2控制模式", "M1调试数值", "M1调试方向", "M2调试数值", "M2调试方向", "M1方向", "M2方向",
        "双环控制模式", "基础轮速", "航向Kp", "航向Ki", "航向Kd", "角速度Kp", "角速度Ki", "角速度Kd",
        "LADRC观测带宽", "LADRC控制带宽", "LADRC对象增益", "最大角速度", "最大差速", "IMU X轴倒置", "IMU Y轴倒置", "IMU Z轴倒置",
        "角速度阶跃幅度", "航向角阶跃幅度", "阶跃保持时间", "航向角反馈倒置",
        "滚球前馈使能", "前后X轴增益", "左右Y轴增益", "前馈滤波常数", "前馈加速度死区", "前馈转速限幅", "控制板角色",
        "臂杆最左点", "臂杆最右点", "臂杆调试速度", "臂杆目标位置", "臂杆调试方向",
        "臂杆加速度", "驱动细分", "调试超时时间", "驱动回零模式", "一圈停车补偿", "臂杆平衡点", "里程每米计数",
        "循迹方向倒置", "偏差滤波时间", "目标转向斜率", "小车惯导来源",
        "光电初始权重", "光电权重增长时间", "光电释放时间", "光电航向推进速度", "航向误差限幅",
        "半圈目标角度", "半圈角度窗口", "半圈确认时间", "横线确认时间",
        "内侧光电位置权重", "外侧光电位置权重", "时间权重曲线指数", "光电融合权重", "惯导融合权重"
    };
    static const char *const infos[ANO_PARAM_COUNT] = {
        "四驱速度环比例系数", "四驱速度环积分系数", "四驱速度环微分系数",
        "航向轴PID比例系数", "航向轴PID积分系数", "航向轴PID微分系数",
        "俯仰轴PID比例系数", "俯仰轴PID积分系数", "俯仰轴PID微分系数",
        "四驱电机型号", "四驱控制死区", "四驱编码器相位线", "车轮直径，单位毫米", "四驱减速比", "匿名上位机波特率",
        "0为速度模式，1为PWM模式", "速度范围0至1000，PWM范围0至3600", "0为正转，1为反转",
        "速度范围0至1000，PWM范围0至3600", "0为正转，1为反转", "0为正向，1为反向，作用于速度和PWM", "0为正向，1为反向，作用于速度和PWM",
        "0为PID，1为LADRC", "M1和M2共同前进速度", "航向外环比例", "航向外环积分", "航向外环微分",
        "角速度内环比例", "角速度内环积分", "角速度内环微分", "线性扩张状态观测器带宽", "角速度控制器带宽",
        "被控对象输入增益，必须大于零", "航向外环目标角速度限幅", "左右轮速度差限幅", "0正常，1倒置", "0正常，1倒置", "0正常，1倒置",
        "角速度阶跃幅度，单位度每秒", "相对Z轴航向阶跃幅度，单位度", "正阶跃和回零各自保持毫秒数", "0原方向，1倒置，仅作用于航向角阶跃",
        "0关闭，1启用；启用后先静止校准", "JY61P X轴到Pitch转速增益，可为负", "JY61P Y轴到Yaw转速增益，可为负",
        "一阶低通时间常数，单位秒", "滤波加速度死区，单位g", "单轴前馈最大转速，单位RPM", "0小车控制板，1臂杆电机板；保存后重启生效",
        "臂杆最向左倾斜绝对角度，修改后须重新回零", "臂杆最向右倾斜绝对角度，修改后须重新回零",
        "速度和定位命令使用的转速，单位RPM", "绝对定位目标角度，必须位于上下限内", "0顺时针，1逆时针",
        "驱动器加速度档位，0至255", "驱动器细分值，1至255，应用配置后生效", "运动看门狗超时，100至5000毫秒", "驱动器回零模式，0至4",
        "检测到A点十字后继续行驶时间，单位毫秒，范围0至2000", "上电回零后自动到达的绝对角度，必须严格位于左右点之间",
        "单轮前进一米对应的累计编码器计数，用于里程标定",
        "0为当前修正方向，1为反向", "归一化循迹偏差一阶低通时间常数，单位秒，0为关闭",
        "目标转向角速度最大变化率，单位度每二次方秒", "0使用JY61P，1使用WheelTec；保存后保持",
        "光电刚触发时的修正权重，范围0至1", "同方向持续触发达到满权重的时间，单位秒",
        "光电居中或换向时航向推进速度衰减时间，单位秒", "满偏差满权重时目标航向推进速度，单位度每秒",
        "目标航向与实际解缠航向最大偏差，单位度", "武装终点检测所需累计Yaw角度，单位度",
        "半圈目标允许窗口，单位度", "Yaw位于半圈窗口内的确认时间，单位毫秒",
        "终点横线连续有效确认时间，单位毫秒", "内侧两路光电位置权重，不得大于外侧权重",
        "最外侧两路光电位置权重，用作位置归一化基准", "时间幂曲线指数，大于1时短时修正更柔和",
        "光电航向参考修正总权重，0关闭光电修正", "惯导航向外环输出权重，0关闭航向保持"
    };
    uint8_t d[ANO_DATA_MAX] = {0}; uint16_t i = 0U;
    if (id >= ANO_PARAM_COUNT) return;
    put_u16(d, &i, id); d[i++] = 8U; strncpy((char *)&d[i], names[id], 20U); i += 20U;
    strncpy((char *)&d[i], infos[id], sizeof(d) - i - 1U); i += (uint16_t)strlen((const char *)&d[i]);
    send_frame(ANO_ID_PARAM_INFO, d, i);
}

static void send_command_count(void)
{
    uint8_t d[3] = {0x00U, (uint8_t)(sizeof(commands) / sizeof(commands[0])), 0U};
    send_frame(ANO_ID_COMMAND_QUERY, d, sizeof(d));
}

static void send_command_info(uint16_t command_id)
{
    uint8_t d[ANO_DATA_MAX] = {0}; uint16_t i = 0U;
    if (command_id >= (uint16_t)(sizeof(commands) / sizeof(commands[0]))) return;
    put_u16(d, &i, command_id);
    memcpy(&d[i], commands[command_id].cid, sizeof(commands[command_id].cid)); i += sizeof(commands[command_id].cid);
    i += 8U;
    strncpy((char *)&d[i], commands[command_id].name, 20U); i += 20U;
    strncpy((char *)&d[i], commands[command_id].info, sizeof(d) - i - 1U); i += (uint16_t)strlen((const char *)&d[i]);
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

static bool arm_limits_valid(void)
{
    return arm_points_valid();
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
    if (exit_debug) arm_debug_active = 0U;
    return true;
}

static bool enter_arm_debug(void)
{
    if (zdt_stepper_motors[0] == NULL) return false;
    menu_stepper_tracking_enable = 0;
    (void)stop_arm_debug(false);
    arm_debug_active = 1U;
    ZDT_Stepper_Read_current_position(zdt_stepper_motors[0]);
    arm_last_position_request_ms = HAL_GetTick();
    return true;
}

static bool start_arm_home(void)
{
    if (!arm_debug_active || arm_motion_active || arm_home_active || !arm_limits_valid() || zdt_stepper_motors[0] == NULL) return false;
    arm_zero_valid = 0U;
    arm_zero_confirm_pending = 0U;
    arm_home_active = 1U;
    arm_home_complete = 0U;
    arm_home_seen_active = 0U;
    arm_home_idle_samples = 0U;
    ZDT_Stepper_trigger_zero_return(zdt_stepper_motors[0], (uint8_t)arm_home_mode, ZDT_STEPPER_SYNC_DISABLE);
    arm_zero_confirm_issued_ms = HAL_GetTick();
    return true;
}

static bool confirm_arm_zero(void)
{
    ZDTStepperMotor *motor = zdt_stepper_motors[0];

    if (!arm_debug_active || !arm_home_complete || !arm_limits_valid() || motor == NULL)
    {
        return false;
    }
    ZDT_Stepper_clear_current_position(motor);
    arm_zero_confirm_pending = 1U;
    arm_zero_confirm_issued_ms = HAL_GetTick();
    return true;
}

static void fail_arm_startup(void)
{
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
    ZDT_Stepper_Set_Software_Limits(zdt_stepper_motors[0], arm_limit_min, arm_limit_max, false);
    arm_debug_active = 0U;
    arm_startup_state = ARM_STARTUP_FAILED;
}

static void start_arm_startup(void)
{
    ZDTStepperMotor *motor = zdt_stepper_motors[0];
    uint32_t now = HAL_GetTick();

    if (App_Role_Get() != APP_ROLE_ARM || motor == NULL || !arm_points_valid()) return;
    menu_stepper_tracking_enable = 0;
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
    uint8_t direction = arm_balance_position >= 0.0f ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW;
    float travel_seconds = fabsf(arm_balance_position) / (arm_debug_speed * 6.0f);
    uint32_t timeout_ms = (uint32_t)(travel_seconds * 1000.0f) + 3000U;
    uint32_t now = HAL_GetTick();

    if (timeout_ms < ARM_STARTUP_MOVE_TIMEOUT_MS) timeout_ms = ARM_STARTUP_MOVE_TIMEOUT_MS;
    if (timeout_ms > 120000U) timeout_ms = 120000U;

    arm_motion_positive = arm_balance_position >= motor->data.current_position ? 1U : 0U;
    arm_startup_move_issued_ms = now;
    ZDT_Stepper_Set_position_accel(motor, direction, arm_debug_speed, (uint8_t)arm_debug_accel,
                                   fabsf(arm_balance_position), ZDT_STEPPER_ABS_POS_MODE,
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
            ZDT_Stepper_Set_Software_Limits(motor, arm_limit_min, arm_limit_max, true);
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
        if (now - motor->data.current_position_ms > ARM_DEBUG_POSITION_STALE_MS ||
            now - motor->data.current_speed_ms > ARM_DEBUG_POSITION_STALE_MS)
        {
            fail_arm_startup();
            return;
        }
    }
    if (fabsf(motor->data.current_position - arm_balance_position) <= ARM_BALANCE_TOLERANCE_DEG &&
        fabsf(motor->data.current_speed) <= 1.0f)
    {
        arm_motion_active = 0U;
        arm_startup_state = ARM_STARTUP_DONE;
    }
}

static bool start_arm_speed(void)
{
    ZDTStepperMotor *motor = zdt_stepper_motors[0];
    uint32_t now = HAL_GetTick();
    bool toward_positive = arm_debug_direction != 0.0f;
    float stopping_margin = arm_debug_speed * 0.006f * (float)(ARM_DEBUG_POSITION_PERIOD_MS * 2U);

    if (!arm_debug_active || !arm_zero_valid || !arm_limits_valid() || motor == NULL ||
        motor->data.current_position_ms == 0U ||
        now - motor->data.current_position_ms > ARM_DEBUG_POSITION_STALE_MS ||
        (toward_positive ? motor->data.current_position + stopping_margin >= arm_limit_max :
                           motor->data.current_position - stopping_margin <= arm_limit_min))
    {
        return false;
    }
    ZDT_Stepper_Set_Speed(motor, toward_positive ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW,
                          arm_debug_speed, (uint8_t)arm_debug_accel, ZDT_STEPPER_SYNC_DISABLE);
    arm_motion_active = 1U;
    arm_motion_positive = toward_positive ? 1U : 0U;
    arm_speed_motion = 1U;
    arm_motion_deadline_ms = now + (uint32_t)arm_debug_timeout_ms;
    return true;
}

static bool start_arm_absolute(void)
{
    ZDTStepperMotor *motor = zdt_stepper_motors[0];
    uint32_t now = HAL_GetTick();
    uint8_t direction;

    if (!arm_debug_active || !arm_zero_valid || !arm_limits_valid() || motor == NULL ||
        motor->data.current_position_ms == 0U || now - motor->data.current_position_ms > ARM_DEBUG_POSITION_STALE_MS ||
        arm_target_position < arm_limit_min || arm_target_position > arm_limit_max)
    {
        return false;
    }
    arm_motion_positive = arm_target_position >= motor->data.current_position ? 1U : 0U;
    direction = arm_target_position >= 0.0f ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW;
    ZDT_Stepper_Set_position_accel(motor, direction, arm_debug_speed, (uint8_t)arm_debug_accel,
                                   fabsf(arm_target_position), ZDT_STEPPER_ABS_POS_MODE,
                                   ZDT_STEPPER_SYNC_DISABLE);
    arm_motion_active = 1U;
    arm_speed_motion = 0U;
    arm_motion_deadline_ms = now + (uint32_t)arm_debug_timeout_ms;
    return true;
}

static bool apply_arm_driver_config(void)
{
    if (!arm_debug_active || arm_motion_active || zdt_stepper_motors[0] == NULL) return false;
    ZDT_Stepper_set_microstepping(zdt_stepper_motors[0], ZDT_STEPPER_STORE,
                                  (uint8_t)arm_driver_microstep);
    invalidate_arm_reference();
    return true;
}

static bool handle_arm_command(uint8_t command_id)
{
    switch (command_id)
    {
    case ANO_COMMAND_STEPPER_FF_CALIBRATE: Stepper_Feedforward_Reset_Bias(); return true;
    case ANO_COMMAND_ARM_DEBUG_ENTER: return enter_arm_debug();
    case ANO_COMMAND_ARM_DEBUG_EXIT: return stop_arm_debug(true);
    case ANO_COMMAND_ARM_HOME: return start_arm_home();
    case ANO_COMMAND_ARM_ZERO_CONFIRM: return confirm_arm_zero();
    case ANO_COMMAND_ARM_RUN_SPEED: return start_arm_speed();
    case ANO_COMMAND_ARM_MOVE_ABSOLUTE: return start_arm_absolute();
    case ANO_COMMAND_ARM_STOP: return stop_arm_debug(false);
    case ANO_COMMAND_ARM_ENABLE:
        if (!arm_debug_active || zdt_stepper_motors[0] == NULL) return false;
        ZDT_Stepper_Enable(zdt_stepper_motors[0], ZDT_STEPPER_ENABLE, ZDT_STEPPER_SYNC_DISABLE);
        return true;
    case ANO_COMMAND_ARM_DISABLE:
        if (!arm_debug_active || zdt_stepper_motors[0] == NULL) return false;
        (void)stop_arm_debug(false);
        ZDT_Stepper_Enable(zdt_stepper_motors[0], ZDT_STEPPER_DISABLE, ZDT_STEPPER_SYNC_DISABLE);
        return true;
    case ANO_COMMAND_ARM_APPLY_DRIVER: return apply_arm_driver_config();
    default: return false;
    }
}

static void process_arm_debug(void)
{
    ZDTStepperMotor *motor = zdt_stepper_motors[0];
    uint32_t now = HAL_GetTick();

    if (motor == NULL) return;
    if (now - arm_last_position_request_ms >= ARM_DEBUG_POSITION_PERIOD_MS)
    {
        arm_last_position_request_ms = now;
        ZDT_Stepper_Read_current_position(motor);
    }
    if (now - arm_last_speed_request_ms >= ARM_DEBUG_POSITION_PERIOD_MS)
    {
        arm_last_speed_request_ms = now;
        ZDT_Stepper_Read_current_speed(motor);
    }
    if (arm_zero_valid && motor->data.current_position_ms != 0U &&
        ((motor->data.current_position <= arm_limit_min && motor->data.current_speed < -0.5f) ||
         (motor->data.current_position >= arm_limit_max && motor->data.current_speed > 0.5f)))
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
        ZDT_Stepper_Set_Software_Limits(motor, arm_limit_min, arm_limit_max, true);
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
    float stopping_margin = arm_speed_motion ? arm_debug_speed * 0.006f * (float)(ARM_DEBUG_POSITION_PERIOD_MS * 2U) : 0.0f;
    bool limit_reached = arm_motion_positive != 0U ?
                         motor->data.current_position + stopping_margin >= arm_limit_max :
                         motor->data.current_position - stopping_margin <= arm_limit_min;
    if ((int32_t)(now - arm_motion_deadline_ms) >= 0 ||
        motor->data.current_position_ms == 0U || now - motor->data.current_position_ms > ARM_DEBUG_POSITION_STALE_MS ||
        limit_reached)
    {
        (void)stop_arm_debug(true);
    }
}

static void send_arm_debug_diagnostic(void)
{
    ZDTStepperMotor *motor = zdt_stepper_motors[0];
    uint8_t d[8];
    uint16_t i = 0U;

    if (motor == NULL || !tx_has_capacity(1U)) return;
    put_float(d, &i, motor->data.current_position);
    put_float(d, &i, motor->data.current_speed);
    send_frame(ANO_ID_ARM_DEBUG_DIAGNOSTIC, d, i);
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
    const uint8_t gray = Gray_Sensor_Get_Stable() & 0x0FU;
    uint8_t d[72];
    uint16_t i = 0U;

    if (!tx_has_capacity(1U)) return;

    for (uint8_t bit = 0U; bit < 4U; bit++)
    {
        put_float(d, &i, (float)((gray >> bit) & 0x01U));
    }
    put_float(d, &i, control->line_offset);
    put_float(d, &i, control->target_rate);
    Car_Lap_Task_Get_Status(&lap);
    put_float(d, &i, (float)lap.elapsed_ms * 0.001f);
    put_float(d, &i, lap.distance_m);
    put_float(d, &i, lap.average_speed_mps);
    put_float(d, &i, control->tracking_angle);
    put_float(d, &i, control->target_heading);
    put_float(d, &i, control->line_weight);
    put_float(d, &i, control->line_heading_rate);
    put_float(d, &i, (float)lap.half_turn_armed);
    put_float(d, &i, (float)lap.state);
    put_float(d, &i, control->line_position_weight);
    put_float(d, &i, control->line_time_weight);
    put_float(d, &i, control->imu_fusion_weight);
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

    if (!Car_IMU_Get_Sample(&imu) || imu.sequence == last_imu_sequence ||
        !tx_has_capacity(4U))
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

static void send_stepper_feedforward_diagnostic(void)
{
    const stepper_feedforward_debug_t *debug = Stepper_Feedforward_Get_Debug();
    uint8_t d[40];
    uint16_t i = 0U;

    if (!tx_has_capacity(1U)) return;
    put_float(d, &i, debug->raw_x_g);
    put_float(d, &i, debug->raw_y_g);
    put_float(d, &i, debug->filtered_x_g);
    put_float(d, &i, debug->filtered_y_g);
    put_float(d, &i, debug->pitch_output_rpm);
    put_float(d, &i, debug->yaw_output_rpm);
    put_float(d, &i, debug->pitch_total_rpm);
    put_float(d, &i, debug->yaw_total_rpm);
    put_float(d, &i, (float)debug->imu_valid);
    put_float(d, &i, (float)debug->calibrated);
    send_frame(ANO_ID_STEPPER_FF_DIAGNOSTIC, d, i);
}

static bool command(const uint8_t *d, uint16_t n)
{
    if (App_Role_Get() == APP_ROLE_ARM)
    {
        if (n == 3U && d[0] == 0x02U && d[1] == 0x12U && d[2] == 0xFFU) return stop_arm_debug(false);
        if (n == 2U && d[0] == 0x12U && (d[1] == 0xFFU || d[1] == 0U)) return stop_arm_debug(false);
        if (n == 3U && d[0] == 0x03U && d[2] == 0x00U) return handle_arm_command(d[1]);
        if (n == 1U) return handle_arm_command(d[0]);
        return false;
    }
    if (App_Role_Get() == APP_ROLE_CAR && n != 0U &&
        (d[0] == 0x10U || d[0] == 0x11U || d[0] == 0x12U || (n == 3U && d[0] == 0x02U))) return false;
    if (n == 3U && d[0] == 0x03U && d[1] == 0x03U && d[2] == 0x00U) { (void)Car_Lap_Task_Stop(); return car_debug_active ? stop_car_debug() : Car_Motor_Set_Speed(0, 0, 0, 0); }
    if (n == 3U && d[0] == 0x02U && d[1] == 0x12U && d[2] == 0xFFU) { ZDT_Stepper_stop(zdt_stepper_motors[0], 0U); return true; }
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_READ_FLASH && d[2] == 0x00U) return Car_Motor_Read_Flash();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_FACTORY_RESET && d[2] == 0x00U) return Car_Motor_Factory_Reset();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_APPLY_CONFIG && d[2] == 0x00U) return apply_car_config();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_DEBUG_ENTER && d[2] == 0x00U) return enter_car_debug();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_DEBUG_START && d[2] == 0x00U) return start_car_debug();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_DEBUG_STOP && d[2] == 0x00U) return stop_car_debug();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_RESTORE_DEFAULTS && d[2] == 0x00U) return restore_defaults();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_ATTITUDE_ENABLE && d[2] == 0x00U) { car_debug_active = 0U; return Car_Attitude_Control_Set_Enabled(true); }
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_ATTITUDE_DISABLE && d[2] == 0x00U) return Car_Lap_Task_Stop();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_RATE_STEP_TEST && d[2] == 0x00U) return start_attitude_step(CAR_ATTITUDE_STEP_RATE);
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_ANGLE_STEP_TEST && d[2] == 0x00U) return start_attitude_step(CAR_ATTITUDE_STEP_ANGLE);
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_STEPPER_FF_CALIBRATE && d[2] == 0x00U) { Stepper_Feedforward_Reset_Bias(); return true; }
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_LAP_START && d[2] == 0x00U) { if (car_debug_active && !stop_car_debug()) return false; return Car_Lap_Task_Start(); }
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_LAP_STOP && d[2] == 0x00U) return Car_Lap_Task_Stop();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_LAP_CLEAR && d[2] == 0x00U) return Car_Lap_Task_Clear();
    if (n == 9U && d[0] == 1U && !car_debug_active && !Car_Attitude_Control_Is_Enabled()) return Car_Motor_Set_Speed(get_i16(&d[1]), get_i16(&d[3]), get_i16(&d[5]), get_i16(&d[7]));
    if (n == 9U && d[0] == 2U && !car_debug_active && !Car_Attitude_Control_Is_Enabled()) return Car_Motor_Set_PWM(get_i16(&d[1]), get_i16(&d[3]), get_i16(&d[5]), get_i16(&d[7]));
    if (n == 1U && d[0] == 3U) { (void)Car_Lap_Task_Stop(); return car_debug_active ? stop_car_debug() : Car_Motor_Set_Speed(0, 0, 0, 0); }
    if (n == 7U && d[0] == 0x10U && d[1] == 0U) { float s = get_float(&d[3]); if (s >= 0.0f && s <= 1000.0f) { ZDT_Stepper_Set_Speed(zdt_stepper_motors[0], d[2] ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW, s, 0U, 0U); return true; } }
    if (n == 12U && d[0] == 0x11U && d[1] == 0U) { float s = get_float(&d[3]), p = get_float(&d[7]); if (s >= 0.0f && s <= 1000.0f && p >= 0.0f && p <= 3600.0f) { ZDT_Stepper_Set_position(zdt_stepper_motors[0], d[2] ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW, s, p, d[11] ? ZDT_STEPPER_ABS_POS_MODE : ZDT_STEPPER_REL_POS_MODE, 0U); return true; } }
    if (n == 2U && d[0] == 0x12U && (d[1] == 0xFFU || d[1] == 0U)) { ZDT_Stepper_stop(zdt_stepper_motors[0], 0U); return true; }
    if (n == 1U && d[0] == ANO_COMMAND_CAR_READ_FLASH) return Car_Motor_Read_Flash();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_FACTORY_RESET) return Car_Motor_Factory_Reset();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_APPLY_CONFIG) return apply_car_config();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_DEBUG_ENTER) return enter_car_debug();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_DEBUG_START) return start_car_debug();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_DEBUG_STOP) return stop_car_debug();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_RESTORE_DEFAULTS) return restore_defaults();
    if (n == 1U && d[0] == ANO_COMMAND_ATTITUDE_ENABLE) { car_debug_active = 0U; return Car_Attitude_Control_Set_Enabled(true); }
    if (n == 1U && d[0] == ANO_COMMAND_ATTITUDE_DISABLE) return Car_Lap_Task_Stop();
    if (n == 1U && d[0] == ANO_COMMAND_RATE_STEP_TEST) return start_attitude_step(CAR_ATTITUDE_STEP_RATE);
    if (n == 1U && d[0] == ANO_COMMAND_ANGLE_STEP_TEST) return start_attitude_step(CAR_ATTITUDE_STEP_ANGLE);
    if (n == 1U && d[0] == ANO_COMMAND_STEPPER_FF_CALIBRATE) { Stepper_Feedforward_Reset_Bias(); return true; }
    if (n == 1U && d[0] == ANO_COMMAND_CAR_LAP_START) { if (car_debug_active && !stop_car_debug()) return false; return Car_Lap_Task_Start(); }
    if (n == 1U && d[0] == ANO_COMMAND_CAR_LAP_STOP) return Car_Lap_Task_Stop();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_LAP_CLEAR) return Car_Lap_Task_Clear();
    return false;
}

static void handle(uint8_t source, uint8_t id, const uint8_t *d, uint16_t n, uint8_t sum, uint8_t add)
{
    if (id == ANO_ID_PARAM_COMMAND && n == 3U) {
        uint16_t v = get_u16(&d[1]);
        if (d[0] == 0U) send_device_info();
        else if (d[0] == 1U) { uint8_t count[3] = {1U, ANO_PARAM_COUNT, 0U}; send_frame(ANO_ID_PARAM_COMMAND, count, 3U); }
        else if (d[0] == 2U) send_param(v);
        else if (d[0] == 3U) send_param_info(v);
        else if (d[0] == 0x10U && v == 0x00AAU && restore_defaults()) send_check(source, id, sum, add);
        else if (d[0] == 0x10U && v == 0x00ABU && storage_ready && save_storage(host_baud_rate)) send_check(source, id, sum, add);
    }
    else if (id == ANO_ID_PARAM_VALUE && n == 6U) {
        bool accepted = false;
        uint16_t parameter_id = get_u16(d); float value = get_float(&d[2]); float *v = parameter(parameter_id);
        if (App_Role_Get() == APP_ROLE_ARM &&
            (parameter_id <= CAR_KD || (parameter_id >= CAR_TYPE && parameter_id <= STEP_ANGLE_FEEDBACK_INVERTED) ||
             parameter_id == CAR_LAP_STOP_DELAY_MS || parameter_id == CAR_LAP_COUNTS_PER_METER ||
             (parameter_id >= TRACKING_DIRECTION_INVERTED && parameter_id < ANO_PARAM_COUNT))) return;
        if (App_Role_Get() == APP_ROLE_CAR &&
            ((parameter_id >= YAW_KP && parameter_id <= PITCH_KD) ||
             (parameter_id >= ARM_LIMIT_MIN && parameter_id <= ARM_HOME_MODE) ||
             parameter_id == ARM_BALANCE_POSITION)) return;
        if (parameter_id == HOST_BAUD && isfinite(value) && value == (float)(uint32_t)value &&
            baud_rate_is_valid((uint32_t)value) && storage_ready) {
            uint32_t requested_baud_rate = (uint32_t)value;

            if (requested_baud_rate == host_baud_rate) {
                accepted = true;
            } else {
                /* Queue the ACK before the blocking flash transaction exceeds the host timeout. */
                send_check(source, id, sum, add);
                if (save_storage(requested_baud_rate)) {
                    pending_baud_rate = requested_baud_rate;
                    baud_switch_at_ms = HAL_GetTick() + ANO_BAUD_SWITCH_DELAY_MS;
                }
            }
        } else if (parameter_id <= CAR_KD || (parameter_id >= CAR_TYPE && parameter_id < HOST_BAUD)) {
            accepted = write_car_parameter(parameter_id, value);
        } else if (parameter_id == CAR_DEBUG_CONTROL_MODE && isfinite(value) && (value == 0.0f || value == 1.0f)) {
            car_debug_control_mode = value;
            accepted = true;
        } else if ((parameter_id == CAR_DEBUG_M1_VALUE || parameter_id == CAR_DEBUG_M2_VALUE) && isfinite(value) &&
                   value >= 0.0f && value <= (car_debug_control_mode == 0.0f ? 1000.0f : 3600.0f)) {
            *v = value;
            accepted = true;
        } else if ((parameter_id == CAR_DEBUG_M1_DIRECTION || parameter_id == CAR_DEBUG_M2_DIRECTION) &&
                   isfinite(value) && (value == 0.0f || value == 1.0f)) {
            *v = value;
            accepted = true;
        } else if ((parameter_id == CAR_M1_DIRECTION || parameter_id == CAR_M2_DIRECTION) &&
                   isfinite(value) && (value == 0.0f || value == 1.0f)) {
            accepted = write_car_parameter(parameter_id, value);
        } else if (parameter_id >= ATTITUDE_MODE && parameter_id <= STEP_ANGLE_FEEDBACK_INVERTED) {
            accepted = write_attitude_parameter(parameter_id, value);
        } else if (parameter_id >= STEPPER_FF_ENABLE && parameter_id <= STEPPER_FF_OUTPUT_LIMIT) {
            accepted = write_feedforward_parameter(parameter_id, value);
        } else if ((parameter_id >= TRACKING_DIRECTION_INVERTED && parameter_id <= TRACKING_TARGET_RATE_SLEW) ||
                   (parameter_id >= TRACKING_LINE_INITIAL_WEIGHT && parameter_id <= TRACKING_HEADING_ERROR_LIMIT) ||
                   (parameter_id >= TRACKING_INNER_POSITION_WEIGHT && parameter_id <= TRACKING_IMU_FUSION_WEIGHT)) {
            accepted = write_tracking_parameter(parameter_id, value);
        } else if (parameter_id == IMU_SOURCE && App_Role_Get() == APP_ROLE_CAR &&
                   isfinite(value) && (value == 0.0f || value == 1.0f)) {
            (void)Car_Attitude_Control_Set_Enabled(false);
            accepted = Car_IMU_Set_Source((car_imu_source_t)(uint8_t)value);
            if (accepted) { imu_source = value; last_imu_sequence = 0U; }
        } else if (parameter_id >= ARM_LIMIT_MIN && parameter_id <= ARM_HOME_MODE) {
            accepted = write_arm_parameter(parameter_id, value);
        } else if (parameter_id == ARM_BALANCE_POSITION) {
            accepted = write_arm_parameter(parameter_id, value);
        } else if (parameter_id == CAR_LAP_STOP_DELAY_MS && isfinite(value) && value == floorf(value) &&
                   value >= 0.0f && value <= 2000.0f) {
            accepted = Car_Lap_Task_Set_Stop_Delay((uint32_t)value);
            if (accepted) car_lap_stop_delay_ms = value;
        } else if (parameter_id == CAR_LAP_COUNTS_PER_METER && isfinite(value)) {
            accepted = Car_Lap_Task_Set_Counts_Per_Meter(value);
            if (accepted) car_lap_counts_per_meter = value;
        } else if (parameter_id >= LAP_HALF_TURN_ANGLE && parameter_id <= LAP_CROSS_CONFIRM_MS) {
            accepted = write_lap_parameter(parameter_id, value);
        } else if (parameter_id == BOARD_ROLE && isfinite(value) && value == floorf(value) &&
                   value >= (float)APP_ROLE_CAR && value <= (float)APP_ROLE_ARM) {
            accepted = App_Role_Set_Persisted((app_role_t)(uint8_t)value);
            if (accepted) board_role = value;
        } else if (v != NULL && isfinite(value) && value >= -1000.0f && value <= 1000.0f) {
            *v = value;
            accepted = true;
        }
        if (accepted) send_check(source, id, sum, add);
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
    reset_local_defaults();
    last_control_telemetry_ms = HAL_GetTick();
    storage_ready = W25Qxx_Init() ? 1U : 0U;
    if (storage_ready) load_storage();
    if (App_Role_Get() == APP_ROLE_ARM) start_arm_startup();
    if (host_baud_rate != huart1.Init.BaudRate) apply_baud_rate(host_baud_rate);
    else HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}
void ANO_Host_Process(void) { while (rx_head != rx_tail) { uint8_t b = rx_queue[rx_head]; rx_head = (rx_head + 1U) % ANO_RX_QUEUE_SIZE; feed(b); } if (App_Role_Get() == APP_ROLE_ARM) process_arm_debug(); if (pending_baud_rate != 0U && (int32_t)(HAL_GetTick() - baud_switch_at_ms) >= 0 && !tx_active && tx_count == 0U) { uint32_t baud_rate = pending_baud_rate; pending_baud_rate = 0U; apply_baud_rate(baud_rate); } if (App_Role_Get() == APP_ROLE_CAR && HAL_GetTick() - last_imu_ms >= ANO_IMU_PERIOD_MS) { last_imu_ms = HAL_GetTick(); send_imu(); } if (HAL_GetTick() - last_control_telemetry_ms >= 20U) { last_control_telemetry_ms = HAL_GetTick(); if (App_Role_Get() == APP_ROLE_CAR) { send_control_telemetry(); send_tracking_diagnostic(); } else { send_stepper_feedforward_diagnostic(); send_arm_debug_diagnostic(); } } if (App_Role_Get() == APP_ROLE_CAR && HAL_GetTick() - last_telemetry_ms >= 100U) { last_telemetry_ms = HAL_GetTick(); send_telemetry(); } if (HAL_GetTick() - last_diagnostic_ms >= 500U) { last_diagnostic_ms = HAL_GetTick(); send_storage_diagnostic(); } }
void ANO_Host_UART_RxCplt_Callback(void) { uint8_t next = (rx_tail + 1U) % ANO_RX_QUEUE_SIZE; if (next != rx_head) { rx_queue[rx_tail] = rx_byte; rx_tail = next; } HAL_UART_Receive_IT(&huart1, &rx_byte, 1U); }
void ANO_Host_UART_TxCplt_Callback(void) { if (tx_active && tx_count) { tx_head = (tx_head + 1U) % ANO_TX_QUEUE_DEPTH; tx_count--; } tx_active = 0U; start_tx(); }
void ANO_Host_UART_Error_Callback(void) { tx_active = 0U; HAL_UART_Receive_IT(&huart1, &rx_byte, 1U); }
uint32_t ANO_Host_Get_BaudRate(void) { return host_baud_rate; }
