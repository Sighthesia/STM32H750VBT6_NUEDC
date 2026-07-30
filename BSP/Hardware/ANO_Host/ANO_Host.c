#include "ANO_Host.h"

#include "Car_Motor_USART.h"
#include "Car_Attitude_Control.h"
#include "JY61P.h"
#include "Stepper_PID.h"
#include "W25Qxx.h"
#include "ZDT_Stepper.h"
#include "usart.h"

#include <math.h>
#include <string.h>

#define ANO_HEAD 0xABU
#define ANO_DEVICE_ADDRESS 0x05U
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
#define ANO_ID_DIAGNOSTIC 0xF5U
#define ANO_RX_QUEUE_SIZE 128U
#define ANO_DATA_MAX 96U
#define ANO_TX_QUEUE_DEPTH 8U
#define ANO_IMU_PERIOD_MS 10U
#define ANO_DEFAULT_BAUD_RATE 460800U
#define ANO_BAUD_SWITCH_DELAY_MS 50U
#define ANO_STORAGE_ADDRESS 0U
#define ANO_STORAGE_MAGIC 0x414E4F57UL
#define ANO_STORAGE_VERSION 5U

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
    ANO_PARAM_COUNT
};

typedef struct { uint8_t state, source, destination, id, sum, checksum; uint16_t length, index; uint8_t data[ANO_DATA_MAX]; } ano_parser_t;
typedef struct { uint8_t cid[3]; const char *name; const char *info; } ano_command_desc_t;

static const ano_command_desc_t commands[] = {
    {{0x03U, 0x03U, 0x00U}, "四驱停止", "将四路电机速度设为零"},
    {{0x02U, 0x12U, 0xFFU}, "ZDT双轴停止", "立即停止两个ZDT闭环步进电机"},
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
    {{0x03U, ANO_COMMAND_ANGLE_STEP_TEST, 0x00U}, "航向角阶跃测试", "按配置幅度执行相对航向角正阶跃和回零响应"}
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
    Car_Attitude_Control_Reset_Defaults();

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

static float *parameter(uint16_t id);
static bool write_car_parameter(uint16_t parameter_id, float value);

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
    frame[index++] = ANO_HEAD; frame[index++] = ANO_DEVICE_ADDRESS; frame[index++] = destination; frame[index++] = id; frame[index++] = (uint8_t)length; frame[index++] = (uint8_t)(length >> 8U);
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
    static float *const values[ANO_PARAM_COUNT] = {
        &car_pid[0], &car_pid[1], &car_pid[2],
        &yaw_pid.Kp, &yaw_pid.Ki, &yaw_pid.Kd,
        &pitch_pid.Kp, &pitch_pid.Ki, &pitch_pid.Kd,
        &car_type, &car_deadzone, &car_line, &car_wheel_diameter, &car_reduction_ratio, NULL,
        &car_debug_control_mode, &car_debug_m1_value, &car_debug_m1_direction,
        &car_debug_m2_value, &car_debug_m2_direction, &car_m1_direction, &car_m2_direction,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL
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
    return id < ANO_PARAM_COUNT ? values[id] : NULL;
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
    static const char name[] = "STM32H750_MOTOR";
    uint8_t d[32] = {0}; uint16_t i = 0U;
    d[i++] = ANO_DEVICE_ADDRESS; put_u16(d, &i, 1U); put_u16(d, &i, 1U); put_u16(d, &i, 0U); put_u16(d, &i, 810U);
    memcpy(&d[i], name, sizeof(name) - 1U); i += sizeof(name) - 1U;
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
        "角速度阶跃幅度", "航向角阶跃幅度", "阶跃保持时间", "航向角反馈倒置"
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
        "角速度阶跃幅度，单位度每秒", "相对Z轴航向阶跃幅度，单位度", "正阶跃和回零各自保持毫秒数", "0原方向，1倒置，仅作用于航向角阶跃"
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
    (void)Car_Attitude_Control_Set_Enabled(false);
    if (!storage_ready || !W25Qxx_EraseSector(ANO_STORAGE_ADDRESS) ||
        !Car_Motor_Restore_Defaults())
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
    (void)Car_Attitude_Control_Set_Enabled(false);
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
    if (!Car_Attitude_Control_Start_Step(mode)) return false;
    car_debug_active = 0U;
    return true;
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

    if (!tx_has_capacity(2U)) return;

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

static int16_t scale_i16(int16_t value, int32_t numerator, int32_t denominator)
{
    return (int16_t)(((int32_t)value * numerator) / denominator);
}

static void send_imu(void)
{
    const jy61p_status_t *imu_status = JY61P_Get_Status();
    const jy61p_sample_t *imu;
    uint8_t d[14];
    uint16_t i;

    if (!imu_status->sample_valid || imu_status->sequence == last_imu_sequence ||
        !tx_has_capacity(4U))
    {
        return;
    }
    imu = JY61P_Get_Sample();

    i = 0U;
    put_i16(d, &i, imu->acceleration.x);
    put_i16(d, &i, imu->acceleration.y);
    put_i16(d, &i, imu->acceleration.z);
    put_i16(d, &i, imu->angular_velocity.x);
    put_i16(d, &i, imu->angular_velocity.y);
    put_i16(d, &i, imu->angular_velocity.z);
    d[i++] = 0U;
    send_frame(ANO_ID_SENSOR_BASIC, d, i);

    i = 0U;
    put_i16(d, &i, imu->magnetic_field.x);
    put_i16(d, &i, imu->magnetic_field.y);
    put_i16(d, &i, imu->magnetic_field.z);
    put_i32(d, &i, 0);
    put_i16(d, &i, imu->temperature / 10);
    d[i++] = 0U;
    d[i++] = 1U;
    send_frame(ANO_ID_SENSOR_EXTENDED, d, i);

    i = 0U;
    put_i16(d, &i, scale_i16(imu->angle.x, 18000, 32768));
    put_i16(d, &i, scale_i16(imu->angle.y, 18000, 32768));
    put_i16(d, &i, scale_i16(imu->angle.z, 18000, 32768));
    d[i++] = 1U;
    send_frame(ANO_ID_ATTITUDE_EULER, d, i);

    i = 0U;
    put_i16(d, &i, scale_i16(imu->quaternion.w, 10000, 32768));
    put_i16(d, &i, scale_i16(imu->quaternion.x, 10000, 32768));
    put_i16(d, &i, scale_i16(imu->quaternion.y, 10000, 32768));
    put_i16(d, &i, scale_i16(imu->quaternion.z, 10000, 32768));
    d[i++] = 1U;
    send_frame(ANO_ID_ATTITUDE_QUATERNION, d, i);

    last_imu_sequence = imu_status->sequence;
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
    if (n == 3U && d[0] == 0x03U && d[1] == 0x03U && d[2] == 0x00U) { (void)Car_Attitude_Control_Set_Enabled(false); return car_debug_active ? stop_car_debug() : Car_Motor_Set_Speed(0, 0, 0, 0); }
    if (n == 3U && d[0] == 0x02U && d[1] == 0x12U && d[2] == 0xFFU) { ZDT_Stepper_stop(zdt_stepper_motors[0], 0U); ZDT_Stepper_stop(zdt_stepper_motors[1], 0U); return true; }
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_READ_FLASH && d[2] == 0x00U) return Car_Motor_Read_Flash();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_FACTORY_RESET && d[2] == 0x00U) return Car_Motor_Factory_Reset();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_APPLY_CONFIG && d[2] == 0x00U) return apply_car_config();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_DEBUG_ENTER && d[2] == 0x00U) return enter_car_debug();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_DEBUG_START && d[2] == 0x00U) return start_car_debug();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_DEBUG_STOP && d[2] == 0x00U) return stop_car_debug();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_RESTORE_DEFAULTS && d[2] == 0x00U) return restore_defaults();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_ATTITUDE_ENABLE && d[2] == 0x00U) { car_debug_active = 0U; return Car_Attitude_Control_Set_Enabled(true); }
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_ATTITUDE_DISABLE && d[2] == 0x00U) return Car_Attitude_Control_Set_Enabled(false);
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_RATE_STEP_TEST && d[2] == 0x00U) return start_attitude_step(CAR_ATTITUDE_STEP_RATE);
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_ANGLE_STEP_TEST && d[2] == 0x00U) return start_attitude_step(CAR_ATTITUDE_STEP_ANGLE);
    if (n == 9U && d[0] == 1U && !car_debug_active && !Car_Attitude_Control_Is_Enabled()) return Car_Motor_Set_Speed(get_i16(&d[1]), get_i16(&d[3]), get_i16(&d[5]), get_i16(&d[7]));
    if (n == 9U && d[0] == 2U && !car_debug_active && !Car_Attitude_Control_Is_Enabled()) return Car_Motor_Set_PWM(get_i16(&d[1]), get_i16(&d[3]), get_i16(&d[5]), get_i16(&d[7]));
    if (n == 1U && d[0] == 3U) { (void)Car_Attitude_Control_Set_Enabled(false); return car_debug_active ? stop_car_debug() : Car_Motor_Set_Speed(0, 0, 0, 0); }
    if (n == 7U && d[0] == 0x10U && d[1] < 2U) { float s = get_float(&d[3]); if (s >= 0.0f && s <= 1000.0f) { ZDT_Stepper_Set_Speed(zdt_stepper_motors[d[1]], d[2] ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW, s, 0U, 0U); return true; } }
    if (n == 12U && d[0] == 0x11U && d[1] < 2U) { float s = get_float(&d[3]), p = get_float(&d[7]); if (s >= 0.0f && s <= 1000.0f && p >= 0.0f && p <= 3600.0f) { ZDT_Stepper_Set_position(zdt_stepper_motors[d[1]], d[2] ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW, s, p, d[11] ? ZDT_STEPPER_ABS_POS_MODE : ZDT_STEPPER_REL_POS_MODE, 0U); return true; } }
    if (n == 2U && d[0] == 0x12U) { if (d[1] == 0xFFU) { ZDT_Stepper_stop(zdt_stepper_motors[0], 0U); ZDT_Stepper_stop(zdt_stepper_motors[1], 0U); return true; } if (d[1] < 2U) { ZDT_Stepper_stop(zdt_stepper_motors[d[1]], 0U); return true; } }
    if (n == 1U && d[0] == ANO_COMMAND_CAR_READ_FLASH) return Car_Motor_Read_Flash();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_FACTORY_RESET) return Car_Motor_Factory_Reset();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_APPLY_CONFIG) return apply_car_config();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_DEBUG_ENTER) return enter_car_debug();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_DEBUG_START) return start_car_debug();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_DEBUG_STOP) return stop_car_debug();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_RESTORE_DEFAULTS) return restore_defaults();
    if (n == 1U && d[0] == ANO_COMMAND_ATTITUDE_ENABLE) { car_debug_active = 0U; return Car_Attitude_Control_Set_Enabled(true); }
    if (n == 1U && d[0] == ANO_COMMAND_ATTITUDE_DISABLE) return Car_Attitude_Control_Set_Enabled(false);
    if (n == 1U && d[0] == ANO_COMMAND_RATE_STEP_TEST) return start_attitude_step(CAR_ATTITUDE_STEP_RATE);
    if (n == 1U && d[0] == ANO_COMMAND_ANGLE_STEP_TEST) return start_attitude_step(CAR_ATTITUDE_STEP_ANGLE);
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
        if (parser.destination == ANO_DEVICE_ADDRESS || parser.destination == ANO_BROADCAST_ADDRESS) {
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
    if (host_baud_rate != huart1.Init.BaudRate) apply_baud_rate(host_baud_rate);
    else HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}
void ANO_Host_Process(void) { while (rx_head != rx_tail) { uint8_t b = rx_queue[rx_head]; rx_head = (rx_head + 1U) % ANO_RX_QUEUE_SIZE; feed(b); } if (pending_baud_rate != 0U && (int32_t)(HAL_GetTick() - baud_switch_at_ms) >= 0 && !tx_active && tx_count == 0U) { uint32_t baud_rate = pending_baud_rate; pending_baud_rate = 0U; apply_baud_rate(baud_rate); } if (HAL_GetTick() - last_imu_ms >= ANO_IMU_PERIOD_MS) { last_imu_ms = HAL_GetTick(); send_imu(); } if (HAL_GetTick() - last_control_telemetry_ms >= 20U) { last_control_telemetry_ms = HAL_GetTick(); send_control_telemetry(); } if (HAL_GetTick() - last_telemetry_ms >= 100U) { last_telemetry_ms = HAL_GetTick(); send_telemetry(); } if (HAL_GetTick() - last_diagnostic_ms >= 500U) { last_diagnostic_ms = HAL_GetTick(); send_storage_diagnostic(); } }
void ANO_Host_UART_RxCplt_Callback(void) { uint8_t next = (rx_tail + 1U) % ANO_RX_QUEUE_SIZE; if (next != rx_head) { rx_queue[rx_tail] = rx_byte; rx_tail = next; } HAL_UART_Receive_IT(&huart1, &rx_byte, 1U); }
void ANO_Host_UART_TxCplt_Callback(void) { if (tx_active && tx_count) { tx_head = (tx_head + 1U) % ANO_TX_QUEUE_DEPTH; tx_count--; } tx_active = 0U; start_tx(); }
void ANO_Host_UART_Error_Callback(void) { tx_active = 0U; HAL_UART_Receive_IT(&huart1, &rx_byte, 1U); }
uint32_t ANO_Host_Get_BaudRate(void) { return host_baud_rate; }
