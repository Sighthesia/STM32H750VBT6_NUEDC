#include "ANO_Host.h"

#include "Car_Motor_USART.h"
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
#define ANO_ID_COMMAND 0xC0U
#define ANO_ID_COMMAND_QUERY 0xC1U
#define ANO_ID_COMMAND_INFO 0xC2U
#define ANO_ID_PARAM_COMMAND 0xE0U
#define ANO_ID_PARAM_VALUE 0xE1U
#define ANO_ID_PARAM_INFO 0xE2U
#define ANO_ID_DEVICE_INFO 0xE3U
#define ANO_ID_TELEMETRY 0xF1U
#define ANO_ID_DIAGNOSTIC 0xF5U
#define ANO_RX_QUEUE_SIZE 128U
#define ANO_DATA_MAX 96U
#define ANO_TX_QUEUE_DEPTH 4U
#define ANO_DEFAULT_BAUD_RATE 460800U
#define ANO_BAUD_SWITCH_DELAY_MS 50U
#define ANO_STORAGE_ADDRESS 0U
#define ANO_STORAGE_MAGIC 0x414E4F57UL
#define ANO_STORAGE_VERSION 1U

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
    {{0x03U, ANO_COMMAND_CAR_DEBUG_STOP, 0x00U}, "停止M1M2调试", "四路PWM置零并退出M1和M2调试模式"}
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
static uint8_t car_debug_active;
static uint32_t last_telemetry_ms;
static uint32_t last_diagnostic_ms;
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
    uint32_t checksum;
} ano_storage_record_t;

static void put_u16(uint8_t *d, uint16_t *i, uint16_t v) { d[(*i)++] = (uint8_t)v; d[(*i)++] = (uint8_t)(v >> 8U); }
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
    car_debug_active = 0U;

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
        record.magic != ANO_STORAGE_MAGIC || record.version != ANO_STORAGE_VERSION ||
        record.checksum != storage_record_checksum(&record) || !baud_rate_is_valid(record.baud_rate))
    {
        return;
    }
    for (uint16_t id = 0U; id < HOST_BAUD; id++)
    {
        float value = record.values[id];
        float *target = parameter(id);
        if (target == NULL || !isfinite(value)) continue;
        if (id <= CAR_KD || id >= CAR_TYPE) (void)write_car_parameter(id, value);
        else if (value >= -1000.0f && value <= 1000.0f) *target = value;
    }
    host_baud_rate = record.baud_rate;
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
    static float *const values[ANO_PARAM_COUNT] = {
        &car_pid[0], &car_pid[1], &car_pid[2],
        &yaw_pid.Kp, &yaw_pid.Ki, &yaw_pid.Kd,
        &pitch_pid.Kp, &pitch_pid.Ki, &pitch_pid.Kd,
        &car_type, &car_deadzone, &car_line, &car_wheel_diameter, &car_reduction_ratio, NULL,
        &car_debug_control_mode, &car_debug_m1_value, &car_debug_m1_direction,
        &car_debug_m2_value, &car_debug_m2_direction
    };
    return id < ANO_PARAM_COUNT ? values[id] : NULL;
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
        "M1M2控制模式", "M1调试数值", "M1调试方向", "M2调试数值", "M2调试方向"
    };
    static const char *const infos[ANO_PARAM_COUNT] = {
        "四驱速度环比例系数", "四驱速度环积分系数", "四驱速度环微分系数",
        "航向轴PID比例系数", "航向轴PID积分系数", "航向轴PID微分系数",
        "俯仰轴PID比例系数", "俯仰轴PID积分系数", "俯仰轴PID微分系数",
        "四驱电机型号", "四驱控制死区", "四驱编码器相位线", "车轮直径，单位毫米", "四驱减速比", "匿名上位机波特率",
        "0为速度模式，1为PWM模式", "速度范围0至1000，PWM范围0至3600", "0为正转，1为反转",
        "速度范围0至1000，PWM范围0至3600", "0为正转，1为反转"
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
    const car_motor_config_t defaults = {
        .type = CAR_MOTOR_TYPE_310,
        .deadzone = 1600U,
        .line = 500U,
        .reduction_ratio = 30U,
        .wheel_diameter_mm = 67.0f,
        .pid_kp = 0.8f,
        .pid_ki = 0.06f,
        .pid_kd = 0.5f
    };

    storage_operation = 3U;
    if (!storage_ready || !W25Qxx_EraseSector(ANO_STORAGE_ADDRESS) ||
        !Car_Motor_Apply_Config(&defaults))
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

static void send_telemetry(void)
{
    const car_motor_telemetry_t *car = Car_Motor_Get_Telemetry(); uint8_t d[ANO_DATA_MAX]; uint16_t i = 0U;
    for (uint8_t n = 0U; n < 4U; n++) put_i32(d, &i, car->total_encoder[n]);
    for (uint8_t n = 0U; n < 4U; n++) put_i32(d, &i, car->encoder_delta_10ms[n]);
    for (uint8_t n = 0U; n < 4U; n++) put_float(d, &i, car->speed[n]);
    for (uint8_t n = 0U; n < 2U; n++) { put_float(d, &i, zdt_stepper_motors[n]->data.current_position); put_float(d, &i, zdt_stepper_motors[n]->data.current_speed); d[i++] = zdt_stepper_motors[n]->data.motor_status_flags; }
    send_frame(ANO_ID_TELEMETRY, d, i);
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
    if (n == 3U && d[0] == 0x03U && d[1] == 0x03U && d[2] == 0x00U) return car_debug_active ? stop_car_debug() : Car_Motor_Set_Speed(0, 0, 0, 0);
    if (n == 3U && d[0] == 0x02U && d[1] == 0x12U && d[2] == 0xFFU) { ZDT_Stepper_stop(zdt_stepper_motors[0], 0U); ZDT_Stepper_stop(zdt_stepper_motors[1], 0U); return true; }
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_READ_FLASH && d[2] == 0x00U) return Car_Motor_Read_Flash();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_FACTORY_RESET && d[2] == 0x00U) return Car_Motor_Factory_Reset();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_APPLY_CONFIG && d[2] == 0x00U) return apply_car_config();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_DEBUG_ENTER && d[2] == 0x00U) return enter_car_debug();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_DEBUG_START && d[2] == 0x00U) return start_car_debug();
    if (n == 3U && d[0] == 0x03U && d[1] == ANO_COMMAND_CAR_DEBUG_STOP && d[2] == 0x00U) return stop_car_debug();
    if (n == 9U && d[0] == 1U && !car_debug_active) return Car_Motor_Set_Speed(get_i16(&d[1]), get_i16(&d[3]), get_i16(&d[5]), get_i16(&d[7]));
    if (n == 9U && d[0] == 2U && !car_debug_active) return Car_Motor_Set_PWM(get_i16(&d[1]), get_i16(&d[3]), get_i16(&d[5]), get_i16(&d[7]));
    if (n == 1U && d[0] == 3U) return car_debug_active ? stop_car_debug() : Car_Motor_Set_Speed(0, 0, 0, 0);
    if (n == 7U && d[0] == 0x10U && d[1] < 2U) { float s = get_float(&d[3]); if (s >= 0.0f && s <= 1000.0f) { ZDT_Stepper_Set_Speed(zdt_stepper_motors[d[1]], d[2] ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW, s, 0U, 0U); return true; } }
    if (n == 12U && d[0] == 0x11U && d[1] < 2U) { float s = get_float(&d[3]), p = get_float(&d[7]); if (s >= 0.0f && s <= 1000.0f && p >= 0.0f && p <= 3600.0f) { ZDT_Stepper_Set_position(zdt_stepper_motors[d[1]], d[2] ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW, s, p, d[11] ? ZDT_STEPPER_ABS_POS_MODE : ZDT_STEPPER_REL_POS_MODE, 0U); return true; } }
    if (n == 2U && d[0] == 0x12U) { if (d[1] == 0xFFU) { ZDT_Stepper_stop(zdt_stepper_motors[0], 0U); ZDT_Stepper_stop(zdt_stepper_motors[1], 0U); return true; } if (d[1] < 2U) { ZDT_Stepper_stop(zdt_stepper_motors[d[1]], 0U); return true; } }
    if (n == 1U && d[0] == ANO_COMMAND_CAR_READ_FLASH) return Car_Motor_Read_Flash();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_FACTORY_RESET) return Car_Motor_Factory_Reset();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_APPLY_CONFIG) return apply_car_config();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_DEBUG_ENTER) return enter_car_debug();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_DEBUG_START) return start_car_debug();
    if (n == 1U && d[0] == ANO_COMMAND_CAR_DEBUG_STOP) return stop_car_debug();
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
            baud_rate_is_valid((uint32_t)value) && storage_ready && save_storage((uint32_t)value)) {
            pending_baud_rate = (uint32_t)value;
            baud_switch_at_ms = HAL_GetTick() + ANO_BAUD_SWITCH_DELAY_MS;
            accepted = true;
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
    storage_ready = W25Qxx_Init() ? 1U : 0U;
    if (storage_ready) load_storage();
    if (host_baud_rate != huart1.Init.BaudRate) apply_baud_rate(host_baud_rate);
    else HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}
void ANO_Host_Process(void) { while (rx_head != rx_tail) { uint8_t b = rx_queue[rx_head]; rx_head = (rx_head + 1U) % ANO_RX_QUEUE_SIZE; feed(b); } if (pending_baud_rate != 0U && (int32_t)(HAL_GetTick() - baud_switch_at_ms) >= 0 && !tx_active && tx_count == 0U) { uint32_t baud_rate = pending_baud_rate; pending_baud_rate = 0U; apply_baud_rate(baud_rate); } if (HAL_GetTick() - last_telemetry_ms >= 100U) { last_telemetry_ms = HAL_GetTick(); send_telemetry(); } if (HAL_GetTick() - last_diagnostic_ms >= 500U) { last_diagnostic_ms = HAL_GetTick(); send_storage_diagnostic(); } }
void ANO_Host_UART_RxCplt_Callback(void) { uint8_t next = (rx_tail + 1U) % ANO_RX_QUEUE_SIZE; if (next != rx_head) { rx_queue[rx_tail] = rx_byte; rx_tail = next; } HAL_UART_Receive_IT(&huart1, &rx_byte, 1U); }
void ANO_Host_UART_TxCplt_Callback(void) { if (tx_active && tx_count) { tx_head = (tx_head + 1U) % ANO_TX_QUEUE_DEPTH; tx_count--; } tx_active = 0U; start_tx(); }
void ANO_Host_UART_Error_Callback(void) { tx_active = 0U; HAL_UART_Receive_IT(&huart1, &rx_byte, 1U); }
uint32_t ANO_Host_Get_BaudRate(void) { return host_baud_rate; }
