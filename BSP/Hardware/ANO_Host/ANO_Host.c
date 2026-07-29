#include "ANO_Host.h"

#include "Car_Motor_USART.h"
#include "Stepper_PID.h"
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
#define ANO_ID_PARAM_COMMAND 0xE0U
#define ANO_ID_PARAM_VALUE 0xE1U
#define ANO_ID_PARAM_INFO 0xE2U
#define ANO_ID_DEVICE_INFO 0xE3U
#define ANO_ID_TELEMETRY 0xF1U
#define ANO_RX_QUEUE_SIZE 128U
#define ANO_DATA_MAX 96U
#define ANO_TX_QUEUE_DEPTH 4U

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
    ANO_PARAM_COUNT
};

typedef struct { uint8_t state, source, destination, id, sum, checksum; uint16_t length, index; uint8_t data[ANO_DATA_MAX]; } ano_parser_t;

static uint8_t rx_byte;
static volatile uint8_t rx_queue[ANO_RX_QUEUE_SIZE], rx_head, rx_tail;
static uint8_t tx_queue[ANO_TX_QUEUE_DEPTH][ANO_DATA_MAX + 8U], tx_length[ANO_TX_QUEUE_DEPTH];
static volatile uint8_t tx_head, tx_tail, tx_count, tx_active;
static ano_parser_t parser;
static float car_pid[3] = {1.0f, 0.0f, 0.0f};
static float car_type = (float)CAR_MOTOR_TYPE_520;
static float car_deadzone;
static float car_line;
static float car_wheel_diameter;
static uint32_t last_telemetry_ms;

static void put_u16(uint8_t *d, uint16_t *i, uint16_t v) { d[(*i)++] = (uint8_t)v; d[(*i)++] = (uint8_t)(v >> 8U); }
static void put_i32(uint8_t *d, uint16_t *i, int32_t v) { memcpy(&d[*i], &v, 4U); *i += 4U; }
static void put_float(uint8_t *d, uint16_t *i, float v) { memcpy(&d[*i], &v, 4U); *i += 4U; }
static uint16_t get_u16(const uint8_t *d) { return (uint16_t)d[0] | ((uint16_t)d[1] << 8U); }
static int16_t get_i16(const uint8_t *d) { return (int16_t)get_u16(d); }
static float get_float(const uint8_t *d) { float v; memcpy(&v, d, 4U); return v; }

static void start_tx(void)
{
    if (!tx_active && tx_count != 0U) { tx_active = 1U; if (HAL_UART_Transmit_IT(&huart1, tx_queue[tx_head], tx_length[tx_head]) != HAL_OK) tx_active = 0U; }
}

static void send_frame(uint8_t id, const uint8_t *data, uint16_t length)
{
    uint8_t sum = 0U, add = 0U; uint16_t index = 0U; uint32_t primask;
    if (length > ANO_DATA_MAX) return;
    primask = __get_PRIMASK(); __disable_irq();
    if (tx_count >= ANO_TX_QUEUE_DEPTH) { __set_PRIMASK(primask); return; }
    uint8_t *frame = tx_queue[tx_tail];
    frame[index++] = ANO_HEAD; frame[index++] = ANO_DEVICE_ADDRESS; frame[index++] = ANO_HOST_ADDRESS; frame[index++] = id; frame[index++] = (uint8_t)length; frame[index++] = (uint8_t)(length >> 8U);
    if (length != 0U)
    {
        memcpy(&frame[index], data, length);
    }
    index += length;
    for (uint16_t i = 0U; i < index; i++) { sum += frame[i]; add += sum; }
    frame[index++] = sum; frame[index++] = add; tx_length[tx_tail] = (uint8_t)index; tx_tail = (tx_tail + 1U) % ANO_TX_QUEUE_DEPTH; tx_count++; start_tx(); __set_PRIMASK(primask);
}

static float *parameter(uint16_t id)
{
    static float *const values[ANO_PARAM_COUNT] = {
        &car_pid[0], &car_pid[1], &car_pid[2],
        &yaw_pid.Kp, &yaw_pid.Ki, &yaw_pid.Kd,
        &pitch_pid.Kp, &pitch_pid.Ki, &pitch_pid.Kd,
        &car_type, &car_deadzone, &car_line, &car_wheel_diameter
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

    return accepted;
}

static void send_check(uint8_t id, uint8_t sum, uint8_t add) { uint8_t d[3] = {id, sum, add}; send_frame(ANO_ID_CHECK, d, sizeof(d)); }
static void send_param(uint16_t id) { uint8_t d[6]; uint16_t i = 0U; float *v = parameter(id); if (v == NULL) return; put_u16(d, &i, id); put_float(d, &i, *v); send_frame(ANO_ID_PARAM_VALUE, d, i); }
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
        "car_kp", "car_ki", "car_kd",
        "yaw_kp", "yaw_ki", "yaw_kd",
        "pitch_kp", "pitch_ki", "pitch_kd",
        "car_type", "car_deadzone", "car_line", "wheel_diameter"
    };
    uint8_t d[23] = {0}; uint16_t i = 0U;
    if (id >= ANO_PARAM_COUNT) return;
    put_u16(d, &i, id); d[i++] = 8U; strncpy((char *)&d[i], names[id], 20U); send_frame(ANO_ID_PARAM_INFO, d, sizeof(d));
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

static void command(const uint8_t *d, uint16_t n)
{
    if (n == 9U && d[0] == 1U) (void)Car_Motor_Set_Speed(get_i16(&d[1]), get_i16(&d[3]), get_i16(&d[5]), get_i16(&d[7]));
    else if (n == 9U && d[0] == 2U) (void)Car_Motor_Set_PWM(get_i16(&d[1]), get_i16(&d[3]), get_i16(&d[5]), get_i16(&d[7]));
    else if (n == 1U && d[0] == 3U) (void)Car_Motor_Set_Speed(0, 0, 0, 0);
    else if (n == 7U && d[0] == 0x10U && d[1] < 2U) { float s = get_float(&d[3]); if (s >= 0.0f && s <= 1000.0f) ZDT_Stepper_Set_Speed(zdt_stepper_motors[d[1]], d[2] ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW, s, 0U, 0U); }
    else if (n == 12U && d[0] == 0x11U && d[1] < 2U) { float s = get_float(&d[3]), p = get_float(&d[7]); if (s >= 0.0f && s <= 1000.0f && p >= 0.0f && p <= 3600.0f) ZDT_Stepper_Set_position(zdt_stepper_motors[d[1]], d[2] ? ZDT_STEPPER_CCW : ZDT_STEPPER_CW, s, p, d[11] ? ZDT_STEPPER_ABS_POS_MODE : ZDT_STEPPER_REL_POS_MODE, 0U); }
    else if (n == 2U && d[0] == 0x12U) { if (d[1] == 0xFFU) { ZDT_Stepper_stop(zdt_stepper_motors[0], 0U); ZDT_Stepper_stop(zdt_stepper_motors[1], 0U); } else if (d[1] < 2U) ZDT_Stepper_stop(zdt_stepper_motors[d[1]], 0U); }
    else if (n == 1U && d[0] == ANO_COMMAND_CAR_READ_FLASH) (void)Car_Motor_Read_Flash();
    else if (n == 1U && d[0] == ANO_COMMAND_CAR_FACTORY_RESET) (void)Car_Motor_Factory_Reset();
}

static void handle(uint8_t id, const uint8_t *d, uint16_t n, uint8_t sum, uint8_t add)
{
    if (id == ANO_ID_PARAM_COMMAND && n >= 3U) { uint16_t v = get_u16(&d[1]); if (d[0] == 0U) send_device_info(); else if (d[0] == 1U) { uint8_t count[3] = {1U, ANO_PARAM_COUNT, 0U}; send_frame(ANO_ID_PARAM_COMMAND, count, 3U); } else if (d[0] == 2U) send_param(v); else if (d[0] == 3U) send_param_info(v); }
    else if (id == ANO_ID_PARAM_VALUE && n == 6U) {
        uint16_t parameter_id = get_u16(d); float value = get_float(&d[2]); float *v = parameter(parameter_id);
        if (parameter_id <= CAR_KD || parameter_id >= CAR_TYPE) {
            (void)write_car_parameter(parameter_id, value);
        } else if (v != NULL && isfinite(value) && value >= -1000.0f && value <= 1000.0f) {
            *v = value;
        }
    }
    else if (id == ANO_ID_COMMAND) command(d, n); else return;
    send_check(id, sum, add);
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
            if (b == add) handle(parser.id, parser.data, parser.length, parser.checksum, b);
        }
        parser.state = 0U;
        break;
    default: parser.state = 0U; break; }
}

void ANO_Host_Init(void)
{
    pid_kp = 3.0f; pid_ki = 1.0f; pid_kd = 0.0f; pid_tau = 0.05f; pid_lim_min = -4.0f; pid_lim_max = 4.0f; pid_lim_min_int = -5.0f; pid_lim_max_int = 5.0f; sample_time_s = 0.01f;
    Stepper_PID_Parameters_Init(&yaw_pid); Stepper_PID_Parameters_Init(&pitch_pid); HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}
void ANO_Host_Process(void) { while (rx_head != rx_tail) { uint8_t b = rx_queue[rx_head]; rx_head = (rx_head + 1U) % ANO_RX_QUEUE_SIZE; feed(b); } if (HAL_GetTick() - last_telemetry_ms >= 100U) { last_telemetry_ms = HAL_GetTick(); send_telemetry(); } }
void ANO_Host_UART_RxCplt_Callback(void) { uint8_t next = (rx_tail + 1U) % ANO_RX_QUEUE_SIZE; if (next != rx_head) { rx_queue[rx_tail] = rx_byte; rx_tail = next; } HAL_UART_Receive_IT(&huart1, &rx_byte, 1U); }
void ANO_Host_UART_TxCplt_Callback(void) { if (tx_active && tx_count) { tx_head = (tx_head + 1U) % ANO_TX_QUEUE_DEPTH; tx_count--; } tx_active = 0U; start_tx(); }
void ANO_Host_UART_Error_Callback(void) { tx_active = 0U; HAL_UART_Receive_IT(&huart1, &rx_byte, 1U); }
