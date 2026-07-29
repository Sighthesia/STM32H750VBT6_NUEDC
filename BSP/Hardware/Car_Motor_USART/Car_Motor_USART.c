#include "Car_Motor_USART.h"

#include "App_Config.h"
#include "usart.h"
#include "ZDT_Stepper.h"
#include "ANO_Host.h"

#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>

#define CAR_MOTOR_FRAME_MAX_LENGTH 96U
#define CAR_MOTOR_TX_QUEUE_DEPTH 8U
#define CAR_MOTOR_VOLTAGE_REQUEST_INTERVAL_MS 1000U
#define CAR_MOTOR_VOLTAGE_TIMEOUT_MS 3000U

static uint8_t car_motor_rx_byte;
static char car_motor_rx_frame[CAR_MOTOR_FRAME_MAX_LENGTH];
static volatile uint8_t car_motor_rx_length;
static volatile uint8_t car_motor_rx_active;
static volatile uint8_t car_motor_frame_ready;
static char car_motor_pending_frame[CAR_MOTOR_FRAME_MAX_LENGTH];

static char car_motor_tx_queue[CAR_MOTOR_TX_QUEUE_DEPTH][CAR_MOTOR_FRAME_MAX_LENGTH];
static uint8_t car_motor_tx_length[CAR_MOTOR_TX_QUEUE_DEPTH];
static volatile uint8_t car_motor_tx_head;
static volatile uint8_t car_motor_tx_tail;
static volatile uint8_t car_motor_tx_count;
static volatile uint8_t car_motor_tx_active;
static uint16_t car_motor_tx_sequence[CAR_MOTOR_TX_QUEUE_DEPTH];
static uint16_t car_motor_next_sequence;
static uint16_t car_motor_last_sequence;
static uint32_t car_motor_last_voltage_request_ms;
static uint32_t car_motor_last_voltage_response_ms;

static car_motor_telemetry_t car_motor_telemetry;

static void Car_Motor_Record_Command(const char *command, car_motor_command_status_t status, uint16_t sequence)
{
    size_t length = strlen(command);
    memset(car_motor_telemetry.last_command, 0, sizeof(car_motor_telemetry.last_command));
    if (length >= sizeof(car_motor_telemetry.last_command))
    {
        length = sizeof(car_motor_telemetry.last_command) - 1U;
    }
    memcpy(car_motor_telemetry.last_command, command, length);
    car_motor_telemetry.command_status = (uint8_t)status;
    car_motor_last_sequence = sequence;
}

static void Car_Motor_Set_Command_Status(uint16_t sequence, car_motor_command_status_t status)
{
    if (sequence == car_motor_last_sequence)
    {
        car_motor_telemetry.command_status = (uint8_t)status;
    }
}

static void Car_Motor_Start_Next_Transmit(void)
{
    while (!car_motor_tx_active && car_motor_tx_count != 0U)
    {
        car_motor_tx_active = 1U;
        if (HAL_UART_Transmit_IT(&huart3,
                                 (uint8_t *)car_motor_tx_queue[car_motor_tx_head],
                                 car_motor_tx_length[car_motor_tx_head]) == HAL_OK)
        {
            return;
        }

        Car_Motor_Set_Command_Status(car_motor_tx_sequence[car_motor_tx_head], CAR_MOTOR_COMMAND_FAILED);
        car_motor_tx_head = (uint8_t)((car_motor_tx_head + 1U) % CAR_MOTOR_TX_QUEUE_DEPTH);
        car_motor_tx_count--;
        car_motor_tx_active = 0U;
    }
}

static bool Car_Motor_Queue_Command(const char *command, bool record_command)
{
    size_t length = strlen(command);
    if (length == 0U || length >= CAR_MOTOR_FRAME_MAX_LENGTH)
    {
        if (record_command) Car_Motor_Record_Command(command, CAR_MOTOR_COMMAND_FAILED, 0U);
        return false;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (car_motor_tx_count >= CAR_MOTOR_TX_QUEUE_DEPTH)
    {
        __set_PRIMASK(primask);
        if (record_command) Car_Motor_Record_Command(command, CAR_MOTOR_COMMAND_FAILED, 0U);
        return false;
    }

    memcpy(car_motor_tx_queue[car_motor_tx_tail], command, length);
    car_motor_tx_length[car_motor_tx_tail] = (uint8_t)length;
    car_motor_tx_sequence[car_motor_tx_tail] = ++car_motor_next_sequence;
    if (record_command) Car_Motor_Record_Command(command, CAR_MOTOR_COMMAND_QUEUED, car_motor_tx_sequence[car_motor_tx_tail]);
    car_motor_tx_tail = (uint8_t)((car_motor_tx_tail + 1U) % CAR_MOTOR_TX_QUEUE_DEPTH);
    car_motor_tx_count++;
    Car_Motor_Start_Next_Transmit();
    __set_PRIMASK(primask);
    return true;
}

static bool Car_Motor_Queue_Batch(char commands[][CAR_MOTOR_FRAME_MAX_LENGTH], uint8_t count)
{
    uint8_t lengths[CAR_MOTOR_TX_QUEUE_DEPTH];
    if (count == 0U || count > CAR_MOTOR_TX_QUEUE_DEPTH)
    {
        return false;
    }

    for (uint8_t i = 0U; i < count; i++)
    {
        size_t length = strlen(commands[i]);
        if (length == 0U || length >= CAR_MOTOR_FRAME_MAX_LENGTH)
        {
            return false;
        }
        lengths[i] = (uint8_t)length;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (car_motor_tx_count > (CAR_MOTOR_TX_QUEUE_DEPTH - count))
    {
        __set_PRIMASK(primask);
        return false;
    }

    for (uint8_t i = 0U; i < count; i++)
    {
        memcpy(car_motor_tx_queue[car_motor_tx_tail], commands[i], lengths[i]);
        car_motor_tx_length[car_motor_tx_tail] = lengths[i];
        car_motor_tx_sequence[car_motor_tx_tail] = ++car_motor_next_sequence;
        if (i == count - 1U) Car_Motor_Record_Command(commands[i], CAR_MOTOR_COMMAND_QUEUED, car_motor_tx_sequence[car_motor_tx_tail]);
        car_motor_tx_tail = (uint8_t)((car_motor_tx_tail + 1U) % CAR_MOTOR_TX_QUEUE_DEPTH);
        car_motor_tx_count++;
    }

    Car_Motor_Start_Next_Transmit();
    __set_PRIMASK(primask);
    return true;
}

static bool Car_Motor_Format_Command(const char *format, ...)
{
    char command[CAR_MOTOR_FRAME_MAX_LENGTH];
    va_list arguments;
    va_start(arguments, format);
    int length = vsnprintf(command, sizeof(command), format, arguments);
    va_end(arguments);

    return length > 0 && (size_t)length < sizeof(command) && Car_Motor_Queue_Command(command, true);
}

static bool Car_Motor_Parse_Int32x4(const char *values, int32_t output[4])
{
    long parsed[4];
    char trailing;
    if (sscanf(values, "%ld,%ld,%ld,%ld%c", &parsed[0], &parsed[1], &parsed[2], &parsed[3], &trailing) != 4)
    {
        return false;
    }

    for (uint8_t i = 0; i < 4U; i++)
    {
        output[i] = (int32_t)parsed[i];
    }
    return true;
}

static bool Car_Motor_Parse_Floatx4(const char *values, float output[4])
{
    float parsed[4];
    char trailing;
    if (sscanf(values, "%f,%f,%f,%f%c", &parsed[0], &parsed[1], &parsed[2], &parsed[3], &trailing) != 4)
    {
        return false;
    }

    memcpy(output, parsed, sizeof(parsed));
    return true;
}

static void Car_Motor_Parse_Frame(const char *frame)
{
    bool parsed = false;
    car_motor_driver_frame_t driver_frame = CAR_MOTOR_DRIVER_FRAME_NONE;
    if (strncmp(frame, "MAll:", 5U) == 0)
    {
        parsed = Car_Motor_Parse_Int32x4(frame + 5, car_motor_telemetry.total_encoder);
        driver_frame = CAR_MOTOR_DRIVER_FRAME_TELEMETRY;
    }
    else if (strncmp(frame, "MTEP:", 5U) == 0)
    {
        parsed = Car_Motor_Parse_Int32x4(frame + 5, car_motor_telemetry.encoder_delta_10ms);
        driver_frame = CAR_MOTOR_DRIVER_FRAME_TELEMETRY;
    }
    else if (strncmp(frame, "MSPD:", 5U) == 0)
    {
        parsed = Car_Motor_Parse_Floatx4(frame + 5, car_motor_telemetry.speed);
        driver_frame = CAR_MOTOR_DRIVER_FRAME_TELEMETRY;
    }
    else if (strncmp(frame, "MVol:", 5U) == 0)
    {
        float voltage;
        char trailing;
        if (sscanf(frame + 5, "%f%c", &voltage, &trailing) == 1 && isfinite(voltage) && voltage >= 0.0f)
        {
            car_motor_telemetry.voltage = voltage;
            car_motor_telemetry.voltage_valid = 1U;
            driver_frame = CAR_MOTOR_DRIVER_FRAME_VOLTAGE;
            car_motor_last_voltage_response_ms = HAL_GetTick();
            parsed = true;
        }
    }
    if (parsed)
    {
        car_motor_telemetry.last_driver_frame = (uint8_t)driver_frame;
    }
}

void Car_Motor_Init(void)
{
    car_motor_rx_length = 0U;
    car_motor_rx_active = 0U;
    car_motor_frame_ready = 0U;
    car_motor_tx_head = 0U;
    car_motor_tx_tail = 0U;
    car_motor_tx_count = 0U;
    car_motor_tx_active = 0U;
    car_motor_next_sequence = 0U;
    car_motor_last_sequence = 0U;
    car_motor_last_voltage_request_ms = 0U;
    car_motor_last_voltage_response_ms = 0U;
    memset(&car_motor_telemetry, 0, sizeof(car_motor_telemetry));
    HAL_UART_Receive_IT(&huart3, &car_motor_rx_byte, 1U);
}

void Car_Motor_Process(void)
{
    char frame[CAR_MOTOR_FRAME_MAX_LENGTH];
    bool frame_ready;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    frame_ready = car_motor_frame_ready != 0U;
    if (frame_ready)
    {
        memcpy(frame, car_motor_pending_frame, sizeof(frame));
        car_motor_frame_ready = 0U;
    }
    __set_PRIMASK(primask);
    if (frame_ready) Car_Motor_Parse_Frame(frame);

    uint32_t now = HAL_GetTick();
    if (car_motor_telemetry.voltage_valid && now - car_motor_last_voltage_response_ms >= CAR_MOTOR_VOLTAGE_TIMEOUT_MS)
    {
        car_motor_telemetry.voltage_valid = 0U;
    }
    if (now - car_motor_last_voltage_request_ms >= CAR_MOTOR_VOLTAGE_REQUEST_INTERVAL_MS &&
        Car_Motor_Queue_Command("$read_vol#", false))
    {
        car_motor_last_voltage_request_ms = now;
    }
}

void Car_Motor_UART_RxCplt_Callback(void)
{
    uint8_t data = car_motor_rx_byte;
    if (data == '$')
    {
        car_motor_rx_active = 1U;
        car_motor_rx_length = 0U;
    }
    else if (car_motor_rx_active && data == '#')
    {
        if (!car_motor_frame_ready)
        {
            car_motor_rx_frame[car_motor_rx_length] = '\0';
            memcpy(car_motor_pending_frame, car_motor_rx_frame, car_motor_rx_length + 1U);
            car_motor_frame_ready = 1U;
        }
        car_motor_rx_active = 0U;
    }
    else if (car_motor_rx_active)
    {
        if (car_motor_rx_length < (CAR_MOTOR_FRAME_MAX_LENGTH - 1U))
        {
            car_motor_rx_frame[car_motor_rx_length++] = (char)data;
        }
        else
        {
            car_motor_rx_active = 0U;
        }
    }

    HAL_UART_Receive_IT(&huart3, &car_motor_rx_byte, 1U);
}

void Car_Motor_UART_TxCplt_Callback(void)
{
    if (!car_motor_tx_active)
    {
        return;
    }

    Car_Motor_Set_Command_Status(car_motor_tx_sequence[car_motor_tx_head], CAR_MOTOR_COMMAND_TRANSMITTED);
    car_motor_tx_head = (uint8_t)((car_motor_tx_head + 1U) % CAR_MOTOR_TX_QUEUE_DEPTH);
    car_motor_tx_count--;
    car_motor_tx_active = 0U;
    Car_Motor_Start_Next_Transmit();
}

void Car_Motor_UART_Error_Callback(void)
{
    car_motor_tx_active = 0U;
    HAL_UART_Receive_IT(&huart3, &car_motor_rx_byte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) { ANO_Host_UART_RxCplt_Callback(); return; }
#if APP_ENABLE_CAR_MOTOR
    if (huart->Instance == USART3)
    {
        Car_Motor_UART_RxCplt_Callback();
    }
#else
    (void)huart;
#endif
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
#if APP_ENABLE_ZDT_STEPPER
    ZDT_Stepper_USRT_RX_callback(huart, size);
#else
    (void)huart;
    (void)size;
#endif
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) { ANO_Host_UART_TxCplt_Callback(); return; }
#if APP_ENABLE_CAR_MOTOR
    if (huart->Instance == USART3)
    {
        Car_Motor_UART_TxCplt_Callback();
        return;
    }
#endif
#if APP_ENABLE_ZDT_STEPPER
    ZDT_Stepper_UART_TxCplt_Callback(huart);
#else
    (void)huart;
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) { ANO_Host_UART_Error_Callback(); return; }
#if APP_ENABLE_CAR_MOTOR
    if (huart->Instance == USART3)
    {
        Car_Motor_UART_Error_Callback();
        return;
    }
#endif
#if APP_ENABLE_ZDT_STEPPER
    ZDT_Stepper_UART_Error_Callback(huart);
#else
    (void)huart;
#endif
}

bool Car_Motor_Set_Type(car_motor_type_t type)
{
    return type >= CAR_MOTOR_TYPE_520 && type <= CAR_MOTOR_TYPE_TT &&
           Car_Motor_Format_Command("$mtype:%u#", (unsigned int)type);
}

bool Car_Motor_Set_Deadzone(uint16_t deadzone)
{
    return deadzone <= 3600U && Car_Motor_Format_Command("$deadzone:%u#", deadzone);
}

bool Car_Motor_Set_Line(uint16_t line)
{
    return Car_Motor_Format_Command("$mline:%u#", line);
}

bool Car_Motor_Set_Reduction_Ratio(uint16_t reduction_ratio)
{
    return reduction_ratio > 0U && Car_Motor_Format_Command("$mphase:%u#", reduction_ratio);
}

bool Car_Motor_Set_Wheel_Diameter(float diameter_mm)
{
    return diameter_mm > 0.0f && Car_Motor_Format_Command("$wdiameter:%.3f#", (double)diameter_mm);
}

bool Car_Motor_Set_PID(float kp, float ki, float kd)
{
    return Car_Motor_Format_Command("$MPID:%.3f,%.3f,%.3f#", (double)kp, (double)ki, (double)kd);
}

bool Car_Motor_Apply_Config(const car_motor_config_t *config)
{
    char commands[6][CAR_MOTOR_FRAME_MAX_LENGTH];
    if (config == NULL || config->type < CAR_MOTOR_TYPE_520 || config->type > CAR_MOTOR_TYPE_TT ||
        config->deadzone > 3600U || config->reduction_ratio == 0U || config->wheel_diameter_mm <= 0.0f)
    {
        return false;
    }

    int lengths[6] = {
        snprintf(commands[0], sizeof(commands[0]), "$mtype:%u#", (unsigned int)config->type),
        snprintf(commands[1], sizeof(commands[1]), "$deadzone:%u#", config->deadzone),
        snprintf(commands[2], sizeof(commands[2]), "$mline:%u#", config->line),
        snprintf(commands[3], sizeof(commands[3]), "$mphase:%u#", config->reduction_ratio),
        snprintf(commands[4], sizeof(commands[4]), "$wdiameter:%.3f#", (double)config->wheel_diameter_mm),
        snprintf(commands[5], sizeof(commands[5]), "$MPID:%.3f,%.3f,%.3f#",
                 (double)config->pid_kp, (double)config->pid_ki, (double)config->pid_kd)
    };
    for (uint8_t i = 0U; i < sizeof(lengths) / sizeof(lengths[0]); i++)
    {
        if (lengths[i] < 0 || lengths[i] >= (int)sizeof(commands[i]))
        {
            return false;
        }
    }

    return Car_Motor_Queue_Batch(commands, sizeof(commands) / sizeof(commands[0]));
}

bool Car_Motor_Factory_Reset(void)
{
    return Car_Motor_Queue_Command("$flash_reset#", true);
}

bool Car_Motor_Restore_Defaults(void)
{
    char commands[7][CAR_MOTOR_FRAME_MAX_LENGTH] = {
        "$flash_reset#",
        "$mtype:2#",
        "$deadzone:1600#",
        "$mline:500#",
        "$mphase:30#",
        "$wdiameter:67.000#",
        "$MPID:0.800,0.060,0.500#"
    };

    if (Car_Motor_Queue_Batch(commands, sizeof(commands) / sizeof(commands[0])))
    {
        return true;
    }

    Car_Motor_Record_Command("$flash_reset#", CAR_MOTOR_COMMAND_FAILED, 0U);
    return false;
}

bool Car_Motor_Set_Speed(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    return m1 >= -1000 && m1 <= 1000 && m2 >= -1000 && m2 <= 1000 &&
           m3 >= -1000 && m3 <= 1000 && m4 >= -1000 && m4 <= 1000 &&
           Car_Motor_Format_Command("$spd:%d,%d,%d,%d#", m1, m2, m3, m4);
}

bool Car_Motor_Set_PWM(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    return m1 >= -3600 && m1 <= 3600 && m2 >= -3600 && m2 <= 3600 &&
           m3 >= -3600 && m3 <= 3600 && m4 >= -3600 && m4 <= 3600 &&
           Car_Motor_Format_Command("$pwm:%d,%d,%d,%d#", m1, m2, m3, m4);
}

bool Car_Motor_Set_Upload(bool total_encoder, bool encoder_delta_10ms, bool speed)
{
    return Car_Motor_Format_Command("$upload:%u,%u,%u#", total_encoder, encoder_delta_10ms, speed);
}

bool Car_Motor_Read_Flash(void)
{
    return Car_Motor_Queue_Command("$read_flash#", true);
}

bool Car_Motor_Read_Voltage(void)
{
    return Car_Motor_Queue_Command("$read_vol#", true);
}

const car_motor_telemetry_t *Car_Motor_Get_Telemetry(void)
{
    return &car_motor_telemetry;
}
