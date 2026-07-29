#include "Car_Motor_USART.h"

#include "App_Config.h"
#include "usart.h"
#include "ZDT_Stepper.h"
#include "ANO_Host.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define CAR_MOTOR_FRAME_MAX_LENGTH 96U
#define CAR_MOTOR_TX_QUEUE_DEPTH 8U

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

static car_motor_telemetry_t car_motor_telemetry;

static void Car_Motor_Start_Next_Transmit(void)
{
    if (car_motor_tx_active || car_motor_tx_count == 0U)
    {
        return;
    }

    car_motor_tx_active = 1U;
    if (HAL_UART_Transmit_IT(&huart3,
                             (uint8_t *)car_motor_tx_queue[car_motor_tx_head],
                             car_motor_tx_length[car_motor_tx_head]) != HAL_OK)
    {
        car_motor_tx_active = 0U;
    }
}

static bool Car_Motor_Queue_Command(const char *command)
{
    size_t length = strlen(command);
    if (length == 0U || length >= CAR_MOTOR_FRAME_MAX_LENGTH)
    {
        return false;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (car_motor_tx_count >= CAR_MOTOR_TX_QUEUE_DEPTH)
    {
        __set_PRIMASK(primask);
        return false;
    }

    memcpy(car_motor_tx_queue[car_motor_tx_tail], command, length);
    car_motor_tx_length[car_motor_tx_tail] = (uint8_t)length;
    car_motor_tx_tail = (uint8_t)((car_motor_tx_tail + 1U) % CAR_MOTOR_TX_QUEUE_DEPTH);
    car_motor_tx_count++;
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

    return length > 0 && (size_t)length < sizeof(command) && Car_Motor_Queue_Command(command);
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
    if (strncmp(frame, "MAll:", 5U) == 0)
    {
        Car_Motor_Parse_Int32x4(frame + 5, car_motor_telemetry.total_encoder);
    }
    else if (strncmp(frame, "MTEP:", 5U) == 0)
    {
        Car_Motor_Parse_Int32x4(frame + 5, car_motor_telemetry.encoder_delta_10ms);
    }
    else if (strncmp(frame, "MSPD:", 5U) == 0)
    {
        Car_Motor_Parse_Floatx4(frame + 5, car_motor_telemetry.speed);
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
    memset(&car_motor_telemetry, 0, sizeof(car_motor_telemetry));
    HAL_UART_Receive_IT(&huart3, &car_motor_rx_byte, 1U);
}

void Car_Motor_Process(void)
{
    char frame[CAR_MOTOR_FRAME_MAX_LENGTH];
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!car_motor_frame_ready)
    {
        __set_PRIMASK(primask);
        return;
    }

    memcpy(frame, car_motor_pending_frame, sizeof(frame));
    car_motor_frame_ready = 0U;
    __set_PRIMASK(primask);
    Car_Motor_Parse_Frame(frame);
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

bool Car_Motor_Set_Wheel_Diameter(float diameter_mm)
{
    return diameter_mm > 0.0f && Car_Motor_Format_Command("$wdiameter:%.3f#", (double)diameter_mm);
}

bool Car_Motor_Set_PID(float kp, float ki, float kd)
{
    return Car_Motor_Format_Command("$MPID:%.3f,%.3f,%.3f#", (double)kp, (double)ki, (double)kd);
}

bool Car_Motor_Factory_Reset(void)
{
    return Car_Motor_Queue_Command("$flash_reset#");
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
    return Car_Motor_Queue_Command("$read_flash#");
}

bool Car_Motor_Read_Voltage(void)
{
    return Car_Motor_Queue_Command("$read_vol#");
}

const car_motor_telemetry_t *Car_Motor_Get_Telemetry(void)
{
    return &car_motor_telemetry;
}
