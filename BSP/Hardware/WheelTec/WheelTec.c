#include "WheelTec.h"

#include "usart.h"

#include <math.h>
#include <string.h>

#define WHEELTEC_BAUD_RATE 115200U
#define WHEELTEC_FRAME_HEAD 0xFCU
#define WHEELTEC_FRAME_END 0xFDU
#define WHEELTEC_TYPE_IMU 0x40U
#define WHEELTEC_TYPE_AHRS 0x41U
#define WHEELTEC_IMU_PAYLOAD_LEN 0x38U
#define WHEELTEC_AHRS_PAYLOAD_LEN 0x30U
#define WHEELTEC_IMU_FRAME_SIZE 64U
#define WHEELTEC_AHRS_FRAME_SIZE 56U
#define WHEELTEC_MAX_FRAME_SIZE WHEELTEC_IMU_FRAME_SIZE
#define WHEELTEC_TIMEOUT_MS 250U

static uint8_t rx_byte;
static uint8_t rx_frame[WHEELTEC_MAX_FRAME_SIZE];
static uint8_t rx_index;
static uint8_t expected_length;
static wheeltec_imu_data_t imu_data;
static wheeltec_ahrs_data_t ahrs_data;
static volatile uint8_t imu_valid;
static volatile uint8_t ahrs_valid;
static volatile uint32_t last_imu_ms;
static volatile uint32_t last_ahrs_ms;
static volatile uint32_t valid_frame_count;
static volatile uint32_t invalid_frame_count;
static volatile uint32_t uart_error_count;

static float WheelTec_Decode_Float(const uint8_t *data)
{
    uint32_t raw = (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
                   ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static uint64_t WheelTec_Decode_U64(const uint8_t *data)
{
    uint64_t value = 0U;
    for (uint8_t i = 0U; i < 8U; i++)
    {
        value |= (uint64_t)data[i] << (8U * i);
    }
    return value;
}

static bool WheelTec_Vector_Is_Finite(const wheeltec_vector3_t *vector)
{
    return isfinite(vector->x) && isfinite(vector->y) && isfinite(vector->z);
}

static void WheelTec_Parse_IMU(void)
{
    wheeltec_imu_data_t next = {0};

    next.angular_velocity.x = WheelTec_Decode_Float(&rx_frame[7]);
    next.angular_velocity.y = WheelTec_Decode_Float(&rx_frame[11]);
    next.angular_velocity.z = WheelTec_Decode_Float(&rx_frame[15]);
    next.acceleration.x = WheelTec_Decode_Float(&rx_frame[19]);
    next.acceleration.y = WheelTec_Decode_Float(&rx_frame[23]);
    next.acceleration.z = WheelTec_Decode_Float(&rx_frame[27]);
    next.magnetic_field.x = WheelTec_Decode_Float(&rx_frame[31]);
    next.magnetic_field.y = WheelTec_Decode_Float(&rx_frame[35]);
    next.magnetic_field.z = WheelTec_Decode_Float(&rx_frame[39]);
    next.imu_temperature = WheelTec_Decode_Float(&rx_frame[43]);
    next.pressure = WheelTec_Decode_Float(&rx_frame[47]);
    next.pressure_temperature = WheelTec_Decode_Float(&rx_frame[51]);
    next.timestamp_us = WheelTec_Decode_U64(&rx_frame[55]);
    if (!WheelTec_Vector_Is_Finite(&next.angular_velocity) ||
        !WheelTec_Vector_Is_Finite(&next.acceleration) ||
        !WheelTec_Vector_Is_Finite(&next.magnetic_field) ||
        !isfinite(next.imu_temperature) || !isfinite(next.pressure) ||
        !isfinite(next.pressure_temperature))
    {
        invalid_frame_count++;
        return;
    }
    next.sequence = imu_data.sequence + 1U;
    imu_data = next;
    imu_valid = 1U;
    valid_frame_count++;
    last_imu_ms = HAL_GetTick();
}

static void WheelTec_Parse_AHRS(void)
{
    wheeltec_ahrs_data_t next = {0};

    next.angular_velocity.x = WheelTec_Decode_Float(&rx_frame[7]);
    next.angular_velocity.y = WheelTec_Decode_Float(&rx_frame[11]);
    next.angular_velocity.z = WheelTec_Decode_Float(&rx_frame[15]);
    next.angle.x = WheelTec_Decode_Float(&rx_frame[19]);
    next.angle.y = WheelTec_Decode_Float(&rx_frame[23]);
    next.angle.z = WheelTec_Decode_Float(&rx_frame[27]);
    next.quaternion_w = WheelTec_Decode_Float(&rx_frame[31]);
    next.quaternion_x = WheelTec_Decode_Float(&rx_frame[35]);
    next.quaternion_y = WheelTec_Decode_Float(&rx_frame[39]);
    next.quaternion_z = WheelTec_Decode_Float(&rx_frame[43]);
    next.timestamp_us = WheelTec_Decode_U64(&rx_frame[47]);
    if (!WheelTec_Vector_Is_Finite(&next.angular_velocity) ||
        !WheelTec_Vector_Is_Finite(&next.angle) || !isfinite(next.quaternion_w) ||
        !isfinite(next.quaternion_x) || !isfinite(next.quaternion_y) ||
        !isfinite(next.quaternion_z))
    {
        invalid_frame_count++;
        return;
    }
    next.sequence = ahrs_data.sequence + 1U;
    ahrs_data = next;
    ahrs_valid = 1U;
    valid_frame_count++;
    last_ahrs_ms = HAL_GetTick();
}

static void WheelTec_Consume_Byte(uint8_t value)
{
    if (rx_index == 0U)
    {
        if (value == WHEELTEC_FRAME_HEAD)
        {
            rx_frame[rx_index++] = value;
        }
        return;
    }

    rx_frame[rx_index++] = value;
    if (rx_index == 3U)
    {
        if (rx_frame[1] == WHEELTEC_TYPE_IMU && rx_frame[2] == WHEELTEC_IMU_PAYLOAD_LEN)
        {
            expected_length = WHEELTEC_IMU_FRAME_SIZE;
        }
        else if (rx_frame[1] == WHEELTEC_TYPE_AHRS && rx_frame[2] == WHEELTEC_AHRS_PAYLOAD_LEN)
        {
            expected_length = WHEELTEC_AHRS_FRAME_SIZE;
        }
        else
        {
            invalid_frame_count++;
            rx_index = value == WHEELTEC_FRAME_HEAD ? 1U : 0U;
            if (rx_index != 0U)
                rx_frame[0] = value;
            expected_length = 0U;
            return;
        }
    }
    if (expected_length != 0U && rx_index == expected_length)
    {
        if (rx_frame[expected_length - 1U] == WHEELTEC_FRAME_END)
        {
            if (rx_frame[1] == WHEELTEC_TYPE_IMU)
                WheelTec_Parse_IMU();
            else
                WheelTec_Parse_AHRS();
        }
        else
        {
            invalid_frame_count++;
        }
        rx_index = 0U;
        expected_length = 0U;
    }
}

void WheelTec_Init(void)
{
    memset(&imu_data, 0, sizeof(imu_data));
    memset(&ahrs_data, 0, sizeof(ahrs_data));
    rx_index = 0U;
    expected_length = 0U;
    imu_valid = 0U;
    ahrs_valid = 0U;
    last_imu_ms = 0U;
    last_ahrs_ms = 0U;
    valid_frame_count = 0U;
    invalid_frame_count = 0U;
    uart_error_count = 0U;

    huart2.Init.BaudRate = WHEELTEC_BAUD_RATE;
    if (HAL_UART_Init(&huart2) == HAL_OK)
    {
        (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
    }
}

bool WheelTec_Get_IMU(wheeltec_imu_data_t *data)
{
    if (data == NULL || imu_valid == 0U)
        return false;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *data = imu_data;
    if (primask == 0U)
        __enable_irq();
    return true;
}

bool WheelTec_Get_AHRS(wheeltec_ahrs_data_t *data)
{
    if (data == NULL || ahrs_valid == 0U)
        return false;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *data = ahrs_data;
    if (primask == 0U)
        __enable_irq();
    return true;
}

void WheelTec_Get_Status(wheeltec_status_t *status)
{
    if (status == NULL)
        return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    status->imu_valid = imu_valid;
    status->ahrs_valid = ahrs_valid;
    status->reserved = 0U;
    status->valid_frame_count = valid_frame_count;
    status->invalid_frame_count = invalid_frame_count;
    status->uart_error_count = uart_error_count;
    uint32_t imu_ms = last_imu_ms;
    uint32_t ahrs_ms = last_ahrs_ms;
    if (primask == 0U)
        __enable_irq();
    uint32_t now = HAL_GetTick();
    status->online = imu_ms != 0U && ahrs_ms != 0U &&
                     (uint32_t)(now - imu_ms) <= WHEELTEC_TIMEOUT_MS &&
                     (uint32_t)(now - ahrs_ms) <= WHEELTEC_TIMEOUT_MS;
}

void WheelTec_UART_RxCplt_Callback(void)
{
    WheelTec_Consume_Byte(rx_byte);
    (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
}

void WheelTec_UART_Error_Callback(void)
{
    uart_error_count++;
    rx_index = 0U;
    expected_length = 0U;
    (void)HAL_UART_Receive_IT(&huart2, &rx_byte, 1U);
}
