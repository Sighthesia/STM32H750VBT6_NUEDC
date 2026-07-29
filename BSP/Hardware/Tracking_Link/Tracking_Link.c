#include "Tracking_Link.h"

#include "main.h"
#include "usart.h"

#define TRACKING_LINK_HEAD_0       0xA5U
#define TRACKING_LINK_HEAD_1       0x5AU
#define TRACKING_LINK_FRAME_SIZE   5U
#define TRACKING_LINK_TIMEOUT_MS   50U
#define TRACKING_LINK_MAX_ANGLE_X10 300

static uint8_t tracking_link_rx_byte;
static uint8_t tracking_link_frame[TRACKING_LINK_FRAME_SIZE];
static uint8_t tracking_link_index;
static volatile uint8_t tracking_link_state;
static volatile int16_t tracking_link_angle_x10;
static volatile uint32_t tracking_link_last_rx_ms;
static volatile uint8_t tracking_link_valid;

static uint8_t Tracking_Link_Crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;

    while (length-- != 0U)
    {
        crc ^= *data++;
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 0x80U) != 0U ? (uint8_t)((crc << 1U) ^ 0x07U) : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

static bool Tracking_Link_Calculate_Angle(uint8_t state, int16_t *angle_x10)
{
    static const int8_t positions[] = {-7, -5, -3, -1, 1, 3, 5, 7};
    int16_t sum = 0;
    uint8_t count = 0U;

    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        if ((state & (uint8_t)(1U << bit)) != 0U)
        {
            sum += positions[bit];
            count++;
        }
    }
    if (count == 0U)
    {
        return false;
    }

    *angle_x10 = (int16_t)((sum * TRACKING_LINK_MAX_ANGLE_X10) / ((int16_t)count * 7));
    return true;
}

void Tracking_Link_Init(void)
{
    tracking_link_index = 0U;
    tracking_link_valid = 0U;
    (void)HAL_UART_Receive_IT(&huart2, &tracking_link_rx_byte, 1U);
}

void Tracking_Link_UART_RxCplt_Callback(void)
{
    const uint8_t byte = tracking_link_rx_byte;

    if (tracking_link_index == 0U)
    {
        tracking_link_index = byte == TRACKING_LINK_HEAD_0 ? 1U : 0U;
    }
    else if (tracking_link_index == 1U)
    {
        tracking_link_index = byte == TRACKING_LINK_HEAD_1 ? 2U : (byte == TRACKING_LINK_HEAD_0 ? 1U : 0U);
    }
    else
    {
        tracking_link_frame[tracking_link_index++] = byte;
        if (tracking_link_index == TRACKING_LINK_FRAME_SIZE)
        {
            int16_t angle_x10;

            tracking_link_frame[0] = TRACKING_LINK_HEAD_0;
            tracking_link_frame[1] = TRACKING_LINK_HEAD_1;
            if (Tracking_Link_Crc8(tracking_link_frame, TRACKING_LINK_FRAME_SIZE - 1U) == tracking_link_frame[4])
            {
                if (Tracking_Link_Calculate_Angle(tracking_link_frame[2], &angle_x10))
                {
                    tracking_link_state = tracking_link_frame[2];
                    tracking_link_angle_x10 = angle_x10;
                    tracking_link_last_rx_ms = HAL_GetTick();
                    tracking_link_valid = 1U;
                }
                else
                {
                    tracking_link_valid = 0U;
                }
            }
            tracking_link_index = 0U;
        }
    }

    (void)HAL_UART_Receive_IT(&huart2, &tracking_link_rx_byte, 1U);
}

void Tracking_Link_UART_Error_Callback(void)
{
    tracking_link_index = 0U;
    (void)HAL_UART_Receive_IT(&huart2, &tracking_link_rx_byte, 1U);
}

bool Tracking_Link_Is_Online(void)
{
    return tracking_link_valid != 0U && (uint32_t)(HAL_GetTick() - tracking_link_last_rx_ms) <= TRACKING_LINK_TIMEOUT_MS;
}

bool Tracking_Link_Get_Angle(int16_t *angle_x10)
{
    uint32_t primask;

    if (angle_x10 == NULL || !Tracking_Link_Is_Online())
    {
        return false;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *angle_x10 = tracking_link_angle_x10;
    __set_PRIMASK(primask);
    return true;
}

bool Tracking_Link_Get_State(uint8_t *state)
{
    if (state == NULL || !Tracking_Link_Is_Online())
    {
        return false;
    }
    *state = tracking_link_state;
    return true;
}
