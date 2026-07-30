#include "Tracking_Link.h"

#include "main.h"
#include "usart.h"

#define TRACKING_LINK_HEAD_0       0xA5U
#define TRACKING_LINK_HEAD_1       0x5AU
#define TRACKING_LINK_FRAME_SIZE   5U
#define TRACKING_LINK_TIMEOUT_MS   50U

static uint8_t tracking_link_rx_byte;
static uint8_t tracking_link_frame[TRACKING_LINK_FRAME_SIZE];
static uint8_t tracking_link_index;
static volatile tracking_link_sample_t tracking_link_sample;
static volatile uint8_t tracking_link_frame_valid;

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

static bool Tracking_Link_Copy_Sample(tracking_link_sample_t *sample)
{
    uint32_t primask;

    if (sample == NULL) return false;
    primask = __get_PRIMASK();
    __disable_irq();
    if (tracking_link_frame_valid == 0U ||
        (uint32_t)(HAL_GetTick() - tracking_link_sample.received_ms) > TRACKING_LINK_TIMEOUT_MS)
    {
        __set_PRIMASK(primask);
        return false;
    }
    sample->state = tracking_link_sample.state;
    sample->sequence = tracking_link_sample.sequence;
    sample->offset_x1000 = tracking_link_sample.offset_x1000;
    sample->angle_x10 = tracking_link_sample.angle_x10;
    sample->pattern = tracking_link_sample.pattern;
    sample->received_ms = tracking_link_sample.received_ms;
    __set_PRIMASK(primask);
    return true;
}

void Tracking_Link_Init(void)
{
    tracking_link_index = 0U;
    tracking_link_frame_valid = 0U;
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
            int16_t offset_x1000;
            int16_t angle_x10;

            tracking_link_frame[0] = TRACKING_LINK_HEAD_0;
            tracking_link_frame[1] = TRACKING_LINK_HEAD_1;
            if (Tracking_Link_Crc8(tracking_link_frame, TRACKING_LINK_FRAME_SIZE - 1U) == tracking_link_frame[4])
            {
                tracking_link_sample.state = tracking_link_frame[2];
                tracking_link_sample.sequence = tracking_link_frame[3];
                tracking_link_sample.pattern = Tracking_Line_Decode(
                    tracking_link_frame[2], &offset_x1000, &angle_x10);
                tracking_link_sample.offset_x1000 = offset_x1000;
                tracking_link_sample.angle_x10 = angle_x10;
                tracking_link_sample.received_ms = HAL_GetTick();
                tracking_link_frame_valid = 1U;
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
    tracking_link_sample_t sample;
    return Tracking_Link_Copy_Sample(&sample);
}

bool Tracking_Link_Get_Sample(tracking_link_sample_t *sample)
{
    return Tracking_Link_Copy_Sample(sample);
}

bool Tracking_Link_Get_Angle(int16_t *angle_x10)
{
    tracking_link_sample_t sample;

    if (angle_x10 == NULL || !Tracking_Link_Copy_Sample(&sample) || sample.pattern != TRACKING_PATTERN_LINE)
    {
        return false;
    }
    *angle_x10 = sample.angle_x10;
    return true;
}

bool Tracking_Link_Get_State(uint8_t *state)
{
    tracking_link_sample_t sample;

    if (state == NULL || !Tracking_Link_Copy_Sample(&sample))
    {
        return false;
    }
    *state = sample.state;
    return true;
}
