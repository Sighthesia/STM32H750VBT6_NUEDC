#include "g030_i2c_link.h"

#include "i2c.h"
#include <string.h>

#define G030_I2C_ADDRESS        (0x12U << 1U)
#define I2C_TRANSPORT_SIZE      64U
#define LWPKT_START_BYTE        0xAAU
#define LWPKT_STOP_BYTE         0x55U
#define LWPKT_CRC_POLYNOMIAL    0x8CU
#define H750_PROTOCOL_ADDRESS   0x11U
#define G030_PROTOCOL_ADDRESS   0x12U
#define CMD_SET_LED_STATE       0x01U
#define RESPONSE_RETRY_MS       10U
#define COMMAND_RETRY_MS        500U

g030_i2c_link_state_t g030_i2c_link_state;

static uint8_t command_transport[I2C_TRANSPORT_SIZE];
static uint8_t response_transport[I2C_TRANSPORT_SIZE];
static uint32_t next_action_tick;
static uint32_t command_sent_tick;
static uint8_t waiting_response;

static uint8_t crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;
    for (uint8_t i = 0U; i < length; ++i)
    {
        uint8_t value = data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            uint8_t mix = (uint8_t)((crc ^ value) & 0x01U);
            crc >>= 1U;
            if (mix != 0U)
            {
                crc ^= LWPKT_CRC_POLYNOMIAL;
            }
            value >>= 1U;
        }
    }
    return crc;
}

static uint8_t build_lwpkt_frame(uint8_t to, uint8_t command, const uint8_t *data,
                                 uint8_t data_length, uint8_t *transport)
{
    if (data == NULL || data_length == 0U || data_length > (I2C_TRANSPORT_SIZE - 8U))
    {
        return 0U;
    }

    uint8_t frame_length = (uint8_t)(data_length + 7U);
    uint8_t *frame = &transport[1];
    frame[0] = LWPKT_START_BYTE;
    frame[1] = H750_PROTOCOL_ADDRESS;
    frame[2] = to;
    frame[3] = command;
    frame[4] = data_length;
    memcpy(&frame[5], data, data_length);
    frame[5U + data_length] = crc8(&frame[1], (uint8_t)(4U + data_length));
    frame[6U + data_length] = LWPKT_STOP_BYTE;
    transport[0] = frame_length;
    return frame_length;
}

static uint8_t parse_response(void)
{
    const uint8_t length = response_transport[0];
    const uint8_t *frame = &response_transport[1];
    if (length != 9U || frame[0] != LWPKT_START_BYTE || frame[1] != G030_PROTOCOL_ADDRESS ||
        frame[2] != H750_PROTOCOL_ADDRESS || frame[3] != CMD_SET_LED_STATE || frame[4] != 2U ||
        frame[8] != LWPKT_STOP_BYTE || frame[7] != crc8(&frame[1], 6U))
    {
        return 0U;
    }

    g030_i2c_link_state.status = frame[5];
    g030_i2c_link_state.led_blinking = frame[6];
    g030_i2c_link_state.response_valid = 1U;
    return 1U;
}

void G030_I2C_Link_Init(void)
{
    memset(&g030_i2c_link_state, 0, sizeof(g030_i2c_link_state));
    memset(command_transport, 0, sizeof(command_transport));
    const uint8_t led_enabled = 1U;
    build_lwpkt_frame(G030_PROTOCOL_ADDRESS, CMD_SET_LED_STATE, &led_enabled,
                      sizeof(led_enabled), command_transport);
    waiting_response = 0U;
    next_action_tick = HAL_GetTick() + 20U;
}

void G030_I2C_Link_Process(void)
{
    uint32_t now = HAL_GetTick();
    if (g030_i2c_link_state.response_valid)
    {
        return;
    }
    if ((int32_t)(now - next_action_tick) < 0)
    {
        return;
    }

    if (!waiting_response)
    {
        if (HAL_I2C_Master_Transmit(&hi2c2, G030_I2C_ADDRESS, command_transport,
                                    sizeof(command_transport), 20U) == HAL_OK)
        {
            waiting_response = 1U;
            command_sent_tick = now;
            next_action_tick = now + RESPONSE_RETRY_MS;
        }
        else
        {
            ++g030_i2c_link_state.tx_error_count;
            next_action_tick = now + COMMAND_RETRY_MS;
        }
        return;
    }

    memset(response_transport, 0, sizeof(response_transport));
    if (HAL_I2C_Master_Receive(&hi2c2, G030_I2C_ADDRESS, response_transport,
                               sizeof(response_transport), 20U) == HAL_OK && parse_response())
    {
        waiting_response = 0U;
        return;
    }

    ++g030_i2c_link_state.rx_error_count;
    if ((now - command_sent_tick) >= COMMAND_RETRY_MS)
    {
        waiting_response = 0U;
    }
    next_action_tick = now + RESPONSE_RETRY_MS;
}
