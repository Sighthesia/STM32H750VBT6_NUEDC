#include "Board_Link.h"

#include "App_Role.h"
#include "Gray_Sensor.h"
#include "Tracking_Link.h"
#include "ZDT_Stepper.h"
#include "usart.h"

#include <string.h>

#define BOARD_LINK_HEAD_0        0xA5U
#define BOARD_LINK_HEAD_1        0x5AU
#define BOARD_LINK_VERSION       1U
#define BOARD_LINK_FRAME_SIZE    9U
#define BOARD_LINK_PERIOD_MS     50U
#define BOARD_LINK_TIMEOUT_MS    250U

static UART_HandleTypeDef *link_uart;
static uint8_t rx_byte;
static uint8_t rx_frame[BOARD_LINK_FRAME_SIZE];
static uint8_t rx_index;
static uint8_t tx_frame[BOARD_LINK_FRAME_SIZE];
static volatile uint8_t tx_active;
static volatile uint8_t peer_role = 0xFFU;
static volatile uint8_t peer_gray;
static volatile uint8_t peer_flags;
static volatile uint32_t last_rx_ms;
static uint32_t last_tx_ms;
static uint32_t initialized_ms;
static uint8_t sequence;
static uint8_t was_online;
static uint8_t offline_stop_sent;

static uint8_t Board_Link_Crc8(const uint8_t *data, uint8_t length)
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

void Board_Link_Init(void)
{
    link_uart = App_Role_Get() == APP_ROLE_CAR ? &huart2 : &huart5;
    rx_index = 0U;
    tx_active = 0U;
    peer_role = 0xFFU;
    peer_gray = 0U;
    peer_flags = 0U;
    last_rx_ms = 0U;
    last_tx_ms = HAL_GetTick() - BOARD_LINK_PERIOD_MS;
    initialized_ms = HAL_GetTick();
    sequence = 0U;
    was_online = 0U;
    offline_stop_sent = 0U;
    (void)HAL_UART_Receive_IT(link_uart, &rx_byte, 1U);
}

void Board_Link_Process(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t online = Board_Link_Is_Online() ? 1U : 0U;

    if (online != 0U)
    {
        offline_stop_sent = 0U;
    }
    if (App_Role_Get() == APP_ROLE_ARM && online == 0U && offline_stop_sent == 0U &&
        (was_online != 0U || (uint32_t)(now - initialized_ms) >= BOARD_LINK_TIMEOUT_MS) &&
        zdt_stepper_motors[0] != NULL)
    {
        ZDT_Stepper_stop(zdt_stepper_motors[0], ZDT_STEPPER_SYNC_DISABLE);
        offline_stop_sent = 1U;
    }
    was_online = online;

    if (tx_active || (uint32_t)(now - last_tx_ms) < BOARD_LINK_PERIOD_MS)
    {
        return;
    }
    tx_frame[0] = BOARD_LINK_HEAD_0;
    tx_frame[1] = BOARD_LINK_HEAD_1;
    tx_frame[2] = BOARD_LINK_VERSION;
    tx_frame[3] = (uint8_t)App_Role_Get();
    tx_frame[4] = sequence++;
    tx_frame[5] = App_Role_Get() == APP_ROLE_CAR ? Gray_Sensor_Read() : 0U;
    tx_frame[6] = App_Role_Get() == APP_ROLE_ARM && Tracking_Link_Is_Online() ? 0x01U : 0U;
    tx_frame[7] = 0U;
    tx_frame[8] = Board_Link_Crc8(tx_frame, BOARD_LINK_FRAME_SIZE - 1U);
    tx_active = 1U;
    last_tx_ms = now;
    if (HAL_UART_Transmit_IT(link_uart, tx_frame, sizeof(tx_frame)) != HAL_OK)
    {
        tx_active = 0U;
    }
}

bool Board_Link_Is_UART(const UART_HandleTypeDef *huart)
{
    return huart != NULL && link_uart != NULL && huart->Instance == link_uart->Instance;
}

void Board_Link_UART_RxCplt_Callback(void)
{
    if (rx_index == 0U)
    {
        rx_index = rx_byte == BOARD_LINK_HEAD_0 ? 1U : 0U;
    }
    else if (rx_index == 1U)
    {
        rx_index = rx_byte == BOARD_LINK_HEAD_1 ? 2U : (rx_byte == BOARD_LINK_HEAD_0 ? 1U : 0U);
    }
    else
    {
        rx_frame[rx_index++] = rx_byte;
        if (rx_index == BOARD_LINK_FRAME_SIZE)
        {
            rx_frame[0] = BOARD_LINK_HEAD_0;
            rx_frame[1] = BOARD_LINK_HEAD_1;
            if (rx_frame[2] == BOARD_LINK_VERSION && rx_frame[3] != (uint8_t)App_Role_Get() &&
                Board_Link_Crc8(rx_frame, BOARD_LINK_FRAME_SIZE - 1U) == rx_frame[8])
            {
                peer_role = rx_frame[3];
                peer_gray = rx_frame[5];
                peer_flags = rx_frame[6];
                last_rx_ms = HAL_GetTick();
            }
            rx_index = 0U;
        }
    }
    (void)HAL_UART_Receive_IT(link_uart, &rx_byte, 1U);
}

void Board_Link_UART_TxCplt_Callback(void)
{
    tx_active = 0U;
}

void Board_Link_UART_Error_Callback(void)
{
    rx_index = 0U;
    tx_active = 0U;
    (void)HAL_UART_Receive_IT(link_uart, &rx_byte, 1U);
}

bool Board_Link_Is_Online(void)
{
    return last_rx_ms != 0U && (uint32_t)(HAL_GetTick() - last_rx_ms) <= BOARD_LINK_TIMEOUT_MS;
}

uint8_t Board_Link_Get_Peer_Role(void)
{
    return Board_Link_Is_Online() ? peer_role : 0xFFU;
}

uint8_t Board_Link_Get_Peer_Gray(void)
{
    return Board_Link_Is_Online() ? peer_gray : 0U;
}
