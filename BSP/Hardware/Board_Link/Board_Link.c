#include "Board_Link.h"

#include "App_Role.h"
#include "Gray_Sensor.h"
#include "Tracking_Link.h"
#include "ZDT_Stepper.h"

#include <string.h>

#define BOARD_LINK_HEAD_0        0xA5U
#define BOARD_LINK_HEAD_1        0x5AU
#define BOARD_LINK_VERSION       1U
#define BOARD_LINK_FRAME_SIZE    9U
#define BOARD_LINK_PERIOD_MS     50U
#define BOARD_LINK_TIMEOUT_MS    500U
#define BOARD_LINK_SPI_TIMEOUT_MS 5U
#define BOARD_LINK_NSS_PORT      GPIOC
#define BOARD_LINK_NSS_PIN       GPIO_PIN_3

static uint8_t rx_frame[BOARD_LINK_FRAME_SIZE];
static uint8_t tx_frame[BOARD_LINK_FRAME_SIZE];
static volatile uint8_t slave_transfer_active;
static volatile uint32_t slave_transfer_started_ms;
static volatile uint8_t peer_role = 0xFFU;
static volatile uint8_t peer_gray;
static volatile uint8_t peer_flags;
static volatile uint32_t last_rx_ms;
static uint32_t last_tx_ms;
static uint32_t initialized_ms;
static uint8_t sequence;
static uint8_t was_online;
static uint8_t offline_stop_sent;
static uint8_t command_pending;
static uint8_t command_start_requested;
static uint8_t command_test;
static uint32_t command_started_ms;
static board_link_command_status_t command_status;
static volatile uint8_t last_ack;
static volatile uint16_t valid_frame_count;
static volatile uint32_t last_spi_error;
SPI_HandleTypeDef hspi3;

#define BOARD_LINK_COMMAND_SELECT  0x10U
#define BOARD_LINK_COMMAND_START   0x20U
#define BOARD_LINK_COMMAND_STOP    0x30U

static void Board_Link_SPI_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = BOARD_LINK_NSS_PIN;
    gpio.Mode = App_Role_Get() == APP_ROLE_CAR ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BOARD_LINK_NSS_PORT, &gpio);
    if (App_Role_Get() == APP_ROLE_CAR)
    {
        HAL_GPIO_WritePin(BOARD_LINK_NSS_PORT, BOARD_LINK_NSS_PIN, GPIO_PIN_SET);
    }

    hspi3.Instance = SPI3;
    hspi3.Init.Mode = App_Role_Get() == APP_ROLE_CAR ? SPI_MODE_MASTER : SPI_MODE_SLAVE;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    /* PC3 is not an SPI3_NSS alternate-function pin; the slave remains selected in software. */
    hspi3.Init.NSS = SPI_NSS_SOFT;
    /* Keep the car-master clock slow enough for the interrupt-driven arm slave. */
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 0U;
    hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(SPI3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);
}

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

static void Board_Link_Build_Frame(void)
{
    tx_frame[0] = BOARD_LINK_HEAD_0;
    tx_frame[1] = BOARD_LINK_HEAD_1;
    tx_frame[2] = BOARD_LINK_VERSION;
    tx_frame[3] = (uint8_t)App_Role_Get();
    tx_frame[4] = sequence++;
    tx_frame[5] = App_Role_Get() == APP_ROLE_CAR ? Gray_Sensor_Read() : 0U;
    tx_frame[6] = App_Role_Get() == APP_ROLE_ARM && Tracking_Link_Is_Online() ? 0x01U : 0U;
    tx_frame[7] = App_Role_Get() == APP_ROLE_ARM ? command_pending : 0U;
    tx_frame[8] = Board_Link_Crc8(tx_frame, BOARD_LINK_FRAME_SIZE - 1U);
}

static void Board_Link_Parse_Frame(void)
{
    if (rx_frame[0] == BOARD_LINK_HEAD_0 && rx_frame[1] == BOARD_LINK_HEAD_1 &&
        rx_frame[2] == BOARD_LINK_VERSION && rx_frame[3] != (uint8_t)App_Role_Get() &&
        Board_Link_Crc8(rx_frame, BOARD_LINK_FRAME_SIZE - 1U) == rx_frame[8])
    {
        peer_role = rx_frame[3];
        peer_gray = rx_frame[5];
        peer_flags = rx_frame[6];
        last_ack = rx_frame[7];
        last_rx_ms = HAL_GetTick();
        valid_frame_count++;
        if (App_Role_Get() == APP_ROLE_ARM && command_pending != 0U && rx_frame[7] == command_pending)
        {
            if ((command_pending & 0xF0U) == BOARD_LINK_COMMAND_SELECT && command_start_requested != 0U)
            {
                command_start_requested = 0U;
                command_pending = (uint8_t)(BOARD_LINK_COMMAND_START | command_test);
                command_started_ms = HAL_GetTick();
            }
            else
            {
                command_pending = 0U;
                command_status = BOARD_LINK_COMMAND_OK;
            }
        }
        else if (App_Role_Get() == APP_ROLE_ARM && command_pending != 0U &&
                 rx_frame[7] == (uint8_t)(command_pending | 0x80U))
        {
            command_pending = 0U;
            command_start_requested = 0U;
            command_status = BOARD_LINK_COMMAND_REJECTED;
        }
    }
}

static void Board_Link_Arm_Slave(void)
{
    Board_Link_Build_Frame();
    slave_transfer_active = HAL_SPI_TransmitReceive_IT(&hspi3, tx_frame, rx_frame,
                                                       BOARD_LINK_FRAME_SIZE) == HAL_OK;
    if (slave_transfer_active != 0U)
    {
        slave_transfer_started_ms = HAL_GetTick();
    }
}

void Board_Link_Init(void)
{
    Board_Link_SPI_Init();
    memset(rx_frame, 0, sizeof(rx_frame));
    memset(tx_frame, 0, sizeof(tx_frame));
    slave_transfer_active = 0U;
    slave_transfer_started_ms = 0U;
    peer_role = 0xFFU;
    peer_gray = 0U;
    peer_flags = 0U;
    last_rx_ms = 0U;
    last_tx_ms = HAL_GetTick() - BOARD_LINK_PERIOD_MS;
    initialized_ms = HAL_GetTick();
    sequence = 0U;
    was_online = 0U;
    offline_stop_sent = 0U;
    command_pending = 0U;
    command_start_requested = 0U;
    command_test = 0U;
    command_status = BOARD_LINK_COMMAND_IDLE;
    last_ack = 0U;
    valid_frame_count = 0U;
    last_spi_error = HAL_SPI_ERROR_NONE;
    if (App_Role_Get() == APP_ROLE_ARM) Board_Link_Arm_Slave();
}

void Board_Link_Process(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t online = Board_Link_Is_Online() ? 1U : 0U;

    if (App_Role_Get() == APP_ROLE_ARM && command_pending != 0U &&
        (uint32_t)(now - command_started_ms) > BOARD_LINK_TIMEOUT_MS)
    {
        command_pending = 0U;
        command_start_requested = 0U;
        command_status = BOARD_LINK_COMMAND_ERROR;
    }

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

    if (App_Role_Get() == APP_ROLE_ARM)
    {
        return;
    }
    if ((uint32_t)(now - last_tx_ms) < BOARD_LINK_PERIOD_MS)
    {
        return;
    }

    Board_Link_Build_Frame();
    last_tx_ms = now;
    HAL_GPIO_WritePin(BOARD_LINK_NSS_PORT, BOARD_LINK_NSS_PIN, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi3, tx_frame, rx_frame,
                                                       BOARD_LINK_FRAME_SIZE,
                                                       BOARD_LINK_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(BOARD_LINK_NSS_PORT, BOARD_LINK_NSS_PIN, GPIO_PIN_SET);
    if (result == HAL_OK)
    {
        Board_Link_Parse_Frame();
    }
}

void Board_Link_NSS_Falling_Callback(void)
{
    /* The slave transfer is pre-armed, so this GPIO edge is informational only. */
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3 && App_Role_Get() == APP_ROLE_ARM)
    {
        slave_transfer_active = 0U;
        Board_Link_Parse_Frame();
        Board_Link_Arm_Slave();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3)
    {
        last_spi_error = HAL_SPI_GetError(hspi);
        slave_transfer_active = 0U;
        if (App_Role_Get() == APP_ROLE_ARM) Board_Link_Arm_Slave();
    }
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

void Board_Link_Select_Test(uint8_t test_id)
{
    if (App_Role_Get() != APP_ROLE_ARM || test_id < 1U || test_id > 6U) return;
    command_test = test_id;
    command_pending = (uint8_t)(BOARD_LINK_COMMAND_SELECT | test_id);
    command_start_requested = 0U;
    command_started_ms = HAL_GetTick();
    command_status = BOARD_LINK_COMMAND_WAITING;
}

void Board_Link_Start_Test(void)
{
    if (App_Role_Get() == APP_ROLE_ARM && command_test >= 1U && command_test <= 6U)
    {
        command_start_requested = 1U;
    }
}

void Board_Link_Stop_Test(void)
{
    if (App_Role_Get() == APP_ROLE_ARM && command_test >= 1U && command_test <= 6U)
    {
        command_pending = (uint8_t)(BOARD_LINK_COMMAND_STOP | command_test);
        command_start_requested = 0U;
        command_started_ms = HAL_GetTick();
        command_status = BOARD_LINK_COMMAND_WAITING;
    }
}

board_link_command_status_t Board_Link_Get_Command_Status(void)
{
    return command_status;
}

uint8_t Board_Link_Get_Last_Command(void) { return command_pending; }
uint8_t Board_Link_Get_Last_Ack(void) { return last_ack; }
uint8_t Board_Link_Get_Peer_Diagnostic(void) { return peer_flags; }
uint16_t Board_Link_Get_Valid_Frame_Count(void) { return valid_frame_count; }
uint32_t Board_Link_Get_Last_Spi_Error(void) { return last_spi_error; }
