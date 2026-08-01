#include "Board_Spi_Link.h"

#include <stdbool.h>
#include <stdint.h>

#include "Car_Lap_Task.h"
#include "spi.h"

#define FRAME_SYNC          0xA5U
#define FRAME_ACK           0x5AU
#define FRAME_SIZE          5U
#define COMMAND_SELECT      0x01U
#define COMMAND_START       0x02U
#define COMMAND_STOP        0x03U

static uint8_t rx_frame[FRAME_SIZE];
static uint8_t tx_frame[FRAME_SIZE];
static volatile uint8_t frame_ready;
static uint8_t selected_test;

static uint8_t frame_checksum(const uint8_t *frame)
{
    return (uint8_t)(frame[0] ^ frame[1] ^ frame[2] ^ frame[3]);
}

static bool command_is_valid(const uint8_t *frame)
{
    return frame[0] == FRAME_SYNC && frame[4] == frame_checksum(frame) &&
           frame[2] >= 1U && frame[2] <= 6U &&
           (frame[1] == COMMAND_SELECT || frame[1] == COMMAND_START || frame[1] == COMMAND_STOP);
}

static bool dispatch_command(uint8_t command, uint8_t test_id)
{
    if (command == COMMAND_SELECT)
    {
        selected_test = test_id;
        return true;
    }
    if (command == COMMAND_STOP) return Car_Lap_Task_Stop();
    if (command == COMMAND_START && selected_test == test_id)
    {
        /* Only vehicle-driving tests enter the existing lap task. */
        if (test_id >= 4U) return Car_Lap_Task_Start();
        return true;
    }
    return false;
}

void Board_Spi_Link_Init(void)
{
    tx_frame[0] = 0U;
    frame_ready = 0U;
    selected_test = 0U;
    (void)HAL_SPI_TransmitReceive_IT(&hspi3, tx_frame, rx_frame, FRAME_SIZE);
}

void Board_Spi_Link_Process(void)
{
    uint8_t command;
    uint8_t test_id;
    uint8_t sequence;

    if (frame_ready == 0U) return;
    __disable_irq();
    frame_ready = 0U;
    command = rx_frame[1];
    test_id = rx_frame[2];
    sequence = rx_frame[3];
    __enable_irq();

    tx_frame[0] = 0U;
    if (command_is_valid(rx_frame) && dispatch_command(command, test_id))
    {
        tx_frame[0] = FRAME_ACK;
        tx_frame[1] = command;
        tx_frame[2] = test_id;
        tx_frame[3] = sequence;
        tx_frame[4] = frame_checksum(tx_frame);
    }
    (void)HAL_SPI_TransmitReceive_IT(&hspi3, tx_frame, rx_frame, FRAME_SIZE);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3) frame_ready = 1U;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3)
    {
        (void)HAL_SPI_Abort_IT(hspi);
        (void)HAL_SPI_TransmitReceive_IT(&hspi3, tx_frame, rx_frame, FRAME_SIZE);
    }
}
