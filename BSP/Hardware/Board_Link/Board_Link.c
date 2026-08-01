#include "Board_Link.h"

#include "App_Role.h"
#define BOARD_LINK_CODE_HOLD_MS  100U

#define BOARD_LINK_PIN_B0_PORT   GPIOB
#define BOARD_LINK_PIN_B0        GPIO_PIN_3
#define BOARD_LINK_PIN_B1_PORT   GPIOC
#define BOARD_LINK_PIN_B1        GPIO_PIN_11
#define BOARD_LINK_PIN_B2_PORT   GPIOC
#define BOARD_LINK_PIN_B2        GPIO_PIN_12
#define BOARD_LINK_PIN_B3_PORT   GPIOC
#define BOARD_LINK_PIN_B3        GPIO_PIN_3

#define BOARD_LINK_COMMAND_STOP  7U

static uint8_t command_code;
static uint8_t command_test;
static uint32_t command_started_ms;
static board_link_command_status_t command_status;

static void Board_Link_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Pin = BOARD_LINK_PIN_B0;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_LINK_PIN_B0_PORT, &gpio);
    gpio.Pin = BOARD_LINK_PIN_B1 | BOARD_LINK_PIN_B2 | BOARD_LINK_PIN_B3;
    HAL_GPIO_Init(GPIOC, &gpio);
    HAL_GPIO_WritePin(BOARD_LINK_PIN_B0_PORT, BOARD_LINK_PIN_B0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, BOARD_LINK_PIN_B1 | BOARD_LINK_PIN_B2 | BOARD_LINK_PIN_B3, GPIO_PIN_RESET);
}

static void Board_Link_Write_Code(uint8_t code)
{
    HAL_GPIO_WritePin(BOARD_LINK_PIN_B0_PORT, BOARD_LINK_PIN_B0,
                      (code & 0x01U) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_LINK_PIN_B1_PORT, BOARD_LINK_PIN_B1,
                      (code & 0x02U) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_LINK_PIN_B2_PORT, BOARD_LINK_PIN_B2,
                      (code & 0x04U) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_LINK_PIN_B3_PORT, BOARD_LINK_PIN_B3,
                      (code & 0x08U) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Board_Link_Init(void)
{
    Board_Link_GPIO_Init();
    command_code = 0U;
    command_test = 0U;
    command_started_ms = 0U;
    command_status = BOARD_LINK_COMMAND_IDLE;
}

void Board_Link_Process(void)
{
    if (command_code != 0U && (uint32_t)(HAL_GetTick() - command_started_ms) >= BOARD_LINK_CODE_HOLD_MS)
    {
        Board_Link_Write_Code(0U);
        command_code = 0U;
        command_status = BOARD_LINK_COMMAND_OK;
    }
}

bool Board_Link_Is_Online(void) { return true; }
uint8_t Board_Link_Get_Peer_Role(void) { return APP_ROLE_CAR; }
uint8_t Board_Link_Get_Peer_Gray(void) { return 0U; }

void Board_Link_Select_Test(uint8_t test_id)
{
    if (test_id >= 1U && test_id <= 6U) command_test = test_id;
}

void Board_Link_Start_Test(void)
{
    if (command_test < 1U || command_test > 6U) return;
    command_code = command_test;
    command_started_ms = HAL_GetTick();
    command_status = BOARD_LINK_COMMAND_WAITING;
    Board_Link_Write_Code(command_code);
}

void Board_Link_Stop_Test(void)
{
    command_code = BOARD_LINK_COMMAND_STOP;
    command_started_ms = HAL_GetTick();
    command_status = BOARD_LINK_COMMAND_WAITING;
    Board_Link_Write_Code(command_code);
}

board_link_command_status_t Board_Link_Get_Command_Status(void) { return command_status; }
uint8_t Board_Link_Get_Last_Command(void) { return command_code; }
uint8_t Board_Link_Get_Last_Ack(void) { return command_status == BOARD_LINK_COMMAND_OK ? 1U : 0U; }
uint8_t Board_Link_Get_Peer_Diagnostic(void) { return 0U; }
uint16_t Board_Link_Get_Valid_Frame_Count(void) { return 0U; }
uint32_t Board_Link_Get_Last_Spi_Error(void) { return 0U; }
