#include "Board_Link.h"

#include "App_Role.h"
#include "Car_Lap_Task.h"
#include "Car_Motor_USART.h"

#define BOARD_LINK_SAMPLE_MS       5U
#define BOARD_LINK_STABLE_MS       10U
#define BOARD_LINK_COMMAND_STOP    7U

#define BOARD_LINK_PIN_B0_PORT     GPIOB
#define BOARD_LINK_PIN_B0          GPIO_PIN_3
#define BOARD_LINK_PIN_B1_PORT     GPIOC
#define BOARD_LINK_PIN_B1          GPIO_PIN_11
#define BOARD_LINK_PIN_B2_PORT     GPIOC
#define BOARD_LINK_PIN_B2          GPIO_PIN_12
#define BOARD_LINK_FINISH_PORT     GPIOC
#define BOARD_LINK_FINISH_PIN      GPIO_PIN_3

static uint8_t sampled_code;
static uint8_t stable_code;
static uint8_t handled_code;
static uint32_t sample_changed_ms;
static uint32_t last_sample_ms;
static uint8_t active_test_code;

static void Board_Link_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = BOARD_LINK_PIN_B0;
    HAL_GPIO_Init(BOARD_LINK_PIN_B0_PORT, &gpio);
    gpio.Pin = BOARD_LINK_PIN_B1 | BOARD_LINK_PIN_B2;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = BOARD_LINK_FINISH_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BOARD_LINK_FINISH_PORT, &gpio);
    HAL_GPIO_WritePin(BOARD_LINK_FINISH_PORT, BOARD_LINK_FINISH_PIN, GPIO_PIN_RESET);
}

static uint8_t Board_Link_Read_Code(void)
{
    uint8_t code = 0U;

    code |= HAL_GPIO_ReadPin(BOARD_LINK_PIN_B0_PORT, BOARD_LINK_PIN_B0) == GPIO_PIN_SET ? 0x01U : 0U;
    code |= HAL_GPIO_ReadPin(BOARD_LINK_PIN_B1_PORT, BOARD_LINK_PIN_B1) == GPIO_PIN_SET ? 0x02U : 0U;
    code |= HAL_GPIO_ReadPin(BOARD_LINK_PIN_B2_PORT, BOARD_LINK_PIN_B2) == GPIO_PIN_SET ? 0x04U : 0U;
    return code;
}

static void Board_Link_Handle_Code(uint8_t code)
{
    if (code == BOARD_LINK_COMMAND_STOP)
    {
        active_test_code = 0U;
        HAL_GPIO_WritePin(BOARD_LINK_FINISH_PORT, BOARD_LINK_FINISH_PIN, GPIO_PIN_RESET);
        (void)Car_Lap_Task_Stop();
        (void)Car_Motor_Set_Speed(0, 0, 0, 0);
    }
    else if (code == 2U || (code >= 4U && code <= 6U))
    {
        active_test_code = code;
        HAL_GPIO_WritePin(BOARD_LINK_FINISH_PORT, BOARD_LINK_FINISH_PIN, GPIO_PIN_RESET);
        if (!Car_Lap_Task_Start_Test(code)) active_test_code = 0U;
    }
    else
    {
        active_test_code = 0U;
        HAL_GPIO_WritePin(BOARD_LINK_FINISH_PORT, BOARD_LINK_FINISH_PIN, GPIO_PIN_RESET);
        (void)Car_Lap_Task_Stop();
        (void)Car_Motor_Set_Speed(0, 0, 0, 0);
    }
}

void Board_Link_Init(void)
{
    Board_Link_GPIO_Init();
    sampled_code = Board_Link_Read_Code();
    stable_code = 0U;
    handled_code = 0U;
    sample_changed_ms = HAL_GetTick();
    last_sample_ms = sample_changed_ms;
    active_test_code = 0U;
}

void Board_Link_Process(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t code;

    if (active_test_code == 2U || active_test_code == 4U)
    {
        car_lap_status_t status;

        Car_Lap_Task_Get_Status(&status);
        if (status.state == CAR_LAP_STATE_FINISHED)
        {
            HAL_GPIO_WritePin(BOARD_LINK_FINISH_PORT, BOARD_LINK_FINISH_PIN, GPIO_PIN_SET);
        }
    }

    if ((uint32_t)(now - last_sample_ms) < BOARD_LINK_SAMPLE_MS) return;
    last_sample_ms = now;
    code = Board_Link_Read_Code();
    if (code != sampled_code)
    {
        sampled_code = code;
        sample_changed_ms = now;
        return;
    }
    if ((uint32_t)(now - sample_changed_ms) < BOARD_LINK_STABLE_MS || stable_code == code) return;

    stable_code = code;
    if (stable_code == 0U)
    {
        handled_code = 0U;
        return;
    }
    if (stable_code != handled_code)
    {
        handled_code = stable_code;
        Board_Link_Handle_Code(stable_code);
    }

}

void Board_Link_NSS_Falling_Callback(void) {}
bool Board_Link_Is_Online(void) { return true; }
uint8_t Board_Link_Get_Peer_Role(void) { return 0U; }
uint8_t Board_Link_Get_Peer_Gray(void) { return 0U; }
