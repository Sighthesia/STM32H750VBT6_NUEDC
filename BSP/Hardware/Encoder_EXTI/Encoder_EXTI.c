/** ***************************************************************************
 * @File Name: Encoder_EXTI.c
 * @brief 编码器外部中断驱动实现
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-15
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */

#include "Encoder_EXTI.h"

/* --------------------------------- 私有变量 ----------------------------------- */
volatile int16_t encoder_count = 0;
static volatile encoder_state_t encoder_state = ENCODER_NO_CHANGE;
static uint8_t last_state;
static int8_t phase_accumulator;

static void encoder_sample(void)
{
    static const int8_t transition[16] =
    {
        0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0
    };
    uint8_t current_state = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET ? 1U : 0U) |
                            (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET ? 2U : 0U);
    int8_t delta = transition[(last_state << 2) | current_state];

    phase_accumulator = (int8_t)(phase_accumulator + delta);
    if (phase_accumulator >= 4)
    {
        encoder_count++;
        encoder_state = ENCODER_INCREASE;
        phase_accumulator = 0;
    }
    else if (phase_accumulator <= -4)
    {
        encoder_count--;
        encoder_state = ENCODER_DECREASE;
        phase_accumulator = 0;
    }
    last_state = current_state;
}

/* --------------------------------- 公共函数 ----------------------------------- */
/** ***************************************************************************
 * @brief 编码器一键初始化函数
 */
void Encoder_Init(void)
{
    last_state = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET ? 1U : 0U) |
                 (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET ? 2U : 0U);
    phase_accumulator = 0;
}

void Encoder_Process(void)
{
    encoder_sample();
}

/** ***************************************************************************
 * @brief 获取编码器状态变化
 * @retval encoder_state_t 编码器状态
 */
encoder_state_t Encoder_Get_State_Change(void)
{
    encoder_state_t current_state = encoder_state;
    encoder_state = ENCODER_NO_CHANGE; // 读取后清除状态
    return current_state;
}

/** ***************************************************************************
 * @brief 获取编码器当前计数值
 * @retval float 编码器计数值
 */
float Encoder_Get_Count(void)
{
    return encoder_count;
}

/** ***************************************************************************
 * @brief 设置编码器计数值
 * @param count 要设置的计数值
 */
void Encoder_Set_Count(int16_t count)
{
    encoder_count = count;
}

/** ***************************************************************************
 * @brief 检查编码器是否增加
 * @retval uint8_t 1-增加，0-未增加
 */
uint8_t Encoder_Is_Increase(void)
{
    encoder_state_t current_state = encoder_state;
    if (current_state == ENCODER_INCREASE)
    {
        encoder_state = ENCODER_NO_CHANGE; // 读取后清除状态
        return 1;
    }
    return 0;
}

/** ***************************************************************************
 * @brief 检查编码器是否减少
 * @retval uint8_t 1-减少，0-未减少
 */
uint8_t Encoder_Is_Decrease(void)
{
    encoder_state_t current_state = encoder_state;
    if (current_state == ENCODER_DECREASE)
    {
        encoder_state = ENCODER_NO_CHANGE; // 读取后清除状态
        return 1;
    }
    return 0;
}
