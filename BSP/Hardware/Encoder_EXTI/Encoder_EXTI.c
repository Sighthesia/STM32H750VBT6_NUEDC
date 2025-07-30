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
volatile int16_t encoder_count = 0;                                // 编码器计数变量
static volatile encoder_state_t encoder_state = ENCODER_NO_CHANGE; // 编码器状态变化标志

/* --------------------------------- 私有函数 ----------------------------------- */
/** ***************************************************************************
 * @brief GPIO外部中断回调函数
 * @param GPIO_Pin 触发中断的GPIO引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint8_t last_state = 0;
    static float increment = 0.1f; // 每次触发增加/减少的数值

    if (GPIO_Pin == GPIO_PIN_7 || GPIO_Pin == GPIO_PIN_8)
    {
        // 读取当前 A、B 相状态（0或1）
        uint8_t a = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
        uint8_t b = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7);

        // 构造当前状态（bit0=A, bit1=B）
        uint8_t current_state = (a << 0) | (b << 1);

        // 只在A相从低到高（上升沿）时判断方向并计数
        if ((last_state & 0x01) == 0 && (current_state & 0x01) == 1) // A相上升沿
        {
            if (b == 0) // B相为低电平时，逆时针
            {
                encoder_count -= increment;
                encoder_state = ENCODER_DECREASE;
            }
            else // B相为高电平时，顺时针
            {
                encoder_count += increment;
                encoder_state = ENCODER_INCREASE;
            }
        }

        // 更新上次状态
        last_state = current_state;
    }
}

/* --------------------------------- 公共函数 ----------------------------------- */
/** ***************************************************************************
 * @brief 编码器一键初始化函数
 */
void Encoder_Init(void)
{
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
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