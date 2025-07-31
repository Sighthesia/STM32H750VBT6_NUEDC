/** ***************************************************************************
 * @File Name: Delay_us_STM32HAL.c
 * @brief 利用 STM32HAL 库的 SysTick 定时器实现微秒级延时
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-08
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#include "Delay_us_STM32HAL.h"

/** ***************************************************************************
 * @brief 微秒延时函数
 * @param  us: 微秒数
 */
void Delay_us(uint32_t us)
{
#ifdef USE_SYSTICK_TIME_SOURCE
    uint32_t startval, tickn, delays, wait;

    startval = SysTick->VAL;
    tickn = HAL_GetTick();
    // sysc = 72000;  //SystemCoreClock / (1000U / uwTickFreq);
    delays = us * SYSTEM_CLOCK_MHZ_FREQUENCY; // sysc / 1000 * udelay;
    if (delays > startval)
    {
        while (HAL_GetTick() == tickn)
        {
        }
        wait = SYSTEM_CLOCK_MHZ_FREQUENCY * 1000 + startval - delays;
        while (wait < SysTick->VAL)
        {
        }
    }
    else
    {
        wait = startval - delays;
        while (wait < SysTick->VAL && HAL_GetTick() == tickn)
        {
        }
    }
#endif
#ifdef USE_TIM_TIME_SOURCE
    // 适配1μs触发的定时器
    uint32_t start_count = __HAL_TIM_GET_COUNTER(&TIM_HANDLE);
    uint32_t target_count = start_count + us; // 1μs = 1 tick
    uint32_t max_count = __HAL_TIM_GET_AUTORELOAD(&TIM_HANDLE);

    // 处理计数器溢出情况
    if (target_count > max_count)
    {
        target_count = target_count - max_count - 1;
        // 等待计数器溢出重置
        while (__HAL_TIM_GET_COUNTER(&TIM_HANDLE) >= start_count)
        {
            // 防止死锁：如果延时时间太长，分段处理
            if (us > max_count)
            {
                break;
            }
        }
    }

    // 等待到达目标计数值
    while (__HAL_TIM_GET_COUNTER(&TIM_HANDLE) < target_count)
    {
    }
#endif
}

#ifdef USE_TIM_TIME_SOURCE
/**
 * @brief 开启定时器（先清空标志位和计数器）
 * @param 无
 * @retval 无
 */
void Delay_us_Timer_Start(void)
{
    // 先停止定时器
    HAL_TIM_Base_Stop(&TIM_HANDLE);

    // 清除所有标志位
    __HAL_TIM_CLEAR_FLAG(&TIM_HANDLE, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&TIM_HANDLE, TIM_FLAG_CC1);
    __HAL_TIM_CLEAR_FLAG(&TIM_HANDLE, TIM_FLAG_CC2);
    __HAL_TIM_CLEAR_FLAG(&TIM_HANDLE, TIM_FLAG_CC3);
    __HAL_TIM_CLEAR_FLAG(&TIM_HANDLE, TIM_FLAG_CC4);

    // 重置计数器到0
    __HAL_TIM_SET_COUNTER(&TIM_HANDLE, 0);

    // 启动定时器
    HAL_TIM_Base_Start(&TIM_HANDLE);
}

/**
 * @brief 关闭定时器
 * @param 无
 * @retval 无
 */
void Delay_us_Timer_Stop(void)
{
    // 停止定时器
    HAL_TIM_Base_Stop(&TIM_HANDLE);

    // 清除所有标志位
    __HAL_TIM_CLEAR_FLAG(&TIM_HANDLE, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&TIM_HANDLE, TIM_FLAG_CC1);
    __HAL_TIM_CLEAR_FLAG(&TIM_HANDLE, TIM_FLAG_CC2);
    __HAL_TIM_CLEAR_FLAG(&TIM_HANDLE, TIM_FLAG_CC3);
    __HAL_TIM_CLEAR_FLAG(&TIM_HANDLE, TIM_FLAG_CC4);

    // 重置计数器到0
    __HAL_TIM_SET_COUNTER(&TIM_HANDLE, 0);
}

/**
 * @brief 检查定时器是否在运行
 * @param 无
 * @retval 1: 运行中, 0: 已停止
 */
uint8_t Delay_us_Timer_IsRunning(void)
{
    // 检查定时器使能位
    return (TIM_INSTANCE->CR1 & TIM_CR1_CEN) ? 1 : 0;
}

/**
 * @brief 重置定时器计数器（不停止定时器）
 * @param 无
 * @retval 无
 */
void Delay_Timer_Reset(void)
{
    // 重置计数器到0
    __HAL_TIM_SET_COUNTER(&TIM_HANDLE, 0);

    // 清除更新标志位
    __HAL_TIM_CLEAR_FLAG(&TIM_HANDLE, TIM_FLAG_UPDATE);
}
#endif
