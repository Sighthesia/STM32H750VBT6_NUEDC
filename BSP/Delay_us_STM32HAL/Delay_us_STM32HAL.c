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

void delay_us(uint32_t udelay)
{
    uint32_t startval, tickn, delays, wait;

    startval = SysTick->VAL;
    tickn = HAL_GetTick();
    // sysc = 72000;  //SystemCoreClock / (1000U / uwTickFreq);
    delays = udelay * SYSTEM_CLOCK_MHZ_FREQUENCY; // sysc / 1000 * udelay;
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
}