/** ***************************************************************************
 * @File Name: Servo_Yahboom_HAL.h
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-08
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#ifndef __SERVO_YAHBOOM_HAL_H
#define __SERVO_YAHBOOM_HAL_H

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

typedef struct
{
    void (*GPIO_SetBits)(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
    void (*GPIO_ResetBits)(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
} servo_yahboom_hal_t;

extern servo_yahboom_hal_t servo_yahboom_hal; // 硬件抽象层回调函数声明

#endif /* __SERVO_YAHBOOM_HAL_H */