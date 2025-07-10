/** ***************************************************************************
 * @File Name: Servo_Yahboom_HAL.c
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
#include "Servo_Yahboom_HAL.h"

/* --------------------------- 此处修改回调函数所用的特定平台的函数 --------------------------- */

void GPIO_SetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

servo_yahboom_hal_t servo_yahboom_hal = {
    .GPIO_SetBits = GPIO_SetBits,
    .GPIO_ResetBits = GPIO_ResetBits};
