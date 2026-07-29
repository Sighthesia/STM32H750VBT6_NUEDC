#ifndef ZDT_STEPPER_HAL_H
#define ZDT_STEPPER_HAL_H

#include "ZDT_Stepper_Config.h"
#include "main.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief ZDT步进电机HAL回调结构体
     */
    typedef struct
    {
        HAL_StatusTypeDef (*Uart_Transmit)(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t size);
        void (*Delay_Ms)(uint32_t ms);
    } zdt_stepper_hal_t;

    // 全局HAL实例
    extern zdt_stepper_hal_t zdt_stepper_hal;

#ifdef __cplusplus
}
#endif

#endif /* ZDT_STEPPER_HAL_H */
/* 自定义部分结束 */
