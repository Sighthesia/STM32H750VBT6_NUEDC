/**
 * @File Name: u8g2_Custom_HW_I2C_STM32_HAL.h
 * @brief u8g2硬件I2C初始化配置文件
 * @Author : Sighthesia
 * @Version : 1.2
 * @Creat Date : 2025-04-25
 * ---------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes : 添加注释;添加初始化函数;添加对c++编译的支持
 * @Modifi Date : 2025-04-27
 * ****************************************************************************/
#ifndef __U8G2_CUSTOM_HW_I2C_STM32_HAL_H__
#define __U8G2_CUSTOM_HW_I2C_STM32_HAL_H__

#include "i2c.h"
#include "u8g2.h"
#include "u8x8.h"

#ifdef __cplusplus
extern "C"
{
#endif
    uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr);
    uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
    void u8g2_Init(u8g2_t *u8g2);
#ifdef __cplusplus
}
#endif
#endif /* __U8G2_CUSTOM_HW_I2C_STM32_HAL_H__ */