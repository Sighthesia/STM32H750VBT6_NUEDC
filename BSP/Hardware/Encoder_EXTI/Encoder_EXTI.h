/** ***************************************************************************
 * @File Name: Encoder_EXTI.h
 * @brief 编码器外部中断驱动
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-15
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */

#ifndef __ENCODER_EXTI_H
#define __ENCODER_EXTI_H

#include "main.h"
#include "stdint.h"

/* --------------------------------- 编码器状态枚举 -------------------------------- */
typedef enum
{
    ENCODER_NO_CHANGE = 0, // 无变化
    ENCODER_INCREASE,      // 增加
    ENCODER_DECREASE       // 减少
} encoder_state_t;

/* --------------------------------- 外部变量声明 --------------------------------- */
extern volatile int16_t encoder_count;

/* --------------------------------- 函数声明 ------------------------------------ */
/** ***************************************************************************
 * @brief 编码器一键初始化函数
 */
void Encoder_Init(void);

/** ***************************************************************************
 * @brief 获取编码器状态变化
 * @retval encoder_state_t 编码器状态
 */
encoder_state_t Encoder_Get_State_Change(void);

/** ***************************************************************************
 * @brief 获取编码器当前计数值
 * @retval float 编码器计数值
 */
float Encoder_Get_Count(void);

/** ***************************************************************************
 * @brief 设置编码器计数值
 * @param count 要设置的计数值
 */
void Encoder_Set_Count(int16_t count);

/** ***************************************************************************
 * @brief 检查编码器是否增加
 * @retval uint8_t 1-增加，0-未增加
 */
uint8_t Encoder_Is_Increase(void);

/** ***************************************************************************
 * @brief 检查编码器是否减少
 * @retval uint8_t 1-减少，0-未减少
 */
uint8_t Encoder_Is_Decrease(void);

#endif /* __ENCODER_EXTI_H */