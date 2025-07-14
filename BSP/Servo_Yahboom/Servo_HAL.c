/** ***************************************************************************
 * @File_Name: Servo_HAL.c
 * @brief 舵机硬件抽象层实现 - STM32平台完整实现
 * @Author: Sighthesia
 * @Version: 1.0.0
 * @Creat_Date: 2025-07-13
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author: Sighthesia
 * @Changes:
 * @Modifi_Date:
 */

#include "Servo_HAL.h"
#include "tim.h"

/* --------------------------------- 此处修改STM32平台HAL函数实现 --------------------------------- */

/** ***************************************************************************
 * @brief STM32平台PWM启动实现
 * @param tim_handle: 定时器句柄
 * @param tim_channel: 定时器通道
 * @return Servo_HAL_Status_t: 操作状态
 */
static Servo_HAL_Status_t Servo_STM32_PWM_Start(void *tim_handle, uint32_t tim_channel)
{
    if (tim_handle == NULL)
    {
        return SERVO_HAL_INVALID_PARAM;
    }

    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)tim_handle;
    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(htim, tim_channel);

    return (status == HAL_OK) ? SERVO_HAL_OK : SERVO_HAL_ERROR;
}

/** ***************************************************************************
 * @brief STM32平台PWM停止实现
 * @param tim_handle: 定时器句柄
 * @param tim_channel: 定时器通道
 * @return Servo_HAL_Status_t: 操作状态
 */
static Servo_HAL_Status_t Servo_STM32_PWM_Stop(void *tim_handle, uint32_t tim_channel)
{
    if (tim_handle == NULL)
    {
        return SERVO_HAL_INVALID_PARAM;
    }

    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)tim_handle;
    HAL_StatusTypeDef status = HAL_TIM_PWM_Stop(htim, tim_channel);

    return (status == HAL_OK) ? SERVO_HAL_OK : SERVO_HAL_ERROR;
}

/** ***************************************************************************
 * @brief STM32平台设置PWM比较值实现
 * @param tim_handle: 定时器句柄
 * @param tim_channel: 定时器通道
 * @param ccr_value: 比较值
 * @return Servo_HAL_Status_t: 操作状态
 */
static Servo_HAL_Status_t Servo_STM32_Set_PWM_CCR(void *tim_handle, uint32_t tim_channel, uint16_t ccr_value)
{
    if (tim_handle == NULL)
    {
        return SERVO_HAL_INVALID_PARAM;
    }

    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)tim_handle;
    __HAL_TIM_SET_COMPARE(htim, tim_channel, ccr_value);

    return SERVO_HAL_OK;
}

/** ***************************************************************************
 * @brief STM32平台延时实现
 * @param delay_ms: 延时时间(毫秒)
 * @return Servo_HAL_Status_t: 操作状态
 */
static Servo_HAL_Status_t Servo_STM32_Delay_Ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
    return SERVO_HAL_OK;
}

/* --------------------------------- 此处修改全局HAL接口实例 --------------------------------- */
const servo_hal_t servo_hal = {
    .PWM_Start = Servo_STM32_PWM_Start,
    .PWM_Stop = Servo_STM32_PWM_Stop,
    .Set_PWM_CCR = Servo_STM32_Set_PWM_CCR,
    .Delay_Ms = Servo_STM32_Delay_Ms};
