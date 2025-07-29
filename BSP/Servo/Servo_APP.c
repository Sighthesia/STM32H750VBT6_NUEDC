/** ***************************************************************************
 * @File Name: Servo_APP.c
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-29
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#include "Servo_APP.h"

// 只读配置表
static const servo_config_t servo_config_list[MAX_SERVO] = {
    {TIM_CHANNEL_3, &htim3, SERVO_0X, 1, "Servo_0X", 1500, 2500, 500, 1500, SERVO_ANGLE_TYPE_270},
    {TIM_CHANNEL_4, &htim3, SERVO_0Y, 1, "Servo_0Y", 1500, 2500, 500, 1500, SERVO_ANGLE_TYPE_180},
};

// 可更改的运行时舵机数组
servo_config_t servo_list[MAX_SERVO];

/** ***************************************************************************
 * @brief 角度转换为PWM值 (支持180/270度舵机)
 */
static uint16_t Angle_To_PWM(uint8_t servo_id, float angle)
{
    if (!Validate_Servo_ID(servo_id))
        return 500;
    servo_angle_type_t angle_type = servo_list[servo_id].angle_type;
    float max_angle = (angle_type == SERVO_ANGLE_TYPE_270) ? 270.0f : 180.0f;
    if (angle > max_angle)
        angle = max_angle;
    else if (angle < 0.0f)
        angle = 0.0f;
    return 500 + (uint16_t)(angle * (2500 - 500) / max_angle);
}

/** ***************************************************************************
 * @brief 验证舵机ID有效性
 */
static uint8_t Validate_Servo_ID(uint8_t servo_id)
{
    return (servo_id < MAX_SERVO);
}

/** ***************************************************************************
 * @brief 设置定时器比较值
 */
static void Set_PWM_CCR(uint8_t servo_index, uint16_t ccr_value)
{
    TIM_HandleTypeDef *htim = servo_list[servo_index].tim_handle;
    uint32_t channel = servo_list[servo_index].tim_channel;
    servo_hal.Set_PWM_CCR(htim, channel, ccr_value);
}

/** ***************************************************************************
 * @brief 从配置表初始化舵机数组
 */
static void Config_Init(void)
{
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        servo_list[i] = servo_config_list[i];
        servo_list[i].pwm = servo_list[i].mid_pwm;
    }
}

/** ***************************************************************************
 * @brief 初始化舵机模块
 */
void Servo_Init(void)
{
    Config_Init();
    Servo_HAL_Start_All();
}

/** ***************************************************************************
 * @brief 按ID设置舵机角度 (支持180/270度舵机)
 */
void Servo_HAL_Set_Angle_By_ID(uint8_t servo_id, float angle)
{
    if (!Validate_Servo_ID(servo_id))
        return;
    if (!servo_list[servo_id].enabled)
        return;
    uint16_t pwm_value = Angle_To_PWM(servo_id, angle);
    if (pwm_value > servo_list[servo_id].max_pwm)
        pwm_value = servo_list[servo_id].max_pwm;
    else if (pwm_value < servo_list[servo_id].min_pwm)
        pwm_value = servo_list[servo_id].min_pwm;
    if (servo_list[servo_id].pwm == pwm_value)
        return;
    servo_list[servo_id].pwm = pwm_value;
    Set_PWM_CCR(servo_id, pwm_value);
}

/** ***************************************************************************
 * @brief 按ID设置舵机PWM值
 */
void Servo_HAL_Set_PWM(uint8_t servo_id, uint16_t pwm)
{
    if (!Validate_Servo_ID(servo_id))
        return;
    if (!servo_list[servo_id].enabled)
        return;
    if (pwm > servo_list[servo_id].max_pwm)
        pwm = servo_list[servo_id].max_pwm;
    else if (pwm < servo_list[servo_id].min_pwm)
        pwm = servo_list[servo_id].min_pwm;
    if (servo_list[servo_id].pwm == pwm)
        return;
    servo_list[servo_id].pwm = pwm;
    Set_PWM_CCR(servo_id, pwm);
}

/** ***************************************************************************
 * @brief 启动所有已启用的舵机
 */
void Servo_HAL_Start_All(void)
{
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_list[i].enabled)
        {
            servo_hal.PWM_Start(servo_list[i].tim_handle, servo_list[i].tim_channel);
            Set_PWM_CCR(i, servo_list[i].mid_pwm);
        }
    }
}

/** ***************************************************************************
 * @brief 停止所有舵机
 */
void Servo_HAL_Stop_All(void)
{
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_list[i].enabled)
        {
            servo_hal.PWM_Stop(servo_list[i].tim_handle, servo_list[i].tim_channel);
        }
    }
}
/* --------------------------------- 自定义部分结束 -------------------------------- */