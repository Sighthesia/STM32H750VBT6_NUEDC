/** ***************************************************************************
 * @File Name: Servo_PWM.c
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-13
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */

#include "Servo_PWM.h"

/* --------------------------------- 此处修改舵机配置 -------------------------------- */
static const servo_config_t servo_config_table[MAX_SERVO] = {
    // {定时器句柄, 定时器通道, 舵机ID, 组别ID, 是否启用, 角度类型, 名称(调试), 当前PWM, 最大PWM, 最小PWM, 中位PWM},
    {&htim3, TIM_CHANNEL_3, SERVO_ID_J0, SERVO_GROUP_0, 1, SERVO_ANGLE_TYPE_270, "Servo_0X", 1097, 2400, 500, 1097}, // X轴舵机 - 270°舵机
    {&htim3, TIM_CHANNEL_4, SERVO_ID_J1, SERVO_GROUP_0, 1, SERVO_ANGLE_TYPE_180, "Servo_0Y", 933, 2100, 400, 933},   // Y轴舵机 - 180°舵机
};

servo_typedef_t servo_array[MAX_SERVO]; // 舵机控制数组

// 角度数组支持小数
volatile float servo_angle[MAX_GROUP][MAX_SERVO];

#ifdef USE_HARDWARE_PWM

/* ------------------------------ 私有变量和函数声明 ------------------------------ */
static uint8_t servo_initialized = 0; // 初始化标志

// 私有函数声明
static void Servo_Init_From_Config_Table(void); // 从配置表初始化舵机数组
static void Servo_Set_CCR(uint8_t servo_index, uint16_t ccr_value);
static uint8_t Servo_Validate_ID(uint8_t servo_id); // 验证舵机ID有效性

/* ------------------------------ 数组驱动核心函数实现 ------------------------------ */

/** ***************************************************************************
 * @brief 从配置表初始化舵机数组 (私有函数)
 * @note 数据驱动设计的核心：通过配置表批量初始化舵机
 * @param 无
 * @return 无
 */
static void Servo_Init_From_Config_Table(void)
{
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        servo_array[i].pwm = servo_config_table[i].default_pwm;
        servo_array[i].max_pwm = servo_config_table[i].max_pwm;
        servo_array[i].min_pwm = servo_config_table[i].min_pwm;
        servo_array[i].mid_pwm = servo_config_table[i].mid_pwm;
        servo_array[i].tim_channel = servo_config_table[i].tim_channel;
        servo_array[i].tim_handle = servo_config_table[i].tim_handle;
        servo_array[i].servo_id = servo_config_table[i].servo_id;
        servo_array[i].group_id = servo_config_table[i].group_id;
        servo_array[i].enabled = servo_config_table[i].enabled;
        servo_array[i].angle_type = servo_config_table[i].angle_type;
        servo_array[i].name = servo_config_table[i].name;
    }
}

/** ***************************************************************************
 * @brief 验证舵机ID有效性 (私有函数)
 * @param  servo_id: 舵机ID
 * @return uint8_t: 1=有效, 0=无效
 */
static uint8_t Servo_Validate_ID(uint8_t servo_id)
{
    return (servo_id >= 0 && servo_id < MAX_SERVO);
}

/** ***************************************************************************
 * @brief 硬件PWM初始化 - 数组驱动版本
 */
void Servo_PWM_TIM_Init(void)
{
    // 1. 从配置表初始化舵机数组
    Servo_Init_From_Config_Table();

    // 2. 批量启动PWM输出 - 表驱动法
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].enabled)
        {
            servo_hal.PWM_Start(servo_array[i].tim_handle, servo_array[i].tim_channel);
            // 设置初始位置
            Servo_Set_PWM_By_ID(servo_array[i].servo_id, servo_array[i].mid_pwm);
        }
    }

    servo_initialized = 1;
}

/** ***************************************************************************
 * @brief 按ID设置舵机角度 (自适应角度类型)
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 * @param  angle: 角度 (根据舵机类型自动适配：180°舵机 0.0-180.0°，270°舵机 0.0-270.0°)
 */
void Servo_Set_Angle_By_ID(uint8_t servo_id, float angle)
{
    if (!Servo_Validate_ID(servo_id))
    {
        return;
    }

    uint8_t index = servo_id;
    if (!servo_array[index].enabled)
    {
        return;
    }

    uint16_t pwm_value = Servo_Angle_To_PWM_By_Type(angle, servo_array[index].angle_type);

    // 限制PWM值范围
    if (pwm_value > servo_array[index].max_pwm)
    {
        pwm_value = servo_array[index].max_pwm;
    }
    else if (pwm_value < servo_array[index].min_pwm)
    {
        pwm_value = servo_array[index].min_pwm;
    }

    // 避免重复设置
    if (servo_array[index].pwm == pwm_value)
    {
        return;
    }

    servo_array[index].pwm = pwm_value;

    // 设置定时器比较值
    Servo_Set_CCR(index, pwm_value);
}

/** ***************************************************************************
 * @brief 按ID设置舵机PWM值
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 * @param  pwm: PWM值(微秒)
 */
void Servo_Set_PWM_By_ID(uint8_t servo_id, uint16_t pwm)
{
    if (!Servo_Validate_ID(servo_id))
    {
        return;
    }

    uint8_t index = servo_id;
    if (!servo_array[index].enabled)
    {
        return;
    }

    // 限制PWM值范围
    if (pwm > servo_array[index].max_pwm)
    {
        pwm = servo_array[index].max_pwm;
    }
    else if (pwm < servo_array[index].min_pwm)
    {
        pwm = servo_array[index].min_pwm;
    }

    // 避免重复设置
    if (servo_array[index].pwm == pwm)
    {
        return;
    }

    servo_array[index].pwm = pwm;

    // 设置定时器比较值
    Servo_Set_CCR(index, pwm);
}

/** ***************************************************************************
 * @brief 批量设置多个舵机角度
 * @param  servo_ids: 舵机ID数组
 * @param  angles: 角度数组(支持小数)
 * @param  count: 舵机数量
 */
void Servo_Set_Multiple_Angles(uint8_t *servo_ids, float *angles, uint8_t count)
{
    if (servo_ids == NULL || angles == NULL || count == 0)
    {
        return;
    }

    for (uint8_t i = 0; i < count; i++)
    {
        Servo_Set_Angle_By_ID(servo_ids[i], angles[i]);
    }
}

/** ***************************************************************************
 * @brief 批量设置多个舵机PWM值
 * @param  servo_ids: 舵机ID数组
 * @param  pwms: PWM值数组(微秒)
 * @param  count: 舵机数量
 */
void Servo_Set_Multiple_PWMs(uint8_t *servo_ids, uint16_t *pwms, uint8_t count)
{
    if (servo_ids == NULL || pwms == NULL || count == 0)
    {
        return;
    }

    for (uint8_t i = 0; i < count; i++)
    {
        Servo_Set_PWM_By_ID(servo_ids[i], pwms[i]);
    }
}

/** ***************************************************************************
 * @brief 启用/禁用指定舵机
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 * @param  enable: 1=启用, 0=禁用
 */
void Servo_Enable_By_ID(uint8_t servo_id, uint8_t enable)
{
    if (!Servo_Validate_ID(servo_id))
    {
        return;
    }

    uint8_t index = servo_id;
    servo_array[index].enabled = enable ? 1 : 0;

    if (servo_initialized)
    {
        if (enable)
        {
            servo_hal.PWM_Start(servo_array[index].tim_handle, servo_array[index].tim_channel);
        }
        else
        {
            servo_hal.PWM_Stop(servo_array[index].tim_handle, servo_array[index].tim_channel);
        }
    }
}

/** ***************************************************************************
 * @brief 配置指定舵机参数
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 * @param  min_pwm: 最小PWM值
 * @param  max_pwm: 最大PWM值
 * @param  mid_pwm: 中位PWM值
 */
void Servo_Set_Config_By_ID(uint8_t servo_id, uint16_t min_pwm, uint16_t max_pwm, uint16_t mid_pwm)
{
    if (!Servo_Validate_ID(servo_id))
    {
        return;
    }

    uint8_t index = servo_id;
    if (!servo_array[index].enabled)
    {
        return;
    }

    servo_array[index].min_pwm = min_pwm;
    servo_array[index].max_pwm = max_pwm;
    servo_array[index].mid_pwm = mid_pwm;
}

/* ========================== 数组驱动核心辅助函数 ========================== */

/** ***************************************************************************
 * @brief 设置舵机CCR值 (私有函数)
 * @param  servo_index: 舵机数组索引
 * @param  ccr_value: CCR值
 */
static void Servo_Set_CCR(uint8_t servo_index, uint16_t ccr_value)
{
    if (!servo_initialized || servo_index >= MAX_SERVO)
    {
        return;
    }

    // 根据通道设置CCR值 - 使用HAL接口
    servo_hal.Set_PWM_CCR(servo_array[servo_index].tim_handle, servo_array[servo_index].tim_channel, ccr_value);
}

/** ***************************************************************************
 * @brief 角度转换为PWM值 (传统180°舵机)
 * @param  angle: 角度 0.0-180.0°(支持小数)
 * @return uint16_t: PWM值(微秒)
 * @note 保留原有函数以兼容旧代码
 */
uint16_t Servo_Angle_To_PWM(float angle)
{
    return Servo_Angle_To_PWM_By_Type(angle, SERVO_ANGLE_TYPE_180);
}

/** ***************************************************************************
 * @brief 根据舵机类型将角度转换为PWM值
 * @param  angle: 角度值(支持小数)
 * @param  angle_type: 舵机角度类型
 * @return uint16_t: PWM值(微秒)
 */
uint16_t Servo_Angle_To_PWM_By_Type(float angle, servo_angle_type_t angle_type)
{
    float max_angle;

    // 根据舵机类型确定最大角度
    switch (angle_type)
    {
    case SERVO_ANGLE_TYPE_180:
        max_angle = 180.0f;
        break;
    case SERVO_ANGLE_TYPE_270:
        max_angle = 270.0f;
        break;
    case SERVO_ANGLE_TYPE_360:
        max_angle = 360.0f;
        break;
    default:
        max_angle = 180.0f; // 默认180°
        break;
    }

    // 限制角度范围
    if (angle > max_angle)
    {
        angle = max_angle;
    }
    else if (angle < 0.0f)
    {
        angle = 0.0f;
    }

    // 线性映射: 0° → 500μs, max_angle° → 2500μs
    return (uint16_t)(SERVO_PWM_MIN + (angle * (SERVO_PWM_MAX - SERVO_PWM_MIN)) / max_angle);
}

/** ***************************************************************************
 * @brief PWM值转换为角度 (传统180°舵机)
 * @param  pwm_value: PWM值(微秒)
 * @return float: 角度 0.0-180.0°(支持小数)
 * @note 保留原有函数以兼容旧代码
 */
float Servo_PWM_To_Angle(uint16_t pwm_value)
{
    return Servo_PWM_To_Angle_By_Type(pwm_value, SERVO_ANGLE_TYPE_180);
}

/** ***************************************************************************
 * @brief 根据舵机类型将PWM值转换为角度
 * @param  pwm_value: PWM值(微秒)
 * @param  angle_type: 舵机角度类型
 * @return float: 角度值(支持小数)
 */
float Servo_PWM_To_Angle_By_Type(uint16_t pwm_value, servo_angle_type_t angle_type)
{
    float max_angle;

    // 根据舵机类型确定最大角度
    switch (angle_type)
    {
    case SERVO_ANGLE_TYPE_180:
        max_angle = 180.0f;
        break;
    case SERVO_ANGLE_TYPE_270:
        max_angle = 270.0f;
        break;
    case SERVO_ANGLE_TYPE_360:
        max_angle = 360.0f;
        break;
    default:
        max_angle = 180.0f; // 默认180°
        break;
    }

    // 限制PWM值范围
    if (pwm_value > SERVO_PWM_MAX)
    {
        pwm_value = SERVO_PWM_MAX;
    }
    else if (pwm_value < SERVO_PWM_MIN)
    {
        pwm_value = SERVO_PWM_MIN;
    }

    // 线性映射: 500μs → 0°, 2500μs → max_angle°
    return ((float)(pwm_value - SERVO_PWM_MIN) * max_angle) / (float)(SERVO_PWM_MAX - SERVO_PWM_MIN);
}

/** ***************************************************************************
 * @brief 停止所有舵机
 * @note 停止所有启用舵机的PWM输出
 */
void Servo_Stop_All(void)
{
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].enabled)
        {
            servo_hal.PWM_Stop(servo_array[i].tim_handle, servo_array[i].tim_channel);
        }
    }
}

/** ***************************************************************************
 * @brief 启动所有舵机
 * @note 启动所有启用舵机的PWM输出
 */
void Servo_Start_All(void)
{
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].enabled)
        {
            servo_hal.PWM_Start(servo_array[i].tim_handle, servo_array[i].tim_channel);
        }
    }
}

/** ***************************************************************************
 * @brief 停止指定舵机
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 */
void Servo_Stop_By_ID(uint8_t servo_id)
{
    if (!Servo_Validate_ID(servo_id))
    {
        return;
    }

    uint8_t index = servo_id;
    if (servo_array[index].enabled)
    {
        servo_hal.PWM_Stop(servo_array[index].tim_handle, servo_array[index].tim_channel);
    }
}

/** ***************************************************************************
 * @brief 启动指定舵机
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 */
void Servo_Start_By_ID(uint8_t servo_id)
{
    if (!Servo_Validate_ID(servo_id))
    {
        return;
    }

    uint8_t index = servo_id;
    if (servo_array[index].enabled)
    {
        servo_hal.PWM_Start(servo_array[index].tim_handle, servo_array[index].tim_channel);
    }
}

/* ========================== 高级功能函数 ========================== */

/** ***************************************************************************
 * @brief 舵机平滑移动
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 * @param  target_angle: 目标角度 0.0-180.0°(支持小数)
 * @param  step_delay: 每步延时(ms)
 */
void Servo_Smooth_Move_By_ID(uint8_t servo_id, float target_angle, uint16_t step_delay)
{
    if (!Servo_Validate_ID(servo_id))
    {
        return;
    }

    uint8_t index = servo_id;
    if (!servo_array[index].enabled)
    {
        return;
    }

    float current_angle = Servo_PWM_To_Angle(servo_array[index].pwm);
    float step_size = 0.5f; // 每步0.5度，可以调整精度

    // 限制目标角度范围
    if (target_angle > 180.0f)
    {
        target_angle = 180.0f;
    }
    else if (target_angle < 0.0f)
    {
        target_angle = 0.0f;
    }

    if (target_angle > current_angle)
    {
        // 正向移动
        for (float angle = current_angle; angle <= target_angle; angle += step_size)
        {
            if (angle > target_angle)
            {
                angle = target_angle; // 确保最后一步到达精确位置
            }
            Servo_Set_Angle_By_ID(servo_id, angle);
            servo_hal.Delay_Ms(step_delay);
        }
    }
    else if (target_angle < current_angle)
    {
        // 反向移动
        for (float angle = current_angle; angle >= target_angle; angle -= step_size)
        {
            if (angle < target_angle)
            {
                angle = target_angle; // 确保最后一步到达精确位置
            }
            Servo_Set_Angle_By_ID(servo_id, angle);
            servo_hal.Delay_Ms(step_delay);
        }
    }
}

/** ***************************************************************************
 * @brief 舵机校准
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 */
void Servo_Calibrate_By_ID(uint8_t servo_id)
{
    if (!Servo_Validate_ID(servo_id))
    {
        return;
    }

    uint8_t index = servo_id;
    if (!servo_array[index].enabled)
    {
        return;
    }

    // 校准序列: 0° -> 90° -> 180° -> 90°
    Servo_Set_Angle_By_ID(servo_id, 0);
    servo_hal.Delay_Ms(1000);

    Servo_Set_Angle_By_ID(servo_id, 90);
    servo_hal.Delay_Ms(1000);

    Servo_Set_Angle_By_ID(servo_id, 180);
    servo_hal.Delay_Ms(1000);

    Servo_Set_Angle_By_ID(servo_id, 90);
    servo_hal.Delay_Ms(1000);
}

/** ***************************************************************************
 * @brief 校准所有舵机
 */
void Servo_Calibrate_All(void)
{
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].enabled)
        {
            Servo_Calibrate_By_ID(servo_array[i].servo_id);
        }
    }
}

/** ***************************************************************************
 * @brief 舵机测试
 */
void Servo_Test(void)
{
    // 测试所有已启用的舵机
    Servo_Calibrate_All();
}

/** ***************************************************************************
 * @brief 打印所有舵机状态
 */
void Servo_Print_Status(void)
{
    // 注意：需要printf支持，或者通过其他方式输出
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].enabled)
        {
            // printf("Servo[%d] %s: PWM=%d, Angle=%d, Channel=0x%08X\r\n",
            //        servo_array[i].servo_id, servo_array[i].name, servo_array[i].pwm,
            //        Servo_PWM_To_Angle(servo_array[i].pwm), servo_array[i].tim_channel);
        }
    }
}

/** ***************************************************************************
 * @brief 获取激活舵机数量
 * @return uint8_t: 激活的舵机数量
 */
uint8_t Servo_Get_Active_Count(void)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].enabled)
        {
            count++;
        }
    }
    return count;
}

/* ========================== 组别控制函数 ========================== */

/** ***************************************************************************
 * @brief 按组别和索引设置舵机角度
 * @param  group_id: 组别ID
 * @param  servo_index: 组内舵机索引 (从0开始)
 * @param  angle: 角度 0.0-180.0°(支持小数)
 */
void Servo_Set_Angle_By_Group(uint8_t group_id, uint8_t servo_index, float angle)
{
    uint8_t found_count = 0;
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].group_id == group_id && servo_array[i].enabled)
        {
            if (found_count == servo_index)
            {
                Servo_Set_Angle_By_ID(servo_array[i].servo_id, angle);
                return;
            }
            found_count++;
        }
    }
}

/** ***************************************************************************
 * @brief 按组别和索引设置舵机PWM值
 * @param  group_id: 组别ID
 * @param  servo_index: 组内舵机索引 (从0开始)
 * @param  pwm: PWM值(微秒)
 */
void Servo_Set_PWM_By_Group(uint8_t group_id, uint8_t servo_index, uint16_t pwm)
{
    uint8_t found_count = 0;
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].group_id == group_id && servo_array[i].enabled)
        {
            if (found_count == servo_index)
            {
                Servo_Set_PWM_By_ID(servo_array[i].servo_id, pwm);
                return;
            }
            found_count++;
        }
    }
}

/** ***************************************************************************
 * @brief 设置指定组别所有舵机的角度
 * @param  group_id: 组别ID
 * @param  angles: 角度数组(支持小数)
 * @param  count: 角度数组长度
 */
void Servo_Set_Group_Angles(uint8_t group_id, float *angles, uint8_t count)
{
    if (angles == NULL || count == 0)
    {
        return;
    }

    uint8_t found_count = 0;
    for (uint8_t i = 0; i < MAX_SERVO && found_count < count; i++)
    {
        if (servo_array[i].group_id == group_id && servo_array[i].enabled)
        {
            Servo_Set_Angle_By_ID(servo_array[i].servo_id, angles[found_count]);
            found_count++;
        }
    }
}

/** ***************************************************************************
 * @brief 设置指定组别所有舵机的PWM值
 * @param  group_id: 组别ID
 * @param  pwms: PWM值数组(微秒)
 * @param  count: PWM数组长度
 */
void Servo_Set_Group_PWMs(uint8_t group_id, uint16_t *pwms, uint8_t count)
{
    if (pwms == NULL || count == 0)
    {
        return;
    }

    uint8_t found_count = 0;
    for (uint8_t i = 0; i < MAX_SERVO && found_count < count; i++)
    {
        if (servo_array[i].group_id == group_id && servo_array[i].enabled)
        {
            Servo_Set_PWM_By_ID(servo_array[i].servo_id, pwms[found_count]);
            found_count++;
        }
    }
}

/** ***************************************************************************
 * @brief 启用/禁用指定组别的所有舵机
 * @param  group_id: 组别ID
 * @param  enable: 1=启用, 0=禁用
 */
void Servo_Enable_By_Group(uint8_t group_id, uint8_t enable)
{
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].group_id == group_id)
        {
            Servo_Enable_By_ID(servo_array[i].servo_id, enable);
        }
    }
}

/** ***************************************************************************
 * @brief 停止指定组别的所有舵机
 * @param  group_id: 组别ID
 */
void Servo_Stop_By_Group(uint8_t group_id)
{
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].group_id == group_id && servo_array[i].enabled)
        {
            Servo_Stop_By_ID(servo_array[i].servo_id);
        }
    }
}

/** ***************************************************************************
 * @brief 启动指定组别的所有舵机
 * @param  group_id: 组别ID
 */
void Servo_Start_By_Group(uint8_t group_id)
{
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].group_id == group_id && servo_array[i].enabled)
        {
            Servo_Start_By_ID(servo_array[i].servo_id);
        }
    }
}

/** ***************************************************************************
 * @brief 校准指定组别的所有舵机
 * @param  group_id: 组别ID
 */
void Servo_Calibrate_By_Group(uint8_t group_id)
{
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].group_id == group_id && servo_array[i].enabled)
        {
            Servo_Calibrate_By_ID(servo_array[i].servo_id);
        }
    }
}

/* ========================== 获取舵机数量信息函数 ========================== */

/** ***************************************************************************
 * @brief 获取已配置的最大舵机数量
 * @note 直接返回舵机ID枚举的最大值，简化程序
 * @return uint8_t: 最大舵机数量(基于枚举)
 */
uint8_t Servo_Get_Max_Configured_Count(void)
{
    return (uint8_t)MAX_SERVO; // 直接使用枚举值，自动计算
}

/** ***************************************************************************
 * @brief 获取指定组别中的舵机数量
 * @param  group_id: 组别ID
 * @return uint8_t: 该组别中激活的舵机数量
 */
uint8_t Servo_Get_Group_Count(uint8_t group_id)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].group_id == group_id && servo_array[i].enabled)
        {
            count++;
        }
    }
    return count;
}

/** ***************************************************************************
 * @brief 获取最大组别数量
 * @note 直接返回组别ID枚举的最大值，简化程序
 * @return uint8_t: 最大组别数量(基于枚举)
 */
uint8_t Servo_Get_Max_Group_Count(void)
{
    return (uint8_t)MAX_GROUP; // 直接使用枚举值，自动计算
}

/** ***************************************************************************
 * @brief 获取指定组别的舵机数组
 * @param  group_id: 组别ID
 * @param  count: 输出参数，返回该组别舵机数量
 * @return servo_typedef_t*: 指向第一个匹配舵机的指针，如果没有则返回NULL
 * @note 此函数返回的是指向原数组的指针，请谨慎使用
 */
servo_typedef_t *Servo_Get_Group_Array(uint8_t group_id, uint8_t *count)
{
    static servo_typedef_t group_array[MAX_SERVO];
    uint8_t group_count = 0;

    if (count == NULL)
    {
        return NULL;
    }

    // 复制指定组别的舵机到临时数组
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].group_id == group_id && servo_array[i].enabled)
        {
            group_array[group_count] = servo_array[i];
            group_count++;
        }
    }

    *count = group_count;
    return group_count > 0 ? group_array : NULL;
}

/* ========================== 多定时器支持函数 ========================== */

/** ***************************************************************************
 * @brief 根据舵机ID获取定时器句柄
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 * @return TIM_HandleTypeDef*: 定时器句柄指针，失败返回NULL
 */
TIM_HandleTypeDef *Servo_Get_TIM_Handle_By_ID(uint8_t servo_id)
{
    if (!Servo_Validate_ID(servo_id))
    {
        return NULL;
    }

    uint8_t index = servo_id;
    return servo_array[index].tim_handle;
}

/** ***************************************************************************
 * @brief 获取指定定时器上的舵机数量
 * @param  tim_handle: 定时器句柄指针
 * @return uint8_t: 该定时器上的舵机数量
 */
uint8_t Servo_Get_Servo_Count_By_TIM(TIM_HandleTypeDef *tim_handle)
{
    if (tim_handle == NULL)
    {
        return 0;
    }

    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].tim_handle == tim_handle && servo_array[i].enabled)
        {
            count++;
        }
    }
    return count;
}

/** ***************************************************************************
 * @brief 动态设置舵机的定时器句柄
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 * @param  tim_handle: 新的定时器句柄指针
 * @note 注意：此函数应在舵机停止状态下调用
 */
void Servo_Set_TIM_Handle_By_ID(uint8_t servo_id, TIM_HandleTypeDef *tim_handle)
{
    if (!Servo_Validate_ID(servo_id) || tim_handle == NULL)
    {
        return;
    }

    uint8_t index = servo_id;

    // 如果舵机正在运行，先停止
    if (servo_array[index].enabled && servo_initialized)
    {
        servo_hal.PWM_Stop(servo_array[index].tim_handle, servo_array[index].tim_channel);
    }

    // 更新定时器句柄
    servo_array[index].tim_handle = tim_handle;

    // 如果舵机之前是启用的，重新启动
    if (servo_array[index].enabled && servo_initialized)
    {
        servo_hal.PWM_Start(servo_array[index].tim_handle, servo_array[index].tim_channel);
    }
}

/** ***************************************************************************
 * @brief 按定时器句柄停止所有相关舵机
 * @param  tim_handle: 定时器句柄指针
 */
void Servo_Stop_By_TIM_Handle(TIM_HandleTypeDef *tim_handle)
{
    if (tim_handle == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].tim_handle == tim_handle && servo_array[i].enabled)
        {
            servo_hal.PWM_Stop(servo_array[i].tim_handle, servo_array[i].tim_channel);
        }
    }
}

/** ***************************************************************************
 * @brief 按定时器句柄启动所有相关舵机
 * @param  tim_handle: 定时器句柄指针
 */
void Servo_Start_By_TIM_Handle(TIM_HandleTypeDef *tim_handle)
{
    if (tim_handle == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
        if (servo_array[i].tim_handle == tim_handle && servo_array[i].enabled)
        {
            servo_hal.PWM_Start(servo_array[i].tim_handle, servo_array[i].tim_channel);
        }
    }
}

/* ========================== 小数角度辅助函数 ========================== */

/** ***************************************************************************
 * @brief 获取指定舵机的当前角度 (自适应角度类型)
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 * @return float: 当前角度(支持小数，根据舵机类型自动适配范围)，失败返回-1.0f
 */
float Servo_Get_Current_Angle_By_ID(uint8_t servo_id)
{
    if (!Servo_Validate_ID(servo_id))
    {
        return -1.0f;
    }

    uint8_t index = servo_id;
    if (!servo_array[index].enabled)
    {
        return -1.0f;
    }

    return Servo_PWM_To_Angle_By_Type(servo_array[index].pwm, servo_array[index].angle_type);
}

/** ***************************************************************************
 * @brief 设置舵机角度(带精度限制)
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 * @param  angle: 目标角度 0.0-180.0°(支持小数)
 * @param  precision: 精度(如0.1表示精确到0.1度)
 */
void Servo_Set_Angle_With_Precision(uint8_t servo_id, float angle, float precision)
{
    if (!Servo_Validate_ID(servo_id) || precision <= 0.0f)
    {
        return;
    }

    // 根据精度四舍五入角度值
    float rounded_angle = ((int)((angle + precision / 2.0f) / precision)) * precision;

    Servo_Set_Angle_By_ID(servo_id, rounded_angle);
}

/** ***************************************************************************
 * @brief 调整舵机角度(相对当前角度增减)
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 * @param  delta_angle: 角度增量(-180.0° ~ +180.0°)
 */
void Servo_Adjust_Angle_By_ID(uint8_t servo_id, float delta_angle)
{
    if (!Servo_Validate_ID(servo_id))
    {
        return;
    }

    float current_angle = Servo_Get_Current_Angle_By_ID(servo_id);
    if (current_angle < 0.0f)
    {
        return; // 获取当前角度失败
    }

    float new_angle = current_angle + delta_angle;
    Servo_Set_Angle_By_ID(servo_id, new_angle);
}

/** ***************************************************************************
 * @brief 角度范围限制函数
 * @param  angle: 输入角度
 * @param  min_angle: 最小角度
 * @param  max_angle: 最大角度
 * @return float: 限制后的角度值
 */
float Servo_Clamp_Angle(float angle, float min_angle, float max_angle)
{
    if (angle < min_angle)
    {
        return min_angle;
    }
    else if (angle > max_angle)
    {
        return max_angle;
    }
    return angle;
}

/** ***************************************************************************
 * @brief 设置指定舵机的角度类型
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 * @param  angle_type: 舵机角度类型
 */
void Servo_Set_Angle_Type_By_ID(uint8_t servo_id, servo_angle_type_t angle_type)
{
    if (!Servo_Validate_ID(servo_id))
    {
        return;
    }

    uint8_t index = servo_id;
    servo_array[index].angle_type = angle_type;
}

/** ***************************************************************************
 * @brief 获取指定舵机的角度类型
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 * @return servo_angle_type_t: 舵机角度类型，失败返回SERVO_ANGLE_TYPE_180
 */
servo_angle_type_t Servo_Get_Angle_Type_By_ID(uint8_t servo_id)
{
    if (!Servo_Validate_ID(servo_id))
    {
        return SERVO_ANGLE_TYPE_180;
    }

    uint8_t index = servo_id;
    return servo_array[index].angle_type;
}

/** ***************************************************************************
 * @brief 获取指定舵机角度类型对应的最大角度值
 * @param  servo_id: 舵机ID (0-MAX_SERVO-1)
 * @return float: 最大角度值，失败返回180.0f
 */
float Servo_Get_Max_Angle_By_ID(uint8_t servo_id)
{
    if (!Servo_Validate_ID(servo_id))
    {
        return 180.0f;
    }

    uint8_t index = servo_id;
    servo_angle_type_t angle_type = servo_array[index].angle_type;

    switch (angle_type)
    {
    case SERVO_ANGLE_TYPE_180:
        return 180.0f;
    case SERVO_ANGLE_TYPE_270:
        return 270.0f;
    case SERVO_ANGLE_TYPE_360:
        return 360.0f;
    default:
        return 180.0f;
    }
}

/* ========================== 角度类型适配辅助函数 ========================== */

#endif /* USE_HARDWARE_PWM */
