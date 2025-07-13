/** ***************************************************************************
 * @File_Name: README_Group_Control.md
 * @brief 舵机组别控制与数量获取功能使用说明
 * @Author: Sighthesia
 * @Version: 1.0.0
 * @Creat_Date: 2025-07-13
 * ----------------------------------------------------------------------------
 * 舵机组别控制功能详细使用说明
 */

# 舵机组别控制与数量获取功能

## 概述

新增的组别控制和多定时器支持功能允许您：
1.  **按功能分组管理舵机**：例如云台组、机械臂组等。
2.  **使用不同的定时器驱动舵机**：突破单个定时器的通道数限制，灵活分配硬件资源。

例如：
- **组别0 (SERVO_GROUP_0)**: 云台组 - 使用 `TIM1`
- **组别1 (SERVO_GROUP_1)**: 机械臂组 - 使用 `TIM2`
- **组别2 (SERVO_GROUP_2)**: 附加组 - 使用 `TIM3`
- **组别3 (SERVO_GROUP_3)**: 预留组 - 备用扩展

## 配置舵机组别和定时器

### 1. 修改配置表

在 `Servo_PWM.c` 中的 `servo_config_table` 添加组别和定时器句柄配置：

```c
static const servo_config_t servo_config_table[MAX_SERVO] = {
    // {定时器通道, 定时器句柄, 舵机ID, 组别ID, 是否启用, "名称", 默认PWM, 最大PWM, 最小PWM, 中位PWM},
    {TIM_CHANNEL_1, &htim1, SERVO_ID_J0, SERVO_GROUP_0, 1, "Servo_1X", 1097, 2400, 500, 1097}, // X轴舵机 - 云台组 (使用TIM1)
    {TIM_CHANNEL_2, &htim1, SERVO_ID_J1, SERVO_GROUP_0, 1, "Servo_1Y", 933, 2100, 400, 933},   // Y轴舵机 - 云台组 (使用TIM1)
    {TIM_CHANNEL_1, &htim2, SERVO_ID_J2, SERVO_GROUP_1, 1, "Servo_2X", 1500, 2500, 500, 1500}, // 机械臂舵机1 - 机械臂组 (使用TIM2)
    {TIM_CHANNEL_2, &htim2, SERVO_ID_J3, SERVO_GROUP_1, 1, "Servo_2Y", 1500, 2500, 500, 1500}, // 机械臂舵机2 - 机械臂组 (使用TIM2)
};
```

## 使用方法

### 1. 按组别控制单个舵机

```c
// 设置云台组(组别0)中的第0个舵机(X轴)到90度
Servo_Set_Angle_By_Group(SERVO_GROUP_0, 0, 90);

// 设置机械臂组(组别1)中的第1个舵机到1500μs PWM值
Servo_Set_PWM_By_Group(SERVO_GROUP_1, 1, 1500);
```

### 2. 批量控制整个组别

```c
// 同时控制云台组所有舵机
uint8_t gimbal_angles[] = {45, 135}; // X轴45度，Y轴135度
Servo_Set_Group_Angles(SERVO_GROUP_0, gimbal_angles, 2);

// 同时控制机械臂组所有舵机
uint16_t arm_pwms[] = {1200, 1800}; // 设置不同的PWM值
Servo_Set_Group_PWMs(SERVO_GROUP_1, arm_pwms, 2);
```

### 3. 组别启停控制

```c
// 启用整个云台组
Servo_Enable_By_Group(SERVO_GROUP_0, 1);

// 禁用整个机械臂组
Servo_Enable_By_Group(SERVO_GROUP_1, 0);

// 停止云台组所有舵机PWM输出
Servo_Stop_By_Group(SERVO_GROUP_0);

// 启动机械臂组所有舵机PWM输出
Servo_Start_By_Group(SERVO_GROUP_1);
```

### 4. 组别校准

```c
// 校准整个云台组
Servo_Calibrate_By_Group(SERVO_GROUP_0);

// 校准整个机械臂组
Servo_Calibrate_By_Group(SERVO_GROUP_1);
```

## 获取舵机数量信息

### 1. 获取各种数量信息

```c
// 获取所有激活的舵机数量
uint8_t active_count = Servo_Get_Active_Count();

// 获取已配置的最大舵机数量(根据配置表)
uint8_t max_configured = Servo_Get_Max_Configured_Count();

// 获取云台组中的舵机数量
uint8_t gimbal_count = Servo_Get_Group_Count(SERVO_GROUP_0);

// 获取最大组别数量
uint8_t max_groups = Servo_Get_Max_Group_Count();

printf("激活舵机: %d, 已配置: %d, 云台组: %d, 最大组别: %d\\n", 
       active_count, max_configured, gimbal_count, max_groups);
```

### 2. 获取组别舵机数组

```c
// 获取云台组所有舵机的信息
uint8_t gimbal_count;
servo_typedef_t *gimbal_servos = Servo_Get_Group_Array(SERVO_GROUP_0, &gimbal_count);

if (gimbal_servos != NULL)
{
    for (uint8_t i = 0; i < gimbal_count; i++)
    {
        printf("云台舵机[%d]: ID=%d, PWM=%d, 名称=%s\\n", 
               i, gimbal_servos[i].servo_id, gimbal_servos[i].pwm, gimbal_servos[i].name);
    }
}
```

## 实际应用示例

### 云台控制示例

```c
void Gimbal_Control_Demo(void)
{
    // 初始化舵机
    Servo_TIM_PWM_Init();
    
    // 云台归中
    uint8_t center_angles[] = {90, 90};
    Servo_Set_Group_Angles(SERVO_GROUP_0, center_angles, 2);
    HAL_Delay(1000);
    
    // 云台扫描
    for (uint8_t x = 30; x <= 150; x += 10)
    {
        Servo_Set_Angle_By_Group(SERVO_GROUP_0, 0, x); // X轴扫描
        HAL_Delay(100);
    }
    
    // 检查云台组状态
    uint8_t gimbal_count = Servo_Get_Group_Count(SERVO_GROUP_0);
    printf("云台组舵机数量: %d\\n", gimbal_count);
}
```

### 机械臂控制示例

```c
void Robot_Arm_Control_Demo(void)
{
    // 检查机械臂组是否有舵机
    uint8_t arm_count = Servo_Get_Group_Count(SERVO_GROUP_1);
    if (arm_count == 0)
    {
        printf("机械臂组无可用舵机\\n");
        return;
    }
    
    // 机械臂初始位置
    uint8_t init_angles[] = {90, 45};
    Servo_Set_Group_Angles(SERVO_GROUP_1, init_angles, arm_count);
    HAL_Delay(1000);
    
    // 机械臂动作序列
    uint8_t action1[] = {120, 60};
    uint8_t action2[] = {60, 120};
    
    Servo_Set_Group_Angles(SERVO_GROUP_1, action1, arm_count);
    HAL_Delay(2000);
    
    Servo_Set_Group_Angles(SERVO_GROUP_1, action2, arm_count);
    HAL_Delay(2000);
    
    // 机械臂归位
    Servo_Set_Group_Angles(SERVO_GROUP_1, init_angles, arm_count);
}
```

## 优势

1. **功能分组**: 将相关舵机分组管理，代码更清晰
2. **批量操作**: 一次性控制整个组别的舵机
3. **动态信息**: 实时获取各组别和总体的舵机数量信息
4. **灵活配置**: 可以轻松调整舵机的组别归属
5. **向后兼容**: 原有的按ID控制方式仍然可用
6. **多定时器支持**: 可以混合使用多个定时器，扩展舵机控制能力

## 多定时器支持函数

新增的函数可以方便地管理和查询与定时器相关的舵机信息：

- `TIM_HandleTypeDef *Servo_Get_TIM_Handle_By_ID(uint8_t servo_id)`
  - **功能**: 根据舵机ID获取其对应的定时器句柄。
  - **示例**: `TIM_HandleTypeDef* tim = Servo_Get_TIM_Handle_By_ID(SERVO_ID_J0);`

- `uint8_t Servo_Get_Servo_Count_By_TIM(TIM_HandleTypeDef *tim_handle)`
  - **功能**: 查询某个特定定时器上挂载了多少个舵机。
  - **示例**: `uint8_t count = Servo_Get_Servo_Count_By_TIM(&htim1);`

- `void Servo_Set_TIM_Handle_By_ID(uint8_t servo_id, TIM_HandleTypeDef *tim_handle)`
  - **功能**: 动态修改舵机的定时器句柄（建议在舵机停止时使用）。
  - **示例**: `Servo_Set_TIM_Handle_By_ID(SERVO_ID_J0, &htim5);`

- `void Servo_Stop_By_TIM_Handle(TIM_HandleTypeDef *tim_handle)`
  - **功能**: 停止某个定时器上所有舵机的PWM输出。
  - **示例**: `Servo_Stop_By_TIM_Handle(&htim2);`

- `void Servo_Start_By_TIM_Handle(TIM_HandleTypeDef *tim_handle)`
  - **功能**: 启动某个定时器上所有舵机的PWM输出。
  - **示例**: `Servo_Start_By_TIM_Handle(&htim2);`

## 注意事项

1. 组别ID从0开始，最大支持4个组别(0-3)
2. 组内舵机索引从0开始计算
3. `Servo_Get_Group_Array()` 返回的是临时数组，不要长期保存指针
4. 确保在调用组别函数前已正确配置舵机的组别ID。
5. 组别功能与原有按ID控制功能可以混合使用。
6. 修改舵机配置表的 `tim_handle` 时，请确保对应的定时器已经初始化。

这样，您就可以更高效地管理和控制多组、多定时器的舵机系统了！
