# STM32 陀螺仪模块使用说明

本模块提供了基于STM32HAL库的陀螺仪数据接收和处理功能，支持IMU和AHRS数据格式。

## 文件结构

```
BSP/Gyro/
├── Gyro_HAL.h              # 硬件抽象层接口定义
├── Gyro_HAL_STM32.c        # STM32平台HAL层实现
├── UART_HAL.h              # UART硬件抽象层接口
├── UART_HAL_STM32.c        # STM32平台UART实现
├── Gyro_APP.h              # 应用层接口定义
├── Gyro_APP.c              # 应用层实现
├── Gyro_Example.h          # 使用示例接口
├── Gyro_Example.c          # 使用示例实现
└── README.md               # 本说明文件
```

## 主要特性

- **分层设计**: HAL层负责硬件接口，APP层提供应用接口
- **多数据格式支持**: 同时支持IMU和AHRS数据包
- **STM32HAL库**: 完全基于STM32HAL库实现，支持STM32H7系列
- **中断驱动**: 使用UART中断接收数据，效率高
- **数据缓存**: 自动缓存最新的陀螺仪数据
- **单位转换**: 提供弧度和角度两种单位的接口

## 硬件连接

### UART引脚配置

支持的UART通道和对应引脚：

| 通道 | 发送引脚 | 接收引脚 | 备注 |
|------|----------|----------|------|
| UART1 | PA9 | PA10 | 调试串口 |
| UART2 | PA2 | PA3 | 通用串口 |
| UART3 | PB10 | PB11 | 通用串口 |
| UART4 | PA0 | PA1 | 通用串口 |
| UART5 | PC12 | PD2 | 陀螺仪数据接收（推荐） |
| UART6 | PC6 | PC7 | 通用串口 |

### 陀螺仪连接

- VCC: 3.3V 或 5V（根据陀螺仪型号）
- GND: 地线
- TX: 连接到STM32的UART接收引脚
- RX: 连接到STM32的UART发送引脚（如需要）

## 快速开始

### 1. 基本初始化

```c
#include "Gyro_APP.h"

int main(void)
{
    // STM32 HAL库初始化
    HAL_Init();
    SystemClock_Config();
    
    // 初始化陀螺仪（使用默认配置）
    GYRO_APP_Init(NULL);
    
    while(1)
    {
        // 处理陀螺仪数据
        GYRO_APP_Process();
        
        // 检查是否有新数据
        if (GYRO_APP_Is_Data_Updated())
        {
            // 处理新数据
            Handle_New_Gyro_Data();
        }
        
        HAL_Delay(1);
    }
}
```

### 2. 自定义配置初始化

```c
#include "Gyro_APP.h"

void Custom_Gyro_Init(void)
{
    gyro_app_config_t config;
    
    // 配置参数
    config.uart_channel = UART_HAL_CHANNEL_5;  // 使用UART5
    config.baud_rate = 921600;                 // 波特率
    config.enable_imu = 1;                     // 启用IMU数据
    config.enable_ahrs = 1;                    // 启用AHRS数据
    
    // 初始化
    if (GYRO_APP_Init(&config) == GYRO_APP_STATUS_OK)
    {
        printf("陀螺仪初始化成功\\n");
    }
}
```

## API 接口说明

### 应用层接口 (Gyro_APP.h)

#### 初始化和控制

- `GYRO_APP_Init()`: 初始化陀螺仪
- `GYRO_APP_Start()`: 启动数据接收
- `GYRO_APP_Stop()`: 停止数据接收
- `GYRO_APP_Process()`: 处理接收数据（主循环调用）
- `GYRO_APP_Reset()`: 重置陀螺仪

#### 数据获取

- `GYRO_APP_Get_Data()`: 获取完整数据结构
- `GYRO_APP_Get_IMU_Data()`: 获取IMU数据
- `GYRO_APP_Get_AHRS_Data()`: 获取AHRS数据
- `GYRO_APP_Is_Data_Updated()`: 检查数据是否更新

#### 便捷接口

- `GYRO_APP_Get_Euler_Degrees()`: 获取欧拉角（度）
- `GYRO_APP_Get_Angular_Velocity_Degrees()`: 获取角速度（度/秒）
- `GYRO_APP_Get_Acceleration()`: 获取加速度（m/s²）
- `GYRO_APP_Get_Magnetometer()`: 获取磁力计数据（mG）
- `GYRO_APP_Get_Quaternion()`: 获取四元数

#### 结构体接口（推荐使用）

- `GYRO_APP_Get_Processed_Data()`: 一次性获取所有处理后的数据
- `GYRO_APP_Get_Euler_Struct()`: 获取欧拉角结构体
- `GYRO_APP_Get_Angular_Velocity_Struct()`: 获取角速度结构体
- `GYRO_APP_Get_Acceleration_Struct()`: 获取加速度结构体
- `GYRO_APP_Get_Magnetometer_Struct()`: 获取磁力计结构体
- `GYRO_APP_Get_Quaternion_Struct()`: 获取四元数结构体

### 示例接口 (Gyro_Example.h)

- `Gyro_Example_Init()`: 示例初始化
- `Gyro_Example_Process()`: 示例处理（主循环调用）
- `Print_Gyro_Data()`: 打印陀螺仪数据
- `Get_Current_Attitude()`: 获取当前姿态角度
- `Is_Gyro_Working()`: 检查陀螺仪工作状态
- `Gyro_Self_Test()`: 陀螺仪自检

## 使用示例

### 结构体方式获取数据（推荐）

#### 一次性获取所有数据
```c
gyro_app_processed_data_t all_data;

if (GYRO_APP_Get_Processed_Data(&all_data) == GYRO_APP_STATUS_OK)
{
    // 检查并使用欧拉角数据
    if (all_data.euler_valid)
    {
        printf("姿态角: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\n",
               all_data.euler.roll, all_data.euler.pitch, all_data.euler.yaw);
    }
    
    // 检查并使用角速度数据
    if (all_data.angular_velocity_valid)
    {
        printf("角速度: %.1f°/s, %.1f°/s, %.1f°/s\n",
               all_data.angular_velocity.roll_rate,
               all_data.angular_velocity.pitch_rate,
               all_data.angular_velocity.yaw_rate);
    }
    
    // 检查并使用加速度数据
    if (all_data.acceleration_valid)
    {
        printf("加速度: X=%.2f, Y=%.2f, Z=%.2f m/s²\n",
               all_data.acceleration.x, all_data.acceleration.y, all_data.acceleration.z);
    }
    
    // 检查并使用磁力计数据
    if (all_data.magnetometer_valid)
    {
        printf("磁力计: X=%.1f, Y=%.1f, Z=%.1f mG\n",
               all_data.magnetometer.x, all_data.magnetometer.y, all_data.magnetometer.z);
    }
    
    // 检查并使用四元数数据
    if (all_data.quaternion_valid)
    {
        printf("四元数: W=%.3f, X=%.3f, Y=%.3f, Z=%.3f\n",
               all_data.quaternion.w, all_data.quaternion.x,
               all_data.quaternion.y, all_data.quaternion.z);
    }
}
```

#### 按需获取特定数据类型
```c
// 获取欧拉角结构体
gyro_app_euler_t euler;
if (GYRO_APP_Get_Euler_Struct(&euler) == GYRO_APP_STATUS_OK && euler.valid)
{
    printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", 
           euler.roll, euler.pitch, euler.yaw);
    printf("数据时间戳: %lu ms\n", euler.timestamp);
}

// 获取角速度结构体
gyro_app_angular_velocity_t angular_velocity;
if (GYRO_APP_Get_Angular_Velocity_Struct(&angular_velocity) == GYRO_APP_STATUS_OK && angular_velocity.valid)
{
    printf("角速度: %.2f°/s, %.2f°/s, %.2f°/s\n",
           angular_velocity.roll_rate, angular_velocity.pitch_rate, angular_velocity.yaw_rate);
}

// 获取加速度结构体
gyro_app_acceleration_t acceleration;
if (GYRO_APP_Get_Acceleration_Struct(&acceleration) == GYRO_APP_STATUS_OK && acceleration.valid)
{
    printf("加速度: X=%.2f, Y=%.2f, Z=%.2f m/s²\n",
           acceleration.x, acceleration.y, acceleration.z);
}
```

### 传统方式获取数据（兼容旧代码）

### 获取姿态角度

```c
float roll, pitch, yaw;

if (GYRO_APP_Get_Euler_Degrees(&roll, &pitch, &yaw) == GYRO_APP_STATUS_OK)
{
    printf("姿态角度: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\\n", roll, pitch, yaw);
}
```

### 获取角速度

```c
float roll_rate, pitch_rate, yaw_rate;

if (GYRO_APP_Get_Angular_Velocity_Degrees(&roll_rate, &pitch_rate, &yaw_rate) == GYRO_APP_STATUS_OK)
{
    printf("角速度: Roll=%.2f°/s, Pitch=%.2f°/s, Yaw=%.2f°/s\\n", 
           roll_rate, pitch_rate, yaw_rate);
}
```

### 获取加速度

```c
float acc_x, acc_y, acc_z;

if (GYRO_APP_Get_Acceleration(&acc_x, &acc_y, &acc_z) == GYRO_APP_STATUS_OK)
{
    printf("加速度: X=%.2f m/s², Y=%.2f m/s², Z=%.2f m/s²\\n", acc_x, acc_y, acc_z);
}
```

## 中断服务程序

需要在`stm32h7xx_it.c`中添加对应的UART中断服务程序：

```c
// 例如使用UART5
void UART5_IRQHandler(void)
{
    uart_hal.UART_HAL_IRQ_Handler(UART_HAL_CHANNEL_5);
}
```

## 注意事项

1. **时钟配置**: 确保UART时钟已正确配置
2. **波特率**: 陀螺仪默认波特率通常为921600
3. **中断优先级**: UART中断优先级设置为5，可根据需要调整
4. **数据格式**: 支持特定的IMU和AHRS数据帧格式
5. **内存使用**: 每个数据包最大80字节缓冲区

## 故障排除

### 无数据接收

1. 检查UART连接是否正确
2. 检查波特率设置
3. 检查陀螺仪供电
4. 使用示例代码测试

### 数据异常

1. 检查数据帧格式是否匹配
2. 检查UART配置（数据位、停止位、校验位）
3. 检查中断是否正常工作

### 编译错误

1. 确保包含了必要的头文件
2. 检查STM32HAL库版本兼容性
3. 确保路径配置正确

## 版本历史

- v1.0: 初始版本，基于STM32HAL库实现
- 支持STM32H7系列MCU
- 完整的分层架构设计
