# 串口接收陀螺仪数据HAL封装使用说明

## 概述

本HAL封装将原有的串口接收陀螺仪数据功能抽象化，提供统一的接口，便于移植和维护。

## 文件结构

```
HARDWARE/
├── UART_HAL.h              # UART硬件抽象层接口定义
├── UART_HAL_STM32.c        # STM32平台UART HAL实现
├── Gyro_HAL.h              # 陀螺仪数据接收HAL接口定义
├── GYRO_HAL_STM32.c        # STM32平台陀螺仪HAL实现
└── gyro_app.c              # 应用层示例代码

USER/
└── main_hal.c              # 使用HAL封装的主函数示例
```

## 核心特性

### 1. UART HAL (UART_HAL.h / UART_HAL_STM32.c)

**功能特点：**
- 支持多达5个UART通道（UART1~UART5）
- 统一的初始化、发送、接收接口
- 中断驱动的数据接收
- 回调函数机制处理接收数据

**主要接口：**
```c
// 初始化UART通道
uart_hal.UART_HAL_Init(channel, config);

// 发送数据
uart_hal.UART_HAL_Send_Byte(channel, data);
uart_hal.UART_HAL_Send_Buffer(channel, buffer, length);

// 设置接收回调
uart_hal.UART_HAL_Set_RX_Callback(channel, callback_function);

// 中断控制
uart_hal.UART_HAL_Enable_RX_IT(channel);
uart_hal.UART_HAL_Disable_RX_IT(channel);
```

### 2. 陀螺仪数据接收HAL (Gyro_HAL.h / GYRO_HAL_STM32.c)

**功能特点：**
- 自动识别IMU和AHRS数据包类型
- 完整的帧检测和校验机制
- IEEE 754浮点数据转换
- 分离的IMU和AHRS数据回调

**主要接口：**
```c
// 初始化陀螺仪接收系统
gyro_hal.GYRO_HAL_Init(uart_channel, baud_rate);

// 设置数据回调函数
gyro_hal.GYRO_HAL_Set_IMU_Callback(imu_callback);
gyro_hal.GYRO_HAL_Set_AHRS_Callback(ahrs_callback);

// 控制接收
gyro_hal.GYRO_HAL_Start_Receive();
gyro_hal.GYRO_HAL_Stop_Receive();

// 主循环处理（非阻塞）
uint8_t count = gyro_hal.GYRO_HAL_Process_Data();
```

## 使用方法

### 1. 基本初始化

```c
#include "UART_HAL.h"
#include "Gyro_HAL.h"

void system_init(void)
{
    // 初始化陀螺仪接收（UART5，波特率921600）
    gyro_hal.GYRO_HAL_Init(UART_HAL_CHANNEL_5, 921600);
    
    // 设置数据处理回调函数
    gyro_hal.GYRO_HAL_Set_IMU_Callback(my_imu_callback);
    gyro_hal.GYRO_HAL_Set_AHRS_Callback(my_ahrs_callback);
    
    // 启动接收
    gyro_hal.GYRO_HAL_Start_Receive();
}
```

### 2. 数据处理回调函数

```c
// IMU数据处理
void my_imu_callback(const gyro_hal_imu_data_t *imu_data)
{
    printf("陀螺仪 X: %f rad/s\n", imu_data->gyroscope_x);
    printf("加速度 X: %f m/s²\n", imu_data->accelerometer_x);
    printf("磁力计 X: %f mG\n", imu_data->magnetometer_x);
    // ... 处理其他数据
}

// AHRS数据处理
void my_ahrs_callback(const gyro_hal_ahrs_data_t *ahrs_data)
{
    printf("横滚角: %f rad\n", ahrs_data->roll);
    printf("俯仰角: %f rad\n", ahrs_data->pitch);
    printf("偏航角: %f rad\n", ahrs_data->heading);
    // ... 处理其他数据
}
```

### 3. 主循环

```c
int main(void)
{
    system_init();
    
    while(1)
    {
        // 处理接收到的数据包（非阻塞）
        gyro_hal.GYRO_HAL_Process_Data();
        
        // 其他应用逻辑...
        delay_ms(1);
    }
}
```

## 数据格式

### IMU数据包 (gyro_hal_imu_data_t)
- **陀螺仪数据**：gyroscope_x/y/z (rad/s)
- **加速度计数据**：accelerometer_x/y/z (m/s²)
- **磁力计数据**：magnetometer_x/y/z (mG)
- **时间戳**：timestamp (us)

### AHRS数据包 (gyro_hal_ahrs_data_t)
- **角速度**：roll_speed/pitch_speed/heading_speed (rad/s)
- **欧拉角**：roll/pitch/heading (rad)
- **四元数**：qw/qx/qy/qz
- **时间戳**：timestamp (us)

## 移植指南

如需移植到其他平台（如ESP32、Arduino等），只需：

1. **保持接口文件不变**：`UART_HAL.h`、`Gyro_HAL.h`
2. **创建新的平台实现**：
   - `UART_HAL_ESP32.c` - ESP32平台UART实现
   - `GYRO_HAL_ESP32.c` - ESP32平台陀螺仪实现
3. **编译时包含对应平台文件**

## 优势

1. **零开销抽象**：通过函数指针直接调用，无性能损失
2. **接口统一**：所有平台使用相同的API
3. **易于维护**：平台相关代码分离，便于调试和更新
4. **高度可移植**：只需实现平台相关部分即可移植
5. **模块化设计**：UART和陀螺仪功能分离，便于复用

## 注意事项

1. 确保在中断服务程序中调用 `uart_hal.UART_HAL_IRQ_Handler(channel)`
2. 主循环中定期调用 `gyro_hal.GYRO_HAL_Process_Data()` 处理数据
3. 回调函数中避免长时间阻塞操作
4. 根据实际硬件连接修改GPIO配置部分
