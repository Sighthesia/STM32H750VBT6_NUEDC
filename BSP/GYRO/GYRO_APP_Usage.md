/** ***************************************************************************
 * @File Name: GYRO_APP_Usage.md
 * @brief 陀螺仪APP层使用说明
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-26
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes : 按照格式规范修改变量名和结构体类型
 * @Modifi Date : 2025-07-26
 */

# 陀螺仪APP层使用说明

## 修改概述
本次修改按照格式规范对GYRO_APP模块进行了重构，主要变更包括：

### 1. 变量命名规范化
- **结构体类型**: 使用蛇形命名法 (snake_case)
  - `IMUData_Packet_t` → `imu_data_packet_t`
  - `AHRSData_Packet_t` → `ahrs_data_packet_t`

- **全局变量**: 使用蛇形命名法，去除`g_`前缀
  - `g_imu_data_packet` → `imu_data_packet`
  - `g_ahrs_data_packet` → `ahrs_data_packet`
  - `g_ttl_receive_flag` → `ttl_receive_flag`

- **结构体成员**: 使用蛇形命名法
  - `RollSpeed` → `roll_speed`
  - `PitchSpeed` → `pitch_speed`
  - `HeadingSpeed` → `heading_speed`
  - `Qw/Qx/Qy/Qz` → `qw/qx/qy/qz`
  - `Timestamp` → `timestamp`
  - `Pressure` → `pressure`

- **函数参数**: 使用蛇形命名法
  - `Data_1/Data_2/Data_3/Data_4` → `data_1/data_2/data_3/data_4`

### 2. 增加中文注释
为数据结构体中的所有变量增加了详细的中文注释：

```c
typedef struct imu_data_packet_t
{
	float gyroscope_x;			// unit: rad/s    角速度X轴
	float gyroscope_y;			// unit: rad/s    角速度Y轴
	float gyroscope_z;			// unit: rad/s    角速度Z轴
	float accelerometer_x;		// m/s^2          线加速度X轴
	float accelerometer_y;		// m/s^2          线加速度Y轴
	float accelerometer_z;		// m/s^2          线加速度Z轴
	float magnetometer_x;		// mG             磁力计X轴
	float magnetometer_y;		// mG             磁力计Y轴
	float magnetometer_z;		// mG             磁力计Z轴
	float imu_temperature;		// C              IMU温度
	float pressure;				// Pa             气压值
	float pressure_temperature; // C              气压传感器温度
	unsigned int timestamp;		// us             时间戳
} imu_data_packet_t;
```

### 3. 解决UART回调函数问题
为了解决您提到的huart判断问题，提供了两种函数：

#### 方案1：直接调用版本（推荐）
```c
void GYRO_UART_IT_Callback_Process(void);
```
- 不需要huart参数
- 直接在HAL回调中调用此函数
- 更简洁，职责单一

#### 方案2：带判断版本
```c
void GYRO_UART_RxCpltCallback(UART_HandleTypeDef *huart);
```
- 内部自动判断huart句柄
- 只有正确的UART才会处理数据
- 适合多个UART的场景

## 使用方法

### 1. 在HAL_UART_RxCpltCallback中使用

**方案1 - 直接调用（推荐）**：
```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)  // 或者你使用的UART句柄
    {
        GYRO_UART_IT_Callback_Process();
    }
}
```

**方案2 - 使用封装函数**：
```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    GYRO_UART_RxCpltCallback(huart);
}
```

### 2. 数据访问示例

```c
// 检查和处理数据
if (TTL_Hex_To_Dec() == 0)
{
    // 访问IMU数据
    float gyro_x = imu_data_packet.gyroscope_x;
    float accel_x = imu_data_packet.accelerometer_x;
    
    // 访问AHRS数据
    float roll = ahrs_data_packet.roll;
    float pitch = ahrs_data_packet.pitch;
    float heading = ahrs_data_packet.heading;
    
    // 访问四元数
    float qw = ahrs_data_packet.qw;
    float qx = ahrs_data_packet.qx;
    float qy = ahrs_data_packet.qy;
    float qz = ahrs_data_packet.qz;
}
```

### 3. 配置说明
在`GYRO_Config.h`中可以配置：
- `USE_DEBUG`: 是否启用调试输出
- `USE_RS485`: 是否使用RS485通信
- `GYRO_RECEIVE_UART_HANDLE`: 接收UART句柄

## 优势

1. **代码可读性提升**: 变量名更清晰，中文注释便于理解
2. **命名规范统一**: 遵循蛇形命名法，符合C语言规范
3. **接口封装优化**: 提供多种调用方式，适应不同场景
4. **维护性增强**: 结构清晰，便于后续修改和扩展

## 兼容性
- 完全兼容STM32HAL库
- 保持原有功能不变
- 接口向下兼容（提供了两种调用方式）
