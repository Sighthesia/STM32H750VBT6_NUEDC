#ifndef __GYRO_APP_H
#define __GYRO_APP_H

#include "Gyro_HAL.h"
#include "main.h"

/* --------------------------------- 此处修改陀螺仪应用层配置 -------------------------------- */
// 应用层状态枚举
typedef enum
{
    GYRO_APP_STATUS_OK = 0,
    GYRO_APP_STATUS_ERROR,
    GYRO_APP_STATUS_INIT_FAILED,
    GYRO_APP_STATUS_NO_DATA
} gyro_app_status_t;

// 陀螺仪应用层配置结构体
typedef struct
{
    uint8_t uart_channel;
    uint32_t baud_rate;
    uint8_t enable_imu;
    uint8_t enable_ahrs;
} gyro_app_config_t;

// 姿态角度结构体（度）
typedef struct
{
    float roll;  // 横滚角
    float pitch; // 俯仰角
    float yaw;   // 偏航角
} gyro_app_euler_t;

// 角速度结构体（度/秒）
typedef struct
{
    float roll_rate;  // 横滚角速度
    float pitch_rate; // 俯仰角速度
    float yaw_rate;   // 偏航角速度
} gyro_app_angular_velocity_t;

// 加速度结构体（m/s²）
typedef struct
{
    float x; // X轴加速度
    float y; // Y轴加速度
    float z; // Z轴加速度
} gyro_app_acceleration_t;

// 磁力计结构体（mG）
typedef struct
{
    float x; // X轴磁力计
    float y; // Y轴磁力计
    float z; // Z轴磁力计
} gyro_app_magnetometer_t;

// 四元数结构体
typedef struct
{
    float w; // W分量
    float x; // X分量
    float y; // Y分量
    float z; // Z分量
} gyro_app_quaternion_t;

// 处理后的陀螺仪数据结构体
typedef struct
{
    gyro_app_euler_t euler;                       // 欧拉角（度）
    gyro_app_angular_velocity_t angular_velocity; // 角速度（度/秒）
    gyro_app_acceleration_t acceleration;         // 加速度（m/s²）
    gyro_app_magnetometer_t magnetometer;         // 磁力计（mG）
    gyro_app_quaternion_t quaternion;             // 四元数
    uint8_t euler_valid;                          // 欧拉角数据有效标志
    uint8_t angular_velocity_valid;               // 角速度数据有效标志
    uint8_t acceleration_valid;                   // 加速度数据有效标志
    uint8_t magnetometer_valid;                   // 磁力计数据有效标志
    uint8_t quaternion_valid;                     // 四元数数据有效标志
    uint32_t system_timestamp;                    // 系统时间戳
} gyro_app_processed_data_t;

// 陀螺仪综合数据结构体
typedef struct
{
    gyro_hal_imu_data_t imu;
    gyro_hal_ahrs_data_t ahrs;
    gyro_app_processed_data_t processed; // 处理后的数据
    uint8_t imu_valid;
    uint8_t ahrs_valid;
    uint32_t system_timestamp;
} gyro_app_data_t;
/* 自定义部分结束 */

/** ***************************************************************************
 * @brief 初始化陀螺仪应用层
 * @param config 配置参数指针
 * @return 初始化结果
 */
gyro_app_status_t GYRO_APP_Init(const gyro_app_config_t *config);

/** ***************************************************************************
 * @brief 启动陀螺仪数据接收
 */
void GYRO_APP_Start(void);

/** ***************************************************************************
 * @brief 停止陀螺仪数据接收
 */
void GYRO_APP_Stop(void);

/** ***************************************************************************
 * @brief 处理陀螺仪数据（应在主循环中调用）
 * @return 处理的数据包数量
 */
uint8_t GYRO_APP_Process(void);

/** ***************************************************************************
 * @brief 获取陀螺仪数据
 * @param data 数据结构指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Data(gyro_app_data_t *data);

/** ***************************************************************************
 * @brief 检查数据是否已更新
 * @return 1-已更新，0-未更新
 */
uint8_t GYRO_APP_Is_Data_Updated(void);

/** ***************************************************************************
 * @brief 获取IMU数据
 * @param imu_data IMU数据指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_IMU_Data(gyro_hal_imu_data_t *imu_data);

/** ***************************************************************************
 * @brief 获取AHRS数据
 * @param ahrs_data AHRS数据指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_AHRS_Data(gyro_hal_ahrs_data_t *ahrs_data);

/** ***************************************************************************
 * @brief 获取欧拉角（度）
 * @param roll 横滚角指针
 * @param pitch 俯仰角指针
 * @param yaw 偏航角指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Euler_Degrees(float *roll, float *pitch, float *yaw);

/** ***************************************************************************
 * @brief 获取角速度（度/秒）
 * @param roll_rate 横滚角速度指针
 * @param pitch_rate 俯仰角速度指针
 * @param yaw_rate 偏航角速度指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Angular_Velocity_Degrees(float *roll_rate, float *pitch_rate, float *yaw_rate);

/** ***************************************************************************
 * @brief 获取加速度（m/s²）
 * @param acc_x X轴加速度指针
 * @param acc_y Y轴加速度指针
 * @param acc_z Z轴加速度指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Acceleration(float *acc_x, float *acc_y, float *acc_z);

/** ***************************************************************************
 * @brief 获取磁力计数据（mG）
 * @param mag_x X轴磁力计指针
 * @param mag_y Y轴磁力计指针
 * @param mag_z Z轴磁力计指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Magnetometer(float *mag_x, float *mag_y, float *mag_z);

/** ***************************************************************************
 * @brief 获取四元数
 * @param qw W分量指针
 * @param qx X分量指针
 * @param qy Y分量指针
 * @param qz Z分量指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Quaternion(float *qw, float *qx, float *qy, float *qz);

/** ***************************************************************************
 * @brief 获取处理后的陀螺仪数据（结构体方式）
 * @param processed_data 处理后的数据结构指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Processed_Data(gyro_app_processed_data_t *processed_data);

/** ***************************************************************************
 * @brief 获取欧拉角结构体
 * @param euler 欧拉角结构体指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Euler_Struct(gyro_app_euler_t *euler);

/** ***************************************************************************
 * @brief 获取角速度结构体
 * @param angular_velocity 角速度结构体指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Angular_Velocity_Struct(gyro_app_angular_velocity_t *angular_velocity);

/** ***************************************************************************
 * @brief 获取加速度结构体
 * @param acceleration 加速度结构体指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Acceleration_Struct(gyro_app_acceleration_t *acceleration);

/** ***************************************************************************
 * @brief 获取磁力计结构体
 * @param magnetometer 磁力计结构体指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Magnetometer_Struct(gyro_app_magnetometer_t *magnetometer);

/** ***************************************************************************
 * @brief 获取四元数结构体
 * @param quaternion 四元数结构体指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Quaternion_Struct(gyro_app_quaternion_t *quaternion);

/** ***************************************************************************
 * @brief 获取接收状态
 * @return 接收状态
 */
gyro_hal_status_t GYRO_APP_Get_Status(void);

/** ***************************************************************************
 * @brief 重置陀螺仪
 */
void GYRO_APP_Reset(void);

#endif /* __GYRO_APP_H */
