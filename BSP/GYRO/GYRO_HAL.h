#ifndef GYRO_HAL_H
#define GYRO_HAL_H

#include "main.h"   // 包含主配置头文件（含HAL库、引脚定义等）
#include "stdint.h" // 包含stdint.h以支持uint8_t等类型

#ifdef __cplusplus
extern "C"
{
#endif

    /* --------------------------------- 此处修改陀螺仪数据状态定义 -------------------------------- */
    // 数据包状态
    typedef enum
    {
        GYRO_HAL_STATUS_IDLE = 0,  // 空闲状态
        GYRO_HAL_STATUS_RECEIVING, // 接收中
        GYRO_HAL_STATUS_COMPLETE   // 接收完成
    } gyro_hal_status_t;

    // 数据包类型
    typedef enum
    {
        GYRO_HAL_PACKET_TYPE_NONE = 0, // 无数据包
        GYRO_HAL_PACKET_TYPE_IMU,      // IMU数据包
        GYRO_HAL_PACKET_TYPE_AHRS      // AHRS数据包
    } gyro_hal_packet_type_t;
    /* 自定义部分结束 */

    /** ***************************************************************************
     * @brief IMU数据包结构体
     */
    typedef struct
    {
        float gyroscope_x;     // 陀螺仪X轴角速度 (rad/s)
        float gyroscope_y;     // 陀螺仪Y轴角速度 (rad/s)
        float gyroscope_z;     // 陀螺仪Z轴角速度 (rad/s)
        float accelerometer_x; // 加速度计X轴 (m/s²)
        float accelerometer_y; // 加速度计Y轴 (m/s²)
        float accelerometer_z; // 加速度计Z轴 (m/s²)
        float magnetometer_x;  // 磁力计X轴 (mG)
        float magnetometer_y;  // 磁力计Y轴 (mG)
        float magnetometer_z;  // 磁力计Z轴 (mG)
        uint32_t timestamp;    // 时间戳 (us)
    } gyro_hal_imu_data_t;

    /** ***************************************************************************
     * @brief AHRS数据包结构体
     */
    typedef struct
    {
        float roll_speed;     // 横滚角速度 (rad/s)
        float pitch_speed;    // 俯仰角速度 (rad/s)
        float heading_speed;  // 偏航角速度 (rad/s)
        float roll;           // 横滚角 (rad)
        float pitch;          // 俯仰角 (rad)
        float heading;        // 偏航角 (rad)
        float qw, qx, qy, qz; // 四元数
        uint32_t timestamp;   // 时间戳 (us)
    } gyro_hal_ahrs_data_t;

    // 数据接收完成回调函数类型
    typedef void (*gyro_hal_imu_callback_t)(const gyro_hal_imu_data_t *imu_data);
    typedef void (*gyro_hal_ahrs_callback_t)(const gyro_hal_ahrs_data_t *ahrs_data);

    // 回调函数结构体定义（成员名：首字母大写，缩写全大写，下划线分隔）
    typedef struct
    {
        void (*GYRO_Init)(uint8_t uart_channel, uint32_t baud_rate);
        void (*Set_RX_Callback)(void (*callback)(uint8_t data));
        void (*Start_Receive)(void);
        void (*Stop_Receive)(void);
        void (*Delay_Ms)(uint32_t ms);
    } gyro_hal_t;

    // 外部声明全局HAL实例（结构体变量名：蛇形命名，全小写）
    extern gyro_hal_t gyro_hal;

#ifdef __cplusplus
}
#endif

#endif /* GYRO_HAL_H */
