#ifndef __GYRO_HAL_H
#define __GYRO_HAL_H

#include <stdint.h>
#include "UART_HAL.h"

/* --------------------------------- 此处修改陀螺仪数据接收配置 -------------------------------- */
// 帧格式定义
#define GYRO_HAL_FRAME_HEAD 0xFC // 帧头
#define GYRO_HAL_FRAME_END 0xFD  // 帧尾
#define GYRO_HAL_TYPE_IMU 0x40   // IMU数据类型
#define GYRO_HAL_TYPE_AHRS 0x41  // AHRS数据类型
#define GYRO_HAL_IMU_LEN 0x38    // IMU数据长度（56+8字节）
#define GYRO_HAL_AHRS_LEN 0x30   // AHRS数据长度（48+8字节）

// 数据缓冲区大小
#define GYRO_HAL_MAX_FRAME_SIZE 80 // 最大帧长度

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

/** ***************************************************************************
 * @brief 陀螺仪数据接收硬件抽象层函数指针结构体
 *
 * 提供统一的陀螺仪数据接收接口，支持IMU和AHRS数据解析
 */
typedef struct
{
    /** ***************************************************************************
     * @brief 初始化陀螺仪数据接收
     * @param uart_channel 使用的UART通道
     * @param baud_rate 波特率
     */
    void (*GYRO_HAL_Init)(uart_hal_channel_t uart_channel, uint32_t baud_rate);

    /** ***************************************************************************
     * @brief 设置IMU数据接收回调函数
     * @param callback IMU数据回调函数
     */
    void (*GYRO_HAL_Set_IMU_Callback)(gyro_hal_imu_callback_t callback);

    /** ***************************************************************************
     * @brief 设置AHRS数据接收回调函数
     * @param callback AHRS数据回调函数
     */
    void (*GYRO_HAL_Set_AHRS_Callback)(gyro_hal_ahrs_callback_t callback);

    /** ***************************************************************************
     * @brief 启动陀螺仪数据接收
     */
    void (*GYRO_HAL_Start_Receive)(void);

    /** ***************************************************************************
     * @brief 停止陀螺仪数据接收
     */
    void (*GYRO_HAL_Stop_Receive)(void);

    /** ***************************************************************************
     * @brief 处理接收到的数据（供主循环调用）
     * @return 处理的数据包数量
     */
    uint8_t (*GYRO_HAL_Process_Data)(void);

    /** ***************************************************************************
     * @brief 获取接收状态
     * @return 当前接收状态
     */
    gyro_hal_status_t (*GYRO_HAL_Get_Status)(void);

    /** ***************************************************************************
     * @brief 重置接收状态机
     */
    void (*GYRO_HAL_Reset)(void);

} gyro_hal_t;

/// 全局实例（由具体平台实现）
extern const gyro_hal_t gyro_hal;

#endif /* __GYRO_HAL_H */
