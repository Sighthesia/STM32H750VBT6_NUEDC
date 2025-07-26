/** ***************************************************************************
 * @File Name: GYRO_APP.c
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-25
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#include "GYRO_APP.h"
#include "GYRO_HAL.h"
#include "GYRO_Config.h"
#include <string.h>
#include <math.h>

/* --------------------------------- 此处修改陀螺仪应用层配置 -------------------------------- */
// 陀螺仪数据存储
static gyro_app_data_t g_gyro_data;
static uint8_t g_data_updated = 0;

// 接收状态机变量
static gyro_hal_status_t g_gyro_status = GYRO_HAL_STATUS_IDLE;
static uint8_t g_rx_buffer[GYRO_MAX_FRAME_SIZE];
static uint8_t g_rx_count = 0;
static uint8_t g_last_byte = 0;
static gyro_hal_packet_type_t g_current_packet_type = GYRO_HAL_PACKET_TYPE_NONE;

// 数据包缓冲区
static uint8_t g_imu_buffer[GYRO_MAX_FRAME_SIZE];
static uint8_t g_ahrs_buffer[GYRO_MAX_FRAME_SIZE];
static uint8_t g_imu_ready = 0;
static uint8_t g_ahrs_ready = 0;

// 配置参数
static gyro_app_config_t g_config = {
    .uart_handle = GYRO_DEFAULT_UART_CHANNEL, // 使用配置文件中的默认值
    .baud_rate = GYRO_DEFAULT_BAUD_RATE,
    .enable_imu = 1,
    .enable_ahrs = 1};

// 内部函数声明
static void UART_RX_Callback(uint8_t data);
static float DATA_Trans(uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4);
static uint32_t Timestamp_Trans(uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4);
static void Parse_IMU_Data(const uint8_t *buffer);
static void Parse_AHRS_Data(const uint8_t *buffer);
static void IMU_Data_Callback(const gyro_hal_imu_data_t *imu_data);
static void AHRS_Data_Callback(const gyro_hal_ahrs_data_t *ahrs_data);
/* 自定义部分结束 */

/** ***************************************************************************
 * @brief UART接收数据回调函数
 * @param data 接收到的数据字节
 */
static void UART_RX_Callback(uint8_t data)
{
    g_rx_buffer[g_rx_count] = data;

    // 检测帧头
    if (((g_last_byte == GYRO_FRAME_END) && (data == GYRO_FRAME_HEAD)) || g_rx_count > 0)
    {
        g_gyro_status = GYRO_HAL_STATUS_RECEIVING;
        g_rx_count++;

        // 检测数据类型
        if (g_rx_count >= 3)
        {
            if (g_rx_buffer[2] == GYRO_TYPE_IMU)
            {
                g_current_packet_type = GYRO_HAL_PACKET_TYPE_IMU;
            }
            else if (g_rx_buffer[2] == GYRO_TYPE_AHRS)
            {
                g_current_packet_type = GYRO_HAL_PACKET_TYPE_AHRS;
            }
        }

        // 检查帧完成
        if (g_current_packet_type == GYRO_HAL_PACKET_TYPE_IMU && g_rx_count >= GYRO_IMU_LEN)
        {
            // IMU数据包接收完成
            memcpy(g_imu_buffer, g_rx_buffer, GYRO_IMU_LEN);
            g_imu_ready = 1;
            g_gyro_status = GYRO_HAL_STATUS_COMPLETE;
            g_rx_count = 0;
            g_current_packet_type = GYRO_HAL_PACKET_TYPE_NONE;
        }
        else if (g_current_packet_type == GYRO_HAL_PACKET_TYPE_AHRS && g_rx_count >= GYRO_AHRS_LEN)
        {
            // AHRS数据包接收完成
            memcpy(g_ahrs_buffer, g_rx_buffer, GYRO_AHRS_LEN);
            g_ahrs_ready = 1;
            g_gyro_status = GYRO_HAL_STATUS_COMPLETE;
            g_rx_count = 0;
            g_current_packet_type = GYRO_HAL_PACKET_TYPE_NONE;
        }

        // 防止缓冲区溢出
        if (g_rx_count >= GYRO_MAX_FRAME_SIZE)
        {
            g_rx_count = 0;
            g_gyro_status = GYRO_HAL_STATUS_IDLE;
            g_current_packet_type = GYRO_HAL_PACKET_TYPE_NONE;
        }
    }

    g_last_byte = data;
}

/** ***************************************************************************
 * @brief 数据转换函数（IEEE 754标准）
 * @param data_1~data_4 4个数据字节
 * @return 转换后的浮点数
 */
static float DATA_Trans(uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4)
{
    union
    {
        float f;
        uint32_t i;
    } u;

    u.i = ((uint32_t)data_4 << 24) | ((uint32_t)data_3 << 16) | ((uint32_t)data_2 << 8) | data_1;
    return u.f;
}

/** ***************************************************************************
 * @brief 时间戳转换函数
 * @param data_1~data_4 4个数据字节
 * @return 转换后的时间戳（微秒）
 */
static uint32_t Timestamp_Trans(uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4)
{
    return ((uint32_t)data_4 << 24) | ((uint32_t)data_3 << 16) | ((uint32_t)data_2 << 8) | data_1;
}

/** ***************************************************************************
 * @brief 解析IMU数据包
 * @param buffer 数据缓冲区
 */
static void Parse_IMU_Data(const uint8_t *buffer)
{
    gyro_hal_imu_data_t imu_data;

    // 解析陀螺仪数据 (rad/s)
    imu_data.gyroscope_x = DATA_Trans(buffer[8], buffer[9], buffer[10], buffer[11]);
    imu_data.gyroscope_y = DATA_Trans(buffer[12], buffer[13], buffer[14], buffer[15]);
    imu_data.gyroscope_z = DATA_Trans(buffer[16], buffer[17], buffer[18], buffer[19]);

    // 解析加速度计数据 (m/s²)
    imu_data.accelerometer_x = DATA_Trans(buffer[20], buffer[21], buffer[22], buffer[23]);
    imu_data.accelerometer_y = DATA_Trans(buffer[24], buffer[25], buffer[26], buffer[27]);
    imu_data.accelerometer_z = DATA_Trans(buffer[28], buffer[29], buffer[30], buffer[31]);

    // 解析磁力计数据 (mG)
    imu_data.magnetometer_x = DATA_Trans(buffer[32], buffer[33], buffer[34], buffer[35]);
    imu_data.magnetometer_y = DATA_Trans(buffer[36], buffer[37], buffer[38], buffer[39]);
    imu_data.magnetometer_z = DATA_Trans(buffer[40], buffer[41], buffer[42], buffer[43]);

    // 解析时间戳 (us)
    imu_data.timestamp = Timestamp_Trans(buffer[44], buffer[45], buffer[46], buffer[47]);

    // 调用处理函数
    IMU_Data_Callback(&imu_data);
}

/** ***************************************************************************
 * @brief 解析AHRS数据包
 * @param buffer 数据缓冲区
 */
static void Parse_AHRS_Data(const uint8_t *buffer)
{
    gyro_hal_ahrs_data_t ahrs_data;

    // 解析角速度数据 (rad/s)
    ahrs_data.roll_speed = DATA_Trans(buffer[8], buffer[9], buffer[10], buffer[11]);
    ahrs_data.pitch_speed = DATA_Trans(buffer[12], buffer[13], buffer[14], buffer[15]);
    ahrs_data.heading_speed = DATA_Trans(buffer[16], buffer[17], buffer[18], buffer[19]);

    // 解析姿态角数据 (rad)
    ahrs_data.roll = DATA_Trans(buffer[20], buffer[21], buffer[22], buffer[23]);
    ahrs_data.pitch = DATA_Trans(buffer[24], buffer[25], buffer[26], buffer[27]);
    ahrs_data.heading = DATA_Trans(buffer[28], buffer[29], buffer[30], buffer[31]);

    // 解析四元数
    ahrs_data.qw = DATA_Trans(buffer[32], buffer[33], buffer[34], buffer[35]);
    ahrs_data.qx = DATA_Trans(buffer[36], buffer[37], buffer[38], buffer[39]);
    ahrs_data.qy = DATA_Trans(buffer[40], buffer[41], buffer[42], buffer[43]);
    ahrs_data.qz = DATA_Trans(buffer[44], buffer[45], buffer[46], buffer[47]);

    // 解析时间戳 (us)
    ahrs_data.timestamp = Timestamp_Trans(buffer[48], buffer[49], buffer[50], buffer[51]);

    // 调用处理函数
    AHRS_Data_Callback(&ahrs_data);
}

/** ***************************************************************************
 * @brief IMU数据接收回调函数
 * @param imu_data IMU数据指针
 */
static void IMU_Data_Callback(const gyro_hal_imu_data_t *imu_data)
{
    if (imu_data == NULL)
        return;

    // 更新IMU数据
    memcpy(&g_gyro_data.imu, imu_data, sizeof(gyro_hal_imu_data_t));
    g_gyro_data.imu_valid = 1;
    g_data_updated = 1;

    // 更新处理后的加速度数据
    g_gyro_data.processed.acceleration.x = imu_data->accelerometer_x;
    g_gyro_data.processed.acceleration.y = imu_data->accelerometer_y;
    g_gyro_data.processed.acceleration.z = imu_data->accelerometer_z;
    g_gyro_data.processed.acceleration_valid = 1;

    // 更新处理后的磁力计数据
    g_gyro_data.processed.magnetometer.x = imu_data->magnetometer_x;
    g_gyro_data.processed.magnetometer.y = imu_data->magnetometer_y;
    g_gyro_data.processed.magnetometer.z = imu_data->magnetometer_z;
    g_gyro_data.processed.magnetometer_valid = 1;

    // 如果没有AHRS数据，使用IMU的陀螺仪数据更新角速度
    if (!g_gyro_data.ahrs_valid)
    {
        g_gyro_data.processed.angular_velocity.roll_rate = imu_data->gyroscope_x * 180.0f / 3.14159265f;
        g_gyro_data.processed.angular_velocity.pitch_rate = imu_data->gyroscope_y * 180.0f / 3.14159265f;
        g_gyro_data.processed.angular_velocity.yaw_rate = imu_data->gyroscope_z * 180.0f / 3.14159265f;
        g_gyro_data.processed.angular_velocity_valid = 1;
    }

    // 更新系统时间戳
    g_gyro_data.system_timestamp = HAL_GetTick();
    g_gyro_data.processed.system_timestamp = g_gyro_data.system_timestamp;
}

/** ***************************************************************************
 * @brief AHRS数据接收回调函数
 * @param ahrs_data AHRS数据指针
 */
static void AHRS_Data_Callback(const gyro_hal_ahrs_data_t *ahrs_data)
{
    if (ahrs_data == NULL)
        return;

    // 更新AHRS数据
    memcpy(&g_gyro_data.ahrs, ahrs_data, sizeof(gyro_hal_ahrs_data_t));
    g_gyro_data.ahrs_valid = 1;
    g_data_updated = 1;

    // 更新处理后的欧拉角数据（转换为度）
    g_gyro_data.processed.euler.roll = ahrs_data->roll * 180.0f / 3.14159265f;
    g_gyro_data.processed.euler.pitch = ahrs_data->pitch * 180.0f / 3.14159265f;
    g_gyro_data.processed.euler.yaw = ahrs_data->heading * 180.0f / 3.14159265f;
    g_gyro_data.processed.euler_valid = 1;

    // 更新处理后的角速度数据（转换为度/秒）
    g_gyro_data.processed.angular_velocity.roll_rate = ahrs_data->roll_speed * 180.0f / 3.14159265f;
    g_gyro_data.processed.angular_velocity.pitch_rate = ahrs_data->pitch_speed * 180.0f / 3.14159265f;
    g_gyro_data.processed.angular_velocity.yaw_rate = ahrs_data->heading_speed * 180.0f / 3.14159265f;
    g_gyro_data.processed.angular_velocity_valid = 1;

    // 更新处理后的四元数数据
    g_gyro_data.processed.quaternion.w = ahrs_data->qw;
    g_gyro_data.processed.quaternion.x = ahrs_data->qx;
    g_gyro_data.processed.quaternion.y = ahrs_data->qy;
    g_gyro_data.processed.quaternion.z = ahrs_data->qz;
    g_gyro_data.processed.quaternion_valid = 1;

    // 更新系统时间戳
    g_gyro_data.system_timestamp = HAL_GetTick();
    g_gyro_data.processed.system_timestamp = g_gyro_data.system_timestamp;
}

/** ***************************************************************************
 * @brief 初始化陀螺仪应用层
 * @param config 配置参数指针
 * @return 初始化结果
 */
gyro_app_status_t GYRO_APP_Init(const gyro_app_config_t *config)
{
    if (config != NULL)
    {
        memcpy(&g_config, config, sizeof(gyro_app_config_t));
    }

    // 清零数据结构
    memset(&g_gyro_data, 0, sizeof(gyro_app_data_t));

    // 重置状态机
    g_gyro_status = GYRO_HAL_STATUS_IDLE;
    g_rx_count = 0;
    g_last_byte = 0;
    g_current_packet_type = GYRO_HAL_PACKET_TYPE_NONE;
    g_imu_ready = 0;
    g_ahrs_ready = 0;
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_imu_buffer, 0, sizeof(g_imu_buffer));
    memset(g_ahrs_buffer, 0, sizeof(g_ahrs_buffer));

    // 初始化HAL层
    gyro_hal.GYRO_Init(g_config.uart_handle, g_config.baud_rate);

    // 设置接收回调函数
    gyro_hal.Set_RX_Callback(UART_RX_Callback);

    // 启动接收
    gyro_hal.Start_Receive();

    return GYRO_APP_STATUS_OK;
}

/** ***************************************************************************
 * @brief 启动陀螺仪数据接收
 */
void GYRO_APP_Start(void)
{
    gyro_hal.Start_Receive();
}

/** ***************************************************************************
 * @brief 停止陀螺仪数据接收
 */
void GYRO_APP_Stop(void)
{
    gyro_hal.Stop_Receive();
}

/** ***************************************************************************
 * @brief 处理陀螺仪数据（应在主循环中调用）
 * @return 处理的数据包数量
 */
uint8_t GYRO_APP_Process(void)
{
    uint8_t processed_count = 0;

    // 处理IMU数据
    if (g_imu_ready)
    {
        Parse_IMU_Data(g_imu_buffer);
        g_imu_ready = 0;
        processed_count++;
    }

    // 处理AHRS数据
    if (g_ahrs_ready)
    {
        Parse_AHRS_Data(g_ahrs_buffer);
        g_ahrs_ready = 0;
        processed_count++;
    }

    return processed_count;
}

/** ***************************************************************************
 * @brief 获取陀螺仪数据
 * @param data 数据结构指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Data(gyro_app_data_t *data)
{
    if (data == NULL)
        return GYRO_APP_STATUS_ERROR;

    memcpy(data, &g_gyro_data, sizeof(gyro_app_data_t));
    return GYRO_APP_STATUS_OK;
}

/** ***************************************************************************
 * @brief 检查数据是否已更新
 * @return 1-已更新，0-未更新
 */
uint8_t GYRO_APP_Is_Data_Updated(void)
{
    uint8_t updated = g_data_updated;
    g_data_updated = 0; // 清除更新标志
    return updated;
}

/** ***************************************************************************
 * @brief 获取IMU数据
 * @param imu_data IMU数据指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_IMU_Data(gyro_hal_imu_data_t *imu_data)
{
    if (imu_data == NULL || !g_gyro_data.imu_valid)
        return GYRO_APP_STATUS_ERROR;

    memcpy(imu_data, &g_gyro_data.imu, sizeof(gyro_hal_imu_data_t));
    return GYRO_APP_STATUS_OK;
}

/** ***************************************************************************
 * @brief 获取AHRS数据
 * @param ahrs_data AHRS数据指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_AHRS_Data(gyro_hal_ahrs_data_t *ahrs_data)
{
    if (ahrs_data == NULL || !g_gyro_data.ahrs_valid)
        return GYRO_APP_STATUS_ERROR;

    memcpy(ahrs_data, &g_gyro_data.ahrs, sizeof(gyro_hal_ahrs_data_t));
    return GYRO_APP_STATUS_OK;
}

/** ***************************************************************************
 * @brief 获取欧拉角（度）
 * @param roll 横滚角指针
 * @param pitch 俯仰角指针
 * @param yaw 偏航角指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Euler_Degrees(float *roll, float *pitch, float *yaw)
{
    if (roll == NULL || pitch == NULL || yaw == NULL || !g_gyro_data.ahrs_valid)
        return GYRO_APP_STATUS_ERROR;

    *roll = g_gyro_data.ahrs.roll * 180.0f / 3.14159265f;
    *pitch = g_gyro_data.ahrs.pitch * 180.0f / 3.14159265f;
    *yaw = g_gyro_data.ahrs.heading * 180.0f / 3.14159265f;

    return GYRO_APP_STATUS_OK;
}

/** ***************************************************************************
 * @brief 获取角速度（度/秒）
 * @param roll_rate 横滚角速度指针
 * @param pitch_rate 俯仰角速度指针
 * @param yaw_rate 偏航角速度指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Angular_Velocity_Degrees(float *roll_rate, float *pitch_rate, float *yaw_rate)
{
    if (roll_rate == NULL || pitch_rate == NULL || yaw_rate == NULL)
        return GYRO_APP_STATUS_ERROR;

    if (g_gyro_data.ahrs_valid)
    {
        *roll_rate = g_gyro_data.ahrs.roll_speed * 180.0f / 3.14159265f;
        *pitch_rate = g_gyro_data.ahrs.pitch_speed * 180.0f / 3.14159265f;
        *yaw_rate = g_gyro_data.ahrs.heading_speed * 180.0f / 3.14159265f;
    }
    else if (g_gyro_data.imu_valid)
    {
        *roll_rate = g_gyro_data.imu.gyroscope_x * 180.0f / 3.14159265f;
        *pitch_rate = g_gyro_data.imu.gyroscope_y * 180.0f / 3.14159265f;
        *yaw_rate = g_gyro_data.imu.gyroscope_z * 180.0f / 3.14159265f;
    }
    else
    {
        return GYRO_APP_STATUS_ERROR;
    }

    return GYRO_APP_STATUS_OK;
}

/** ***************************************************************************
 * @brief 获取加速度（m/s²）
 * @param acc_x X轴加速度指针
 * @param acc_y Y轴加速度指针
 * @param acc_z Z轴加速度指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Acceleration(float *acc_x, float *acc_y, float *acc_z)
{
    if (acc_x == NULL || acc_y == NULL || acc_z == NULL || !g_gyro_data.imu_valid)
        return GYRO_APP_STATUS_ERROR;

    *acc_x = g_gyro_data.imu.accelerometer_x;
    *acc_y = g_gyro_data.imu.accelerometer_y;
    *acc_z = g_gyro_data.imu.accelerometer_z;

    return GYRO_APP_STATUS_OK;
}

/** ***************************************************************************
 * @brief 获取磁力计数据（mG）
 * @param mag_x X轴磁力计指针
 * @param mag_y Y轴磁力计指针
 * @param mag_z Z轴磁力计指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Magnetometer(float *mag_x, float *mag_y, float *mag_z)
{
    if (mag_x == NULL || mag_y == NULL || mag_z == NULL || !g_gyro_data.imu_valid)
        return GYRO_APP_STATUS_ERROR;

    *mag_x = g_gyro_data.imu.magnetometer_x;
    *mag_y = g_gyro_data.imu.magnetometer_y;
    *mag_z = g_gyro_data.imu.magnetometer_z;

    return GYRO_APP_STATUS_OK;
}

/** ***************************************************************************
 * @brief 获取四元数
 * @param qw W分量指针
 * @param qx X分量指针
 * @param qy Y分量指针
 * @param qz Z分量指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Quaternion(float *qw, float *qx, float *qy, float *qz)
{
    if (qw == NULL || qx == NULL || qy == NULL || qz == NULL || !g_gyro_data.ahrs_valid)
        return GYRO_APP_STATUS_ERROR;

    *qw = g_gyro_data.ahrs.qw;
    *qx = g_gyro_data.ahrs.qx;
    *qy = g_gyro_data.ahrs.qy;
    *qz = g_gyro_data.ahrs.qz;

    return GYRO_APP_STATUS_OK;
}

// /** ***************************************************************************
//  * @brief 获取接收状态
//  * @return 接收状态
//  */
// gyro_hal_status_t GYRO_APP_Get_Status(void)
// {
//     return gyro_hal.GYRO_HAL_Get_Status();
// }

// /** ***************************************************************************
//  * @brief 重置陀螺仪
//  */
// void GYRO_APP_Reset(void)
// {
//     gyro_hal.GYRO_HAL_Reset();
//     memset(&g_gyro_data, 0, sizeof(gyro_app_data_t));
//     g_data_updated = 0;
// }

/** ***************************************************************************
 * @brief 获取处理后的陀螺仪数据（结构体方式）
 * @param processed_data 处理后的数据结构指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Processed_Data(gyro_app_processed_data_t *processed_data)
{
    if (processed_data == NULL)
        return GYRO_APP_STATUS_ERROR;

    memcpy(processed_data, &g_gyro_data.processed, sizeof(gyro_app_processed_data_t));
    return GYRO_APP_STATUS_OK;
}

/** ***************************************************************************
 * @brief 获取欧拉角结构体
 * @param euler 欧拉角结构体指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Euler_Struct(gyro_app_euler_t *euler)
{
    if (euler == NULL || !g_gyro_data.processed.euler_valid)
        return GYRO_APP_STATUS_ERROR;

    memcpy(euler, &g_gyro_data.processed.euler, sizeof(gyro_app_euler_t));
    return GYRO_APP_STATUS_OK;
}

/** ***************************************************************************
 * @brief 获取角速度结构体
 * @param angular_velocity 角速度结构体指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Angular_Velocity_Struct(gyro_app_angular_velocity_t *angular_velocity)
{
    if (angular_velocity == NULL || !g_gyro_data.processed.angular_velocity_valid)
        return GYRO_APP_STATUS_ERROR;

    memcpy(angular_velocity, &g_gyro_data.processed.angular_velocity, sizeof(gyro_app_angular_velocity_t));
    return GYRO_APP_STATUS_OK;
}

/** ***************************************************************************
 * @brief 获取加速度结构体
 * @param acceleration 加速度结构体指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Acceleration_Struct(gyro_app_acceleration_t *acceleration)
{
    if (acceleration == NULL || !g_gyro_data.processed.acceleration_valid)
        return GYRO_APP_STATUS_ERROR;

    memcpy(acceleration, &g_gyro_data.processed.acceleration, sizeof(gyro_app_acceleration_t));
    return GYRO_APP_STATUS_OK;
}

/** ***************************************************************************
 * @brief 获取磁力计结构体
 * @param magnetometer 磁力计结构体指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Magnetometer_Struct(gyro_app_magnetometer_t *magnetometer)
{
    if (magnetometer == NULL || !g_gyro_data.processed.magnetometer_valid)
        return GYRO_APP_STATUS_ERROR;

    memcpy(magnetometer, &g_gyro_data.processed.magnetometer, sizeof(gyro_app_magnetometer_t));
    return GYRO_APP_STATUS_OK;
}

/** ***************************************************************************
 * @brief 获取四元数结构体
 * @param quaternion 四元数结构体指针
 * @return 获取结果
 */
gyro_app_status_t GYRO_APP_Get_Quaternion_Struct(gyro_app_quaternion_t *quaternion)
{
    if (quaternion == NULL || !g_gyro_data.processed.quaternion_valid)
        return GYRO_APP_STATUS_ERROR;

    memcpy(quaternion, &g_gyro_data.processed.quaternion, sizeof(gyro_app_quaternion_t));
    return GYRO_APP_STATUS_OK;
}
