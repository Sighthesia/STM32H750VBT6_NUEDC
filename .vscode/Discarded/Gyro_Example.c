#include "Gyro_APP.h"
#include <stdio.h>

/* --------------------------------- 此处修改陀螺仪使用示例配置 -------------------------------- */
// 示例使用的数据更新标志
static uint8_t g_print_data = 0;
static uint32_t g_last_print_time = 0;

// 打印间隔（毫秒）
#define PRINT_INTERVAL_MS 1000
/* 自定义部分结束 */

/** ***************************************************************************
 * @brief 陀螺仪使用示例初始化
 */
void Gyro_Example_Init(void)
{
    gyro_app_config_t config;

    // 配置陀螺仪参数
    config.uart_channel = UART_HAL_CHANNEL_5; // 使用UART5
    config.baud_rate = 921600;                // 波特率921600
    config.enable_imu = 1;                    // 启用IMU数据
    config.enable_ahrs = 1;                   // 启用AHRS数据

    // 初始化陀螺仪
    gyro_app_status_t status = GYRO_APP_Init(&config);

    if (status == GYRO_APP_STATUS_OK)
    {
        printf("陀螺仪初始化成功\r\n");
    }
    else
    {
        printf("陀螺仪初始化失败\r\n");
    }
}

/** ***************************************************************************
 * @brief 陀螺仪数据处理示例（在主循环中调用）
 */
void Gyro_Example_Process(void)
{
    // 处理接收到的数据
    uint8_t processed_count = GYRO_APP_Process();

    // 检查是否有新数据
    if (GYRO_APP_Is_Data_Updated())
    {
        g_print_data = 1;
    }

    // 定时打印数据
    uint32_t current_time = HAL_GetTick();
    if (g_print_data && (current_time - g_last_print_time >= PRINT_INTERVAL_MS))
    {
        g_last_print_time = current_time;
        g_print_data = 0;

        Print_Gyro_Data();
    }
}

/** ***************************************************************************
 * @brief 打印陀螺仪数据
 */
void Print_Gyro_Data(void)
{
    gyro_app_processed_data_t processed_data;

    if (GYRO_APP_Get_Processed_Data(&processed_data) == GYRO_APP_STATUS_OK)
    {
        printf("\r\n========== 陀螺仪数据 ==========\r\n");

        // 打印欧拉角数据（如果有效）
        if (processed_data.euler_valid)
        {
            printf("欧拉角 (度): Roll=%.2f, Pitch=%.2f, Yaw=%.2f\r\n",
                   processed_data.euler.roll,
                   processed_data.euler.pitch,
                   processed_data.euler.yaw);
        }

        // 打印角速度数据（如果有效）
        if (processed_data.angular_velocity_valid)
        {
            printf("角速度 (度/秒): Roll=%.2f, Pitch=%.2f, Yaw=%.2f\r\n",
                   processed_data.angular_velocity.roll_rate,
                   processed_data.angular_velocity.pitch_rate,
                   processed_data.angular_velocity.yaw_rate);
        }

        // 打印四元数数据（如果有效）
        if (processed_data.quaternion_valid)
        {
            printf("四元数: W=%.3f, X=%.3f, Y=%.3f, Z=%.3f\r\n",
                   processed_data.quaternion.w,
                   processed_data.quaternion.x,
                   processed_data.quaternion.y,
                   processed_data.quaternion.z);
        }

        // 打印加速度数据（如果有效）
        if (processed_data.acceleration_valid)
        {
            printf("加速度 (m/s²): X=%.2f, Y=%.2f, Z=%.2f\r\n",
                   processed_data.acceleration.x,
                   processed_data.acceleration.y,
                   processed_data.acceleration.z);
        }

        // 打印磁力计数据（如果有效）
        if (processed_data.magnetometer_valid)
        {
            printf("磁力计 (mG): X=%.2f, Y=%.2f, Z=%.2f\r\n",
                   processed_data.magnetometer.x,
                   processed_data.magnetometer.y,
                   processed_data.magnetometer.z);
        }

        printf("数据有效性: 欧拉角=%s, 角速度=%s, 加速度=%s, 磁力计=%s, 四元数=%s\r\n",
               processed_data.euler_valid ? "有效" : "无效",
               processed_data.angular_velocity_valid ? "有效" : "无效",
               processed_data.acceleration_valid ? "有效" : "无效",
               processed_data.magnetometer_valid ? "有效" : "无效",
               processed_data.quaternion_valid ? "有效" : "无效");
        printf("系统时间戳: %lu ms\r\n", processed_data.system_timestamp);
        printf("============================\r\n\r\n");
    }
    else
    {
        printf("获取陀螺仪数据失败\r\n");
    }
} /** ***************************************************************************
   * @brief 获取当前姿态角度（简化接口）
   * @param roll 横滚角指针（度）
   * @param pitch 俯仰角指针（度）
   * @param yaw 偏航角指针（度）
   * @return 获取结果
   */
gyro_app_status_t Get_Current_Attitude(float *roll, float *pitch, float *yaw)
{
    return GYRO_APP_Get_Euler_Degrees(roll, pitch, yaw);
}

/** ***************************************************************************
 * @brief 检查陀螺仪是否正常工作
 * @return 1-正常，0-异常
 */
uint8_t Is_Gyro_Working(void)
{
    gyro_app_data_t data;

    if (GYRO_APP_Get_Data(&data) == GYRO_APP_STATUS_OK)
    {
        // 检查数据是否在合理时间内更新（5秒内）
        uint32_t current_time = HAL_GetTick();
        if ((current_time - data.system_timestamp) < 5000)
        {
            return (data.imu_valid || data.ahrs_valid) ? 1 : 0;
        }
    }

    return 0;
}

/** ***************************************************************************
 * @brief 陀螺仪自检
 * @return 自检结果
 */
gyro_app_status_t Gyro_Self_Test(void)
{
    printf("开始陀螺仪自检...\r\n");

    // 重置陀螺仪
    GYRO_APP_Reset();
    HAL_Delay(100);

    // 等待数据
    uint32_t start_time = HAL_GetTick();
    uint32_t timeout = 10000; // 10秒超时

    while (HAL_GetTick() - start_time < timeout)
    {
        GYRO_APP_Process();

        if (Is_Gyro_Working())
        {
            printf("陀螺仪自检通过\r\n");
            return GYRO_APP_STATUS_OK;
        }

        HAL_Delay(100);
    }

    printf("陀螺仪自检失败 - 超时\r\n");
    return GYRO_APP_STATUS_ERROR;
}

/** ***************************************************************************
 * @brief 演示结构体方式获取陀螺仪数据的示例
 */
void Demo_Struct_Data_Access(void)
{
    printf("\r\n========== 结构体方式数据访问演示 ==========\r\n");

    // 获取欧拉角数据
    gyro_app_euler_t euler;
    if (GYRO_APP_Get_Euler_Struct(&euler) == GYRO_APP_STATUS_OK)
    {
        printf("欧拉角结构体数据:\r\n");
        printf("  Roll: %.2f 度\r\n", euler.roll);
        printf("  Pitch: %.2f 度\r\n", euler.pitch);
        printf("  Yaw: %.2f 度\r\n", euler.yaw);
        printf("  有效性: %s\r\n", euler.valid ? "有效" : "无效");
        printf("  时间戳: %lu ms\r\n", euler.timestamp);
    }

    // 获取角速度数据
    gyro_app_angular_velocity_t angular_velocity;
    if (GYRO_APP_Get_Angular_Velocity_Struct(&angular_velocity) == GYRO_APP_STATUS_OK)
    {
        printf("\r\n角速度结构体数据:\r\n");
        printf("  Roll Rate: %.2f 度/秒\r\n", angular_velocity.roll_rate);
        printf("  Pitch Rate: %.2f 度/秒\r\n", angular_velocity.pitch_rate);
        printf("  Yaw Rate: %.2f 度/秒\r\n", angular_velocity.yaw_rate);
        printf("  有效性: %s\r\n", angular_velocity.valid ? "有效" : "无效");
        printf("  时间戳: %lu ms\r\n", angular_velocity.timestamp);
    }

    // 获取加速度数据
    gyro_app_acceleration_t acceleration;
    if (GYRO_APP_Get_Acceleration_Struct(&acceleration) == GYRO_APP_STATUS_OK)
    {
        printf("\r\n加速度结构体数据:\r\n");
        printf("  X: %.2f m/s²\r\n", acceleration.x);
        printf("  Y: %.2f m/s²\r\n", acceleration.y);
        printf("  Z: %.2f m/s²\r\n", acceleration.z);
        printf("  有效性: %s\r\n", acceleration.valid ? "有效" : "无效");
        printf("  时间戳: %lu ms\r\n", acceleration.timestamp);
    }

    // 获取磁力计数据
    gyro_app_magnetometer_t magnetometer;
    if (GYRO_APP_Get_Magnetometer_Struct(&magnetometer) == GYRO_APP_STATUS_OK)
    {
        printf("\r\n磁力计结构体数据:\r\n");
        printf("  X: %.2f mG\r\n", magnetometer.x);
        printf("  Y: %.2f mG\r\n", magnetometer.y);
        printf("  Z: %.2f mG\r\n", magnetometer.z);
        printf("  有效性: %s\r\n", magnetometer.valid ? "有效" : "无效");
        printf("  时间戳: %lu ms\r\n", magnetometer.timestamp);
    }

    // 获取四元数数据
    gyro_app_quaternion_t quaternion;
    if (GYRO_APP_Get_Quaternion_Struct(&quaternion) == GYRO_APP_STATUS_OK)
    {
        printf("\r\n四元数结构体数据:\r\n");
        printf("  W: %.3f\r\n", quaternion.w);
        printf("  X: %.3f\r\n", quaternion.x);
        printf("  Y: %.3f\r\n", quaternion.y);
        printf("  Z: %.3f\r\n", quaternion.z);
        printf("  有效性: %s\r\n", quaternion.valid ? "有效" : "无效");
        printf("  时间戳: %lu ms\r\n", quaternion.timestamp);
    }

    printf("==========================================\r\n\r\n");
}

/** ***************************************************************************
 * @brief 演示一次性获取所有数据的示例（推荐使用）
 */
void Demo_All_Data_Access(void)
{
    printf("\r\n========== 一次性获取所有数据演示 ==========\r\n");

    gyro_app_processed_data_t all_data;
    if (GYRO_APP_Get_Processed_Data(&all_data) == GYRO_APP_STATUS_OK)
    {
        printf("一次性获取所有传感器数据:\r\n\r\n");

        if (all_data.euler_valid)
        {
            printf("✓ 欧拉角: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\r\n",
                   all_data.euler.roll, all_data.euler.pitch, all_data.euler.yaw);
        }
        else
        {
            printf("✗ 欧拉角数据无效\r\n");
        }

        if (all_data.angular_velocity_valid)
        {
            printf("✓ 角速度: Roll=%.1f°/s, Pitch=%.1f°/s, Yaw=%.1f°/s\r\n",
                   all_data.angular_velocity.roll_rate,
                   all_data.angular_velocity.pitch_rate,
                   all_data.angular_velocity.yaw_rate);
        }
        else
        {
            printf("✗ 角速度数据无效\r\n");
        }

        if (all_data.acceleration_valid)
        {
            printf("✓ 加速度: X=%.2f, Y=%.2f, Z=%.2f m/s²\r\n",
                   all_data.acceleration.x, all_data.acceleration.y, all_data.acceleration.z);
        }
        else
        {
            printf("✗ 加速度数据无效\r\n");
        }

        if (all_data.magnetometer_valid)
        {
            printf("✓ 磁力计: X=%.1f, Y=%.1f, Z=%.1f mG\r\n",
                   all_data.magnetometer.x, all_data.magnetometer.y, all_data.magnetometer.z);
        }
        else
        {
            printf("✗ 磁力计数据无效\r\n");
        }

        if (all_data.quaternion_valid)
        {
            printf("✓ 四元数: W=%.3f, X=%.3f, Y=%.3f, Z=%.3f\r\n",
                   all_data.quaternion.w, all_data.quaternion.x,
                   all_data.quaternion.y, all_data.quaternion.z);
        }
        else
        {
            printf("✗ 四元数数据无效\r\n");
        }

        printf("\r\n数据更新时间: %lu ms\r\n", all_data.system_timestamp);
        printf("距离当前时间: %lu ms\r\n", HAL_GetTick() - all_data.system_timestamp);
    }
    else
    {
        printf("获取数据失败!\r\n");
    }

    printf("==========================================\r\n\r\n");
}

/* --------------------------------- 自定义部分结束 -------------------------------- */
