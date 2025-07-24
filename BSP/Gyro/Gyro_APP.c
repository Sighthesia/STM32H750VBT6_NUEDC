#include "UART_HAL.h"
#include "Gyro_HAL.h"
#include <stdio.h>

/* --------------------------------- 此处修改应用层配置 -------------------------------- */
// 使用UART5接收陀螺仪数据，波特率921600
#define GYRO_UART_CHANNEL UART_HAL_CHANNEL_5
#define GYRO_UART_BAUD 921600

// 使用UART1输出数据，波特率115200
#define OUTPUT_UART_CHANNEL UART_HAL_CHANNEL_1
#define OUTPUT_UART_BAUD 115200
/* 自定义部分结束 */

/** ***************************************************************************
 * @brief IMU数据接收回调函数
 * @param imu_data IMU数据结构体指针
 */
static void IMU_Data_Callback(const gyro_hal_imu_data_t *imu_data)
{
    printf("**********    IMU Data    **********\r\n");
    printf("Gyroscope X: %f rad/s\r\n", imu_data->gyroscope_x);
    printf("Gyroscope Y: %f rad/s\r\n", imu_data->gyroscope_y);
    printf("Gyroscope Z: %f rad/s\r\n", imu_data->gyroscope_z);
    printf("Accelerometer X: %f m/s²\r\n", imu_data->accelerometer_x);
    printf("Accelerometer Y: %f m/s²\r\n", imu_data->accelerometer_y);
    printf("Accelerometer Z: %f m/s²\r\n", imu_data->accelerometer_z);
    printf("Magnetometer X: %f mG\r\n", imu_data->magnetometer_x);
    printf("Magnetometer Y: %f mG\r\n", imu_data->magnetometer_y);
    printf("Magnetometer Z: %f mG\r\n", imu_data->magnetometer_z);
    printf("Timestamp: %d us\r\n", imu_data->timestamp);
    printf("**********************************\r\n\r\n");
}

/** ***************************************************************************
 * @brief AHRS数据接收回调函数
 * @param ahrs_data AHRS数据结构体指针
 */
static void AHRS_Data_Callback(const gyro_hal_ahrs_data_t *ahrs_data)
{
    printf("**********   AHRS Data    **********\r\n");
    printf("Roll Speed: %f rad/s\r\n", ahrs_data->roll_speed);
    printf("Pitch Speed: %f rad/s\r\n", ahrs_data->pitch_speed);
    printf("Heading Speed: %f rad/s\r\n", ahrs_data->heading_speed);
    printf("Roll: %f rad\r\n", ahrs_data->roll);
    printf("Pitch: %f rad\r\n", ahrs_data->pitch);
    printf("Heading: %f rad\r\n", ahrs_data->heading);
    printf("Quaternion W: %f\r\n", ahrs_data->qw);
    printf("Quaternion X: %f\r\n", ahrs_data->qx);
    printf("Quaternion Y: %f\r\n", ahrs_data->qy);
    printf("Quaternion Z: %f\r\n", ahrs_data->qz);
    printf("Timestamp: %d us\r\n", ahrs_data->timestamp);
    printf("**********************************\r\n\r\n");
}

/** ***************************************************************************
 * @brief 初始化陀螺仪接收系统
 */
void Gyro_System_Init(void)
{
    uart_hal_config_t output_uart_config;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    LED_Init();
    delay_init();

    // 初始化输出串口
    output_uart_config.baud_rate = OUTPUT_UART_BAUD;
    output_uart_config.data_bits = 8;
    output_uart_config.stop_bits = 1;
    output_uart_config.parity = 0;
    uart_hal.UART_HAL_Init(OUTPUT_UART_CHANNEL, &output_uart_config);

    // 初始化陀螺仪接收
    gyro_hal.GYRO_HAL_Init(GYRO_UART_CHANNEL, GYRO_UART_BAUD);

    // 设置数据回调函数
    gyro_hal.GYRO_HAL_Set_IMU_Callback(IMU_Data_Callback);
    gyro_hal.GYRO_HAL_Set_AHRS_Callback(AHRS_Data_Callback);

    // 启动接收
    gyro_hal.GYRO_HAL_Start_Receive();

    printf("Gyroscope data receiving system initialized.\r\n");
    printf("Waiting for IMU/AHRS data...\r\n\r\n");
}

/** ***************************************************************************
 * @brief 主循环处理函数
 */
void Gyro_System_Process(void)
{
    // 处理接收到的陀螺仪数据
    uint8_t processed_count = gyro_hal.GYRO_HAL_Process_Data();

    // 可选：显示接收状态
    static uint32_t status_counter = 0;
    if (++status_counter >= 100000)
    { // 降低状态输出频率
        status_counter = 0;

        gyro_hal_status_t status = gyro_hal.GYRO_HAL_Get_Status();
        switch (status)
        {
        case GYRO_HAL_STATUS_IDLE:
            // printf("Status: IDLE\r\n");
            break;
        case GYRO_HAL_STATUS_RECEIVING:
            printf("Status: RECEIVING...\r\n");
            break;
        case GYRO_HAL_STATUS_COMPLETE:
            printf("Status: PACKET COMPLETE\r\n");
            break;
        }
    }
}
