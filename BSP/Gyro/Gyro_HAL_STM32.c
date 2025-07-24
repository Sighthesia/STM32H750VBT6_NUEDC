#include "Gyro_HAL.h"
#include "UART_HAL.h"
#include "sys.h"
#include <math.h>
#include <string.h>

/* --------------------------------- 此处修改STM32平台陀螺仪接收配置 -------------------------------- */
// 接收状态机变量
static gyro_hal_status_t g_gyro_status = GYRO_HAL_STATUS_IDLE;
static uart_hal_channel_t g_uart_channel = UART_HAL_CHANNEL_5;
static uint8_t g_rx_buffer[GYRO_HAL_MAX_FRAME_SIZE];
static uint8_t g_rx_count = 0;
static uint8_t g_last_byte = 0;
static gyro_hal_packet_type_t g_current_packet_type = GYRO_HAL_PACKET_TYPE_NONE;

// 数据包缓冲区
static uint8_t g_imu_buffer[GYRO_HAL_MAX_FRAME_SIZE];
static uint8_t g_ahrs_buffer[GYRO_HAL_MAX_FRAME_SIZE];
static uint8_t g_imu_ready = 0;
static uint8_t g_ahrs_ready = 0;

// 回调函数指针
static gyro_hal_imu_callback_t g_imu_callback = NULL;
static gyro_hal_ahrs_callback_t g_ahrs_callback = NULL;

// 内部函数声明
static void UART_RX_Callback(uart_hal_channel_t channel, uint8_t data);
static float DATA_Trans(uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4);
static uint32_t Timestamp_Trans(uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4);
static void Parse_IMU_Data(const uint8_t *buffer);
static void Parse_AHRS_Data(const uint8_t *buffer);
/* 自定义部分结束 */

/** ***************************************************************************
 * @brief UART接收数据回调函数
 * @param channel UART通道号
 * @param data 接收到的数据字节
 */
static void UART_RX_Callback(uart_hal_channel_t channel, uint8_t data)
{
    g_rx_buffer[g_rx_count] = data;

    // 检测帧头
    if (((g_last_byte == GYRO_HAL_FRAME_END) && (data == GYRO_HAL_FRAME_HEAD)) || g_rx_count > 0)
    {
        g_gyro_status = GYRO_HAL_STATUS_RECEIVING;
        g_rx_count++;

        // 检测数据类型
        if (g_rx_count >= 3)
        {
            if ((g_rx_buffer[1] == GYRO_HAL_TYPE_IMU) && (g_rx_buffer[2] == GYRO_HAL_IMU_LEN))
            {
                g_current_packet_type = GYRO_HAL_PACKET_TYPE_IMU;
            }
            else if ((g_rx_buffer[1] == GYRO_HAL_TYPE_AHRS) && (g_rx_buffer[2] == GYRO_HAL_AHRS_LEN))
            {
                g_current_packet_type = GYRO_HAL_PACKET_TYPE_AHRS;
            }
        }
    }
    else
    {
        g_rx_count = 0;
        g_gyro_status = GYRO_HAL_STATUS_IDLE;
    }

    g_last_byte = data;

    // 检查是否接收完整IMU数据包
    if ((g_current_packet_type == GYRO_HAL_PACKET_TYPE_IMU) &&
        (g_rx_count == (GYRO_HAL_IMU_LEN + 8)))
    {
        g_rx_count = 0;
        g_current_packet_type = GYRO_HAL_PACKET_TYPE_NONE;
        g_gyro_status = GYRO_HAL_STATUS_COMPLETE;

        if (g_rx_buffer[GYRO_HAL_IMU_LEN + 8 - 1] == GYRO_HAL_FRAME_END)
        {
            memcpy(g_imu_buffer, g_rx_buffer, GYRO_HAL_IMU_LEN + 8);
            g_imu_ready = 1;
        }
    }

    // 检查是否接收完整AHRS数据包
    if ((g_current_packet_type == GYRO_HAL_PACKET_TYPE_AHRS) &&
        (g_rx_count == (GYRO_HAL_AHRS_LEN + 8)))
    {
        g_rx_count = 0;
        g_current_packet_type = GYRO_HAL_PACKET_TYPE_NONE;
        g_gyro_status = GYRO_HAL_STATUS_COMPLETE;

        if (g_rx_buffer[GYRO_HAL_AHRS_LEN + 8 - 1] == GYRO_HAL_FRAME_END)
        {
            memcpy(g_ahrs_buffer, g_rx_buffer, GYRO_HAL_AHRS_LEN + 8);
            g_ahrs_ready = 1;
        }
    }
}

/** ***************************************************************************
 * @brief 16进制转浮点型数据
 * @param data_1 数据字节1（低位）
 * @param data_2 数据字节2
 * @param data_3 数据字节3
 * @param data_4 数据字节4（高位）
 * @return 转换后的浮点数
 */
static float DATA_Trans(uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4)
{
    long long transition_32;
    float tmp = 0;
    int sign = 0;
    int exponent = 0;
    float mantissa = 0;

    transition_32 = 0;
    transition_32 |= data_4 << 24;
    transition_32 |= data_3 << 16;
    transition_32 |= data_2 << 8;
    transition_32 |= data_1;

    // IEEE 754浮点格式解析
    sign = (transition_32 & 0x80000000) ? -1 : 1;
    exponent = ((transition_32 >> 23) & 0xff) - 127;
    mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
    tmp = sign * mantissa * pow(2, exponent);

    return tmp;
}

/** ***************************************************************************
 * @brief 时间戳转换
 * @param data_1 数据字节1（低位）
 * @param data_2 数据字节2
 * @param data_3 数据字节3
 * @param data_4 数据字节4（高位）
 * @return 时间戳
 */
static uint32_t Timestamp_Trans(uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4)
{
    uint32_t timestamp = 0;
    timestamp |= data_4 << 24;
    timestamp |= data_3 << 16;
    timestamp |= data_2 << 8;
    timestamp |= data_1;
    return timestamp;
}

/** ***************************************************************************
 * @brief 解析IMU数据包
 * @param buffer 数据缓冲区
 */
static void Parse_IMU_Data(const uint8_t *buffer)
{
    if (buffer == NULL || g_imu_callback == NULL)
        return;

    gyro_hal_imu_data_t imu_data;

    // 解析陀螺仪数据
    imu_data.gyroscope_x = DATA_Trans(buffer[7], buffer[8], buffer[9], buffer[10]);
    imu_data.gyroscope_y = DATA_Trans(buffer[11], buffer[12], buffer[13], buffer[14]);
    imu_data.gyroscope_z = DATA_Trans(buffer[15], buffer[16], buffer[17], buffer[18]);

    // 解析加速度计数据
    imu_data.accelerometer_x = DATA_Trans(buffer[19], buffer[20], buffer[21], buffer[22]);
    imu_data.accelerometer_y = DATA_Trans(buffer[23], buffer[24], buffer[25], buffer[26]);
    imu_data.accelerometer_z = DATA_Trans(buffer[27], buffer[28], buffer[29], buffer[30]);

    // 解析磁力计数据
    imu_data.magnetometer_x = DATA_Trans(buffer[31], buffer[32], buffer[33], buffer[34]);
    imu_data.magnetometer_y = DATA_Trans(buffer[35], buffer[36], buffer[37], buffer[38]);
    imu_data.magnetometer_z = DATA_Trans(buffer[39], buffer[40], buffer[41], buffer[42]);

    // 解析时间戳
    imu_data.timestamp = Timestamp_Trans(buffer[55], buffer[56], buffer[57], buffer[58]);

    // 调用回调函数
    g_imu_callback(&imu_data);
}

/** ***************************************************************************
 * @brief 解析AHRS数据包
 * @param buffer 数据缓冲区
 */
static void Parse_AHRS_Data(const uint8_t *buffer)
{
    if (buffer == NULL || g_ahrs_callback == NULL)
        return;

    gyro_hal_ahrs_data_t ahrs_data;

    // 解析角速度
    ahrs_data.roll_speed = DATA_Trans(buffer[7], buffer[8], buffer[9], buffer[10]);
    ahrs_data.pitch_speed = DATA_Trans(buffer[11], buffer[12], buffer[13], buffer[14]);
    ahrs_data.heading_speed = DATA_Trans(buffer[15], buffer[16], buffer[17], buffer[18]);

    // 解析欧拉角
    ahrs_data.roll = DATA_Trans(buffer[19], buffer[20], buffer[21], buffer[22]);
    ahrs_data.pitch = DATA_Trans(buffer[23], buffer[24], buffer[25], buffer[26]);
    ahrs_data.heading = DATA_Trans(buffer[27], buffer[28], buffer[29], buffer[30]);

    // 解析四元数
    ahrs_data.qw = DATA_Trans(buffer[31], buffer[32], buffer[33], buffer[34]);
    ahrs_data.qx = DATA_Trans(buffer[35], buffer[36], buffer[37], buffer[38]);
    ahrs_data.qy = DATA_Trans(buffer[39], buffer[40], buffer[41], buffer[42]);
    ahrs_data.qz = DATA_Trans(buffer[43], buffer[44], buffer[45], buffer[46]);

    // 解析时间戳
    ahrs_data.timestamp = Timestamp_Trans(buffer[47], buffer[48], buffer[49], buffer[50]);

    // 调用回调函数
    g_ahrs_callback(&ahrs_data);
}

/** ***************************************************************************
 * @brief 初始化陀螺仪数据接收
 * @param uart_channel 使用的UART通道
 * @param baud_rate 波特率
 */
static void Init(uart_hal_channel_t uart_channel, uint32_t baud_rate)
{
    uart_hal_config_t uart_config;

    g_uart_channel = uart_channel;

    // 配置UART参数
    uart_config.baud_rate = baud_rate;
    uart_config.data_bits = 8;
    uart_config.stop_bits = 1;
    uart_config.parity = 0;

    // 初始化UART
    uart_hal.UART_HAL_Init(uart_channel, &uart_config);

    // 设置接收回调函数
    uart_hal.UART_HAL_Set_RX_Callback(uart_channel, UART_RX_Callback);

    // 重置状态
    Reset();
}

/** ***************************************************************************
 * @brief 设置IMU数据接收回调函数
 * @param callback IMU数据回调函数
 */
static void Set_IMU_Callback(gyro_hal_imu_callback_t callback)
{
    g_imu_callback = callback;
}

/** ***************************************************************************
 * @brief 设置AHRS数据接收回调函数
 * @param callback AHRS数据回调函数
 */
static void Set_AHRS_Callback(gyro_hal_ahrs_callback_t callback)
{
    g_ahrs_callback = callback;
}

/** ***************************************************************************
 * @brief 启动陀螺仪数据接收
 */
static void Start_Receive(void)
{
    uart_hal.UART_HAL_Enable_RX_IT(g_uart_channel);
}

/** ***************************************************************************
 * @brief 停止陀螺仪数据接收
 */
static void Stop_Receive(void)
{
    uart_hal.UART_HAL_Disable_RX_IT(g_uart_channel);
}

/** ***************************************************************************
 * @brief 处理接收到的数据（供主循环调用）
 * @return 处理的数据包数量
 */
static uint8_t Process_Data(void)
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
 * @brief 获取接收状态
 * @return 当前接收状态
 */
static gyro_hal_status_t Get_Status(void)
{
    return g_gyro_status;
}

/** ***************************************************************************
 * @brief 重置接收状态机
 */
static void Reset(void)
{
    g_gyro_status = GYRO_HAL_STATUS_IDLE;
    g_rx_count = 0;
    g_last_byte = 0;
    g_current_packet_type = GYRO_HAL_PACKET_TYPE_NONE;
    g_imu_ready = 0;
    g_ahrs_ready = 0;

    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_imu_buffer, 0, sizeof(g_imu_buffer));
    memset(g_ahrs_buffer, 0, sizeof(g_ahrs_buffer));
}

// 绑定实现到全局结构体
const gyro_hal_t gyro_hal = {
    .GYRO_HAL_Init = Init,
    .GYRO_HAL_Set_IMU_Callback = Set_IMU_Callback,
    .GYRO_HAL_Set_AHRS_Callback = Set_AHRS_Callback,
    .GYRO_HAL_Start_Receive = Start_Receive,
    .GYRO_HAL_Stop_Receive = Stop_Receive,
    .GYRO_HAL_Process_Data = Process_Data,
    .GYRO_HAL_Get_Status = Get_Status,
    .GYRO_HAL_Reset = Reset,
};
