/** ***************************************************************************
 * @File Name: GYRO_APP.c
 * @brief 陀螺仪应用层实现文件
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-26
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes : 按照格式规范修改变量名和结构体类型
 * @Modifi Date : 2025-07-26
 */
#include "GYRO_APP.h"

/* -------------------------------- 此处修改全局变量定义 ------------------------------- */
imu_data_packet_t imu_data_packet;
ahrs_data_packet_t ahrs_data_packet;
uint8_t ttl_receive_flag;
uint8_t frame_data[64];
uint8_t imu_frame_data[64];
uint8_t ahrs_frame_data[56];
int imu_data_ready_flag = 0;
int ahrs_data_ready_flag = 0;
extern int Time_count;
/* ---------------------------------- 自定义部分结束 --------------------------------- */

/** ***************************************************************************
 * @brief 陀螺仪UART中断回调处理函数
 * @note 自带UART句柄判断，可直接在中断回调函数中使用
 * @param  huart: UART句柄
 */
void GYRO_UART_IT_Callback_Process(UART_HandleTypeDef *huart)
{
	if (huart == &GYRO_RECEIVE_UART_HANDLE)
	{
		static uint8_t frame_count = 0;
		static uint8_t receive_state_count = 0;
		static uint8_t last_receive_num = 0;
		uint8_t current_receive_data;
		static uint8_t imu_frame_flag = 0;
		static uint8_t ahrs_frame_flag = 0;

#if USE_RS485
		RS485_RX_RE;
		RS485_RX_DE;
#endif /* USE_RS485 */

		ttl_receive_flag = 1;

		current_receive_data = uart_rx_buffer[0];		// 读取接收到的数据
		frame_data[frame_count] = current_receive_data; // 串口数据填入数组

		if (((last_receive_num == FRAME_END) && (current_receive_data == FRAME_HEAD)) || frame_count > 0)
		{
			receive_state_count = 1;
			frame_count++;
			if ((frame_data[1] == TYPE_IMU) && (frame_data[2] == IMU_LEN))
				imu_frame_flag = 1;
			if ((frame_data[1] == TYPE_AHRS) && (frame_data[2] == AHRS_LEN))
				ahrs_frame_flag = 1;
		}
		else
			frame_count = 0;
		last_receive_num = current_receive_data;

		if (imu_frame_flag == 1 && frame_count == IMU_RS) // 将本帧数据保存至imu_frame_data数组中
		{
			frame_count = 0;
			imu_frame_flag = 0;
			imu_data_ready_flag = 1;
			if (frame_data[IMU_RS - 1] == FRAME_END) // 帧尾校验
				memcpy(imu_frame_data, frame_data, sizeof(frame_data));
		}
		if (ahrs_frame_flag == 1 && frame_count == AHRS_RS) //
		{
			frame_count = 0;
			ahrs_frame_flag = 0;
			ahrs_data_ready_flag = 1;
			if (frame_data[AHRS_RS - 1] == FRAME_END)
				memcpy(ahrs_frame_data, frame_data, sizeof(frame_data));
			for (int i = 0; i < sizeof(frame_data); i++)
				frame_data[i] = 0;
		}

		gyro_hal.UART_Receive(); // 重新启动UART接收中断
	}
}

/** ***************************************************************************
 * @brief  将TTL格式的十六进制数转换为十进制数
 * @return uint8_t: 转换后的十进制数
 */
uint8_t TTL_Hex_To_Dec(void)
{
	uint8_t process_result = 0;
	if (ahrs_data_ready_flag == 1)
	{
		if (ahrs_frame_data[1] == TYPE_AHRS && ahrs_frame_data[2] == AHRS_LEN)
		{
			ahrs_data_packet.roll_speed = Hex_To_Float(ahrs_frame_data[7], ahrs_frame_data[8], ahrs_frame_data[9], ahrs_frame_data[10]);	   // 横滚角速度
			ahrs_data_packet.pitch_speed = Hex_To_Float(ahrs_frame_data[11], ahrs_frame_data[12], ahrs_frame_data[13], ahrs_frame_data[14]);   // 俯仰角速度
			ahrs_data_packet.heading_speed = Hex_To_Float(ahrs_frame_data[15], ahrs_frame_data[16], ahrs_frame_data[17], ahrs_frame_data[18]); // 偏航角速度

			ahrs_data_packet.roll = Hex_To_Float(ahrs_frame_data[19], ahrs_frame_data[20], ahrs_frame_data[21], ahrs_frame_data[22]);	 // 横滚角
			ahrs_data_packet.pitch = Hex_To_Float(ahrs_frame_data[23], ahrs_frame_data[24], ahrs_frame_data[25], ahrs_frame_data[26]);	 // 俯仰角
			ahrs_data_packet.heading = Hex_To_Float(ahrs_frame_data[27], ahrs_frame_data[28], ahrs_frame_data[29], ahrs_frame_data[30]); // 偏航角

			ahrs_data_packet.qw = Hex_To_Float(ahrs_frame_data[31], ahrs_frame_data[32], ahrs_frame_data[33], ahrs_frame_data[34]); // 四元数
			ahrs_data_packet.qx = Hex_To_Float(ahrs_frame_data[35], ahrs_frame_data[36], ahrs_frame_data[37], ahrs_frame_data[38]);
			ahrs_data_packet.qy = Hex_To_Float(ahrs_frame_data[39], ahrs_frame_data[40], ahrs_frame_data[41], ahrs_frame_data[42]);
			ahrs_data_packet.qz = Hex_To_Float(ahrs_frame_data[43], ahrs_frame_data[44], ahrs_frame_data[45], ahrs_frame_data[46]);
			ahrs_data_packet.timestamp = timestamp(ahrs_frame_data[47], ahrs_frame_data[48], ahrs_frame_data[49], ahrs_frame_data[50]); // 时间戳
#if USE_DEBUG
			AHRS_Data_To_PC();
#endif // USE_DEBUG
		}
		ahrs_data_ready_flag = 0;
	}

	if (imu_data_ready_flag == 1)
	{
		if (imu_frame_data[1] == TYPE_IMU && imu_frame_data[2] == IMU_LEN)
		{
			imu_data_packet.gyroscope_x = Hex_To_Float(imu_frame_data[7], imu_frame_data[8], imu_frame_data[9], imu_frame_data[10]); // 角速度
			imu_data_packet.gyroscope_y = Hex_To_Float(imu_frame_data[11], imu_frame_data[12], imu_frame_data[13], imu_frame_data[14]);
			imu_data_packet.gyroscope_z = Hex_To_Float(imu_frame_data[15], imu_frame_data[16], imu_frame_data[17], imu_frame_data[18]);

			imu_data_packet.accelerometer_x = Hex_To_Float(imu_frame_data[19], imu_frame_data[20], imu_frame_data[21], imu_frame_data[22]); // 线加速度
			imu_data_packet.accelerometer_y = Hex_To_Float(imu_frame_data[23], imu_frame_data[24], imu_frame_data[25], imu_frame_data[26]);
			imu_data_packet.accelerometer_z = Hex_To_Float(imu_frame_data[27], imu_frame_data[28], imu_frame_data[29], imu_frame_data[30]);

			imu_data_packet.magnetometer_x = Hex_To_Float(imu_frame_data[31], imu_frame_data[32], imu_frame_data[33], imu_frame_data[34]); // 磁力计数据
			imu_data_packet.magnetometer_y = Hex_To_Float(imu_frame_data[35], imu_frame_data[36], imu_frame_data[37], imu_frame_data[38]);
			imu_data_packet.magnetometer_z = Hex_To_Float(imu_frame_data[39], imu_frame_data[40], imu_frame_data[41], imu_frame_data[42]);

			imu_data_packet.timestamp = timestamp(imu_frame_data[55], imu_frame_data[56], imu_frame_data[57], imu_frame_data[58]); // 时间戳
#if USE_DEBUG
			IMU_Data_To_PC();
#endif // USE_DEBUG
		}
		imu_data_ready_flag = 0;
	}
	return process_result;
}

/** ***************************************************************************
 * @brief  将16进制数据转换为浮点型数据
 * @param  data_1: 低字节
 * @param  data_2: 次低字节
 * @param  data_3: 次高字节
 * @param  data_4: 高字节
 * @return float: 转换后的浮点数
 */
float Hex_To_Float(uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4)
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
	sign = (transition_32 & 0x80000000) ? -1 : 1; // 符号位
	// 先右移操作，再按位与计算，出来结果是30到23位对应的e
	exponent = ((transition_32 >> 23) & 0xff) - 127;
	// 将22~0转化为10进制，得到对应的x系数
	mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
	tmp = sign * mantissa * pow(2, exponent);
	return tmp;
}

/** ***************************************************************************
 * @brief 将AHRS数据发送到PC
 */
void AHRS_Data_To_PC(void)
{
	printf("AHRS: The RollSpeed =  %f\r\n", ahrs_data_packet.roll_speed);
	printf("AHRS: The PitchSpeed =  %f\r\n", ahrs_data_packet.pitch_speed);
	printf("AHRS: The HeadingSpeed =  %f\r\n", ahrs_data_packet.heading_speed);
	printf("AHRS: The Roll =  %f\r\n", ahrs_data_packet.roll);
	printf("AHRS: The Pitch =  %f\r\n", ahrs_data_packet.pitch);
	printf("AHRS: The Heading =  %f\r\n", ahrs_data_packet.heading);
	printf("AHRS: The Quaternion.Qw =  %f\r\n", ahrs_data_packet.qw);
	printf("AHRS: The Quaternion.Qx =  %f\r\n", ahrs_data_packet.qx);
	printf("AHRS: The Quaternion.Qy =  %f\r\n", ahrs_data_packet.qy);
	printf("AHRS: The Quaternion.Qz =  %f\r\n", ahrs_data_packet.qz);
	printf("AHRS: The Timestamp =  %d\r\n", ahrs_data_packet.timestamp);
}

/** ***************************************************************************
 * @brief 将IMU数据发送到PC
 */
void IMU_Data_To_PC(void)
{
	// printf("Now start sending IMU data.\r\n");
	printf("IMU: The gyroscope_x =  %f\r\n", imu_data_packet.gyroscope_x);
	printf("IMU:The gyroscope_y =  %f\r\n", imu_data_packet.gyroscope_y);
	printf("IMU:The gyroscope_z =  %f\r\n", imu_data_packet.gyroscope_z);
	printf("IMU:The accelerometer_x =  %f\r\n", imu_data_packet.accelerometer_x);
	printf("IMU:The accelerometer_y =  %f\r\n", imu_data_packet.accelerometer_y);
	printf("IMU:The accelerometer_z =  %f\r\n", imu_data_packet.accelerometer_z);
	printf("IMU:The magnetometer_x =  %f\r\n", imu_data_packet.magnetometer_x);
	printf("IMU:The magnetometer_y =  %f\r\n", imu_data_packet.magnetometer_y);
	printf("IMU:The magnetometer_z =  %f\r\n", imu_data_packet.magnetometer_z);
	printf("IMU:The Timestamp =  %d\r\n", imu_data_packet.timestamp);
	// printf("Now the data of IMU has been sent.\r\n");
}

long long timestamp(uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4)
{
	unsigned int transition_32;
	transition_32 = 0;
	transition_32 |= data_4 << 24;
	transition_32 |= data_3 << 16;
	transition_32 |= data_2 << 8;
	transition_32 |= data_1;
	return transition_32;
}
