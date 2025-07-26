/** ***************************************************************************
 * @File Name: GYRO_APP.h
 * @brief 陀螺仪应用层头文件
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-26
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes : 按照格式规范修改变量名和结构体类型
 * @Modifi Date : 2025-07-26
 */
#ifndef __GYRO_APP_H
#define __GYRO_APP_H

#include "GYRO_HAL.h"
#include "GYRO_Config.h"
#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "math.h"

#define FRAME_HEADER 0X7B // Frame_header
#define FRAME_TAIL 0X7D	  // Frame_tail
#define SEND_DATA_SIZE 24
#define RECEIVE_DATA_SIZE 11
#define IMU_RS 64
#define AHRS_RS 56
#define INSGPS_RS 80

// FDlink candata
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GROUND 0xf0
#define IMU_LEN 0x38	// 56+8  8组数据
#define AHRS_LEN 0x30	// 48+8  7组数据
#define INSGPS_LEN 0x42 // 72+8  10组数据
#define IMU_CAN 9
#define AHRS_CAN 8
#define INSGPS_CAN 11

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

typedef struct ahrs_data_packet_t
{
	float roll_speed;		// unit: rad/s    横滚角速度
	float pitch_speed;		// unit: rad/s    俯仰角速度
	float heading_speed;	// unit: rad/s    偏航角速度
	float roll;				// unit: rad      横滚角
	float pitch;			// unit: rad      俯仰角
	float heading;			// unit: rad      偏航角/航向角
	float qw;				// w              四元数W分量
	float qx;				// x              四元数X分量
	float qy;				// y              四元数Y分量
	float qz;				// z              四元数Z分量
	unsigned int timestamp; // us             时间戳
} ahrs_data_packet_t;

extern uint8_t ttl_receive;

extern imu_data_packet_t imu_data_packet;
extern ahrs_data_packet_t ahrs_data_packet;

/** ***************************************************************************
 * @brief 陀螺仪UART中断回调处理函数(直接调用版本)
 */
void GYRO_UART_IT_Callback_Process(void);

uint8_t TTL_Hex_To_Dec(void);
float Hex_To_Float(uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4);
void IMU_Data_To_PC(void);
long long timestamp(uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4);

#endif /* __GYRO_APP_H */
