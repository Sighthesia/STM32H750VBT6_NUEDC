/** ***************************************************************************
 * @File Name: usartx.h
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-26
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#ifndef __USRATX_H
#define __USRATX_H

#include "GYRO_Config.h"
#include "stdio.h"
#include "stdint.h"
#include "main.h"
#include "usart.h"
#include "string.h"
#include "math.h"
#include "stm32h7xx_hal.h"

/* ----------------------------- 此处定义帧格式相关宏 ---------------------------- */
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

typedef struct IMUData_Packet_t
{
	float gyroscope_x;			// unit: rad/s
	float gyroscope_y;			// unit: rad/s
	float gyroscope_z;			// unit: rad/s
	float accelerometer_x;		// m/s^2
	float accelerometer_y;		// m/s^2
	float accelerometer_z;		// m/s^2
	float magnetometer_x;		// mG
	float magnetometer_y;		// mG
	float magnetometer_z;		// mG
	float imu_temperature;		// C
	float Pressure;				// Pa
	float pressure_temperature; // C
	unsigned int Timestamp;		// us
} IMUData_Packet_t;

typedef struct AHRSData_Packet_t
{
	float RollSpeed;		// unit: rad/s
	float PitchSpeed;		// unit: rad/s
	float HeadingSpeed;		// unit: rad/s
	float Roll;				// unit: rad
	float Pitch;			// unit: rad
	float Heading;			// unit: rad
	float Qw;				// w          //Quaternion
	float Qx;				// x
	float Qy;				// y
	float Qz;				// z
	unsigned int Timestamp; // us// unit: us
} AHRSData_Packet_t;

extern uint8_t ttl_receive;

extern IMUData_Packet_t IMUData_Packet;
extern AHRSData_Packet_t AHRSData_Packet;

/* -------------------------------- 此处修改函数声明 ------------------------------- */
/** ***************************************************************************
 * @brief UART接收中断回调函数
 * @param huart: UART句柄指针
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/** ***************************************************************************
 * @brief 启动UART中断接收
 */
void GYRO_UART_Start_Receive_IT(void);
/* ---------------------------------- 自定义部分结束 --------------------------------- */

long long timestamp(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4);
void AHRSData2PC(void);
void IMUData2PC(void);

float XYZ_Target_Speed_transition(uint8_t High, uint8_t Low);
uint8_t TTL_Hex2Dec(void);
float DATA_Trans(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4);

/** ***************************************************************************
 * @brief 串口发送函数
 */
void Usart1_Send(uint8_t data);
// void Usart2_Send(uint8_t data); // 需要在usart.h中添加huart2声明
void Usart3_Send(uint8_t data);

#endif /* __USRATX_H */
