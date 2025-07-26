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

#define FRAME_HEADER 0X7B // Frame_header
#define FRAME_TAIL 0X7D	  // Frame_tail
#define SEND_DATA_SIZE 24
#define RECEIVE_DATA_SIZE 11
#define IMU_RS 64
#define AHRS_RS 56
#define INSGPS_RS 80

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
	uint32_t Timestamp;			// us
} IMUData_Packet_t;

typedef struct AHRSData_Packet_t
{
	float RollSpeed;	// unit: rad/s
	float PitchSpeed;	// unit: rad/s
	float HeadingSpeed; // unit: rad/s
	float Roll;			// unit: rad
	float Pitch;		// unit: rad
	float Heading;		// unit: rad
	float Qw;			// w          //Quaternion
	float Qx;			// x
	float Qy;			// y
	float Qz;			// z
	uint32_t Timestamp; // us// unit: us
} AHRSData_Packet_t;

extern uint8_t ttl_receive;

extern IMUData_Packet_t IMUData_Packet;
extern AHRSData_Packet_t AHRSData_Packet;

long long timestamp(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4);
void AHRSData2PC(void);
void IMUData2PC(void);

float XYZ_Target_Speed_transition(uint8_t High, uint8_t Low);
uint8_t TTL_Hex2Dec(void);
float DATA_Trans(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4);

#endif // __USRATX_H
