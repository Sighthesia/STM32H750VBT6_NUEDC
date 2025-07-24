#ifndef __USRATX_H
#define __USRATX_H 

#include "stdio.h"
#include "sys.h"


#define FRAME_HEADER      0X7B //Frame_header //֡ͷ
#define FRAME_TAIL        0X7D //Frame_tail   //֡β
#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11
#define IMU_RS 64
#define AHRS_RS 56
#define INSGPS_RS 80
#define RS485_RX_DE		PAout(11)	//485模式控制.0,接收;1,发送.
#define RS485_RX_RE		PAout(12)	//485模式控制.0,接收;1,发送.

extern u8 ttl_receive;
void USART3_SEND(void);
void USART1_SEND(void);
void usart2_send(u8 data);
void usart3_send(u8 data);
void usart1_send(u8 data);
void uart1_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound);
void uart5_init(u32 bound);

int USART1_IRQHandler(void);
int USART2_IRQHandler(void);
int USART3_IRQHandler(void);
void AHRSData2PC(void);
void IMUData2PC(void);
float XYZ_Target_Speed_transition(u8 High,u8 Low);
u8 TTL_Hex2Dec(void);  

float Data_Trans(u8 Data_1,u8 Data_2,u8 Data_3,u8 Data_4);

#endif

