#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <stdint.h>

#define EN_USART1_RX 1

#define RINGBUFF_LEN (200)
#define RINGBUFF_OK 1
#define RINGBUFF_ERR 0

typedef struct
{
    uint16_t Head;
    uint16_t Tail;
    uint16_t Lenght;
    uint8_t Ring_data[RINGBUFF_LEN];
} RingBuff_t;

extern RingBuff_t Uart2_RingBuff, Uart1_RingBuff, Uart3_RingBuff;
extern uint8_t uart1_rx_buffer, uart3_rx_buffer;
// extern UART_HandleTypeDef huart1, huart3, huart5;

uint8_t Write_RingBuff(RingBuff_t *ringbuff, uint8_t data);
uint8_t Read_RingBuff(RingBuff_t *ringbuff, uint8_t *rData);
void RingBuff_Init(RingBuff_t *ringbuff);

uint8_t DataDecode1(RingBuff_t *ringbuff, uint8_t *data1, uint8_t *data2, uint8_t *data3, uint8_t *data4);

void USART1_init(uint32_t baudrate);
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);
void USART1_Send_U8(uint8_t ch);

void USART3_Init(uint32_t baudrate);
void USART3_Send_U8(uint8_t ch);

void UART5_init(uint32_t baudrate);
void UART5_Send_U8(uint8_t ch);
void UART5_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);

void usart_send(UART_HandleTypeDef *huart, uint8_t *data, uint8_t len);
int8_t receiveJson(RingBuff_t *ringbuff, char *str);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
