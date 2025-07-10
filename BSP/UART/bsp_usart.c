/**
 * ******************************************************************************
 * @file    bsp_usart.c (HAL库版本)
 * @brief   STM32H7xx HAL库 UART驱动
 *
 * 使用说明:
 * 1. 需要在main.c或其他地方定义以下变量:
 *    - UART_HandleTypeDef huart1, huart3, huart5;
 * 2. 串口初始化通过CubeMX生成，本文件只提供辅助功能
 * 3. 环形缓冲区用于数据接收和处理
 * 4. 使用HAL_UART_RxCpltCallback处理接收中断
 * ******************************************************************************
 */

#include "bsp_usart.h"
#include <stdio.h>

// 接收缓冲区变量定义
uint8_t uart1_rx_buffer = 0;
uint8_t uart3_rx_buffer = 0;

RingBuff_t Uart2_RingBuff, Uart1_RingBuff, Uart3_RingBuff; // 定义ringBuff结构体变量

// USART1 ---- 与PC通信，输出调试信息
void USART1_init(uint32_t baudrate)
{
	// HAL库版本的UART初始化在CubeMX中生成，这里只需要初始化环形缓冲区
	RingBuff_Init(&Uart1_RingBuff);

	// 启用UART接收中断
	HAL_UART_Receive_IT(&huart1, &uart1_rx_buffer, 1);
}

void RingBuff_Init(RingBuff_t *ringbuff)
{
	// 初始化环形缓冲区
	ringbuff->Head = 0;
	ringbuff->Tail = 0;
	ringbuff->Lenght = 0;
}

uint8_t Write_RingBuff(RingBuff_t *ringbuff, uint8_t data)
{
	if (ringbuff->Lenght >= RINGBUFF_LEN) // 缓冲区已满
	{
		return RINGBUFF_ERR;
	}
	ringbuff->Ring_data[ringbuff->Tail] = data;
	ringbuff->Tail = (ringbuff->Tail + 1) % RINGBUFF_LEN; // 环形操作
	ringbuff->Lenght++;
	return RINGBUFF_OK;
}

/**
 * @Brief: UART1发送数据
 * @Note:
 * @Parm: ch:待发送的数据
 * @Retval:
 */
void USART1_Send_U8(uint8_t ch)
{
	HAL_UART_Transmit(&huart1, &ch, 1, HAL_MAX_DELAY);
}

/**
 * @Brief: UART1发送数据
 * @Note:
 * @Parm: BufferPtr:待发送的数据  Length:数据长度
 * @Retval:
 */
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
	HAL_UART_Transmit(&huart1, BufferPtr, Length, HAL_MAX_DELAY);
}

// HAL库中断回调函数 - 接收完成
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		// 将接收到的数据写入环形缓冲区
		Write_RingBuff(&Uart1_RingBuff, uart1_rx_buffer);
		// 重新启动接收中断
		HAL_UART_Receive_IT(&huart1, &uart1_rx_buffer, 1);
	}
	else if (huart->Instance == USART3)
	{
		Write_RingBuff(&Uart3_RingBuff, uart3_rx_buffer);
		HAL_UART_Receive_IT(&huart3, &uart3_rx_buffer, 1);
	}
}

/// 重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口 */
	uint8_t data = (uint8_t)ch;
	HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY);
	return (ch);
}

/// 重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
	uint8_t data;
	/* 等待串口输入数据 */
	HAL_UART_Receive(&huart1, &data, 1, HAL_MAX_DELAY);
	return (int)data;
}

// USART3
void USART3_Init(uint32_t baudrate)
{
	// HAL库版本的UART初始化在CubeMX中生成，这里只需要初始化环形缓冲区
	RingBuff_Init(&Uart3_RingBuff);

	// 启用UART接收中断
	HAL_UART_Receive_IT(&huart3, &uart3_rx_buffer, 1);
}

void usart_send(UART_HandleTypeDef *huart, uint8_t *data, uint8_t len)
{
	HAL_UART_Transmit(huart, data, len, HAL_MAX_DELAY);
}

// 发送一个字符
void USART3_Send_U8(uint8_t ch)
{
	HAL_UART_Transmit(&huart3, &ch, 1, HAL_MAX_DELAY);
}

// 发送一个字符串
/**
 * @Brief: UsART3发送数据
 * @Note:
 * @Parm: BufferPtr:待发送的数据  Length:数据长度
 * @Retval:
 */

// 数据解码函数保持不变
uint8_t DataDecode1(RingBuff_t *ringbuff, uint8_t *data1, uint8_t *data2, uint8_t *data3, uint8_t *data4)
{
	static uint8_t uart_dec_count;
	static uint8_t uart_rec_data[6];
	uint8_t ret = 1;

	if (Read_RingBuff(ringbuff, &uart_rec_data[uart_dec_count]) == RINGBUFF_ERR)
	{
		return 1;
	}
	if ((uart_dec_count == 0) && (uart_rec_data[uart_dec_count] != 0x55))
	{ // 检测第一个数据是否为0x55
		uart_rec_data[uart_dec_count] = 0;
	}
	else if ((uart_dec_count == 1) && (uart_rec_data[uart_dec_count] != 0xaa))
	{ // 检测第二个数据是否为0xaa
		uart_rec_data[uart_dec_count] = 0;
		uart_rec_data[uart_dec_count - 1] = 0;
		uart_dec_count = 0;
	}
	else if ((uart_dec_count == 6) && (uart_rec_data[uart_dec_count] != 0xfa))
	{ // 检测尾帧是否为0xfa
		uart_rec_data[uart_dec_count] = 0;
		uart_rec_data[uart_dec_count - 1] = 0;
		uart_rec_data[uart_dec_count - 2] = 0;
		uart_rec_data[uart_dec_count - 3] = 0;
		uart_rec_data[uart_dec_count - 4] = 0;
		uart_rec_data[uart_dec_count - 5] = 0;
		uart_dec_count = 0;
	}
	else
	{
		if (uart_dec_count == 6) // 成功接受一帧数据
		{
			*data1 = uart_rec_data[2];
			*data2 = uart_rec_data[3];
			*data3 = uart_rec_data[4];
			*data4 = uart_rec_data[5];
			ret = 0;
		}
		uart_dec_count++;
		if (uart_dec_count == 7)
		{
			uart_dec_count = 0;
		}
	}
	return ret;
}

// // UART5
// void UART5_init(uint32_t baudrate)
// {
// 	// HAL库版本的UART初始化在CubeMX中生成
// 	// 这里可以添加特定的初始化代码
// }

// // 发送一个字符
// void UART5_Send_U8(uint8_t ch)
// {
// 	HAL_UART_Transmit(&huart5, &ch, 1, HAL_MAX_DELAY);
// }

// // 发送数组
// void UART5_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
// {
// 	HAL_UART_Transmit(&huart5, BufferPtr, Length, HAL_MAX_DELAY);
// }

// JSON接收函数
int8_t receiveJson(RingBuff_t *ringbuff, char *str)
{
	static uint8_t reading, j;
	uint8_t readbuff, ok = 0;
	if (Read_RingBuff(&Uart1_RingBuff, &readbuff) == RINGBUFF_OK)
	{
		if (readbuff == '{')
		{
			reading = 1;
			str[j++] = readbuff;
			ok = 0;
		}
		else if (readbuff == '}' && reading == 1)
		{
			reading = 0;
			str[j++] = readbuff;
			str[j++] = '\0';
			ok = 1;
		}
		else if (reading == 1)
		{
			str[j++] = readbuff;
		}
		else
		{
			j = 0;
			reading = 0;
			ok = 0;
		}
	}
	return ok;
}

uint8_t Read_RingBuff(RingBuff_t *ringbuff, uint8_t *rData)
{
	if (ringbuff->Lenght == 0) // 缓冲区空
	{
		return RINGBUFF_ERR;
	}
	*rData = ringbuff->Ring_data[ringbuff->Head];		  // 先进先出FIFO，从头部读取
	ringbuff->Head = (ringbuff->Head + 1) % RINGBUFF_LEN; // 环形操作
	ringbuff->Lenght--;
	return RINGBUFF_OK;
}
