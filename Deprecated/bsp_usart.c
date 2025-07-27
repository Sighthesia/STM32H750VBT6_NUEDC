/**
 ******************************************************************************
 * @file    bsp_usart.c
 * @brief   STM32H7xx HAL库 UART驱动实现
 *
 * @details
 * 1. 需要在 main.c 或其他文件定义以下变量:
 *    - UART_HandleTypeDef huart1, huart3, huart5;
 * 2. 串口初始化由 CubeMX 生成，本文件仅提供环形缓冲区和数据收发等辅助功能。
 * 3. 环形缓冲区用于数据接收与处理。
 * 4. 使用 HAL_UART_RxCpltCallback 处理接收中断。
 ******************************************************************************
 */

#include "bsp_usart.h"

// 接收缓冲区变量
uint8_t uart1_rx_buffer = 0;
uint8_t uart3_rx_buffer = 0;

// 环形缓冲区结构体变量
RingBuff_t Uart2_RingBuff, Uart1_RingBuff, Uart3_RingBuff;

/**
 * @brief  初始化USART1，配置环形缓冲区并开启接收中断
 * @param  baudrate 波特率（实际初始化由CubeMX完成）
 */
void USART1_init(uint32_t baudrate)
{
	// HAL库版本的UART初始化在CubeMX中生成，这里只需要初始化环形缓冲区
	RingBuff_Init(&Uart1_RingBuff);

	// 启用UART接收中断
	HAL_UART_Receive_IT(&huart1, &uart1_rx_buffer, 1);
}

/**
 * @brief  初始化环形缓冲区
 * @param  ringbuff 指向环形缓冲区结构体的指针
 */
void RingBuff_Init(RingBuff_t *ringbuff)
{
	// 初始化环形缓冲区
	ringbuff->Head = 0;
	ringbuff->Tail = 0;
	ringbuff->Lenght = 0;
}

/**
 * @brief  向环形缓冲区写入一个字节
 * @param  ringbuff 指向环形缓冲区结构体的指针
 * @param  data     要写入的数据
 * @retval RINGBUFF_OK 写入成功
 * @retval RINGBUFF_ERR 缓冲区已满
 */
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
 * @brief  UART1发送单字节数据
 * @param  ch 待发送的数据
 */
void USART1_Send_U8(uint8_t ch)
{
	HAL_UART_Transmit(&huart1, &ch, 1, HAL_MAX_DELAY);
}

/**
 * @brief  UART1发送数据数组
 * @param  BufferPtr 待发送的数据指针
 * @param  Length    数据长度
 */
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
	HAL_UART_Transmit(&huart1, BufferPtr, Length, HAL_MAX_DELAY);
}

/**
 * @brief  重定向printf到USART1
 */
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口 */
	uint8_t data = (uint8_t)ch;
	HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY);
	return (ch);
}

/**
 * @brief  重定向scanf/getchar到USART1
 */
int fgetc(FILE *f)
{
	uint8_t data;
	/* 等待串口输入数据 */
	HAL_UART_Receive(&huart1, &data, 1, HAL_MAX_DELAY);
	return (int)data;
}

/**
 * @brief  初始化USART3，配置环形缓冲区并开启接收中断
 * @param  baudrate 波特率（实际初始化由CubeMX完成）
 */
void USART3_Init(uint32_t baudrate)
{
	// HAL库版本的UART初始化在CubeMX中生成，这里只需要初始化环形缓冲区
	RingBuff_Init(&Uart3_RingBuff);

	// 启用UART接收中断
	HAL_UART_Receive_IT(&huart3, &uart3_rx_buffer, 1);
}

/**
 * @brief  通用UART发送数据
 * @param  huart UART句柄指针
 * @param  data  数据指针
 * @param  len   数据长度
 */
void usart_send(UART_HandleTypeDef *huart, uint8_t *data, uint8_t len)
{
	HAL_UART_Transmit(huart, data, len, HAL_MAX_DELAY);
}

/**
 * @brief  USART3发送单字节数据
 * @param  ch 待发送的数据
 */
void USART3_Send_U8(uint8_t ch)
{
	HAL_UART_Transmit(&huart3, &ch, 1, HAL_MAX_DELAY);
}

/**
 * @brief  数据解码函数，按协议解析一帧数据
 * @param  ringbuff 环形缓冲区指针
 * @param  data1~data4 解码后的数据指针
 * @retval 0 解码成功
 * @retval 1 解码失败
 */
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

/**
 * @brief  JSON格式数据接收函数，从环形缓冲区读取一帧JSON字符串
 * @param  ringbuff 环形缓冲区指针
 * @param  str      存储JSON字符串的缓冲区
 * @retval 1 接收完成
 * @retval 0 未接收到完整JSON
 */
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

/**
 * @brief  从环形缓冲区读取一个字节
 * @param  ringbuff 环形缓冲区指针
 * @param  rData    读取到的数据指针
 * @retval RINGBUFF_OK 读取成功
 * @retval RINGBUFF_ERR 缓冲区为空
 */
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
