#ifndef BOARD_LINK_H
#define BOARD_LINK_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

void Board_Link_Init(void);
void Board_Link_Process(void);
bool Board_Link_Is_UART(const UART_HandleTypeDef *huart);
void Board_Link_UART_RxCplt_Callback(void);
void Board_Link_UART_TxCplt_Callback(void);
void Board_Link_UART_Error_Callback(void);
bool Board_Link_Is_Online(void);
uint8_t Board_Link_Get_Peer_Role(void);
uint8_t Board_Link_Get_Peer_Gray(void);

#endif /* BOARD_LINK_H */
