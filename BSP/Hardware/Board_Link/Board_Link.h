#ifndef BOARD_LINK_H
#define BOARD_LINK_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

extern SPI_HandleTypeDef hspi3;

typedef enum
{
    BOARD_LINK_COMMAND_IDLE = 0,
    BOARD_LINK_COMMAND_WAITING,
    BOARD_LINK_COMMAND_OK,
    BOARD_LINK_COMMAND_REJECTED,
    BOARD_LINK_COMMAND_ERROR
} board_link_command_status_t;

void Board_Link_Init(void);
void Board_Link_Process(void);
void Board_Link_NSS_Falling_Callback(void);
bool Board_Link_Is_Online(void);
uint8_t Board_Link_Get_Peer_Role(void);
uint8_t Board_Link_Get_Peer_Gray(void);
void Board_Link_Select_Test(uint8_t test_id);
void Board_Link_Start_Test(void);
void Board_Link_Stop_Test(void);
board_link_command_status_t Board_Link_Get_Command_Status(void);
uint8_t Board_Link_Get_Last_Command(void);
uint8_t Board_Link_Get_Last_Ack(void);
uint8_t Board_Link_Get_Peer_Diagnostic(void);
uint16_t Board_Link_Get_Valid_Frame_Count(void);
uint32_t Board_Link_Get_Last_Spi_Error(void);

#endif /* BOARD_LINK_H */
