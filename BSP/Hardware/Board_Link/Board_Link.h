#ifndef BOARD_LINK_H
#define BOARD_LINK_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

void Board_Link_Init(void);
void Board_Link_Process(void);
void Board_Link_NSS_Falling_Callback(void);
bool Board_Link_Is_Online(void);
uint8_t Board_Link_Get_Peer_Role(void);
uint8_t Board_Link_Get_Peer_Gray(void);

#endif /* BOARD_LINK_H */
