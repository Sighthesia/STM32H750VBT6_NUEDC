#ifndef TRACKING_LINK_H
#define TRACKING_LINK_H

#include <stdbool.h>
#include <stdint.h>

void Tracking_Link_Init(void);
void Tracking_Link_UART_RxCplt_Callback(void);
void Tracking_Link_UART_Error_Callback(void);
bool Tracking_Link_Get_Angle(int16_t *angle_x10);
bool Tracking_Link_Get_State(uint8_t *state);
bool Tracking_Link_Is_Online(void);

#endif
