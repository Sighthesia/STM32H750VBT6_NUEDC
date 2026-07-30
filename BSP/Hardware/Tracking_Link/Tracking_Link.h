#ifndef TRACKING_LINK_H
#define TRACKING_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "Tracking_Line_Model.h"

typedef struct
{
    uint8_t state;
    uint8_t sequence;
    int16_t offset_x1000;
    int16_t angle_x10;
    tracking_pattern_t pattern;
    uint32_t received_ms;
} tracking_link_sample_t;

void Tracking_Link_Init(void);
void Tracking_Link_UART_RxCplt_Callback(void);
void Tracking_Link_UART_Error_Callback(void);
bool Tracking_Link_Get_Sample(tracking_link_sample_t *sample);
bool Tracking_Link_Get_Angle(int16_t *angle_x10);
bool Tracking_Link_Get_State(uint8_t *state);
bool Tracking_Link_Is_Online(void);

#endif
