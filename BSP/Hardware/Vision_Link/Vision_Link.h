#ifndef VISION_LINK_H
#define VISION_LINK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float position_cm;
    float velocity_cm_s;
    uint32_t received_ms;
} vision_link_sample_t;

void Vision_Link_Init(void);
void Vision_Link_Process(void);
void Vision_Link_UART_RxCplt_Callback(void);
void Vision_Link_UART_Error_Callback(void);
bool Vision_Link_Get_Sample(vision_link_sample_t *sample);
bool Vision_Link_Is_Online(void);

#endif /* VISION_LINK_H */
