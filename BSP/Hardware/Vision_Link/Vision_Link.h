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

typedef struct
{
    uint32_t received_bytes;
    uint32_t valid_frames;
    uint32_t parse_errors;
    uint32_t uart_errors;
    uint32_t last_uart_error;
    uint32_t last_uart_isr;
    uint32_t restart_failures;
} vision_link_diagnostic_t;

void Vision_Link_Init(void);
void Vision_Link_Process(void);
void Vision_Link_UART_RxCplt_Callback(void);
void Vision_Link_UART_Error_Callback(void);
void Vision_Link_Set_Kalman_Enabled(bool enabled);
void Vision_Link_Reset_Kalman(void);
bool Vision_Link_Get_Sample(vision_link_sample_t *sample);
bool Vision_Link_Get_Last_Sample(vision_link_sample_t *sample);
bool Vision_Link_Is_Online(void);
void Vision_Link_Get_Diagnostic(vision_link_diagnostic_t *diagnostic);

#endif /* VISION_LINK_H */
