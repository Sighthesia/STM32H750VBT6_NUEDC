#ifndef ANO_HOST_H
#define ANO_HOST_H

#include <stdint.h>

#define ANO_COMMAND_CAR_READ_FLASH 0x20U
#define ANO_COMMAND_CAR_FACTORY_RESET 0x21U
#define ANO_COMMAND_CAR_APPLY_CONFIG 0x22U
#define ANO_COMMAND_CAR_DEBUG_ENTER 0x23U
#define ANO_COMMAND_CAR_DEBUG_START 0x24U
#define ANO_COMMAND_CAR_DEBUG_STOP 0x25U
#define ANO_COMMAND_CAR_RESTORE_DEFAULTS 0x26U

void ANO_Host_Init(void);
void ANO_Host_Process(void);
void ANO_Host_UART_RxCplt_Callback(void);
void ANO_Host_UART_TxCplt_Callback(void);
void ANO_Host_UART_Error_Callback(void);
uint32_t ANO_Host_Get_BaudRate(void);

#endif /* ANO_HOST_H */
