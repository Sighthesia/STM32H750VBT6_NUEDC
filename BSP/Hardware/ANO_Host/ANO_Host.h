#ifndef ANO_HOST_H
#define ANO_HOST_H

void ANO_Host_Init(void);
void ANO_Host_Process(void);
void ANO_Host_UART_RxCplt_Callback(void);
void ANO_Host_UART_TxCplt_Callback(void);
void ANO_Host_UART_Error_Callback(void);

#endif /* ANO_HOST_H */
