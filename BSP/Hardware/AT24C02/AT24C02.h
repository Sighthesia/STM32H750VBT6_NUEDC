#ifndef AT24C02_H
#define AT24C02_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    AT24C02_STATUS_OK,
    AT24C02_STATUS_INVALID_ARGUMENT,
    AT24C02_STATUS_DEVICE_NOT_READY,
    AT24C02_STATUS_READ_FAILED,
    AT24C02_STATUS_WRITE_FAILED,
    AT24C02_STATUS_WRITE_CYCLE_TIMEOUT
} at24c02_status_t;

bool AT24C02_Read(uint8_t address, uint8_t *data, uint16_t length);
bool AT24C02_Write(uint8_t address, const uint8_t *data, uint16_t length);
at24c02_status_t AT24C02_GetLastStatus(void);

#endif /* AT24C02_H */
