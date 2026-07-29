#ifndef W25QXX_H
#define W25QXX_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    W25QXX_STATUS_OK,
    W25QXX_STATUS_NOT_FOUND,
    W25QXX_STATUS_TIMEOUT,
    W25QXX_STATUS_PROGRAM_FAILED,
    W25QXX_STATUS_ERASE_FAILED,
    W25QXX_STATUS_VERIFY_FAILED,
    W25QXX_STATUS_INVALID_ARGUMENT
} w25qxx_status_t;

bool W25Qxx_Init(void);
uint32_t W25Qxx_GetDeviceId(void);
bool W25Qxx_Read(uint32_t address, uint8_t *data, uint32_t length);
bool W25Qxx_EraseSector(uint32_t address);
bool W25Qxx_Write(uint32_t address, const uint8_t *data, uint32_t length);
w25qxx_status_t W25Qxx_GetLastStatus(void);

#endif /* W25QXX_H */
