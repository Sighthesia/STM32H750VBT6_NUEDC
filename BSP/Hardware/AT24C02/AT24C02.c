#include "AT24C02.h"

#include "i2c.h"

#include <string.h>

#define AT24C02_I2C_ADDRESS (0x50U << 1U)
#define AT24C02_PAGE_SIZE 8U
#define AT24C02_TIMEOUT_MS 20U
#define AT24C02_WRITE_CYCLE_MS 5U

static at24c02_status_t last_status;

bool AT24C02_Read(uint8_t address, uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U || (uint16_t)address + length > 256U)
    {
        last_status = AT24C02_STATUS_INVALID_ARGUMENT;
        return false;
    }
    if (HAL_I2C_IsDeviceReady(&hi2c1, AT24C02_I2C_ADDRESS, 3U, AT24C02_TIMEOUT_MS) != HAL_OK)
    {
        last_status = AT24C02_STATUS_DEVICE_NOT_READY;
        return false;
    }
    if (HAL_I2C_Mem_Read(&hi2c1, AT24C02_I2C_ADDRESS, address,
                         I2C_MEMADD_SIZE_8BIT, data, length,
                         AT24C02_TIMEOUT_MS) != HAL_OK)
    {
        last_status = AT24C02_STATUS_READ_FAILED;
        return false;
    }
    last_status = AT24C02_STATUS_OK;
    return true;
}

bool AT24C02_Write(uint8_t address, const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U || (uint16_t)address + length > 256U)
    {
        last_status = AT24C02_STATUS_INVALID_ARGUMENT;
        return false;
    }
    if (HAL_I2C_IsDeviceReady(&hi2c1, AT24C02_I2C_ADDRESS, 3U, AT24C02_TIMEOUT_MS) != HAL_OK)
    {
        last_status = AT24C02_STATUS_DEVICE_NOT_READY;
        return false;
    }
    while (length != 0U)
    {
        uint16_t page_remaining = AT24C02_PAGE_SIZE - (address % AT24C02_PAGE_SIZE);
        uint16_t chunk = length < page_remaining ? length : page_remaining;
        if (HAL_I2C_Mem_Write(&hi2c1, AT24C02_I2C_ADDRESS, address,
                              I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, chunk,
                              AT24C02_TIMEOUT_MS) != HAL_OK)
        {
            last_status = AT24C02_STATUS_WRITE_FAILED;
            return false;
        }
        HAL_Delay(AT24C02_WRITE_CYCLE_MS);
        if (HAL_I2C_IsDeviceReady(&hi2c1, AT24C02_I2C_ADDRESS, 3U,
                                  AT24C02_TIMEOUT_MS) != HAL_OK)
        {
            last_status = AT24C02_STATUS_WRITE_CYCLE_TIMEOUT;
            return false;
        }
        address = (uint8_t)(address + chunk);
        data += chunk;
        length -= chunk;
    }
    last_status = AT24C02_STATUS_OK;
    return true;
}

at24c02_status_t AT24C02_GetLastStatus(void)
{
    return last_status;
}
