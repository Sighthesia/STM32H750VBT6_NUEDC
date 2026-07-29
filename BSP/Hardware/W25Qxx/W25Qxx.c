#include "W25Qxx.h"

#include "main.h"

#define W25QXX_COMMAND_READ_ID 0x9FU
#define W25QXX_COMMAND_READ_DATA 0x03U
#define W25QXX_COMMAND_WRITE_ENABLE 0x06U
#define W25QXX_COMMAND_PAGE_PROGRAM 0x02U
#define W25QXX_COMMAND_SECTOR_ERASE 0x20U
#define W25QXX_COMMAND_READ_STATUS_1 0x05U
#define W25QXX_STATUS_BUSY 0x01U
#define W25QXX_PAGE_SIZE 256U
#define W25QXX_SECTOR_SIZE 4096U
#define W25QXX_TIMEOUT_MS 3000U

static uint32_t device_id;
static w25qxx_status_t last_status;

static void select_chip(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); }
static void deselect_chip(void) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); }

static uint8_t transfer(uint8_t value)
{
    uint8_t received = 0U;
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (value & 0x80U) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
        value <<= 1U;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        received = (uint8_t)((received << 1U) | (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_SET ? 1U : 0U));
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    return received;
}

static void send_address(uint32_t address)
{
    (void)transfer((uint8_t)(address >> 16U));
    (void)transfer((uint8_t)(address >> 8U));
    (void)transfer((uint8_t)address);
}

static uint8_t read_status(void)
{
    uint8_t status;
    select_chip();
    (void)transfer(W25QXX_COMMAND_READ_STATUS_1);
    status = transfer(0xFFU);
    deselect_chip();
    return status;
}

static bool wait_ready(void)
{
    uint32_t start = HAL_GetTick();
    while ((read_status() & W25QXX_STATUS_BUSY) != 0U)
    {
        if (HAL_GetTick() - start >= W25QXX_TIMEOUT_MS)
        {
            last_status = W25QXX_STATUS_TIMEOUT;
            return false;
        }
    }
    return true;
}

static void write_enable(void)
{
    select_chip();
    (void)transfer(W25QXX_COMMAND_WRITE_ENABLE);
    deselect_chip();
}

bool W25Qxx_Init(void)
{
    select_chip();
    (void)transfer(W25QXX_COMMAND_READ_ID);
    device_id = ((uint32_t)transfer(0xFFU) << 16U) | ((uint32_t)transfer(0xFFU) << 8U) | transfer(0xFFU);
    deselect_chip();
    if ((device_id & 0xFF0000U) != 0xEF0000U)
    {
        last_status = W25QXX_STATUS_NOT_FOUND;
        return false;
    }
    last_status = W25QXX_STATUS_OK;
    return true;
}

uint32_t W25Qxx_GetDeviceId(void) { return device_id; }

bool W25Qxx_Read(uint32_t address, uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0U)
    {
        last_status = W25QXX_STATUS_INVALID_ARGUMENT;
        return false;
    }
    select_chip();
    (void)transfer(W25QXX_COMMAND_READ_DATA);
    send_address(address);
    while (length-- != 0U) *data++ = transfer(0xFFU);
    deselect_chip();
    last_status = W25QXX_STATUS_OK;
    return true;
}

bool W25Qxx_EraseSector(uint32_t address)
{
    if ((address % W25QXX_SECTOR_SIZE) != 0U)
    {
        last_status = W25QXX_STATUS_INVALID_ARGUMENT;
        return false;
    }
    write_enable();
    select_chip();
    (void)transfer(W25QXX_COMMAND_SECTOR_ERASE);
    send_address(address);
    deselect_chip();
    if (!wait_ready())
    {
        if (last_status == W25QXX_STATUS_TIMEOUT) last_status = W25QXX_STATUS_ERASE_FAILED;
        return false;
    }
    last_status = W25QXX_STATUS_OK;
    return true;
}

bool W25Qxx_Write(uint32_t address, const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0U)
    {
        last_status = W25QXX_STATUS_INVALID_ARGUMENT;
        return false;
    }
    while (length != 0U)
    {
        uint32_t page_remaining = W25QXX_PAGE_SIZE - (address % W25QXX_PAGE_SIZE);
        uint32_t chunk = length < page_remaining ? length : page_remaining;
        write_enable();
        select_chip();
        (void)transfer(W25QXX_COMMAND_PAGE_PROGRAM);
        send_address(address);
        for (uint32_t i = 0U; i < chunk; i++) (void)transfer(data[i]);
        deselect_chip();
        if (!wait_ready())
        {
            if (last_status == W25QXX_STATUS_TIMEOUT) last_status = W25QXX_STATUS_PROGRAM_FAILED;
            return false;
        }
        address += chunk;
        data += chunk;
        length -= chunk;
    }
    last_status = W25QXX_STATUS_OK;
    return true;
}

w25qxx_status_t W25Qxx_GetLastStatus(void) { return last_status; }
