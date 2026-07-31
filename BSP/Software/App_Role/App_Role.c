#include "App_Role.h"

#include "Flash_Layout.h"
#include "W25Qxx.h"

#include <string.h>

#define APP_ROLE_STORAGE_MAGIC   0x524F4C45UL
#define APP_ROLE_STORAGE_VERSION 1U

_Static_assert(FLASH_APP_ROLE_STORAGE_ADDRESS % W25QXX_FLASH_SECTOR_SIZE == 0U,
               "App role storage must be sector aligned");
_Static_assert(FLASH_APP_ROLE_STORAGE_ADDRESS != FLASH_ARM_CALIBRATION_STORAGE_A &&
               FLASH_APP_ROLE_STORAGE_ADDRESS != FLASH_ARM_CALIBRATION_STORAGE_B,
               "App role storage must not overlap arm calibration slots");

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint8_t role;
    uint8_t reserved[2];
    uint32_t checksum;
} app_role_record_t;

static app_role_t active_role = APP_ROLE_CAR;

static uint32_t App_Role_Checksum(const app_role_record_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t checksum = 0x9E3779B9U;

    for (uint32_t i = 0U; i < sizeof(*record) - sizeof(record->checksum); i++)
    {
        checksum = (checksum << 5U) ^ (checksum >> 27U) ^ data[i];
    }
    return checksum;
}

void App_Role_Init(void)
{
    app_role_record_t record = {0};

    active_role = APP_ROLE_CAR;
    if (!W25Qxx_Init())
    {
        return;
    }
    if (W25Qxx_Read(FLASH_APP_ROLE_STORAGE_ADDRESS, (uint8_t *)&record, sizeof(record)) &&
        record.magic == APP_ROLE_STORAGE_MAGIC && record.version == APP_ROLE_STORAGE_VERSION &&
        record.role <= APP_ROLE_ARM && record.checksum == App_Role_Checksum(&record))
    {
        active_role = (app_role_t)record.role;
        return;
    }

    memset(&record, 0, sizeof(record));
    if (W25Qxx_Read(FLASH_APP_ROLE_LEGACY_ADDRESS, (uint8_t *)&record, sizeof(record)) &&
        record.magic == APP_ROLE_STORAGE_MAGIC && record.version == APP_ROLE_STORAGE_VERSION &&
        record.role <= APP_ROLE_ARM && record.checksum == App_Role_Checksum(&record))
    {
        active_role = (app_role_t)record.role;
        (void)App_Role_Set_Persisted(active_role);
    }
}

app_role_t App_Role_Get(void)
{
    return active_role;
}

bool App_Role_Set_Persisted(app_role_t role)
{
    app_role_record_t record = {0};
    app_role_record_t verify = {0};

    if (role > APP_ROLE_ARM || !W25Qxx_Init())
    {
        return false;
    }
    record.magic = APP_ROLE_STORAGE_MAGIC;
    record.version = APP_ROLE_STORAGE_VERSION;
    record.role = (uint8_t)role;
    record.checksum = App_Role_Checksum(&record);
    return W25Qxx_EraseSector(FLASH_APP_ROLE_STORAGE_ADDRESS) &&
           W25Qxx_Write(FLASH_APP_ROLE_STORAGE_ADDRESS, (const uint8_t *)&record, sizeof(record)) &&
           W25Qxx_Read(FLASH_APP_ROLE_STORAGE_ADDRESS, (uint8_t *)&verify, sizeof(verify)) &&
           memcmp(&record, &verify, sizeof(record)) == 0;
}

const char *App_Role_Get_Name(void)
{
    return active_role == APP_ROLE_ARM ? "STM32H750_ARM" : "STM32H750_CAR";
}

uint8_t App_Role_Get_ANO_Address(void)
{
    return active_role == APP_ROLE_ARM ? 0x06U : 0x05U;
}
