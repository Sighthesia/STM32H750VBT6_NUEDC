#include "App_Role.h"

app_role_t App_Role_Get(void)
{
    return APP_ROLE_ARM;
}

const char *App_Role_Get_Name(void)
{
    return "STM32H750_ARM";
}

uint8_t App_Role_Get_ANO_Address(void)
{
    return 0x06U;
}
