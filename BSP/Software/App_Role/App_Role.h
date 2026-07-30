#ifndef APP_ROLE_H
#define APP_ROLE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    APP_ROLE_CAR = 0,
    APP_ROLE_ARM = 1
} app_role_t;

void App_Role_Init(void);
app_role_t App_Role_Get(void);
bool App_Role_Set_Persisted(app_role_t role);
const char *App_Role_Get_Name(void);
uint8_t App_Role_Get_ANO_Address(void);

#endif /* APP_ROLE_H */
