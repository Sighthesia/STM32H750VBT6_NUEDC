#ifndef APP_ROLE_H
#define APP_ROLE_H

#include <stdint.h>

typedef enum
{
    APP_ROLE_CAR = 0,
    APP_ROLE_ARM = 1
} app_role_t;

app_role_t App_Role_Get(void);
const char *App_Role_Get_Name(void);
uint8_t App_Role_Get_ANO_Address(void);

#endif /* APP_ROLE_H */
