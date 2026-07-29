#ifndef G030_I2C_LINK_H
#define G030_I2C_LINK_H

#include <stdint.h>

typedef struct
{
    volatile uint8_t response_valid;
    volatile uint8_t status;
    volatile uint8_t led_blinking;
    volatile uint32_t tx_error_count;
    volatile uint32_t rx_error_count;
} g030_i2c_link_state_t;

extern g030_i2c_link_state_t g030_i2c_link_state;

void G030_I2C_Link_Init(void);
void G030_I2C_Link_Process(void);

#endif
