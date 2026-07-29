#ifndef AT24C02_H
#define AT24C02_H

#include <stdbool.h>
#include <stdint.h>

bool AT24C02_Read(uint8_t address, uint8_t *data, uint16_t length);
bool AT24C02_Write(uint8_t address, const uint8_t *data, uint16_t length);

#endif /* AT24C02_H */
