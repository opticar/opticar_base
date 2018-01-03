#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

bool I2C_Init();
bool I2C_Write(uint8_t addr, uint8_t reg, uint8_t data);
bool I2C_WriteBurst(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t size);
uint8_t I2C_Read(uint8_t addr, uint8_t reg);
bool I2C_ReadBurst(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t size);
bool I2C_Timeout();

#ifdef __cplusplus
}
#endif
