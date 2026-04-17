#ifndef __I2C_H__
#define __I2C_H__

#include "stm32g4xx_hal.h"
#include <stdio.h>

typedef enum
{
    I2C_READ_STATE_BUSY = 0U,
    I2C_READ_STATE_DONE
} I2C_ReadState_t;

void i2c_init(void);
void i2c_read_bytesBlock(uint16_t dev_addr, uint16_t reg, uint8_t *recv_buffer, uint8_t len);
void i2c_read_bytesAsync(uint16_t dev_addr, uint16_t reg, uint8_t *recv_buffer, uint8_t len);
I2C_ReadState_t i2c_get_readState(void);
void i2c_set_readState(I2C_ReadState_t current_state);

#endif /* __I2C_H__ */
