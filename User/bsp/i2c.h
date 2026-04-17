#ifndef __I2C_H__
#define __I2C_H__

#include "stm32g4xx_hal.h"

typedef enum
{
    I2C_READ_STATE_BUSY = 0U,
    I2C_READ_STATE_DONE
} i2c_readState_e;

void i2c_init(void);
void i2c_read_bytesBlock(uint16_t dev_addr, uint16_t reg, uint8_t *recv_buffer, uint8_t len);
void i2c_read_bytesAsync(uint16_t dev_addr, uint16_t reg, uint8_t *recv_buffer, uint8_t len);
i2c_readState_e i2c_get_readState(void);
void i2c_set_readState(i2c_readState_e current_state);

#endif /* __I2C_H__ */
