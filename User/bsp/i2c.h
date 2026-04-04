#ifndef __I2C_H__
#define __I2C_H__

#include "stm32g4xx_hal.h"

extern I2C_HandleTypeDef hi2c3;

void i2c_init(void);
void i2c_write_bytes(uint16_t reg, uint8_t *send_buffer, uint8_t len);
void i2c_read_bytes(uint16_t reg, uint8_t *recv_buffer, uint8_t len);

#endif /* __I2C_H__ */
