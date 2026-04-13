#ifndef __I2C_H__
#define __I2C_H__

#include "stm32g4xx_hal.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c3;

void i2c_init(void);

void i2c_read_bytes(uint16_t dev_addr, uint16_t reg, uint8_t *recv_buffer, uint8_t len);
HAL_StatusTypeDef i2c_read_bytes_it(uint16_t dev_addr, uint16_t reg, uint8_t *recv_buffer, uint8_t len);
uint8_t i2c_read_is_busy(void);
uint8_t i2c_read_is_done(void);
void i2c_read_clear_done_flag(void);

#endif /* __I2C_H__ */
