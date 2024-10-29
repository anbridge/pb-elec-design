/*
 * pb_helper.h
 *
 *  Created on: Oct 20, 2024
 *      Author: main
 */
#include <stdint.h>
#include "stm32wbxx_hal.h"

#define I2C_MUX_ADDR 0x70

#ifndef PB_HELPER_H_
#define PB_HELPER_H_


extern int8_t set_i2c_mux_index(I2C_HandleTypeDef *i2c_bus, uint8_t index);
extern void put_double_in_buffer(double db, uint8_t *buffer, int buffer_length, int starting_index);
extern void put_float_in_buffer(float ft, uint8_t *buffer, int buffer_length, int starting_index);


#endif /* PB_HELPER_H_ */
