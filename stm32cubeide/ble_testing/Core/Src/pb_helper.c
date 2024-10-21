#include "pb_helper.h"

int8_t set_i2c_mux_index(I2C_HandleTypeDef *i2c_bus, uint8_t index)
{
	if (index > 7)return -1;
	index = 0x1 << index;
	HAL_I2C_Master_Transmit(i2c_bus, I2C_MUX_ADDR << 1, &index, 1, 1000);
}

void put_double_in_buffer(double db, uint8_t *buffer, int buffer_length, int starting_index)
{
	if((starting_index + 4) > buffer_length)return;
	  union {
	    double dbl;
	    unsigned char bytes[8];
	  } double_to_bytes;
	double_to_bytes.dbl = db;
	for(int i = 0; i < 8; i++){
		buffer[starting_index+i] = double_to_bytes.bytes[i];
	}
}
