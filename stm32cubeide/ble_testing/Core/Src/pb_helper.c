#include "pb_helper.h"

int8_t set_i2c_mux_index(I2C_HandleTypeDef *i2c_bus, uint8_t index)
{
	if (index > 7)return -1;
	index = 0x1 << index;
	HAL_I2C_Master_Transmit(i2c_bus, I2C_MUX_ADDR << 1, &index, 1, 1000);
	return 0;
}

void put_double_in_buffer(double db, uint8_t *buffer, int buffer_length, int starting_index)
{
	if((starting_index + sizeof(double)) > buffer_length)return;
	  union {
	    double dbl;
	    unsigned char bytes[sizeof(double)];
	  } double_to_bytes;
	double_to_bytes.dbl = db;
	for(int i = 0; i < sizeof(double); i++){
		buffer[starting_index+i] = double_to_bytes.bytes[i];
	}
}

void put_float_in_buffer(float ft, uint8_t *buffer, int buffer_length, int starting_index)
{
	if((starting_index + sizeof(float)) > buffer_length)return;
	  union {
	    float ftl;
	    unsigned char bytes[sizeof(float)];
	  } float_to_bytes;
	float_to_bytes.ftl = ft;
	for(int i = 0; i < sizeof(float); i++){
		buffer[starting_index+i] = float_to_bytes.bytes[i];
	}
}

void put_int32_in_buffer(int32_t i, uint8_t *buffer, int buffer_length, int starting_index)
{
	if((starting_index + sizeof(int32_t)) > buffer_length)return;
	  union {
	    int32_t intl;
	    unsigned char bytes[sizeof(int32_t)];
	  } int32_to_bytes;
	int32_to_bytes.intl = i;
	for(int i = 0; i < sizeof(int32_t); i++){
		buffer[starting_index+i] = int32_to_bytes.bytes[i];
	}
}

