#ifndef _MESSAGE_H_
#define _MESSAGE_H_

#include <stdint.h>

#define I2C_SLAVE_ADDR 16

typedef struct _vector
{
	float x, y, z;
} vector_t;

int send_data(const void* data, uint8_t size);
int recv_data(void* data, uint8_t size);

#endif