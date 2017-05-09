#ifndef _I2C_MASTER_H_
#define _I2C_MASTER_H_

#include "message.h"
#include <wiringPiI2C.h>
#include <stdarg.h>

int setup_i2c( int addr );

void message(const char* fmt, ...);
void panic(const char* fmt, ...);

#endif
