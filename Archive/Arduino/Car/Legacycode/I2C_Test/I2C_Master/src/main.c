#include "I2C_Master.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <linux/i2c-dev.h>

#define I2C_DEVICE "/dev/i2c-1"

int i2cfd = -1;
const char* program_name = "I2C_Master";

int main(int argc, char** argv)
{
	// initialize the i2c bus on the specified address
	if( setup_i2c(I2C_SLAVE_ADDR) == -1 ){
		panic("unable to setup i2c bus on address %d", I2C_SLAVE_ADDR);
	}

	message("setup i2c bus on address %d.", I2C_SLAVE_ADDR);
 	
	// set program name for messages
	program_name = argv[0];

	// create a data packet to send
	vector_t vector;
	vector.x = 3;
	vector.y = .14;
	vector.z = 0.00159;
	
	// send the data
	message("sending vector={%f,%f,%f}", vector.x, vector.y, vector.z);
	send_data(&vector, sizeof(vector));
	message("vector sent!");

	message("requesting data...");

	char buffer[64] = {0};
	ssize_t len = 0;
	size_t i = 0;

	char cmd = 0;
	write(i2cfd, &cmd, 1);

	do {
		read(i2cfd, &cmd, 1);
		buffer[i++] = cmd;
	} while( i < 64 && cmd != 0 );

	if( len < 0 ){
		panic("unable to read i2c bus");
	}

	message("recieved: %s", buffer);

/*
	for(int i = 0; i <  25; ++i){
		int result = 0;
		while( (result = wiringPiI2CRead(i2cfd)) == 0 );
		if( result == -1 ){
			perror("i2c read");
		}
		message("recieved: \'%c\' (0x%X)", result, result);
	}
*/
	//message("recieved: %d", result);
	
	// exiting
	message("done. exiting...");

	close(i2cfd);
	
	return 0;
}

// print an error message and exit
void panic(const char* fmt, ...)
{
	char buffer[512];
	va_list va;
	va_start(va, fmt);
	vsprintf(buffer, fmt, va);
	va_end(va);
	
	perror(buffer);
	exit(-1);
}

// print a nice message with the program name
void message(const char* fmt, ...)
{
	char buffer[512];
	va_list va;
	va_start(va, fmt);
	vsprintf(buffer, fmt, va);
	va_end(va);
	
	printf("%s: %s\n", program_name, buffer);
}

// Setup the i2c connection
int setup_i2c( int addr )
{
	i2cfd = open(I2C_DEVICE, O_RDWR);
	if( i2cfd < 0 ){
		return -1;
	}

	if( ioctl(i2cfd, I2C_SLAVE, addr) < 0 ){
		return -1;
	}

/*
	// initialize wiringPi and the address and check for error
	if( (i2cfd = wiringPiI2CSetup(addr)) == -1 ){
		return -1;
	}
*/
	return 0;
}

// send a buffer over i2c to the device
int send_data(const void* data, uint8_t size)
{
	write(i2cfd, &size, 1);
	write(i2cfd, data, size);
/*
	for(uint32_t i = 0; i < size; ++i){
		write(i2cfd, ((uint32_t*)((uint32_t)data + i)), 1);
	}
*/
	return 0;
}
