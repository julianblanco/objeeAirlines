#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_
#include "line_parser.h"
//#include "variables.h"
#include "GPS.h"
#include <Wire.h>



//used in communication.h
static int head = 0;
static int tail = 0;
static char circ_buff[128];
static char nmea[128];
static int z = 0;
static int nmea_complete = 0;


// Populate nmea buffer as chars are read in from the circular buffer, return complete string flag
int fill_nmea(char new_char);
// Parse GPS string
char parse_GPS(char* stringg, float* Lat, float* Long);

// Place new character into circular buffer **No overflow checking**
void circ_buff_set(char new_char);
// Return char = 0 if there is no new data
char circ_buff_get(void);
char checkNMEA(float* lat, float* longitude);

#endif // _COMMUNICATION_H_
