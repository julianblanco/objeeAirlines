#include "Communication.h"

//===================================================================
//
//	Program:		Communication.cpp
//
//	Author:			Julian Blanco
//	Date:			Sep 4 2015
//
//	Description:	Description
//
//===================================================================


// Read nmea string from circular buffer and parse when string is complete
char checkNMEA(float* lat, float* longitude)//pass in long)
{
  char buffered_char;
  char Fix;

  do
  {
    // Read any new characters from the circular buffer to Serial
    buffered_char = circ_buff_get();

    if (buffered_char != 0)
    {
      // Fill NMEA string char by char from buffer
      nmea_complete = fill_nmea(buffered_char);
      // Serial.print(buffered_char);

      // If nmea string is complete, call parsing function
      if (nmea_complete)
      {
        Fix=parse_GPS(nmea, lat, longitude); //pass in lat and long);
      }
    }
  } while (buffered_char != 0);
  return Fix;
}


// Populate nmea buffer as chars are read in from the circular buffer, return complete string flag
int fill_nmea(char new_char)
{
  // Place data in linear array, reset to beginning at carriage return
  static int index = -1;
  index++;
  nmea[index] = new_char;
  if (new_char == '\r')
  {
    index = 0;
    return (1);
  }
  else
  {
    return (0);
  }
}


// Parse GPS string
char parse_GPS(char* string, float* Lat, float* Long)
{
  char Fix;
  char temp_str[10];
  
  // For RMC NMEA string, use the 3rd field (comma delimited) as the fix quality
   if (strstr(string, "RMC") != NULL)
  {
    line_parser(string, ',');
    get_field(temp_str, 3);
    Fix = temp_str[0];
  }
  if (Fix == 'A')
  {
    // For GGA NMEA string, us the 3rd and 5th field as lat and long
    if (strstr(string, "GGA") != NULL)
    {
      line_parser(string, ',');

      get_field(temp_str, 3);
      *Lat = convertDegMinToDecDeg(atof(temp_str));
     
     
      get_field(temp_str, 5);
      *Long = convertDegMinToDecDeg(atof(temp_str));
    }
  }
 
  return Fix;
}


// Place new character into circular buffer **No overflow checking**
void circ_buff_set(char new_char)
{
  circ_buff[head] = new_char;
  head = (head + 1) & 127;
}


// Return char = 0 if there is no new data
char circ_buff_get(void)
{
  char temp_char = 0;
  if ((tail) != head) {
    temp_char = circ_buff[tail];
    tail = (tail + 1) & 127;
  }

  return (temp_char);
}


