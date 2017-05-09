
#include <cstdint>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
using namespace std;

int main()

{

    float hour=0;
    float seconds=0;
    int degree=0;
    float milliseconds=0;
    long minutes;
    float minute=0;
    char degreebuff[10];
    float latitude_fixed=0;
    float latitude=0;
    float latitudeDegrees=0;

    string a="$GNGGA,155858,522.2155,N,-73540.0789,W,1,08,0.9,2.3378,M,46.9,M,,*47";
      cout<< a <<endl;
    
    const char *p = a.c_str();

    // get time
    p = strchr(p, ',')+1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);

    milliseconds = fmod(timef, 1.0) * 1000;

    // parse out latitude
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 2);
      p += 2;
      degreebuff[2] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      latitude_fixed = degree + minutes;
      latitude = degree / 100000 + minutes * 0.000006F;
      latitudeDegrees = (latitude-100*int(latitude/100))/60.0;
      latitudeDegrees += int(latitude/100);
    }
      cout <<latitudeDegrees <<endl;
  
}