
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include "files/Adafruit_Sensor.h"
#include "files/Adafruit_BNO055.h"
#include "files/Adafruit_BMP085.h"
#include "files/Adafruit_BMP280.h"
#include "files/LPS.h"
#include "files/utility/imumaths.h"
#include "files/GNSS.h"


#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/gps.h"
#include "sensors/pitot.h"
#include "sensors/sdcard.h"
#include "sensors/sonar.h"

#include "tactics.h"
#include "Navigation.h"
#include "craftcontrol.h"
#include "serial.h"
#include "plservo.h"

// #ifdef ARDUINO_SAMD_ZERO
// 	#include "files/ServoM.h"
// 	#include "files/ServoTimerM.h"
// #else
// 	#ifdef TEENSY
		#include <Servo.h>
// 	#endif
// #endif