#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define BNO055_SAMPLERATE_DELAY_MS (50)

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup(void)
{
  Serial.begin(115200);

  if(!bno.begin()) while(1);
  delay(1);
  bno.setExtCrystalUse(true);
}

void loop(void)
{
  
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  int sys=system;
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float x=euler.x();
  float y=euler.y();
  float z=euler.z();
  String bla = "$GNGYRO,"+String(sys)+","+String(x)+","+String(y)+","+String(z)+','+"*****";
  Serial.println(bla);
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
