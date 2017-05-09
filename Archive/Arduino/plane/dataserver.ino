#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup(void)
{
  Serial.begin(9600);

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
  int x=euler.x());
  int y=euler.y());
  int z=euler.z());
  String bla = string(itoa(sys))+","+itoa(x)+","+itoa(y)+","+itoa(z);
  Serial.println(bla);
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
