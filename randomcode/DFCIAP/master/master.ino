#define SS 10
#include <SPI.h>

void setup (void)
{
  Serial.begin (115200);
  delay(10);
  Serial.println ("Begin SPI");
  pinMode (SS, OUTPUT);
  digitalWrite(SS, HIGH);  
  SPI.begin ();
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  
}  // end of setup

int idx =1;
void loop (void)
{   
      float myLat = spitrans(10,'a');
      float myLong = spitrans(10,'b');
      Serial.print("myLat = ");Serial.println(myLat,6);   
      Serial.print("myLong = ");Serial.println(myLong,6);     
      Serial.println((idx*1000)/millis());
      delay(1);
      idx++;

}  // end of loop






float spitrans(int deviceline, char deviceselect)

{
      digitalWrite(deviceline, LOW);
      char databuff[4] = {0,0,0,0};//,'b','z','z'}; 
      SPI.transfer(deviceselect);
      delay(1);
      databuff[0]=SPI.transfer(0);
      delay(1);
      databuff[1]=SPI.transfer(0);
      delay(1);
      databuff[2]=SPI.transfer(0);
      delay(1);
      databuff[3]=SPI.transfer(0);
      float data = *((float*)databuff);
      digitalWrite(deviceline, HIGH);
      return data;

}