char rc =' ';


void setup()
{
Serial.begin(9600);
Serial2.begin(9600);
}


void loop()
{
rc =  Serial.read();
Serial.print(rc);
delayMicroseconds(500);
}



  
