    /*************************************************************************
     * 
     * objeeAirlines
     * __________________
     * 
     *  Written by Julian Blanco
     *  
     *  Spi slave code written by
        Nick Gammon
        February 2011
     */


    #include "Adafruit_BNO055.h"

    float yawInput=0;
    float yawOffset=0;
    float pitchInput=0;
    float rollInput=0;

    int calibration=0;

    volatile byte command = 0;
    Adafruit_BNO055 bno = Adafruit_BNO055();

    void setup (void)
    {

      if(!bno.begin())while(1){};
      bno.setExtCrystalUse(true);
      // have to send on master in, *slave out*
      pinMode(MISO, OUTPUT);
      // turn on SPI in slave mode
      SPCR |= _BV(SPE);
      // turn on interrupts
      SPCR |= _BV(SPIE);
    }  // end of setup

    // SPI interrupt routine
    ISR (SPI_STC_vect)
    {
      union first_union{
        float f;
        byte b[4];}
      data;
      
      byte c = SPDR;
      
      data.f = 10.345;

      command = c; 

      switch (command)
      {
      // no command? then this is the command
      case 0:
        
        SPDR = 0;
        break;
        
      // incoming byte, return byte result
      case 'a':
        
        SPDR = data.b[0];  
        break;
        
      // incoming byte, return byte result
      case 'b':
        
        SPDR = data.b[1];  
        break;

      // incoming byte, return byte result    
      case 'c':
        
        SPDR = data.b[2];  
        break;

      // incoming byte, return byte result    
      case 'd':
        
        SPDR = data.b[3];  
        break;

      } // end of switch

    }  // end of interrupt service routine (ISR) SPI_STC_vect

    float tiempo=millis();
    void loop (void)
    {
      
      // if SPI not active, clear current command
      if (digitalRead (SS) == HIGH) command = 0;

        if(millis()>(tiempo+10))
        {
          tiempo=millis();

          SampleGyro(bno);
        }

    }  // end of loop

void SampleGyro(Adafruit_BNO055 &gyroIMU){
    /* Display the floating  point data */
    imu::Vector<3> euler = gyroIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
    // Disable interrupts while modifying the global variable

    yawInput=euler.x()-yawOffset;

    if (yawInput >= 360) {
          yawInput -= 360;
    } else if (yawInput < 0) {
          yawInput += 360;
    }
    pitchInput=euler.y();
    //rollInput=fmod((euler.z()+(360+90)), 360)-180;
    rollInput=euler.z();
   // uint8_t system, gyro, accel, mag = 0;
    //gyroIMU.getCalibration(&system, &gyro, &accel, &mag);
    //calibration=system;
    // re-enable interrupts once your global variable
    // modification is done.
 
}