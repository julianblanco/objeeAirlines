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
    #include "Adafruit_BMP280.h"
    float pressure=0;
    float altitude=0;
    float intialpressure=0;
    volatile byte command = 0;
    Adafruit_BMP280 barometer;

    void setup (void)
    {
      //Serial.begin(9600);
      //Serial.println("Begin");
      if (!barometer.begin())while (1);
      // turn on SPI in slave mode
      SPCR |= _BV(SPE);
      // turn on interrupts
      SPCR |= _BV(SPIE);
      intialpressure=barometer.readPressure();
    }  // end of setup

    // SPI interrupt routine
    ISR (SPI_STC_vect)
    {
      union first_union{
        float f;
        byte b[4];}
      data;
      
      byte c = SPDR;
      
      data.f = 512;

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

        if(millis()>(tiempo+1000))
        {
          tiempo=millis();
          altitude=barometer.readAltitude(1013.25);
          //Serial.println(altitude);
        }



    }  // end of loop

