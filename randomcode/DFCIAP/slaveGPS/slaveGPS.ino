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


    #include "GNSS.h"

    #define mySerial Serial
    GNSS GNSS(&mySerial);

    int Fix=1;
    int new_GPS_data = 1;
    float myLat = 0;
    float myLong = 0;
    float GpsSpeed = 0;
    float Gpsheading = 0;
    float GpsAltitude = 0;
    float GpsSat = 0;
    float Hour = 0;
    float Minute = 0;
    float Seconds = 0;

    int calibration=0;

    volatile byte command = 0;

    void setup (void)
    {

      Serial.begin(38400);
      GNSS.begin(38400);
     
      // have to send on master in, *slave out*
      pinMode(MISO, OUTPUT);
      // turn on SPI in slave mode
      SPCR |= _BV(SPE);
      // turn on interrupts
      SPCR |= _BV(SPIE);

      Serial.println("hello");
    }  // end of setup

    // SPI interrupt routine
    ISR (SPI_STC_vect)
    {


      byte c = SPDR;
      
     

      command = c; 

      switch (command)
      {
      // no command? then this is the command
      case 0:
        
        SPDR = 0;
        break;
        
      // incoming byte, return byte result
      case 'a':
        
        AVR_SPI_slave_put_32(myLat);
        break;
        
      // incoming byte, return byte result
      case 'b':
        
        AVR_SPI_slave_put_32(myLong);
        break;

      // incoming byte, return byte result    
      case 'c':
        
        AVR_SPI_slave_put_32(GpsSpeed);
        break;

      // incoming byte, return byte result    
      case 'd':
        
        AVR_SPI_slave_put_32(GpsAltitude);
        break;

      } // end of switch

    }  // end of interrupt service routine (ISR) SPI_STC_vect


    


    void AVR_SPI_slave_put_32(uint32_t c){

      uint8_t dummy;
      SPDR = c >> 24;                          // upper byte sent first
      while(!(SPSR & (1 << SPIF)));
      dummy = SPDR;  
      SPDR = c >> 16;                          // upper byte sent first
      while(!(SPSR & (1 << SPIF)));
      dummy = SPDR;
      SPDR = c >> 8;                          // upper byte sent first
      while(!(SPSR & (1 << SPIF)));
      dummy = SPDR;                         // clear the SPI IF
      SPDR = c;                               // lower byte sent
      while(!(SPSR & (1 << SPIF)));
      dummy = SPDR;                           // clear the SPI IF
      }


    float tiempo=millis();

    void loop (void)
    {
      
      // if SPI not active, clear current command
      if (digitalRead (SS) == HIGH) command = 0;
      
       char c = GNSS.read();

       if (c) UDR0 = c;
       Serial_Sample();
      

    }  // end of loop



void Serial_Sample() {

    if (GNSS.newNMEAreceived()) {
        if (GNSS.parse(GNSS.lastNMEA()))                          // this also sets the newNMEAreceived() flag to false
        { 

            Fix=1;
            new_GPS_data = 1;
            myLat=GNSS.latitudeDegrees;
            //Serial.println(myLat);
            myLong=GNSS.longitudeDegrees;
            GpsSpeed=GNSS.speed;
            Gpsheading=GNSS.angle;
            GpsAltitude=GNSS.altitude;
            GpsSat=GNSS.satellites;
            Hour=GNSS.hour;
            Minute=GNSS.minute;
            Seconds=GNSS.seconds;
        }
  
      
    }

  }