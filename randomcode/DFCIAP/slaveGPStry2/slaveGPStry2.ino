    /*************************************************************************
     * 
     * objeeAirlines
     * __________________
     * 
     *  Written by Julian Blanco
     *  
     *  Spi slave code written by
        Caleb Stewart and Julian Blanco
        Jan 2017
     */


    #include "GNSS.h"

    #define mySerial Serial
    GNSS GNSS(&mySerial);

    int Fix=1;
    int new_GPS_data = 1;
    volatile float myLat = 3.14;
    volatile float myLong = 6.28;
    volatile float GpsSpeed = 0;
    volatile float Gpsheading = 0;
    volatile float GpsAltitude = 0;
    volatile float GpsSat = 0;
    volatile float Hour = 0;
    volatile float Minute = 0;
    volatile float Seconds = 0;

    int calibration=0;

    float spi_buf =0;
    volatile int spi_idx =0;
    volatile byte command = 0;

    void setup (void)
    {
 // CLKPR = (1 << CLKPCE);
 // CLKPR = 0x01;
      Serial.begin(9600);
      GNSS.begin(9600);
     
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

        case 'a': spi_buf = myLat; spi_idx = 0; break;
        case 'b': spi_buf = myLong; spi_idx = 0; break;
        case 'c': spi_buf = GpsSpeed; spi_idx = 0; break;
        case 'd': spi_buf = Gpsheading; spi_idx = 0; break;
        case 'e': spi_buf = GpsAltitude; spi_idx = 0; break;
        case 'f': spi_buf = GpsSat; spi_idx = 0; break;
        case 'h': spi_buf = Hour; spi_idx = 0; break;
        case 'i': spi_buf = Minute; spi_idx = 0; break;
        case 'j': spi_buf = Seconds; spi_idx = 0; break;
        //case 'z': spi_idx = spi_idx; break;


      } // end of switch
      
    SPDR = ((char*)(&spi_buf))[spi_idx]; 
    spi_idx = (spi_idx + 1) & 0x3; // increment and modulus 4 to reset

    }  // end of interrupt service routine (ISR) SPI_STC_vect


  


    float tiempo=millis();

    void loop (void)
    {
      
      // if SPI not active, clear current command
      if (digitalRead (SS) == HIGH) command = 0;
       char c = GNSS.read();
      if (c) UDR0 = c; //serial print
       Serial_Sample();
      

    }  // end of loop



void Serial_Sample() {

    if (GNSS.newNMEAreceived()) {
        if (GNSS.parse(GNSS.lastNMEA()))                          // this also sets the newNMEAreceived() flag to false
        { 

            Fix=1;
            new_GPS_data = 1;
            myLat=GNSS.latitudeDegrees;
            myLong=GNSS.longitudeDegrees;
            GpsSpeed=GNSS.speed;
            Gpsheading=GNSS.angle;
            GpsAltitude=GNSS.altitude;
            GpsSat=GNSS.satellites;
            Hour=GNSS.hour;
            Minute=GNSS.minute;
            Seconds=GNSS.seconds;
            //Serial.println(myLat);
        }
  
      
    }

  }