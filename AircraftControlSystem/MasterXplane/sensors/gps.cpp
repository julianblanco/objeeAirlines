#include "gps.h"


void Serial_Sample(GNSS &gpsobject) {

    if (gpsobject.newNMEAreceived()) {
        if (gpsobject.parse(gpsobject.lastNMEA()))                          // this also sets the newNMEAreceived() flag to false
        {
              Fix=1;
            new_GPS_data = 1;
            myLat=gpsobject.latitudeDegrees;
            projectedLat=myLat;
            myLong=gpsobject.longitudeDegrees;
            projectedLong = myLong;
            GpsSpeed=gpsobject.speed;
            Gpsheading=gpsobject.angle;
            GpsAltitude=gpsobject.altitude;
            GpsSat=gpsobject.satellites;
            Hour=gpsobject.hour;
            Minute=gpsobject.minute;
            Seconds=gpsobject.seconds;
        }
  
      
    }
}



