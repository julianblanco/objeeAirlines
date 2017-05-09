
class GPSDATA
{
  public:
    float rawLat;
    float rawLong;
    float GpsHead;
    float GpsSpeed;
    float GpsAltitude;
    float GpsFixQual;
    char Fix;
    int GpsSat;
    float myLat;
    float myLong;
    float SOG;
    float Gpsheading;
};


class userIO
{
  public:
    int useroverrideFlag;
    int gpsUserClear;
};


class gyro
{
  public:
    float lastEuler;
    float currentEuler;
};

class altitudeData
{
  public:

    float barometer;
};

class navigationData
{
  public:
    float headingError;
    int waypointNumber;
    int flagdist;
    float distanceToTarget;         // current distance to target (current waypoint)
    float targetLat;
    float targetLong;
    float trueHeading;
    float targetHeading;
    float precision;
};
