double myLat =  41.327592;
double myLong = -72.045777;
double utmeast=0;
double utmnorth=0;
String zonez ="";

double calclat;
double calclong;


void setup(){
  Serial.begin(9600);

}
void loop(){
 latLongToUtm(myLat,myLong, utmeast, utmnorth, zonez);
 Serial.print("mylat: ");Serial.print(myLat,6);
 Serial.print(" myLong: ");Serial.print(myLong,6);
 Serial.print(" north: ");Serial.print(utmnorth,2);
 Serial.print(" east: ");Serial.print(utmeast,2);

 utmToLatLong(utmeast,utmnorth, "18T" ,calclat,calclong);
 Serial.println("");
 Serial.println("");

  Serial.print("calclat: ");Serial.print(calclat,6);
 Serial.print(" calcLong: ");Serial.print(calclong,6);

 Serial.println("");
 Serial.println("");


delay(10000);
}


#define         WGS84_A               6378137.0
#define         WGS84_ECCSQ           0.00669437999013
//#define         WGS84_A               6378137.0
//#define         WGS84_ECCSQ           0.00669438002290    //NAD

/* This routine determines the correct UTM letter designator for the given
   latitude and returns 'Z' if latitude is outside the UTM limits of 84N to 80S
   Written by Chuck Gantz- chuck.gantz@globalstar.com */

char UTMLetterDesignator(double Lat)
{
  char LetterDesignator;
  
  if((84 >= Lat) && (Lat >= 72)) LetterDesignator = 'X';
  else if((72 > Lat) && (Lat >= 64)) LetterDesignator = 'W';
  else if((64 > Lat) && (Lat >= 56)) LetterDesignator = 'V';
  else if((56 > Lat) && (Lat >= 48)) LetterDesignator = 'U';
  else if((48 > Lat) && (Lat >= 40)) LetterDesignator = 'T';
  else if((40 > Lat) && (Lat >= 32)) LetterDesignator = 'S';
  else if((32 > Lat) && (Lat >= 24)) LetterDesignator = 'R';
  else if((24 > Lat) && (Lat >= 16)) LetterDesignator = 'Q';
  else if((16 > Lat) && (Lat >= 8)) LetterDesignator = 'P';
  else if(( 8 > Lat) && (Lat >= 0)) LetterDesignator = 'N';
  else if(( 0 > Lat) && (Lat >= -8)) LetterDesignator = 'M';
  else if((-8> Lat) && (Lat >= -16)) LetterDesignator = 'L';
  else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
  else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
  else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
  else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
  else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
  else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
  else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
  else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
  else LetterDesignator = 'Z'; 
  return LetterDesignator;
}



void latLongToUtm(double Lat, double Long, double& UTMEasting, double& UTMNorthing, String& UTMZone) {
  double LongOrigin, LongOriginRad;
  double eccPrimeSquared;
  double k0 = 0.9996, N, T, C, A, M;
  double LatRad = Lat * M_PI / 180.0;
  double LongRad = Long * M_PI / 180.0;
  int ZoneNumber;

  ZoneNumber = (int)((Long + 180) / 6) + 1;
  
  if(Lat >= 56.0 && Lat < 64.0 && Long >= 3.0 && Long < 12.0)
    ZoneNumber = 32;
  
  // Special zones for Svalbard
  if(Lat >= 72.0 && Lat < 84.0) {
    if(Long >= 0.0  && Long <  9.0) ZoneNumber = 31;
    else if(Long >= 9.0  && Long < 21.0) ZoneNumber = 33;
    else if(Long >= 21.0 && Long < 33.0) ZoneNumber = 35;
    else if(Long >= 33.0 && Long < 42.0) ZoneNumber = 37;
  }
  // +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;  
  LongOriginRad = LongOrigin * M_PI / 180.0;

  // compute the UTM Zone from the latitude and longitude
//  sprintf(UTMZone, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));
  //Stringstream sstr;
  Serial.print(" Zone Number: ");
 Serial.print(ZoneNumber);Serial.println(UTMLetterDesignator(Lat));
  //UTMZone = sstr.str(); fizme

  eccPrimeSquared = WGS84_ECCSQ / (1 - WGS84_ECCSQ);
  N = WGS84_A / sqrt(1 - WGS84_ECCSQ * sin(LatRad) * sin(LatRad));
  T = tan(LatRad) * tan(LatRad);
  C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
  A = cos(LatRad) * (LongRad-LongOriginRad);
  M = WGS84_A * ((1 - WGS84_ECCSQ / 4 - 3 * WGS84_ECCSQ * WGS84_ECCSQ / 64
            - 5 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 256) * LatRad 
           - (3 * WGS84_ECCSQ / 8 + 3 * WGS84_ECCSQ * WGS84_ECCSQ / 32
              + 45 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 1024) * 
           sin(2 * LatRad) + (15 * WGS84_ECCSQ * WGS84_ECCSQ / 256 +
                              45 * WGS84_ECCSQ * WGS84_ECCSQ * 
                              WGS84_ECCSQ / 1024) * sin(4 * LatRad) 
           - (35 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 3072) * 
           sin(6 * LatRad));
 UTMEasting = (double)(k0 * N * (A + (1 - T + C) * A * A * A / 6
                                   + (5 - 18 * T + T * T + 72 * C - 
                                      58 * eccPrimeSquared)* 
                                  A * A * A * A *A / 120) + 500000.0);
  UTMNorthing = (double)(k0 * (M + N * tan(LatRad) * 
                                (A * A / 2 + (5 - T + 9 * C + 4 * C * C)
                                 * A * A * A *A / 24
                                 + (61 - 58 * T + T * T + 
                                    600 * C - 330 * eccPrimeSquared) * 
                                 A * A * A * A * A * A / 720)));
  if(Lat < 0)
    UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere
}

// void latLongToUtm(double Lat, double Long, double* UTMEasting, double* UTMNorthing, char* UTMZone) {
//   String zone;
//   latLongToUtm(Lat, Long, UTMEasting, UTMNorthing, zone);
//   strcpy(UTMZone, zone.c_str());
// }
  /* converts UTM coords to lat/long.  Equations from USGS Bulletin 1532 
     East Longitudes are positive, West longitudes are negative. 
     North latitudes are positive, South latitudes are negative
     Lat and Long are in decimal degrees. 
     Written by Chuck Gantz- chuck.gantz@globalstar.com */

void utmToLatLong(double UTMEasting, double UTMNorthing, const String& UTMZone, double& Lat,  double& Long)
{
  double k0 = 0.9996, eccPrimeSquared, N1, T1, C1, R1, D, M;
  double e1 = (1 - sqrt(1 - WGS84_ECCSQ))/(1 + sqrt(1 - WGS84_ECCSQ));
  double LongOrigin, mu, phi1, phi1Rad, x, y;
  int ZoneNumber, NorthernHemisphere; // 1 for northern hem., 0 for southern
  char* ZoneLetter;
  
  x = UTMEasting - 500000.0; /* remove 500,000 meter offset for longitude */
  y = UTMNorthing;
  
  ZoneNumber = strtoul(UTMZone.c_str(), &ZoneLetter, 10);
  if((*ZoneLetter - 'N') >= 0)
    NorthernHemisphere = 1;  /* point is in northern hemisphere */
  else {
    NorthernHemisphere = 0;  /* point is in southern hemisphere */
    y -= 10000000.0;         /* remove 10,000,000 meter offset 
                                used for southern hemisphere */
  }
  
  /* +3 puts origin in middle of zone */
  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;  
  
  eccPrimeSquared = (WGS84_ECCSQ) / (1 - WGS84_ECCSQ);
  
  M = y / k0;
  mu = M / (WGS84_A * (1 - WGS84_ECCSQ / 4 - 
                       3 * WGS84_ECCSQ * WGS84_ECCSQ / 64 - 5 * WGS84_ECCSQ * 
                       WGS84_ECCSQ * WGS84_ECCSQ / 256));
  phi1Rad = mu + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * mu) +
    (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * mu) +
    (151 * e1 * e1 * e1 / 96) * sin(6 * mu);
  phi1 = dgc_r2d(phi1Rad);
  
  N1 = WGS84_A / sqrt(1 - WGS84_ECCSQ * sin(phi1Rad) * sin(phi1Rad));
  T1 = tan(phi1Rad) * tan(phi1Rad);
  C1 = eccPrimeSquared * cos(phi1Rad) * cos(phi1Rad);
  R1 = WGS84_A * (1 - WGS84_ECCSQ) / 
    pow(1 - WGS84_ECCSQ * sin(phi1Rad) * sin(phi1Rad), 1.5);
  D = x / (N1 * k0);
  
  Lat = phi1Rad - (N1 * tan(phi1Rad) / R1) * 
    (D * D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * eccPrimeSquared) * 
     D * D * D * D / 24 +
     (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 
      252 * eccPrimeSquared - 3 * C1 * C1) * D * D * D * D * D * D / 720);
  Lat = dgc_r2d(Lat);

  Long = (D - (1 + 2 * T1 + C1) * D * D * D / 6 + 
           (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 
            8 * eccPrimeSquared + 24 * T1 * T1)
          * D * D * D * D * D / 120) / cos(phi1Rad);
  Long = LongOrigin + dgc_r2d(Long);
}

void utmToLatLong(double UTMEasting, double UTMNorthing, const char UTMZone, double& Lat,  double& Long) {
  String zone = UTMZone;
  utmToLatLong(UTMEasting, UTMNorthing, zone, Lat, Long);
}




inline double dgc_r2d(double theta) {
    return (theta * 180.0 / M_PI);
}

inline double dgc_d2r(double theta) {
    return (theta * M_PI / 180.0);
}

inline float dgc_r2df(float theta) {
    return (theta * 180.0 / M_PI);
}