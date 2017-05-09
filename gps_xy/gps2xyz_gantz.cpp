#pragma rtGlobals=1		// Use modern global access method.
CONSTANT  DEG2RAD = 0.01745329;
CONSTANT  RAD2DEG = 57.29577951;
Structure GPSpos
	variable UTMNorthing
	variable UTMEasting
	string UTMzone
Endstructure
 
Function LLtoUTM( ReferenceEllipsoid,   Lat,  Long, gps)
//converts lat/long to UTM coords.  Equations from USGS Bulletin 1532 
//East Longitudes are positive, West longitudes are negative. 
//North latitudes are positive, South latitudes are negative
//Lat and Long are in decimal degrees
	//Written by Chuck Gantz- chuck.gantz@globalstar.com
	variable ReferenceEllipsoid, Lat, Long
	struct GPSpos &gps
 
	variable UTMEasting,  UTMNorthing
	string UTMZone
 
	make/o/d/n=(23) EquatorialRadius, SquareOfEccentricity
	 make/o/n=23/T EllipsoidName
	 EquatorialRadius={6377563, 	6378160, 6377397, 6377484, 6378206, 6378249, 6377276, 6378166, 6378150, 6378160, 6378137, 6378200, 6378270, 6378388, 6378245, 6377340, 6377304, 6378155, 6378160, 6378165, 6378145, 6378135, 6378137}
	SquareOfEccentricity={.00667054,.006694542,.006674372,.006674372,.006768658,.006803511,.006637847,.006693422,.006693422,.006694605,.00669438,.006693422,.00672267,.00672267,.006693422,.00667054,.006637847,.006693422,.006694542,.006693422,.006694542,.006694318,.00669438}
	EllipsoidName={"Airy","Australian National","Bessel 1841","Bessel 1841 (Nambia) ","Clarke 1866","Clarke 1880","Everest","Fischer 1960 (Mercury) ","Fischer 1968","GRS 1967", "GRS 1980", "Helmert 1906","Hough",  "International", "Krassovsky", "Modified Airy", "Modified Everest",	 "Modified Fischer 1960", "South American 1969","WGS 60","WGS 66","WGS-72", "WGS-84"}
 
 
	variable a = EquatorialRadius[ReferenceEllipsoid]
	variable eccSquared = SquareOfEccentricity[ReferenceEllipsoid];
	variable k0 = 0.9996;
 
	variable LongOrigin;
	variable eccPrimeSquared;
	variable N, T, C, AA, M;
 
//Make sure the longitude is between -180.00 .. 179.9
	variable LongTemp = (Long+180)-floor((Long+180)/360)*360-180; // -180.00 .. 179.9;
 
	variable LatRad = Lat*DEG2RAD;
	variable LongRad = LongTemp*DEG2RAD;
	variable LongOriginRad;
	variable    ZoneNumber;
 
	ZoneNumber = floor((LongTemp + 180)/6) + 1;
 
	if( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
		ZoneNumber = 32;
	endif
  // Special zones for Svalbard
	if( Lat >= 72.0 && Lat < 84.0 ) 
	  if  ( LongTemp >= 0.0  && LongTemp <  9.0 ) 
	  	ZoneNumber = 31;
	  elseif( LongTemp >= 9.0  && LongTemp < 21.0 )
	  	ZoneNumber = 33;
	  elseif(LongTemp >= 21.0 && LongTemp < 33.0 )
	  	ZoneNumber = 35;
	  elseif(LongTemp >= 33.0 && LongTemp < 42.0 ) 
	  	ZoneNumber = 37;
	  endif
	 endif
	LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;  //+3 puts origin in middle of zone
	LongOriginRad = LongOrigin * DEG2RAD;
 
	//compute the UTM Zone from the latitude and longitude
 
	eccPrimeSquared = (eccSquared)/(1-eccSquared);
 
	N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
	T = tan(LatRad)*tan(LatRad);
	C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
	AA = cos(LatRad)*(LongRad-LongOriginRad);
 
	M = (1	- eccSquared/4		- 3*eccSquared*eccSquared/64	- 5*eccSquared*eccSquared*eccSquared/256)*LatRad
	M -=  (3*eccSquared/8	+ 3*eccSquared*eccSquared/32	+ 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
	M += (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
	M -=  (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad)
	M *= a
 
	UTMEasting = (k0*N*(AA+(1-T+C)*AA*AA*AA/6+ (5-18*T+T*T+72*C-58*eccPrimeSquared)*AA*AA*AA*AA*AA/120)+ 500000.0);
 
	UTMNorthing = (k0*(M+N*tan(LatRad)*(AA*AA/2+(5-T+9*C+4*C*C)*AA*AA*AA*AA/24+ (61-58*T+T*T+600*C-330*eccPrimeSquared)*AA*AA*AA*AA*AA*AA/720)));
	if(Lat < 0)
		UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere
	endif
 
	gps.UTMEasting = UTMEasting
	gps.UTMNorthing = UTMNorthing
	gps.UTMZone = num2str(ZoneNumber)+ UTMLetterDesignator(Lat)
	print gps
End
 
Function/t UTMLetterDesignator( Lat)
variable Lat
//This routine determines the correct UTM letter designator for the given latitude
//returns "Z" if latitude is outside the UTM limits of 84N to 80S
	//Written by Chuck Gantz- chuck.gantz@globalstar.com
	string LetterDesignator;
 
	if((84 >= Lat) && (Lat >= 72)) 
		LetterDesignator = "X";
	elseif((72 > Lat) && (Lat >= 64))
		 LetterDesignator = "W";
	elseif((64 > Lat) && (Lat >= 56)) 
		LetterDesignator = "V";
	elseif((56 > Lat) && (Lat >= 48)) 
		LetterDesignator = "U";
	elseif((48 > Lat) && (Lat >= 40))
	 LetterDesignator = "T";
	elseif((40 > Lat) && (Lat >= 32))
	 LetterDesignator = "S";
	elseif((32 > Lat) && (Lat >= 24))
	 LetterDesignator = "R";
	elseif((24 > Lat) && (Lat >= 16))
	 LetterDesignator = "Q";
	elseif((16 > Lat) && (Lat >= 8)) 
	LetterDesignator = "P";
	elseif(( 8 > Lat) && (Lat >= 0)) 
	LetterDesignator = "N";
	elseif(( 0 > Lat) && (Lat >= -8)) 
	LetterDesignator = "M";
	elseif((-8> Lat) && (Lat >= -16)) 
	LetterDesignator = "L";
	elseif((-16 > Lat) && (Lat >= -24))
	 LetterDesignator = "K";
	elseif((-24 > Lat) && (Lat >= -32))
	 LetterDesignator = "J";
	elseif((-32 > Lat) && (Lat >= -40)) 
	LetterDesignator = "H";
	elseif((-40 > Lat) && (Lat >= -48)) 
	LetterDesignator = "G";
	elseif((-48 > Lat) && (Lat >= -56))
	 LetterDesignator = "F";
	elseif((-56 > Lat) && (Lat >= -64)) 
	LetterDesignator = "E";
	elseif((-64 > Lat) && (Lat >= -72))
	 LetterDesignator = "D";
	elseif((-72 > Lat) && (Lat >= -80)) 
	LetterDesignator = "C";
	else
		 LetterDesignator = "Z"; //This is here as an error flag to show that the Latitude is outside the UTM limits
	endif
	return LetterDesignator;
End