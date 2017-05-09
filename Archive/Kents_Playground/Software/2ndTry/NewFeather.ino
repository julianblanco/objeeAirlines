// Includes


// Hardware defines
#define BAROMETER
#define XBEE
#define SDCARD

// User defined
#define IMU_PERIOD 50
#define BARO_PERIOD 50
#define SERVO_PERIOD 50
#define LOOP_PERIOD 5


// Pass serial object to GNSS
#define GNSS Serial1
GNSS gnss(&GNSS);



void setup()
{
	// Initialize serial communications
	Wire.begin();
	gnss.begin(9600);
	#ifdef XBEE
		xbee.begin(9600);
	#else
		Serial.begin(9600);
	#endif


	// Initialize IMU
	if(!bno.begin())
	{
		#ifdef XBEE
			xbee.print("ERROR: BNO055 IMU not found\n");
		#else
			Serial.print("ERROR: BNO055 IMU not found\n");
		#endif
		while( 1 ){};
	}
	bno.setExtCrystalUse(true);
  	SampleGyro(bno);


	// Initialize Barometer
	#ifdef BAROMETER
		if (!bmp.begin())
		{
			#ifdef XBEE
				xbee.print("ERROR: BMP085 barometer not found\n");
			#else
				Serial.print("ERROR: BMP085 barometer not found\n");
			#endif
			while( 1 ){};
		}
		// MOVE THESE TO BAROMETER FILE
		intialpressure=bmp.readPressure();
		intialAltitude=bmp.readAltitude();
	#endif


	// Initialize GPS
	


	// Create and initialize servo objects
	// Center servos


	// Wait for IMU calibration
	while(calibration != 3)
	{
		delay(20);
		SampleGyro(bno);
	}


	// Initialize with magnetic heading?
	


	// Wait for Fix (unless in test mode)
	while(!gnss.fix)
	{
		gnss.read();
		Serial_Sample(GNSS);
	}


	// Initialize and load first waypoint



	// Open SD logger
	#ifdef SDCARD
		#ifdef XBEE
			xbee.print("Initializing SD card...\n");
		#else
			Serial.print("Initializing SD card...\n");
		#endif
		SDinit();
	#endif



	// servoBegin(aileron,elevator,rudder);

	// waypointing(); // curious, cause of double waypoint startup?
	// //xbee.println("objeeAirlines Autonomous Veichile Log File");
	// CenterAll(aileron,elevator,rudder);


	// #ifdef fullScale
	// CenterAll(aileron,elevator,rudder);
	// ServoWiggle(aileron,elevator,rudder);

	// #endif


	// CenterAll(aileron,elevator,rudder);
	// ServoWiggle(aileron,elevator,rudder);// Initialize servos
	// CenterAll(aileron,elevator,rudder);

	// ServoInit(aileron,elevator,rudder);  // Initialize servos
	// imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
	// yawOffset= euler.x(); //yax
	// // yawOffset=SampleGyroX();
}

void loop()
{
	// Record loop start time

	// Increment IMU count
	// Check for (IMU count * LOOP_PERIOD) > IMU period
	// Read new IMU data
	// Update orientation estimation

	// Increment Baro count
	// Check for (Baro count * LOOP_PERIOD) > Baro period
	// Read new Baro data
	// Update position estimation

	// Check char buffer
	// Check for full string
	// Read new GPS data
	// Update heading and position estimation
	// Set new desired heading

	// Check Pitot count
	// Check for (Pitot count * LOOP_PERIOD) > Pitot period
	// Read new Pitot data
	// Update position airspeed estimation
	// Dead recon position estimation

	// Increment Servo count
	// Check for (Servo count * LOOP_PERIOD) > Servo period
	// Calculate xyz control responses
	// Map control responses to control surfaces
	// Send control responses

	// Loop until LOOP_PERIOD reached
	// Check char buffer for GPS
}