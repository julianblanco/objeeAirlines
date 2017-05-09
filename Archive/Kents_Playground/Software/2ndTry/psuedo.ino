// Includes

// User defined
#define IMU_PERIOD 50
#define BARO_PERIOD 50
#define SERVO_PERIOD 50
#define LOOP_PERIOD 5




void setup()
{
	// Initialize IMU

	// Initialize Barometer

	// Initialize GPS

	// Create and initialize servo objects

	// Wait for Fix (unless in test mode)

	// Open SD logger
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