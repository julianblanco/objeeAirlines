/*	Author: Caleb Stewart
	Date: 14Jan16
	Notes: You're welcome
*/

#ifndef SENSOR_H
#define SENSOR_H


// Framework for standardized sensor data interface
class Sensor
{
public:
	Sensor(const char* type);
	virtual ~Sensor();

	// Redefine float operator
	virtual operator float() {
		return (*this)[0];
	}

	// Redefine square bracket operator
	virtual float operator[](int idx) = 0;

	virtual bool update() = 0;

	const char* getType() const { return type; }

private:
	char type[128];
};


#endif  //SENSOR_H