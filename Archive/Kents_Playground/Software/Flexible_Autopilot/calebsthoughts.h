/* author: Caleb Stewart
	notes: you're welcome.
*/
class Sensor
{
public:
	Sensor(const char* type);
	virtual ~Sensor();


	virtual operator float() {
		return (*this)[0];
	}
	virtual float operator[](int idx) = 0;
	virtual bool update() = 0;

	const char* getType() const { return type; }

private:
	char type[128];
};

#define GPS_LAT 0
#define GPS_LON 1

class GPS : public Sensor
{
public:
	GPS();
	~GPS();

	virtual float operator[](int idx)
	{
		return m_values[idx];
	}

	virtual bool update()
	{
		return true;
	}

private:
	float data[2];
};

class AirSpeed : public Sensor
{
public:
	AirSpeed(Sensor& bar);
	~AirSpeed();

	virtual float operator[](int idx)
	{
		if( idx != 0 ) throw std::exception();
		return m_speed;
	}

private:
	float m_speed;
};

class Craft
{
public:
	Craft(float x);

	void update();

private:
	GPS m_gps;
	IMU m_imu;
	AirSpeed m_air;
	Barometer m_bar;
	float m_altitude;
};

Craft::Craft() : m_bar(), m_air(m_bar)
{ }

void Craft::update()
{
	m_gps.update();
	m_imu.update();
	m_air.update();

	m_altitude = m_gps[ALT]*0.5 + (float)(m_bar)*0.5;
}


int main()
{
	Craft craft(0.5);
	Craft.m_gps

}