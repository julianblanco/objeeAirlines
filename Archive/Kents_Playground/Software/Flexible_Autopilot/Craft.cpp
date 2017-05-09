#include "Craft.h"

Craft::Craft() : m_bar(), m_air(m_bar)
{ }

Craft::~Craft(){ }



void set_control_surface(int index, Control_Surface* servo)
{
	control_surface[index] = servo;
}



void Craft::update()
{
	m_gps.update();
	m_imu.update();
	m_air.update();

	m_altitude = m_gps[ALT]*0.5 + (float)(m_bar)*0.5;
}