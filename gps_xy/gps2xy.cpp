float degtorad(float deg)
{
  float rad = deg*(3.14159265359/180);
  return rad;
}

float latgenerator( float currentlat, float currentlong,float distance,float course )
{
float newlat=0;
float degree = distance/111073;
course=degtorad(course);
float lat=degree*cos(course);
newlat=currentlat+lat;
return newlat;


}

float longgenerator( float currentlat, float currentlong,float distance,float course )
{
float longi=0;

float degree = distance/111073;
course=degtorad(course);
longi=degree*sin(course);
longi=currentlong+longi;
return longi;

}
