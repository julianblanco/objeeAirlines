/*
* @Author: john & julian
* @Date:   2017-03-06 01:24:52
* @Last Modified by:   julian
* @Last Modified time: 2017-03-06 02:53:38
*/
#include <math.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include <SD.h>

float totaldisplacement;
float x;
float y;
float z;
float eulerx;
float eulery;
float eulerz;

float linearacceldata[3][2];
float eulerdata[3][1];
float velocitydata[3][1];
float dt[3][1];
float projectedvelocity[3][1];
float DCM[3][3];
float posdata[3][1];

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup(void)
{
   if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
      while(1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  SDinit();
  // uint8_t system, gyro, accel, mag = 0;
  //  while(system != 3)
  // {
  //    delay(500);
  //    bno.getCalibration(&system, &gyro, &accel, &mag);

  // }
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
intertialdoubleintegral();
}

// C = A*B
void multiply(float* A, float* B, int m, int p, int n, float* C)
{
// A = input matrix (m x p)
// B = input matrix (p x n)
// m = number of rows in A
// p = number of columns in A = number of rows in B
// n = number of columns in B
// C = output matrix = A*B (m x n)
int i, j, k;
for (i = 0; i < m; i++)
	for(j = 0; j < n; j++)
	{
		C[n * i + j] = 0;
		for (k = 0; k < p; k++)
			C[n * i + j] = C[n * i + j] + A[p * i + k] * B[n * k + j];
	}
}
void eulerAnglesToDCM(float R[3][3],float theta1,float theta2 ,float theta3)
{
// Calculate rotation about x axis
float theta[3]= {theta1,theta2,theta3};

R[0][0]  =  cos(theta[1])*cos(theta[0]);
R[0][1]  =  cos(theta[1])*sin(theta[0]);  
R[0][2]  =  -sin(theta[1]);

R[1][0]  =  (sin(theta[2])*sin(theta[1])*cos(theta[0])) - (sin(theta[0])*cos(theta[2]));  
R[1][1]  =  (sin(theta[2])*sin(theta[1])*sin(theta[0])) + (cos(theta[0])*cos(theta[2]));  
R[1][2]  =  (sin(theta[2])*cos(theta[1]));

R[2][0]  =  (cos(theta[2])*sin(theta[1])*cos(theta[0])) + (sin(theta[2])*sin(theta[0]));  
R[2][1]  =  (cos(theta[2])*sin(theta[1])*sin(theta[0])) - (sin(theta[2])*cos(theta[0]));  
R[2][2]  =  (cos(theta[2])*cos(theta[1]));   

// Matrix.Print((float*)R_x, 3, 3, "Rot Mat");

}

float degtorad(float deg)
{
float rad = deg*(3.14159265359/180);
return rad;
}


float intertialdoubleintegral(){


// Sample data
imu::Vector<3> linacceel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
linearacceldata[0][0] =linacceel.x();
linearacceldata[1][0]  =linacceel.y();
linearacceldata[2][0]  =linacceel.z();

imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
eulerdata[0][0] =degtorad(euler.x());
eulerdata[1][0] =degtorad(euler.y());
eulerdata[2][0] =degtorad(euler.z());


//Take derv of time 
dt[1][0] = millis();
dt[2][0] = (dt[1][0] - dt[0][0])/2;
dt[0][0] = dt[1][0];

//Implementing cumptrapz***********************************************
velocitydata[0][0] = dt[2][0]*(linearacceldata[0][0]+linearacceldata[0][1]);
velocitydata[1][0] = dt[2][0]*(linearacceldata[1][0]+linearacceldata[1][1]);
velocitydata[2][0] = dt[2][0]*(linearacceldata[2][0]+linearacceldata[2][1]);
linearacceldata[0][1]=linearacceldata[0][0];
linearacceldata[1][1]=linearacceldata[1][0];
linearacceldata[2][1]=linearacceldata[2][0];


//implemeting the projection DCM * VELOCITY
// need to shove some varbiles in here
eulerAnglesToDCM(DCM,eulerdata[0][0],eulerdata[1][0],eulerdata[2][0]);
multiply((float*)DCM, (float*)velocitydata, 3, 3, 1, (float*)projectedvelocity);

//Second integration
posdata[0][0] = dt[2][0]*(projectedvelocity[0][0]+projectedvelocity[0][1]);
posdata[1][0] = dt[2][0]*(projectedvelocity[1][0]+projectedvelocity[1][1]);
posdata[2][0] = dt[2][0]*(projectedvelocity[2][0]+projectedvelocity[2][1]);

//total displacement
totaldisplacement =3.28084 * sqrt( ((posdata[0][0]*posdata[0][0])+(posdata[1][0]*posdata[1][0])));


sdlog();
}







//Why is this here?
File myFile;
File myFile1;
File myFile2;
char nameoffile[16];
char nameoffile1[16];
char nameoffile2[16];
//
void sdlog()
{
     myFile = SD.open(nameoffile, FILE_WRITE);
     if (myFile) {
        myFile.print(x,6);myFile.print(',');
        myFile.print(y,6);myFile.print(',');
        myFile.print(z,6);myFile.print(',');
        myFile.print(eulerx,6);myFile.print(',');
        myFile.print(eulery,6);myFile.print(',');
        myFile.print(eulerz,6);myFile.print(',');
        myFile.print(totaldisplacement,6);myFile.print(',');
       	myFile.println(dt[1][0]);
      }
       myFile.close();
}


void SDinit()
{
  if (!SD.begin(4)) {
   // xbee.println("initialization failed!");
    return;
  }
  //xbee.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
   // make it long enough to hold your longest file name, plus a null terminator
  int n = 0;
  snprintf(nameoffile, sizeof(nameoffile), "data%03d.txt", n); // includes a three-digit sequence number in the file name
  while(SD.exists(nameoffile)) {
    n++;
    snprintf(nameoffile, sizeof(nameoffile), "data%03d.txt", n);
  }
  // Serial.println(n);
  // Serial.println(nameoffile);
  //now nameoffile[] contains the name of a file that doesn't exist
   myFile= SD.open(nameoffile,FILE_READ);
   myFile.close();
   myFile= SD.open(nameoffile,FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    //Serial2.print("objeeAirlines Autonomous Veichile Test");
    
    // close the file:
    myFile.close();
    //Serial2.println("done.");
  } else {
    // if the file didn't open, print an error:
    //Serial2.println("error opening test.txt");
  }
}


