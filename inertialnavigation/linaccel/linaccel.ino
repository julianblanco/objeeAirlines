#include "MatrixMath.h"
 
float trapzd(float value1, float value2);

void setup() {

Serial.begin(9600);
delay(500);
//zyx
//
float eulerangle [10][3] = {
{301.625000,-14.000000,-9.250000},
{301.875000,-14.062500,-8.625000},
{302.312500,-14.062500,-8.375000},
{303.000000,-14.000000,-8.312500},
{304.125000,-13.875000,-8.187500},
{303.250000,-14.500000,-8.437500},
{302.750000,-14.812500,-8.687500},
{302.875000,-15.250000,-8.812500},
{302.937500,-15.687500,-8.625000},
{302.875000,-15.937500,-8.187500},
};


float linearaccel[10][3] = {
{4.370000,-1.200000,0.090000},
{4.390000,-0.440000,1.350000},
{3.620000,-0.470000,0.960000},
{0.900000,-0.560000,0.590000},
{1.170000,-0.930000,0.740000},
{5.470000,-0.320000,1.010000},
{5.860000,-0.250000,0.820000},
{3.750000,-0.350000,-0.200000},
{3.990000,-0.370000,0.550000},
{4.740000,-0.130000,1.120000}
};

float projectedvelocity[10][3];

float DCM[3][3];


for(int idxa = 0;idxa<10; idxa++)
{
   for(int idxb =0; idxb<3;idxb++)
   {
    eulerangle[idxa][idxb] = degtorad(eulerangle[idxa][idxb]);
   }
}
Matrix.Print((float*)eulerangle, 10, 3, "DCM");


for(int idxa = 0;idxa<10; idxa++)
{
    projectedvelocity[idxa][0] = trapzd(linearaccel[idxa][0],linearaccel[idxa+1][0]);
    projectedvelocity[idxa][1] = trapzd(linearaccel[idxa][1],linearaccel[idxa+1][0]);
    projectedvelocity[idxa][2] = trapzd(linearaccel[idxa][2],linearaccel[idxa+1][0]);

}
Matrix.Print((float*)projectedvelocity, 10, 3, "DCM");

//float DCM[3][3];
// eulerAnglesToRotationMatrix(rotmat,eulerangle);
// Matrix.Print((float*)rotmat, 3, 3, "Rot Mat");
float R[3];
for(int idxa = 0;idxa<10; idxa++)
{
  delay(50);
  eulerAnglesToDCM(DCM,eulerangle[idxa][0],eulerangle[idxa][1],eulerangle[idxa][2]);
   Matrix.Print((float*)DCM, 3, 3, "DCM");
  float temp[3]= {projectedvelocity[idxa][0],projectedvelocity[idxa][1],projectedvelocity[idxa][2]};

  Matrix.Multiply((float*)DCM, (float*)temp, 3, 3, 1, (float*)R);
  Matrix.Print((float*)R, 1, 3, "Matrice");
  Serial.println(idxa);
  // delay(50);
}

}

void loop()
{

}

float trapzd(float value1, float value2)
// This routine computes the nth stage of refinement of an extended trapezoidal rule. func is input
// as a pointer to the function to be integrated between limits a and b, also input. When called with
// n=1, the routine returns the crudest estimate of  b
// a f(x)dx. Subsequent calls with n=2,3,...
// (in that sequential order) will improve the accuracy by adding 2n-2 additional interior points.
{
	
float s=0.5*(value1+value2); //This replaces s by its refined value.
return s;
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

// // Checks if a matrix is a valid rotation matrix.
// bool isRotationMatrix(Mat &R)
// {
//     Mat Rt;
//     transpose(R, Rt);
//     Mat shouldBeIdentity = Rt * R;
//     Mat I = Mat::eye(3,3, shouldBeIdentity.type());
     
//     return  norm(I, shouldBeIdentity) < 1e-6;
     
// }
 
// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
// void rotationMatrixToEulerAngles(float R[3][3],float& eulervector[3])
// {
 
//     assert(isRotationMatrix(R));
     
//     float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
//     bool singular = sy < 1e-6; // If
 
//     float x, y, z;
//     if (!singular)
//     {
//         x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
//         y = atan2(-R.at<double>(2,0), sy);
//         z = atan2(R.at<double>(1,0), R.at<double>(0,0));
//     }
//     else
//     {
//         x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
//         y = atan2(-R.at<double>(2,0), sy);
//         z = 0;
//     }

//     eulervector[0] = x;
//     eulervector[1] = y;
//     eulervector[2] = z;
     
     
     
// }

