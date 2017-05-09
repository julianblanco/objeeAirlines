/*
* @Author: john & julian
* @Date:   2017-03-06 01:24:52
* @Last Modified by:   julian
* @Last Modified time: 2017-03-06 02:53:38
*/

#include <iostream>
#include <math.h>


using namespace std;


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
// void diff(float A[][1], float B[][1]){// implementing DIFF
// 	int m = sizeof(A) / sizeof(float);
// 	for ( int i = 0; i < m; i++ ){

// 		// cout << linearaccel[0][i] << endl;
// 		if ( i != m - 1 ){
// 			B[i][0] = -1*(A[0][i] - A[0][i+1])/2;
// 			// cout << dt[i][1] << endl;
// 		}
// 	}

// }

void cumtrapz (float* time,float* inmatrix,float* outmatrix){

//Implementing cumptrapz***********************************************
	int i, j;
	int m =10;int n=3;
	float tempmat[10][3];

	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			tempmat[n * i + j] = time[n * i]*(inmatrix[n * i + j]+inmatrix[n * i + j]);
		}
		float cumsum = 0;
		float cumsum1 = 0;
		float cumsum2 = 0;
		for ( int i = 0; i < 10; i++ ){

			
			cumsum += tempmat[i][0];
			cumsum1 += tempmat[i][1];
			cumsum2 += tempmat[i][2];		

			outmatrix[i][0]=cumsum;
			outmatrix[i][1]=cumsum1;
			outmatrix[i][2]=cumsum2;

		}


}

int main(){


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

	float time[10][1] = {
	{7.958},
	{7.979},
	{7.999},
	{8.019},
	{8.044},
	{8.079},
	{8.097},
	{8.122},
	{8.141},
	{8.160},
	};

	float velocitydata[10][3];
	float cumtrapz[10][3];
	float dt[10][1];
	float cumsum = 0;
	float cumsum1 = 0;
	float cumsum2 = 0;
	float projectedvelocity[10][3];
	float DCM[3][3];
	float R[3][1];
	float temp[3][1];
	float posdata[10][3];
	float totaldisplacement 


	// implementing DIFF
	int m = sizeof(time) / sizeof(float);
	for ( int i = 0; i < m ; i++ ){

		// cout << linearaccel[0][i] << endl;
		if ( i != m - 1 ){
			dt[i][1] = -1*(time[0][i] - time[0][i+1])/2;
			// cout << dt[i][1] << endl;
		}
	}
	
	
	//Implementing cumptrapz***********************************************
	cumtrapz((float*)time,(float*)linearaccel,(float*)velocitydata);
	//***********************************************

//implemeting the projection DCM * VELOCITY



for(int idxa = 0;idxa<10; idxa++)
{
   for(int idxb =0; idxb<3;idxb++)
   {
    eulerangle[idxa][idxb] = degtorad(eulerangle[idxa][idxb]);
   }
}

// cout << "" <<endl;

for(int idxa = 0;idxa<10; idxa++)
{
eulerAnglesToDCM(DCM,eulerangle[idxa][0],eulerangle[idxa][1],eulerangle[idxa][2]);

temp[0][0]=velocitydata[idxa][0];
temp[1][0]=velocitydata[idxa][1];
temp[2][0]=velocitydata[idxa][2];

// cout << DCM[0][0]<<" "<<DCM[1][0]<<" "<<DCM[2][0]<< endl;
// cout << temp[0][0]<<" "<<temp[1][0]<<" "<<temp[2][0]<< endl;
// cout <<""<<endl;
multiply((float*)DCM, (float*)temp, 3, 3, 1, (float*)R);
// cout << R[0][0]<<" "<<R[1][0]<<" "<<R[2][0]<< endl;
// cout <<R[2][0]<< endl;
projectedvelocity[idxa][0] = R[0][0];
projectedvelocity[idxa][1] = R[1][0];
projectedvelocity[idxa][2] = R[2][0];
// cout<<endl;
// cout <<""<<endl;
}
//***********************************************************

//Implementing cumptrapz***********************************************
	for ( int i = 0; i < m-1 ; i++ ){

		// cout << linearaccel[0][i] << endl;
		
			cumtrapz[i+1][0] = dt[0][i+1]*(projectedvelocity[i][0]+projectedvelocity[i+1][0]);
			cumtrapz[i+1][1] = dt[0][i+1]*(projectedvelocity[i][1]+projectedvelocity[i+1][1]);
			cumtrapz[i+1][2] = dt[0][i+1]*(projectedvelocity[i][2]+projectedvelocity[i+1][2]);
			cout << cumtrapz[i+1][2]<< endl;
		
	}
	// cout << "" << endl;

	cumsum = 0;
	cumsum1 = 0;
	cumsum2 = 0;
	cout<<""<<endl;

	for ( int i = 0; i < m; i++ ){

		

		cumsum += cumtrapz[i][0];
		cumsum1 += cumtrapz[i][1];
		cumsum2 += cumtrapz[i][2];		

		posdata[i][0]=cumsum;
		posdata[i][1]=cumsum1;
		posdata[i][2]=cumsum2;

		cout << cumsum2<< endl;

	}	
	

	totaldisplacement=3.28084 * sqrt( ((posdata[9][0]*posdata[9][0])+(posdata[9][1]*posdata[9][1])));
//***********************************************
cout<<endl;

cout <<"Final Displacement: "<<totaldisplacement<< " Feet"<<endl;

  


























    return 0;
}



