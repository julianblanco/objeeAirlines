#include "BasicLinearAlgebra.h" // downloaded library

void setup()
{
	Serial.begin(115200);
	
	Matrix<2,1> xp;
	Matrix<2,2> Pp;

		// A, H, Q, R, x, P
	Matrix<2,2,float> A;
	A(0,0) = 1, A(1,0) = 0;
	A(1,0) = 0, A(1,1) = 1;

	Matrix<1,2,float> H;
	H(0,0) = 1, H(0,1) = 0;

	Matrix<2,2,float> Q;
	Q(0,0) = .25, Q(0,1) = 0;
	Q(1,0) = 0  , Q(1,1) = .25;

	Matrix<1,1,float> R;
	R(0,0) = 1;

	Matrix<2,1,float> x;
	x(0,0) = 40;
	x(1,0) = 0;

	Matrix<2,2,float> P;
	P(0,0) = 1, P(0,1) = 0;
	P(1,0) = 0, P(1,1) = 1;

	Serial << "A: " << A << "\n";

	Serial << "H: " << H << "\n";

	Serial << "x: " << x << "\n";

	Serial << "P: " << P << "\n";
}

void loop()
{
	// xp = A*x;  // won't pull values from setup()

/* BROKE BROKE BROKE BROKE BROKE :( */



}