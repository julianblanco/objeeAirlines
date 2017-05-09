#include "MatrixMath.h" // downloaded library

int gps_flag=0;
double dummy;
double utmeast=0;
double utmnorth=0;
String zonez ="";

double myLat;
double myLong;
double calclat;
double calclong;

// defines matrix dimensions
#define one (1)
#define two (2)
#define three (3)
#define four (4)

// global variables & matrix sizes
double A[four][four];
double H[four][four];

double test[four][four];
double Q[four][four];
double R[four][four];
double x[four][one];
double P[four][four];
double I[four][four];
double Y[four][one];
double PM[four][four];
double PM2[four][one];

// working matricies / outputs
double input[four][one];  
double xp[four][one];  
double Pp[four][four];  
double K[four][four];  
double inversion[four][four];  
double transposeA[four][four];  
double transposeH[four][four];
double temp1[four][four];  
double temp2[four][four];  
double temp3[four][one];  
double temp4[four][one];
double temp5[four][four];
double temp6[four][four];

double xpos = 700000; // UTM Easting
double xvelocity = 0;
double ypos = 4000000; // UTM Northing
double yvelocity = 0;

double velocity=0;
double trueHeading=0;

double xpos_corrected;
double ypos_corrected;
double xvel_corrected;
double yvel_corrected;


double time_diff;
double time_diff_prev;
double current_time;

double radi=0;
#define delayTime 0.0667

String inputString = "";
boolean stringComplete = false;  // whether the string is complete
int secondflag=0;
// the setup function runs once when you press reset or power the board
void setup()
{

  Serial.begin(115200);
  Serial2.begin(115200);
   Serial3.begin(9600);
  delay(500);
  Serial.println("Kalman Filter Initializing...");
  Serial2.println("Kalman Filter Initializing...");

current_time = (millis())/1000;
time_diff_prev = current_time;

A[0][0] = 1, A[0][1] = current_time, A[0][2] = 0, A[0][3] = 0; // x position
A[1][0] = 0, A[1][1] = 1, A[1][2] = 0, A[1][3] = 0;           // x velocity
A[2][0] = 0, A[2][1] = 0, A[2][2] = 1, A[2][3] = current_time ; // y position
A[3][0] = 0, A[3][1] = 0, A[3][2] = 0, A[3][3] = 1;           // y velocity

H[0][0] = 1.0, H[0][1] = 0.0, H[0][2] = 0.0, H[0][3] = 0.0; 
H[1][0] = 0.0, H[1][1] = 1.0, H[1][2] = 0.0, H[1][3] = 0.0;
H[2][0] = 0.0, H[2][1] = 0.0, H[2][2] = 1.0, H[2][3] = 0.0;
H[3][0] = 0.0, H[3][1] = 0.0, H[3][2] = 0.0, H[3][3] = 1.0;

test[0][0] = 2.0, test[0][1] = 0.0, test[0][2] = 0.0, test[0][3] = 0.0; 
test[1][0] = 1.0, test[1][1] = 2.0, test[1][2] = 0.0, test[1][3] = 0.0;
test[2][0] = 0.0, test[2][1] = 0.0, test[2][2] = 1.0, test[2][3] = 0.0;
test[3][0] = 0.0, test[3][1] = 0.0, test[3][2] = 0.0, test[3][3] = 1.0;

Q[0][0] = .25, Q[0][1] = 0, Q[0][2] = 0, Q[0][3] = 0; // process noise
Q[1][0] = 0, Q[1][1] = .25, Q[1][2] = 0, Q[1][3] = 0;
Q[2][0] = 0, Q[2][1] = 0, Q[2][2] = .25, Q[2][3] = 0;
Q[3][0] = 0, Q[3][1] = 0, Q[3][2] = 0, Q[3][3] = .25;

R[0][0] = 2, R[0][1] = 0, R[0][2] = 0, R[0][3] = 0; // measurement noise
R[1][0] = 0, R[1][1] = 2, R[1][2] = 0, R[1][3] = 0;
R[2][0] = 0, R[2][1] = 0, R[2][2] = 2, R[2][3] = 0;
R[3][0] = 0, R[3][1] = 0, R[3][2] = 0, R[3][3] = 2;

x[0][0] = 700000; // x position
x[1][0] = 1; // x velocity
x[2][0] = 4000000; // y position
x[3][0] = 1; // y velocity

P[0][0] = 2, P[0][1] = 0, P[0][2] = 0, P[0][3] = 0; // error covariance
P[1][0] = 0, P[1][1] = 2, P[1][2] = 0, P[1][3] = 0;
P[2][0] = 0, P[2][1] = 0, P[2][2] = 2, P[2][3] = 0;
P[3][0] = 0, P[3][1] = 0, P[3][2] = 0, P[3][3] = 2;

I[0][0] = 1, I[0][1] = 0, I[0][2] = 0, I[0][3] = 0; // identity matrix
I[1][0] = 0, I[1][1] = 1, I[1][2] = 0, I[1][3] = 0;
I[2][0] = 0, I[2][1] = 0, I[2][2] = 1, I[2][3] = 0;
I[3][0] = 0, I[3][1] = 0, I[3][2] = 0, I[3][3] = 1;

Y[0][0] = 0; // x position measured
Y[1][0] = 0; // x velocity measured
Y[2][0] = 0; // y position measured
Y[3][0] = 0; // y velocity measured

input[0][0] = 0;
input[1][0] = 0;
input[2][0] = 0;
input[3][0] = 0;
 
xp[0][0] = 0; // xpdentxpty matrxpx
xp[1][0] = 0;
xp[2][0] = 0;
xp[3][0] = 0;

Pp[0][0] = 0; Pp[0][1] = 0, Pp[0][2] = 0, Pp[0][3] = 0; // PpdentPpty matrPpx
Pp[1][0] = 0, Pp[1][1] = 0, Pp[1][2] = 0, Pp[1][3] = 0;
Pp[2][0] = 0, Pp[2][1] = 0, Pp[2][2] = 0, Pp[2][3] = 0;
Pp[3][0] = 0, Pp[3][1] = 0, Pp[3][2] = 0, Pp[3][3] = 0;

K[0][0] = 0; // KdentKty matrKx
K[1][0] = 0;
K[2][0] = 0;
K[3][0] = 0;

inversion[0][0] = 0, inversion[0][1] = 0, inversion[0][2] = 0, inversion[0][3] = 0; // inversiondentinversionty matrinversionx
inversion[1][0] = 0, inversion[1][1] = 0, inversion[1][2] = 0, inversion[1][3] = 0;
inversion[2][0] = 0, inversion[2][1] = 0, inversion[2][2] = 0, inversion[2][3] = 0;
inversion[3][0] = 0, inversion[3][1] = 0, inversion[3][2] = 0, inversion[3][3] = 0;


transposeA[0][0] = 1, transposeA[0][1] = 0, transposeA[0][2] = 0, transposeA[0][3] = 0; // transposeAdenttransposeAty matrtransposeAx
transposeA[1][0] = current_time, transposeA[1][1] = 1, transposeA[1][2] = 0, transposeA[1][3] = 0;
transposeA[2][0] = 0, transposeA[2][1] = 0, transposeA[2][2] = 1, transposeA[2][3] = current_time;
transposeA[3][0] = 0, transposeA[3][1] = 0, transposeA[3][2] = 0, transposeA[3][3] = 1;

transposeH[0][0] = 1, transposeH[0][1] = 0, transposeH[0][2] = 0, transposeH[0][3] = 0; // transposeHdenttransposeHty matrtransposeHx
transposeH[1][0] = 0, transposeH[1][1] = 1; transposeH[1][2] = 0, transposeH[1][3] = 0;
transposeH[2][0] = 0, transposeH[2][1] = 0, transposeH[2][2] = 1, transposeH[2][3] = 0;
transposeH[3][0] = 0, transposeH[3][1] = 0, transposeH[3][2] = 0, transposeH[3][3] = 1;

temp1[0][0] = 0, temp1[0][1] = 0, temp1[0][2] = 0, temp1[0][3] = 0; // temp1denttemp1ty matrtemp1x
temp1[1][0] = 0, temp1[1][1] = 0, temp1[1][2] = 0, temp1[1][3] = 0;
temp1[2][0] = 0, temp1[2][1] = 0, temp1[2][2] = 0, temp1[2][3] = 0;
temp1[3][0] = 0, temp1[3][1] = 0, temp1[3][2] = 0, temp1[3][3] = 0;
 
temp2[0][0] = 0, temp2[0][1] = 0, temp2[0][2] = 0, temp2[0][3] = 0; // temp2denttemp2ty matrtemp2x
temp2[1][0] = 0, temp2[1][1] = 0, temp2[1][2] = 0, temp2[1][3] = 0;
temp2[2][0] = 0, temp2[2][1] = 0, temp2[2][2] = 0, temp2[2][3] = 0;
temp2[3][0] = 0, temp2[3][1] = 0, temp2[3][2] = 0, temp2[3][3] = 0;

temp3[0][0] = 0; // temp3denttemp3ty matrtemp3x
temp3[1][0] = 0;
temp3[2][0] = 0;
temp3[3][0] = 0;

temp4[0][0] = 0; // temp4denttemp4ty matrtemp4x
temp4[1][0] = 0;
temp4[2][0] = 0;
temp4[3][0] = 0;

temp5[0][0] = 0, temp5[0][1] = 0, temp5[0][2] = 0, temp5[0][3] = 0; // temp5denttemp5ty matrtemp5x
temp5[1][0] = 0, temp5[1][1] = 0, temp5[1][2] = 0, temp5[1][3] = 0;
temp5[2][0] = 0, temp5[2][1] = 0, temp5[2][2] = 0, temp5[2][3] = 0;
temp5[3][0] = 0, temp5[3][1] = 0, temp5[3][2] = 0, temp5[3][3] = 0;

temp6[0][0] = 0.2352, temp6[0][1] = -0.0031, temp6[0][2] = 0, temp6[0][3] = 0; // temp6denttemp6ty matrtemp6x
temp6[1][0] = -0.0031, temp6[1][1] = 0.2353, temp6[1][2] = 0, temp6[1][3] = 0;
temp6[2][0] = 0, temp6[2][1] = 0, temp6[2][2] = 0.2352, temp6[2][3] = -0.0031;
temp6[3][0] = 0, temp6[3][1] = 0, temp6[3][2] = -0.0031, temp6[3][3] = 0.2353;


for(int idx =0;idx<1000;idx++)
{
recvWithStartEndMarkers();
delay(1);
}

}
  int flag=1;

// the loop function runs over and over again forever
void loop()
{
 
  // bring in time and input_data
  current_time=(millis())/1000;


  latLongToUtm(myLat,myLong, xpos, ypos, zonez);
  kalman_filter();
  utmToLatLong(xpos_corrected,ypos_corrected, "18T" ,calclat,calclong);

  recvWithStartEndMarkers();
  recvWithStartEndMarkers();
  recvWithStartEndMarkers();
  recvWithStartEndMarkers();
  recvWithStartEndMarkers();
  recvWithStartEndMarkers();
  recvWithStartEndMarkers();
  recvWithStartEndMarkers();
  recvWithStartEndMarkers();

  if((millis()/1000)<(current_time+delayTime)) {
  Serial.print(myLat,8);Serial.print(",");
  Serial.print(myLong,8);Serial.print(",");
  Serial.print(calclat,8);Serial.print(",");
  Serial.print(calclong,8);Serial.print(",");
  Serial.print(xpos,2);Serial.print(",");
  Serial.print(ypos,2);Serial.print(",");
  Serial.print(xpos_corrected,2);Serial.print(",");
  Serial.print(ypos_corrected,2);Serial.print(",");
  Serial.print(time_diff,6);Serial.print(",");

  Serial.print(PM[0][0],2);Serial.print(",");
  Serial.print(PM[0][1],2);Serial.print(",");
  Serial.print(PM[0][2],2);Serial.print(",");
  Serial.print(PM[0][3],2);Serial.print(",");

  Serial.print(PM[1][0],2);Serial.print(",");
  Serial.print(PM[1][1],2);Serial.print(",");
  Serial.print(PM[1][2],2);Serial.print(",");
  Serial.print(PM[1][3],2);Serial.print(",");

  Serial.print(PM[2][0],2);Serial.print(",");
  Serial.print(PM[2][1],2);Serial.print(",");
  Serial.print(PM[2][2],2);Serial.print(",");
  Serial.print(PM[2][3],2);Serial.print(",");

  Serial.print(PM[3][0],2);Serial.print(",");
  Serial.print(PM[3][1],2);Serial.print(",");
  Serial.print(PM[3][2],2);Serial.print(",");
  Serial.print(PM[3][3],2);Serial.print(",");

  Serial.print(PM2[0][0],2);Serial.print(",");
  Serial.print(PM2[1][0],2);Serial.print(",");
  Serial.print(PM2[2][0],2);Serial.print(",");
  Serial.print(PM2[3][0],2);Serial.print(",");



  Serial.println(secondflag);
  notify_swarm();
  }//
  recvWithStartEndMarkers();
  recvWithStartEndMarkers();
  recvWithStartEndMarkers();
  recvWithStartEndMarkers();
  recvWithStartEndMarkers();
  recvWithStartEndMarkers();
  recvWithStartEndMarkers();
  recvWithStartEndMarkers();

  delay(10);

}

void kalman_filter()
{

  input[0][0] = xpos;
  input[1][0] = xvelocity;
  input[2][0] = ypos;
  input[3][0] = yvelocity;

  time_diff = (current_time - time_diff_prev);
  time_diff_prev = current_time;
  // time_diff = 0.028;
  A[0][1] = time_diff;
  A[2][3] = time_diff;
  transposeA[1][0] = time_diff;
  transposeA[3][2] = time_diff;


  // // kalman filter
  if(gps_flag == 0)
  {
  Matrix.Multiply((double*)A, (double*)x, four, four, one, (double*)xp); // xp output (4x1)

  Matrix.Multiply((double*)A, (double*)P, four, four, four, (double*)temp1);
  Matrix.Multiply((double*)temp1, (double*)transposeA, four, four, four, (double*)temp2);
  Matrix.Add((double*)temp2, (double*)Q, four, four, (double*)Pp); // Pp output (4x4)

  Matrix.Copy((double*) Pp, four, one, (double*) P);
  Matrix.Copy((double*) xp, four, one, (double*) x);

  }
  if(gps_flag==1){
    Matrix.Multiply((double*)A, (double*)x, four, four, one, (double*)xp); // xp output (4x1)

    Matrix.Multiply((double*)A, (double*)P, four, four, four, (double*)temp1);
    Matrix.Multiply((double*)temp1, (double*)transposeA, four, four, four, (double*)temp2);
    Matrix.Add((double*)temp2, (double*)Q, four, four, (double*)Pp); // Pp output (4x4)

    //Matrix.Copy((double*)Pp, four, four, (double*) temp1);

    Matrix.Multiply((double*)H, (double*)Pp, four, four, four, (double*)temp5);
    Matrix.Multiply((double*)temp5, (double*)transposeH, four, four, four, (double*)temp2);
    Matrix.Add((double*)temp2, (double*)R, four, four, (double*)inversion);
    Matrix.Invert((double*)inversion, four);
    Matrix.Multiply((double*)Pp, (double*)H, four, four, four, (double*)temp1);
    Matrix.Multiply((double*)temp1, (double*)inversion, four, four, four, (double*)K); // K output (4x4)

    Matrix.Multiply((double*)H, (double*)input, four, four, one, (double*)Y);  // Y output (4x1)

    Matrix.Multiply((double*)H, (double*)xp, four, four, one, (double*)temp3);
    Matrix.Subtract((double*)Y, (double*)temp3, four, one, (double*)temp4);
    Matrix.Multiply((double*)K, (double*)temp4, four, four, one, (double*)temp3);
    Matrix.Add((double*)xp, (double*)temp3, four, one, (double*)x); // x output (4x1)

    Matrix.Multiply((double*)K, (double*)H, four, four, four, (double*)temp1);
    Matrix.Subtract((double*)I, (double*)temp1, four, four, (double*)temp2);
    Matrix.Multiply((double*)temp2, (double*)Pp, four, four, four, (double*)temp1);
    Matrix.Multiply((double*)H, (double*)temp1, four, four, four, (double*)P);

                 P[0][1] = 0, P[0][2] = 0, P[0][3] = 0; // error covariance
    P[1][0] = 0,              P[1][2] = 0, P[1][3] = 0;
    P[2][0] = 0, P[2][1] = 0,              P[2][3] = 0;
    P[3][0] = 0, P[3][1] = 0, P[3][2] = 0;

    Matrix.Copy((double*)P, four, four, (double*) PM);
    Matrix.Copy((double*)x, four, one, (double*) PM2);

    gps_flag=0;
    flag=0;
  } 

  xpos_corrected = x[0][0];
  xvel_corrected = x[1][0];
  ypos_corrected = x[2][0];
  yvel_corrected = x[3][0];

  // Matrix.Print((double*)x,two,one,"State Estimate:");

}

void notify_swarm( )
{ 
         Serial3.print("<04,");
         Serial3.print(calclat,6); Serial3.print(',');
         Serial3.print(calclong,6); Serial3.print(',');
         Serial3.println('>');
        
}
void recvWithStartEndMarkers( ) {
    static boolean recvInProgress = false;
    
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    const byte numChars = 74;
    static char receivedChars[numChars];             // IF BROKEN LOOK HERE, array of chars -> string
  //x
 // if (Matlab.available() > 0) {
    if ( Serial2.available() ) {
        rc =  Serial2.read();
        // Serial.print(rc);
        // Serial.println("shtisandgiggle");
        if (recvInProgress == true) {
            if (rc != endMarker) {
             //  Serial.println(ndx);
                receivedChars[ndx] = rc;
                // Serial.print(receivedChars[ndx]);
                 //  Serial.println("test");
                 //  Serial.println(receivedChars);
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                    // ndx=0;
                    // rc=NULL;
                    // memset(receivedChars, 0, sizeof(receivedChars));
                    //  Serial.println("test1");

                    //  Serial.println(receivedChars);

                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                // Serial.println("");
                // Serial.println(receivedChars);
                tokenCreator(receivedChars,sizeof(inputString));
                                    // Serial.println("test2");
                 
                stringComplete = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
                               // Serial.println("test3");

           //  Serial.println(receivedChars);

        }
    }
}


void tokenCreator(char instr[],int strleng ){
 char * pch;
  //Serial.println("Splitting string \"%s\" into tokens:\n");

// strtok returns a pointer to a character array, not an index.
// char *s = strtok(receivedBytes,",");
// Serial.println(s);
 // Serial.println(instr);
  pch = strtok (instr,",");
  int index=0;
  while (pch != NULL)
  {

    stringparse(pch,index);
    pch = strtok (NULL, ",");
    //Serial.println(a);
  index++;
  
  }


}

void stringparse(char buffer[80],int ind)
{
  //  Serial.println(buffer);
  static int choice=0;
  //Serial.print(buffer);
  if(ind==0){
  if((strcmp(buffer,"08")==0))choice=1;
  if((strcmp(buffer,"09")==0))choice=3;
  if((strcmp(buffer,"10")==0))choice=2;
  }

  if (choice == 1) {

    if (ind == 1) myLat = atof(buffer);
    if (ind == 2) myLong = atof(buffer);
    // if(ind==3) myAltitude=atof(buffer);
    if (ind == 3) velocity = atof(buffer);           
    if (ind==4)    trueHeading=atof(buffer);
    if (ind == 5) choice = 0;
    gps_flag=1;

    radi=degtorad(trueHeading);
    xvelocity=velocity*sin(radi);
    yvelocity=velocity*cos(radi);

    inputString = "";
  }

  if(choice==2){

    if(ind==1) dummy=atof(buffer);
    if(ind==2) dummy=atof(buffer);
    if(ind==3) trueHeading=atof(buffer);
    if(ind==4) dummy=atof(buffer);
    if(ind==5) choice=0;

    radi=degtorad(trueHeading);
    xvelocity=velocity*sin(radi);
    yvelocity=velocity*cos(radi);
    inputString="";
  }


}





#define         WGS84_A               6378137.0
#define         WGS84_ECCSQ           0.00669437999013
//#define         WGS84_A               6378137.0
//#define         WGS84_ECCSQ           0.00669438002290    //NAD

/* This routine determines the correct UTM letter designator for the given
   latitude and returns 'Z' if latitude is outside the UTM limits of 84N to 80S
   Written by Chuck Gantz- chuck.gantz@globalstar.com */

char UTMLetterDesignator(double Lat)
{
  char LetterDesignator;
  
  if((84 >= Lat) && (Lat >= 72)) LetterDesignator = 'X';
  else if((72 > Lat) && (Lat >= 64)) LetterDesignator = 'W';
  else if((64 > Lat) && (Lat >= 56)) LetterDesignator = 'V';
  else if((56 > Lat) && (Lat >= 48)) LetterDesignator = 'U';
  else if((48 > Lat) && (Lat >= 40)) LetterDesignator = 'T';
  else if((40 > Lat) && (Lat >= 32)) LetterDesignator = 'S';
  else if((32 > Lat) && (Lat >= 24)) LetterDesignator = 'R';
  else if((24 > Lat) && (Lat >= 16)) LetterDesignator = 'Q';
  else if((16 > Lat) && (Lat >= 8)) LetterDesignator = 'P';
  else if(( 8 > Lat) && (Lat >= 0)) LetterDesignator = 'N';
  else if(( 0 > Lat) && (Lat >= -8)) LetterDesignator = 'M';
  else if((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
  else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
  else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
  else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
  else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
  else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
  else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
  else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
  else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
  else LetterDesignator = 'Z'; 
  return LetterDesignator;
}




void latLongToUtm(double Lat, double Long, double& UTMEasting, double& UTMNorthing, String& UTMZone) {
  double LongOrigin, LongOriginRad;
  double eccPrimeSquared;
  double k0 = 0.9996, N, T, C, A, M;
  double LatRad = Lat * M_PI / 180.0;
  double LongRad = Long * M_PI / 180.0;
  int ZoneNumber;

  ZoneNumber = (int)((Long + 180) / 6) + 1;
  
  if(Lat >= 56.0 && Lat < 64.0 && Long >= 3.0 && Long < 12.0)
    ZoneNumber = 32;
  
  // Special zones for Svalbard
  if(Lat >= 72.0 && Lat < 84.0) {
    if(Long >= 0.0  && Long <  9.0) ZoneNumber = 31;
    else if(Long >= 9.0  && Long < 21.0) ZoneNumber = 33;
    else if(Long >= 21.0 && Long < 33.0) ZoneNumber = 35;
    else if(Long >= 33.0 && Long < 42.0) ZoneNumber = 37;
  }
  // +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;  
  LongOriginRad = LongOrigin * M_PI / 180.0;

  // compute the UTM Zone from the latitude and longitude
//  sprintf(UTMZone, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));
  //Stringstream sstr;
 //  Serial.print(" Zone Number: ");
 // Serial.print(ZoneNumber);Serial.println(UTMLetterDesignator(Lat));
  //UTMZone = sstr.str();// fizme

  eccPrimeSquared = WGS84_ECCSQ / (1 - WGS84_ECCSQ);
  N = WGS84_A / sqrt(1 - WGS84_ECCSQ * sin(LatRad) * sin(LatRad));
  T = tan(LatRad) * tan(LatRad);
  C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
  A = cos(LatRad) * (LongRad-LongOriginRad);
  M = WGS84_A * ((1 - WGS84_ECCSQ / 4 - 3 * WGS84_ECCSQ * WGS84_ECCSQ / 64
            - 5 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 256) * LatRad 
           - (3 * WGS84_ECCSQ / 8 + 3 * WGS84_ECCSQ * WGS84_ECCSQ / 32
              + 45 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 1024) * 
           sin(2 * LatRad) + (15 * WGS84_ECCSQ * WGS84_ECCSQ / 256 +
                              45 * WGS84_ECCSQ * WGS84_ECCSQ * 
                              WGS84_ECCSQ / 1024) * sin(4 * LatRad) 
           - (35 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 3072) * 
           sin(6 * LatRad));
 UTMEasting = (double)(k0 * N * (A + (1 - T + C) * A * A * A / 6
                                   + (5 - 18 * T + T * T + 72 * C - 
                                      58 * eccPrimeSquared)* 
                                  A * A * A * A *A / 120) + 500000.0);
  UTMNorthing = (double)(k0 * (M + N * tan(LatRad) * 
                                (A * A / 2 + (5 - T + 9 * C + 4 * C * C)
                                 * A * A * A *A / 24
                                 + (61 - 58 * T + T * T + 
                                    600 * C - 330 * eccPrimeSquared) * 
                                 A * A * A * A * A * A / 720)));
  if(Lat < 0)
    UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere
}

// void latLongToUtm(double Lat, double Long, double* UTMEasting, double* UTMNorthing, char* UTMZone) {
//   String zone;
//   latLongToUtm(Lat, Long, UTMEasting, UTMNorthing, zone);
//   strcpy(UTMZone, zone.c_str());
// }
  /* converts UTM coords to lat/long.  Equations from USGS Bulletin 1532 
     East Longitudes are positive, West longitudes are negative. 
     North latitudes are positive, South latitudes are negative
     Lat and Long are in decimal degrees. 
     Written by Chuck Gantz- chuck.gantz@globalstar.com */

void utmToLatLong(double UTMEasting, double UTMNorthing, const String& UTMZone, double& Lat,  double& Long)
{
  double k0 = 0.9996, eccPrimeSquared, N1, T1, C1, R1, D, M;
  double e1 = (1 - sqrt(1 - WGS84_ECCSQ))/(1 + sqrt(1 - WGS84_ECCSQ));
  double LongOrigin, mu, phi1, phi1Rad, x, y;
  int ZoneNumber, NorthernHemisphere; // 1 for northern hem., 0 for southern
  char* ZoneLetter;
  
  x = UTMEasting - 500000.0; /* remove 500,000 meter offset for longitude */
  y = UTMNorthing;
  
  ZoneNumber = strtoul(UTMZone.c_str(), &ZoneLetter, 10);
  if((*ZoneLetter - 'N') >= 0)
    NorthernHemisphere = 1;  /* point is in northern hemisphere */
  else {
    NorthernHemisphere = 0;  /* point is in southern hemisphere */
    y -= 10000000.0;         /* remove 10,000,000 meter offset 
                                used for southern hemisphere */
  }
  
  /* +3 puts origin in middle of zone */
  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;  
  
  eccPrimeSquared = (WGS84_ECCSQ) / (1 - WGS84_ECCSQ);
  
  M = y / k0;
  mu = M / (WGS84_A * (1 - WGS84_ECCSQ / 4 - 
                       3 * WGS84_ECCSQ * WGS84_ECCSQ / 64 - 5 * WGS84_ECCSQ * 
                       WGS84_ECCSQ * WGS84_ECCSQ / 256));
  phi1Rad = mu + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * mu) +
    (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * mu) +
    (151 * e1 * e1 * e1 / 96) * sin(6 * mu);
  phi1 = dgc_r2d(phi1Rad);
  
  N1 = WGS84_A / sqrt(1 - WGS84_ECCSQ * sin(phi1Rad) * sin(phi1Rad));
  T1 = tan(phi1Rad) * tan(phi1Rad);
  C1 = eccPrimeSquared * cos(phi1Rad) * cos(phi1Rad);
  R1 = WGS84_A * (1 - WGS84_ECCSQ) / 
    pow(1 - WGS84_ECCSQ * sin(phi1Rad) * sin(phi1Rad), 1.5);
  D = x / (N1 * k0);
  
  Lat = phi1Rad - (N1 * tan(phi1Rad) / R1) * 
    (D * D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * eccPrimeSquared) * 
     D * D * D * D / 24 +
     (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 
      252 * eccPrimeSquared - 3 * C1 * C1) * D * D * D * D * D * D / 720);
  Lat = dgc_r2d(Lat);

  Long = (D - (1 + 2 * T1 + C1) * D * D * D / 6 + 
           (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 
            8 * eccPrimeSquared + 24 * T1 * T1)
          * D * D * D * D * D / 120) / cos(phi1Rad);
  Long = LongOrigin + dgc_r2d(Long);
}





inline double dgc_r2d(double theta) {
    return (theta * 180.0 / M_PI);
}

inline double dgc_d2r(double theta) {
    return (theta * M_PI / 180.0);
}

inline double dgc_r2df(double theta) {
    return (theta * 180.0 / M_PI);
}


double degtorad(double deg)
{
  double rad = deg*(3.14159265359/180);
  return rad;
}
