#include "MatrixMath.h" // downloaded library

// defines matrix dimensions
#define one (1)
#define two (2)

// global variables & matrix sizes
float A[two][two];
float H[one][two];
float Q[two][two];
float R[one][one];
float x[two][one];
float P[two][two];


float input_data = 0;

// working matricies / outputs
float input[one][one];
float xp[two][one];
float Pp[two][two];
float K[two][one];
float inversion[one][one];
float transposeA[two][two];
float transposeH[two][one];
float temp1[two][two];
float temp2[two][two];
float temp3[one][one];
float temp4[one][two];
float temp5[two][one];
float temp6[one][one];




float myLat = 0;
float myLong = 0;
float myAltitude = 0;
float pitchInput = 0;
float rollInput = 0;
float trueHeading = 0;
float GpsSpeed = 0;
float AirSpeed = 0;
String inputString = "";
boolean stringComplete = false;  // whether the string is complete
float time_diff;
float time_diff_prev;
float currenttime;
// the setup function runs once when you press reset or power the board
void setup()
{

  Serial.begin(115200);
  Serial2.begin(115200);
  delay(500);
  Serial.println("Kalman Filter Initializing...");
  Serial2.println("Kalman Filter Initializing...");

  input[0][0] = pitchInput;

  A[0][0] = 1, A[0][1] = currenttime;
  A[1][0] = 0, A[1][1] = 1;

  H[0][0] = 1, H[0][1] = 0;

  Q[0][0] = .25, Q[0][1] = 0;
  Q[1][0] = 0,   Q[1][1] = .25;

  R[0][0] = 1;

  x[0][0] = 40;
  x[1][0] = 0;

  P[0][0] = 1, P[0][1] = 0;
  P[1][0] = 0, P[1][1] = 1;

  Matrix.Transpose((float*)A, two, two, (float*)transposeA);
  Matrix.Transpose((float*)H, one, two, (float*)transposeH);
}

// the loop function runs over and over again forever
void loop()
{
 
  // bring in time and input_data
  recvWithStartEndMarkers(Serial2);
  delay(1);

}

void kalmanshit()
{
  Matrix.Print((float*)x, two, one, "State Estimate:");
  input[0][0] = pitchInput;
  A[0][1] = time_diff;


  // kalman filter
  Matrix.Multiply((float*)A, (float*)x, two, two, one, (float*)xp); // xp output

  Matrix.Multiply((float*)A, (float*)P, two, two, two, (float*)temp1);
  Matrix.Multiply((float*)temp1, (float*)transposeA, two, two, two, (float*)temp2);
  Matrix.Add((float*)temp2, (float*)Q, two, two, (float*)Pp); // Pp output

  Matrix.Multiply((float*)H, (float*)Pp, one, two, two, (float*)temp4);
  Matrix.Multiply((float*)temp4, (float*)transposeH, one, two, one, (float*)temp3);
  Matrix.Add((float*)temp3, (float*)R, one, one, (float*)inversion);
  Matrix.Invert((float*)inversion, one);
  Matrix.Multiply((float*)Pp, (float*)transposeH, two, two, one, (float*)temp5);
  Matrix.Multiply((float*)temp5, (float*)inversion, two, one, one, (float*)K); // K output

  Matrix.Multiply((float*)H, (float*)xp, one, two, one, (float*)temp3);
  Matrix.Subtract((float*)input, (float*)temp3, one, one, (float*)temp6);
  Matrix.Multiply((float*)K, (float*)temp6, two, one, one, (float*)temp5);
  Matrix.Add((float*)xp, (float*)temp5, two, one, (float*)x); // x output

  Matrix.Multiply((float*)K, (float*)H, two, one, two, (float*)temp1);
  Matrix.Multiply((float*)temp1, (float*)Pp, two, two, two, (float*)temp2);
  Matrix.Subtract((float*)Pp, (float*)temp2, two, two, (float*)P);

  // Matrix.Print((float*)x,two,one,"State Estimate:");
}


void recvWithStartEndMarkers(HardwareSerial &xbeeSer) {
  static boolean recvInProgress = false;

  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
  const byte numChars = 74;
  static char receivedChars[numChars];             // IF BROKEN LOOK HERE, array of chars -> string
  //x
  // if (Matlab.available() > 0) {
  if (xbeeSer.available() ) {
    rc = xbeeSer.read();
    //xbeeSer.print(rc);
    //xbeeSer.println("shtisandgiggle");
    if (recvInProgress == true) {
      if (rc != endMarker) {
        // xbeeSer.println(ndx);
        receivedChars[ndx] = rc;
        //xbeeSer.print(receivedChars[ndx]);
        // xbeeSer.println("test");
        // xbeeSer.println(receivedChars);
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
          // ndx=0;
          // rc=NULL;
          // memset(receivedChars, 0, sizeof(receivedChars));
          // xbeeSer.println("test1");

          // xbeeSer.println(receivedChars);

        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        //xbeeSer.println("");
        //xbeeSer.println(receivedChars);
        tokenCreator(receivedChars, sizeof(inputString), xbeeSer);
        //xbeeSer.println("test2");

        stringComplete = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
      //xbeeSer.println("test3");

      // xbeeSer.println(receivedChars);

    }
  }
}


void tokenCreator(char instr[], int strleng, HardwareSerial &xbeeSer) {
  char * pch;
  //Serial.println("Splitting string \"%s\" into tokens:\n");

  // strtok returns a pointer to a character array, not an index.
  // char *s = strtok(receivedBytes,",");
  // Serial.println(s);
  //xbeeSer.println(instr);
  pch = strtok (instr, ",");
  int index = 0;
  while (pch != NULL)
  {

    stringparse(pch, index, xbeeSer);
    pch = strtok (NULL, ",");
    //Serial.println(a);
    index++;

  }
}


void stringparse(char buffer[80], int ind, HardwareSerial &xbeeSer)
{
  // xbeeSer.println(buffer);
  static int choice = 0;
  //Serial.print(buffer);
  if (ind == 0) {
    if ((strcmp(buffer, "08") == 0))choice = 1;
    if ((strcmp(buffer, "09") == 0))
    {
      choice = 3;

    }
    if ((strcmp(buffer, "10") == 0))
    {
      choice = 2;
      time_diff_prev = currenttime;
      currenttime = ((millis() / 1000.00));
      time_diff = currenttime - time_diff_prev;
   Serial.print("\t"); Serial.println(pitchInput);
  Serial.print("\t"); Serial.println(time_diff, 10);
  kalmanshit();
    }

  }//end indices

  if (choice == 1) {

    if (ind == 1) myLat = atof(buffer);
    if (ind == 2) myLong = atof(buffer);
    // if(ind==3) myAltitude=atof(buffer);
    if (ind == 3) GpsSpeed = atof(buffer);
    if (ind == 4) AirSpeed = atof(buffer);
    if (ind == 5) choice = 0;
    inputString = "";
  }

  if (choice == 2) {

    if (ind == 1) pitchInput = atof(buffer);
    if (ind == 2) rollInput = atof(buffer);
    if (ind == 3) trueHeading = atof(buffer);
    if (ind == 4) myAltitude = atof(buffer);
    if (ind == 5) choice = 0;
    inputString = "";
  }
  if (choice == 3) {

    if (ind == 1) myLat = atof(buffer);
    if (ind == 2) myLong = atof(buffer);
    if (ind == 3) myAltitude = atof(buffer);
    if (ind == 4) pitchInput = atof(buffer);
    if (ind == 5) rollInput = atof(buffer);
    if (ind == 6) trueHeading = atof(buffer);
    if (ind == 7) GpsSpeed = atof(buffer);
    if (ind == 8) choice = 0;
    inputString = "";
  }
}
