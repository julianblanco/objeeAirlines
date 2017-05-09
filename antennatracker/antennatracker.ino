#include <Servo.h>
#include <math.h>

Servo panServo;  
Servo tiltServo;  

float myLat;
float myLong;
float myAltitude;
float dummy;
String inputString ;
bool stringComplete;

float antennasLat= 41.327592;
float antennasLong= -72.045777;

void setup() {
  panServo.attach(9);  // attaches the servo on pin 9 to the servo object
  tiltServo.attach(10);  // attaches the servo on pin 9 to the servo object
  Serial.begin(115200);
  panServo.write(1500);
  tiltServo.write(1500);
  delay(3000);
}





/*


<08,41.320106,-72.045224,44.68,44.70>

North 41.333018, -72.046387
runway ne 41.336244, -72.038083

south 41.320106, -72.045224
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  _____         .__         .__                        
  /     \ _____  |__| ____   |  |   ____   ____ ______  
 /  \ /  \\__  \ |  |/    \  |  |  /  _ \ /  _ \\____ \ 
/    Y    \/ __ \|  |   |  \ |  |_(  <_> |  <_> )  |_> >
\____|__  (____  /__|___|  / |____/\____/ \____/|   __/ 
        \/     \/        \/                     |__| 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
*/

void loop() {
    recvWithStartEndMarkers();
    panServo.write(panAngle2Servo(courseToWaypoint(antennasLat, antennasLong, myLat, myLong)));
    tiltServo.write(tiltAngle2Servo(courseToWaypoint(antennasLat, antennasLong, myLat, myLong),0));
   
    delay(1);
  }




//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/*
___________                   __  .__                       
\_   _____/_ __  ____   _____/  |_|__| ____   ____   ______ 
 |    __)|  |  \/    \_/ ___\   __\  |/  _ \ /    \ /  ___/ 
 |     \ |  |  /   |  \  \___|  | |  (  <_> )   |  \\___ \  
 \___  / |____/|___|  /\___  >__| |__|\____/|___|  /____  > 
     \/             \/     \/                    \/     \/  

*/
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 float panAngle2Servo(float desiredangle)
 {
    // int returnservo =0;

    if(desiredangle>90 && desiredangle <270)
    {
      desiredangle = desiredangle -180;

    }
    else
    {
    if(desiredangle >269)
    {
      desiredangle = desiredangle -360;
    }
    }

    int returnservo = map(desiredangle,-90,90 ,2000,1000);
    return returnservo;
 }


 float tiltAngle2Servo(float desiredangle ,float altitude)
 {
    // int returnservo =0;

    float distance = distanceToWaypoint(antennasLat, antennasLong, myLat, myLong);
    float angleofplane = round( atan (myAltitude/distance) * 180/3.14159265 );
        Serial.print(myAltitude);Serial.print(',');
     Serial.print(distance);Serial.print(',');
    Serial.println(angleofplane );
    angleofplane = map(angleofplane,0,90 ,0,500);


    if(desiredangle>90 && desiredangle <270)
    {
      desiredangle = 2200 - angleofplane;

    }
    else
    {
       desiredangle = 800  + angleofplane;
    }


    


    int returnservo = desiredangle;
    return returnservo;
 }


 float courseToWaypoint(float lat1, float long1, float lat2, float long2)
{
// returns course in degrees (North=0, West=270) from position 1 to position 2,
// both specified as signed decimal-degrees latitude and longitude.
// Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
// Courtesy of Maarten Lamers
float dlon = radians(long2 - long1);
lat1 = radians(lat1);
lat2 = radians(lat2);
float a1 = sin(dlon) * cos(lat2);
float a2 = sin(lat1) * cos(lat2) * cos(dlon);
a2 = cos(lat1) * sin(lat2) - a2;
a2 = atan2(a1, a2);
if (a2 < 0.0)
{
    a2 += TWO_PI;
}
return degrees(a2);
}  // courseToWaypoint()


float distanceToWaypoint(float Lat1, float Long1, float Lat2, float Long2)
{
float dist;
float dLat = (float)(Lat2 - Lat1);                                    // difference of latitude in 1/10 000 000 degrees
float dLon = (float)(Long2 - Long1) * cos(Lat1) ; //
dist = sqrt(sq(dLat) + sq(dLon)) * 110312;

return dist;

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
    if ( Serial.available() ) {
        rc =  Serial.read();
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

  if(choice==1){

    if(ind==1) myLat=atof(buffer);
    if(ind==2) myLong=atof(buffer);
    // if(ind==3) myAltitude=atof(buffer);
    if(ind==3)  myAltitude=atof(buffer);
    if(ind==4) dummy=atof(buffer);
    if(ind==5) choice=0;
    inputString="";
  }

    if(choice==2){

    if(ind==1) dummy=atof(buffer);
    if(ind==2) dummy=atof(buffer);
    if(ind==3) dummy=atof(buffer);
    if(ind==4) dummy=atof(buffer);
    if(ind==5) choice=0;
    inputString="";
  }
  if(choice==3){

    if(ind==1) dummy=atof(buffer);
    if(ind==2) dummy=atof(buffer);
    if(ind==3) myAltitude=atof(buffer);
    if(ind==4) dummy=atof(buffer);
    if(ind==5) dummy=atof(buffer);
    if(ind==6) dummy=atof(buffer);
    if(ind==7) dummy=atof(buffer);
    if(ind==8) choice=0;
    inputString="";
  }
}













// float constrain_sample(float input,float mini, float maxi)
// {

// static float temp = input;
// if(input>maxi)input =maxi;
// if(input<mini)input = mini;
// if(isinf(input))Serial.print("WTF");
// if(isnan(input))Serial.print("WTF2");
// return input;

// }

// void sampleandfilter()
// {
//   // sens = analogRead(A1);
//   // sens1 = analogRead(A2);
//   // pos = analogRead(A0);
//   //pos = map(pos,0,1023,10,50);
//   distance = (((6787.00)/(sens - 3))-4);
//   //distance2 =(((6787.00)/(sens1 - 3))-4);
//   if(distance<25 )
//     {distanceavg= distance;}
//   else if(distance2<25)
//     {
//       distanceavg = (60-distance2);
//     }
//   else
//     {

//       distance=constrain_sample(distance,25,35);
//       distance2=constrain_sample(distance2,25,35);
//       distanceavg=constrain_sample((distance+(60-distance2))/2,25,35);
//     }

//   distanceavg = constrain_sample(distanceavg,0,60);
// }
  



// float controlPID(float ErrorCurrent, float Kp, float Ki, float Kd, int useKP, int useKI, int useKD, float satuarationpoint, float offset, float &Integralresponse, float &ErrorOld) 
// {
//   //**************************** Proportional***********************************************
//   //Calculate proportional response
//   float pResponse = ErrorCurrent * Kp;

//   //**************************** Integral***********************************************
//   //Calculate Integral response and add to accumulator: THINK ABOUT HOW TO RESET THE INTEGRAL RESPONSE
//   Integralresponse += ErrorCurrent * Ki;

//   if (Integralresponse > satuarationpoint) Integralresponse = satuarationpoint;
//   if (Integralresponse < -1*satuarationpoint) Integralresponse = -1*satuarationpoint;


//   //**************************** Derivative***********************************************

//   float timePID =25;// time elapsed per clock cycle; it works, but should be 100ms not 10. NEED TO HOOK UP TO OSCOPE
//   //Calculate dx/dt
//   float derivativeterm =(ErrorOld - ErrorCurrent)/timePID;
//   //pass current variable to error1 and overwrite previous value
//   ErrorOld = ErrorCurrent;
//   //Calculate derivative Response
//   float derivativeResponse = (0-derivativeterm)*Kd;      // tends towards derivativeSetpoint
//   //**************************** Total***********************************************
//   //Calculate total Response
//   float response = pResponse*useKP + Integralresponse*useKI +derivativeResponse*useKD+ offset;// + Integralresponse*useKI +derivativeResponse*useKD+offset;
//   return response;
// }

// float runningAverage(float M) {
//   #define LM_SIZE 15
//   static float LM[LM_SIZE];      // LastMeasurements
//   static byte index = 0;
//   static float sum = 0;
//   static byte count = 0;

//   // keep sum updated to improve speed.
//   sum -= LM[index];
//   LM[index] = M;
//   sum += LM[index];
//   index++;
//   index = index % LM_SIZE;
//   if (count < LM_SIZE) count++;

//   return sum / count;
// }


// void notifyuser()
// {
//   Serial.print(sens1);Serial.print(",");
//   Serial.print(distance); Serial.print(",");
//   Serial.print(distance2); Serial.print(",");
//   Serial.print(distanceavg); Serial.print(",");
//   Serial.print(error); Serial.print(",");
//   Serial.print(response); Serial.print(",");
//   Serial.print(pos); Serial.print(",");
//   Serial.print(intaccum); Serial.print(",");
//   Serial.print(tiempo); Serial.print(",");
//   Serial.println();
// }
