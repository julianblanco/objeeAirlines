#include "stepResp.h"
#include "../variables.h"
#include "Arduino.h"
#include "../craft.h"


void stepResp()
{
// 	samplebarometer(bmp);
// uint32_t counter1=0;

// //while loop allows step response to occur until it reaches 40 degrees and then the plane will attempt to recover


// while(pitchInput<70)
// 	{
//     counter1+=1;
// 	 uint32_t tiempo=millis();
// 	 char c = GNSS.read();
// 	 Serial_Sample(GNSS);

	
// 	 if (counter1%5) SampleGyro(bno);
// 	 if (counter1%10000) sdlog();
//    if (counter1%10)samplebarometer(bmp);
// 	 if (counter1%10)  UpdateServos(aileron,elevator,rudder);
// 	 stepResponse(heading, rollSetpoint, pitchSetpoint, trueHeading, rollInput, pitchInput,1,0,0);
	 
	 
//   	//delayTime =10 so should be 100 hz
// 	 while(millis()<tiempo+delayTime){char c = GNSS.read();}

//    if(counter1>500) break;
// 	}
}


void stepResponse(float yawTargetValue, float rollTargetValue, float pitchTargetValue, float yawIn, float rollIn, float pitchIn,float pitchtest, float rolltest,float yawtest) {
//   // Calculate angular error

//   float yawError =  angular_diff( yawTargetValue,yawIn);
//   yawResponse = controlPID(yawError,yawKp,yawKi,yawKd,1,0,1,40,400,400,1500);

//   //Next Three lines bank the plane based on yaw error, and set satuaration point.

//   //use next line if at sufficient altitude
//   //fixme
//   rollTargetValue =   -0.04*yawResponse; 
//   if (rollTargetValue > rollMaxAllowed) rollTargetValue=rollMaxAllowed;
//   if (rollTargetValue < -1*rollMaxAllowed) rollTargetValue=-1*rollMaxAllowed;

//   // Use shortest angle
//   float rollError = rollTargetValue - rollIn;

// pitchTargetValue=altitudeControl(0f,myAltitude,targetAlt,0,0);


//   float pitchError = pitchTargetValue - pitchIn;

 
//    pitchResponse = controlPID(pitchError,pitchKp,pitchKi,pitchKd,1,0,1,40,400,400,1500);
//   rollServoOutput = controlPID(rollError,rollKp,rollKi,rollKd,1,0,1,40,400,400,1500);


//   //rollIn=rollIn*(3.14159/180);

//   yawServoOutput = controlPID(yawError,-1*yawKp,yawKi,yawKd,1,0,0,40,400,400,1500);
//   pitchServoOutput = controlPID(pitchError,-1*pitchKp,pitchKi,pitchKd,1,0,0,40,400,400,1500);



// //If statments check to see what test is being conducted and jams the control surface hard.
//   if(rolltest)
//   {
//   	rollServoOutput=1700;
//   }
//    if(pitchtest)
//   {
//   	pitchServoOutput=2000;
//   }
//    if(yawtest)
//   {
//   yawServoOutput=2000;
//   }

 // yawServoOutput=(cos(rollIn)*yawResponse + .25*sin(rollIn)*pitchResponse)+1500;
  //pitchServoOutput=(.25*sin(rollIn)*yawResponse + cos(rollIn)*pitchResponse)+1500;

}

