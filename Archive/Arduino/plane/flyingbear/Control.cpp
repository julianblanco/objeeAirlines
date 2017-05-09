#include "Control.h"

// Servo trim and control coefficients
const int elev_sub_trim = 100;
int servo_trim[] = {
  -78, 0, 0};
const float P_coef[] = {
  15, 20, -1, -5};
const float I_coef[] = {
  .02, .01, -.005, 0};
const float D_coef[] = {
  (-4 / GYRO_SENS), (-2 / GYRO_SENS), (2 / GYRO_SENS), 0};
const float set[] = {
  -5, 0, 0, AIRSPEED};


TServo elevator, r_aileron, l_aileron;
TServo* servo_ptr[] = {
  &elevator, &r_aileron, &l_aileron};

TControlVals pitch, roll, yaw, pitot;
TControlVals* ctrl_vals[] = {
  &pitch, &roll, &yaw, &pitot};




void ServoInit() {
  // Initialize control values
  
  for (int i = 0; i < 3; i++) {
    // Setup servo
    servo_ptr[i]->center = 1500 + servo_trim[i];
    servo_ptr[i]->servo.attach(i+4, SERVO_LOWER, SERVO_UPPER);  // Attach servos to pins 5 - 7
    servo_ptr[i]->servo.writeMicroseconds(servo_ptr[i]->center);
  }
  delay(500);

  // Test servos
  for (int i = 0; i < 3; i++) {
    servo_ptr[i]->servo.writeMicroseconds(SERVO_LOWER + servo_trim[i]);
    delay(250);
    servo_ptr[i]->servo.writeMicroseconds(SERVO_UPPER + servo_trim[i]);
    delay(250);
    servo_ptr[i]->servo.writeMicroseconds(servo_ptr[i]->center);
  }
  delay(1000);
}

void ControlResponse(int use_pitot) {
  // Calculate angular error
 
}

void PID(   ) {

}




