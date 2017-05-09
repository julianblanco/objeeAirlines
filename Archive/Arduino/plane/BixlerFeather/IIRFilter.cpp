#include "IIRFilter.h"
#include <Arduino.h>
#include "variables.h"

// Update heading estimation based on difference in gyro data between loops and incoming GPS data
float update_heading_estimate(float GPS_head, float gyro_current, float estimated_head, bool use_GPS,float GPS_COEF)
{
         new_GPS_data = 0;
        // Static variable to hold last gyro value
        static float gyro_last = 0;

        // Update estimated heading using the change in the gyroscope since the last iteration
        estimated_head = correct_wrap(estimated_head + angular_diff(gyro_current, gyro_last));

        // Update estimated heading using the new GPS value
        use_GPS = 0;
        if (use_GPS)
                estimated_head = correct_wrap(estimated_head + GPS_COEF*(angular_diff(GPS_head, estimated_head)));

        gyro_last = gyro_current;

        return estimated_head;
}

// Fix any 360 degree wrap around that may occur while updating angular position
float correct_wrap(float current_heading)
{
// Correct 360 deg wrap around
if (current_heading >= 360)
        current_heading -= 360;
else if (current_heading < 0)
        current_heading += 360;

return(current_heading);
}


// Find the shortest signed angular difference between a target and source angle
float angular_diff(float target_angle, float source_angle)
{
        // Find simple difference
        float diff = target_angle - source_angle;
        if (diff > 180)
                diff -= 360;
        else if (diff < -180)
                diff += 360;

        return(diff);
}


void IIRFilter(void){
    // IIR Filter
    ////////////////////////////////////////////////////////////////////////////////////////////////////
   
    trueHeading = update_heading_estimate(Gpsheading, yawInput, trueHeading, use_GPS, gpsCoef);
  
}
