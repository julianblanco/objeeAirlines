#include "IIRFilter.h"
#include <Arduino.h>

//function [estimated_headOut, gyro_last] = HeadingIIRFilter(GPS_headput, estimated_headIn, gyroCurrent, gyroLast,  inputPrecision)
//HEADINGIIRFILTER syncs the gyro realtive heading and gps true heading
//under as new estimated_head heading. This will allow realtively accurate
//steering for some time after GPS fails. Run this recursively with self.
// -----------------------------------------------------------------------
// Parameters:
// 1: GPS_headput --> this is the true heading from a gps sensor
// 2: estimated_headIn --> the function is meant to be recursive, this is
//                          the output from the last run
// 3: gyroCurrent --> this is the relative heading from a gyro sensor
// 4: gyroLast --> last gyro heading from previous iteration of function
// 5: inputPrecision --> this is the decimal precision of the output heading
// ------------------------------------------------------------------------
// Output(s):
// 1: estimated_headOut --> correlated output heading in degrees true
// 2: gyro_last --> last gyro heading that will be fed into recursive function
// ------------------------------------------------------------------------
// Inifinite Impulse Response Filter
// ------------------------------------------------------------------------
// gyro runs 100Hz test at 75Hz, GPS runs at 1Hz test at 1Hz.
// Test gyro at 90 degrees, GPS at 0 degrees. Gyro needs to correct to GPS
// Start with precision of +-1 degree (epsilon value, ?ceiling fcn?)
// Start with 1 degree of freedom; to do this, fix the GPS value.
// Final product needs to work with changing GPS and Gyro; 2 variables
// -----------------------------------------------------------------------
// Important Note: the gyro and the GPS need to be synced computationally
// only, this will not involve movement of the vehicle.


// Update heading estimation based on difference in gyro data between loops and incoming GPS data
float update_heading_estimate(float GPS_head, float gyro_current, float estimated_head, bool use_GPS,float GPS_COEF)
{
        // Static variable to hold last gyro value
        static float gyro_last = 0;

        // Update estimated heading using the change in the gyroscope since the last iteration
        estimated_head = correct_wrap(estimated_head + angular_diff(gyro_current, gyro_last));

        // Update estimated heading using the new GPS value
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
