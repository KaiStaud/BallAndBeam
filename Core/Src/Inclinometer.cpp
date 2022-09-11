/*
 * Inclinometer.cpp
 *
 *  Created on: 27.05.2022
 *      Author: kai
 */

#include <Inclinometer.h>
#include <math.h>
using namespace SoftwareSensors;

Inclinometer::Inclinometer(){
	F_zeroDegrees = 981;
}

// Calibrate absolute Zero Degrees:
// Force on Axis needs to be in Milli-Newtons:
void Inclinometer::calibrate_ZeroDegrees(signed int Fy_calibrate,signed int Fz_calibrate){

 /* Acos expects values between [-1,1]
 * Calculate the resulting vector from second vector
 */
F_max = sqrt(pow(Fy_calibrate, 2)+pow(Fz_calibrate, 2));
alias_zeroDegrees = acos(F_max / Fy_calibrate);

}

// Get current Angle,
//dependet on Calibration the angle might be negative:
signed int Inclinometer::get_InclineAngle(signed int Fy){
signed int angle = acos(double(Fy) / double(F_max))*180.0 / PI;
return angle - alias_zeroDegrees;
}


