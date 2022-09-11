/*
 * Inclinometer.h
 *
 *  Created on: May 27, 2022
 *      Author: kai
 *
 *  Calculates inclined angle from IMUs measurement.
 *  Angle is calculated from Force on Y-Axis.
 *  Calibration is done by passing y and z Forces to Calibrate-Function
 *  Can be calibrated to give alarm at specified angles.
 *
 */

#ifndef INC_INCLINOMETER_H_
#define INC_INCLINOMETER_H_

#include <inttypes.h>

namespace SoftwareSensors{

class Inclinometer{
public:
	// Auto calibrates zero degrees to F= 9.810N
	Inclinometer();

	// Calibrate absolute Zero Degrees:
	// Force on Axis needs to be in Milli-Newtons:
	void calibrate_ZeroDegrees(signed int Fy_calibrate,signed int Fz_calibrate);

	// Get current Angle,
	//dependet on Calibration the angle might be negative:
	signed int get_InclineAngle(signed int Fy);
private:
	signed int F_zeroDegrees;
	signed int alias_zeroDegrees;
	uint16_t F_max; // Maximum Force if y-Axis is in line with Fg
	const double PI = 3.14159265;

};
}


#endif /* INC_INCLINOMETER_H_ */
