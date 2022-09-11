/*
 * SysInit.h
 *
 *  Created on: 11.09.2022
 *      Author: kai
 */

#ifndef SYSINIT_SYS_INIT_H_
#define SYSINIT_SYS_INIT_H_

#include "../Inc/ICM20948_WE.h"

namespace Init{

enum class LightBarrierCheck{
	LeftLBTriggered,
	RightLBTriggered,
	LeftLBError,
	RightLBError
};

enum IMUCheck{
	ReadOutOK,
	ReadOutError
};

double read_tilt_angle(ICM20948_WE myIMU);
LightBarrierCheck sensor_checkup(ICM20948_WE myIMU);
}


#endif /* SYSINIT_SYS_INIT_H_ */
