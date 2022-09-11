/*
 * SysInit.cpp
 *
 *  Created on: 11.09.2022
 *      Author: kai
 */

#include "sys_init.h"

#include "main.h"
#include "stm32g4xx_hal_gpio.h"
#include "math.h"
namespace Init{

double read_tilt_angle(ICM20948_WE myIMU)
{
	  myIMU.readSensor();
	  myIMU.getAccRawValues();
	  auto val = myIMU.getGValues();
	  return asin(val.z/1)*180/3.14159265;
}

LightBarrierCheck sensor_checkup(ICM20948_WE myIMU)
{
	LightBarrierCheck ret;
	  // Alle sensoren testen:
	  // Distanz-Sensor:
	  double tilt_angle = read_tilt_angle(myIMU);
	  if(read_tilt_angle(myIMU) > 0)
	  {
	  //	// Sensor senken, bis untere Kalibrierposition -30° erreicht ist
	  	  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
	  	  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
	  	  HAL_Delay(10);
	  	  ret = LightBarrierCheck::RightLBTriggered;
	  	while(read_tilt_angle(myIMU) > -23);
	  }
	  else
	  {
	  	  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
	  	  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	  	  HAL_Delay(10);

	  	  ret = LightBarrierCheck::LeftLBTriggered;
	  //	// Sensor heben, bis obere Kalibrierposition erreicht ist:
	  	while(read_tilt_angle(myIMU) < 25);
	  }
	  	  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
	  	  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	  	  return ret;
}

}

