/*
 * L298n.h
 *
 *  Created on: Apr 20, 2022
 *      Author: kai
 */

#ifndef INC_L298N_H_
#define INC_L298N_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"
typedef void (*CallBackFunction)();

class L298N {
public:
	typedef enum {
		FORWARD = 0, BACKWARD = 1, STOP = -1
	} Direction;

//   L298N(uint8_t IN1_Pin,GPIO_TypeDef IN1_Port, uint8_t IN2_Pin, GPIO_TypeDef IN2_Port,TIM_HandleTypeDef Timer, uint8_t Timer_Channel);
	L298N(uint8_t IN1_Pin, uint8_t IN2_Pin, TIM_HandleTypeDef Timer,
			uint8_t Timer_Channel);

	void setSpeed(unsigned short pwmVal);
	unsigned short getSpeed();
	void forward();
	void forwardFor(unsigned long delay, CallBackFunction callback);
	void forwardFor(unsigned long delay);
	void backward();
	void backwardFor(unsigned long delay, CallBackFunction callback);
	void backwardFor(unsigned long delay);
	void run(L298N::Direction direction);
	void runFor(unsigned long delay, L298N::Direction direction);
	void runFor(unsigned long delay, L298N::Direction direction,
			CallBackFunction callback);
	void stop();
	void reset();
	bool isMoving();
	Direction getDirection();

private:
	TIM_HandleTypeDef _timer;
	uint8_t _timerChannel;
	GPIO_TypeDef _portIN1;
	GPIO_TypeDef _portIN2;
	uint8_t _pinIN1;
	uint8_t _pinIN2;
	uint8_t _pwmVal;
	unsigned long _lastMs;
	bool _canMove;
	bool _isMoving;
	L298N::Direction _direction;
	static void fakeCallback();
};

#endif /* INC_L298N_H_ */
