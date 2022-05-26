/*
 L298N.cpp - Libreria per gestire i movimenti di un motore DC con il modulo L298N
 Autore:   Andrea Lombardo
 Sito web: http://www.lombardoandrea.com
 */

#include "L298n.h"
#include "stm32g4xx_hal_tim.h"
typedef void (*CallBackFunction)();

//L298N::L298N(uint8_t IN1_Pin,GPIO_TypeDef IN1_Port, uint8_t IN2_Pin, GPIO_TypeDef IN2_Port,TIM_HandleTypeDef Timer, uint8_t Timer_Channel)
L298N::L298N(uint8_t IN1_Pin, uint8_t IN2_Pin, TIM_HandleTypeDef Timer,
		uint8_t Timer_Channel) {
	_timer = Timer;
	_timerChannel = Timer_Channel;
	_pinIN1 = IN1_Pin;
	_pinIN2 = IN2_Pin;
	//_portIN1 = IN1_Port;
	//_portIN2 = IN2_Port;
	_pwmVal = 0;
	_isMoving = false;
	_canMove = true;
	_lastMs = 0;
	_direction = STOP;

	stop();
}

void L298N::setSpeed(unsigned short pwmVal) {
	_pwmVal = pwmVal;
	__HAL_TIM_SET_COMPARE(&_timer, _timerChannel, pwmVal);

}

unsigned short L298N::getSpeed() {
	return _pwmVal;
}

void L298N::forward() {
	HAL_GPIO_WritePin(&_portIN1, _pinIN1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(&_portIN2, _pinIN2, GPIO_PIN_RESET);
	setSpeed(_pwmVal);
	_direction = FORWARD;
	_isMoving = true;
}

void L298N::backward() {
	HAL_GPIO_WritePin(&_portIN1, _pinIN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(&_portIN2, _pinIN2, GPIO_PIN_SET);
	setSpeed(_pwmVal);
	_direction = BACKWARD;
	_isMoving = true;
}

void L298N::run(L298N::Direction direction) {
	switch (direction) {
	case BACKWARD:
		this->backward();
		break;
	case FORWARD:
		this->forward();
		break;
	case STOP:
		this->stop();
		break;
	}
}

//Timing and callback
/*
 void L298N::runFor(unsigned long delay, L298N::Direction direction, CallBackFunction callback)
 {

 if ((_lastMs == 0) && _canMove)
 {
 _lastMs = millis();

 switch (direction)
 {
 case FORWARD:
 this->forward();
 break;
 case BACKWARD:
 this->backward();
 break;
 case STOP:
 default:
 this->stop();
 break;
 }
 }

 if (((millis() - _lastMs) > delay) && _canMove)
 {
 this->stop();
 _lastMs = 0;
 _canMove = false;

 callback();
 }
 }
 */
void L298N::runFor(unsigned long delay, L298N::Direction direction) {
	this->runFor(delay, direction, fakeCallback);
}

void L298N::forwardFor(unsigned long delay, CallBackFunction callback) {
	this->runFor(delay, FORWARD, callback);
}

void L298N::forwardFor(unsigned long delay) {
	this->runFor(delay, FORWARD);
}

void L298N::backwardFor(unsigned long delay, CallBackFunction callback) {
	this->runFor(delay, BACKWARD, callback);
}

void L298N::backwardFor(unsigned long delay) {
	this->runFor(delay, BACKWARD);
}

void L298N::stop() {
	HAL_GPIO_WritePin(&_portIN1, _pinIN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(&_portIN2, _pinIN2, GPIO_PIN_RESET);
	setSpeed(0);
	_direction = STOP;
	_isMoving = false;
}

void L298N::reset() {
	_canMove = true;
}

bool L298N::isMoving() {
	return _isMoving;
}

L298N::Direction L298N::getDirection() {
	return _direction;
}

void L298N::fakeCallback() {
}
