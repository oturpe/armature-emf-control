#include "PiController.h"

PiController::PiController(int16_t target,
	                       int16_t positionCoeff,
	                       int16_t integralCoeff)
  : target(target),
    positionCoeff(positionCoeff),
    integralCoeff(integralCoeff),
    integral(0) {
}

int16_t PiController::control(int16_t input) {
  int16_t position = input - target;
  integral += position;

  int16_t change = (position / positionCoeff) + (integral / integralCoeff);
  return target - change;
}