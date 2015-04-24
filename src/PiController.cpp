#include "PiController.h"

PiController::PiController(int16_t target,
                           int16_t positionCoeff,
                           int16_t integralCoeff,
                           int16_t resolution)
  : target(target),
    positionCoeff(positionCoeff),
    integralCoeff(integralCoeff),
    resolution(resolution),
    integral(0) {
}

void PiController::setTarget(int16_t newTarget) {
  if (newTarget - target < resolution && target - newTarget < resolution) {
    return;
  }

  target = newTarget;
  integral = 0;
}

int16_t PiController::control(int16_t input) {
  int16_t position = input - target;
  integral += position;

  int16_t change = (position / positionCoeff) + (integral / integralCoeff);
  return target - change;
}