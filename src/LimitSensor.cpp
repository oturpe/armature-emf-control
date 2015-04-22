#include "config.h"

#include <avr/io.h>

#include "Atmega328pUtils.h"

#include "LimitSensor.h"

Direction LimitSensor::opposite(Direction direction) {
    switch (direction) {
    case D_CLOCKWISE:
      return D_COUNTER_CLOCKWISE;
    case D_COUNTER_CLOCKWISE:
      return D_CLOCKWISE;
    case D_BOTH:
      return D_BOTH;
    case D_NONE:
      return D_NONE;
    }
  }

LimitSensor::LimitSensor(uint16_t sensingInterval)
  : counter(0), sensingInterval(sensingInterval) {
}

Direction LimitSensor::senseLimit() {
  counter = (counter + 1) % sensingInterval;
  if (counter)
    return D_NONE;

  bool clockwiseSensor = false;
  bool counterClockwiseSensor = false;

  // Read clockwise sensr from pin ADC1
  ADMUX &= ~BV(MUX3) & ~BV(MUX2) & ~BV(MUX1);
  ADMUX |= BV(MUX0);

  // start conversion and wait until value is available
  ADCSRA |= BV(ADSC);
  while(ADCSRA & BV(ADSC));

  if (ADC > POSITION_SENSOR_THRESHOLD_CW)
    clockwiseSensor = true;

  // Read counter clockwise sensor from pin ADC2
 ADMUX &= ~BV(MUX3) & ~BV(MUX2) & ~BV(MUX0);
 ADMUX |= BV(MUX1);

  // start conversion and wait until value is available
  ADCSRA |= BV(ADSC);
  while(ADCSRA & BV(ADSC));

  if (ADC > POSITION_SENSOR_THRESHOLD_CCW)
    counterClockwiseSensor = true;

if(clockwiseSensor && counterClockwiseSensor)
  return D_BOTH;

if(!clockwiseSensor && !counterClockwiseSensor)
  return D_NONE;

return clockwiseSensor ? D_CLOCKWISE : D_COUNTER_CLOCKWISE;
}

