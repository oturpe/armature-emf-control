// Knife motor control
//
// Firmware for atmega328p based dc motor controller with adjustable speed for
// a kinetic sculpture to be presented in the Slovenian Pavilion of Venice
// Biennale 2015.
//
// This is a simple controller based on adjusting motor pwm drive based on
// measured counter emf.
//
// Author: Otto Urpelainen
// Email: oturpe@iki.fi

#include "config.h"

#include <avr/io.h>
#include <util/delay.h>

#include "Atmega328pUtils.h"

#include "AveragingDataSet.h"
#include "PiController.h"
#include "LimitSensor.h"

#ifdef DEBUG
#include "Debug.h"
#endif

/// Initializes pin D6 as phase correct pwm.
///
/// Initialization does not include enabling pwm. That can is done using
/// enablePwm(unit8_t) function. This function leaves the pwm pin value low,
/// so if pwm is disabled with disablePwm() function, pin goes low.
void initializePwm() {
  // Set PD6 as output
  DDRD |= BV(DDD6);

  // Non-inverting mode
  TCCR0A |= BV(COM0A1);
  // Phase correct pwm mode
  TCCR0A |= BV(WGM00);

  Atmega328p::setTimer0Prescaler(PWM_PRESCALER);
}

/// Enables pwm output retaining the duty cycle that was in effect when it was
/// last disabled.
void enablePwm() {
  TCCR0A |= BV(COM0A1);
}

/// Enables pwm output using given duty cycle.
///
/// \param value
///    Requested duty cycle value
void enablePwm(uint8_t value) {
  OCR0A = value;
  enablePwm();
}

/// Disables pwm output. It can be enabled again with either enablePwm() or
/// enablePwm(uint8_t) function.
void disablePwm() {
  TCCR0A &= ~BV(COM0A1);
}

/// Initializes analog to digital conversion by setting the reference and
/// prescaler.
void initializeAdc() {
  Atmega328p::setVoltageReference(Atmega328p::VREF_VCC);
  Atmega328p::setAdcPrescalerValue(ADC_PRESCALER);

  // Enable adc
  ADCSRA |= BV(ADEN);

// Disable digital inout from pins that are used for adc.
  DIDR0 |= BV(ADC0D) | BV(ADC1D) | BV(ADC2D) | BV(ADC3D);
}

/// Sense motor's electromotive force.
///
/// This function starts analog to digital conversion and waits until a value
/// is available.
///
/// This function should only be called when the motor is not fed with pwm,
/// otherwise effect of changing pwm current might give inconsistent readings.
///
/// Values returned by this function grow when motor's counter-emf grows (i.e.
/// it spins faster). Range is 0 ... 1023
///
/// \return
///   Voltage inverse
uint16_t senseEmf() {
  // Select analog input ADC0
   ADMUX &= ~BV(MUX0) & ~BV(MUX1) & ~BV(MUX2) & ~BV(MUX3);

  // start conversion and wait until value is available
  ADCSRA |= BV(ADSC);
  while(ADCSRA & BV(ADSC));

  // Measurement is done with inverse voltage. Invert again.
  return 1023 - ADC;
}

/// Stops the motor and loops endlessly. This function in intended for stopping
/// operation case something unexpected happens
void halt() {
  disablePwm();
  PORTD |= BV(PORTD5);
  while(true);
}

/// Initializes the pin that is used to control direction setting.
void initializeDirectionSetting() {
  // Set PD7 as output
  DDRD |= BV(DDD7);
}

/// Sets motor rotation direction by flipping the relay.
///
/// \param direction
///    Direction to rotate to. For D_NONE or D_BOTH does not change the
///    direction.
void setDirection(Direction direction) {
  switch(direction) {
  case D_CLOCKWISE:
    PORTD &= ~BV(PORTD7);
    break;
  case D_COUNTER_CLOCKWISE:
    PORTD |= BV(PORTD7);
    break;
  default:
    break;
  }
}

uint16_t stationaryMotorEmf;

/// Initializes speed setting by reading the counter-emf sense value and setting
/// it as maximum possible. This value is used for calibrating the speed setting
/// potentiometer reading.
///
/// This function must be called when the motor is not rotating to function as
/// intended.
void initializeSpeedSetting() {
  // Select analog input ADC0
   ADMUX &= ~BV(MUX3) & ~BV(MUX2) & ~BV(MUX1) & ~BV(MUX0);

  // start conversion and wait until value is available
  ADCSRA |= BV(ADSC);
  while(ADCSRA & BV(ADSC));

  // Measurement is done with inverse voltage. Invert again. A little bit of
  // safety is added on top.
  stationaryMotorEmf = (1023 - ADC) + 20;
}

/// Reads the speed setting potetiometer and returns value between minimum
/// and maximum setting. Minimum value is configured with macro SPEED_MINIMUM,
/// maximum is initialized during startup with initializeSpeedSetting() call.
int16_t senseSpeedSetting() {
// Select analog input ADC3
   ADMUX &= ~BV(MUX3) & ~BV(MUX2);
   ADMUX |= BV(MUX1) | BV(MUX0);

  // start conversion and wait until value is available
  ADCSRA |= BV(ADSC);
  while(ADCSRA & BV(ADSC));

  int32_t rawReading = ADC;
  int32_t range = stationaryMotorEmf - TARGET_EMF_MINIMUM;
  return ((rawReading * range)/1023) + TARGET_EMF_MINIMUM;
}

/// Limits give value by given minimum and maximum values.
///
/// \param value
///    Value to limit
///
/// \param min
///    Minimum value
///
/// \param max
///    Maximum value
///
/// \return
///    Limited value
int16_t limit(int16_t value, int16_t min, int16_t max) {
  if(value < min)
    return min;
  if(value > max)
    return max;

  return value;
}

int main() {
  initializePwm();
  initializeAdc();
  initializeDirectionSetting();
  initializeSpeedSetting();

  #ifdef DEBUG
    Debug debug(DEBUG_FREQ);
  #endif

  AveragingDataSet readings(0);
  LimitSensor limitSensor(1);
  Direction currentLimit = D_NONE;
  PiController controller(TARGET_EMF_MINIMUM, POSITION_COEFF, INTEGRAL_COEFF);
  while(true) {
    // Run motor
    _delay_ms(50);

    int16_t feed;
    _delay_ms(1);
    Direction limit = limitSensor.senseLimit();
    controller.setTarget(senseSpeedSetting());

    // Disable pwm during relay and back-emf manipulation to reduce error and
    // transients
    disablePwm();

    Direction newDirection;
    switch(limit) {
    case D_CLOCKWISE:
    case D_COUNTER_CLOCKWISE:
      if(limit == currentLimit) {
        break;
      }
      _delay_ms(500);
      newDirection = LimitSensor::opposite(limit);
      setDirection(newDirection);
      currentLimit = limit;
      _delay_ms(500);
      break;
    case D_NONE:
      for(int i = 0; i < AVG_WINDOW; i++) {
        _delay_ms(1);
        readings.add(senseEmf(), false);
      }

      feed = controller.control(readings.average());
      feed = ::limit(feed, 0, 1023);

      break;
    case D_BOTH:
      //halt();
      break;
    }

    enablePwm(feed/4);
    #ifdef DEBUG
      debug.printInfo(feed/64);
    #endif
  }
}