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

// TODOS:

#include "config.h"

#include <avr/io.h>
#include <util/delay.h>

#include "Atmega328pUtils.h"
#include "AveragingDataSet.h"
#include "PiController.h"

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

/// Enables pwm output using given duty cycle.
///
/// \param value
///    Requested duty cycle value
void enablePwm(uint8_t value) {
  OCR0A = value;
  TCCR0A |= BV(COM0A1);
}

/// Disables pwm output.
void disablePwm() {
  TCCR0A &= ~BV(COM0A1);
}

/// Initializes motor's electro motive force sensing with analog to digital
/// conversion by setting the reference.
void initializeEmfSense() {
  Atmega328p::setVoltageReference(Atmega328p::VREF_VCC);
  Atmega328p::setAdcPrescalerValue(Atmega328p::ADC_PSV_128);

  // Use analog input ADC1 with digital input disabled
  // AD0 is the default, no need to set MUXn bits of ADMUX
  // ADMUX |= BV(MUX0);
  DIDR0 |= BV(ADC0D);

  // Enable adc
  ADCSRA |= BV(ADEN);
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
  // start conversion and wait until value is available
  ADCSRA |= BV(ADSC);
  while(ADCSRA & BV(ADSC));

  // Measurement is done with inverse voltage. Invert again.
  return 1023 - ADC;
}

int main() {
  initializePwm();
  initializeEmfSense();

  #ifdef DEBUG
    Debug debug(DEBUG_FREQ);
  #endif

  AveragingDataSet readings(0);
  PiController controller(TARGET_EMF, POSITION_COEFF, INTEGRAL_COEFF);
  while(true) {
    // Run motor
    _delay_ms(20);

    // Disable pwm for measurement time
    disablePwm();

    for(int i = 0; i < AVG_WINDOW; i++) {
      _delay_ms(1);
      readings.add(senseEmf(), false);
    }

    int16_t newPwm = controller.control(readings.average());
    enablePwm(newPwm/4);

    #ifdef DEBUG
      debug.printInfo(readings.average()/100);
    #endif
  }
}