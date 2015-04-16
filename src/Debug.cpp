#include "Debug.h"

#include <avr/io.h>
#include <util/delay.h>

#include "Atmega328pUtils.h"

Debug::Debug(uint16_t reportingFrequency)
  : reportingFrequency(reportingFrequency), counter(0) {
  // Pin D5 as output.
  DDRD |= BV(DDD5);
}


void Debug::printInfo(uint16_t value) {
  counter = (counter + 1) % reportingFrequency;
  if(counter)
    return;

  for(uint16_t i = 0; i < value; i++) {
    PORTD |= BV(PORTD5);
    _delay_ms(50);
    PORTD &= ~BV(PORTD5);
    _delay_ms(50);
  }
}
