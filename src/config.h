// Cpu frequency for util/delay.h
#define F_CPU 16000000

#define PWM_PRESCALER Atmega328p::PSV_8
#define PWM_INITIAL 0

#define AVG_WINDOW 10

// Target value for ir controller
#define TARGET_EMF 245
// How many points of difference is needed change pwm by one step
#define POSITION_COEFF 1
#define INTEGRAL_COEFF 2

// Debugging definitions
//#define DEBUG
#define DEBUG_FREQ 1
