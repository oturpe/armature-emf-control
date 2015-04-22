// Cpu frequency for util/delay.h
#define F_CPU 8000000

#define PWM_PRESCALER Atmega328p::PSV_1
#define PWM_INITIAL 0

#define ADC_PRESCALER Atmega328p::ADC_PSV_32

#define AVG_WINDOW 3

// Target value for ir controller
#define TARGET_EMF /*245*/ 300
// Smallest value that can be set for target emf. This value should be a little
// less that the smallest value that makes the motor move to maximize range of
// speed setting potentiometer.
#define TARGET_EMF_MINIMUM 200
// How many points of difference is needed change pwm by one step
#define POSITION_COEFF 1
#define INTEGRAL_COEFF 2

#define POSITION_SENSOR_THRESHOLD_CW 450
#define POSITION_SENSOR_THRESHOLD_CCW 450

// Debugging definitions
//#define DEBUG
#define DEBUG_FREQ 5

