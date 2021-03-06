// Cpu frequency for util/delay.h
#define F_CPU 8000000

#define PWM_PRESCALER Atmega328p::PSV_1
#define PWM_INITIAL 0

#define ADC_PRESCALER Atmega328p::ADC_PSV_32

#define LOOP_SENSE_FREQ_LIMIT 20
#define LOOP_SENSE_FREQ_SPEED 100

#define AVG_WINDOW 4

// Smallest value that can be set for target emf. This value should be a little
// less that the smallest value that makes the motor move to maximize range of
// speed setting potentiometer.
#define TARGET_EMF_MINIMUM 180
// Scaler for dividing potentiometer reading to acquire target emf
// value.
#define TARGET_EMF_SCALER 12
// How many points of difference is needed change pwm by one step
#define CONTROL_P_COEFF 8
#define CONTROL_I_DIVISOR 32

// Minimum change of pi controller resolution change that is regocnized. The
// purpose of this value is to stop resetting of controller integral value
// due to variance in potentiometer reading.
#define CONTROL_TARGET_RES 3

#define POSITION_SENSOR_THRESHOLD_CW 450
#define POSITION_SENSOR_THRESHOLD_CCW 450

// Balancing differences of speed between different rotation directions.
#define OFFSET_CW 0

// Debugging definitions
//#define DEBUG
#define DEBUG_FREQ 5
