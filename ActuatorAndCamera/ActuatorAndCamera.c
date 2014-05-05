/*
 * ActuatorAndCamera.c
 *
 * Created: 4/7/2014 6:04:37 PM
 *  Author: Carlos Patiño
 *
 * This code produces an PWM output signal from Pin 3 of the Arduino Uno.
 * The PWM duty-cycle is first set at a user-defined maximum value, and is progressively decreased
 * up to a user-defined minimum value. The speed at which this ramp-down occurs can be adjusted.
 *
 * Once the duty-cycle ramp-down is completed, this code will produce a single square wave pulse
 * from Pin 7, which provides a trigger signal to a camera. The camera is triggered on detection
 * of the falling edge of a square wave pulse (also called a TTL signal).
 *
 * Unless specified otherwise, duty-cycle is always expressed as a percentage and not as a fraction.
 *
 * List of user-defined inputs:
 *		MAX_DUTYCYCLE
 *		MIN_DUTYCYCLE
 *		NUM_DUTYCYCLES
 *		DELAY
 *
 * Note that the linear solenoid being driven is an intermittent-duty model with a maximum "on" time of 
 * 1 minute. Nominal duty cycle is 25%, presumably meaning that for every minute that the solenoid is 
 * fully "on", it must be 3 minutes fully "off". However, if the "on" and "off" cycle is fast enough 
 * (currently set at ~1000 Hz) and the total time interval through which the solenoid is being powered 
 * is not excessive, duty cycles up to 100% can be safely applied to the solenoid.
 */ 

#define F_CPU ((unsigned long) 16000000) // Define CPU frequency as 16MHz.
#include <avr/io.h> // Include basic AVR input/output functions.
#include <avr/interrupt.h> // Include package for interrupts
#include <util/delay.h> // Include delay package to use _delay_ms() function.

#include "serial.h" // Include ability to print to serial terminal.

#include <stdlib.h> // Include package that defines several general functions.
#include <assert.h> // Include assert() function to catch user-defined input errors.

// Define maximum and minimum duty cycles to be applied. Must be an integer.
#define MAX_DUTYCYCLE 63
#define MIN_DUTYCYCLE 25

/* Define number of distinct duty cycles to be applied. The duty cycle will be decreased progressively
from MAX_DUTYCYCLE to MIN_DUTYCYCLE this many steps. Must be declared as an integer. 
NUM_DUTYCYCLES >= 2 */
#define NUM_DUTYCYCLES 200

/* Define, in milliseconds (ms), how much to delay each successive decrease of the Pin 3 output duty cycle.
A higher value indicates a greater delay in between each duty cycle decrease, and thus a longer time
to reach MIN_DUTYCYCLE. */
#define DELAY 5

// Include method to set up PWM output from Pin 3
void initPWM(void);

// Include method to set the specific duty cycle for Pin 3 output
void setDutyCycle(int);

int main(void)
{	
	// Initialize serial communication (for printing to serial terminal). Used for testing.
	init_uart(); 
	
	/* 
	Initialize the pins, timers, and registers necessary to run a Pulse-Width Modulated (PWM) output
	from Pin 3.
	*/
	initPWM();
	
	/* 
	Initialize an array of duty cycles which will be sequentially set as the duty cycle of the 
	Pin 3 output. The 1st element of dutyCycleArray is maxDutyCycle. The last element of dutyCycleArray 
	is minDutyCycle. Each element of dutyCycleArray in between its first and last element are equally 
	spaced apart.
	NOTE: To avoid using values of type double or float, and still maintain a greater number of significant 
	figures, all elements of dutyCycleArray will be multiplied by 100. 
	*/
	// Initialize array. All elements default to 0.
	int dutyCycleArray[NUM_DUTYCYCLES]; 
	
	// Set first element of dutyCycleArray to MAX_DUTYCYCLE.
	// Reminder: All elements in dutyCycleArray are multiplied by 100 to avoid using values of type double.
	dutyCycleArray[0] =  MAX_DUTYCYCLE*100;
		
	// Set the difference between each element of dutyCycleArray
	int diff= ((MAX_DUTYCYCLE*100) - (MIN_DUTYCYCLE*100))/(NUM_DUTYCYCLES - 1);
	
	// Initialize each element of dutyCycleArray separately.
	for (int i= 1; i < NUM_DUTYCYCLES; i++)	{
		dutyCycleArray[i]= (dutyCycleArray[i-1] - diff);
	}
	
	// Initially, set Pin 3 output duty cycle to maxDutyCycle (for maximum spring compression)
	setDutyCycle(MAX_DUTYCYCLE);
	
	// isDutyCycleMax is true when the duty cycle of Pin 3 is at maxDutyCycle, and false otherwise.
	int isDutyCycleMax= 1;
	
    while(1)
    {
		/* Check if the Pin 3 output duty cycle is high. If true, progressively 
		lower the duty cycle of Pin 3 from its maxDutyCycle to minDutyCycle */
		if (isDutyCycleMax) {
			
			// Update isDutyCycleHigh to reflect that Pin 3 output duty cycle will be decreased.
			isDutyCycleMax= 0;
			
			/* Set each element of dutyCycleArray as the duty cycle of Pin 3 output.
			The speed at which the duty cycle of Pin 3 is decreased is determined by the
			time delay in between each loop iteration, as defined by the user in DELAY. */
			for (int j = 0; j < NUM_DUTYCYCLES; j++) {
				
				/* All elements in dutyCycleArray were multiplied by 100 to avoid using values of type double.
				These elements are then divided by 100 and cast explicitly to type integer so that the input to
				setDutyCycle() is a duty cycle expressed as a percentage between 0 and 100 */
				setDutyCycle((int) dutyCycleArray[j]/100);
				_delay_ms(DELAY);
				
				/* Printing statements for testing purposes. */
				// printf("The current duty cycle is %d\n", (int) dutyCycleArray[j]/100);
				// _delay_ms(15);
				//printf("The current output register value is %d\n", OCR2B);
				//_delay_ms(15);
			}
			
			/* Provide a low TTL pulse (a square wave between 0-5V) to the camera to trigger it. */
			DDRD |= (1<<PIND7); // Pin 7 (PD7) is output
			PORTD |= (1<<PIND7); // Set Pin 7 to high.
			_delay_us(50); // Maintain Pin 7 at high for 50 microseconds.
			PORTD &= ~(1<<PIND7); // Set Pin 7 to low.
			
		}
		
		/* Printing statement for testing purposes. */
		// printf("The current output register value is %d\n", OCR2B);
		// _delay_ms(1000);

    }
}

/* 
* Initializes the pins, timers, and registers necessary to run a Pulse-Width Modulated (PWM)
* signal from Pin 3 of the Arduino Uno.
* Note that the actual duty cycle of the output from Pin 3 is NOT set in initPWM().
* The duty cycle of Pin 3 output is set in method setDutyCycle().
*/
void initPWM(void)
{
	/*
	The ATmega328P (the microcontroller on the Arduino Uno) has three timers known as Timer 0, Timer 1, 
	and Timer 2. Each timer has two output compare registers that control the PWM width for the timer's 
	two outputs: when the timer reaches the compare register value, the corresponding output is toggled.
	
	The timer can either run from 0 to 255, or from 0 to a fixed value. 
	The 16-bit Timer 1 has additional modes to supports timer values up to 16 bits. 
	Only Timer 2 will be used here, so the timer will take a value from 0 to 255 only.
	
	The Timer/Counter Control Registers TCCRnA and TCCRnB hold the main control bits for the timer.
	The character 'n' in the abbreviation for these registers stands for either 0, 1, or 2, depending
	on whether Timer 0, 1, or 2 is being used, respectively. These registers hold several groups of bits:

	- Waveform Generation Mode bits (WGM): these control the overall mode of the timer.
		(These bits are split between TCCRnA and TCCRnB.)
	- Clock Select bits (CS): these control the clock prescaler
	- Compare Match Output A Mode bits (COMnA): these enable/disable/invert output A
	- Compare Match Output B Mode bits (COMnB): these enable/disable/invert output B
	
	The Output Compare Registers OCRnA and OCRnB set the levels at which outputs A and B will be affected.
	
	The main PWM modes are "Fast PWM" and "Phase-correct PWM". This code will use Fast PWM, as it is the simplest.
	Note that in Fast PWM, both of the outputs have the same frequency.
	*/
	
	// Set pins 3 and 11 (Timer 2) as output 
	DDRD |= (1<<PIND3); // Pin 3 (PD3) is output
	DDRB |= (1<<PINB3); // Pin 11 (PB3) is output (note: PIN 11 is not being used to control solenoid. Only Pin 3 is used).
	
	// Clear the timer control register for Timer 2 (probably not necessary, should be default 0 anyway).
	TCCR2A= 0;
	TCCR2B= 0;
	
	// Set WGM bits on TCCR2A to 11 to select fast PWM.
	TCCR2A |= (1<<WGM21)|(1<<WGM20);
	
	// Set the COM2A bits and COM2B bits to 10 to provide non-inverted PWM for outputs A and B.
	TCCR2A |= (1<<COM2A1)|(1<<COM2B1);
	
	/* Set the CS bits to 100 to set the prescaler to divide the clock by 64.
	For prescaler of 64, the output frequency is 16 MHz / 64 / 256 = 976.5625Hz */
	TCCR2B |= (1<<CS22);	
	
}

/*
* Sets the duty cycle of the output from Pin 3.
* Input dutyCycle is the user-desired duty cycle as a percentage.
* Precondition: 0 <= dutyCycle <= 100.
*/
void setDutyCycle(int dutyCycle) {
	
	// Verify precondition.
	assert(0 <= dutyCycle);
	assert(dutyCycle <= 100);
	
	/* 
	Convert function input dutyCycle into the register value to pass onto the Output Compare Register in
	order to achieve the desired duty cycle output. A particular register value indicates a particular duty 
	cycle by the formula below:
	([register value] + 1) / 256 = duty cycle (as a fraction)
	*/
	int registerValue = (dutyCycle*256/100) - 1;
	
	// Set the Output Compare Registers for Timer 2 to set the desired duty cycle for each output.
	// Note that only Pin 3 is used to control solenoid. Pin 11 is set here only as an example.
	OCR2A = 154; // Output A (Pin 11) has a duty cycle of (154+1)/256 = 60.54% 
	OCR2B = registerValue; // Output B (Pin 3) has a duty cycle of (registerValue+1)/256.
	
}