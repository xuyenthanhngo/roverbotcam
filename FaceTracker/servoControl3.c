// servoControl3.c
//	
//		Basic code using an ATmega328P to establish two independent
//	16-bit PWM signals on OC1A and OC1B.  Four buttons are implemented
//	without interrupts to manually adjust servo positions.
//
//	Updates:	added servomotor macros
//				added commands for up-left, down-left, up-right, down-right
//
// Author(s):	Jeremy Greenwood

#define F_CPU 8000000UL										// 8 MHz

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart328P.h"

#define	STEP		2		// Define step-size for changing servo position
#define DELAY		8		// Define how many milliseconds to pause during pulse methods
#define PAN_MIN		350		// Define the four extreme servo positions to bound movement
#define PAN_MAX		1200
#define TILT_MIN	600
#define TILT_MAX	1000

// Outputs
#define PAN_SERVO	PB1
#define TILT_SERVO	PB2
#define TRANSMIT_X	PD1 	// IO macro assignment exception

// Inputs
#define RECEIVE_X			PD0
#define PAN_LEFT_BUTTON		PD4
#define PAN_RIGHT_BUTTON	PD5
#define TILT_DOWN_BUTTON	PD6
#define TILT_UP_BUTTON		PD7

// Macros for IO: B_reg -> outputs, D_reg -> inputs
#define off(bit) 	PORTB &= ~(1 << bit)					// Clear output bit
#define on(bit) 	PORTB |= (1 << bit)						// Set output bit
#define flip(bit) 	PORTB ^= (1 << bit)						// Toggle output bit
#define out(bit)	DDRB |= (1 << bit)						// Define output bit
#define in(bit)		DDRD |= (0 << bit)						// Define input bit
#define pullup(bit)	PORTD |= (1 << bit)						// Define pullup resistor for bit
#define get(bit) 	(PIND & (1 << bit))						// Read input bit (parenthesis necessary)

// Servomotor macros
#define move_left(step)		pan_duty -= step
#define move_right(step)	pan_duty += step
#define move_up(step)		tilt_duty += step
#define move_down(step)		tilt_duty -= step
#define limit_outputs()		if( pan_duty < PAN_MIN ) pan_duty = PAN_MIN;if( pan_duty > PAN_MAX ) pan_duty = PAN_MAX;if( tilt_duty > TILT_MAX ) tilt_duty = TILT_MAX;if( tilt_duty < TILT_MIN ) tilt_duty = TILT_MIN
#define pulse_pan(word)		OCR1A = word
#define pulse_tilt(word)	OCR1B = word

// Prototypes
void init_PWM( void );

volatile int	input = 0;

ISR( USART_RX_vect ) {

	input = Rx();
}

int main (void)
{
	init_UART( 8000000, 9600 );							// Pass CPU frequency and desired baud rate, respectively
	en_Rx();

	in( PAN_LEFT_BUTTON );									// Assign inputs
	in( PAN_RIGHT_BUTTON );
	in( TILT_DOWN_BUTTON );
	in( TILT_UP_BUTTON );
	in( RECEIVE_X );

	pullup( PAN_LEFT_BUTTON );								// Assign internal pullup resistors
	pullup( PAN_RIGHT_BUTTON );
	pullup( TILT_DOWN_BUTTON );
	pullup( TILT_UP_BUTTON );

	int pan_duty = 780, tilt_duty = 750;					// Assign initial servo positions

	init_PWM();

	pulse_pan( pan_duty );									// Set PWM duty cycle
	pulse_tilt( tilt_duty );

	out(PAN_SERVO);											// Assign outputs
	out(TILT_SERVO);
	DDRD |= (1 << TRANSMIT_X);

   	while(1) {

	//*********** ADDED CASES FOR DIAGONALS 1,3,7, AND 9 ************

		switch( input ) {

			case '1':					
				move_left(STEP);
			
			case '2':
				move_down(STEP);
				break;

			case '3':
				move_down(STEP);
				move_right(STEP);
				break;

			case '4':
				move_left(STEP);
				break;

			case '6':
				move_right(STEP);
				break;

			case '7':
				move_left(STEP);
				
			case '8':
				move_up(STEP);
				break;

			case '9':
				move_up(STEP);
				move_right(STEP);
		}
		input = 0;
	
		// process button presses
		if( !get( PAN_LEFT_BUTTON ) )
			move_left(STEP);
			
		if( !get( PAN_RIGHT_BUTTON ) )
			move_right(STEP);

		if( !get( TILT_DOWN_BUTTON ) )
			move_down(STEP);
			
		if( !get( TILT_UP_BUTTON ) )
			move_up(STEP);

		limit_outputs();
		pulse_pan( pan_duty );
		pulse_tilt( tilt_duty );
		_delay_ms( DELAY );								// Don't change pulse width faster than 60 Hz (16.7 ms)
	}
}


void init_PWM( void ) {

//	**** Set up PWM output on OC0B *******	

  	TCCR1A = (0 << WGM11) | (0 << WGM10)				// Configure timer mode to "Phase and frequency correct PWM" mode
		| (1 << COM1A1) | (0 << COM1A0)					// Clear OC1A/OC1B on Compare Match when upcounting.
		| (1 << COM1B1) | (0 << COM1B0);				// Set OC1A/OC1B on Compare Match when downcounting.

  	TCCR1B = (0 << CS12) | (1 << CS11) | (0 << CS10)	// Set timer prescaler to Fcpu/8 (table 15-5)
		| (0 << WGM12) | (1 << WGM13);					// Set to PWM mode 8 (table 15-4)

	ICR1 = 10000;										// Set ICR1 to 10,000 for a period of 20 ms
}

