// ***************************************
//	uart328P.c
// ***************************************

#include "uart328P.h"

// ***************************************
// **** Initialize UART ******************	
// ***************************************

//void init_UART( void ) {

void init_UART( unsigned long cpu_freq, unsigned long baudrate ) {

// calculate baud prescale value
	unsigned long baud_prescale = (((cpu_freq / (baudrate * 8UL))) - 1);	// Double speed mode calculation for UBRR

// **** Enable Rx and Tx *****************

	UCSR0B = (1 << RXEN0) | (1 << TXEN0)				// Enable Receive and Transmit
		| (0 << RXCIE0);								// Rx interrupt not enabled by default, use en_Rx() function

// **** Default frame format: 8data, 1 stop ****

// **** Set baud rate ********************

	UBRR0L = baud_prescale;									
	UBRR0H = (baud_prescale >> 8);

	UCSR0A |= (1 << U2X0);								// Set double baud rate
}

// ***************************************
// **** UART Transmit ********************	
// ***************************************

void Tx( unsigned char data ) {
	
// **** Wait for empty Tx buffer *********

	while( (UCSR0A & (1 << UDRE0)) == 0 ){};

// **** Put data in buffer, send data ****

	UDR0 = data;
}

// ***************************************
// **** UART Receive *********************	
// ***************************************

unsigned char Rx( void ) {

// **** Wait for data to be received *****

	while( (UCSR0A & (1 << RXC0)) == 0 ){};

// **** Return received data in buffer ***

	return UDR0;
}

void en_Rx( void ) {

	sei();												// Enable global interrupts
	UCSR0B |= (1 << RXCIE0);							// Enable Rx interrupt
}

void dis_Rx( void ) {

	UCSR0B &= ~(1 << RXCIE0);							// Disable Rx interrupt
}
