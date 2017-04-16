// ***************************************
//	uart328P.h
// ***************************************

#include <avr/io.h>
#include <avr/interrupt.h>

void init_UART( unsigned long, unsigned long );
void Tx( unsigned char data );
unsigned char Rx( void );
void en_Rx( void );
void dis_Rx( void );
