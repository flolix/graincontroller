/*
* include file for suart.c
* its a software uart transmitter. code is mostly "borrowed" by Peter Danneger
*
* project: radiosands
* target: AtMega328
*
* coded by guistlerei
*/

#ifndef _suart_h_
#define _suart_h_

			// size must be in range 2 .. 256
#define STX_SIZE	20

#define	uputs(x)	uputs_((uint8_t*)(x))	// avoid char warning


void suart_init( void );
void uputchar( uint8_t c );			// send byte
void uputs_( uint8_t *s );			// send string from SRAM


#endif
