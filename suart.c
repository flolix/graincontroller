/************************************************************************/
/*                                                                      */
/*                      Software UART with FIFO, using T1               */
/*                                                                      */
/*              Author: P. Dannegger                                    */
/*                                                                      */
/************************************************************************/
//				Target: ATtiny44
#include <avr/interrupt.h>
#include "suart.h"

#define	BAUD	31250

#define BIT_TIME	(uint16_t)(F_CPU * 1.0 / BAUD + 0.5)

#define	TX_HIGH		((1<<COM1A1) | (1<<COM1A0))
#define	TX_LOW		(TX_HIGH ^ (1<<COM1A0))
#define	TX_OUT		TCCR1A		// use compare output mode

#define ROLLOVER( x, max )	x = ++x >= max ? 0 : x
					// count up and wrap around

struct bits {
  uint8_t b0:1;
  uint8_t b1:1;
  uint8_t b2:1;
  uint8_t b3:1;
  uint8_t b4:1;
  uint8_t b5:1;
  uint8_t b6:1;
  uint8_t b7:1;
} __attribute__((__packed__));


#define SBIT_(port,pin) ((*(volatile struct bits*)&port).b##pin)
#define	SBIT(x,y)	SBIT_(x,y)


#define	STXD_DDR	SBIT( DDRB,  PB1 )

#define vu8(x)  (*(volatile uint8_t *)&(x))

static uint8_t stx_buff[STX_SIZE];
static uint8_t stx_in;
static uint8_t stx_out;
static uint8_t stx_data;
static uint8_t stx_state;



void suart_init( void )
{
  OCR1A = BIT_TIME - 1;
  TCCR1A = TX_HIGH;			// set OC1A high, T1 mode 4
  TCCR1B = (1<<WGM12) | (1<<CS10);	// noise canceler, 1-0 transition,
					// CLK/1, T1 mode 4 (CTC)
  TCCR1C = 1<<FOC1A;
  stx_state = 0;
  stx_in = 0;
  stx_out = 0;
  STXD_DDR = 1;				// output enable
  //TIFR1 = 1<<ICF1;			// clear pending interrupt
  TIMSK1 = 1<<OCIE1A;		// enable tx and wait for start
}



void uputchar( uint8_t c )			// transmit byte
{
  uint8_t i = stx_in;

  ROLLOVER( i, STX_SIZE );
  stx_buff[stx_in] = ~c;		// complement for stop bit after data
  while( i == vu8(stx_out));		// until at least one byte free
					// stx_out modified by interrupt !
  stx_in = i;
}


void uputs_( uint8_t *s )			// transmit string from SRAM
{
  while( *s )
    uputchar( *s++ );
}

/******************************	Interrupts *******************************/

ISR( TIMER1_COMPA_vect )				// transmit data bits
{
  if( stx_state ){
    stx_state--;
    TX_OUT = TX_HIGH;
    if( stx_data & 1 )				// lsb first
      TX_OUT = TX_LOW;
    stx_data >>= 1;
    return;
  }
  if( stx_in != stx_out ){			// next byte to sent
    stx_data = stx_buff[stx_out];
    ROLLOVER( stx_out, STX_SIZE );
    stx_state = 9;				// 8 data bits + stop
    TX_OUT = TX_LOW;				// start bit
  }
}
