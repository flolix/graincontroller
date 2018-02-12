#include <si4735.h>
  // using hardware twi 
#include <i2cmaster.h>
#include <uart.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <stdbool.h>
#include <stdarg.h>

#define UART_BAUD_RATE 38400  // Baudrate


#define PHASE_A     (PIND & 1<<PD3)
#define PHASE_B     (PIND & 1<<PD4)

volatile int8_t enc_delta;          // -128 ... 127
static int8_t last;

//using timer2
void encode_init( void )
{
  int8_t new;

  new = 0;
  //if( PHASE_A ) new = 3;
  //if( PHASE_B ) new ^= 1;       // convert gray to binary
  last = new;                   // power on state
  enc_delta = 0;
  TCCR2A |= (1<<WGM21);
  TCCR2B |= (1<<CS22);   // CTC, prescaler 64
  OCR2A = 124;
  TIMSK2 |= 1<<OCIE2A;
}

ISR( TIMER2_COMPA_vect )            // 1ms for manual movement
{
  int8_t new, diff;

  new = 0;
  if( PHASE_A ) new = 3;
  if( PHASE_B ) new ^= 1;          // convert gray to binary
  diff = last - new;               // difference last - new
  if( diff & 1 ) {                 // bit 0 = value (1)
    last = new;                    // store new as next last
    enc_delta += (diff & 2) - 1;   // bit 1 = direction (+/-)
  }
}



int8_t encode_read(void)         // read single step encoders
{
  int16_t val;

  cli();
  val = enc_delta;
  if (val < 10 && val > -10) val = 0; else enc_delta = 0;
 
  sei();
  return val / 10;                   // counts since last call
}

void stop_encoder(void) {
    enc_delta = 0;
    TIMSK2 &= ~(1<<OCIE2A);
}


uint16_t position2freq(int32_t pos) {
    return (pos / 1) + 8880;
}

int32_t freq2position(uint16_t fre) {
    return (fre - 8880) * 1;
}


uint8_t get_command(char * buf) {
    uart_puts("in get command\n");
    uint8_t command = 0xfe;
        if (memcmp(buf, "/radio/frequency?", 17) == 0) {
            command = 4;
        } else if (memcmp(buf, "/radio/frequency", 16) == 0) {
            command = 2;
        } else if (memcmp(buf, "/ping", 5) == 0) {
            command = 1;
        } else if (memcmp(buf, "/midi", 5) == 0) {
            command = 3;
        }

    return command;
} 


enum OSC_STATE {buf_empty, rec_command, is_paramlist_there, rec_paramlist, rec_param};

static char buf[256];
static uint8_t i;
static uint8_t p;
static uint8_t paramliststart;

uint8_t parsechar(char c) {
    static enum OSC_STATE osc_state = buf_empty;
    uint8_t done = 0;
    static uint8_t command;

    buf[i] = c;
    i++;

    if (osc_state == buf_empty) {
        osc_state = rec_command;
        buf[0] = c;
        i = 1;
        p = 0;
        command = 0;
    } else if (osc_state == rec_command) {
        if ( c == 0xa ) {
            osc_state = buf_empty;
            done = get_command(buf) | 0b01000000;
            // kein osc commando..
        } else if (c == '\0' && (i & 0b11) == 0) {
            //parameters?
            osc_state = is_paramlist_there;
            command = get_command(buf);
       }
    } else if (osc_state == is_paramlist_there) {
        if (c == ',') {
            //firstparam
            osc_state = rec_paramlist;
            paramliststart = i;
        } else {
            osc_state = buf_empty;
            done = 0xff; //missing paramlist
        }
    } else if (osc_state == rec_paramlist) {
        if (c == '\0' && (i & 0b11) == 0) {
            if (buf[paramliststart] != '\0') osc_state = rec_param; 
            else {
                done = command;
                osc_state = buf_empty;
            }
        } 
    } else if (osc_state == rec_param) {
        if ((i & 0b11) == 0) {
            //params[p] =  buf[i -4];
            p++;
            if (buf[paramliststart +p ] != '\0') osc_state =rec_param;
            else {
                uart_puts("sc_done\n");
                done = command;
                osc_state = buf_empty;
            }
        }
    } 

    return done;
}

#include "suart.h"
#define STXD    SBIT( PORTB, PB1)

void midi_putc(char c) {
    uputchar(c);
}


void init_stepper(void) {
    DDRC |= 0b1111;
    DDRB |= (1<<PB5);
    PORTB &= ~(1<<PB5);
    TCCR0A |= (1<<WGM01);
    TCCR0B |= (1<<CS02);
    OCR0A = 200;
}

volatile int8_t direction;

volatile int32_t position = 0;
volatile int32_t endpos = 0;


void turn(uint16_t freq) {
    char buffer[10];

    uart_puts("heretoo\n");
    cli();
    endpos = freq2position(freq);
    stop_encoder();
    if (endpos < position) direction = -1; else direction = 1;
    if (direction == -1) uart_puts("-1 \n");
    sei();

    uart_puts("endpos");

    utoa(endpos, buffer,  10);
    uart_puts(buffer);
    
    uart_puts("position");

    utoa(position, buffer, 10);
    uart_puts(buffer);

    uart_putc('\n');

    PORTB |= (1<< PD5);
    TIMSK0 |= 1<<OCIE0A;
}

uint8_t anzahlnulls (uint8_t i) {
    return  4 - (i & 0b11);
}


void createOSCMessage(char * command, char * paramlist, ...) {
    char buffer[10];
    va_list argumente;
    uint8_t anzahl, commandlength, i, an;
    int32_t intarg;
    va_start(argumente, paramlist);
    anzahl = strlen(paramlist);
    
    char buf[256];
    commandlength = strlen(command);
    strcpy(buf, command);
    an = anzahlnulls(commandlength);
    for (i = 0 ; i < an; i++) buf[commandlength+i] = '\0'; 
    commandlength += an;
    buf[commandlength] = ',';
    commandlength++;
    strcpy(&buf[commandlength], paramlist);
    commandlength += anzahl;
    an = anzahlnulls(anzahl+1);
    for (i = 0; i< an; i++) buf[commandlength+ i] = '\0';
    commandlength += an; 
    int8_t j;
    for (i = 0; i< anzahl; i++) {
        if (paramlist[i] == 'i') {
            //uart_puts("Int Arg found.. \n");
            intarg =  va_arg(argumente, int32_t);
            char * test = (char *) &intarg;
            for (j = 3; j >= 0; j--) {
                //uart_putc( * (test+j));
                buf[commandlength] = * (test+j);
                commandlength++;
            }

        }
    }
    va_end(argumente);
    //commandlength += 4;
    //uart_puts("OSC Generierung..");



    //uart_putc('\n');
/*
    utoa(buffer, i, 10);
    uart_puts(buffer);
    uart_putc('\n');
*/
    for (i = 0; i < commandlength; i++) uart_putc(buf[i]);
    uart_putc('\n');
}


void setandsendfreq(uint16_t freq) {
    //getfreq();
    //_delay_ms(100);
    //setfreq(freq);
    //_delay_ms(100);
    //freq = getfreq();
    //createOSCMessage("/radio/frequency", "i" , (int32_t) freq);
    uart_puts("after osc\n");
}


uint8_t steps[4] = {0b1010, 0b1001, 0b0101, 0b0110};

ISR (TIMER0_COMPA_vect) {
    static uint8_t presc = 0;
    static uint8_t step = 0;

    if (position == endpos) {
    
uart_puts("thesame\n");

        TIMSK0 &= ~(1<<OCIE0A);
        PORTB &= ~(1 << PB5);
        direction = 0; 
        //getfreq();
        //_delay_ms(100);
        //setandsendfreq(position2freq(position));
        presc = 0;
        //encode_init();
    }

 else {

        PORTC = steps[step];
        step += direction;
        if (step == 255) step = 3;
        if (step > 3) step = 0;
        position += direction;

    //teiler

        presc++;
        if (presc == 255) {
            presc = 0;
            if (direction == +1) {
                //seekup();
                //uart_puts("seek up\n");
            } else {
                //seekdown();
                //uart_puts("seek down\n");
            }
        }
    }

}


int main(void)
{
    unsigned int c;
    char buffer[256];
    uint8_t received;

    uint16_t freq;

    int16_t enc;
    uint8_t result;
    //cli();
    init_stepper();

    uart_init (UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU) );

    //sei();
    uart_puts("radiosands controller V0.3\n");

    _delay_ms(300);
    i2c_init();                             // initialize I2C library
    powerup();    
    _delay_ms(800);
    setfreq(8880);
    //seekup();
    _delay_ms(500);
    freq = getfreq();
    position = freq2position(freq);
    endpos = position;
	
    uint8_t j;

    //encode_init();

    suart_init();

    sei();

    for (;;) {
        enc = encode_read();

	if (enc != 0) {
                position += 10 * enc;
		//freq += 10 * enc;
		enc = 0;
                freq = position2freq(position);
		setfreq(freq);
                uart_puts("/radio/frequency ");
		utoa(freq, buffer, 10);
		uart_puts(buffer);
                uart_puts(" MHz");
		uart_putc('\n');
	}
	
	c = uart_getc();
	if (c & UART_NO_DATA) {
	} else {
		received = (uint8_t) c & 0xff;

                uart_putc(received);
                uart_putc('\n');

                result = parsechar(received);
                //result == 0;

                if (result != 0) {
                    uart_puts("command ");
                    utoa(result, buffer, 10);
                    uart_puts(buffer);
                    uart_puts(" received\n");
                }

                if (result == (2 | 0b01000000)) {
                    stop_encoder();
                    char * dot;

                    freq = atoi(&buf[16]) * 100; 
                    dot = strchr(&buf[17], '.');
                    freq += atoi(dot+1);

                    turn( freq);

                    utoa(freq, buffer,10);
                    uart_puts("set frequency");
                    uart_puts(buffer);
                    uart_putc('\n');
                    //setfreq(freq);
                } else if (result == 2) {
                    uart_puts("here\n");
                    //stop_encoder();
                    freq = (uint16_t) (buf[26] << 8);
                    freq += buf[27];
                    turn(freq);
                    //setfreq(freq);
                } else if (result == 1 ) {
                    //memcpy (buf , "/ping\0\0\0,\0\0\0\n", 13);
                    //for (i = 0;i < 13; i++) uart_putc(buf[i]);
                    //uart_puts("received osc /ping\n");
                    createOSCMessage("/ping", "");
                } else if (result == (1 | 0b01000000)) {
                    uart_puts("/ping\n");
                } else if (result == (3 | 0b01000000)) {
                    char * space1;
                    char * space2;
                    uint8_t byte = 0;
                    uart_puts("Sending MIDI Message: ");
                    byte = atoi(&buf[5]);
                    utoa(byte, buffer, 10);
                    uart_puts(buffer);
                    uart_putc(',');
                    midi_putc (byte);
                    space1 = strchr(&buf[6], ' ');
                    byte = atoi(space1);
                    utoa(byte, buffer, 10);
                    uart_puts(buffer);
                    uart_putc(','); 
                    midi_putc (byte);
                    space2 = strchr(space1+1,' ');
                    byte = atoi(space2);
                    utoa(byte, buffer, 10);
                    uart_puts(buffer);
                    midi_putc (byte);
                    uart_putc('\n');
                    
                } else if (result == 3) {
                    midi_putc(buf[19]);
                    midi_putc(buf[19+4]);
                    midi_putc(buf[19+8]); 
                    /*
                    uart_puts("MIDI");
                    uart_putc(buf[19]);
                    uart_putc(buf[19+4]);
                    uart_putc(buf[19+8]);
                    */
                } else if (result == (4 | 0b01000000)) {
                    freq = getfreq();
                    uart_puts("/radio/frequency ");
                    utoa(freq, buffer,10);
                    uart_puts(buffer);
                    uart_puts(" MHz");
                    uart_putc('\n');
                } else if (result == 4) {
                    freq = getfreq();
                    createOSCMessage("/radio/frequency","i", (int32_t) freq);
                    //memcpy (buf, "/radio/frequency\0\0\0\0,i\0\0\0\0\0\0\n",29); 
                    //buf[26] = (uint8_t) (freq >> 8);
                    //buf[27] = (uint8_t) (freq & 0xff);
                    //for (i = 0; i< 29; i++) uart_putc(buf[i]); 
                }
                    /* 
                    uart_puts("freq\n");
                    uart_putc( (uint8_t) (freq >> 8));
                    uart_putc( (uint8_t) (freq & 0xff));
                    uart_puts("ECHO\n");
                    uart_putc(result);
                    for (j = 0; j < i; j++) {
                        uart_putc(buf[j]);
                    }
                    uart_puts("DONE\n");
                    */
	}

   }



}
