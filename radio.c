/*
* Main file..
*
* project: radiosands
* target: AtMega328
*
* coded by guistlerei
*/
#include "light_ws2812.h"
#include "si4735.h"
  // using hardware twi 
#include "i2cmaster.h"
#include "uart.h"
#include "debug.h"

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

//uint8_t DEBUG = true;
uint8_t DEBUG = false;

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
    enc_delta -= (diff & 2) - 1;   // bit 1 = direction (+/-)
  }
}



int8_t encode_read(void)         // read single step encoders
{
  int16_t val;

  cli();
  val = enc_delta;
  //if (val < 10 && val > -10) val = 0; else enc_delta = 0;
    enc_delta = 0; 
  sei();
  //return val / 10;                   // counts since last call
    return val;
}

void stop_encoder(void) {
    cli();
    TIMSK2 &= ~(1<<OCIE2A);
    sei();
    enc_delta = 0;
}


uint16_t position2freq(int32_t pos) {
    return (pos / 0.75) + 8590;
}

int32_t freq2position(uint16_t fre) {
    return (fre - 8590) * 0.75;
}


uint8_t get_command(char * buf) {
    debputs(DEBUG, "in get command");
    uint8_t command = 0xfe;
        if (memcmp(buf, "/radio/frequency?", 17) == 0) {
            command = 4;
        } else if (memcmp(buf, "/radio/frequency", 16) == 0) {
            command = 2;
        } else if (memcmp(buf, "/ping", 5) == 0) {
            command = 1;
        } else if (memcmp(buf, "/midi", 5) == 0) {
            command = 3;
        } else if (memcmp(buf, "/debug/on",9) == 0) {
            command = 5;
        } else if (memcmp(buf, "/debug/off", 10) == 0) {
            command = 6;
        } else if (memcmp(buf, "/radio/power/up", 15) == 0) {
            command = 7;
        } else if (memcmp(buf, "/radio/power/down", 17) == 0) {
            command = 8;
        } else if (memcmp(buf, "/radio/seek/up", 14) == 0) {
            command = 9;
        } else if (memcmp(buf, "/radio/seek/down", 16) == 0) {
            command = 10;
        }  else if (memcmp(buf, "/leds", 5) == 0) {
            command = 20; 
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
            if (buf[paramliststart +p ] != '\0') osc_state = rec_param;
            else {
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
    DDRB |= (1<<PB5) | (1 << PB4);
    PORTB &= ~(1<<PB5);
    PORTB &= ~(1<<PB4);
    TCCR0A |= (1<<WGM01);
    //prescaler 256
    TCCR0B |= (1<<CS02);

    TCCR0B |= (1<<CS00);    //damit prescaler 1024
    //OCR0A = 255; // bisher
    OCR0A = 90; //speedtest;
}

volatile int8_t direction;

volatile int16_t position = 0;
volatile int16_t endpos = 0;

void stepenable(uint8_t en) {
    uint8_t portb = PORTB & 0b11001111; 
    PORTB = portb | (en << 4);
}

enum {STOPPED, RAMP_UP, RAMP_DOWN, RUNNING};
volatile uint8_t stepper_state = STOPPED;
volatile uint8_t speed = 0;

void turn(uint16_t freq) {
    stepper_state = RAMP_UP;
    speed = 0;
    debputs(DEBUG, "in turn");
    //_delay_ms(100);
    cli();
    endpos = freq2position(freq);
    stop_encoder();
    if (endpos < position) direction = -1; else direction = 1;
    //if (direction == -1) uart_puts("-1 \n");
    sei();
    //debprintf(DEBUG, "direction %i", direction);
    //debprintf(DEBUG, "endpos %i, position %i", endpos, position);
    //_delay_ms(100);
    //PORTB |= (1<< PD5);
    cli();
    stepenable(0b11);
    TIMSK0 |= 1<<OCIE0A;
    sei();
}



uint8_t anzahlnulls (uint8_t i) {
    return  4 - (uint8_t) (i & 0b11);
}


void createOSCMessage(char * command, char * paramlist, ...) {
    va_list argumente;
    uint8_t anzahl, commandlength, i, an;
    int32_t intarg;
    va_start(argumente, paramlist);
    anzahl = strlen(paramlist);
    
    char buf[128];
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
    getfreq();  // only here to stop seeking, if it does..
    _delay_ms(100);
    setfreq(freq);
    _delay_ms(100);
    freq = getfreq();
    createOSCMessage("/radio/frequency", "i" , (int32_t) freq);
    debputs(DEBUG, "after osc");
}
//                     1     0        3        2
//                    AaBb
//uint8_t steps[4] = {0b1010, 0b1001, 0b0101, 0b0110};
//                    2       1       0        3
uint8_t steps[4] = { 0b0110,0b1010,0b1001,  0b0101};
//uint8_t steps[8] = {0b1001, 0b1000, 0b1010, 0b0010, 0b0110,0b0100, 0b0101, 0b0001};
//uint8_t steps_enable[8] = {0b11, 0b10, 0b11, 0b01, 0b11, 0b10, 0b11, 0b01};
uint8_t steps_enable[8] = {0b11, 0b01, 0b11, 0b10, 0b11, 0b01, 0b11, 0b10};

//543210
//000011
//110000


volatile uint8_t RADIO_THERE = 0;

void stepper_stop(void) {
    TIMSK0 &= ~(1<<OCIE0A);
    direction =0 ;
    speed = 0;
    _delay_ms(10);
    if (RADIO_THERE) setandsendfreq(position2freq(position));
    stepenable(0);
    encode_init();
}

uint16_t runden(uint16_t v) {
    uint8_t v8 =  v % 10;
    if (v8 <=5 ) return v - v8;
    else return v+ (10-v8);    
}



//uint8_t speedtable [4] = {255, 110, 90};
uint8_t speedtable [4] = {200, 90,25};

ISR (TIMER0_COMPA_vect) {
    static uint8_t presc = 0;
    static uint8_t step = 0;
    static uint8_t speed_presc = 0;

    speed_presc++;
    if (speed_presc == 5) speed_presc = 0;

    if (stepper_state == RAMP_UP) {
        if (speed_presc == 0) {
            OCR0A = speedtable[speed];
            speed++;
            if (speed >= 3) stepper_state = RUNNING;
        }
    } else if (stepper_state == RAMP_DOWN) {
        stepper_stop();
        stepper_state = STOPPED;
        return;
    }

    if (position == endpos) {
        debprintf(DEBUG,"reached %i", position);
        stepper_state = RAMP_DOWN;
        OCR0A = 255;
        //stepper_stop();
        //TIMSK0 &= ~(1<<OCIE0A);
        //PORTB &= ~(1 << PB5);
        //stepenable(0);
        //direction = 0; 
        //getfreq();
        //_delay_ms(100);
        presc = 0;
        return;
    } 

    //OCR0A = 255; // bisher
    //else if (steeper_state == RAMP_DOWN) {
    //    speed--;
    //    if (speed == 0) stepper_state =STOPPED;
    //} 
    
    PORTC = steps[step];
    //stepenable(steps_enable[step]);
    step += direction;
    if (step == 255) step = 3;
    if (step > 3) step = 0;
    //if (step > 7) step = 0;
    position += direction;

    //teiler
    if (!RADIO_THERE) return;

    presc++;
    if (presc == 200) {
        setfreq(runden(position2freq(position)));
        presc = 0;
    /*
        if (direction == +1) {
            seekup();
            //uart_puts("seek up\n");
        } else {
            seekdown();
            //uart_puts("seek down\n");
        }
    */
    }
}


int main(void) {
    uint8_t j;
    ws2812_setleds_rgb(0,0,0,18);
    ws2812_setleds_rgb(20, 20, 20, 1);

    unsigned int c;
    char buffer[256];
    uint8_t received;

    uint16_t freq;

    int16_t enc;
    uint8_t result;
    init_stepper();
    //stepper_move_left();
    position = freq2position(10800);
    turn(0); //will never reach..
    _delay_ms(3000);
    stepper_stop(); 
    _delay_ms(100);
    position = freq2position(8590); //ganze linke home position
    turn(8750);
    ws2812_setleds_rgb(20, 20, 20, 5);

    cli();
    uart_init (UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU) );
    i2c_init();                             // initialize I2C library
    PORTC |= (1<< PC4) | (1 << PC5);        // using the internal pull ups!
    sei();

    uart_puts("radiosands controller V0.3\n");
    debputs(DEBUG, ">>> DEBUG MODE ON <<<");
    debputs(DEBUG, ">>> /debug/off to turn it off");

    _delay_ms(300);
    ws2812_setleds_rgb(20, 20, 20, 8);
    uint8_t erg = powerup();
    debprintf(DEBUG, "powerup return %i", erg);
    if (erg == 1) {    
        RADIO_THERE = 1;
        _delay_ms(800);
        //setfreq(8880);
        //freq = getfreq();
        //endpos = freq2position(freq);
        turn(8880);
        freq = 8880;
    } else {
        debputs(DEBUG, "ERROR: Could not connect to radio chip.");
        ws2812_setleds_rgb(50, 0, 0, 9);
        for (;;);
    }

    encode_init();

    cli();
    suart_init();
    sei();

    ws2812_setleds_rgb(20, 20, 20, 18);
    for (;;) {

        enc = encode_read();

	if (enc != 0) {

                cli();
                //position += 10 * enc;
                position += enc;
		//freq += 10 * enc;
                int8_t diff = runden(position2freq(position)) - freq;
                if (abs(diff) != 0) {
                    freq += diff;
                    sei();

                    enc = 0;
                    setfreq(freq);
                    uart_puts("/radio/frequency ");
                    utoa(freq, buffer, 10);
                    uart_puts(buffer);
                    uart_puts(" MHz");
                    uart_putc('\n');
                }
	}
	
	c = uart_getc();
	if (c & UART_NO_DATA) {
	} else {
		received = (uint8_t) c & 0xff;

                result = parsechar(received);

                if (result != 0) {
                    debprintf(DEBUG, "command %i received", result);
                    //_delay_ms(100);
                }

                if (result == 20) {
                    ws2812_setleds_rgb(buf[19], buf[19+4], buf[19+8], 18);
                } else if (result == (20 | 0b01000000)) {
                    char * space1;
                    char * space2;
                    uint8_t byte1 = 0, byte2 =0, byte3 =0;

                    byte1 = atoi(&buf[5]);
                    space1 = strchr(&buf[6], ' ');
                    byte2 = atoi(space1);
                    space2 = strchr(space1+1,' ');
                    byte3 = atoi(space2);
                    ws2812_setleds_rgb(byte1, byte2,byte3, 18);
                    debprintf(DEBUG, "Setting LEDS to : %i, %i, %i", byte1, byte2, byte3);
                } else if ((result & 5)== 5 ) { 
                    DEBUG = true;
                    debputs(DEBUG, ">>> DEBUG MODE ON <<<");
                    //createOSCMessage("/debug/on", "");
                } else if ((result & 6) == 6) {
                    debputs(DEBUG, ">>> DEBUG MODE OFF <<<");
                    DEBUG = false;
                    //createOSCMessage("/debug/off", "");
                } else if (result == 7 ) {
                    //power up
                    powerup();
                    //_delay_ms(100);
                    setandsendfreq(position2freq(position));
                } else if (result == 8) {
                    //power down
                    powerdown();
                } else if ((result & 9) == 9) {
                    //seek up
                    getfreq(); //stop running seek..
                    seekup();
                } else if ((result & 10) == 10) {
                    //seek down
                    getfreq(); //stop running seek..
                    seekdown();
                } else if (result == (2 | 0b01000000)) {
                    //stop_encoder();
                    //char * dot;
                    //freq = atoi(&buf[16]) * 100; 
                    //dot = strchr(&buf[17], '.');
                    //freq += atoi(dot+1);
                    if (stepper_state != STOPPED) {
                        stepper_state = RAMP_DOWN;
                        _delay_ms(100);
                    }
                    freq = atoi(&buf[16]); 
                    turn( freq);
                    utoa(freq, buffer,10);
                    uart_puts("set frequency");
                    uart_puts(buffer);
                    uart_putc('\n');
                    //setfreq(freq);
                } else if (result == 2) {
                    //stop_encoder();
                    if (stepper_state != STOPPED) {
                        stepper_state = RAMP_DOWN;
                        _delay_ms(100);
                    } 
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
                    uint8_t byte1 = 0, byte2 =0, byte3 =0;

                    byte1 = atoi(&buf[5]);
                    space1 = strchr(&buf[6], ' ');
                    byte2 = atoi(space1);
                    space2 = strchr(space1+1,' ');
                    byte3 = atoi(space2);

                    midi_putc (byte1);
                    midi_putc (byte2);
                    midi_putc (byte3);

                    debprintf(DEBUG, "Forwarding MIDI Message to axoloti: (%i, %i, %i)", byte1, byte2, byte3);
                } else if (result == 3) {
                    midi_putc(buf[19]);
                    midi_putc(buf[19+4]);
                    midi_putc(buf[19+8]); 
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
                }
       	}
   }
}
