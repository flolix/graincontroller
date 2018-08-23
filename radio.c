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
#include "version.h"
#include "OSC.h"
#include "soft_reset.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <stdbool.h>
#include <stdarg.h>

#define UART_BAUD_RATE 38400  // Baudrate

#define PHASE_A     (PIND & 1<<PD3)
#define PHASE_B     (PIND & 1<<PD4)


uint8_t DEBUG = 1;
//uint8_t DEBUG = false;

volatile int8_t enc_delta;          // -128 ... 127
static int8_t last;

volatile int16_t position = 0;
volatile uint16_t freq;

//using timer2
void encode_init( void ) {
  int8_t new;
  new = 0;
  last = new;                   // power on state
  enc_delta = 0;
  TCCR2A |= (1<<WGM21);
  TCCR2B |= (1<<CS22);   // CTC, prescaler 64
  OCR2A = 40;  // ca. 3 kHz
  TIMSK2 |= 1<<OCIE2A;
}

uint16_t runden(uint16_t v) {
    uint8_t v8 =  (uint8_t) (v % 10);
    if (v8 < 5 ) return v - v8;
    else return v + (10-v8);    
}

#define scalefactor  6.0

uint16_t position2freq(int32_t pos) {
    return (pos + (scalefactor * 8590)) / scalefactor;
}

int32_t freq2position(uint16_t fre) {
    return (fre - 8590) * scalefactor;
}


uint8_t volatile PLEASE_UPDATE_STATION = 0;

ISR( TIMER2_COMPA_vect )         
{
  int8_t new, diff;
  static uint8_t PRESC = 120;
  static uint8_t PRESC1 = 10;

  new = 0;
  if( PHASE_A ) new = 3;
  if( PHASE_B ) new ^= 1;          // convert gray to binary
  diff = last - new;               // difference last - new
  if( diff & 1 ) {                 // bit 0 = value (1)
    last = new;                    // store new as next last
    enc_delta -= (diff & 2) - 1;   // bit 1 = direction (+/-)
  }

    PRESC--;
    if (!PRESC) {
        // ca. 25Hz
        PRESC = 120;
        //position += enc_delta >> 3;
        position += enc_delta;
        enc_delta = 0;
        PRESC1--;
        if (!PRESC1) {
            //ca. 5 Hz
            PRESC1 = 5;
            PLEASE_UPDATE_STATION = 1;
        }
    }  
}


enum commands { NOCOMMAND, RADIO_FREQ, RADIO_FREQ_QUERY, PING, MIDI, DEBUGON, DEBUGOFF, RADIO_POWON, RADIO_POWOFF,RADIO_SEUP, RADIO_SEDOWN, LEDS, SHUTDOWN, REV, FAILURE }; 

uint8_t get_command(char * buf) {
    //debputs(DEBUG, "in get command");
    uint8_t command = FAILURE;
        if (memcmp(buf, "/radio/frequency?", 17) == 0) {
            command = RADIO_FREQ_QUERY;
        } else if (memcmp(buf, "/radio/frequency", 16) == 0) {
            command = RADIO_FREQ;
        } else if (memcmp(buf, "/ping", 5) == 0) {
            command = PING;
        } else if (memcmp(buf, "/midi", 5) == 0) {
            command = MIDI;
        } else if (memcmp(buf, "/debug/on",9) == 0) {
            command = DEBUGON;
        } else if (memcmp(buf, "/debug/off", 10) == 0) {
            command = DEBUGOFF;
        } else if (memcmp(buf, "/radio/power/up", 15) == 0) {
            command = RADIO_POWON;
        } else if (memcmp(buf, "/radio/power/down", 17) == 0) {
            command = RADIO_POWOFF;
        } else if (memcmp(buf, "/radio/seek/up", 14) == 0) {
            command = RADIO_SEUP;
        } else if (memcmp(buf, "/radio/seek/down", 16) == 0) {
            command = RADIO_SEDOWN;
        }  else if (memcmp(buf, "/leds", 5) == 0) {
            command = LEDS; 
        }  else if (memcmp(buf, "/shutdown", 9) == 0) {
            command = SHUTDOWN;
        }  else if (memcmp(buf, "/revision", 9) == 0) {
            command = REV;
        }
    return command;
} 


enum OSC_STATE {buf_empty, rec_command, is_paramlist_there, rec_paramlist, rec_param};

static char buf[60];
static uint8_t i = 0;
static uint8_t p;
static uint8_t paramliststart;

uint8_t parseOSCchar(char c) {
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
            done = 0xff;
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
    PORTB &= 0b11110000;
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

volatile int16_t endpos = 0;

void stepenable(uint8_t en) {
    uint8_t portb = PORTB & 0b11001111; 
    PORTB = portb | (en << 4);
}

enum {STOPPED, RAMP_UP, RAMP_DOWN, RUNNING};
volatile uint8_t stepper_state = STOPPED;
volatile uint8_t speed = 0;
volatile uint16_t ramppos = 0;

void turn(uint16_t freq) {
    stepper_state = RAMP_UP;
    speed = 0;
    debputs(DEBUG, "in turn");
    //_delay_ms(100);
    cli();
    endpos = freq2position(freq);
    //stop_encoder();
    if (endpos < position) direction = -1; else direction = 1;
    ramppos = endpos + direction * (-1) * 100;

    switch (direction) {
        case -1:
            if (ramppos > position) ramppos = position;
            break;
        case 1:
            if (ramppos < position) ramppos = position;
    }

    //if (direction == -1) uart_puts("-1 \n");
    sei();
    cli();
    stepenable(0b11);
    TIMSK0 |= 1<<OCIE0A;
    sei();
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
    //_delay_ms(10);
    //if (RADIO_THERE) setandsendfreq(position2freq(position));
    stepenable(0);
    //encode_init();
}



//uint8_t speedtable [4] = {255, 110, 90};
uint8_t speedtable [4] = {200, 90,25};

ISR (TIMER0_COMPA_vect) {
    //static uint8_t presc = 200;
    static uint8_t step = 0;
    static uint8_t speed_presc = 5;

    speed_presc--;

    if (!speed_presc) {
        speed_presc = 5;
        if (stepper_state == RAMP_UP) {
            OCR0A = speedtable[speed];
            speed++;
            if (speed >= 3) stepper_state = RUNNING;
        } else if (stepper_state == RAMP_DOWN) {
            speed--;
            OCR0A = speedtable[speed];
            if (speed == 0) speed = 1;
        }
    }

    if (direction == -1) {
        if (position < ramppos) stepper_state = RAMP_DOWN;
        if (position < endpos) stepper_state = STOPPED;
        
    }

    if (direction == 1) {
        if (position > ramppos) stepper_state = RAMP_DOWN;
        if (position > endpos) stepper_state = STOPPED;
    }


    if (stepper_state == STOPPED) {
        debprintf(DEBUG,"reached %i", position);
        stepper_stop();
        return;
    } 

    
    PORTC = steps[step];
    step += direction;
    if (step == 255) step = 3;
    if (step > 3) step = 0;
}


void peripherieturnon(void) {
    DDRD |= (1 << PD5);
    PORTD |= (1 << PD5);
}

void peripherieturnoff(void) {
    PORTD &= ~(1 << PD5);
}

void gatheringhome(void) {
    debputs(DEBUG, "gathering home");
    position = freq2position(10800);
    turn(8590); //will never reach..
    _delay_ms(3000);
    stepper_stop(); 
    _delay_ms(50);
    position = freq2position(8590); //ganze linke home position
    freq = 8750;
    turn(8750);
    _delay_ms(3000);
}


uint8_t startdelay(void) {
    //lauflicht..
    ws2812_setleds_rgb(0,0,0,18);
    uint8_t i = 1;
    while ( !(PIND & (1<< PD2)) && i <= 18  ) {
        ws2812_setleds_rgb(30,30,30,i);
        _delay_ms(166);
        i++;
    } 
    ws2812_setleds_rgb(0,0,0,18);
    if (i <= 18) return 0;
    else return 1;
}

volatile uint8_t RUN;

struct cRGB ledar[18];

void startup(void) {
    peripherieturnon();

    cli();
    uart_init (UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU) );
    suart_init();
    sei();

    i2c_init();                             // initialize I2C library
    //PORTC |= (1<< PC4) | (1 << PC5);        // using the internal pull ups!
    
    init_stepper();                         //stepper init

    encode_init();                          //encoder init

    gatheringhome();                        //home position finden


    uint8_t erg = si4735_powerup();                        // radio einschalten
    debprintf(DEBUG, "powerup return %i", erg);
    if (erg == 1) {    
        RADIO_THERE = 1;
        _delay_ms(800);
        //setfreq(8880);
        //freq = getfreq();
        //endpos = freq2position(freq);
        turn(8880);
        _delay_ms(2000);
        //freq = 8880;
    } else {
        debputs(DEBUG, "ERROR: Could not connect to radio chip.");
        ws2812_setleds_rgb(50, 0, 0, 9);
        for (;;);
    }

    //turn(8880);
   
    //everything done
    //XXX wait for raspberry pi!

                     createLedArray(ledar,20 ,0,0 , 18);  
                    modifyLedArray(ledar, 18);
                    ws2812_setleds(ledar, 18);
    OSCcreateMessage("/poweredup", "");    
     DDRD &= ~(1 << PD2);        // its an input
    PORTD |= (1 << PD2);                // pull up für reed eingang
    EIMSK |= (1 << INT0);            // externen Interrupt freigeben
    RUN = 1;
}


void sleep(void) {
        DDRD &= ~(1 << PD2);        // its an input
        PORTD |= (1 << PD2);                // pull up für reed eingang
        EIMSK |= (1 << INT0);            // externen Interrupt freigeben
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        RUN = 0;
        ws2812_setleds_rgb(0,0,0,18);
        ws2812_setleds_rgb(0,0,20,1);
        sleep_mode();                 
}


void shutdown(void) {
    //radio ausschalten
    si4735_powerdown();

    //auschaltmessage an maxmsp
    OSCcreateMessage("/shuttingdown", "");

    //ausschaltmessage an raspberry
    OSCcreateMessage("/raspberry/shutdown", "");
    _delay_ms(3000);
    
    peripherieturnoff();

    sleep();
}



ISR(INT0_vect) {
    // aha, das reed relais wurde gedrückt..
    cli();
    EIMSK &= ~(1 << INT0);           // externen Interrupt sperren
    sei();
   
    if (startdelay()) {
        sei();
        if (RUN) shutdown();
        if (!RUN) soft_reset();
    }
    sei();
    if (!RUN) sleep();          //still sleepy
    //back to normal
    ws2812_setleds(ledar, 18);
    //ws2812_setleds_rgb(0,0,0,18);
    EIMSK |= (1 << INT0);            // externen Interrupt freigeben
}

int main(void) {
    MCUSR = 0;
    wdt_disable();

    unsigned int c;
    uint8_t received;

    uint8_t result;

    startup();
    //struct cRGB * ledar = malloc(18+18+18);

    int8_t STATION_UPDATED = 0;
    //main loop
    for (;;) {
        if (PLEASE_UPDATE_STATION) {
            PLEASE_UPDATE_STATION = 0;
            cli();
            int8_t diff = runden(position2freq(position)) - freq;
            sei();
            if (diff != 0) {
                STATION_UPDATED = 8;
                freq = runden(position2freq(position));
                si4735_setfreq(freq);
            } else {
                if (STATION_UPDATED >= 0) {
                    STATION_UPDATED--; 
                } 
                if (STATION_UPDATED == 0) { 
                    //ok.. jetzt wurde der sender verstellt ..
                    //message an max msp mit der neuen Frequenz senden..
                    OSCcreateMessage("/radio/frequency", "i", (int32_t) freq);
                } 
            }
        }

        // prüfen ob neues zeichen verfügbar
	c = uart_getc();
	if (c & UART_NO_DATA) {
	} else {
		received = (uint8_t) c & 0xff;
                result = parseOSCchar(received);
                if (result != 0) {
                    debprintf(DEBUG, "command %i received", result);
                }
                if (result == REV) {
                    OSCcreateMessage("/revision", "s", REVISION);
                } else if (result == 30) {
                    shutdown();
                } else if (result == LEDS) {
                    createLedArray(ledar,buf[19], buf[19+4], buf[19+8], 18);  
                    modifyLedArray(ledar, 18);
                    ws2812_setleds(ledar, 18);
                    //ws2812_setleds_rgb(buf[19], buf[19+4], buf[19+8], 18);
                } else if (result == DEBUGON ) { 
                    DEBUG = true;
                    debputs(DEBUG, ">>> DEBUG MODE ON <<<");
                    OSCcreateMessage("/debug/on", "");
                } else if (result == DEBUGOFF) {
                    debputs(DEBUG, ">>> DEBUG MODE OFF <<<");
                    DEBUG = false;
                    OSCcreateMessage("/debug/off", "");
                } else if (result == RADIO_POWON ) {
                    //power up
                    si4735_powerup();
                    _delay_ms(50);
                    //setandsendfreq(position2freq(position));
                    freq = position2freq(position);
                } else if (result == RADIO_POWOFF) {
                    //power down
                    si4735_powerdown();
                } else if (result == RADIO_SEUP) {
                    //seek up
                    si4735_getfreq(); //stop running seek..
                    si4735_seekup();
                } else if (result == RADIO_SEDOWN) {
                    //seek down
                    si4735_getfreq(); //stop running seek..
                    si4735_seekdown();
               } else if (result == RADIO_FREQ) {
                    if (stepper_state != STOPPED) {
                        stepper_state = RAMP_DOWN;
                        _delay_ms(50);
                    } 
                    freq = (uint16_t) (buf[26] << 8);
                    freq += buf[27];
                    turn(freq);
                } else if (result == PING ) {
                    OSCcreateMessage("/ping", "");
               } else if (result == MIDI) {
                    midi_putc(buf[19]);
                    midi_putc(buf[19+4]);
                    midi_putc(buf[19+8]); 
               } else if (result == RADIO_FREQ_QUERY) {
                    //freq = getfreq();
                    OSCcreateMessage("/radio/frequency","i", (int32_t) freq);
               }
       	}
   }
}
