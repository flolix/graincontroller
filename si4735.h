/*
* include file for si4735.c, for controlling a si4735 vi HW TWI
*
* project: radiosands
* target: AtMega328
*
* coded by guistlerei
*/

#include <stdint.h>

void setfreq(uint16_t freq) ;
void seekup(void) ;
void seekdown(void) ;
void powerup(void) ;
void powerdown(void) ;
uint16_t getfreq(void) ;
