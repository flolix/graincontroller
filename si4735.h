/*
* include file for si4735.c, for controlling a si4735 vi HW TWI
*
* project: radiosands
* target: AtMega328
*
* coded by guistlerei
*/

#include <stdint.h>

void si4735_setfreq(uint16_t freq) ;
void si4735_seekup(void) ;
void si4735_seekdown(void) ;
uint8_t si4735_powerup(void) ;
void si4735_powerdown(void) ;
uint16_t si4735_getfreq(void) ;
