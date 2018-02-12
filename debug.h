/*
* provides a debug interface via uart
*
* project: radiosands
* target: AtMega328
*
* coded by guistlerei
*/

#include <stdbool.h>

bool DEBUG = true;

void debputs(uint8_t debug, char * s); 

void debprintf(uint8_t debug, char * fs, ...);
