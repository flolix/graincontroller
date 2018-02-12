#include <stdint.h>
#include "uart.h"
#include <stdio.h>

void debputs(uint8_t debug, char * s) {
    if (!debug) return;
    uart_puts(s);
    uart_putc('\n');
}

void debprintf(uint8_t debug, char * fs, ...) {
    if (!debug) return;
    char buffer[256];
    va_list argumente;
    va_start(argumente, fs);
    vsnprintf( (char *) &buffer, 256, fs, argumente);
    va_end(argumente);
    uart_puts(buffer);
    uart_putc('\n');
}



