#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#include "debug.h"
#include "uart.h"


uint8_t OSCanzahlnulls (uint8_t i) {
    return  4 - (uint8_t) (i & 0b11);
}


uint8_t OSCformatstring(char * str, char * bu) {
    uint8_t an, i, len;
    len = strlen(str);
    strcpy(bu, str);
    an = OSCanzahlnulls(len);
    for (i = 0; i< an; i++) bu[i+len] = '\0';
    len += an;
    //buffer ausgeben..
    return len;
}

void OSCcreateMessage(char * command, char * paramlist, ...) {
    va_list argumente;
    uint8_t anzahl, commandlength, i, an;
    int32_t intarg;
    va_start(argumente, paramlist);
    anzahl = strlen(paramlist);
    
    char buf[40];
    char pl[6] = ",";

    commandlength = OSCformatstring(command, &buf[0]);

    strcat(pl, paramlist); 
    uint8_t cl = OSCformatstring(pl, &buf[commandlength]);
    commandlength += cl;
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
        if (paramlist[i] == 's') {
            char * strarg;
            strarg = va_arg(argumente, char *);
            commandlength += OSCformatstring(strarg, &buf[commandlength]);
        }
    }
    va_end(argumente);

    for (i = 0; i < commandlength; i++) uart_putc(buf[i]);
    uart_putc('\n');
}


