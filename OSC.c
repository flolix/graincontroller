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
    return len;
}

void OSCcreateMessage(char * command, char * paramlist, ...) {
    va_list argumente;
    uint8_t anzahl, msglength, i, an;
    int32_t intarg;
    va_start(argumente, paramlist);
    anzahl = strlen(paramlist);
    
    char buf[40];
    char pl[8] = ",";

    msglength = OSCformatstring(command, &buf[0]);

    strcat(pl, paramlist); 
    msglength  += OSCformatstring(pl, &buf[msglength]);
    int8_t j;
    for (i = 0; i< anzahl; i++) {
        if (paramlist[i] == 'i') {
            intarg =  va_arg(argumente, int32_t);
            char * test = (char *) &intarg;
            for (j = 3; j >= 0; j--) {
                //uart_putc( * (test+j));
                buf[msglength] = * (test+j);
                msglength++;
            }

        }
        if (paramlist[i] == 's') {
            char * strarg;
            strarg = va_arg(argumente, char *);
            msglength += OSCformatstring(strarg, &buf[msglength]);
        }
    }
    va_end(argumente);

    for (i = 0; i < msglength; i++) uart_putc(buf[i]);
    uart_putc('\n');
}

