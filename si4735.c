/*
* for controlling a si4735 vi HW TWI
*
* project: radiosands
* target: AtMega328
*
* coded by guistlerei
*/

#include <stdint.h>
#include <i2cmaster.h>
#include <util/delay.h>
#include <debug.h>

#define DevSi4735  0x22      // device address of SI4735, see datasheet

void si4735_setfreq(uint16_t freq) {
    uint8_t h,l;
    i2c_start_wait(DevSi4735+I2C_WRITE);   // set device address and write mode
    i2c_write(0x20);                       // write command 
    i2c_write(0x00);
	h = (uint8_t) (freq >> 8);
	l = (uint8_t) (freq & 0xff);
    i2c_write(h);                          // write frequency
    i2c_write(l);
    i2c_write(0x00);
    i2c_stop();
}

void si4735_seekup(void) {
    i2c_start_wait(DevSi4735+I2C_WRITE);   
    i2c_write(0x21);                      
    i2c_write(0x08);
    i2c_stop();
}

void si4735_seekdown(void) {
    i2c_start_wait(DevSi4735+I2C_WRITE);   
    i2c_write(0x21);                      
    i2c_write(0x00);
    i2c_stop();
}


uint8_t si4735_powerup(void) {
    uint8_t erg = i2c_start(DevSi4735+I2C_WRITE);
    uint8_t tries = 5;
    while (erg == 1 && tries != 0) {
        tries--;
        _delay_ms(100);
        erg = i2c_start(DevSi4735+I2C_WRITE);
    }
    if (tries == 0 && erg == 1) return 0;
    //i2c_start_wait(DevSi4735+I2C_WRITE); 

    i2c_write(0x01);                    
    i2c_write(0x10);   
    i2c_write(0x05);                   
    i2c_stop();
    return 1;
}

void si4735_powerdown(void) {
    i2c_start_wait(DevSi4735+I2C_WRITE); 

    i2c_write(0x11);   
    i2c_stop();
}  

uint16_t si4735_getfreq(void) {
    uint8_t freqh, freql;

    i2c_start_wait(DevSi4735+I2C_WRITE); 
    i2c_write(0x22);   
    i2c_write(0x03);   
    i2c_stop();

    i2c_start_wait(DevSi4735+I2C_READ);       // set device address and read mode
    i2c_readAck();
    i2c_readAck();
    freqh = i2c_readAck();
    freql = i2c_readAck();
    i2c_readNak();                    // read one byte
    i2c_stop();
    return (uint16_t) 256*freqh + freql;
}


