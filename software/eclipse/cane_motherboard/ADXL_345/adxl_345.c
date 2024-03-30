/*
 * adxl_345.c
 *
 *  Created on: 02-08-2014
 *      Author: Intre
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "../MKUART/mkuart.h"
#include "../I2C_TWI/i2c_twi.h"

#include "adxl_345.h"


// zmienne i bufory dla dczytu pomiarów
uint8_t test_buf[1];
int16_t measure_buf[6];
volatile uint16_t x, y, z;

void adxl345_init (void){
        power_on();
        standby ();
        measure ();
        connect_test( ADXL345_ADR, DEV_ID, 1, test_buf);
}

//funkcje inicjalizacji pracy czujnika ADXL345
void power_on (void){
        TWI_start();
        TWI_write(ADXL345_ADR);
        TWI_write(POWER_CTL);
        TWI_write(0);
        TWI_stop();
}

void standby (void){
        TWI_start();
        TWI_write(ADXL345_ADR);
        TWI_write(POWER_CTL);
        TWI_write(16);
        TWI_stop();
}

void measure (void){
        TWI_start();
        TWI_write(ADXL345_ADR);
        TWI_write(POWER_CTL);
        TWI_write(8);
        TWI_stop();
}

//funkcja testu po³aczenia z akcelerometrem ADXL345
void connect_test(uint8_t Adress, uint8_t Mode, uint8_t Length, uint8_t *buf) {
        TWI_start();
        TWI_write(Adress);
        TWI_write(Mode);
        TWI_start();
        TWI_write(Adress +1);
        while (Length--) *buf++ = TWI_read( Length ? ACK : NACK );
        TWI_stop();
}


//wyœwietlanie testu po³¹czenia z akcelerometrem ADXL 345
void display_adxl345_connect_test (void){
        if(test_buf[0]==229)   // wartoœæ z noty katalogiwej czujnkia œwiadcz¹ca o poprawnej komunikacji
        	uart_puts("ADXL OK");	// wyœlij tekst
		uart_putc('\r');			// wyœlij znak CR (enter)
		uart_putc('\n');			// wyœlij znak LF (nowa linia)
        _delay_ms(2000);
}


//funkcja odczytu pomiarów osi XYZ
void adxl345_measure_read (void){
        measure_i2c_read( ADXL345_ADR, XYZ_DATA, 6, measure_buf);
}

void measure_i2c_read(uint8_t Adress, uint8_t Mode, uint8_t Length, int16_t *buf) {
        TWI_start();
        TWI_write(Adress);
        TWI_write(Mode);
        TWI_start();
        TWI_write(Adress +1);
        while (Length--) *buf++ = TWI_read( Length ? ACK : NACK );
        TWI_stop();
}

//funkcja konwersji pomiarów osi XYZ
void measure_convert (void){
    x = (((int16_t)measure_buf[1]) << 8) | measure_buf[0];
    y = (((int16_t)measure_buf[3]) << 8) | measure_buf[2];
    z = (((int16_t)measure_buf[5]) << 8) | measure_buf[4];
}

void adxl345_display_measure (void){


        measure_convert();

		uart_puts("X=");	// wyœlij tekst
		uart_putint(x, 10);	// wyœlij liczbê
		uart_puts(" Y=");	// wyœlij tekst
		uart_putint(y, 10);	// wyœlij liczbê
		uart_puts(" Z=");	// wyœlij tekst
		uart_putint(z, 10);	// wyœlij liczbê
		uart_putc('\r');			// wyœlij znak CR (enter)
		uart_putc('\n');			// wyœlij znak LF (nowa linia)

}
