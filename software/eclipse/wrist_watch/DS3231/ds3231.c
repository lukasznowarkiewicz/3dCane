/*
 * ds3231.c
 *
 *  Created on: 25-02-2017
 *      Author: Jacek
 */

#include <avr/io.h>
#include "ds3231.h"
#include "../I2C_TWI/i2c_twi.h"
#include "../MKUART/mkuart.h"

void DS3231_init( void ) {
	uint8_t ctrl = 0;
	TWI_write_buf( DS3231_ADDR, 0x0e, 1, &ctrl );
}

void DS3231_get_datetime( TDATETIME * dt ) {
	uint8_t i;
	uint8_t buf[7];
	TWI_read_buf( DS3231_ADDR, 0x00, 7, buf );
	for( i=0; i<7; i++ ) dt->bytes[i] = bcd2dec( buf[i] );
}


void DS3231_show_time( TDATETIME * dt ) {

	uart_putint(dt->hh, 10);	// wyœlij liczbê
	uart_puts(":");	// wyœlij tekst
	uart_putint(dt->mm, 10);	// wyœlij liczbê
	uart_puts(":");	// wyœlij tekst
	uart_putint(dt->ss, 10);	// wyœlij liczbê
	uart_putc('\r');			// wyœlij znak CR (enter)
	uart_putc('\n');			// wyœlij znak LF (nowa linia)
//	lcd_int(dt->hh);
//	lcd_int(dt->mm);
//	lcd_int(dt->ss);
}

void DS3231_show_date( TDATETIME * dt ) {
	uart_putint(dt->day, 10);	// wyœlij liczbê
	uart_puts(".");	// wyœlij tekst
	uart_putint(dt->month, 10);	// wyœlij liczbê
	uart_puts(".");	// wyœlij tekst
	uart_putint(dt->year, 10);	// wyœlij liczbê
	uart_puts(" ");	// wyœlij tekst
	uart_putint(dt->dayofwek, 10);	// wyœlij liczbê
	uart_putc('\r');			// wyœlij znak CR (enter)
	uart_putc('\n');			// wyœlij znak LF (nowa linia)
//	lcd_int(dt->year);
//	lcd_int(dt->month);
//	lcd_int(dt->day);
}

uint8_t DS3231_get_hour( TDATETIME * dt ){

	return dt->hh;

}
uint8_t DS3231_get_min( TDATETIME * dt ){
	return dt->mm;
}

void DS3231_set_time( uint8_t hh, uint8_t mm, uint8_t ss ) {
	uint8_t buf[3];
	buf[0]=dec2bcd(ss);
	buf[1]=dec2bcd(mm);
	buf[2]=dec2bcd(hh);
	TWI_write_buf( DS3231_ADDR, 0x00, 3, buf );
}

void DS3231_set_date( uint8_t year, uint8_t month, uint8_t day, uint8_t dayofweek ) {
	uint8_t buf[4];
	buf[0]=dayofweek;
	buf[1]=dec2bcd(day);
	buf[2]=dec2bcd(month);
	buf[3]=dec2bcd(year);
	TWI_write_buf( DS3231_ADDR, 0x03, 4, buf );
}


void DS3231_get_temp( TTEMP * tmp ) {
	uint8_t buf[2];
	TWI_read_buf( DS3231_ADDR, 0x11, 2, buf );
	tmp->cel = buf[0] ;

	TWI_read_buf( DS3231_ADDR, 0x0e, 1, buf );
	buf[0] |= (1<<5);
	TWI_write_buf( DS3231_ADDR, 0x0e, 1, buf );
}


void show_temperature( TTEMP * tmp ) {

//	lcd_int(tmp->cel);
}



// konwersja liczby dziesiêtnej na BCD
uint8_t dec2bcd(uint8_t dec) {
return ((dec / 10)<<4) | (dec % 10);
}

// konwersja liczby BCD na dziesiêtn¹
uint8_t bcd2dec(uint8_t bcd) {
    return ((((bcd) >> 4) & 0x0F) * 10) + ((bcd) & 0x0F);
}
