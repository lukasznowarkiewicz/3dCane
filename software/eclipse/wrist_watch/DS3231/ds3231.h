/*
 * ds3231.h
 *
 *  Created on: 25-02-2017
 *      Author: Jacek
 */

#ifndef DS3231_H_
#define DS3231_H_

#define DS3231_ADDR 0xD0


typedef union {
	uint8_t bytes[7];
	struct {
		uint8_t ss;
		uint8_t mm;
		uint8_t hh;
		uint8_t dayofwek;
		uint8_t day;
		uint8_t month;
		uint8_t year;
	};
} TDATETIME;


typedef struct {
	int8_t cel;
} TTEMP;

TDATETIME datetime;
TTEMP temperature;

void DS3231_init( void );
uint8_t dec2bcd(uint8_t dec);
uint8_t bcd2dec(uint8_t bcd);
void DS3231_get_datetime( TDATETIME * dt );
void DS3231_show_time( TDATETIME * dt );

uint8_t DS3231_get_hour( TDATETIME * dt );
uint8_t DS3231_get_min( TDATETIME * dt );

void DS3231_show_date( TDATETIME * dt );
void DS3231_set_time( uint8_t hh, uint8_t mm, uint8_t ss );
void DS3231_set_date( uint8_t year, uint8_t month, uint8_t day, uint8_t dayofweek );
void DS3231_get_temp( TTEMP * tmp );
void show_temperature( TTEMP * tmp );





#endif /* DS3231_H_ */
