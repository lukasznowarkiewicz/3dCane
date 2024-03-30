/*
 * adxl_345.h
 *
 *  Created on: 02-08-2014
 *      Author: Intre
 */

#ifndef ADXL_345_H_
#define ADXL_345_H_

//adres akcelerometru ADXL 345 0x53 przemno¿ony x2 dla trybu 8 bit
#define ADXL345_ADR 0xA6

//definicje rejestrów ADXL 345
#define DEV_ID 0x00
#define POWER_CTL 0x2d
#define XYZ_DATA 0x32

// zmienne i bufory dla dczytu pomiarów
uint8_t test_buf[1];
int16_t measure_buf[6];
volatile uint16_t x, y, z;

//deklaracje funkcji
void adxl345_init (void);
void power_on (void);
void standby (void);
void measure (void);
void connect_test (uint8_t Adress, uint8_t Mode, uint8_t Length, uint8_t *buf);
void display_adxl345_connect_test (void);
void adxl345_measure_read(void);
void measure_i2c_read(uint8_t Adress, uint8_t Mode, uint8_t Length, int16_t *buf);
void measure_convert (void);
void adxl345_display_measure (void);


#endif /* ADXL_345_H_ */
