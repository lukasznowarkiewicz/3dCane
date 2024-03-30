/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>
*/

#ifndef _MIRF_H_
#define _MIRF_H_

#include <avr/io.h>

// Mirf settings
#define mirf_CH         2
#define mirf_PAYLOAD    1  // d³ugoœc bloku danych
#define mirf_AUTOACK	1  // ustawia liczbê prób odebrania danych

// Pin definitions for chip select and chip enabled of the MiRF module
#define CE  		PB4
#define PORT_CE   	PORTB
#define DDR_CE    	DDRB

#define CSN 		PC2
#define PORT_CSN   	PORTC
#define DDR_CSN    	DDRC

// Definicje do ustawiania rejestrów MiRF dla trybu transmisji lub odbierania
#define mirf_CONFIG     ((1<<MASK_RX_DR) | (1<<EN_CRC) | (0<<CRCO)) // CRC - 1 byte
#define TX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) )
#define RX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) ) // 01001011

// Definitions for selecting and enabling MiRF module
#define mirf_CSN_hi     PORT_CSN |=  (1<<CSN);
#define mirf_CSN_lo     PORT_CSN &= ~(1<<CSN);
#define mirf_CE_hi      PORT_CE |=  (1<<CE);
#define mirf_CE_lo      PORT_CE &= ~(1<<CE);

// Public standart functions
extern void mirf_init();
extern void mirf_config();
extern void mirf_send(uint8_t * value, uint8_t len);
extern void mirf_set_RADDR(uint8_t * adr);
extern void mirf_set_TADDR(uint8_t * adr);
extern uint8_t mirf_data_ready();
extern void mirf_get_data(uint8_t * data);


// Public extended functions
extern void mirf_config_register(uint8_t reg, uint8_t value);
extern void mirf_read_register(uint8_t reg, uint8_t * value, uint8_t len);
extern void mirf_write_register(uint8_t reg, uint8_t * value, uint8_t len);

#endif /* _MIRF_H_ */
