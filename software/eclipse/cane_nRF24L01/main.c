/*
 * main.c  
 *
 *  Created on: 15-10-2013
 *      Author: Adam Peterwas
 *      na podstawie   http://www.tinkerer.eu/AVRLib/nRF24L01
 *
 */


#include "NRF24L01/mirf.h"
#include "NRF24L01/nRF24L01.h"
#include "SPI/spi.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "MKUART/mkuart.h"

#define tryb	2 // 1-nadaj, 2- odbierz, 3- register

    uint8_t nadaj_buf[10];
    uint8_t odebrany_buf[16];

#define SCK 	PB7
#define MOSI 	PB5
//#define MISO 	PB6
#define CS 		PC2

int main (void)
    {

	DDRA |= (1<<PA6);


	DDRC |= (1<<PC5);
	PORTC |= (1<<PC5);

	// init SPI
	DDRB |= (1 << MOSI) | (1 << SCK);
	PORTB |= (1 << MOSI) | (1 << SCK);
	DDRC |= (1 << CS);
	PORTC |= (1 << CS);

//	spi_init();
	PORTD |= 1<<PD4;
	PCICR |= (1<<PCIE3);
	PCMSK3 |= (1<<PCINT28);


    mirf_init() ;
    _delay_ms(50);

    mirf_config();
    uart_set(1);
    USART_Init(__UBRR);		// inicjalizacja USART
    sei();

	uart_putc('\r');			// wyœlij znak CR (enter)
	uart_putc('\n');			// wyœlij znak LF (nowa linia)
	uart_puts("START");	// wyœlij tekst
	uart_putc('\r');			// wyœlij znak CR (enter)
	uart_putc('\n');			// wyœlij znak LF (nowa linia)

    while(1)
	{
    	uart_putint(mirf_data_ready(),10);
    	uart_puts("\n\r");
    	_delay_ms(500);
#if tryb == 1// Nadajnik
	nadaj_buf[0]++;
	mirf_send(nadaj_buf,1); // iloœæ bajtów

//	UART_puts_P(PSTR("Nadano "));
//	UART_puts_P(PSTR(" - "));
//	UART_putint(nadaj_buf[0],10);
//	UART_puts_P(PSTR("\n\r"));
	_delay_ms(500);

#endif
#if tryb == 2 // Odbiornik

	while (!mirf_data_ready());

	mirf_get_data(odebrany_buf);
	PORTA ^= (1<<PA6);
	uart_puts("Odebrano ");
	uart_puts(" - ");
	uart_putint(odebrany_buf[0],10);
	uart_puts("\n\r");

#endif
#if tryb == 3 // Zobacz Data Register
	for(uint8_t nr=0;nr<16;nr++)
	    {
	    mirf_read_register(nr,odebrany_buf,1);
	    UART_puts_P(PSTR("Address "));
	    UART_putint(nr,10);
	    UART_puts_P(PSTR(" - "));
	    UART_putint(odebrany_buf[0],2);
	    UART_puts_P(PSTR("\n\r"));
	    _delay_ms(1000);
	    }
	while(1);
#endif
	}
    }


