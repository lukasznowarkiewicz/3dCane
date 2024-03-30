/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

*/

#ifndef _SPI_H_
#define _SPI_H_

#include <avr/io.h>

#define PORT_SPI    PORTB
#define DDR_SPI     DDRB

#define DD_SS       DDB2 // wejœcie Slave Select
#define DD_MOSI     DDB3
#define DD_MISO     DDB4
#define DD_SCK      DDB5


extern void spi_init();
extern void spi_transfer (uint8_t * dataout, uint8_t * datain, uint8_t len);
extern void spi_transmit (uint8_t * dataout, uint8_t len);
extern uint8_t spi_fast_shift (uint8_t data);


#endif /* _SPI_H_ */
