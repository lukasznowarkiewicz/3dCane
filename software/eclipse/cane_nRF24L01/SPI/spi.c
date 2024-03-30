/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

*/

#include "spi.h"

#include <avr/io.h>
#include <avr/interrupt.h>

// **********************************************************************************************************************
void spi_init()

{
//    DDR_SPI &= ~((1<<DD_MOSI)|(1<<DD_MISO)|(1<<DD_SS)|(1<<DD_SCK));// Initialize pins for spi communication
//    DDR_SPI |= ((1<<DD_MOSI)|(1<<DD_SS)|(1<<DD_SCK));// Define the following pins as output

    SPCR = ((1<<SPE)|               // SPI Enable
            (0<<SPIE)|              // SPI Interupt Enable
            (0<<DORD)|              // Data Order (0:MSB first / 1:LSB first)
            (1<<MSTR)|              // Master/Slave select   
            (0<<SPR1)|(1<<SPR0)|    // SPI Clock Rate --> fOSC/16
            (0<<CPOL)|              // Clock Polarity (0:SCK low / 1:SCK hi when idle)
            (0<<CPHA));             // Clock Phase (0:leading / 1:trailing edge sampling)

    SPSR = (1<<SPI2X);              // Podw�jna szybko�� SPI
}

// *********************************************************************************************************************
void spi_transfer (uint8_t * dataout, uint8_t * datain, uint8_t len) // wy�lij  i odbierz zadan� ilo�c bajt�w
// Shift full array through target device
    {
       uint8_t i;      
       for (i = 0; i < len; i++)
	   {
	   SPDR = dataout[i];
	   while((SPSR & (1<<SPIF))==0);
	   datain[i] = SPDR;
	   }
    }

// *********************************************************************************************************************
void spi_transmit (uint8_t * dataout, uint8_t len) // wy�lij zadan� ilo�c bajt�w
// Shift full array to target device without receiving any byte
{
       uint8_t i;      
       for (i = 0; i < len; i++) {
             SPDR = dataout[i];
             while((SPSR & (1<<SPIF))==0);
       }
}

// **********************************************************************************************************************
uint8_t spi_fast_shift (uint8_t data) // wy�lij i odbierz jeden bajt
// Clocks only one byte to target device and returns the received one
{
    SPDR = data;
    while((SPSR & (1<<SPIF))==0);
    return SPDR;
}

