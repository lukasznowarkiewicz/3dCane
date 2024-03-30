/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

*/

#include "../NRF24L01/mirf.h"
#include "../NRF24L01/nRF24L01.h"
#include "../SPI/spi.h"
#include <avr/io.h>
#include <avr/interrupt.h>


volatile uint8_t PTX;// Flaga oznaczaj¹ca tryb transmisji

// **********************************************************************************************************************
void mirf_init() // Inicjuje piny i przerwanie komunikacji z modu³em MiRF
    {
    // Define CSN and CE as Output and set them to default

    DDR_CSN |= (1<<CSN);
    DDR_CE  |= (1<<CE);
    mirf_CE_lo;
    mirf_CSN_hi;

    // Initialize external interrupt 2 (PB2)
//    MCUCR = (1<<ISC2); //Narastaj¹ce zbocze INT2 generuje ¿¹danie przerwania.
//    GICR  = (1<<INT2); // Activate INT2

    // Initialize spi module
	SPCR |= (1 << SPE) | (1 << MSTR);
	SPSR |= (1 << SPI2X); // masymalny zegar SCK
    }

// ********************************************************************************************************************
void mirf_config() 
// Ustawia wa¿ne rejestry w module MiRF i ustawia modu³ w tryb odbioru
{
    // Ustaw kana³ RF
    mirf_config_register(RF_CH,mirf_CH); // 5,2

    // Ustaw iloœc bajtów w bloku danych
    mirf_config_register(RX_PW_P0, mirf_PAYLOAD); //11,16

    // Ustaw iloœc prób odebrania danych
 //   mirf_config_register(EN_AA ,mirf_AUTOACK); // 1,1

    // Start receiver 
    PTX = 0;        // Start in receiving mode
    RX_POWERUP;     // 0-PRX,1-POWER UP,2-1 byte,3-Enable CRC,4-Reflect MAX_RT as active low interrupt on the IRQpin,
		    // 5-Reflect TX_DS as active low interrupt on the IRQ pin,6-RX_DR Interrupt not reflected on the IRQpin,
    mirf_CE_hi;     // Listening for pakets
}

// *********************************************************************************************************************
void mirf_set_RADDR(uint8_t * adr) 
// Ustawia adres odbioru
{
    mirf_CE_lo;
    mirf_write_register(RX_ADDR_P0,adr,5);
    mirf_CE_hi;
}

// *********************************************************************************************************************
void mirf_set_TADDR(uint8_t * adr)
// Sets the transmitting address
{
    mirf_write_register(TX_ADDR, adr,5);
}

// *********************************************************************************************************************
ISR(PCINT3_vect)
{
    uint8_t status;

    if (PTX) // Jeœli w trybie nadawania
	{
        // Read MiRF status
        mirf_CSN_lo;                                // Pull down chip select
        status = spi_fast_shift(NOP);               // Read status register
        mirf_CSN_hi;                                // Pull up chip select

        mirf_CE_lo;                             // Deactivate transreceiver
        RX_POWERUP;                             // Power up in receiving mode
        mirf_CE_hi;                             // Listening for pakets
        PTX = 0;                                // Set to receiving mode

        // Przywróæ rejestr stanu do dalszej interakcji
        mirf_config_register(STATUS,(1<<TX_DS)|(1<<MAX_RT)); // Reset status register
	}
}

// *********************************************************************************************************************
extern uint8_t mirf_data_ready() 
// Sprawdza, czy dane s¹ dostêpne do odczytu
{
//    if (PTX) return 0;
    uint8_t status;
    // Read MiRF status 
    mirf_CSN_lo;                                // Pull down chip select
    status = spi_fast_shift(NOP);               // Read status register
    mirf_CSN_hi;
    return status & (1<<2);
}

// **********************************************************************************************************************
extern void mirf_get_data(uint8_t * data) 
// Odczytuje bajty mirf_PAYLOAD do tablicy danych
{
    mirf_CSN_lo;                               // Pull down chip select
    spi_fast_shift( R_RX_PAYLOAD );            // Send cmd to read rx payload
    spi_transfer(data,data,mirf_PAYLOAD); 	// Read payload
    mirf_CSN_hi;                               // Pull up chip select
    mirf_config_register(STATUS,(1<<RX_DR));   // Reset status register
}

// *********************************************************************************************************************
void mirf_config_register(uint8_t reg, uint8_t value)
// ustawia tylko jeden bajt rejestru danych MiRF
{
    mirf_CSN_lo;
    spi_fast_shift(W_REGISTER | (REGISTER_MASK & reg));
    spi_fast_shift(value);
    mirf_CSN_hi;
}

// **********************************************************************************************************************
void mirf_read_register(uint8_t reg, uint8_t * value, uint8_t len)
// Odczytuje tablicê bajtów z danej pozycji startowej w rejestrach MiRF.
{
    mirf_CSN_lo;
    spi_fast_shift(R_REGISTER | (REGISTER_MASK & reg));
    spi_transfer(value,value,len);
    mirf_CSN_hi;
}

// ********************************************************************************************************************
void mirf_write_register(uint8_t reg, uint8_t * value, uint8_t len) 
// Zapisuje tablicê bajtów do rejestrów MiRF.
{
    mirf_CSN_lo;
    spi_fast_shift(W_REGISTER | (REGISTER_MASK & reg));
    spi_transmit(value,len);
    mirf_CSN_hi;
}

// *********************************************************************************************************************
void mirf_send(uint8_t * value, uint8_t len) 
// Wysy³a pakiet danych na adres domyœlny Pamiêtaj, aby wys³aæ poprawn¹ iloœæ bajtów
{
//    while (PTX) {}                  // Wait until last paket is send

    mirf_CE_lo;

    PTX = 1;                        // Set to transmitter mode
    TX_POWERUP;                     // Power up
    
    mirf_CSN_lo;                    // Pull down chip select
    spi_fast_shift( FLUSH_TX );     // Write cmd to flush tx fifo
    mirf_CSN_hi;                    // Pull up chip select
    
    mirf_CSN_lo;                    // Pull down chip select
    spi_fast_shift( W_TX_PAYLOAD ); // Write cmd to write payload
    spi_transmit(value,len);   	    // Write payload
    mirf_CSN_hi;                    // Pull up chip select
    
    mirf_CE_hi;                     // Start transmission
}
