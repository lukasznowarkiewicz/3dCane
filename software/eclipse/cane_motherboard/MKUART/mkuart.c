/*
 * mkuart.c
 *
 *  Created on: 2010-09-04
 *       Autor: Miros≈Çaw Karda≈õ
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>


#include "mkuart.h"

char uart=0;

// definiujemy w koÒcu nasz bufor UART_RxBuf
volatile char UART_RxBuf0[UART_RX_BUF_SIZE];
// definiujemy indeksy okreúlajπce iloúÊ danych w buforze
volatile uint8_t UART_RxHead0; // indeks oznaczajπcy Ñg≥owÍ wÍøaî
volatile uint8_t UART_RxTail0; // indeks oznaczajπcy Ñogon wÍøaî



// definiujemy w koÒcu nasz bufor UART_RxBuf
volatile char UART_TxBuf0[UART_TX_BUF_SIZE];
// definiujemy indeksy okreúlajπce iloúÊ danych w buforze
volatile uint8_t UART_TxHead0; // indeks oznaczajπcy Ñg≥owÍ wÍøaî
volatile uint8_t UART_TxTail0; // indeks oznaczajπcy Ñogon wÍøaî

// definiujemy w koÒcu nasz bufor UART_RxBuf
volatile char UART_RxBuf1[UART_RX_BUF_SIZE];
// definiujemy indeksy okreúlajπce iloúÊ danych w buforze
volatile uint8_t UART_RxHead1; // indeks oznaczajπcy Ñg≥owÍ wÍøaî
volatile uint8_t UART_RxTail1; // indeks oznaczajπcy Ñogon wÍøaî



// definiujemy w koÒcu nasz bufor UART_RxBuf
volatile char UART_TxBuf1[UART_TX_BUF_SIZE];
// definiujemy indeksy okreúlajπce iloúÊ danych w buforze
volatile uint8_t UART_TxHead1; // indeks oznaczajπcy Ñg≥owÍ wÍøaî
volatile uint8_t UART_TxTail1; // indeks oznaczajπcy Ñogon wÍøaî

void uart_set(char nr) {
	uart=nr;
}



void USART_Init( uint16_t baud ) {
	if(!uart) {
	/* Ustawienie prÍdkoúci */
	UBRR0H = (uint8_t)(baud>>8);
	UBRR0L = (uint8_t)baud;
	/* Za≥πczenie nadajnika I odbiornika */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Ustawienie format ramki: 8bitÛw danych, 1 bit stopu */
	UCSR0C = (1<<UMSEL01)|(3<<UCSZ00);

		// jeúli nie  korzystamy z interefejsu RS485
		UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	}
	else {
	/* Ustawienie prÍdkoúci */
	UBRR1H = (uint8_t)(baud>>8);
	UBRR1L = (uint8_t)baud;
	/* Za≥πczenie nadajnika I odbiornika */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	/* Ustawienie format ramki: 8bitÛw danych, 1 bit stopu */
	UCSR1C = (1<<UMSEL11)|(3<<UCSZ10);

		// jeúli nie  korzystamy z interefejsu RS485
		UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1);
	}
}

// definiujemy funkcjÍ dodajπcπ jeden bajtdoz bufora cyklicznego
void uart_putc( char data ) {

	if(!uart) {

	uint8_t tmp_head0;

    tmp_head0  = (UART_TxHead0 + 1) & UART_TX_BUF_MASK;

          // pÍtla oczekuje jeøeli brak miejsca w buforze cyklicznym na kolejne znaki
    while ( tmp_head0 == UART_TxTail0 );

    UART_TxBuf0[tmp_head0] = data;
    UART_TxHead0 = tmp_head0;

    // inicjalizujemy przerwanie wystÍpujπce, gdy bufor jest pusty, dziÍki
    // czemu w dalszej czÍúci wysy≥aniem danych zajmie siÍ juø procedura
    // obs≥ugi przerwania
    UCSR0B |= (1<<UDRIE0);
	}
	else {

	uint8_t tmp_head1;

    tmp_head1  = (UART_TxHead1 + 1) & UART_TX_BUF_MASK;

          // pÍtla oczekuje jeøeli brak miejsca w buforze cyklicznym na kolejne znaki
    while ( tmp_head1 == UART_TxTail1 );

    UART_TxBuf1[tmp_head1] = data;
    UART_TxHead1 = tmp_head1;

    // inicjalizujemy przerwanie wystÍpujπce, gdy bufor jest pusty, dziÍki
    // czemu w dalszej czÍúci wysy≥aniem danych zajmie siÍ juø procedura
    // obs≥ugi przerwania
    UCSR1B |= (1<<UDRIE1);
	}

}


void uart_puts(char *s)		// wysy≥a ≥aÒcuch z pamiÍci RAM na UART
{
  register char c;
  while ((c = *s++)) uart_putc(c);			// dopÛki nie napotkasz 0 wysy≥aj znak
}

void uart_putint(int value, int radix)	// wysy≥a na port szeregowy tekst
{
	char string[17];			// bufor na wynik funkcji itoa
	itoa(value, string, radix);		// konwersja value na ASCII
	uart_puts(string);			// wyúlij string na port szeregowy
}


// definiujemy procedurÍ obs≥ugi przerwania nadawczego, pobierajπcπ dane z bufora cyklicznego
ISR( USART0_UDRE_vect)  {
    // sprawdzamy czy indeksy sπ rÛøne
    if ( UART_TxHead0 != UART_TxTail0 ) {
    	// obliczamy i zapamiÍtujemy nowy indeks ogona wÍøa (moøe siÍ zrÛwnaÊ z g≥owπ)
    	UART_TxTail0 = (UART_TxTail0 + 1) & UART_TX_BUF_MASK;
    	// zwracamy bajt pobrany z bufora  jako rezultat funkcji
    	UDR0 = UART_TxBuf0[UART_TxTail0];
    } else {
	// zerujemy flagÍ przerwania wystÍpujπcego gdy bufor pusty
	UCSR0B &= ~(1<<UDRIE0);
    }
}

// definiujemy procedurÍ obs≥ugi przerwania nadawczego, pobierajπcπ dane z bufora cyklicznego
ISR( USART1_UDRE_vect)  {
    // sprawdzamy czy indeksy sπ rÛøne
    if ( UART_TxHead1 != UART_TxTail1 ) {
    	// obliczamy i zapamiÍtujemy nowy indeks ogona wÍøa (moøe siÍ zrÛwnaÊ z g≥owπ)
    	UART_TxTail1 = (UART_TxTail1 + 1) & UART_TX_BUF_MASK;
    	// zwracamy bajt pobrany z bufora  jako rezultat funkcji
    	UDR1 = UART_TxBuf1[UART_TxTail1];
    } else {
	// zerujemy flagÍ przerwania wystÍpujπcego gdy bufor pusty
	UCSR1B &= ~(1<<UDRIE1);
    }
}


// definiujemy funkcjÍ pobierajπcπ jeden bajt z bufora cyklicznego
char uart_getc(void) {
	if(!uart) {
    // sprawdzamy czy indeksy sπ rÛwne
    if ( UART_RxHead0 == UART_RxTail0 ) return 0;

    // obliczamy i zapamiÍtujemy nowy indeks Ñogona wÍøaî (moøe siÍ zrÛwnaÊ z g≥owπ)
    UART_RxTail0 = (UART_RxTail0 + 1) & UART_RX_BUF_MASK;
    // zwracamy bajt pobrany z bufora  jako rezultat funkcji
    return UART_RxBuf0[UART_RxTail0];
	}
	else {
    // sprawdzamy czy indeksy sπ rÛwne
    if ( UART_RxHead1 == UART_RxTail1 ) return 0;

    // obliczamy i zapamiÍtujemy nowy indeks Ñogona wÍøaî (moøe siÍ zrÛwnaÊ z g≥owπ)
    UART_RxTail1 = (UART_RxTail1 + 1) & UART_RX_BUF_MASK;
    // zwracamy bajt pobrany z bufora  jako rezultat funkcji
    return UART_RxBuf1[UART_RxTail1];
	}
}

// definiujemy procedurÍ obs≥ugi przerwania odbiorczego, zapisujπcπ dane do bufora cyklicznego
ISR( USART0_RX_vect ) {
    uint8_t tmp_head0;
    char data0;

    data0 = UDR0; //pobieramy natychmiast bajt danych z bufora sprzÍtowego

    // obliczamy nowy indeks Ñg≥owy wÍøaî
    tmp_head0 = ( UART_RxHead0 + 1) & UART_RX_BUF_MASK;

    // sprawdzamy, czy wπø nie zacznie zjadaÊ w≥asnego ogona
    if ( tmp_head0 == UART_RxTail0 ) {
    	// tutaj moøemy w jakiú wygodny dla nas sposÛb obs≥uøyÊ  b≥πd spowodowany
    	// prÛbπ nadpisania danych w buforze, mog≥oby dojúÊ do sytuacji gdzie
    	// nasz wπø zaczπ≥by zjadaÊ w≥asny ogon
    } else {
	UART_RxHead0 = tmp_head0; 		// zapamiÍtujemy nowy indeks
	UART_RxBuf0[tmp_head0] = data0; 	// wpisujemy odebrany bajt do bufora
    }
}

// definiujemy procedurÍ obs≥ugi przerwania odbiorczego, zapisujπcπ dane do bufora cyklicznego
ISR( USART1_RX_vect ) {
    uint8_t tmp_head1;
    char data1;

    data1 = UDR1; //pobieramy natychmiast bajt danych z bufora sprzÍtowego

    // obliczamy nowy indeks Ñg≥owy wÍøaî
    tmp_head1 = ( UART_RxHead1 + 1) & UART_RX_BUF_MASK;

    // sprawdzamy, czy wπø nie zacznie zjadaÊ w≥asnego ogona
    if ( tmp_head1 == UART_RxTail1 ) {
    	// tutaj moøemy w jakiú wygodny dla nas sposÛb obs≥uøyÊ  b≥πd spowodowany
    	// prÛbπ nadpisania danych w buforze, mog≥oby dojúÊ do sytuacji gdzie
    	// nasz wπø zaczπ≥by zjadaÊ w≥asny ogon
    } else {
	UART_RxHead1 = tmp_head1; 		// zapamiÍtujemy nowy indeks
	UART_RxBuf1[tmp_head1] = data1; 	// wpisujemy odebrany bajt do bufora
    }
}
