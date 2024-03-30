/*
 * mkuart.c
 *
 *  Created on: 2010-09-04
 *       Autor: Mirosław Kardaś
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>


#include "mkuart.h"

char uart=0;

// definiujemy w ko�cu nasz bufor UART_RxBuf
volatile char UART_RxBuf0[UART_RX_BUF_SIZE];
// definiujemy indeksy okre�laj�ce ilo�� danych w buforze
volatile uint8_t UART_RxHead0; // indeks oznaczaj�cy �g�ow� w�a�
volatile uint8_t UART_RxTail0; // indeks oznaczaj�cy �ogon w�a�



// definiujemy w ko�cu nasz bufor UART_RxBuf
volatile char UART_TxBuf0[UART_TX_BUF_SIZE];
// definiujemy indeksy okre�laj�ce ilo�� danych w buforze
volatile uint8_t UART_TxHead0; // indeks oznaczaj�cy �g�ow� w�a�
volatile uint8_t UART_TxTail0; // indeks oznaczaj�cy �ogon w�a�

// definiujemy w ko�cu nasz bufor UART_RxBuf
volatile char UART_RxBuf1[UART_RX_BUF_SIZE];
// definiujemy indeksy okre�laj�ce ilo�� danych w buforze
volatile uint8_t UART_RxHead1; // indeks oznaczaj�cy �g�ow� w�a�
volatile uint8_t UART_RxTail1; // indeks oznaczaj�cy �ogon w�a�



// definiujemy w ko�cu nasz bufor UART_RxBuf
volatile char UART_TxBuf1[UART_TX_BUF_SIZE];
// definiujemy indeksy okre�laj�ce ilo�� danych w buforze
volatile uint8_t UART_TxHead1; // indeks oznaczaj�cy �g�ow� w�a�
volatile uint8_t UART_TxTail1; // indeks oznaczaj�cy �ogon w�a�

void uart_set(char nr) {
	uart=nr;
}



void USART_Init( uint16_t baud ) {
	if(!uart) {
	/* Ustawienie pr�dko�ci */
	UBRR0H = (uint8_t)(baud>>8);
	UBRR0L = (uint8_t)baud;
	/* Za��czenie nadajnika I odbiornika */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Ustawienie format ramki: 8bit�w danych, 1 bit stopu */
	UCSR0C = (1<<UMSEL01)|(3<<UCSZ00);

		// je�li nie  korzystamy z interefejsu RS485
		UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	}
	else {
	/* Ustawienie pr�dko�ci */
	UBRR1H = (uint8_t)(baud>>8);
	UBRR1L = (uint8_t)baud;
	/* Za��czenie nadajnika I odbiornika */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	/* Ustawienie format ramki: 8bit�w danych, 1 bit stopu */
	UCSR1C = (1<<UMSEL11)|(3<<UCSZ10);

		// je�li nie  korzystamy z interefejsu RS485
		UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1);
	}
}

// definiujemy funkcj� dodaj�c� jeden bajtdoz bufora cyklicznego
void uart_putc( char data ) {

	if(!uart) {

	uint8_t tmp_head0;

    tmp_head0  = (UART_TxHead0 + 1) & UART_TX_BUF_MASK;

          // p�tla oczekuje je�eli brak miejsca w buforze cyklicznym na kolejne znaki
    while ( tmp_head0 == UART_TxTail0 );

    UART_TxBuf0[tmp_head0] = data;
    UART_TxHead0 = tmp_head0;

    // inicjalizujemy przerwanie wyst�puj�ce, gdy bufor jest pusty, dzi�ki
    // czemu w dalszej cz�ci wysy�aniem danych zajmie si� ju� procedura
    // obs�ugi przerwania
    UCSR0B |= (1<<UDRIE0);
	}
	else {

	uint8_t tmp_head1;

    tmp_head1  = (UART_TxHead1 + 1) & UART_TX_BUF_MASK;

          // p�tla oczekuje je�eli brak miejsca w buforze cyklicznym na kolejne znaki
    while ( tmp_head1 == UART_TxTail1 );

    UART_TxBuf1[tmp_head1] = data;
    UART_TxHead1 = tmp_head1;

    // inicjalizujemy przerwanie wyst�puj�ce, gdy bufor jest pusty, dzi�ki
    // czemu w dalszej cz�ci wysy�aniem danych zajmie si� ju� procedura
    // obs�ugi przerwania
    UCSR1B |= (1<<UDRIE1);
	}

}


void uart_puts(char *s)		// wysy�a �a�cuch z pami�ci RAM na UART
{
  register char c;
  while ((c = *s++)) uart_putc(c);			// dop�ki nie napotkasz 0 wysy�aj znak
}

void uart_putint(int value, int radix)	// wysy�a na port szeregowy tekst
{
	char string[17];			// bufor na wynik funkcji itoa
	itoa(value, string, radix);		// konwersja value na ASCII
	uart_puts(string);			// wy�lij string na port szeregowy
}


// definiujemy procedur� obs�ugi przerwania nadawczego, pobieraj�c� dane z bufora cyklicznego
ISR( USART0_UDRE_vect)  {
    // sprawdzamy czy indeksy s� r�ne
    if ( UART_TxHead0 != UART_TxTail0 ) {
    	// obliczamy i zapami�tujemy nowy indeks ogona w�a (mo�e si� zr�wna� z g�ow�)
    	UART_TxTail0 = (UART_TxTail0 + 1) & UART_TX_BUF_MASK;
    	// zwracamy bajt pobrany z bufora  jako rezultat funkcji
    	UDR0 = UART_TxBuf0[UART_TxTail0];
    } else {
	// zerujemy flag� przerwania wyst�puj�cego gdy bufor pusty
	UCSR0B &= ~(1<<UDRIE0);
    }
}

// definiujemy procedur� obs�ugi przerwania nadawczego, pobieraj�c� dane z bufora cyklicznego
ISR( USART1_UDRE_vect)  {
    // sprawdzamy czy indeksy s� r�ne
    if ( UART_TxHead1 != UART_TxTail1 ) {
    	// obliczamy i zapami�tujemy nowy indeks ogona w�a (mo�e si� zr�wna� z g�ow�)
    	UART_TxTail1 = (UART_TxTail1 + 1) & UART_TX_BUF_MASK;
    	// zwracamy bajt pobrany z bufora  jako rezultat funkcji
    	UDR1 = UART_TxBuf1[UART_TxTail1];
    } else {
	// zerujemy flag� przerwania wyst�puj�cego gdy bufor pusty
	UCSR1B &= ~(1<<UDRIE1);
    }
}


// definiujemy funkcj� pobieraj�c� jeden bajt z bufora cyklicznego
char uart_getc(void) {
	if(!uart) {
    // sprawdzamy czy indeksy s� r�wne
    if ( UART_RxHead0 == UART_RxTail0 ) return 0;

    // obliczamy i zapami�tujemy nowy indeks �ogona w�a� (mo�e si� zr�wna� z g�ow�)
    UART_RxTail0 = (UART_RxTail0 + 1) & UART_RX_BUF_MASK;
    // zwracamy bajt pobrany z bufora  jako rezultat funkcji
    return UART_RxBuf0[UART_RxTail0];
	}
	else {
    // sprawdzamy czy indeksy s� r�wne
    if ( UART_RxHead1 == UART_RxTail1 ) return 0;

    // obliczamy i zapami�tujemy nowy indeks �ogona w�a� (mo�e si� zr�wna� z g�ow�)
    UART_RxTail1 = (UART_RxTail1 + 1) & UART_RX_BUF_MASK;
    // zwracamy bajt pobrany z bufora  jako rezultat funkcji
    return UART_RxBuf1[UART_RxTail1];
	}
}

// definiujemy procedur� obs�ugi przerwania odbiorczego, zapisuj�c� dane do bufora cyklicznego
ISR( USART0_RX_vect ) {
    uint8_t tmp_head0;
    char data0;

    data0 = UDR0; //pobieramy natychmiast bajt danych z bufora sprz�towego

    // obliczamy nowy indeks �g�owy w�a�
    tmp_head0 = ( UART_RxHead0 + 1) & UART_RX_BUF_MASK;

    // sprawdzamy, czy w�� nie zacznie zjada� w�asnego ogona
    if ( tmp_head0 == UART_RxTail0 ) {
    	// tutaj mo�emy w jaki� wygodny dla nas spos�b obs�u�y�  b��d spowodowany
    	// pr�b� nadpisania danych w buforze, mog�oby doj�� do sytuacji gdzie
    	// nasz w�� zacz��by zjada� w�asny ogon
    } else {
	UART_RxHead0 = tmp_head0; 		// zapami�tujemy nowy indeks
	UART_RxBuf0[tmp_head0] = data0; 	// wpisujemy odebrany bajt do bufora
    }
}

// definiujemy procedur� obs�ugi przerwania odbiorczego, zapisuj�c� dane do bufora cyklicznego
ISR( USART1_RX_vect ) {
    uint8_t tmp_head1;
    char data1;

    data1 = UDR1; //pobieramy natychmiast bajt danych z bufora sprz�towego

    // obliczamy nowy indeks �g�owy w�a�
    tmp_head1 = ( UART_RxHead1 + 1) & UART_RX_BUF_MASK;

    // sprawdzamy, czy w�� nie zacznie zjada� w�asnego ogona
    if ( tmp_head1 == UART_RxTail1 ) {
    	// tutaj mo�emy w jaki� wygodny dla nas spos�b obs�u�y�  b��d spowodowany
    	// pr�b� nadpisania danych w buforze, mog�oby doj�� do sytuacji gdzie
    	// nasz w�� zacz��by zjada� w�asny ogon
    } else {
	UART_RxHead1 = tmp_head1; 		// zapami�tujemy nowy indeks
	UART_RxBuf1[tmp_head1] = data1; 	// wpisujemy odebrany bajt do bufora
    }
}
