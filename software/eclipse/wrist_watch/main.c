/*
 * main.c
 *
 *  Created on: 25-02-2017
 *      Author: Jacek
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>

#include "MKUART/mkuart.h"
#include "I2C_TWI/i2c_twi.h"   //biblioteka Mikrokontrolery AVR Jêzyk C Podstawy programowania M. Kardaœ
#include "ADXL_345/adxl_345.h"
#include "PetitFS/diskio.h"
#include "PetitFS/pff.h"
#include "DS3231/ds3231.h"
#include "xitoa.h"

#define KL1 (1<<PD4)
#define KL2 (1<<PD5)

#define KL KL1|KL2

#define LED (1<<PC0)
#define ACS (1<<PC3)

#define MUTE 	PORTB |= (1<<PB0);
#define UNMUTE	PORTB &= ~(1<<PB0);
#define SD_SLOT_EMPTY (PIND & (1<<PD6))

// obs³uga Timer0 z preskalerem = 8
#define TMR_START TCCR0B |= (1<<CS01)
#define TMR_STOP TCCR0B &= ~(1<<CS01)
//// obs³uga Timer0 z preskalerem = 64
#define TMR64_START TCCR0B |= (1<<CS01)|(1<<CS00)
#define TMR64_STOP TCCR0B &= ~((1<<CS01)|(1<<CS00))

//*************** makra i zmienne na potrzeby obs³ugi PetitFAT
#define SCK 	PB5
#define MOSI 	PB3
#define MISO 	PB4
#define CS 		PD7

#define FCC(c1,c2,c3,c4)	(((DWORD)c4<<24)+((DWORD)c3<<16)+((WORD)c2<<8)+(BYTE)c1)	/* FourCC */

// definicja struktury z parametrami WAV
typedef struct {
	uint8_t prescaler :1;
	uint8_t resolution;
	uint16_t khz;
} _FLAGS;

volatile _FLAGS FLAGS; // definicja struktury

volatile uint8_t can_read;

FATFS Fs; /* File system object */
DIR Dir; /* Directory object */
FILINFO Fno; /* File information */

WORD rb;

static DWORD load_header(void);
static UINT play(const char *dir, const char *fn);

#define BUF_SIZE 512			// maksymalny rozmiar pojedynczego bufora
uint8_t buf[2][BUF_SIZE]; // podwójny bufor do odczytu z karty SD

volatile uint8_t nr_buf; // indeks aktywnego buforu

/************** funkcja SuperDebounce do obs³ugi pojedynczych klawiszy ***************/
void SuperDebounce(uint8_t * key_state, volatile uint8_t *KPIN,
		uint8_t key_mask1, uint8_t key_mask2, uint16_t rep_time, uint16_t rep_wait,
		void (*push_proc)(void), void (*rep_proc)(void) );

void kl1_press(void);
void kl1_rep(void);
void kl2_press(void);
void kl2_rep(void);

void kl_press(void);
void kl_rep(void);

uint16_t pomiar(uint8_t kanal);

volatile uint16_t Timer1, Timer2, Timer3, Timer4;	/* timery programowe 100Hz */
volatile uint8_t Timer5; // chrg

uint8_t menu=0;

BYTE Buff[256];

char pbuf[7];	//bufor do nazw WAV

#define CHRG (!(PINC & (1<<PC1)))

#define LED_ON	PORTC |= (1<<PC0)
#define LED_OFF	PORTC &= ~(1<<PC0)

uint8_t bat_proc(void);

//void charging(void);
void bat_charg(void);

uint8_t godz;
uint8_t min;
uint8_t bat_p;
int main(void) {

	USART_Init( __UBRR );			// inicjalizacja UART

	 /*----------------------------- inicjalizacja ADC  ---------------*/
	ADMUX |= (1<<REFS1) | (1<<REFS0); // REF = 2.56V
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //w³¹czenie ADC, preskaler = 128

    /*----------------------------- inicjalizacja przerwañ  ---------------*/
	// konfiguracja Timer0 (samplowanie)
	TCCR0A = (1 << WGM01); // tryb CTC
	TIMSK0 = (1 << OCIE0A); // zezwolenie na przerwanie CompareMatch

	// konfiguracja PWM (Timer1) noœna
	TCCR1A = (1 << WGM10) | (1 << COM1A1) | (0 << COM1A0);
//	TCCR1B = (1 << CS10);

   	/* Timer2 – inicjalizacja przerwania co 10ms */
   	TCCR2A 	|= (1<<WGM21);			// tryb pracy CTC
   	TCCR2B 	|= (1<<CS22)|(1<<CS21)|(1<<CS20);	// preskaler = 1024
   	OCR2A 	= 117;					// przerwanie porównania co 10ms (100Hz)
   	TIMSK2 	|= (1<<OCIE2A);			// Odblokowanie przerwania CompareMatch



	sei();							// globalne odblokowanie przerwañ

	 /*----------------------------- inicjalizacja DS3231  ---------------*/
	DS3231_init();
	DS3231_set_time(13,31,0);
	DS3231_set_date(16,2,25,6);

	 /*----------------------------- inicjalizacja ADXL345  ---------------*/
    i2cSetBitrate( 100 );     // ustawienie prêdkoœæ 100 kHz na magistrali I2C
	DDRC |= ACS;
	PORTC |= ACS;
	adxl345_init();
//	display_adxl345_connect_test();

	// init SPI
	PORTB |= MISO; // podci¹gniêcie miso do VCC
	DDRB |= (1 << MOSI) | (1 << SCK);
	DDRD |= (1 << CS);
	PORTD |= (1 << CS);	//SD OFF
	DDRB |= (1<<PB2);	//NRF CS OUT
	PORTB |= (1<<PB2);	//NRF OFF

	SPCR |= (1 << SPE) | (1 << MSTR);
	SPSR |= (1 << SPI2X); // masymalny zegar SCK


	 /*----------------------------- inicjalizacja wzmacniacza  ---------------*/
//	DDRB |= (1 << PB1); // ustaw piny PWM1 (OC1A)
	DDRB |= (1 << PB0); // ustaw pin wzmacniacza
	MUTE;




	 /*----------------------------- inicjalizacja karty SD  ---------------*/

	PORTD |= (1 << PD6);	//podci¹gniêcie DETECT

	disk_initialize();
	pf_mount(&Fs); /* Initialize FS */
//	pf_opendir(&Dir, ""); /* Open sound file directory (root dir) */


	uart_putc('\r');			// wyœlij znak CR (enter)
	uart_putc('\n');			// wyœlij znak LF (nowa linia)
	uart_puts("START");	// wyœlij tekst
	uart_putc('\r');			// wyœlij znak CR (enter)
	uart_putc('\n');			// wyœlij znak LF (nowa linia)

	DDRC |= LED;	//inicjalizacja LED
	PORTD |= KL1|KL2;	// podci¹gniêcie przycisków

	PORTC |= (1<<PC1);	//podci¹gniêcie CHRG

	uint8_t k1=0, k2=0, k=0;  // zmienne pomocnicze do przechowywania stanu klawiszy
	uint8_t gest = 0;
	UNMUTE;

//	uint8_t a=1;
//	char pbuf[7];
	uint8_t charging =0;
	int ax=0;
	int ay=0;
	int az=0;

//	uint8_t cmd = 0;
	while(1) {

//		play("aku","baterii.wav");
		SuperDebounce(&k1, &PIND, KL1, KL2, 100, 50, kl1_press, kl1_rep );
		SuperDebounce(&k2, &PIND, KL2, KL1, 100, 50, kl2_press, kl2_rep );
		SuperDebounce(&k, &PIND, KL, 0 , 100, 50, kl_press, kl_rep );

//		if(CHRG) LED_ON;
//		else LED_OFF;

		if(!Timer2) {	//akcelerometr
			Timer2=100;
			adxl345_measure_read();       //odczyt pomiarów
//			adxl345_display_measure();

			if(!ax && !ay && !az) {
				ax=x;
				ay=y;
				az=z;
			}
//			uart_puts("X1=");	// wyœlij tekst
//			uart_putint(ax, 10);	// wyœlij liczbê
//			uart_puts(" Y1=");	// wyœlij tekst
//			uart_putint(ay, 10);	// wyœlij liczbê
//			uart_puts(" Z1=");	// wyœlij tekst
//			uart_putint(az, 10);	// wyœlij liczbê
//			uart_putc('\r');			// wyœlij znak CR (enter)
//			uart_putc('\n');			// wyœlij znak LF (nowa linia)
//			uart_putc('\r');			// wyœlij znak CR (enter)
//			uart_putc('\n');			// wyœlij znak LF (nowa linia)

			if( ((x-20) > ax) || ((x+20) < ax) || ((y-20) > ay) || ((y+20) < ay) || ((z-20) > az) || ((z+20) < az)) if((!Timer5) && charging) bat_charg();

			ax=x;
			ay=y;
			az=z;

			if(((y > -100) | (y<100)) & (x < -100) ) {
				if(gest) {
					gest=0;
					PORTC |= LED;
					Timer3=0;
					uart_puts("PLAY");	// wyœlij tekst
					uart_putc('\r');			// wyœlij znak CR (enter)
					uart_putc('\n');			// wyœlij znak LF (nowa linia)
					PORTC &= ~LED;

				} else {
				Timer3=50; //wykryto pocz¹tek gestu
				}
			}
		}
		if(Timer3>0) {
			if(((y > -100) | (y<100)) & (x > 190) )	 {
				gest=1;
			}
		} else gest=0;

//		DS3231_get_datetime(&datetime);

//		sprintf(pbuf, "%02d.wav", 1);
//		uart_puts(pbuf);	// wyœlij tekst
//		uart_putc('\r');			// wyœlij znak CR (enter)
//		uart_putc('\n');			// wyœlij znak LF (nowa linia)
//		play("godz","01.wav");
//		play("","baterii.wav");
//		sprintf(pbuf, "%02d.wav", 2);
//		uart_puts(pbuf);	// wyœlij tekst
//		uart_putc('\r');			// wyœlij znak CR (enter)
//		uart_putc('\n');			// wyœlij znak LF (nowa linia)
//		play("min",pbuf);

//		if(!Timer4) {
//
//
//		}

//		sprintf(pbuf, "%d.wav", a);
//		pf_opendir(&Dir, ""); /* Open sound file directory (root dir) */
//				uart_puts("PLAY");	// wyœlij tekst
//				uart_putc('\r');			// wyœlij znak CR (enter)
//				uart_putc('\n');			// wyœlij znak LF (nowa linia)
////					play("godz","sasiedzi.wav");
//				play("",pbuf);
//					play("","baterii.wav");
//
//		}

		if(!Timer5) if(CHRG) Timer5=20; //wykryto ladowarke

		if(Timer5 == 1) if(CHRG) if(!charging) {
			charging = 1;
			uart_puts("lad podl");	// wyœlij tekst
			uart_putc('\r');			// wyœlij znak CR (enter)
			uart_putc('\n');			// wyœlij znak LF (nowa linia)
			play("poz","lad_pod.wav");
		}
		if(!CHRG) if(!Timer5) if(charging){
			charging = 0;
			uart_puts("lad odl");	// wyœlij tekst
			uart_putc('\r');			// wyœlij znak CR (enter)
			uart_putc('\n');			// wyœlij znak LF (nowa linia)
			play("poz","lad_odl.wav");
			LED_OFF;
			Timer5=20;
		}



	}


}
ISR(TIMER2_COMPA_vect)
{
	uint8_t n;

	n = Timer1;		/* 100Hz Timer1 */
	if (n) Timer1 = --n;
	n = Timer2;		/* 100Hz Timer2 */
	if (n) Timer2 = --n;
	n = Timer3;		/* 100Hz Timer3 */
	if (n) Timer3 = --n;
	n = Timer4;		/* 100Hz Timer4 */
	if (n) Timer4 = --n;
	n = Timer5;		/* 100Hz Timer4 */
	if (n) Timer5 = --n;
}


void kl2_press(void) {
	switch(menu) {
	case 0:
		DS3231_get_datetime(&datetime);
		play("godz","godzina.wav");
		sprintf(pbuf, "%02d.wav", DS3231_get_hour(&datetime));
		play("godz",pbuf);
		sprintf(pbuf, "%02d.wav", DS3231_get_min(&datetime));
		play("min",pbuf);
		break;
	case 1: if(godz <= 22 ) godz++;
			else godz=0;
		sprintf(pbuf, "%02d.wav", godz);
		play("godz",pbuf);
		break;
	case 2:	if(min <= 58 ) min++;
			else min=0;
		sprintf(pbuf, "%02d.wav", min);
		play("min",pbuf);
		break;
	}
}

void kl2_rep(void) {
	switch(menu) {
	case 0: menu++;
			play("poz","zm_godz.wav");
			DS3231_get_datetime(&datetime);
			godz = DS3231_get_hour(&datetime);
			sprintf(pbuf, "%02d.wav", godz);
			play("godz",pbuf);
		break;
	case 1:
		menu++;
		play("poz","zm_min.wav");
		DS3231_get_datetime(&datetime);
		min = DS3231_get_min(&datetime);
		sprintf(pbuf, "%02d.wav", min);
		play("min",pbuf);
		break;
	case 2:
		menu=0;
		play("poz","nowy.wav");
		break;
	}
}

void kl1_press(void) {
	switch(menu) {
	case 0:
		bat_p = bat_proc();
		if(bat_p == 11) bat_p=10;

		sprintf(pbuf, "%d0.wav", bat_p);
		play("aku",pbuf);
		break;
	case 1: if(godz >=1 ) godz--;
			else godz=23;
		sprintf(pbuf, "%02d.wav", godz);
		play("godz",pbuf);
		break;
	case 2:	if(min >= 1 ) min--;
			else min=59;
		sprintf(pbuf, "%02d.wav", min);
		play("min",pbuf);
		break;
	}
}

void kl1_rep(void) {
	switch(menu) {
	case 0:
		break;
	case 1:
		menu++;
		play("poz","zm_min");
		DS3231_get_datetime(&datetime);
		min = DS3231_get_min(&datetime);
		sprintf(pbuf, "%02d.wav", min);
		play("min",pbuf);
		break;
	case 2:
		menu=0;
		DS3231_set_time(godz, min,0);
		play("poz","now_czas_ust.wav");
		break;
	}
}

void kl_press(void) {
	switch(menu) {
	case 0:
		break;
	case 1:
		break;
	case 2:
		break;
	}

}

void kl_rep(void) {
}


/************** funkcja SuperDebounce do obs³ugi pojedynczych klawiszy ***************
 * 							AUTOR: Miros³aw Kardaœ
 * ZALETY:
 * 		- nie wprowadza najmniejszego spowalnienia
 * 		- posiada funkcjê REPEAT (powtarzanie akcji dla d³u¿ej wciœniêtego przycisku)
 * 		- mo¿na przydzieliæ ró¿ne akcje dla trybu REPEAT i pojedynczego klikniêcia
 * 		- mo¿na przydzieliæ tylko jedn¹ akcjê wtedy w miejsce drugiej przekazujemy 0 (NULL)
 *
 * Wymagania:
 * 	Timer programowy utworzony w oparciu o Timer sprzêtowy (przerwanie 100Hz)
 *
 * 	Parametry wejœciowe:
 * 	*key_state - wskaŸnik na zmienn¹ w pamiêci RAM (1 bajt)
 *  *KPIN - nazwa PINx portu na którym umieszczony jest klawisz, np: PINB
 *  key_mask1 - maska klawisza lub klawiszy np: (1<<PB3)
 *  key_mask2 - maska pozosta³ego klawisza lub klawiszy np: (1<<PB3)
 *  rep_time - czas powtarzania funkcji rep_proc w trybie REPEAT
 *  rep_wait - czas oczekiwania do przejœcia do trybu REPEAT
 *  push_proc - wskaŸnik do w³asnej funkcji wywo³ywanej raz po zwolenieniu przycisku
 *  rep_proc - wskaŸnik do w³asnej funkcji wykonywanej w trybie REPEAT
 **************************************************************************************/
void SuperDebounce(uint8_t * key_state, volatile uint8_t *KPIN,
		uint8_t key_mask1, uint8_t key_mask2, uint16_t rep_time, uint16_t rep_wait,
		void (*push_proc)(void), void (*rep_proc)(void) ) {

	enum {idle, debounce, go_rep, wait_rep, rep};

	if(!rep_time) rep_time=20;
	if(!rep_wait) rep_wait=150;

	uint8_t key_press = 0;
	if(key_mask2)  {
		key_press = ((!(*KPIN & key_mask1)) && (*KPIN & key_mask2));
		if(*key_state>0) if(!(*KPIN & key_mask2)) *key_state = 0;
	}
	if(!key_mask2) key_press = !(*KPIN & key_mask1);


	if( key_press && !*key_state ) {
		*key_state = debounce;
		Timer1 = 15;
	} else
	if( *key_state  ) {

		if( key_press && debounce==*key_state && !Timer1 ) {
			*key_state = 2;
			Timer1=5;
		} else
		if( !key_press && *key_state>1 && *key_state<4 ) {
			if(push_proc) push_proc();						/* KEY_UP */
			*key_state=idle;
		} else
		if( key_press && go_rep==*key_state && !Timer1 ) {
			*key_state = wait_rep;
			Timer1=rep_wait;
		} else
		if( key_press && wait_rep==*key_state && !Timer1 ) {
			*key_state = rep;
		} else
		if( key_press && rep==*key_state && !Timer1 ) {
			Timer1 = rep_time;
			if(rep_proc) rep_proc();						/* KEY_REP */
		}
	}
	if( *key_state>=3 && !key_press ) *key_state = idle;
}

//***************** przerwanie TIMER0 - samplowanie ******************************************
ISR(TIMER0_COMPA_vect) {

	static uint16_t buf_idx; // indeks w pojedynczym buforze
	static uint8_t v1; // zmienna do przechowywania próbek

	// jeœli próbki monofoniczne
	v1 = buf[nr_buf][buf_idx++]; // pobieramy próbkê MONO do zmiennej v1

	OCR1A = v1; // próbka na wyjœcie PWM1, kana³ L

	if (buf_idx > BUF_SIZE - 1) {
		buf_idx = 0; // reset indeksu bufora
		can_read = 1; // flaga = 1
		nr_buf ^= 0x01; // zmiana bufora na kolejny
	}

}
// *************************** koniec przerwania ****************************************

/* 0:Invalid format, 1:I/O error, >1:Number of samples */
static DWORD load_header(void) {
	DWORD sz;
	uint8_t *wsk_buf = &buf[0][0];

	if (pf_read(wsk_buf, 12, &rb))
		return 1; /* Load file header (12 bytes) */

	if (rb != 12 || LD_DWORD(wsk_buf+8) != FCC('W','A','V','E'))
		return 0;

	for (;;) {
		pf_read(wsk_buf, 8, &rb); /* Get Chunk ID and size */
		if (rb != 8)
			return 0;
		sz = LD_DWORD(&wsk_buf[4]); /* Chunk size */

		switch (LD_DWORD(&wsk_buf[0])) { /* FCC */
		case FCC('f','m','t',' '): /* 'fmt ' chunk */
			if (sz > 100 || sz < 16)
				return 0; /* Check chunk size */

			pf_read(wsk_buf, sz, &rb); /* Get content */

			if (rb != sz)
				return 0;

			OCR0A = 67; // obliczona wartoœæ OCR0
			break;

		case FCC('d','a','t','a'): /* 'data' chunk (start to PLAY) */
			return sz;

		case FCC('L','I','S','T'): /* 'LIST' chunk (skip) */
		case FCC('f','a','c','t'): /* 'fact' chunk (skip) */
			pf_lseek(Fs.fptr + sz);
			break;

		default: /* Unknown chunk (error) */
			return 0;
		}
	}
	return 0;
}

// ******************  funkcja  P L A Y  ********************************
static UINT play(const char *dir, const char *fn) {

	DWORD sz;
	FRESULT res;
	Buff[0] = 0;
	xsprintf((char*)Buff, PSTR("%s/%s"), dir, fn);
	res = pf_open((char*)Buff);		/* Open sound file */

	if (res == FR_OK) {

		sz = load_header(); /* Load file header */
		if (sz < 256)
			return (UINT) sz;

		pf_lseek(0);

		pf_read(&buf[0][0], BUF_SIZE, &rb); // za³aduj pierwsz¹ czêœæ bufora
		pf_read(&buf[1][0], BUF_SIZE, &rb); // za³aduj drug¹ czêœæ bufora


		if (!FLAGS.prescaler) TMR_START; // start Timera0 (samplowanie) z
		else TMR64_START; // preskalerem zale¿nym od czêstotliwoœci
//		UNMUTE;
//		OCR1A = 128;
		TCCR1B |= (1 << CS10);
		DDRB |= (1<<PB1);

		while (1) {
			if (can_read) { // jeœli flaga ustawiona w obs³udze przerwania

				pf_read(&buf[nr_buf ^ 0x01][0], BUF_SIZE, &rb); // odczytaj kolejny bufor
				if (rb < BUF_SIZE)
					break; // jeœli koniec pliku przerwij pêtlê while(1)
				can_read = 0;
			}
		}
		DDRB &= ~(1<<PB1);
		OCR1A = 128;
		TCCR1B &= ~(1 << CS10);

//		MUTE;
		if (!FLAGS.prescaler)
			TMR_STOP; // wy³¹czenie Timera0 (samplowania)
		else TMR64_STOP;
	}
	return res;
}
uint16_t pomiar(uint8_t kanal)
{
	ADMUX = (ADMUX & 0xF8) | kanal;
	ADCSRA |= (1<<ADSC);
	while( ADCSRA & (1<<ADSC) );
	return ADCW;
}
uint8_t bat_proc(void){

	uint8_t vol=0;

	if(!Timer4) {

	uint32_t pm = pomiar(7);

	pm *= 86;
	pm /= 100;
//	uart_putint(pm,10);
//	uart_putc('\r');			// wyœlij znak CR (enter)
	uart_putc('\n');
	// wyœlij znak LF (nowa linia)


	if(pm >= 418) {
		Timer4 = 3000;
		vol = 11;
	}
	if( (pm < 418) && (pm >= 400) ) vol = 10;
	if( (pm < 400) && (pm >= 395) ) vol = 9;
	if( (pm < 395) && (pm >= 390) ) vol = 8;
	if( (pm < 390) && (pm >= 380) ) vol = 7;
	if( (pm < 380) && (pm>= 377) ) vol = 6;
	if( (pm < 377) && (pm>= 372) ) vol = 5;
	if( (pm < 372) && (pm>= 368) ) vol = 4;
	if( (pm < 368) && (pm>= 365) ) vol = 3;
	if( (pm < 365) && (pm>= 360) ) vol = 2;
	if( (pm < 360) && (pm>= 344) ) vol = 1;
	if(pm < 344 )  vol = 0; //sprawdz czy dobrze jest
	} else vol =11;

	return vol;
}

void bat_charg(void){

	uint8_t vol = 0;
	vol = bat_proc();

	if(vol <= 10) {
	sprintf(pbuf, "%d0.wav", vol);
	uart_puts(pbuf);
	uart_putc('\r');			// wyœlij znak CR (enter)
	uart_putc('\n');			// wyœlij znak LF (nowa linia)
	play("aku", pbuf);
	} else 	play("poz", "lad_zak.wav");
}
