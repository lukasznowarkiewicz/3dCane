/*----------------------------- KOMENDY UART  ---------------
 *	4 - power down
 *	5 - power up
 *	1,2,3 - czujniki
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include "MKUART/mkuart.h"
#include <avr/power.h>
#include <avr/sleep.h>
#include <stdio.h>
#include "xitoa.h"

volatile uint16_t PulseWidth, LastCapture; //Zmierzona szer sygnalu i pamiec zegara
uint16_t result, result0, result1, result2; //przeliczona odleg³osc (wynik pomiaru)
uint8_t czujnik=0,new=0;

//TRIGGER
void trigger(uint8_t cz) {

//	_delay_ms(80);

if(cz == 0){
 _delay_us(1);
 PORTB |= (1<<PB2);     //uruchomienie pomiaru SC-HC04
 _delay_us(10);
 PORTB &= ~(1<<PB2);
 _delay_us(20);
}
if(cz == 1){
 _delay_us(1);
 PORTD |= (1<<PD7);     //uruchomienie pomiaru SC-HC04
 _delay_us(10);
 PORTD &= ~(1<<PD7);
 _delay_us(20);
}
if(cz == 2){
 _delay_us(1);
 PORTD |= (1<<PD6);     //uruchomienie pomiaru SC-HC04
 _delay_us(10);
 PORTD &= ~(1<<PD6);
}

}

int main(void) {

	//WE/WY
	DDRD |= (1<<PD7) | (1<<PD6);       //jako wyjscie
	DDRB |= (1<<PB2);

	DDRB |= (1<<PB1);	// mosfet
	DDRD |= (1<<PD5);	// rezystor pull-down

	USART_Init(__UBRR); // inicjalizacja UART
	OSCCAL = 165;

	//TIMER1
	TCCR1B |= (1<<ICES1);           //Zbocze narastaj¹ce wywo³a przerwanie
	TCCR1B |= (1<<CS11);            // preskaler 8
	TIMSK  |=  (1<<TICIE1);         // zezwolenie na przerwanie od ICP
	wdt_enable(WDTO_30MS);

	uint8_t sleep = 0;

	ACSR = (1<<ACD);	//ADC OFF
//	MCUCR = (1<<SE);
	sei(); //globalne odblokowanie przerwan

//	uart_putc('\r'); // wyœlij znak CR (enter)
//	uart_putc('\n'); // wyœlij znak LF (nowa linia)
//	uart_puts("START"); // wyœlij tekst
//	uart_putc('\r'); // wyœlij znak CR (enter)
//	uart_putc('\n'); // wyœlij znak LF (nowa linia)
	int numDelays = 0;
	uint8_t cmd = 0;
	//PETLA GLOWNA
	while(1) {

		wdt_reset();
		if(!cmd) cmd = uart_getc(); // 4 - sleep 5- wake up
		else if(sleep && (cmd<4)) cmd=0;

		if(cmd>47) cmd = cmd-48;
		if(cmd == 4) {
			PORTB |= (1<<PB1);

//			uart_putint(4, 10);	// wyœlij liczbê 3
//			uart_putc('\r'); // wyœlij znak CR (enter)
//			uart_putc('\n'); // wyœlij znak LF (nowa linia)
			TIMSK  &=  ~(1<<TICIE1);         // zezwolenie na przerwanie od ICP
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			sleep_cpu() ;
			sleep_enable();
			sleep = 1;
			cmd = 0;
		}
		if(cmd == 5) {
			PORTB &= ~(1<<PB1);
//			uart_putint(5, 10);	// wyœlij liczbê 3
//			uart_putc('\r'); // wyœlij znak CR (enter)
//			uart_putc('\n'); // wyœlij znak LF (nowa linia)
			TIMSK  |=  (1<<TICIE1);         // zezwolenie na przerwanie od ICP
			sleep_disable() ;
			sleep = 0;
			cmd = 0;
		}

		if(!sleep) {

		_delay_ms(1);
		numDelays++;
		if (numDelays > 200) {

//			uart_putint(123, 10);	// wyœlij czujnik 1
//			uart_puts("A");

			numDelays = 0;
			trigger(czujnik);                      //wywo³anie pomiaru
			czujnik++;
			if(czujnik == 3 ) czujnik = 0;
//			uart_puts("3");
//			if(!czujnik) {

				switch(cmd) {
				case 1: uart_putint(result0, 10);	// wyœlij czujnik 1
					break;
				case 2: uart_putint(result1, 10);	// wyœlij liczbê 2
					break;
				case 3:	uart_putint(result2, 10);	// wyœlij liczbê 3
					break;
				}

				if(cmd) {
					uart_puts("A");
					cmd = 0;
				}

		}
		}
}
}


	//PRZERWANIE OD CAPTURE PIN
	ISR(TIMER1_CAPT_vect) {
	if( (TCCR1B & (1<<ICES1)) ) {
	        LastCapture = ICR1;                             //jesli zbocze narastajace, zlap ICR1
	} else {												//Jeœli zbocze opadajace oblicz PW
	        PulseWidth = ICR1 - LastCapture;
	    	result = PulseWidth / 58; //obliczenie dystansu w cm
	    	if(czujnik == 0 ) result0 = result;
	    	if(czujnik == 1 ) result1 = result;
	    	if(czujnik == 2 ) result2 = result;
	}
	TCCR1B ^= (1<<ICES1);                                   //Zmiana zbocza wyw przerwanie
	}

