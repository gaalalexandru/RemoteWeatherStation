 /*
 * timer_handler.c
 *
 * Created: 10/5/2017 9:31:23 PM
 *  Author: Gaal Alexandru
 */

//Select timer0 (8 bit) to use for PWM generation
//Select timer1 (16 bit) to handle status LED
//Select timer2 (8 bit) to use for millisecond counter & delay functionality

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "configuration.h"
#include "timer_handler.h"

/************************************************************************/
/*	                          Global Variables                          */
/************************************************************************/
volatile uint32_t timer_system_ms = 0;  //system startup counter in milliseconds

/************************************************************************/
/*	                  Timer Initialization Functions                    */
/************************************************************************/

/* Timer0 - 8bit Initialization function*/
#ifdef USE_TIMER_0
void timer0_init(void)
{
	TCNT0 = 127;  //to half the timer interval
	#if ATMEGA48
	TCCR0B = (1 << CS01);  //prescaler 8 (timer0 clock = system clock / 8)
	TIMSK0  |= (1 << TOIE0);  //Timer0 Overflow Interrupt Enable
	#elif ATMEGA8
	TCCR0 = (1 << CS01);  //prescaler 8 (timer0 clock = system clock / 8)
	TIMSK  |= (1 << TOIE0);  //Timer0 Overflow Interrupt Enable
	#endif
}
#endif //USE_TIMER_0


/* Timer1 - 16bit Initialization function*/
#ifdef USE_TIMER_1
void timer1_init(void)
{
	TCCR1A = 0;
	//Clear Timer on Compare mode and /1024 prescaler
	//timer1 clock = system clock / 1024
	TCCR1B = (1 << WGM12)|(1 << CS12)|(1 << CS10);
	TCNT1 = 0;
	//XYZ clock cycles is equivalent to 0.1 s with the following setup:
	
	//A: system clock 8 Mhz
		//timer1 clock prescaler (divider) = 1024 => timer1 clock 7812.5 Hz
		//8000000 / 1024 = 7812.5 (1 second)
		//7812.5 / 2 = 3906.25 (0.5 second)
		//7812.5 / 10 = 781.25 (0.1 second)
			
	//B: system clock 4 Mhz
		//timer1 clock prescaler (divider) = 1024 => timer1 clock 3906.25 Hz
		//4000000 / 1024 = 3906.25 (1 second)
		//3906 / 2 = 1953 (0.5 second)
		//3906 / 10 = 390 (0.1 second)
		
	OCR1A = 390;  //0.1 seconds
	#if ATMEGA48
	TIMSK1 |= (1 << OCIE1A);  //Timer1 Output Compare A Match Interrupt Enable
	#elif ATMEGA8
	TIMSK  |= (1 << OCIE1A);  //Timer1 Output Compare A Match Interrupt Enable
	#endif
}
#endif //USE_TIMER_1

/* Timer2 - 8bit Initialization function*/
void timer2_init(void)
{
	//Clear Timer on Compare mode and /64 prescaler, OC pin disabled
	//timer2 clock = system clock / 64
	#if ATMEGA48
	TCCR2A = (1 << WGM21);
	//TCCR2B = (1 << CS22);  //for 64 prescaler
	TCCR2B = (1 << CS21)|(1 << CS20);  //for 32 prescaler
	#elif ATMEGA8
	TCCR2 = (1 << WGM21)|(1 << CS21)|(1 << CS20);
	#endif
	TCNT2 = 0;
	
	//XYZ clock cycles is equivalent to 1 ms with the following setup:

	//A: system clock 8 MHz
		//timer2 clock prescaler (divider) = 64 => timer1 clock 125 kHz
		//8000000 / 64 = 125000 (1 second)
		// 125000 / 1000 = 125 (1 millisecond)
		
	//B: system clock 4 MHz
		//timer2 clock prescaler (divider) = 32 => timer1 clock 125 kHz
		//4000000 / 32 = 125000 (1 second)
		// 125000 / 1000 = 125 (1 millisecond)
		

			
	#if ATMEGA48
	OCR2A = 125;
	TIMSK2  |= (1 << OCIE2A);  //Enable Timer1 output compare trigger OCIE2A	
	#elif ATMEGA8
	OCR2 = 125;
	TIMSK  |= (1 << OCIE2);  //Enable Timer1 output compare trigger OCIE2
	#endif //ATMEGA8
}

/************************************************************************/
/*	                 Timer Delay / Counter Functions                    */
/************************************************************************/
/* Millisecond wait function*/
void timer_delay_ms(uint32_t delay)
{
	//HINT: To increase time accuracy use a 0.97 coefficient on target_ms or delay 
	//Lack of accuracy of ~ 3.5% possible due to internal oscillator
	uint32_t target_ms = timer_ms() + (uint32_t)delay;
	while(timer_ms() < target_ms) { /*Wait*/ }
}

/* Millisecond counter function since system start-up*/
inline uint32_t timer_ms(void)
{
	//Not necessary to make atomic operation since it's 
	//a short and fast function and it is not critical if
	//returned value is +/- 1 ms in this case
	return timer_system_ms;
}

/************************************************************************/
/*	               Timer Interrupt Service Routines                     */
/************************************************************************/

#ifdef USE_TIMER_0
/* Timer0 Overflow Interrupt function*/
ISR (TIMER0_OVF_vect)
{
	TCNT0 = 127;
	//pwm_update();
}

#endif //USE_TIMER_0

#ifdef USE_TIMER_1
/* Timer1 Compare Match A Interrupt function*/
ISR (TIMER1_COMPA_vect)
{
	//TOGGLE_STATUS_LED; @AleGaa not valid anymore
	//status_led_update();
}
#endif //USE_TIMER_1

/* Timer2 Interrupt function*/
#if ATMEGA48
ISR (TIMER2_COMPA_vect)
#elif ATMEGA8
ISR (TIMER2_COMP_vect)
#endif
{
	timer_system_ms++; //increment every 1 ms
}




