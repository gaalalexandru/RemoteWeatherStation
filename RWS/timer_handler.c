 /*
 * timer_handler.c
 *
 * Created: 10/5/2017 9:31:23 PM
 *  Author: Gaal Alexandru
 */

//Select timer0 (8 bit) to use for low precision millisecond counter
//Select timer1 (16 bit) to handle status LED
//Select timer2 (8 bit) to use for high precision 1 second real time counter functionality                               

#define TIMER_LOG_ACTIV (1)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "configuration.h"
#include "timer_handler.h"
#if TIMER_LOG_ACTIV
#include "uart_handler.h"
#endif  //TIMER_LOG_ACTIV

#define USE_TIMER_0 (1)
#define USE_TIMER_1 (0)
#define USE_TIMER_2 (1)
#define TOGGLE_STATUS_LED	(STATUS_LED_PORT ^= (1 << STATUS_LED_PIN))
#define TIMER2_USE_EXTERNAL_CRYSTAL (1)  //Make it true, to activate clock source from external crystal


typedef struct {
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
} st_date_time;

/************************************************************************/
/*	                          Global Variables                          */
/************************************************************************/
volatile uint32_t timer_system_ms = 0;  //system startup counter in milliseconds
//volatile uint32_t timer_sec_rtc = 0;  //rtc second counter
volatile st_date_time timestamp = {18,9,16,0,39,0};  //timestamp
/************************************************************************/
/*	                  Timer Initialization Functions                    */
/************************************************************************/

/* Timer0 - 8bit Initialization function*/
#if USE_TIMER_0
void timer0_init(void)
{
	//B: system clock 4 Mhz
	//timer0 clock prescaler (divider) = 8 => timer1 clock 500 kHz
	//4000000 / 8 = 500000 (1 second)
	//500000 / 1000 = 500 (1 millisecond)
	//500 / 2 = 250 (0.5 millisecond)
	//use reload value 5 and in ISR increment ms counter every 2nd time
	TCNT0 = 5;  //to reduce the timer interval
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
#if USE_TIMER_1
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

/* Timer2 - 8bit Initialization function
 * use external crystal 32.768 kHz
 * timer2 clock prescaler (divider) = 128 => timer1 clock 256 Hz
 * 32768 / 128 = 256 (1 second)
 */
#if USE_TIMER_2
void timer2_init(void)
{
	#if TIMER2_USE_EXTERNAL_CRYSTAL
	ASSR |= (1<<AS2);
	#if ATMEGA48
	//TODO
	#elif ATMEGA8
	TCCR2 = (1 << CS22)|(1 << CS20);  //mode 0, 128 prescaler	
	#endif  //ATMEGA8
	while(ASSR & (1<<TCR2UB)) {}
	
	TCNT2 = 0;
	while(ASSR & (1<<TCN2UB)) {}
	#if ATMEGA48
		//TIMSK2  |= (1 << OCIE2A);  //Enable Timer1 output compare trigger OCIE2A	
	#elif ATMEGA8
		TIMSK  |= (1 << TOIE2)/*|(1 << OCIE2)*/;  //Enable Timer2 overflow interrupt
	#endif //ATMEGA8
	#endif //TIMER2_USE_EXTERNAL_CRYSTAL
}
#endif //USE_TIMER_2

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

#if USE_TIMER_0
/* Timer0 Overflow Interrupt function*/
ISR (TIMER0_OVF_vect)
{
	static uint8_t u8switch = 0;
	if(u8switch) {  //every 2nd time
		timer_system_ms++; //increment 1 ms
	}
	u8switch ^= 1; //toggle switch
	TCNT0 = 5;  //reset to 5
	
}
#endif //USE_TIMER_0



#if USE_TIMER_1
/* Timer1 Compare Match A Interrupt function*/
ISR (TIMER1_COMPA_vect)
{
	//TOGGLE_STATUS_LED; @AleGaa not valid anymore
	//status_led_update();
}
#endif //USE_TIMER_1

#if USE_TIMER_2
/* Timer2 Interrupt functions*/
/*Overflow ISR*/
ISR (TIMER2_OVF_vect)
{
	static uint8_t u8days;
	//timer_sec_rtc++;  //precision increment every second
	timestamp.second = (timestamp.second + 1) % 60;
	if (timestamp.second == 0) {  //1 minute passed
		timestamp.minute  = (timestamp.minute + 1) % 60;
		if (timestamp.minute == 0) {  //1 hour passed
			timestamp.hour  = (timestamp.hour + 1) % 24;
			if (timestamp.hour == 0) { //1 day passed
				switch(timestamp.month)
				{
					case 1:  //handler 31 day months
					case 3:
					case 5:
					case 7:
					case 8:
					case 10:
					case 12:
						u8days = 31;
					break;
					case 4:  //handler 30 day months
					case 6:
					case 9:
					case 11:
						u8days = 30;
					break;
					case 2:   //handle February and leap years
						if (timestamp.year % 4 != 0) {
							u8days = 28;
						} else {
							u8days = 29;
						}
				}
				timestamp.day = ((timestamp.day % u8days) + 1);
				if(timestamp.day == u8days) { //1 month passed
					timestamp.month = ((timestamp.month % 12) + 1);
					if(timestamp.month == 12) {
						timestamp.year++; //1 year passed
					}
				}
			}
		}
	}
	#if TIMER_LOG_ACTIV
	TOGGLE_STATUS_LED;
	//uart_send_udec(timer_sec_rtc); uart_newline();
	//uart_send_udec(timer_system_ms); uart_newline();
	uart_send_udec(20); uart_send_udec(timestamp.year); uart_send_string("/");
	uart_send_udec(timestamp.month); uart_send_string("/");
	uart_send_udec(timestamp.day); uart_send_string(" -> ");
	uart_send_udec(timestamp.hour); uart_send_string(":");
	uart_send_udec(timestamp.minute); uart_send_string(":");
	uart_send_udec(timestamp.second);
	uart_newline();
	#endif //TIMER_LOG_ACTIV
}

/*Compare value ISR*/
#if 0
	#if ATMEGA48
	ISR (TIMER2_COMPA_vect)
	#elif ATMEGA8
	ISR (TIMER2_COMP_vect)
	#endif
	{
		timer_system_ms++; //increment every 1 ms
	}
#endif
#endif //USE_TIMER_2
