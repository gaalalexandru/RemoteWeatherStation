/*
 * timer_handler.h
 *
 * Created: 10/5/2017 9:32:37 PM
 *  Author: Gaal Alexandru
 */ 

#ifndef TIMEHDL_H_
#define TIMEHDL_H_

typedef struct {
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
} st_date_time;

void timer0_init(void);
void timer1_init(void);
void timer2_init(void);
uint32_t timer_ms(void);
void timer_delay_ms(uint32_t delay);

#endif /* TIMEHDL_H_ */