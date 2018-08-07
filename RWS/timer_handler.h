/*
 * timer_handler.h
 *
 * Created: 10/5/2017 9:32:37 PM
 *  Author: Gaal Alexandru
 */ 

#ifndef TIMEHDL_H_
#define TIMEHDL_H_

void timer0_init(void);
void timer1_init(void);
void timer2_init(void);

uint32_t timer_ms(void);
void timer_delay_ms(uint32_t delay);

#endif /* TIMEHDL_H_ */