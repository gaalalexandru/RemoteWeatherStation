/*
 * main.c
 *
 * Created: 8/3/2018 10:46:04 PM
 *  Author: alexandru.gaal
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include "spi_handler.h"

int main(void)
{
	cli();  //Disable interrupts
	
	
	sei();  // enable global interrupts
	
    while(1)
    {
        //TODO:: Please write your application code 
    }
}
