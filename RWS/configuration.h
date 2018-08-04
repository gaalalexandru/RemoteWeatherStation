/*
 * configuration.h
 *
 * Created: 8/3/2018 11:52:44 PM
 *  Author: alexandru.gaal
 */ 


#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <avr/io.h>
#include <avr/portpins.h>

#define FOSC 4000000// Clock Speed
#define F_CPU FOSC
//////////////////////////////////////////////////////////////////////////
//							STATUS LED CONFIGURATIONS					//
//////////////////////////////////////////////////////////////////////////
#define STATUS_LED_DDR		DDRD
#define STATUS_LED_PORT		PORTD
#define STATUS_LED_PIN		PIN4


//////////////////////////////////////////////////////////////////////////
//							UART CONFIGURATIONS							//
//////////////////////////////////////////////////////////////////////////
#define BAUD 38400 // Old value only for terminal control: 9600
#define SET_U2X (1)
#define DONT_USE_ISR_UART (0)
#if SET_U2X
#define MYUBRR ((FOSC/(8*BAUD))-1)
#else
#define MYUBRR ((FOSC/(16*BAUD))-1)
#endif

//////////////////////////////////////////////////////////////////////////
//							 SPI CONFIGURATIONS							//
//////////////////////////////////////////////////////////////////////////
#define SPI_USE_INT		(0)
#define SPI_DDR			DDRB
#define SPI_PORT		PORTB
#define SPI_MOSI_PIN	(3)
#define SPI_SS_PIN		(2)
#define SPI_MISO_PIN	(4)
#define SPI_SCK_PIN		(5)

#define CS_BME280_DDR	DDRB
#define CS_BME280_PORT	PORTB
#define CS_BME280_PIN	(0)

#define CS_LIS3MDL_DDRB	DDRB
#define CS_LIS3MDL_PORT	PORTB
#define CS_LIS3MDL_PIN	(1)

#define CS_FLASH_DDRB	DDRB
#define CS_FLASH_PORT	PORTD
#define CS_FLASH_PIN	(5)

#endif /* CONFIGURATION_H_ */