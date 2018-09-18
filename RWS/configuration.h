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

//#define MAIN_LOG_ACTIV

//////////////////////////////////////////////////////////////////////////
//					CONTROLLER SELECTION AND CONFIGURATION				//
//////////////////////////////////////////////////////////////////////////
#if defined (__AVR_ATmega48PB__)  && !defined (__AVR_ATmega8__)
#define ATMEGA48 (1)
#define ATMEGA8	(0)
#elif defined (__AVR_ATmega8__)  && !defined (__AVR_ATmega48PB__)
#define ATMEGA48 (0)
#define ATMEGA8	(1)
#else
#error "Please specify one target controller"
#endif  //controller selection

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

#define USE_UART_DEC_OUTPUT
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

#define CS_LIS3MDL_DDR	DDRB
#define CS_LIS3MDL_PORT	PORTB
#define CS_LIS3MDL_PIN	(1)

#define CS_FLASH_DDR	DDRD
#define CS_FLASH_PORT	PORTD
#define CS_FLASH_PIN	(5)

//////////////////////////////////////////////////////////////////////////
//						 MEASUREMENTS CONFIGURATION						//
//////////////////////////////////////////////////////////////////////////
#define WEATHER_MONITORING_INTERVAL_MS (60000)
#define WEATHER_MONITORING_ACCELERATION (10)

#define DATA_FRAME_SIZE	(64)
#define DATA_FRAME_SEPARATOR (0x55)

#define POWER_SAVE_ACTIV (1)
#endif /* CONFIGURATION_H_ */