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

//////////////////////////////////////////////////////////////////////////
//							 SPI CONFIGURATIONS							//
//////////////////////////////////////////////////////////////////////////
#define SPI_PORT		PORTB
#define SPI_MOSI_PIN	(3)
#define SPI_SS_PIN		(2)
#define SPI_MISO_PIN	(4)
#define SPI_SCK_PIN		(5)

#define CS_BME280_PORT	PORTB
#define CS_BME280_PIN	(0)

#define CS_LIS3MDL_PORT	PORTB
#define CS_LIS3MDL_PIN	(1)

#define CS_FLASH_PORT	PORTD
#define CS_FLASH_PIN	(5)

#endif /* CONFIGURATION_H_ */