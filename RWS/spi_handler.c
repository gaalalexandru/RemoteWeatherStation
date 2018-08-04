/*
 * spi_handler.c
 *
 * Created: 8/3/2018 11:48:19 PM
 *  Author: alexandru.gaal
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdbool.h>
#include "configuration.h"
#include "spi_handler.h"
/************************************************************************/
/*                 Initialization Function definitions                  */
/************************************************************************/

/* SPI Initialization function*/
void spi_init(void)
{
	//For operating in master mode, MOSI, SS, SCK pins must be set as output
	SPI_PORT |= (1<<SPI_MOSI_PIN)|(1<<SPI_SS_PIN)|(1<<SPI_SCK_PIN);
	SPCR = 0;
	SPSR = 0;
	SPSR |= (1<<SPI2X);
	SPCR |= (1<<SPE)|(1<<MSTR)/*|(1<<CPOL)|(1<<CPHA)*/|(1<<SPR0)|(1<<SPR1);
}

/************************************************************************/
/*                      Output Function definitions                     */
/************************************************************************/

/* SPI Send 1 Byte function */
void spi_send_char(uint8_t u8data)
{
	SPDR = u8data;
	while(!(SPSR & (1<<SPIF))){}
}

/************************************************************************/
/*                      Input Function definitions                      */
/************************************************************************/

/* SPI Get 1 Byte function */
uint8_t spi_get_char( void )
{
	while(!(SPSR & (1<<SPIF))){}
	return SPDR;	
}

/************************************************************************/
/*                      Other Function definitions                      */
/************************************************************************/


/************************************************************************/
/*                    Interrupt Function definitions                    */
/************************************************************************/