/*
 * spi_handler.c
 *
 * Created: 8/3/2018 11:48:19 PM
 *  Author: alexandru.gaal
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdbool.h>

#include "configuration.h"
#include "spi_handler.h"
#include <util/delay.h>
/************************************************************************/
/*                 Initialization Function definitions                  */
/************************************************************************/

/* SPI Initialization function*/
void spi_init(void)
{
	//For operating in master mode, MOSI, SS, SCK pins must be set as output
	SPI_DDR |= (1<<SPI_MOSI_PIN)|(1<<SPI_SS_PIN)|(1<<SPI_SCK_PIN);
	SPCR = 0;
	SPSR = 0;
	SPSR |= (1<<SPI2X);
	//SPI enabled, master mode
	//polarity & phase 00
	//spi clock pre-scaler = 64, see table bellow
	#if SPI_USE_INT
		SPCR |= (1<<SPIE)|(1<<SPE)|(1<<MSTR)/*|(1<<CPOL)|(1<<CPHA)*/|(1<<SPR0)|(1<<SPR1);
	#else
		SPCR |= (1<<SPE)|(1<<MSTR)/*|(1<<CPOL)|(1<<CPHA)*/|(1<<SPR0)|(1<<SPR1);
	#endif  //SPI_USE_INT
/*
SPI2	SPR1	SPR0	SCK Frequency
0		0		0		fosc/4
0		0		1		fosc/16
0		1		0		fosc/64
0		1		1		fosc/128
1		0		0		fosc/2
1		0		1		fosc/8
1		1		0		fosc/32
1		1		1		fosc/64
*/
}

/************************************************************************/
/*                      Output Function definitions                     */
/************************************************************************/

/* SPI Send 1 Byte function */
uint8_t spi_send_char(uint8_t u8data)
{
	SPDR = u8data;
	while(!(SPSR & (1<<SPIF))){}
	//_delay_ms(20);
	return SPDR;
}

/************************************************************************/
/*                      Input Function definitions                      */
/************************************************************************/

/* SPI Get 1 Byte function */
uint8_t spi_get_char( void )
{
	//while(!(SPSR & (1<<SPIF))){}
	return SPDR;
}

/************************************************************************/
/*                      Other Function definitions                      */
/************************************************************************/


/************************************************************************/
/*                    Interrupt Function definitions                    */
/************************************************************************/

// SPI Transmission/reception complete ISR
#if SPI_USE_INT
ISR(SPI_STC_vect)
{
	// Code to execute
	// whenever transmission/reception
	// is complete.
}
#endif  //SPI_USE_INT