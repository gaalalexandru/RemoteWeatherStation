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

typedef enum {
	bme280 = 0,
	lis3mdl,
	flash
	} ten_spi_devices;
	
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
	
	//Init CS line for all SPI devices, as output
	INIT_CS_PIN(CS_BME280_DDR,CS_BME280_PIN);
	INIT_CS_PIN(CS_LIS3MDL_DDR,CS_LIS3MDL_PIN);
	INIT_CS_PIN(CS_FLASH_DDR,CS_FLASH_PIN);
	//Disable CS line for all SPI devices, by setting them high
	SET_CS_PIN(CS_BME280_PORT,CS_BME280_PIN);
	SET_CS_PIN(CS_LIS3MDL_PORT,CS_LIS3MDL_PIN);
	SET_CS_PIN(CS_FLASH_PORT,CS_FLASH_PIN);
}

/************************************************************************/
/*                SPI Send / Receive function definitions               */
/************************************************************************/
/* SPI Send and Receive 1 Byte function */
uint8_t spi_transfer_generic(uint8_t u8data)
{
	SPDR = u8data;
	while(!(SPSR & (1<<SPIF))){}
	return SPDR;
}

/**
* @brief Callback function for reading and writing registers over SPI
* @param dev_id		: Library agnostic parameter to identify the device to communicate with
* @param reg_addr	: Register address
* @param reg_data	: Pointer to the array containing the data to be read or written
* @param len		: Length of the array of data
* @return	Zero for success, non-zero otherwise
*/
int8_t spi_transfer_bme280(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
	//select which Chip Select pin has	to be set low to activate the relevant device on the SPI bus
	if(dev_id == bme280) CLEAR_CS_PIN(CS_BME280_PORT,CS_BME280_PIN);
	spi_transfer_generic(reg_addr); // Write the register address, ignore the return
	for (uint16_t i = 0; i < len; i++)
	{
		reg_data[i] = spi_transfer_generic(reg_data[i]);
	}
	if(dev_id == bme280) SET_CS_PIN(CS_BME280_PORT,CS_BME280_PIN);
	return rslt;
}

/* In BME280 Write Scenario 
 * Data on the SPI bus should be like
 * |---------------------+--------------+-------------|
 * | MOSI                | MISO         | Chip Select |
 * |---------------------+--------------|-------------|
 * | (don't care)        | (don't care) | HIGH        |
 * | (reg_addr)          | (don't care) | LOW         |
 * | (reg_data[0])       | (don't care) | LOW         |
 * | (....)              | (....)       | LOW         |
 * | (reg_data[len - 1]) | (don't care) | LOW         |
 * | (don't care)        | (don't care) | HIGH        |
 * |---------------------+--------------|-------------|
 */

/* In BME280 Read Scenario 
 * Data on the SPI bus should be like
 * |----------------+---------------------+-------------|
 * | MOSI           | MISO                | Chip Select |
 * |----------------+---------------------|-------------|
 * | (don't care)   | (don't care)        | HIGH        |
 * | (reg_addr)     | (don't care)        | LOW         |
 * | (don't care)   | (reg_data[0])       | LOW         |
 * | (....)         | (....)              | LOW         |
 * | (don't care)   | (reg_data[len - 1]) | LOW         |
 * | (don't care)   | (don't care)        | HIGH        |
 * |----------------+---------------------|-------------|
 */

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

#if 0
switch(dev_id) {
	case bme280:
	CLEAR_CS_PIN(CS_BME280_PORT,CS_BME280_PIN);
	break;
	case lis3mdl:
	CLEAR_CS_PIN(CS_LIS3MDL_PORT,CS_LIS3MDL_PIN);
	break;
	case flash:
	CLEAR_CS_PIN(CS_FLASH_PORT,CS_FLASH_PIN);
	break;
	default:
	break;
}

switch(dev_id) {
	case bme280:
	SET_CS_PIN(CS_BME280_PORT,CS_BME280_PIN);
	break;
	case lis3mdl:
	SET_CS_PIN(CS_LIS3MDL_PORT,CS_LIS3MDL_PIN);
	break;
	case flash:
	SET_CS_PIN(CS_FLASH_PORT,CS_FLASH_PIN);
	break;
	default:
	break;
	}
#endif