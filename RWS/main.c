/*
 * main.c
 *
 * Created: 8/3/2018 10:46:04 PM
 *  Author: alexandru.gaal
  */ 
/*
 * References for Remote Weather Station software:
 * For Bosch BME 280 Temperature, pressure, humidity sensor see: https://github.com/BoschSensortec/BME280_driver
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "configuration.h"
#include "spi_handler.h"
#include "uart_handler.h"

#include "bme280/bme280.h"

#include <util/delay.h>

#define INIT_STATUS_LED		(STATUS_LED_DDR |= (1 << STATUS_LED_PIN))
#define TOGGLE_STATUS_LED	(STATUS_LED_PORT ^= (1 << STATUS_LED_PIN))
#define STATUS_LED_ON		(STATUS_LED_PORT |= (1 << STATUS_LED_PIN))
#define STATUS_LED_OFF		(STATUS_LED_PORT &= ~(1 << STATUS_LED_PIN))

#define INIT_PORT(x,y)	(x) |=  (1<<(y))
#define SET_PORT(x,y)	(x) |=(1<<(y))
#define CLEAR_PORT(x,y)	(x) &= ~(1<<(y))

struct bme280_dev sensor_data;
int8_t rslt = BME280_OK;

/* Sensor_0 interface over SPI with native chip select line */
sensor_data.dev_id = 0;
sensor_data.intf = BME280_SPI_INTF;
sensor_data.read = user_spi_read;
sensor_data.write = user_spi_write;
sensor_data.delay_ms = user_delay_ms;

rslt = bme280_init(&dev);




int main(void)
{
	static uint8_t spi_response = 0;
	INIT_STATUS_LED;
	
	INIT_PORT(CS_BME280_DDR,CS_BME280_PIN);
	CLEAR_PORT(CS_BME280_PORT,CS_BME280_PIN);
	_delay_ms(100);
	SET_PORT(CS_BME280_PORT,CS_BME280_PIN);
	
	cli();  //Disable interrupts
	uart_init(MYUBRR);
	spi_init();
	sei();  // enable global interrupts
	
	
	uart_send_string("Drivers initialized");
	uart_newline();
    while(1)
    {
		uart_send_char(0x30);
		uart_newline();
		TOGGLE_STATUS_LED;
		/*
		uart_send_char(0x30);
		uart_send_char('1');
		uart_send_char(0x32);
		uart_send_char('3');
		uart_newline();
		
		spi_send_char(0x00);
		spi_send_char(0x11);
		spi_send_char(0x33);
		spi_send_char(0x55);
		spi_send_char(0xff);
		*/

		CLEAR_PORT(CS_BME280_PORT,CS_BME280_PIN);
/*
		uart_send_char(spi_send_char(0xFD));
		uart_send_char(spi_send_char(0xFE));
		uart_send_char(spi_send_char(0x00));*/
		uart_send_char(spi_send_char(0xD0));
		SET_PORT(CS_BME280_PORT,CS_BME280_PIN);
		
		uart_newline();
		
/*
		CLEAR_PORT(CS_BME280_PORT,CS_BME280_PIN);
		spi_send_char(0xFD);
		uart_send_char(spi_get_char());	
		uart_send_char(spi_get_char());	
		SET_PORT(CS_BME280_PORT,CS_BME280_PIN);
*/

		
		//uart_send_char(spi_response);
		uart_newline();
		_delay_ms(500);
    }
}
