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
#include "timer_handler.h"
#include "spi_handler.h"
#include "uart_handler.h"
#include "bme280/bme280.h"
#include "sst25/sst25_flash_handler.h"
#include "sst25/sst25_flash_map.h"

#define INIT_STATUS_LED		(STATUS_LED_DDR |= (1 << STATUS_LED_PIN))
#define TOGGLE_STATUS_LED	(STATUS_LED_PORT ^= (1 << STATUS_LED_PIN))
#define STATUS_LED_ON		(STATUS_LED_PORT |= (1 << STATUS_LED_PIN))
#define STATUS_LED_OFF		(STATUS_LED_PORT &= ~(1 << STATUS_LED_PIN))

#define INIT_PORT(x,y)	(x) |=  (1<<(y))
#define SET_PORT(x,y)	(x) |=(1<<(y))
#define CLEAR_PORT(x,y)	(x) &= ~(1<<(y))

#define MAIN_LOG_ACTIV (0)

struct bme280_dev sensor_interf;


void print_sensor_data(struct bme280_data *comp_data)
{
	#if MAIN_LOG_ACTIV
	uart_send_string("Temperature: ");	uart_send_udec(comp_data->temperature);	uart_newline();
	uart_send_string("Pressure: ");	uart_send_udec(comp_data->pressure);uart_newline();
	uart_send_string("Humidity: ");	uart_send_udec(comp_data->humidity);uart_newline();
	#endif // MAIN_LOG_ACTIV
}

void print_sensor_raw_data(struct bme280_uncomp_data *comp_data)
{
	#if MAIN_LOG_ACTIV
	uart_send_string("RAW Temperature: ");	uart_send_udec(comp_data->temperature);	uart_newline();
	uart_send_string("RAW Pressure: ");	uart_send_udec(comp_data->pressure);uart_newline();
	uart_send_string("RAW Humidity: ");	uart_send_udec(comp_data->humidity);uart_newline();
	#endif // MAIN_LOG_ACTIV
}

int main(void)
{
	int8_t rslt = BME280_OK;
	struct bme280_data comp_data;
	struct bme280_uncomp_data uncomp_data;	
	uint8_t pu8dreturnata[5] = {0,0,0,0,0};

	INIT_STATUS_LED;
	
	cli();  //Disable interrupts
	uart_init(MYUBRR);
	timer0_init(); //global timer init
	timer2_init(); //global timer init
	spi_init();
	sei();  // enable global interrupts

	uart_send_string("Timer, UART & SPI Drivers initialized");	uart_newline();
	sst25_flash_init();

	/*uart_send_string("testing memory write:"); uart_newline();	
	sst25_begin_write(HOUR_0_ADDR | MINUTE_2_REL_START_ADDR);
	sst25_write(0x38);
	sst25_write(0x39);
	sst25_write(0x40);
	sst25_write(0x41);
	uart_send_string("write done"); uart_newline();	*/

	//sst25_erase_chip();
	
	/*uart_send_string("testing memory read 1:");
	uart_newline();
	sst25_read_array(HOUR_0_ADDR | MINUTE_2_REL_START_ADDR,pu8dreturnata,4);
	uart_send_string("read data 1: ");
	uart_send_string(pu8dreturnata);
	uart_newline();*/

	/*uart_send_string("testing memory array write:"); uart_newline();	
	sst25_begin_write(HOUR_1_ADDR | MINUTE_0_REL_START_ADDR);
	sst25_write_array(pu8dreturnata,4);
	uart_send_string("write done"); uart_newline();*/

	/*uart_send_string("testing memory read 2:");
	uart_newline();
	sst25_read_array(HOUR_1_ADDR | MINUTE_0_REL_START_ADDR,pu8dreturnata,4);
	uart_send_string("read data 2: ");
	uart_send_string(pu8dreturnata);
	uart_newline();*/

	/*
	uart_send_string("testing memory read:");
	uart_newline();
	sst25_read_array(0x00000000,pu8dreturnata,4);
	uart_send_string("read data: ");
	uart_send_string(pu8dreturnata);
	uart_newline();

	uart_send_string("testing memory read:");
	uart_newline();
	sst25_read_array(HOUR_0_ADDR | MINUTE_1_REL_START_ADDR,pu8dreturnata,4);
	uart_send_string("read data: ");
	uart_send_string(pu8dreturnata);
	uart_newline();
	*/
	
	
	/* Sensor_0 interface over SPI with native chip select line */
	sensor_interf.dev_id = 0;
	sensor_interf.intf = BME280_SPI_INTF;
	sensor_interf.read = spi_transfer_bme280;
	sensor_interf.write = spi_transfer_bme280;
	sensor_interf.delay_ms = timer_delay_ms;
	rslt = bme280_init(&sensor_interf);
		
	#if MAIN_LOG_ACTIV
	uart_send_string("BME280 sensor initialized with state: "); uart_send_dec(rslt); uart_newline();
	uart_send_string("BME280 sensor calibration data: "); uart_newline();
	uart_send_string("dig_T1 = "); uart_send_dec(sensor_interf.calib_data.dig_T1); uart_newline();
	uart_send_string("dig_T2 = "); uart_send_dec(sensor_interf.calib_data.dig_T2); uart_newline();
	uart_send_string("dig_T3 = "); uart_send_dec(sensor_interf.calib_data.dig_T3); uart_newline();
	uart_send_string("dig_P1 = "); uart_send_dec(sensor_interf.calib_data.dig_P1); uart_newline();
	uart_send_string("dig_P2 = "); uart_send_dec(sensor_interf.calib_data.dig_P2); uart_newline();
	uart_send_string("dig_P3 = "); uart_send_dec(sensor_interf.calib_data.dig_P3); uart_newline();
	uart_send_string("dig_P4 = "); uart_send_dec(sensor_interf.calib_data.dig_P4); uart_newline();
	uart_send_string("dig_P5 = "); uart_send_dec(sensor_interf.calib_data.dig_P5); uart_newline();
	uart_send_string("dig_P6 = "); uart_send_dec(sensor_interf.calib_data.dig_P6); uart_newline();
	uart_send_string("dig_P7 = "); uart_send_dec(sensor_interf.calib_data.dig_P7); uart_newline();
	uart_send_string("dig_P8 = "); uart_send_dec(sensor_interf.calib_data.dig_P8); uart_newline();
	uart_send_string("dig_P9 = "); uart_send_dec(sensor_interf.calib_data.dig_P9); uart_newline();
	uart_send_string("dig_H1 = "); uart_send_dec(sensor_interf.calib_data.dig_H1); uart_newline();
	uart_send_string("dig_H2 = "); uart_send_dec(sensor_interf.calib_data.dig_H2); uart_newline();
	uart_send_string("dig_H3 = "); uart_send_dec(sensor_interf.calib_data.dig_H3); uart_newline();
	uart_send_string("dig_H4 = "); uart_send_dec(sensor_interf.calib_data.dig_H4); uart_newline();
	uart_send_string("dig_H5 = "); uart_send_dec(sensor_interf.calib_data.dig_H5); uart_newline();
	uart_send_string("dig_H6 = "); uart_send_dec(sensor_interf.calib_data.dig_H6); uart_newline();
	uart_send_string("t_fine = "); uart_send_dec(sensor_interf.calib_data.t_fine); uart_newline();												
	#endif  //MAIN_LOG_ACTIV
	
	rslt = bme280_setup_weather_monitoring_meas(&sensor_interf);
	#if MAIN_LOG_ACTIV
	uart_send_string("BME280 sensor setup with state: ");uart_send_dec(rslt);uart_newline();
	#endif  //MAIN_LOG_ACTIV
	
    while(1)
    {	
		//wait 6 seconds during development, 60 seconds in final product
		timer_delay_ms(WEATHER_MONITORING_INTERVAL_MS  / WEATHER_MONITORING_ACCELERATION);
		
		rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &sensor_interf);  //trigger forced measurement
		sensor_interf.delay_ms(40);  //delay needed for measurement to complete
		#if MAIN_LOG_ACTIV
		uart_send_string("BME280 sensor force mode trigger with state: ");uart_send_dec(rslt);uart_newline();
		#endif  //MAIN_LOG_ACTIV
		
		
		rslt = bme280_get_raw_sensor_data(BME280_ALL, &uncomp_data, &sensor_interf);
		print_sensor_raw_data(&uncomp_data);
		#if MAIN_LOG_ACTIV
		uart_send_string("BME280 sensor RAW read with state: ");uart_send_dec(rslt);uart_newline();
		#endif  //MAIN_LOG_ACTIV


		/*
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &sensor_interf);
		print_sensor_data(&comp_data);
		#if MAIN_LOG_ACTIV
		uart_send_string("BME280 sensor read with state: ");uart_send_dec(rslt);uart_newline();
		#endif  //MAIN_LOG_ACTIV
		*/
	
    }
}