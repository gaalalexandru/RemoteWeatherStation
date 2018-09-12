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

#define INIT_STATUS_LED		(STATUS_LED_DDR |= (1 << STATUS_LED_PIN))
#define TOGGLE_STATUS_LED	(STATUS_LED_PORT ^= (1 << STATUS_LED_PIN))
#define STATUS_LED_ON		(STATUS_LED_PORT |= (1 << STATUS_LED_PIN))
#define STATUS_LED_OFF		(STATUS_LED_PORT &= ~(1 << STATUS_LED_PIN))

#define INIT_PORT(x,y)	(x) |=  (1<<(y))
#define SET_PORT(x,y)	(x) |=(1<<(y))
#define CLEAR_PORT(x,y)	(x) &= ~(1<<(y))

struct bme280_dev sensor_interf;
int8_t rslt = BME280_OK;

#define OUTPUT_LOG

void print_sensor_data(struct bme280_data *comp_data)
{
	#ifdef BME280_FLOAT_ENABLE
	//printf("%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
	#else
	#ifdef OUTPUT_LOG
	uart_send_string("Temperature: ");	uart_send_udec(comp_data->temperature);	uart_newline();
	uart_send_string("Pressure: ");	uart_send_udec(comp_data->pressure);uart_newline();
	uart_send_string("Humidity: ");	uart_send_udec(comp_data->humidity);uart_newline();
	#endif // OUTPUT_LOG
	#endif
}


#if WEATHER_MONITORING
int8_t setup_measurement_weather_monitoring(struct bme280_dev *dev)
//int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
	//int8_t rslt;
	//uint8_t settings_sel;
	

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_1X;
	dev->settings.osr_t = BME280_OVERSAMPLING_1X;
	dev->settings.filter = BME280_FILTER_COEFF_OFF;
	
	/*settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_FILTER_SEL;*/
	//rslt = bme280_set_sensor_settings(settings_sel, dev);
	#ifdef OUTPUT_LOG
	uart_send_string("BME280 sensor setup with state: ");uart_send_char(0x35+rslt);	uart_newline();
	#endif  //OUTPUT_LOG
	//return rslt;
	return bme280_set_sensor_settings(BME280_OSR_PRESS_SEL|BME280_OSR_TEMP_SEL|BME280_OSR_HUM_SEL|BME280_FILTER_SEL, dev);
}
#else	
int8_t setup_measurement_normal_mode(struct bme280_dev *dev)
//int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
	int8_t rslt;
	uint8_t settings_sel;
	

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;
	dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, dev);
	#ifdef OUTPUT_LOG
	uart_send_string("BME280 sensor setup with state: ");uart_send_char(0x35+rslt);	uart_newline();
	#endif  //OUTPUT_LOG
	
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);
	#ifdef OUTPUT_LOG
	uart_send_string("BME280 sensor setup with state: ");uart_send_char(0x35+rslt);uart_newline();
	#endif  //OUTPUT_LOG
	return rslt;
}
#endif //WEATHER_MONITORING

int main(void)
{
	uint8_t *pu8dreturnata = "";
	
	struct bme280_data comp_data;
	INIT_STATUS_LED;
	
	cli();  //Disable interrupts
	timer2_init(); //global timer init
	uart_init(MYUBRR);
	spi_init();
	sei();  // enable global interrupts
	
	#ifdef OUTPUT_LOG
	uart_send_string("Timer, UART & SPI Drivers initialized");	uart_newline();
	#endif  //OUTPUT_LOG
	
	/* Sensor_0 interface over SPI with native chip select line */
	sensor_interf.dev_id = 0;
	sensor_interf.intf = BME280_SPI_INTF;
	sensor_interf.read = spi_transfer_bme280;
	sensor_interf.write = spi_transfer_bme280;
	sensor_interf.delay_ms = timer_delay_ms;

	sst25_flash_init();
	uart_send_string("testing memory read:");
	uart_newline();
	sst25_read_array(0x00000000,pu8dreturnata,2);
	uart_send_string("read data: ");
	while(*pu8dreturnata){
		uart_send_char(*pu8dreturnata);
		pu8dreturnata++;
	}
	uart_newline();	

	rslt = bme280_init(&sensor_interf);
	

	
	#ifdef OUTPUT_LOG
	uart_send_string("BME280 sensor initialized with state: ");
	uart_send_char(0x35+rslt);
	uart_newline();
	#endif  //OUTPUT_LOG
	
	#if WEATHER_MONITORING
	setup_measurement_weather_monitoring(&sensor_interf);
	#else
	setup_measurement_normal_mode(&sensor_interf);
	#endif  //WEATHER_MONITORING
	
    while(1)
    {
		uart_newline();
		TOGGLE_STATUS_LED;

		#if WEATHER_MONITORING
		timer_delay_ms(6000);
		rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &sensor_interf);  //trigger forced measurement
		sensor_interf.delay_ms(40);  //delay needed for measurement to complete
		#ifdef OUTPUT_LOG
		uart_send_string("BME280 sensor force mode trigger with state: ");uart_send_char(0x35+rslt);uart_newline();
		#endif  //OUTPUT_LOG
		#else
		timer_delay_ms(1000);
		#endif  //WEATHER_MONITORING
		
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &sensor_interf);
		print_sensor_data(&comp_data);
		#ifdef OUTPUT_LOG
		uart_send_string("BME280 sensor read with state: ");uart_send_char(0x35+rslt);uart_newline();
		#endif  //OUTPUT_LOG
    }
}