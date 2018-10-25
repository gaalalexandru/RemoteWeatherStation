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
#include <avr/sleep.h>

#include "configuration.h"
#include "timer_handler.h"
#include "spi_handler.h"
#include "uart_handler.h"
#include "bme280/bme280.h"
#include "sst25/sst25_flash_handler.h"
#include "sst25/sst25_flash_map.h"
#if USE_LIS3MDL
#include "lis3mdl/lis3mdl.h"
#endif
#include "auxiliary/auxiliary_functions.h"
#include "esp8266/esp_wifi_handler.h"

#define INIT_STATUS_LED		(STATUS_LED_DDR |= (1 << STATUS_LED_PIN))
#define TOGGLE_STATUS_LED	(STATUS_LED_PORT ^= (1 << STATUS_LED_PIN))
#define STATUS_LED_ON		(STATUS_LED_PORT |= (1 << STATUS_LED_PIN))
#define STATUS_LED_OFF		(STATUS_LED_PORT &= ~(1 << STATUS_LED_PIN))

#define INIT_PORT(x,y)	(x) |=  (1<<(y))
#define SET_PORT(x,y)	(x) |=(1<<(y))
#define CLEAR_PORT(x,y)	(x) &= ~(1<<(y))

#define MAIN_LOG_ACTIV (0)
#define DATAFRAME_LOG_ACTIV (0)
#define PRINT_BME280_PROCESSED_OUTPUT (0)
#define PRINT_LIS3MDL_PROCESSED_OUTPUT (1)

#if MAIN_LOG_ACTIV
#define LOG_DEC(x,y) uart_send_string(x); uart_send_dec(y); uart_newline();
#define LOG_HEX(x,y) uart_send_string(x); uart_send_uhex(y); uart_newline();
#define LOG_STR(x,y) uart_send_string(x); uart_send_string(y); uart_newline();
#define LOG_CHR(x,y) uart_send_string(x); uart_send_char(y); uart_newline();
#else
#define LOG_DEC(x,y)
#define LOG_HEX(x,y)
#define LOG_STR(x,y)
#define LOG_CHR(x,y)
#endif  //MAIN_LOG_ACTIV

struct bme280_dev bme280_interf;

extern volatile st_date_time timestamp; //timestamp
extern volatile uint8_t g_u8start_measurement; //flag to trigger measurement start
uint8_t g_u8data_frame[DATA_FRAME_SIZE];

void print_bme280_data(struct bme280_data *comp_data)
{
	uart_send_string("Temperature: ");	uart_send_udec(comp_data->temperature);	uart_newline();
	uart_send_string("Pressure: ");	uart_send_udec(comp_data->pressure);uart_newline();
	uart_send_string("Humidity: ");	uart_send_udec(comp_data->humidity);uart_newline();
}

void print_bme280_raw_data(struct bme280_uncomp_data *comp_data)
{
	uart_send_string("RAW Temperature: ");	uart_send_udec(comp_data->temperature);	uart_newline();
	uart_send_string("RAW Pressure: ");	uart_send_udec(comp_data->pressure);uart_newline();
	uart_send_string("RAW Humidity: ");	uart_send_udec(comp_data->humidity);uart_newline();
}

#if USE_LIS3MDL
void print_lis3mdl_data(lis3mdl_data_st *stprint_data)
{
	uart_send_dec(stprint_data->x_mag);uart_send_string("   ");
	uart_send_dec(stprint_data->y_mag);uart_send_string("   ");
	uart_send_dec(stprint_data->z_mag);uart_send_string("   ");
	uart_send_dec(stprint_data->temperature); uart_newline();
}
#endif

void save_measurements(void)
{
	uint32_t u32address = 0;
	u32address = (HOUR_0_ADDR + (timestamp.hour*SST25_SECTOR_SIZE) + (timestamp.minute * DATA_FRAME_SIZE));
	//erase the chip completely only when data is sent out...
	LOG_HEX("Write dataframe to: 0x", u32address)
	sst25_begin_write(u32address);
	sst25_write_array(g_u8data_frame,DATA_FRAME_SIZE);
}

int main(void)
{
	int8_t rslt = BME280_OK;
	
	#if USE_BME280
	struct bme280_uncomp_data uncomp_data;
	#endif
	
	#if PRINT_BME280_PROCESSED_OUTPUT
	struct bme280_data comp_data;
	#endif
	
	uint8_t i = 0;
	
	#if USE_LIS3MDL
	lis3mdl_data_st lis3mdl_print_data;
	uint8_t u8lis3mdl_data[8] = {0,0,0,0,0,0,0,0};
	#endif
	
	INIT_STATUS_LED;
	
	cli();  //Disable interrupts
	uart_init(MYUBRR);
	timer0_init(); //global timer init
	timer2_init(); //global timer init
	spi_init();
	sei();  // enable global interrupts
	LOG_STR("Compiled on: ",__DATE__);
	LOG_STR("Compiled at: ",__TIME__);
	LOG_STR("Drivers initialized: ","Timer, UART & SPI");
	sst25_flash_init();
	LOG_STR("Hardware initialized: ","SST25 Flash Memory");
	
	#if USE_BME280
	/* Sensor_0 interface over SPI with native chip select line */
	bme280_interf.dev_id = 0;
	bme280_interf.intf = BME280_SPI_INTF;
	bme280_interf.read = spi_transfer_sensors;
	bme280_interf.write = spi_transfer_sensors;
	bme280_interf.delay_ms = timer_delay_ms;
	rslt = bme280_init(&bme280_interf);
	LOG_DEC("BME280 sensor initialized with state: ",rslt);
	rslt = bme280_setup_weather_monitoring_meas(&bme280_interf);
	LOG_DEC("BME280 sensor setup with state: ",rslt);
	LOG_STR("Hardware initialized: ","BME280 Sensor");
	#endif //USE_BME280
	
	#if USE_LIS3MDL
	rslt = lis3mdl_init();
	LOG_DEC("LIS3MDL sensor initialized with state: ",rslt);
	LOG_STR("Hardware initialized: ","LIS3MDL Sensor");
	#endif //USE_LIS3MDL
	
	esp_init();
	
	timer_delay_ms(10);
	#if POWER_SAVE_ACTIV
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	#endif //POWER_SAVE_ACTIV


	
    while(1)
    {	
		#if POWER_SAVE_ACTIV
			LOG_CHR("Sleep",' ');
			timer_delay_ms(10);
			sleep_mode();
			LOG_CHR("Wakeup",' ');
		#endif //POWER_SAVE_ACTIV
		
		#if USE_LIS3MDL
		lis3mdl_single_meas();
		timer_delay_ms(5);  //3.7 ms necessay for LIS3MDL startup
		i = lis3mdl_read_meas(u8lis3mdl_data);
		lis3mdl_idle();
		#endif
		
		if(g_u8start_measurement) 
		{
			g_u8start_measurement = 0;

			#if MAIN_LOG_ACTIV
			uart_send_udec(20); uart_send_udec(timestamp.year); uart_send_string("/");
			uart_send_udec(timestamp.month); uart_send_string("/");
			uart_send_udec(timestamp.day); uart_send_string(" -> ");
			uart_send_udec(timestamp.hour); uart_send_string(":");
			uart_send_udec(timestamp.minute); uart_send_string(":");
			uart_send_udec(timestamp.second);
			uart_newline();
			#endif //MAIN_LOG_ACTIV
			
			#if USE_LIS3MDL
			lis3mdl_single_meas();
			#endif
						
			rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &bme280_interf);  //trigger forced measurement
			LOG_DEC("BME280 sensor force mode trigger with state: ",rslt);
			bme280_interf.delay_ms(40);  //delay needed for measurement to complete
			rslt = bme280_get_raw_sensor_data(BME280_ALL, &uncomp_data, &bme280_interf);
			LOG_DEC("BME280 sensor RAW read with state: ",rslt);
			
			#if USE_LIS3MDL
			rslt = lis3mdl_read_meas(u8lis3mdl_data);
			LOG_DEC("LIS3MDL read state:  ",rslt);
			lis3mdl_idle();
			#endif //USE_LIS3MDL
			
			g_u8data_frame[0] = DATA_FRAME_SEPARATOR;
			g_u8data_frame[1] = timestamp.year;
			g_u8data_frame[2] = timestamp.month;
			g_u8data_frame[3] = timestamp.day;
			g_u8data_frame[4] = timestamp.hour;
			g_u8data_frame[5] = timestamp.minute;
			g_u8data_frame[6] = timestamp.second;
			g_u8data_frame[7] = DATA_FRAME_SEPARATOR;
			g_u8data_frame[8] = (uint8_t)((uncomp_data.pressure>>12) & 0xFF);
			g_u8data_frame[9] = (uint8_t)((uncomp_data.pressure>>4) & 0xFF);
			g_u8data_frame[10] = (uint8_t)(uncomp_data.pressure & 0x0F);
			g_u8data_frame[11] = DATA_FRAME_SEPARATOR;
			g_u8data_frame[12] = (uint8_t)((uncomp_data.temperature>>12) & 0xFF);
			g_u8data_frame[13] = (uint8_t)((uncomp_data.temperature>>4) & 0xFF);
			g_u8data_frame[14] = (uint8_t)(uncomp_data.temperature & 0x0F);
			g_u8data_frame[15] = DATA_FRAME_SEPARATOR;
			g_u8data_frame[16] = (uint8_t)((uncomp_data.humidity>>8) & 0xFF);
			g_u8data_frame[17] = (uint8_t)(uncomp_data.humidity & 0xFF);
			g_u8data_frame[18] = DATA_FRAME_SEPARATOR;
			for (i=19; i<36; i++) {
				g_u8data_frame[i] = 0xFF;
			}
			#if USE_LIS3MDL
			g_u8data_frame[36] = DATA_FRAME_SEPARATOR;
			g_u8data_frame[37] = u8lis3mdl_data[0];
			g_u8data_frame[38] = u8lis3mdl_data[1];
			g_u8data_frame[39] = u8lis3mdl_data[2];
			g_u8data_frame[40] = u8lis3mdl_data[3];
			g_u8data_frame[41] = u8lis3mdl_data[4];
			g_u8data_frame[42] = u8lis3mdl_data[5];
			g_u8data_frame[43] = DATA_FRAME_SEPARATOR;
			//temperature LSB and MSB byte orders cannot be swapped, so swap here
			g_u8data_frame[44] = u8lis3mdl_data[6];
			g_u8data_frame[45] = u8lis3mdl_data[7];
			g_u8data_frame[46] = DATA_FRAME_SEPARATOR;
			#endif
			for (i=47; i<DATA_FRAME_SIZE; i++) {
				g_u8data_frame[i] = 0xFF;
			}
			//save_measurements();
			#if DATAFRAME_LOG_ACTIV
			for (i=0; i<19; i++) {
				uart_send_dec(i); uart_send_char('>'); uart_send_dec(g_u8data_frame[i]); uart_newline();
			}
			uart_newline();
			for (i=36; i<46; i++) {
				uart_send_dec(i); uart_send_char('>'); uart_send_dec(g_u8data_frame[i]); uart_newline();
			}
			uart_newline();
			timer_delay_ms(10);
			#endif //DATAFRAME_LOG_ACTIV
		}

		#if (PRINT_BME280_PROCESSED_OUTPUT && USE_BME280)
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme280_interf);
		print_bme280_data(&comp_data);
		uart_send_string("BME280 sensor read with state: ");uart_send_dec(rslt);uart_newline();
		#endif //PRINT_BME280_OUTPUT
		
		#if (PRINT_LIS3MDL_PROCESSED_OUTPUT && USE_LIS3MDL)
		lis3mdl_process_meas(u8lis3mdl_data, &lis3mdl_print_data);
		print_lis3mdl_data(&lis3mdl_print_data);
		timer_delay_ms(10);
		#endif //(PRINT_LIS3MDL_PROCESSED_OUTPUT && USE_LIS3MDL)
    }
}

/* TODO for minimizing power consumption:
 * sleep in power save mode
 * disable ADC, analog comparator
 * disable brown out detector - page 40 data sheet
 * disable internal voltage ref
 * disable watchdog timer
 * if sending out data before going to sleep, consider using a few ms delay befor sleep instruction
 
 * TODO for reset:
 * save date & time to eeprom once every minute
 * after reset or power up, read last time from eeprom
 */