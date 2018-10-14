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
	#if MAIN_LOG_ACTIV
	uart_send_string("Write dataframe to: 0x"); uart_send_uhex(u32address); uart_newline();
	#endif //MAIN_LOG_ACTIV
	sst25_begin_write(u32address);
	sst25_write_array(g_u8data_frame,DATA_FRAME_SIZE);
}

int main(void)
{
	int8_t rslt = BME280_OK;
	struct bme280_uncomp_data uncomp_data;	
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

	uart_send_string("Timer, UART & SPI Drivers initialized");	uart_newline();
	uart_send_string("Compiled on: ");uart_send_string(__DATE__);	uart_newline();
	uart_send_string("Compiled at: ");uart_send_string(__TIME__);	uart_newline();
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
	bme280_interf.dev_id = 0;
	bme280_interf.intf = BME280_SPI_INTF;
	bme280_interf.read = spi_transfer_sensors;
	bme280_interf.write = spi_transfer_sensors;
	bme280_interf.delay_ms = timer_delay_ms;
	rslt = bme280_init(&bme280_interf);
		
	#if MAIN_LOG_ACTIV
	uart_send_string("BME280 sensor initialized with state: "); uart_send_dec(rslt); uart_newline();
	uart_send_string("BME280 sensor calibration data: "); uart_newline();
	uart_send_string("dig_T1 = "); uart_send_dec(bme280_interf.calib_data.dig_T1); uart_newline();
	uart_send_string("dig_T2 = "); uart_send_dec(bme280_interf.calib_data.dig_T2); uart_newline();
	uart_send_string("dig_T3 = "); uart_send_dec(bme280_interf.calib_data.dig_T3); uart_newline();
	uart_send_string("dig_P1 = "); uart_send_dec(bme280_interf.calib_data.dig_P1); uart_newline();
	uart_send_string("dig_P2 = "); uart_send_dec(bme280_interf.calib_data.dig_P2); uart_newline();
	uart_send_string("dig_P3 = "); uart_send_dec(bme280_interf.calib_data.dig_P3); uart_newline();
	uart_send_string("dig_P4 = "); uart_send_dec(bme280_interf.calib_data.dig_P4); uart_newline();
	uart_send_string("dig_P5 = "); uart_send_dec(bme280_interf.calib_data.dig_P5); uart_newline();
	uart_send_string("dig_P6 = "); uart_send_dec(bme280_interf.calib_data.dig_P6); uart_newline();
	uart_send_string("dig_P7 = "); uart_send_dec(bme280_interf.calib_data.dig_P7); uart_newline();
	uart_send_string("dig_P8 = "); uart_send_dec(bme280_interf.calib_data.dig_P8); uart_newline();
	uart_send_string("dig_P9 = "); uart_send_dec(bme280_interf.calib_data.dig_P9); uart_newline();
	uart_send_string("dig_H1 = "); uart_send_dec(bme280_interf.calib_data.dig_H1); uart_newline();
	uart_send_string("dig_H2 = "); uart_send_dec(bme280_interf.calib_data.dig_H2); uart_newline();
	uart_send_string("dig_H3 = "); uart_send_dec(bme280_interf.calib_data.dig_H3); uart_newline();
	uart_send_string("dig_H4 = "); uart_send_dec(bme280_interf.calib_data.dig_H4); uart_newline();
	uart_send_string("dig_H5 = "); uart_send_dec(bme280_interf.calib_data.dig_H5); uart_newline();
	uart_send_string("dig_H6 = "); uart_send_dec(bme280_interf.calib_data.dig_H6); uart_newline();
	uart_send_string("t_fine = "); uart_send_dec(bme280_interf.calib_data.t_fine); uart_newline();												
	#endif  //MAIN_LOG_ACTIV
	
	rslt = bme280_setup_weather_monitoring_meas(&bme280_interf);
	#if MAIN_LOG_ACTIV
	uart_send_string("BME280 sensor setup with state: ");uart_send_dec(rslt);uart_newline();
	#endif  //MAIN_LOG_ACTIV
	
	#if USE_LIS3MDL
	lis3mdl_init();
	#endif
	
	timer_delay_ms(10);
	#if POWER_SAVE_ACTIV
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	#endif //POWER_SAVE_ACTIV

#ifdef MAIN_TEST_2COMPLEMENT_CONVERSION	
	uart_send_string("testing conversion from 2s complement:"); uart_newline();
	uart_send_udec(60643); uart_send_char('='); uart_send_dec(conv_2compl_to_signed_dec(60643,16)); uart_newline();  //expected: -4893
	uart_send_udec(27875); uart_send_char('='); uart_send_dec(conv_2compl_to_signed_dec(27875,16)); uart_newline();  //expected: 27875
	uart_send_udec(40163); uart_send_char('='); uart_send_dec(conv_2compl_to_signed_dec(40163,16)); uart_newline();  //expected: -25373
	uart_send_udec(7406); uart_send_char('='); uart_send_dec(conv_2compl_to_signed_dec(7406,16)); uart_newline();  //expected: 7406
	uart_send_udec(62190); uart_send_char('='); uart_send_dec(conv_2compl_to_signed_dec(62190,16)); uart_newline();  //expected: -3346
	timer_delay_ms(100);
#endif //MAIN_TEST_2COMPLEMENT_CONVERSION
	
    while(1)
    {	
		#if POWER_SAVE_ACTIV
			#if MAIN_LOG_ACTIV
				uart_send_char('S');
			#endif  //MAIN_LOG_ACTIV
			timer_delay_ms(10);
			sleep_mode();
			#if MAIN_LOG_ACTIV
				uart_send_char('W');
			#endif  //MAIN_LOG_ACTIV
		#endif //POWER_SAVE_ACTIV

		lis3mdl_single_meas();
		i = lis3mdl_read_meas(u8lis3mdl_data);
		lis3mdl_idle();
		timer_delay_ms(10);
		
		#if (USE_LIS3MDL && PRINT_LIS3MDL_PROCESSED_OUTPUT)
		lis3mdl_single_meas();
		timer_delay_ms(5);
		i = lis3mdl_read_meas(u8lis3mdl_data);
		lis3mdl_idle();
		lis3mdl_process_meas(u8lis3mdl_data, &lis3mdl_print_data);
		print_lis3mdl_data(&lis3mdl_print_data);
		timer_delay_ms(10);
		#endif
		
		
		if(g_u8start_measurement) 
		{
			g_u8start_measurement = 0;
			//wait 6 seconds during development, 60 seconds in final product
			//timer_delay_ms(WEATHER_MONITORING_INTERVAL_MS  / WEATHER_MONITORING_ACCELERATION);
			
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
			bme280_interf.delay_ms(40);  //delay needed for measurement to complete
			#if MAIN_LOG_ACTIV
			uart_send_string("BME280 sensor force mode trigger with state: ");uart_send_dec(rslt);uart_newline();
			#endif  //MAIN_LOG_ACTIV
			
			rslt = bme280_get_raw_sensor_data(BME280_ALL, &uncomp_data, &bme280_interf);
			
			#if MAIN_LOG_ACTIV
			uart_send_string("BME280 sensor RAW read with state: ");uart_send_dec(rslt);uart_newline();
			print_bme280_raw_data(&uncomp_data);
			#endif  //MAIN_LOG_ACTIV
			
			#if USE_LIS3MDL
			i = lis3mdl_read_meas(u8lis3mdl_data);
			#if MAIN_LOG_ACTIV
			uart_send_string("lis3mdl read state: "); uart_send_char(0x30 + i); uart_newline();
			#endif  //MAIN_LOG_ACTIV
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

		#if PRINT_BME280_PROCESSED_OUTPUT
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme280_interf);
		print_bme280_data(&comp_data);
		uart_send_string("BME280 sensor read with state: ");uart_send_dec(rslt);uart_newline();
		#endif //PRINT_BME280_OUTPUT
		
		//#if (PRINT_LIS3MDL_PROCESSED_OUTPUT && USE_LIS3MDL)
		//lis3mdl_process_meas(u8lis3mdl_data, &lis3mdl_print_data);
		//print_lis3mdl_data(&lis3mdl_print_data);
		//timer_delay_ms(10);
		//#endif //(PRINT_LIS3MDL_PROCESSED_OUTPUT && USE_LIS3MDL)
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