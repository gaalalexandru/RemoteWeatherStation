/*
 * lis3mdl.c
 *
 * Created: 9/18/2018 4:29:13 PM
 *  Author: alexandru.gaal
 */ 


#define LIS3MDL_LOG_ACTIV (1)  //enable UART log for lis3mdl driver module

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "../configuration.h"
#include "../spi_handler.h"
#include "lis3mdl.h"
#include "lis3mdl_defs.h"

#if LIS3MDL_LOG_ACTIV
#include "../uart_handler.h"
#endif //SST25_LOG_ACTIV

#define LIS3MDL_SPI_DEV_ID	(1)





struct lis3mdl_sensor_data_s
{
  int16_t x_mag;              /* Measurement result for x axis */
  int16_t y_mag;              /* Measurement result for y axis */
  int16_t z_mag;              /* Measurement result for z axis */
  int16_t temperature;        /* Measurement result for temperature sensor */
};

//struct lis3mdl_dev_s
//{
//  struct lis3mdl_dev_s *flink;     /* Supports a singly linked list of
//                                        * drivers */
//  struct spi_dev_s *spi;           /* Pointer to the SPI instance */
//  struct lis3mdl_config_s *config; /* Pointer to the configuration
//                                        * of the LIS3MDL sensor */
//  uint8_t/*sem_t*/ datasem;                       /* Manages exclusive access to this
//                                        * structure */
//  struct lis3mdl_sensor_data_s data;   /* The data as measured by the sensor */
//  struct work_s work;                  /* The work queue is responsible for
//                                        * retrieving the data from the
//                                        * sensor after the arrival of new
//                                        * data was signalled in an interrupt */
//};

static uint8_t get_id(uint8_t u8expected_id) {
	uint8_t u8id = 0;
	ENABLE_CS_LIS3MDL;
	spi_transfer_generic(READ_OP|ADDR_CONST|WHO_AM_I);
	u8id = spi_transfer_generic(0xFF);
	if(u8id == u8expected_id) {
		#if LIS3MDL_LOG_ACTIV
		uart_send_string("ID match: "); uart_send_uhex(u8id); uart_newline();
		#endif //LIS3MDL_LOG_ACTIV
	} else {
		#if LIS3MDL_LOG_ACTIV
		uart_send_string("ID NOT match: "); uart_send_uhex(u8id); uart_newline();
		#endif //LIS3MDL_LOG_ACTIV
	}
	DISABLE_CS_LIS3MDL;
	return (u8id == u8expected_id);
}


void lis3mdl_init(void) {
	uint8_t u8data = 0;

	u8data = (1 << TEMP_EN);  //temp meas enabled, low power mode, output data rate 0.625 Hz
	spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(WRITE_OP|ADDR_CONST|CTRL_REG1), &u8data, 1);
	
	u8data = (1 << FS1) | (1 << FS0);  //full scale selection ±16 gauss
	spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(WRITE_OP|ADDR_CONST|CTRL_REG2), &u8data, 1);
	
	u8data = (1 << LP) | (1 << MD0);  //low power, single conversion mode
	spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(WRITE_OP|ADDR_CONST|CTRL_REG3), &u8data, 1);
	
	u8data = 0;  //low power Z axis, MSB first
	spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(WRITE_OP|ADDR_CONST|CTRL_REG4), &u8data, 1);
	
	u8data = 0;  //not fast read, continuous update
	spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(WRITE_OP|ADDR_CONST|CTRL_REG5), &u8data, 1);
	
	get_id(LIS3MDL_MANUF_ID);
}

void lis3mdl_read_meas(uint8_t) {
//}
//uint8_t lis3mdl_data_ready(void) {
	uint8_t u8data = 0;
	uint8_t retry_count = 10;
	uint8_t lis3mdl_meas[8] = {0,0,0,0,0,0,0,0};
	do {
		spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(READ_OP|ADDR_CONST|STATUS_REG), &u8data, 1);	
	} while ((u8data & (1<<ZYXDA) == 0) && (retry_count--);
	spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(READ_OP|ADDR_INCR|OUT_X_H), &lis3mdl_meas, 8);
	
}