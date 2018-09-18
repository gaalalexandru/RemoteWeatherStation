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

uint8_t get_id(void) {
	uint8_t u8id = 0;
	ENABLE_CS_LIS3MDL;
	spi_transfer_generic(READ_OP|ADDR_CONST|WHO_AM_I);
	u8id = spi_transfer_generic(0x00);
	DISABLE_CS_LIS3MDL;
	if(u8id == LIS3MDL_MANUF_ID) {
		#if LIS3MDL_LOG_ACTIV
		uart_send_string("ID match: "); uart_send_uhex(u8id); uart_newline();
		#endif //LIS3MDL_LOG_ACTIV
		return 1;
	} else {
		#if LIS3MDL_LOG_ACTIV
		uart_send_string("ID NOT match: "); uart_send_uhex(u8id); uart_newline();
		#endif //LIS3MDL_LOG_ACTIV
		return 0;
	}
}


void lis3mdl_init(void) {
	
}