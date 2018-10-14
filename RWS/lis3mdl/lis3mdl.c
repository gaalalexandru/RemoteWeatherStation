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
#include "../auxiliary/auxiliary_functions.h"

#if LIS3MDL_LOG_ACTIV
#include "../uart_handler.h"
#endif //SST25_LOG_ACTIV

#define LIS3MDL_SPI_DEV_ID	(1)
#define LIS3MDL_FULL_SCALE	(8)
#define LIS3MDL_BYTE_ORDER_MSB (1)

static uint8_t get_id(const uint8_t u8expected_id) {
	uint8_t u8id = 0;
	ENABLE_CS_LIS3MDL;
	spi_transfer_generic(READ_OP|ADDR_CONST|WHO_AM_I);
	u8id = spi_transfer_generic(0xFF);
	if(u8id == u8expected_id) {
		#if LIS3MDL_LOG_ACTIV
		uart_send_string("ID OK: "); uart_send_uhex(u8id); uart_newline();
		#endif //LIS3MDL_LOG_ACTIV
	} else {
		#if LIS3MDL_LOG_ACTIV
		uart_send_string("ID NOT OK: "); uart_send_uhex(u8id); uart_newline();
		#endif //LIS3MDL_LOG_ACTIV
	}
	DISABLE_CS_LIS3MDL;
	return (u8id == u8expected_id);
}


uint8_t lis3mdl_init(void) {
	uint8_t u8data = 0;


	//temp meas enabled, OM = High performance mode, DO = output data rate 80 Hz
	u8data = (1 << TEMP_EN) | (1 << DO2) | (1 << DO1) | (1 << DO0) | (1 << OM1);
	spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(WRITE_OP|ADDR_CONST|CTRL_REG1), &u8data, 1);

	#if LIS3MDL_BYTE_ORDER_MSB
	u8data = (1 << OMZ1) | (1 << BLE); //Z axis High performance , MSB of data on lower register address
	#else 
	u8data = (1 << OMZ1); //Z axis High performance , LSB of data on lower register address
	#endif  //LIS3MDL_BYTE_ORDER_MSB
	spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(WRITE_OP|ADDR_CONST|CTRL_REG4), &u8data, 1);
	
	//u8data = (1 << FS0);  //full scale selection ±8 gauss
	
	#if (LIS3MDL_FULL_SCALE == 4)
	u8data = 0;  //full scale selection ±4 gauss	
	#elif (LIS3MDL_FULL_SCALE == 8)
	u8data = (1 << FS0);  //full scale selection ±8 gauss
	#endif
	spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(WRITE_OP|ADDR_CONST|CTRL_REG2), &u8data, 1);
	
	u8data = 0;//(1<<BDU);  //not fast read, block data update until is read out
	spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(WRITE_OP|ADDR_CONST|CTRL_REG5), &u8data, 1);

	u8data = (1 << MD1) | (1 << MD0);  //power down mode, initiate single conversion mode at every 1 min.
	//u8data = (1 << LP) | (1 << MD1) | (1 << MD0);  //low power, power down mode, initiate single conversion mode at every 1 min.
	spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(WRITE_OP|ADDR_CONST|CTRL_REG3), &u8data, 1);
	
	u8data = get_id(LIS3MDL_MANUF_ID);
	return u8data;
}

uint8_t lis3mdl_read_meas(uint8_t *pu8read_data) {
	uint8_t u8data = 0;
	uint8_t u8retry_count = 9;
							#if 1
							do {
								spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(READ_OP|ADDR_CONST|STATUS_REG), &u8data, 1);
							} while (((u8data & (1<<ZYXDA)) == 0) && (u8retry_count--));
							spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(READ_OP|ADDR_INCR|OUT_X_L), pu8read_data, 8);
							#endif
	#if 0
	//spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(READ_OP|ADDR_INCR|OUT_Z_H), pu8read_data, 3);
	CLEAR_CS_PIN(CS_LIS3MDL_PORT,CS_LIS3MDL_PIN);
	spi_transfer_generic((READ_OP|ADDR_CONST|TEMP_OUT_L));
	pu8read_data[0] = spi_transfer_generic(0xFF);
	spi_transfer_generic((READ_OP|ADDR_CONST|TEMP_OUT_H));
	pu8read_data[1] = spi_transfer_generic(0xFF);	
	SET_CS_PIN(CS_LIS3MDL_PORT,CS_LIS3MDL_PIN);
	uart_newline();
	uart_send_dec(pu8read_data[0]);
	uart_newline();
	uart_send_dec(pu8read_data[1]);
	uart_newline();
	get_id(LIS3MDL_MANUF_ID);
	#endif
	return u8retry_count;
}

void lis3mdl_single_meas(void) {
	uint8_t u8data =/* (1 << LP) | */(1 << MD0);
	spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(WRITE_OP|ADDR_CONST|CTRL_REG3), &u8data, 1);
}

void lis3mdl_idle(void) {
	uint8_t u8data = /*(1 << LP) |*/ (1 << MD1) | (1 << MD0);
	spi_transfer_sensors(LIS3MDL_SPI_DEV_ID,(WRITE_OP|ADDR_CONST|CTRL_REG3), &u8data, 1);
}

void lis3mdl_process_meas(const uint8_t *pu8read_data, lis3mdl_data_st *stprocessed_data) {
	uint16_t u16workint = 0;
	int32_t i32workint = 0;
	uint16_t u16gain = 0;
	uint8_t u8fullscale = LIS3MDL_FULL_SCALE;
	
	switch (u8fullscale) {
		case 4:
			u16gain = 6842;
		break;
		case 8:
			u16gain = 3421;
		break;
		case 12:
			u16gain = 2281;
		break;
		case 16:
			u16gain = 1711;
		break;
	}
	
	
	//process X magnetic data
	//u16workint = (uint16_t)CONCAT_BYTES(pu8read_data[0], pu8read_data[1]);
	u16workint = (uint16_t)CONCAT_BYTES(pu8read_data[1], pu8read_data[0]);  //use this if BLE bit is cleared
	//stprocessed_data->x_mag = conv_2compl_to_signed_dec(u16workint,16);
	i32workint = conv_2compl_to_signed_dec(u16workint,16);
	i32workint = (1000 * i32workint) / u16gain;
	stprocessed_data->x_mag = (int16_t)i32workint;
	
	//process Y magnetic data
	//u16workint = (uint16_t)CONCAT_BYTES(pu8read_data[2], pu8read_data[3]);
	u16workint = (uint16_t)CONCAT_BYTES(pu8read_data[3], pu8read_data[2]);  //use this if BLE bit is cleared
	//stprocessed_data->y_mag = conv_2compl_to_signed_dec(u16workint,16);
	i32workint = conv_2compl_to_signed_dec(u16workint,16);
	i32workint = (1000 * i32workint) / u16gain;
	stprocessed_data->y_mag = (int16_t)i32workint;

	//process Z magnetic data
	//u16workint = (uint16_t)CONCAT_BYTES(pu8read_data[4], pu8read_data[5]);
	u16workint = (uint16_t)CONCAT_BYTES(pu8read_data[5], pu8read_data[4]);  //use this if BLE bit is cleared
	//stprocessed_data->z_mag = conv_2compl_to_signed_dec(u16workint,16);	
	i32workint = conv_2compl_to_signed_dec(u16workint,16);
	i32workint = (1000 * i32workint) / u16gain;
	stprocessed_data->z_mag = (int16_t)i32workint;

	//process Temp data
	//u16workint = (uint16_t)CONCAT_BYTES(pu8read_data[6], pu8read_data[7]);
	u16workint = (uint16_t)CONCAT_BYTES(pu8read_data[7], pu8read_data[6]);  //use this if BLE bit is cleared
	//stprocessed_data->temperature = conv_2compl_to_signed_dec(u16workint,16);
	i32workint = conv_2compl_to_signed_dec(u16workint,16);
	i32workint = (i32workint >> 3) + 25;
	stprocessed_data->temperature = (int16_t)i32workint;
}