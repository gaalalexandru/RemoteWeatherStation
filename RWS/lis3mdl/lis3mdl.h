/*
 * lis3mdl.h
 *
 * Created: 9/18/2018 4:29:30 PM
 *  Author: alexandru.gaal
 */ 


#ifndef LIS3MDL_H_
#define LIS3MDL_H_

typedef struct /*lis3mdl_sensor_data_s*/
{
	int16_t x_mag;		/* Measurement result for x axis */
	int16_t y_mag;		/* Measurement result for y axis */
	int16_t z_mag;		/* Measurement result for z axis */
	int16_t temperature;/* Measurement result for temperature sensor */
} lis3mdl_data_st;

uint8_t lis3mdl_init(void);
uint8_t lis3mdl_read_meas(uint8_t *pu8read_data);
void lis3mdl_process_meas(const uint8_t *pu8read_data, lis3mdl_data_st *stprocessed_data);
void lis3mdl_single_meas(void);
void lis3mdl_idle(void);

#endif /* LIS3MDL_H_ */