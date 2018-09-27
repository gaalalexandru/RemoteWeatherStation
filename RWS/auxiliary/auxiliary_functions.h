/*
 * auxiliary_functions.h
 *
 * Created: 9/27/2018 8:44:54 AM
 *  Author: alexandru.gaal
 */ 


#ifndef AUXILIARY_FUNCTIONS_H_
#define AUXILIARY_FUNCTIONS_H_

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define CONCAT_BYTES(msb, lsb)     (((uint16_t)msb << 8) | (uint16_t)lsb)
int16_t convert_2complement(uint16_t u16complement);
int16_t conv_2compl_to_signed_dec(uint16_t u16complement, uint8_t u8bits);


#endif /* AUXILIARY_FUNCTIONS_H_ */