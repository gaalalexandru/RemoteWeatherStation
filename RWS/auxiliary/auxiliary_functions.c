/*
 * auxiliary_functions.c
 *
 * Created: 9/27/2018 8:44:41 AM
 *  Author: alexandru.gaal
 */ 

#include <stdint.h>

int16_t conv_2compl_to_signed_dec(uint16_t u16complement, uint8_t u8bits) {
	uint8_t idx = 0;
	int16_t i16result = 0;
	
	for (idx = 0; idx < (u8bits - 1); idx++) {
		if(u16complement & (1<<idx)) {
			i16result += (1<<idx);
		}
	}
	//check the most significant bit, if this is set, we have a negative number
	if(u16complement & (1<<(u8bits - 1))) {
		i16result -= (1<<(u8bits - 1));
	}
	
	return i16result;
}

/*This is not working properly, so disable it*/
#if 0
int16_t convert_2complement(uint16_t u16complement) {
	uint8_t i;
	int16_t i16sign = 1;
	
	if(u16complement & (1 << 15)) {
		i16sign = -1;
	}
	for(i=0;i<16;i++) {
		u16complement = ((u16complement & (1 << i))?(u16complement & (~(1 << i))):(u16complement | (1 << i)));
	}
	
	return (int16_t)(i16sign * (u16complement + 1));
}
#endif