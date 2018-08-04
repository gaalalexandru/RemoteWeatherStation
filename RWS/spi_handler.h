/*
 * spi_handler.h
 *
 * Created: 8/4/2018 12:15:48 AM
 *  Author: alexandru.gaal
 */ 


#ifndef SPI_HANDLER_H_
#define SPI_HANDLER_H_

void spi_init(void);
void spi_send_char(uint8_t u8data);
uint8_t spi_get_char( void );



#endif /* SPI_HANDLER_H_ */