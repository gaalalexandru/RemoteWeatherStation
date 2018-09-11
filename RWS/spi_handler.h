/*
 * spi_handler.h
 *
 * Created: 8/4/2018 12:15:48 AM
 *  Author: alexandru.gaal
 */ 


#ifndef SPI_HANDLER_H_
#define SPI_HANDLER_H_

void spi_init(void);
uint8_t spi_transfer(uint8_t u8data);
int8_t spi_transfer_bme280(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

#endif /* SPI_HANDLER_H_ */