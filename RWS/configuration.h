/*
 * configuration.h
 *
 * Created: 8/3/2018 11:52:44 PM
 *  Author: alexandru.gaal
 */ 


#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <avr/io.h>
#include <avr/portpins.h>

//#define MAIN_LOG_ACTIV

//////////////////////////////////////////////////////////////////////////
//					CONTROLLER SELECTION AND CONFIGURATION				//
//////////////////////////////////////////////////////////////////////////
#if defined (__AVR_ATmega48PB__)  && !defined (__AVR_ATmega8__)
#define ATMEGA48 (1)
#define ATMEGA8	(0)
#elif defined (__AVR_ATmega8__)  && !defined (__AVR_ATmega48PB__)
#define ATMEGA48 (0)
#define ATMEGA8	(1)
#else
#error "Please specify one target controller"
#endif  //controller selection

#define FOSC 4000000// Clock Speed
#define F_CPU FOSC


//////////////////////////////////////////////////////////////////////////
//							STATUS LED CONFIGURATIONS					//
//////////////////////////////////////////////////////////////////////////
#define STATUS_LED_DDR		DDRD
#define STATUS_LED_PORT		PORTD
#define STATUS_LED_PIN		PIN4


//////////////////////////////////////////////////////////////////////////
//							UART CONFIGURATIONS							//
//////////////////////////////////////////////////////////////////////////
#define BAUD 38400 // Old value only for terminal control: 9600
#define SET_U2X (1)
#define DONT_USE_ISR_UART (0)
#if SET_U2X
#define MYUBRR ((FOSC/(8*BAUD))-1)
#else
#define MYUBRR ((FOSC/(16*BAUD))-1)
#endif

#define USE_UART_DEC_OUTPUT
#define USE_UART_HEX_OUTPUT
//////////////////////////////////////////////////////////////////////////
//							 SPI CONFIGURATIONS							//
//////////////////////////////////////////////////////////////////////////
#define SPI_USE_INT		(0)
#define SPI_DDR			DDRB
#define SPI_PORT		PORTB
#define SPI_MOSI_PIN	(3)
#define SPI_SS_PIN		(2)
#define SPI_MISO_PIN	(4)
#define SPI_SCK_PIN		(5)

#define CS_BME280_DDR	DDRB
#define CS_BME280_PORT	PORTB
#define CS_BME280_PIN	(0)

#define CS_LIS3MDL_DDR	DDRB
#define CS_LIS3MDL_PORT	PORTB
#define CS_LIS3MDL_PIN	(1)

#define CS_FLASH_DDR	DDRD
#define CS_FLASH_PORT	PORTD
#define CS_FLASH_PIN	(5)

//////////////////////////////////////////////////////////////////////////
//						 MEASUREMENTS CONFIGURATION						//
//////////////////////////////////////////////////////////////////////////
#define USE_LIS3MDL (0)
#define USE_BME280		(1)
#define WEATHER_MONITORING_INTERVAL_MS (60000)
#define WEATHER_MONITORING_ACCELERATION (10)

#define DATA_FRAME_SIZE	(64)
#define DATA_FRAME_SEPARATOR (0x55)

#define POWER_SAVE_ACTIV (1)

//////////////////////////////////////////////////////////////////////////
//						 ESP & WIFI CONFIGURATION						//
//////////////////////////////////////////////////////////////////////////
#define ESP_RST_PORT	PORTD
#define ESP_RST_DDR		DDRD
#define ESP_RST_PIN		PIN7
#define ESP_ENABLE_PORT	PORTD
#define ESP_ENABLE_DDR	DDRD
#define ESP_ENABLE_PIN	PIN6

#define ESP_ACCPNT_PORT		"1002"  //port of ESP Access Point TCP Server
#define ESP_SERVER_PORT		"1001"  //port of Master TCP Server

//#define WIFI_SSID_PASSWORD	"\"UPC5C34B5E\",\"jsUsje5vd4ue\"\r\n"		/*232255504335433334423545222C226A7355736A65357664347565220D0A*/
//#define WIFI_SSID_PASSWORD	"\"AndroidAP\",\"stargate\"\r\n"			/*2322416E64726F69644150222C227374617267617465220D0A*/
//#define WIFI_SSID_PASSWORD	"\"ASUS_X008D\",\"86c423b622c8\"\r\n"		/*2322415355535F5830303844222C22383663343233623632326338220D0A*/
#define WIFI_SSID_PASSWORD	"\"MyASUS\",\"Zuzuk1man\"\r\n"				/*23224D7941535553222C225A757A756B316D616E220D0A*/
//#define WIFI_SSID_PASSWORD	"\"FELINVEST\",\"1234qwe$\"\r\n"			/*232246454C494E56455354222C223132333471776524220D0A*/
//#define WIFI_SSID_PASSWORD	"\"BogdanMobile\",\"bogdan123\"\r\n" 		/*2322426F6764616E4D6F62696C65222C22626F6764616E313233220D0A*/
//#define WIFI_SSID_PASSWORD		"Blank"
#define WIFI_CHECKCONNECTION_FUNCTION	(10)	//interval in seconds between connection checks; 0 = off
#define WIFI_CHECKCONNECTION_ATTEMPTS (3)	//retry connection attempts
#endif /* CONFIGURATION_H_ */