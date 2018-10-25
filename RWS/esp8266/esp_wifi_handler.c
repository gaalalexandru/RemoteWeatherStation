/*
 * wifi_handler.c
 *
 * Created: 10/17/2017 9:57:41 PM
 *  Author: Gaal Alexandru, Bogdan Rat
 */ 
//Wifi module type: ESP8266

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "esp_wifi_handler.h"
#include "../configuration.h"
#include "../uart_handler.h"
#include "../timer_handler.h"
//#include "status_led.h"
//#include "animation_handler.h"
//#include "pwm_handler.h"
//#include "eeprom_handler.h"

// Pins have to be digital output
// CH_PD: Chip enable. Keep it on high (3.3V) for normal operation
// RST_ESP: Reset. Keep it on high (3.3V) for normal operation. Put it on 0V to reset the chip.
#define RST_ESP_DIR	ESP_RST_DDR |= (1 << ESP_RST_PIN)
#define	CH_PD_DIR	ESP_ENABLE_DDR |= (1 << ESP_ENABLE_PIN)
#define RST_ESP_SET(x)	((x) ? (ESP_RST_PORT |= (1 << ESP_RST_PIN)) : (ESP_RST_PORT &= ~(1 << ESP_RST_PIN)))
#define	CH_PD_SET(x)	((x) ? (ESP_ENABLE_PORT |= (1 << ESP_ENABLE_PIN)) : (ESP_ENABLE_PORT &= ~(1 << ESP_ENABLE_PIN)))

/*#define ESP_DEBUG (0)*/
#define SERIAL_RESULT_BUFFER_SIZE 101

//Preliminary ESP access point states
#define ESP_AP_INIT				0
#define ESP_AP_SETMUX			1
#define ESP_AP_START_TCP_SERVER	2
#define ESP_AP_CONFIG_RECEIVE	3
#define ESP_AP_CONFIG_CHECK		4
#define ESP_AP_CONFIG_SUCCESS	5

//Preliminary ESP station states
#define ESP_STA_INIT				0
#define ESP_STA_SETMODE				1
#define ESP_STA_CONNECT				2
#define ESP_STA_CHECK_IP			3
#define ESP_STA_CHECK_STATUS		4
#define ESP_STA_SETMUX				5
#define ESP_STA_START_TCP_SERVER	6
#define ESP_STA_CHECK_CONNECTION	7	//if changes occur, update timer_handler.c also
#define ESP_STA_WAIT_COMMANDS		8

#define ESP_FORCE_WIFI_SETUP (0)

#define ESP_AP_OFF	(0)
#define ESP_AP_ON	(1)

#define true  1
#define false 0

/* To define the maximum waiting time for a response*/
#define SET_RESPONSE_TIMEOUT(x)	(response_max_timestamp =  (timer_ms() + ((x) * 1000)))

/* Will return true if timeout expired*/
#define WAITING_RESPONSE()	(response_max_timestamp > timer_ms())

/************************************************************************/
/*                           Global variables                           */
/************************************************************************/
volatile uint8_t bEspIsConnected = false;	// ESP has ip or not
volatile uint8_t bEspIsInit = false;	// ESP is fully init or not

char serialResult[SERIAL_RESULT_BUFFER_SIZE];
char clientIPString[15];
char stationIPString[15];

// store received ssid and pass to be used when connection is lost
// esp will then try to reconnect to these credentials
// rather than using macros
char wifiCredentials[32];

// most often ID is 0 but we can have up to 4 active connections
// so we want to reply to the right sender who sent ssid and pass
uint8_t senderID = 0;
	
volatile uint8_t esp_sta_current_state = 0;
volatile uint8_t esp_ap_current_state = 0;

//char esp_wifi_ssid[ESP_SSID_MAX_LENGTH];
//char esp_wifi_pass[ESP_PASS_MAX_LENGTH];

volatile uint32_t response_max_timestamp;
//extern volatile status_led_mode_t status_led_mode;

/************************************************************************/
/*                      Wifi UART interface functions                   */
/************************************************************************/

uint8_t receive_serial()
{
	memset(serialResult, 0, SERIAL_RESULT_BUFFER_SIZE-1);
	SET_RESPONSE_TIMEOUT(4);
	
	while(uart_rx_buflen() == 0 && WAITING_RESPONSE()) { /*wait*/ }
	if(uart_rx_buflen() > 0)
	{
		uart_get_string(serialResult, SERIAL_RESULT_BUFFER_SIZE-1);
		if(strlen(serialResult))
		{ //sometime it return empty string :/

			if(strstr(serialResult, "busy p..") != NULL)
			{
				timer_delay_ms(500);
				return receive_serial();  //Try again
			}
			return true;
		} 
		else
		{
			timer_delay_ms(100);
			return receive_serial(); // so, we've to try again
		}
	}
	return false;
}

static uint8_t check_return(char *compareWord)
{
	if(receive_serial())
	{
		if(strstr(serialResult, compareWord) != NULL)
		{
			return true;
		}
		else	
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

static uint8_t check_until_timeout(char *compareWord, uint8_t maxWaitTime)
{
	uint32_t timeoutTimestamp  = timer_ms() + (maxWaitTime * 1000);
	do {
		if(check_return(compareWord)) {
			return true;
		}
	} while(timer_ms() < timeoutTimestamp);
	return false;
}

static uint8_t send_command(char *sentCommand, char *compareWord)
{
	uart_send_string(sentCommand); uart_send_char('\r'); uart_send_char('\n');
	if(!check_until_timeout(sentCommand, 1))
	{
		return false;		
	}	
	return check_until_timeout(compareWord, 1);
}

//helper function that extracts station IP
//extracted data is stored in global variables
inline static void esp_aux_calc_station_ip(char *workString)
{
	char *stationIP_begin = NULL;
	char *stationIP_end = NULL;
	stationIP_begin = strstr(workString, "STAIP");
	stationIP_begin += 7;
	stationIP_end = strchr(stationIP_begin, 0x22);	// ' " '
	uart_flush();
	strncpy(stationIPString, stationIP_begin, (uint8_t)(stationIP_end-stationIP_begin));
}

//helper function that extracts client ID and IP
//extracted data is stored in global variables
inline static void esp_aux_client_data(char *workString)
{
	char *clientIP_begin = NULL;
	char *clientIP_end = NULL;
	senderID = atoi(workString);	// store ID of sender
	workString += 2;  //jump to "," before message length
	clientIP_begin = strchr(workString, 0x2c);	//jump to next ','
	clientIP_begin++;  //jump to start of IP
	clientIP_end = strchr(clientIP_begin, 0x2c);	//jump to next ','
	strncpy(clientIPString, clientIP_begin, (uint8_t)(clientIP_end-clientIP_begin));
}

// Creates TCP server and sends data
// ID is the connection number
// destination is the IP to which data is sent
// message is what is sent
static void esp_response(uint8_t ID, char *destination, char *message)
{
	//AleGaa The next block is deactivated because we will use persistent TCP connection when communicating with ESP AP
	//Note: Rarely strange behavior was observed, sometimes when joining the router wifi network, the connection is closed
	#if 0  
	uart_send_string("AT+CIPSTART=");
	uart_send_udec(ID);
	uart_send_string(",\"TCP\"");
	uart_send_char(0x2c);	// ,
	uart_send_char(0x22);	// "
	uart_send_string(destination);
	uart_send_char(0x22);	// "
	uart_send_char(0x2c);	// ,
	uart_send_string(ESP_CFG_DEV_PORT);
	uart_newline();
	#endif
	
	#if 0 //Wait for TCP server success signal before sending data to client
	char workString[32];
	memset(workString, 0, 32);
	do
	{
		uart_get_string(workString, 32);	// at this point, workString contains response from AT+CIPSTART
	} while ((strstr(workString, "OK") == NULL) && (strstr(workString, "ALREADY") == NULL));
	#else
	timer_delay_ms(2000);
	#endif
	
	uart_send_string("AT+CIPSEND=");
	uart_send_udec(ID);
	uart_send_char(0x2c);	// ,
	uart_send_udec(strlen(message));
	uart_newline();
	
	timer_delay_ms(200);
	uart_send_string(message);
	uart_newline();
}

/************************************************************************/
/*                        Wifi handling functions                       */
/************************************************************************/


void esp_init(void)
{
	//Set the direction and value of ESP 8266 Reset (RST) and Enable (CH_PD) pin
	RST_ESP_DIR;
	CH_PD_DIR;
	RST_ESP_SET(1);
	RST_ESP_SET(0);
	RST_ESP_SET(1);
	CH_PD_SET(1);
	timer_delay_ms(6000);  //Wait 6 second until ESP is started and finishes standard junk output :)
	//Set ESP8266 mode (1 = Station, 2 = Soft Access Point, 3 = Sta + SoftAP)
	//AleGaa: Since CWMODE is set in flash memory of ESP, might be useful
	//to set the mode only at "manufacturing" via serial terminal
	//and do not set it on runtime.
	//this could make the initialization slightly simpler and faster
	//feature to be implemented TODO
	bEspIsInit = false;
	strcpy(wifiCredentials, WIFI_SSID_PASSWORD);
	while(!send_command("AT+CWMODE=3", "OK")) {};  //setup AccessPoint and STAtion mode
}

// fills parameter with the response for AT+CIFSR command
// and also returns true/false if station has/hasn't got an IP
static inline uint8_t esp_check_connection(char workString[])
{
	
	/*
	To check if we are already connected to a wifi network:
	Send command to ESP "AT+CIFSR"
	Check if response string contains station IP nr. 0.0.0.0.
	If string doesn't contain "STAIP,\"0.0.", pointer of strstr().
	will be NULL and the condition will be true, esp has IP.
	If string contains "STAIP,\"0.0.", pointer off strstr() not NULL,
	the condition will be false, esp has no IP.
	*/
	memset(workString, 0, SERIAL_RESULT_BUFFER_SIZE-1);
	uint8_t u8index = 0;  // index for buffer, and character counter.
	
	uart_flush();
	uart_send_string("AT+CIFSR\r\n");
	timer_delay_ms(200);
	
	do
	{
		workString[u8index] = uart_get_char();
		if((workString[u8index] != '\n')  && (workString[u8index] != '\r'))
		{
			u8index++;
		}
	}
	while (strstr(workString,"STAMAC") == NULL);
	
	uart_flush();
	
	if(strstr(workString, "STAIP,\"0.0.") == NULL)
	//if return string doesn't contain 0.0. esp has IP
	{
		return true;  
	}
	return false;
}



static inline void esp_ap_control(uint8_t ap_new_state)
{
	static uint8_t ap_old_state = ESP_AP_ON;
	if((ap_new_state == ESP_AP_OFF) && (ap_old_state != ESP_AP_OFF))
	{
		if(send_command("AT+CWMODE=1", "OK"))
		{
			ap_old_state = ap_new_state;
		}
	}
	else if((ap_new_state == ESP_AP_ON) && (ap_old_state != ESP_AP_ON))
	{
		if(send_command("AT+CWMODE=3", "OK"))
		{
			ap_old_state = ap_new_state;
		}
	}
	else
	{
		//do nothing
	}
}

void esp_check_current_setup(void)
{
#if ESP_FORCE_WIFI_SETUP  //for development of wifi setup functionalities
	esp_wifi_setup();
#else //normal run
	/* 
	Call "esp_check_connection()" function, if the return result is
	TRUE: In this case we can proceed to MUX setting and TCP server start.
	FALSE: will start the routines for new wifi network setup.
	*/
	char ipCheckResult[SERIAL_RESULT_BUFFER_SIZE];
	if(esp_check_connection(ipCheckResult))  //esp station has IP
	{
		//Start ESP State Machine
		//Go directly to MUX setting
		bEspIsConnected = true;
		esp_sta_current_state = ESP_STA_SETMUX;
	}
	else  //esp station has no IP
	{
		//Start ESP Access Point and get wifi configuration from mobile device
		esp_wifi_setup();
	}
#endif //ESP_FORCE_WIFI_SETUP	
}

// this function will turn on and setup the ESP Access Point
// mobile device will connect to this network
// mobile device will send the SSID and password of home network
// ESP will send to mobile device the IP of ESP on home network
void esp_wifi_setup(void)
{
	char ipCheckResult[SERIAL_RESULT_BUFFER_SIZE];
	//memset(ipCheckResult, 0, SERIAL_RESULT_BUFFER_SIZE-1);
	
	char workString[32];
	uint8_t CWJAP_OK = 0;
	uint8_t CWJAP_FAIL = 0;
	char *currStrPos = NULL;
	
	esp_ap_control(ESP_AP_ON);	
	
	esp_ap_current_state = ESP_AP_INIT;
	bEspIsInit = false;
	bEspIsConnected = false;
	memset(workString, 0, 32);
	memset(clientIPString, 0, 15);
	memset(stationIPString, 0, 15);
	
	while(esp_ap_current_state < ESP_AP_CONFIG_SUCCESS)
	{
		switch (esp_ap_current_state)
		{
			case ESP_AP_INIT:
				//Synchronize ATMEGA8 with ESP8266
				if(send_command("AT", "OK"))
				{
					//esp_sta_current_state = ESP_AP_SETMODE;
					//AleGaa: skip setmode command for now.
					esp_ap_current_state = ESP_AP_SETMUX;
				}
				uart_flush();
			break;
			case ESP_AP_SETMUX:
				//Set ESP to accept multiple connections
				if(send_command("AT+CIPMUX=1", "OK"))
				{
					//why was this here? todo: uart_send_string(serialResult);
					esp_ap_current_state = ESP_AP_START_TCP_SERVER;
				}
				uart_flush();
			break;			
			case ESP_AP_START_TCP_SERVER:
				send_command("AT+CIPSERVER=0", "OK");
				//Start TCP server on a manually selected port
				//if(send_command("AT+CIPSERVER=1,1002", "OK"))
				#if 1
				uart_send_string("AT+CIPSERVER=1,");
				uart_send_string(ESP_ACCPNT_PORT);
				uart_newline();
				timer_delay_ms(100);
				send_command("AT+CIPSTO=60", "OK");	
				send_command("AT+CIPDINFO=1", "OK");
				esp_ap_current_state = ESP_AP_CONFIG_RECEIVE;
				#endif
				
				#if 0
				if(send_command(strcat("AT+CIPSERVER=1,", ESP_AP_PORT), "OK"))
				{
					send_command("AT+CIPDINFO=1", "OK");	// detailed information (IP & PORT) in +IPD
					send_command("AT+CIPSTO=60", "OK");		// time until tcp server connection is closed
					esp_ap_current_state = ESP_AP_CONFIG_RECEIVE;
				}
				#endif
				uart_flush();
			break;
			case ESP_AP_CONFIG_RECEIVE:
				/*
				1. mobile device -> ESP: #\"SSID\",\"PASSWORD"\
				2a. CURRENT VERSION: ATMEGA store to 1 big string the SSID, PASSWORD in the req format
				2b. FEATURE VERSION (TO DO): ATMEGA store to separate string the SSID, PASSWORD, PORT
				3. ESP connect to SSID with PASSWORD (enable station mode)
				4. ESP check IP, if OK go to 5, if not OK, go to 1.
				5. ESP -> mobile device: #stationIP
				*/
				
				CWJAP_OK = 0;
				CWJAP_FAIL = 0;

				if(check_until_timeout("+IPD,", 5))
				{
					//response example:
					//+IPD,0,25,192.168.4.2,50511:#"FELINVEST","1234qwe$"
					currStrPos = strstr(serialResult, "+IPD,");  //find start of response
					currStrPos += 5;	// jump to ID of sender
					esp_aux_client_data(currStrPos);				
					currStrPos = strchr(currStrPos, '#');  //find start of SSID
					currStrPos++;
					strcpy(workString, currStrPos);		// at this point, workString contains SSID and PASS
					strcpy(wifiCredentials, currStrPos);
					esp_ap_current_state = ESP_AP_CONFIG_CHECK;
				}
				uart_flush();
			break;
			
			case ESP_AP_CONFIG_CHECK:
				//connect to new SSID
				uart_flush();
				bEspIsConnected = false;
				timer_delay_ms(1000);
				uart_send_string("AT+CWJAP=");	// at this point, workString contains SSID and PASS
				uart_send_string(workString);
				uart_newline();
				timer_delay_ms(1000);
				
				do
				{
					uart_get_string(workString, 32);	// at this point, workString contains response from AT+CWJAP
					if(strstr(workString, "OK") != NULL)
					{
						CWJAP_OK = 1;
					}
					else if(strstr(workString, "FAIL") != NULL)
					{
						CWJAP_FAIL = 1;
					}				
				} while ((CWJAP_OK == 0) && (CWJAP_FAIL == 0));
				
				// if esp succeeded in connecting to ssid and pass provided
				if(CWJAP_OK == 1)
				{
					timer_delay_ms(1000);
					if(esp_check_connection(ipCheckResult))  //esp station has IP
					{
						esp_aux_calc_station_ip(ipCheckResult);
						esp_response(senderID, clientIPString, stationIPString);
						timer_delay_ms(2000);
						
						//bEspIsConnected = true;
						esp_ap_current_state = ESP_AP_CONFIG_SUCCESS;
					}
				}
				
				// if esp failed in connecting to ssid and pass provided, go back to receive ssid and pass state
				else if(CWJAP_FAIL == 1)
				{
					esp_response(senderID, clientIPString, "Could not connect");
					bEspIsConnected = false;
					timer_delay_ms(2000);
					esp_ap_current_state = ESP_AP_CONFIG_RECEIVE;
				}
				
			uart_flush();
			break;
		}
	}
	#if 0
	if (esp_ap_current_state == ESP_AP_CONFIG_SUCCESS)
	{
		if(eeprom_read_byte(EEL_AP_ALWAYS_ON) == AP_NOT_ALWAYS_ON)
		{
			esp_ap_control(ESP_AP_OFF);
		}
		esp_sta_current_state = ESP_STA_START_TCP_SERVER;
	}
	#endif
}

void esp_state_machine(void)
{
	char ipCheckResult[SERIAL_RESULT_BUFFER_SIZE];
	char workString[32];
	static uint8_t retry_connect = 0;
	static uint8_t noconnection_count = 0;
	char *currStrPos, *dataPtr;	
	uint8_t CWJAP_OK = 0;
	uint8_t CWJAP_FAIL = 0;
	uint8_t u8work_int = 0;  //uint8 for general use
	
	memset(workString, 0, 32);
	
	while(esp_sta_current_state < ESP_STA_WAIT_COMMANDS) 
	{
		switch (esp_sta_current_state)
		{
			case ESP_STA_INIT:
				//Synchronize ATMEGA8 with ESP8266
				if(send_command("AT", "OK"))
				{
					esp_sta_current_state = ESP_STA_CONNECT/*ESP_STA_SETMODE*/;
					//status_led_mode = wait_for_ip;
				}
				uart_flush();	
			break;

			case ESP_STA_CONNECT:
				//Set Access Point SSID and Password
				//send_command not used, because it's necessary to wait a little longer
				//before checking for OK response
				CWJAP_OK = 0;
				CWJAP_FAIL = 0;
				
				uart_flush();
				uart_send_string("AT+CWJAP=");
				uart_send_string(wifiCredentials);
				uart_newline();
				timer_delay_ms(1000);
				
				#if 0
				do
				{
					uart_get_string(workString, 32);	// at this point, workString contains response from AT+CWJAP
					if(strstr(workString, "OK") != NULL)
					{
						CWJAP_OK = 1;
					}
					else if(strstr(workString, "FAIL") != NULL)
					{
						CWJAP_FAIL = 1;
					}
				} while ((CWJAP_OK == 0) && (CWJAP_FAIL == 0));
				
				if(CWJAP_OK == 1)
				{
					noconnection_count = 0;
					esp_sta_current_state = ESP_STA_SETMUX;
				}
				else if(CWJAP_FAIL == 1)
				{
					noconnection_count++;
					if(noconnection_count == WIFI_CHECKCONNECTION_ATTEMPTS)
					{
						uart_flush();
						timer_delay_ms(100);
						esp_wifi_setup();
					}
				}
				#endif
				
				if(esp_check_connection(ipCheckResult))
				{
					noconnection_count = 0;
					esp_sta_current_state = ESP_STA_SETMUX;
				}
				else
				{
					noconnection_count++;
					if(noconnection_count == WIFI_CHECKCONNECTION_ATTEMPTS)
					{
						uart_flush();
						timer_delay_ms(100);
						esp_wifi_setup();
					}
				}
				
				uart_flush();
			break;		
			
			#if 0
			case ESP_STA_CHECK_IP:		
				//Check if ESP received IP from AP
				if(esp_check_connection(ipCheckResult))  //esp station has IP
				{
					//IP received, meaning successful connection to router
					esp_sta_current_state = ESP_STA_SETMUX;
				}
				else
				{
					//This will be true if no IP received from router
					esp_sta_current_state = ESP_STA_CONNECT;  //Reset state to connect
					retry_connect++;
					if(retry_connect > 5)
					{
						status_led_mode = error_indication;
						//AleGaa: In case of unsuccessful connection to AP
						//reset the system or raise a visual warning
						//feature to be implemented TODO
					}					
				}
				uart_flush();
			break;
			#endif
			
			case ESP_STA_SETMUX:
				//Set ESP to accept multiple connections
				if(send_command("AT+CIPMUX=1", "OK"))
				{
					esp_sta_current_state = ESP_STA_START_TCP_SERVER;
				}
				uart_flush();
			break;
			
			case ESP_STA_START_TCP_SERVER:
				//Close previously created server; replies "no change" if none was created before
				send_command("AT+CIPSERVER=0", "OK");
				timer_delay_ms(100);
				//Start STA TCP server on selected port
				if(send_command("AT+CIPSERVER=1,1001", "OK"))
				{
					esp_sta_current_state = ESP_STA_WAIT_COMMANDS;
					if(bEspIsConnected == false)
					{
						#if STARTUP_ANIMATION_FUNCTION
						animation_init();
						#endif //STARTUP_ANIMATION_FUNCTION
					}
					bEspIsConnected = true;
					bEspIsInit = true;
				}
				uart_flush();
			break;
			
			case ESP_STA_CHECK_CONNECTION:
				if(esp_check_connection(ipCheckResult))  //esp station has IP
				{
					// if it was disconnected before, restart animation
					if(bEspIsConnected == false)
					{
						#if STARTUP_ANIMATION_FUNCTION
						animation_init();
						#endif //STARTUP_ANIMATION_FUNCTION
					}
					
					bEspIsConnected = true;
					bEspIsInit = true;
					noconnection_count = 0;
					
					esp_sta_current_state = ESP_STA_WAIT_COMMANDS;
				}
				else
				{
					bEspIsInit = false;
					esp_sta_current_state = ESP_STA_INIT;
					//Turn AP back on if necessary
				}
				uart_flush();
			break;
		}  //end of switch (esp_sta_current_state)
	}  //end of while( esp_sta_current_state < ESP_STATE_WAIT_COMMANDS)

	
	//Command format sent through TCP:
	//"+IPD,x,y:$H#DD"
	//x = connection number
	//y = number of characters in message, after ":"
	//$ = start of channel number character(s)
	//H = HEX value of PWM channel number to be changed
	//# = start of PWM duty cycle value character(s)
	//DD = HEX value of PWM duty cycle for selected channel
	if(esp_sta_current_state == ESP_STA_WAIT_COMMANDS) 
	{
		//status_led_mode = connected_to_ap;
		if(check_until_timeout("+IPD,", 1))
		{
			currStrPos = strstr(serialResult, "+IPD,");		
			esp_aux_client_data(serialResult);
			currStrPos = strchr(currStrPos, ':');  //find end of client IP, Port nr
			dataPtr = currStrPos + 1;
			
			if(*dataPtr == '$')  //if we receive a command
			{
				esp_response(senderID, clientIPString, "Error");
			}
			else if(*dataPtr == '#') //if we receive a command
			{
				dataPtr++;
				switch (*dataPtr)
				{
					case 'E':  //E command: get STA IP
						if(esp_check_connection(ipCheckResult))  //esp station has IP
						{
							esp_aux_calc_station_ip(ipCheckResult);
							esp_response(senderID, clientIPString, stationIPString);
						}
					break;
					
					case 'F':  //F command: activate / deactivate ESP auto connect to saved network
						dataPtr++;
						if(*dataPtr == '0')  //deactivate auto connect
						{
							send_command("AT+CWAUTOCONN=0", "OK");
							esp_response(senderID, clientIPString, "0");
						}
						else if(*dataPtr == '1') //activate auto connect
						{
							send_command("AT+CWAUTOCONN=1", "OK");
							esp_response(senderID, clientIPString, "1");
						}
						else{ /*do nothing*/ }				
					break;
					#ifdef ESP_CUSTOM_COMMANDS					
					case 'G':  //G command
						esp_response(senderID, clientIPString, "response");		
					break;
					case 'H':  //H command
						esp_response(senderID, clientIPString, "response");
					break;
					case 'I':  //I command
						esp_response(senderID, clientIPString, "response");
					break;
					case 'J':  //J command
						esp_response(senderID, clientIPString, "response");
					break;
					case 'K': //K command
						esp_response(senderID, clientIPString, "response");
					break;
					case 'L': //L command
						esp_response(senderID, clientIPString, "response");
					break;
					#endif //ESP_CUSTOM_COMMANDS	
					default:
						//do nothing
					break;
				}
				u8work_int = 0;  //reset
				timer_delay_ms(2000);
			}

			// if +IPD command is sent at the same time as AT+CIFSR triggered from timer
			// then +IPD is not recognized and pwm is not updated
			// client would have to send the command again
			uart_flush();
		}
		
		#if 1
		// check connection only when WIFI [DISCONNECT] is found in serialResult
		// up to this point, receive_serial function was called above
		// this seems to work best for a beta version
		// more tests needed to see if fully stable
		#if WIFI_CHECKCONNECTION_FUNCTION
		if(strstr(serialResult, "WIFI") != NULL)
		{
			uart_flush();
			esp_sta_current_state = ESP_STA_CHECK_CONNECTION;
		}
		#endif	//WIFI_CHECKCONNECTION_FUNCTION
		#endif
	}
}

