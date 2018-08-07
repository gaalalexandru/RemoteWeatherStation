/*
 * usart_handler.c
 *
 * Created: 10/4/2017 10:17:44 PM
 *  Author: Gaal Alexandru
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdbool.h>
#include "configuration.h"
#include "uart_handler.h"

#define TXMASK (TX_BUFFER_SIZE-1)
#define RXMASK (RX_BUFFER_SIZE-1)

//controller adaptations for ATMEGA48
#if ATMEGA48
#define UCSRA	UCSR0A
#define UCSRB	UCSR0B
#define UCSRC	UCSR0C

#define UBRRH	UBRR0H
#define UBRRL	UBRR0L
#define U2X		U2X0

#define RXEN	RXEN0
#define TXEN	TXEN0
#define RXCIE	RXCIE0
#define UDRIE	UDRIE0
#define RXC		RXC0
#define UDR		UDR0

#define USART_RXC_vect USART_RX_vect

#endif
/************************************************************************/
/*                           Global variables                           */
/************************************************************************/

// The variables are static (only valid withing this file) and volatile (used in interrupts).
static volatile unsigned char txbuf[TX_BUFFER_SIZE];    // transmit buffer
static volatile unsigned char rxbuf[RX_BUFFER_SIZE];    // receive buffer

static volatile unsigned char tx_in;   // buffer head/tail indices
static volatile unsigned char tx_out;  // NOTE! these are 1 byte; no atomic for double byte needed

static volatile unsigned char rx_in;
static volatile unsigned char rx_out;

static volatile unsigned char cr_flag;  // added for auto detect CR,LF,CRLF

/************************************************************************/
/*                 Initialization Function definitions                  */
/************************************************************************/

/* UART Initialization function*/
void uart_init(uint32_t ubrr)
{
	UCSRA = 0x00; // turn everything off
	UCSRB = 0x00;
	UCSRC = 0x00;
	UBRRH = (uint8_t)(ubrr>>8);//Set baud rate
	UBRRL = (uint8_t)ubrr;
	#if SET_U2X
	UCSRA = (1<<U2X);
	#endif
	//Receive and transmit enabled, receive complete interrupt enabled (USART_RXC)
	UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);
		
	#if ATMEGA48
	//8 data bits, 1 stop bit, UART operation, no parity
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	#elif ATMEGA8
	// Set the most used serial settings: asynchrone, no parity, 8 bit, 1 stop bit.
	// The ATmega8 uses an URSEL bit, which is not present on newer chips.
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
	#endif
	
	cr_flag = false;  // set default, no 'cr' detected yet.
	tx_in = tx_out = rx_in = rx_out = 0;  // set all buffer indices to zero.
}

/************************************************************************/
/*                      Output Function definitions                     */
/************************************************************************/

/* UART Send 1 Byte function */
void uart_send_char(char data)
{
	// If the character does not fit in buffer, it will still be transmitted, but it waits
	// until characters are transmitted and free space is available in the buffer.
	// If the transmit-buffer is full, wait for it, interrupts will empty it
	// There is no timeout.
	while((TX_BUFFER_SIZE - uart_tx_buflen()) <= 2) {}
	// Add data to the transmit buffer, enable transmit interrupts
	txbuf[tx_in & TXMASK] = data;  // set character in circular buffer
	tx_in++;  // increment head index
	UCSRB |= (1<<UDRIE);  // Enable UDR empty interrupt, the ISR will move from buffer to UART
}

/* UART Send String function */
void uart_send_string(char *pt){
	while(*pt){
		uart_send_char(*pt);
		pt++;
	}
}

#ifdef USE_UART_DEC_OUTPUT
/* UART Send Unsigned Decimal function */
void uart_send_udec(uint32_t n)
{
	// This function uses recursion to convert decimal number
	//   of unspecified length as an ASCII string
	if(n >= 10){
		uart_send_udec(n/10);
		n = n%10;
	}
	uart_send_char(n+'0'); //n is between 0 and 9
}
#endif  //USE_UART_DEC_OUTPUT

#ifdef USE_UART_HEX_OUTPUT
/* UART Send Unsigned Hex function */
void uart_send_uhex(uint32_t number)
{
	// This function uses recursion to convert the number of
	// unspecified length as an ASCII string
	if(number >= 0x10){
		uart_send_uhex(number/0x10);
		uart_send_uhex(number%0x10);
	}
	else{
		if(number < 0xA) {
			uart_send_char(number+'0');
		}
		else {
			uart_send_char((number-0x0A)+'A');
		}
	}
}
#endif

/************************************************************************/
/*                      Input Function definitions                      */
/************************************************************************/

/* UART Get 1 Byte function */
uint8_t uart_get_char( void )
{
	unsigned char data;
	// Wait for data. See also UART_rbuflen()
	while (uart_rx_buflen() == 0) {}
	data = rxbuf[rx_out & RXMASK];
	rx_out++;
	return data;
	
	#if DONT_USE_ISR_UART
	while ( !(UCSRA & (1<<RXC)) ) {}
	return(UDR);
	#endif
}

/* UART Get String function */
uint16_t uart_get_string(char *buffer, uint16_t bufsize)
{
	//Get string that ends with a CR, LF or CRLF,  but subject to max chars
	//The return value is the string length.
	//The string is terminated with '\0' if it reaches the maximum size.
	//But if the maximum size is reached, the function still waits for a CR or LF.
	//A flag 'cr_flag' is used for auto detection of CR,LF,CRLF.
	//Note that due to the autodetection of CR,LF,CRLF, the UART_rbuflen()
	//is not the same as the strlen() of the returned string.
	//For example: UART_rbuflen() could be 2, and the string could be empty (or not).
	//Note that mixing UART_gets() and UART_getc() could break the autodetection of CR,LF,CRLF.
	
	uint8_t i = 0;  // index for buffer, and character counter.
	uint8_t eol = false;  // flag for end-of-line
	
	if (bufsize > 0)
	{
		while (!eol)
		{
			buffer[i] = (char) uart_get_char();  // get a fresh character from the input buffer
			if (buffer[i] == '\r')  // check for CR
			{
				cr_flag = true;
				eol = true;  // end this line
			}
			else if (buffer[i] == '\n')  // check for LF
			{
				if (cr_flag) {  // Previous character was CR, ignore the LF, don't advance index.
					cr_flag = false;
				}
				else {  // No CR flag: end this line.
					eol = true;
				}
			}
			else
			{
				cr_flag = false;  // normal character, clear flag
				if (i < (bufsize - 1)) { // is there still a place for the next character ?
					i++;  // advance to next position.
				}
			}
		}
		// The CR ('\r') or LF ('\n') is overwritten by the string terminator.
		buffer[i] = '\0';  // End the string with '\0'
	}
	// Return the length of the string.
	// It could be a truncated string if too many characters were received.
	// It should be the same as strlen(buffer)
	return (i);
}

/************************************************************************/
/*                      Other Function definitions                      */
/************************************************************************/

void uart_flush(){
	while(uart_rx_buflen() > 0) 
	{
		uart_get_char();
	}
	rx_in = rx_out = 0;
	cr_flag = false;
}

// Number of bytes as yet untransmitted
// Don't call this from an ISR
uint8_t uart_tx_buflen (void)
{
	return(tx_in - tx_out);
}

// Number of bytes in receive buffer
// Call this to avoid blocking in UART_getc()
uint8_t uart_rx_buflen (void)
{
	return (rx_in - rx_out);
}

/************************************************************************/
/*                    Interrupt Function definitions                    */
/************************************************************************/

// Transmit complete interrupt
// USART transmit Data Register Empty interrupt.
ISR (USART_UDRE_vect)
{
	if(tx_in != tx_out)  // is buffer empty ?
	{
		UDR = txbuf[tx_out & TXMASK];
		tx_out++;  // TMASK makes it unnecessary to range limit this
	}
	else
	{
		UCSRB &= ~(1 << UDRIE);  // buffer is empty, disable interrupt
	}
}

// Receive interrupt
// In case of an overflow of the receive buffer,
// the old data is overwritten, but not in a circular way.
// Good:
//    A CR or LF at the end is still passed on.
// Bad:
//    The characters before the CR or LF might all be gone,
//    if the number of received characters is the same as the input buffer.
//
ISR (USART_RXC_vect)
{
	rxbuf[rx_in & RXMASK] = UDR;  // Put received char in buffer (no check for overflow)
	rx_in++;  // RMASK makes it unnecessary to range limit this
}