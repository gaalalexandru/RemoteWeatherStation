/*
 * uart_handler.h
 *
 * Created: 10/4/2017 10:32:57 PM
 *  Author: Gaal Alexandru
 */ 

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
   Program 4.12, Section 4.9.4, Figures 4.26 and 4.40

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#ifndef USARTHDL_H_
#define USARTHDL_H_

// standard ASCII symbols
#define CR   0x0D
#define LF   0x0A
#define BS   0x08
#define ESC  0x1B
//#define SP   0x20
#define DEL  0x7F

// Buffersizes must be a power of 2 in size !
//    This is for very simple code for a circular buffer.
// Buffersizes are in 8 bits, set it to 8, 16, 32, 64 or 128.
//    This is for very simple code without the need for atomic programming.
// The buffer is used for speed. Characters are send in an interrupt routine.
// If more characters are send than the buffer could contain, the functions waits for a place in the buffer.
// The receiving of characters does not handle overflow very well, make the buffers large enough.
// #define TX_BUFFER_SIZE 32
// #define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64
#define RX_BUFFER_SIZE 128

#define uart_newline() uart_send_char(CR); uart_send_char(LF);

/************************************************************************/
/*                 Initialization Function declaration                  */
/************************************************************************/

//---------------------uart_init---------------------
// Initialize the USART
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: unsigned int calculated BAUDRATE
// Output: none
void uart_init( uint32_t ubrr);

/************************************************************************/
/*                      Output Function declarations                    */
/************************************************************************/

//------------uart_send_char------------
// Output 8-bit to serial port
// Put ASCII or non-ASCII byte, blocks (waits) if buffer is full
// Input: data is an 8-bit ASCII character to be transferred
// Output: none
void uart_send_char( char data );

//------------uart_send_string------------
// Print string from RAM
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void uart_send_string(char *pt);

//-----------------------uart_send_udec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void uart_send_udec(uint32_t n);


//-----------------------uart_send_dec-----------------------
// Output a 32-bit number in signed decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void uart_send_dec(int32_t n);

//--------------------------uart_send_uhex----------------------------
// Output a 32-bit number in unsigned hexadecimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1 to 8 digits with no space before or after
void uart_send_uhex(uint32_t number);

/************************************************************************/
/*                       Input Function declarations                    */
/************************************************************************/

//------------uart_get_char---------------------
// Wait for new serial port input
// Put ASCII or non-ASCII byte, blocks (waits) if buffer is full
// Input: none
// Output: ASCII code for key typed
uint8_t uart_get_char( void );

//------------uart_get_string------------
// Get a string with autodetect of CR,LF,CRLF, but subject to max chars.
// Accepts ASCII characters from the serial port
//    and adds them to a string until <enter> is typed
//    or until max length of the string is reached.
// Input: pointer to empty buffer, size of buffer
// Output: Null terminated string
uint16_t uart_get_string(char *buffer, uint16_t bufsize);

/************************************************************************/
/*                      Other Function declarations                     */
/************************************************************************/
void uart_puts_P(const char *p);    // Print string from FLASH MEMORY
void uart_flush();
uint8_t uart_tx_buflen(void);   // Get number of as yet untransmitted bytes.
uint8_t uart_rx_buflen(void);   // Get number of bytes in the receive buffer or zero.

#endif /* USARTHDL_H_ */
