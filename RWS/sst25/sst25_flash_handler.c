/*********************************************************************
 *
 *  SPI Flash Memory Driver
 *  - Tested with SST 25VF016B 
 *  - Expected compatibility with other SST (Microchip) SST25 series 
 *    devices
 *
 *********************************************************************
 * FileName:        SPIFlash.c
 * Dependencies:    SPIFlash.h
 * Processor:       PIC18, PIC24F, PIC24H, dsPIC30F, dsPIC33F, PIC32
 * Compiler:        Microchip C32 v1.11b or higher
 *					Microchip C30 v3.23 or higher
 *					Microchip C18 v3.30 or higher
 *					HI-TECH PICC-18 PRO 9.63PL2 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright (C) 2002-2010 Microchip Technology Inc.  All rights
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy, and
 * distribute:
 * (i)  the Software when embedded on a Microchip microcontroller or
 *      digital signal controller product ("Device") which is
 *      integrated into Licensee's product; or
 * (ii) ONLY the Software driver source files ENC28J60.c, ENC28J60.h,
 *		ENCX24J600.c and ENCX24J600.h ported to a non-Microchip device
 *		used in conjunction with a Microchip ethernet controller for
 *		the sole purpose of interfacing with the ethernet controller.
 *
 * You should refer to the license agreement accompanying this
 * Software for additional information regarding your rights and
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 *
 * Author               		Date    Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * E. Wood              		3/20/08 Original
 * Dave Collier/H. Schlunder	6/09/10	Update for SST25VF010A
 * Alexandru Gaal				9/12/18 Renamed and removed adaptations 
                                        for PIC controllers
********************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "../configuration.h"
#include "../spi_handler.h"
#include "../uart_handler.h"

#include "sst25_flash_handler.h"

#define SST25_LOG_ACTIV (1)  //enable UART log for sst25_flash_handler module

#define READ				0x03    // SPI Flash opcode: Read up up to 25MHz
#define READ_FAST			0x0B    // SPI Flash opcode: Read up to 50MHz with 1 dummy byte
#define ERASE_4K			0x20    // SPI Flash opcode: 4KByte sector erase
#define ERASE_32K			0x52    // SPI Flash opcode: 32KByte block erase
#define ERASE_SECTOR		0xD8    // SPI Flash opcode: 64KByte block erase
#define ERASE_ALL			0x60    // SPI Flash opcode: Entire chip erase
#define WRITE				0x02    // SPI Flash opcode: Write one byte (or a page of up to 256 bytes, depending on device)
#define WRITE_WORD_STREAM	0xAD    // SPI Flash opcode: Write continuous stream of 16-bit words (AAI mode); available on SST25VF016B (but not on SST25VF010A)
#define WRITE_BYTE_STREAM	0xAF    // SPI Flash opcode: Write continuous stream of bytes (AAI mode); available on SST25VF010A (but not on SST25VF016B)
#define RDSR				0x05    // SPI Flash opcode: Read Status Register
#define EWSR				0x50    // SPI Flash opcode: Enable Write Status Register
#define WRSR				0x01    // SPI Flash opcode: Write Status Register
#define WREN				0x06    // SPI Flash opcode: Write Enable
#define WRDI				0x04    // SPI Flash opcode: Write Disable / End AAI mode
#define RDID				0x90    // SPI Flash opcode: Read ID
#define JEDEC_ID			0x9F    // SPI Flash opcode: Read JEDEC ID
#define EBSY				0x70    // SPI Flash opcode: Enable write BUSY status on SO pin
#define DBSY				0x80    // SPI Flash opcode: Disable write BUSY status on SO pin

#define BUSY    0x01    // Mask for Status Register BUSY (Internal Write Operaiton in Progress status) bit
#define WEL     0x02    // Mask for Status Register WEL (Write Enable status) bit
#define BP0     0x04    // Mask for Status Register BP0 (Block Protect 0) bit
#define BP1     0x08    // Mask for Status Register BP1 (Block Protect 1) bit
#define BP2     0x10    // Mask for Status Register BP2 (Block Protect 2) bit
#define BP3     0x20    // Mask for Status Register BP3 (Block Protect 3) bit
#define AAI     0x40    // Mask for Status Register AAI (Auto Address Increment Programming status) bit
#define BPL     0x80    // Mask for Status Register BPL (BPx block protect bit read-only protect) bit


// Internal pointer to address being written
static uint32_t g_u32WriteAddr;

// SPI Flash device capabilities
static union
{
	uint8_t v;
	struct
	{
		uint8_t bWriteWordStream : 1;	// Supports AAI Word opcode (0xAD)
		uint8_t bWriteByteStream : 1;	// Supports AAI Byte opcode (0xAF)
		uint8_t bPageProgram : 1;		// Supports Byte program opcode with up to 256 bytes/page (0x02)
		uint8_t filler : 5;
	} bits;
} deviceCaps;


static void _WaitWhileBusy(void);
//static void _GetStatus(void);


/*****************************************************************************
  Function:
	void sst25_flash_init(void)

  Description:
	Initializes SPI Flash module.

  Precondition:
	None

  Parameters:
	None

  Returns:
	None

  Remarks:
	This function is only called once during the lifetime of the application.

  Internal:
	This function sends WRDI to clear any pending write operation, and also
	clears the software write-protect on all memory locations.
  ***************************************************************************/
void sst25_flash_init(void)
{
	volatile uint8_t u8SerialReturnData = 0;

	// Activate chip select
	ENABLE_CS_FLASH;
	
	// Read Device ID code to determine supported device capabilities/instructions
	// Send instruction
	spi_transfer(RDID);
	// Send 3 byte address (0x000000), returned value of spi_transfer is not important now
	spi_transfer(0x00);  //i=0
	spi_transfer(0x00);  //i=1
	spi_transfer(0x00);  //i=2
	
	//get Manufacture ID
	u8SerialReturnData = spi_transfer(0x00);  //i=3, sent value through spi_transfer is not important now
	#if SST25_LOG_ACTIV
	uart_send_string("manufacturer ID: ");
	uart_send_char(u8SerialReturnData);
	uart_newline();
	#endif //SST25_LOG_ACTIV
	
	//get Device ID
	u8SerialReturnData = spi_transfer(0x00);  //i=4, sent value through spi_transfer is not important now
	#if SST25_LOG_ACTIV
	uart_send_string("device ID: ");
	uart_send_char(u8SerialReturnData);
	uart_newline();	
	#endif //SST25_LOG_ACTIV
	
	// Deactivate chip select
	DISABLE_CS_FLASH;
	
	// Decode Device Capabilities Flags from Device ID
	deviceCaps.v = 0x00;
	switch(u8SerialReturnData)
	{
		case 0x43:	// SST25LF020(A)	(2 Mbit)	0xAF, 14us, AAI Byte
		case 0x48:	// SST25VF512(A)	(512 Kbit)	0xAF, 14us, AAI Byte
		case 0x49:	// SST25VF010A		(1 Mbit)	0xAF, 14us, AAI Byte
			deviceCaps.bits.bWriteByteStream = 1;
			break;
			
		case 0x4B:	// SST25VF064C		(64 Mbit)	0x02, 1.5ms/256 byte page, no AAI
			deviceCaps.bits.bPageProgram = 1;
			break;
		//case 0x01:	// SST25WF512		(512 Kbit)	0xAD, 50us, AAI Word
		//case 0x02:	// SST25WF010		(1 Mbit)	0xAD, 50us, AAI Word
		//case 0x03:	// SST25WF020		(2 Mbit)	0xAD, 50us, AAI Word
		//case 0x04:	// SST25WF040		(4 Mbit)	0xAD, 50us, AAI Word
		//case 0x05:	// SST25WF080		(8 Mbit)	0xAD, 14us, AAI Word
		//case 0x41:	// SST25VF016B		(16 Mbit)	0xAD,  7us, AAI Word
		//case 0x4A:	// SST25VF032B		(32 Mbit)	0xAD,  7us, AAI Word
		//case 0x8C:	// SST25VF020B		(2 Mbit)	0xAD,  7us, AAI Word
		//case 0x8D:	// SST25VF040B		(4 Mbit)	0xAD,  7us, AAI Word
		//case 0x8E:	// SST25VF080B		(8 Mbit)	0xAD,  7us, AAI Word				
		// Assume AAI Word programming is supported for the above commented 
		// devices and unknown devices.
		default:	
			deviceCaps.bits.bWriteWordStream = 1;
	}

	// Clear any pre-existing AAI write mode
	// This may occur if the MCU is reset during a write, but the Flash is
	// not tied to the same hardware reset.
	sst25_send_byte_command(WRDI);

	// Execute Enable-Write-Status-Register (EWSR) instruction
	sst25_send_byte_command(EWSR);

	// Clear Write-Protect on all memory locations
	ENABLE_CS_FLASH;
	spi_transfer(WRSR);
	spi_transfer(0x00); // Clear all block protect bits
	DISABLE_CS_FLASH;
}


/*****************************************************************************
  Function:
	void sst25_read_array(uint32_t u32address, uint8_t *pu8data, uint16_t u16len)

  Description:
	Reads an array of bytes from the SPI Flash module.

  Precondition:
	sst25_flash_init has been called, and the chip is not busy (should be
	handled elsewhere automatically.)

  Parameters:
	dwAddress - Address from which to read
	vData - Where to store data that has been read
	wLength - Length of data to read

  Returns:
	None
  ***************************************************************************/
void sst25_read_array(uint32_t u32address, uint8_t *pu8data, uint16_t u16len)
					 //(uint32_t dwAddress, uint8_t *vData, uint16_t wLength)
{
	// Ignore operations when the destination is NULL or nothing to read
	if(pu8data == NULL || u16len == 0) 
	{
		uart_send_string("error");
		uart_newline();
		return;
	}

	ENABLE_CS_FLASH;

	#if 0//SST25_LOG_ACTIV
	uart_send_string("Address split into bytes: ");
	uart_newline();
	uart_send_char(((uint8_t*)&u32address)[2]);
	uart_newline();
	uart_send_char(((uint8_t*)&u32address)[1]);
	uart_newline();
	uart_send_char(((uint8_t*)&u32address)[0]);
	uart_newline();
	#endif  //SST25_LOG_ACTIV

	// Issue READ command with address
	spi_transfer(READ);
	spi_transfer(((uint8_t*)&u32address)[2]);
	spi_transfer(((uint8_t*)&u32address)[1]);
	spi_transfer(((uint8_t*)&u32address)[0]);

	// Read data
	while(u16len--)
	{
		*pu8data++ = spi_transfer(0xFF);
// 		uart_send_string("reading....");
// 		uart_send_char(*pu8data);
// 		uart_newline();
	}
	
	
	
	DISABLE_CS_FLASH;
}

/*****************************************************************************
  Function:
	void sst25_begin_write(uint32_t u32Addr)

  Summary:
	Prepares the SPI Flash module for writing.

  Description:
	Prepares the SPI Flash module for writing.  Subsequent calls to
	sst25_write or sst25_flash_write_array will begin at this location and
	continue sequentially.

  Precondition:
	sst25_flash_init has been called.

  Parameters:
	u32Addr - Address where the writing will begin

  Returns:
	None

  Remarks:
	Flash parts have large sector sizes, and can only erase entire sectors
	at once.  The SST parts for which this library was written have sectors
	that are 4kB in size.  Your application must ensure that writes begin on
	a sector boundary so that the sst25_flash_write functions will erase the
	sector before attempting to write.  Entire sectors need not be written
	at once, so applications can begin writing to the front of a sector,
	perform other tasks, then later call sst25_begin_write and point to an
	address in this sector that has not yet been programmed.  However, care
	must taken to ensure that writes are not attempted on addresses that are
	not in the erased state.  The chip will provide no indication that the
	write has failed, and will silently ignore the command.
  ***************************************************************************/
void sst25_begin_write(uint32_t u32Addr)
{
	g_u32WriteAddr = u32Addr;
}

/*****************************************************************************
  Function:
	void sst25_write(uint8_t u8data)

  Summary:
	Writes a byte to the SPI Flash part.

  Description:
	This function writes a byte to the SPI Flash part.  If the current
	address pointer indicates the beginning of a 4kB sector, the entire
	sector will first be erased to allow writes to proceed.  If the current
	address pointer indicates elsewhere, it will be assumed that the sector
	has already been erased.  If this is not true, the chip will silently
	ignore the write command.

  Precondition:
	sst25_flash_init and sst25_begin_write have been called, and the current
	address is either the front of a 4kB sector or has already been erased.

  Parameters:
	u8data - The byte to write to the next memory location.

  Returns:
	None

  Remarks:
	See Remarks in sst25_begin_write for important information about Flash
	memory parts.
  ***************************************************************************/
void sst25_write(uint8_t u8data)
{
	// If address is a boundary, erase a sector first
	if((g_u32WriteAddr & SPI_FLASH_SECTOR_MASK) == 0u)
	{
		sst25_erase_sector(g_u32WriteAddr);
	}

	sst25_send_byte_command(WREN);
	
	#if SST25_LOG_ACTIV
	uart_send_string("writing started...");
	uart_newline();
	#endif //SST25_LOG_ACTIV
	
	ENABLE_CS_FLASH;
	spi_transfer(WRITE); // Issue WRITE command with address
	spi_transfer(((uint8_t*)&g_u32WriteAddr)[2]);
	spi_transfer(((uint8_t*)&g_u32WriteAddr)[1]);
	spi_transfer(((uint8_t*)&g_u32WriteAddr)[0]);
	spi_transfer(u8data);  // Write the byte
	g_u32WriteAddr++;  // Move to next address
	DISABLE_CS_FLASH;
	
	_WaitWhileBusy();
	
	#if SST25_LOG_ACTIV
	uart_send_string("...writing finished");
	uart_newline();
	#endif //SST25_LOG_ACTIV
}

/*****************************************************************************
  Function:
	void sst25__write_array(uint8_t* u8data, uint16_t u16len)

  Summary:
	Writes an array of bytes to the SPI Flash part.

  Description:
	This function writes an array of bytes to the SPI Flash part.  When the
	address pointer crosses a sector boundary (and has more data to write),
	the next sector will automatically be erased.  If the current address
	pointer indicates an address that is not a sector boundary and is not
	already erased, the chip will silently ignore the write command until the
	next sector boundary is crossed.

  Precondition:
	sst25_flash_init and SPIFlashBeginWrite have been called, and the current
	address is either the front of a sector or has already been erased.

  Parameters:
	u8data - The array to write to the next memory location
	u16len - The length of the data to be written

  Returns:
	None

  Remarks:
	See Remarks in SPIFlashBeginWrite for important information about Flash
	memory parts.
  ***************************************************************************/
void sst25_write_array(uint8_t* u8data, uint16_t u16len)
{
	volatile uint8_t Dummy;
	bool isStarted;
	uint8_t vOpcode;
	uint8_t i;

	// Do nothing if no data to process
	if(u16len == 0u)
		return;

	// If starting at an odd address, write a single byte
	if((g_u32WriteAddr & 0x01) && u16len)
	{
		sst25_write(*u8data);
		u8data++;
		u16len--;
	}

	// Assume we are using AAI Word program mode unless changed later
	vOpcode = WRITE_WORD_STREAM;	

	isStarted = false;

	// Loop over all remaining WORDs
	while(u16len > 1)
	{
		// Don't do anything until chip is ready
		_WaitWhileBusy();

		// If address is a sector boundary
		if((g_u32WriteAddr & SPI_FLASH_SECTOR_MASK) == 0)
			sst25_erase_sector(g_u32WriteAddr);

		// If not yet started, initiate AAI mode
		if(!isStarted)
		{
			// Enable writing
			sst25_send_byte_command(WREN);

			// Select appropriate programming opcode.  The WRITE_WORD_STREAM 
			// mode is the default if neither of these flags are set.
			if(deviceCaps.bits.bWriteByteStream)
				vOpcode = WRITE_BYTE_STREAM;
			else if(deviceCaps.bits.bPageProgram)
			{
				// Note: Writing one byte at a time is extremely slow (ex: ~667 
				// bytes/second write speed on SST SST25VF064C).  You can 
				// improve this by over a couple of orders of magnitude by 
				// writing a function to write full pages of up to 256 bytes at 
				// a time.  This is implemented this way only because I don't 
				// have an SST25VF064C handy to test with right now. -HS
				while(u16len--)
					sst25_write(*u8data++);
				return;
			}

			// Activate the chip select
			ENABLE_CS_FLASH;

			// Issue WRITE_xxx_STREAM command with address
			spi_transfer(vOpcode);
			//Dummy = spi_transfer(Dummy);

			spi_transfer(((uint8_t*)&g_u32WriteAddr)[2]);
			//Dummy = spi_transfer(Dummy);

			spi_transfer(((uint8_t*)&g_u32WriteAddr)[1]);
			//Dummy = spi_transfer(Dummy);

			spi_transfer(((uint8_t*)&g_u32WriteAddr)[0]);
			//Dummy = spi_transfer(Dummy);

			isStarted = true;
		}
		// Otherwise, just write the AAI command again
		else
		{
			// Assert the chip select pin
			ENABLE_CS_FLASH;

			// Issue the WRITE_STREAM command for continuation
			spi_transfer(vOpcode);
			Dummy = spi_transfer(Dummy);
		}

		// Write a byte or two
		for(i = 0; i <= deviceCaps.bits.bWriteWordStream; i++)
		{
			spi_transfer(*u8data++);		
			g_u32WriteAddr++;
			u16len--;
			Dummy = spi_transfer(Dummy);
		}

		// Release the chip select to begin the write
		DISABLE_CS_FLASH;

		// If a boundary was reached, end the write
		if((g_u32WriteAddr & SPI_FLASH_SECTOR_MASK) == 0)
		{
			_WaitWhileBusy();
			sst25_send_byte_command(WRDI);
			isStarted = false;
		}
	}

	// Wait for write to complete, then exit AAI mode
	_WaitWhileBusy();
	sst25_send_byte_command(WRDI);

	// If a byte remains, write the odd address
	if(u16len)
		sst25_write(*u8data);
}


/*****************************************************************************
  Function:
	void sst25_erase_sector(uint32_t u32address)

  Summary:
	Erases a sector.

  Description:
	This function erases a sector in the Flash part.  It is called
	internally by the sst25_flash_write functions whenever a write is attempted
	on the first byte in a sector.

  Precondition:
	sst25_flash_init has been called.

  Parameters:
	u32address - The address of the sector to be erased.

  Returns:
	None

  Remarks:
	See Remarks in SPIFlashBeginWrite for important information about Flash
	memory parts.
  ***************************************************************************/
void sst25_erase_sector(uint32_t u32address)
{
	//volatile uint8_t Dummy;

	// Enable writing
	sst25_send_byte_command(WREN);

	// Activate the chip select
	ENABLE_CS_FLASH;

	// Issue ERASE command with address
	spi_transfer(ERASE_4K);
	//Dummy = spi_transfer(Dummy);

	spi_transfer(((uint8_t*)&u32address)[2]);
	//Dummy = spi_transfer(Dummy);

	spi_transfer(((uint8_t*)&u32address)[1]);
	//Dummy = spi_transfer(Dummy);

	spi_transfer(((uint8_t*)&u32address)[0]);
	//Dummy = spi_transfer(Dummy);

	// Deactivate chip select to perform the erase
	DISABLE_CS_FLASH;

	// Wait for erase to complete
	_WaitWhileBusy();
}


/*****************************************************************************
  Function:
	void sst25_send_byte_command(uint8_t cmd)

  Summary:
	Sends a single-byte command to the SPI Flash part.

  Description:
	This function sends a single-byte command to the SPI Flash part.  It is
	used for commands such as WREN, WRDI, and EWSR that must have the chip
	select activated, then deactivated immediately after the command is
	transmitted.

  Precondition:
	sst25_flash_init has been called.

  Parameters:
	cmd - The single-byte command code to send

  Returns:
	None
  ***************************************************************************/
void sst25_send_byte_command(uint8_t cmd)
{
	uint8_t u8SerialData  = cmd;
	ENABLE_CS_FLASH;
	spi_transfer(u8SerialData); // Send instruction, ignore the return
	DISABLE_CS_FLASH;
}


/*****************************************************************************
  Function:
	static void _WaitWhileBusy(void)

  Summary:
	Waits for the SPI Flash part to indicate it is idle.

  Description:
	This function waits for the SPI Flash part to indicate it is idle.  It is
	used in the programming functions to wait for operations to complete.

  Precondition:
	sst25_flash_init has been called.

  Parameters:
	None

  Returns:
	None
  ***************************************************************************/
static void _WaitWhileBusy(void)
{
	volatile uint8_t Dummy;

	ENABLE_CS_FLASH;
	// Send Read Status Register instruction
	spi_transfer(RDSR);
	do  // Poll the BUSY bit
	{
		Dummy = spi_transfer(0x00);
		//Dummy = spi_transfer(Dummy);
	} while(Dummy & BUSY);
	DISABLE_CS_FLASH;
}

/*****************************************************************************
  Function:
	static void _GetStatus()

  Summary:
	Reads the status register of the part.

  Description:
	This function reads the status register of the part.  It was written
	for debugging purposes, and is not needed for normal operation.  Place
	a breakpoint at the last instruction and check the "status" variable to
	see the result.

  Precondition:
	sst25_flash_init has been called.

  Parameters:
	None

  Returns:
	None
  ***************************************************************************/
//static void _GetStatus()
//{
//	volatile uint8_t Dummy;
//  static uint8_t statuses[16];
//  static uint8_t *status = statuses;
//
//  // Activate chip select
//  SPIFLASH_CS_IO = 0;
//  ClearSPIDoneFlag();
//
//  // Send Read Status Register instruction
//  SPIFLASH_SSPBUF = RDSR;
//  WaitForDataByte();
//  Dummy = SPIFLASH_SSPBUF;
//
//  SPIFLASH_SSPBUF = 0x00;
//  WaitForDataByte();
//  *status = SPIFLASH_SSPBUF;
//  status++;
//
//  // Deactivate chip select
//  SPIFLASH_CS_IO = 1;
//
//  if(status == &statuses[10])
//      statuses[15] = 0;
//}



