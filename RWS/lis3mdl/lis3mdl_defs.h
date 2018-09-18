/*
 * lis3mdl_defs.h
 *
 * Created: 9/18/2018 6:19:49 PM
 *  Author: alexandru.gaal
 */ 


#ifndef LIS3MDL_DEFS_H_
#define LIS3MDL_DEFS_H_

//LIS3MDL operation definitions
#define READ_OP		(1<<7)
#define WRITE_OP	(0<<7)
#define ADDR_INCR	(1<<6)
#define ADDR_CONST	(0<<6)

//LIS3MDL internal register definitions
#define WHO_AM_I	(0x0F)
#define CTRL_REG1	(0x20)
#define CTRL_REG2	(0x21)
#define CTRL_REG3	(0x22)
#define CTRL_REG4	(0x23)
#define CTRL_REG5	(0x24)
#define STATUS_REG	(0x27)
#define OUT_X_L		(0x28)
#define OUT_X_H		(0x29)
#define OUT_Y_L		(0x2A)
#define OUT_Y_H		(0x2B)
#define OUT_Z_L		(0x2C)
#define OUT_Z_H		(0x2D)
#define TEMP_OUT_L	(0x2E)
#define TEMP_OUT_H	(0x2F)
#define INT_CFG		(0x30)
#define INT_SRC		(0x31)
#define INT_THS_L	(0x32)
#define INT_THS_H	(0x33)

#define LIS3MDL_MANUF_ID	(0x3D)

//LIS3MDL register mask definitions
//		Name	Pos		Hint Val.
//CTRL_REG1 0x20 register masks
#define TEMP_EN	(7)		//1
#define OM1		(6)		//0 - Low-power mode
#define OM0		(5)		//0
#define	DO2		(4)		//0 - Output Data Rate 0.625 Hz
#define DO1		(3)		//0
#define DO0		(2)		//0
#define FAST_ODR (1)	//0
#define ST		(0)		//0
//CTRL_REG2 0x21 register masks
#define FS1		(6)		//???
#define FS0		(5)		//???
#define REBOOT	(3)		//0
#define SOFT_RST (2)	//0
//CTRL_REG3 0x22 register masks
#define LP		(0)		//1 - Low-power mode
#define SIM		(2)		//0 - 4 wire interface
#define MD1		(1)		//0 - Single conversion mode
#define MD0		(0)		//1
//CTRL_REG4 0x23 register masks
#define OMZ1	(3)		//0 - Low-power mode
#define	OMZ0	(2)		//0
#define BLE		(1)		//0 - data LSb at lower address
//CTRL_REG5 0x24 register masks
#define FAST_READ (7)	//0
#define BDU		(6)		//10 ???
//STATUS_REG 0x25 register masks
#define ZYXOR	(7)
#define ZOR		(6)
#define YOR		(5)
#define XOR		(4)
#define ZYXDA	(3)
#define ZDA		(2)
#define YDA		(1)
#define XDA		(0)
//INT_CFG 0x30 register masks
#define XIEN	(7)		//0
#define YIEN	(6)		//0
#define ZIEN	(5)		//0
#define IEA		(2)		//0
#define LIR		(1)		//0
#define IEN		(0)		//0
//INT_SRC 0x31 register masks
#define PTH_X	(7)
#define PTH_Y	(6)
#define PTH_Z	(5)
#define NTH_X	(4)
#define NTH_Y	(3)
#define NTH_Z	(2)
#define MROI	(1)
#define INT		(0)

#endif /* LIS3MDL_DEFS_H_ */