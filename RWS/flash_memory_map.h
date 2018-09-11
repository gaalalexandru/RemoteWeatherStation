/*
 * flash_memory_map.h
 *
 * Created: 9/11/2018 4:26:43 PM
 *  Author: alexandru.gaal
 */ 


#ifndef FLASH_MEMORY_MAP_H_
#define FLASH_MEMORY_MAP_H_

#define SECTOR_0_START_ADDR		(0X000000)
#define SECTOR_1_START_ADDR		(0X001000)
#define SECTOR_2_START_ADDR		(0X002000)
#define SECTOR_3_START_ADDR		(0X003000)
#define SECTOR_4_START_ADDR		(0X004000)
#define SECTOR_5_START_ADDR		(0X005000)
#define SECTOR_6_START_ADDR		(0X006000)
#define SECTOR_7_START_ADDR		(0X007000)
#define SECTOR_8_START_ADDR		(0X008000)
#define SECTOR_9_START_ADDR		(0X009000)
#define SECTOR_10_START_ADDR	(0X00A000)
#define SECTOR_11_START_ADDR	(0X00B000)
#define SECTOR_12_START_ADDR	(0X00C000)
#define SECTOR_13_START_ADDR	(0X00D000)
#define SECTOR_14_START_ADDR	(0X00E000)
#define SECTOR_15_START_ADDR	(0X00F000)
#define SECTOR_16_START_ADDR	(0X010000)
#define SECTOR_17_START_ADDR	(0X011000)
#define SECTOR_18_START_ADDR	(0X012000)
#define SECTOR_19_START_ADDR	(0X013000)
#define SECTOR_20_START_ADDR	(0X014000)
#define SECTOR_21_START_ADDR	(0X015000)
#define SECTOR_22_START_ADDR	(0X016000)
#define SECTOR_23_START_ADDR	(0X017000)
#define SECTOR_24_START_ADDR	(0X018000)
#define SECTOR_25_START_ADDR	(0X019000)
#define SECTOR_26_START_ADDR	(0X01A000)
#define SECTOR_27_START_ADDR	(0X01B000)
#define SECTOR_28_START_ADDR	(0X01C000)
#define SECTOR_29_START_ADDR	(0X01D000)
#define SECTOR_30_START_ADDR	(0X01E000)
#define SECTOR_31_START_ADDR	(0X01F000)

#define HOUR_0_ADDR		SECTOR_8_START_ADDR
#define HOUR_1_ADDR		SECTOR_9_START_ADDR
#define HOUR_2_ADDR		SECTOR_10_START_ADDR
#define HOUR_3_ADDR		SECTOR_11_START_ADDR
#define HOUR_4_ADDR		SECTOR_12_START_ADDR
#define HOUR_5_ADDR		SECTOR_13_START_ADDR
#define HOUR_6_ADDR		SECTOR_14_START_ADDR
#define HOUR_7_ADDR		SECTOR_15_START_ADDR
#define HOUR_8_ADDR		SECTOR_16_START_ADDR
#define HOUR_9_ADDR		SECTOR_17_START_ADDR
#define HOUR_10_ADDR	SECTOR_18_START_ADDR
#define HOUR_11_ADDR	SECTOR_19_START_ADDR
#define HOUR_12_ADDR	SECTOR_20_START_ADDR
#define HOUR_13_ADDR	SECTOR_21_START_ADDR
#define HOUR_14_ADDR	SECTOR_22_START_ADDR
#define HOUR_15_ADDR	SECTOR_23_START_ADDR
#define HOUR_16_ADDR	SECTOR_24_START_ADDR
#define HOUR_17_ADDR	SECTOR_25_START_ADDR
#define HOUR_18_ADDR	SECTOR_26_START_ADDR
#define HOUR_19_ADDR	SECTOR_27_START_ADDR
#define HOUR_20_ADDR	SECTOR_28_START_ADDR
#define HOUR_21_ADDR	SECTOR_29_START_ADDR
#define HOUR_22_ADDR	SECTOR_30_START_ADDR
#define HOUR_23_ADDR	SECTOR_31_START_ADDR

#define MINUTE_0_REL_START_ADDR		(0x000000)
#define MINUTE_1_REL_START_ADDR		(0x000040)
#define MINUTE_2_REL_START_ADDR		(0x000080)
#define MINUTE_3_REL_START_ADDR		(0x0000C0)
#define MINUTE_4_REL_START_ADDR		(0x000100)
#define MINUTE_5_REL_START_ADDR		(0x000140)
#define MINUTE_6_REL_START_ADDR		(0x000180)
#define MINUTE_7_REL_START_ADDR		(0x0001C0)
#define MINUTE_8_REL_START_ADDR		(0x000200)
#define MINUTE_9_REL_START_ADDR		(0x000240)
#define MINUTE_10_REL_START_ADDR	(0x000280)
#define MINUTE_11_REL_START_ADDR	(0x0002C0)
#define MINUTE_12_REL_START_ADDR	(0x000300)
#define MINUTE_13_REL_START_ADDR	(0x000340)
#define MINUTE_14_REL_START_ADDR	(0x000380)
#define MINUTE_15_REL_START_ADDR	(0x0003C0)
#define MINUTE_16_REL_START_ADDR	(0x000400)
#define MINUTE_17_REL_START_ADDR	(0x000440)
#define MINUTE_18_REL_START_ADDR	(0x000480)
#define MINUTE_19_REL_START_ADDR	(0x0004C0)
#define MINUTE_20_REL_START_ADDR	(0x000500)
#define MINUTE_21_REL_START_ADDR	(0x000540)
#define MINUTE_22_REL_START_ADDR	(0x000580)
#define MINUTE_23_REL_START_ADDR	(0x0005C0)
#define MINUTE_24_REL_START_ADDR	(0x000600)
#define MINUTE_25_REL_START_ADDR	(0x000640)
#define MINUTE_26_REL_START_ADDR	(0x000680)
#define MINUTE_27_REL_START_ADDR	(0x0006C0)
#define MINUTE_28_REL_START_ADDR	(0x000700)
#define MINUTE_29_REL_START_ADDR	(0x000740)
#define MINUTE_30_REL_START_ADDR	(0x000780)
#define MINUTE_31_REL_START_ADDR	(0x0007C0)
#define MINUTE_32_REL_START_ADDR	(0x000800)
#define MINUTE_33_REL_START_ADDR	(0x000840)
#define MINUTE_34_REL_START_ADDR	(0x000880)
#define MINUTE_35_REL_START_ADDR	(0x0008C0)
#define MINUTE_36_REL_START_ADDR	(0x000900)
#define MINUTE_37_REL_START_ADDR	(0x000940)
#define MINUTE_38_REL_START_ADDR	(0x000980)
#define MINUTE_39_REL_START_ADDR	(0x0009C0)
#define MINUTE_40_REL_START_ADDR	(0x000A00)
#define MINUTE_41_REL_START_ADDR	(0x000A40)
#define MINUTE_42_REL_START_ADDR	(0x000A80)
#define MINUTE_43_REL_START_ADDR	(0x000AC0)
#define MINUTE_44_REL_START_ADDR	(0x000B00)
#define MINUTE_45_REL_START_ADDR	(0x000B40)
#define MINUTE_46_REL_START_ADDR	(0x000B80)
#define MINUTE_47_REL_START_ADDR	(0x000BC0)
#define MINUTE_48_REL_START_ADDR	(0x000C00)
#define MINUTE_49_REL_START_ADDR	(0x000C40)
#define MINUTE_50_REL_START_ADDR	(0x000C80)
#define MINUTE_51_REL_START_ADDR	(0x000CC0)
#define MINUTE_52_REL_START_ADDR	(0x000D00)
#define MINUTE_53_REL_START_ADDR	(0x000D40)
#define MINUTE_54_REL_START_ADDR	(0x000D80)
#define MINUTE_55_REL_START_ADDR	(0x000DC0)
#define MINUTE_56_REL_START_ADDR	(0x000E00)
#define MINUTE_57_REL_START_ADDR	(0x000E40)
#define MINUTE_58_REL_START_ADDR	(0x000E80)
#define MINUTE_59_REL_START_ADDR	(0x000EC0)

#endif /* FLASH_MEMORY_MAP_H_ */