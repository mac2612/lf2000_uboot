/*
 * include/mfgdata.h 
 *
 * Copyright 2012 LeapFrog Enterprises Inc.
 *
 * Leapfrog Firmware <firmware@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef MFGDATA_H
#define MFGDATA_H

struct MFGDATA {
	int	TSCalData[7];		// 0
	int	BLCaldata;		// 28
	int	ADCCalData[2];		// 32 [0]=constant, [1]=slope_256
	int	AudioGain;		// 40
	u8	SerialNumber[24];	// 44
	int	SNValidFlag;		// 68
	u8	SirusBarcode[32];	// 72
	u8	Locale[8];		// 104
	int	TSPressure[24];		// 112
	u32	SoftBoardId;		// 208
	int	AclBias[3];		// 212
	int	MfgVersion;		// 224
	int	BL2CalData[5];		// 228
	u8	MacAddress[20];		// 248
	u16	IPv6Address[8];		// 268
	int	LCDMType;		// 284
	u8	Sku[11];		// 288
	long	AccelCalData[10];	// 299
	u8	reserved[3753];		// 339
	u32	Checksum;		// 4092
} __attribute__ ((__packed__));

#define LF2000_NOR_FLASH_BASE_LOW	0x00000000
#define LF2000_NOR_FLASH_BASE_HIGH0	0x34000000
#define LF2000_NOR_FLASH_BASE_HIGH1	0x04000000

#define LF2000_NOR_FLASH_SIZE		( 512 * 1024 )
#define LF2000_NOR_BOOT_SIZE		( LF2000_NOR_FLASH_SIZE - 2 * sizeof( struct MFGDATA ) )

#endif /* MFGDATA_H*/

