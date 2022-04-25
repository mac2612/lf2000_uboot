/* cbf.h -- access to Common Boot Format, a simple packing format for binaries.
 *
 * Copyright 2009-2011 LeapFrog Enterprises Inc.
 *
 * Robert Dowling <rdowling@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CBF__H__
#define __CBF__H__

#ifdef U_BOOT
void cbf_init(void);
int cbf_process_chunk(u8 *buf, u32 len, u8 **suggested_buf);
int cbf_get_jump_address(void **jump);
int cbf_get_status(void);
int cbf_get_data_size(void);
u32 cbf_checksum (u8 *buf, u32 len, u32 incoming);

/* including these #defines here for temporary convenience */
#define RAMDISK_SECTOR 		0x200
#define RAMDISK_SECTOR_SIZE 0x200
#define BYTES_PER_SECTOR 	0x200

// These definitions result in a 12MB FAT16 ramdisk
// (The number of clusters dictates whether FAT12 or FAT16 is used.  FAT12
//  supports no more than 4077 clusters.)
//#define SECTORS_PER_DISK	32768 /* New for 16MB; here's the 12MB value: 24576*/
#define SECTORS_PER_DISK	24576
#define RAMDISK_SIZE   		(RAMDISK_SECTOR_SIZE * SECTORS_PER_DISK)

#else
void cbf_init();
int cbf_process_chunk(u8 *buf, u32 len, u8 **suggested_buf);
int cbf_get_jump_address(void **jump);
int cbf_get_status();
int cbf_get_data_size();
u32 cbf_checksum (u8 *buf, u32 len, u32 incoming);
#endif

#endif /* __CBF__H__ */
