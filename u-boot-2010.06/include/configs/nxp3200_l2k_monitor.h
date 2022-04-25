/*
 * (C) Copyright 2011 LeapFrog Enterprises
 * Scott Esters <sesters@leapfrog.com>
 *
 * Configuation settings for the LF2000 L2K board
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*-----------------------------------------------------------------------
 * nexell soc headers
 */
#ifndef	__ASM_STUB_PROCESSOR_H__
#include <platform.h>
#endif

/*-----------------------------------------------------------------------
 * allow to overwrite serial and ethaddr
 */
#define CONFIG_ENV_OVERWRITE

#define CONFIG_SYS_HUSH_PARSER			/* use "hush" command parser	*/
#ifdef 	CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "

#endif

/*-----------------------------------------------------------------------
 * High Level Configuration Options
 * (easy to change)
 */
#undef CONFIG_USE_IRQ		     						/* Not used: not need IRQ/FIQ stuff	*/
#define CONFIG_SYS_HZ	   				1000			/* decrementer freq: 1ms ticks */
#define CONFIG_ARCH_CPU_INIT
#define CONFIG_L2_OFF

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS	   		1				/* nexell soc has 1 bank of dram */

#define CONFIG_CMDLINE_TAG				1				/* enable passing of CMDLINE ATAG  */
#define CONFIG_REVISION_TAG				1				/* enable passing of REVISION ATAG */

/*
 * Size of malloc() pool
 */
#define CONFIG_SYS_MALLOC_LEN			(1*1024*1024)	/* default 512K, for UBIFS 1M */
#define CONFIG_SYS_GBL_DATA_SIZE		128	     		/* size in bytes reserved for initial data */
/*
 * select serial console configuration
 */
#define CONFIG_BAUDRATE		   			CFG_UART_DEBUG_BAUDRATE
#define CONFIG_SYS_BAUDRATE_TABLE	   	{ 9600, 19200, 38400, 57600, 115200 }

/*-----------------------------------------------------------------------
 * Command definition
 */
#define	CONFIG_CMD_BDI				/* board info	*/
#define CONFIG_CMD_BMP
#define	CONFIG_CMD_IMI				/* image info	*/
#define	CONFIG_CMD_BOOTD
#define	CONFIG_CMD_DHCP
#define	CONFIG_CMD_NET
#define	CONFIG_CMD_MEMORY
#define	CONFIG_CMD_PING
#define	CONFIG_CMD_RUN				/* run commands in an environment variable	*/
#define CONFIG_CMDLINE_EDITING		/* add command line history	*/

/*-----------------------------------------------------------------------
 * This must be included AFTER the definition of CONFIG_COMMANDS (if any)
 */
#include <config_cmd_default.h>

/* refer to common/env_common.c	*/
#define CONFIG_BOOTDELAY	   	1

#define CONFIG_ZERO_BOOTDELAY_CHECK	/* check for keypress on bootdelay==0 */
/* When bootdelay is zero, if you want to prevent autoboot you must start
 * pressing a console key as soon as you press the power button.
 */

#define CONFIG_ETHADDR		   			12:24:45:4a:59:24:56
#define CONFIG_NETMASK		   			255.255.255.0
#define CONFIG_IPADDR					192.168.1.151
#define CONFIG_SERVERIP					192.168.1.31
#define CONFIG_GATEWAYIP				192.168.1.254
#define CONFIG_BOOTFILE					"m2kmonitor"  		/* File to load	*/
#define CONFIG_BOOTCOMMAND				"mmc init; mmc rescan 0; fatload mmc 0:2 82000000 m2kmonitor; go 82000000"

/* load from NAND or SD */
#define CONFIG_NOR_BOOTCOMMAND				"nboot 82000000 0 02000000; go 82000000"
#define CONFIG_SD_BOOTCOMMAND				"mmc init; mmc rescan 0; fatload mmc 0:2 82000000 m2kmonitor; go 82000000"

#define CONFIG_BOOTCOMMAND				CONFIG_NOR_BOOTCOMMAND

/*-----------------------------------------------------------------------
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_PROMPT				"nxp3200_l2k_monitor# "					/* Monitor Command Prompt   */
#define CONFIG_SYS_LONGHELP				       									/* undef to save memory	   */
#define CONFIG_SYS_CBSIZE		   		256		   								/* Console I/O Buffer Size  */
#define CONFIG_SYS_PBSIZE		   		(CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16) /* Print Buffer Size */
#define CONFIG_SYS_MAXARGS			   	16		       							/* max number of command args   */
#define CONFIG_SYS_BARGSIZE			   	CONFIG_SYS_CBSIZE	       				/* Boot Argument Buffer Size    */

#define CONFIG_SYS_LOAD_ADDR			CFG_KERNEL_TEXT_BASE					/* default kernel load address */
#define CONFIG_SYS_MEMTEST_START		CFG_MEM_PHY_SYSTEM_BASE					/* memtest works on */
#define CONFIG_SYS_MEMTEST_END			CFG_MEM_PHY_SYSTEM_SIZE
#define CONFIG_BOARD_REVISION_DEFAULT	0x205			/* default Board Revision L2K         */

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE		   		(256*1024)	  /* regular stack: 256K */

#if (CONFIG_STACKSIZE > ((TEXT_BASE & 0x0FFFFFFF) - CONFIG_SYS_MALLOC_LEN))
#error	"There is no enough space for stack ...."
#endif

/*-----------------------------------------------------------------------
 * Ethernet drivers (DM9000 / W5300)
 */
#if (1)
/* DM9000 */
//#define CONFIG_DRIVER_DM9000			1
//#define CONFIG_DM9000_BASE	   			CFG_EXT_PHY_BASEADDR_ETHER	/* DM9000: 0x10000000(CS4) */
//#define DM9000_IO	   					CONFIG_DM9000_BASE
//#define DM9000_DATA	   					(CONFIG_DM9000_BASE + 0x4)
//#define CONFIG_DM9000_DEBUG
#else
/* W5300 */
#define CONFIG_DRIVER_W5300				1
#define CONFIG_DRIVER_DEBUG				0							/* or 0 */
#define W5300_BASE						CFG_EXT_PHY_BASEADDR_ETHER	/* W5300: 0x0C000000(CS3) */
#define W5300_MAX_CHANNEL				2
#define W5300_TMSR_ARRAY				{32,32}
#define W5300_RMSR_ARRAY				{32,32}
#endif

#define CONFIG_NET_RETRY_COUNT			100
#define CONFIG_NET_MULTI				/* needed for eth.c (u-boot-09.11)*/

/*-----------------------------------------------------------------------
 * NAND FLASH
 */
#define	CONFIG_SYS_NO_FLASH
#define	CONFIG_CMD_NAND
#if !defined(CONFIG_CMD_NAND)

#define	CONFIG_SYS_NO_FLASH
#undef  CONFIG_CMD_IMLS					/* list all images found in flash	*/

#define CONFIG_ENV_SIZE					0x1000
#define CONFIG_ENV_IS_NOWHERE

#else	/* CONFIG_CMD_NAND */

#define CONFIG_SYS_64BIT_VSPRINTF		/* needed for nand_util.c (u-boot-09.11)*/

// ECC Correction seems to fail when the ATAP NAND is probed
//#define CONFIG_SYS_MAX_NAND_DEVICE		2
//#define CONFIG_SYS_NAND_BASE_LIST		{ CONFIG_SYS_NAND_BASE, CONFIG_SYS_NAND_BASE }

#define CONFIG_SYS_MAX_NAND_DEVICE		1
#define CONFIG_SYS_NAND_MAX_CHIPS		1

#define	CONFIG_SYS_NAND_BASE			PHY_BASEADDR_NAND_STATIC	/* Nand controller base	*/
#define CONFIG_SYS_NAND_BASE_LIST		{ CONFIG_SYS_NAND_BASE }

//#define CONFIG_MTD_NAND_VERIFY_WRITE								/* set when software ecc mode */
#define CONFIG_SYS_NAND_HW_ECC
//#define CONFIG_CMD_NAND_MLC

#ifdef  CONFIG_CMD_NAND_MLC
#define	CONFIG_SYS_NAND_HW_ECC
#endif

/* etc configure macro */
#undef  CONFIG_CMD_IMLS											/* list all images found in flash	*/

/*-----------------------------------------------------------------------
 * NAND FLASH and environment organization
 */
//#define DEBUG_ENV
#define CONFIG_ENV_IS_IN_NAND									/* default: CONFIG_ENV_IS_NOWHERE */
#define	CONFIG_ENV_OFFSET				2*1024*1024				/* 0x00200000 = erase block boundry */
#define CONFIG_ENV_SIZE           			16*1024				/* 1 block size */
#define CONFIG_ENV_RANGE				16*1024*1024 			/* avoid bad block */

#endif	/* CONFIG_CMD_NAND */

/*-----------------------------------------------------------------------
 * Board specification configure
 */
#define CONFIG_MACH_NXP3200_L2K_DTK

// for other nand write function
//#define	CFG_NAND_WRITE_YAFFS2
#define	CFG_NAND_WRITE_BAD
#define	NAND_WRITE_BAD					(0x01)
#define	NAND_WRITE_YAFFS2				(0x02)

/*-----------------------------------------------------------------------
 * Wakeup functions
 */
#define	CONFIG_PM_WAKEUP
//#define	CONFIG_PM_WAKEUP_ASM

/*-----------------------------------------------------------------------
 * LCD config
 * support only 8/16 bpp LCD
 */
#if 1	// DEBUG_LCD
#define	CONFIG_LCD
#define	CONFIG_LCD_LOGO
#define	CONFIG_LCD_INFO
#define	CONFIG_LCD_INFO_BELOW_LOGO

#define LCD_BPP							LCD_COLOR32	/* BMP file's depth */
#define CONFIG_BMP_32BPP
#define CONFIG_WHITE_ON_BLACK			1
#define CONFIG_SPLASH_SCREEN			1
#define CONFIG_SYS_CONSOLE_IS_IN_ENV	1			/* set stdout to serial port */
#endif

/*-----------------------------------------------------------------------
 * UBI
 */
// #define	CONFIG_CMD_UBIFS
#ifdef CONFIG_CMD_UBIFS

#define CONFIG_RBTREE
#define	CONFIG_CMD_UBI
#define	CONFIG_LZO

#ifdef CONFIG_CMD_UBI
#if (1*1024*1024 > CONFIG_SYS_MALLOC_LEN)
	#error "To UBI, heap length must be more than 1MB..."
#endif
#endif

//#define	CONFIG_UBIFS_FS_DEBUG
#ifdef  CONFIG_UBIFS_FS_DEBUG
#define	CONFIG_UBIFS_FS_DEBUG_MSG_LVL	1	/* For ubifs debug message = 0 ~ 3 */
#endif

#endif // CONFIG_CMD_UBIFS

/*-----------------------------------------------------------------------
 * MTD
 */
#if defined(CONFIG_CMD_UBIFS)
#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS
#define CONFIG_CMD_MTDPARTS

// align with block size,
// use: size@offset(name)
//
#define PART_BOOT			"512k(u-boot)ro,"
#define PART_ENV			"512k(env),"
#define PART_KERNEL			"8m(kernel),"
#define PART_FS				"32m(fs),"
#define PART_SPACE			"-(space)"

#define MTDIDS_DEFAULT		"nand0=nx_nand.0"
#define MTDPARTS_DEFAULT	"mtdparts=nx_nand.0:"	\
							PART_BOOT			\
							PART_ENV			\
							PART_KERNEL			\
							PART_FS				\
							PART_SPACE

//#define CONFIG_MTD_DEBUG
#ifdef  CONFIG_MTD_DEBUG
#define CONFIG_MTD_DEBUG_VERBOSE		3	/* For nand debug message = 0 ~ 3 */
#endif

#endif

/*-----------------------------------------------------------------------
 * Extra env
 */
#ifndef MTDPARTS_DEFAULT
#define	MTDPARTS_DEFAULT
#endif

#define CONFIG_EXTRA_ENV_SETTINGS	"norbootcmd="CONFIG_NOR_BOOTCOMMAND"\0"\
					"sdbootcmd="CONFIG_SD_BOOTCOMMAND"\0"  \
					MTDPARTS_DEFAULT"\0"

/*-----------------------------------------------------------------------
 * SDMMC
 */
#define CONFIG_CMD_MMC
#define CONFIG_MMC
#define	CONFIG_GENERIC_MMC

#if defined(CONFIG_GENERIC_MMC)
#define	CONFIG_MISC_INIT_R
#endif

/*-----------------------------------------------------------------------
 * FAT
 */
#ifdef 	CONFIG_GENERIC_MMC
#define	CONFIG_DOS_PARTITION
#define	CONFIG_CMD_FAT
#endif

/*-----------------------------------------------------------------------
 * USB_OTG
 */
#define CONFIG_USB_DEVICE
#define CONFIG_USB_LF2000_OTGD
#define CONFIG_USB_GADGET_LF2000_OTGD 
#define CONFIG_USB_GADGET_DUALSPEED
	/* 4nov11 added #define CONFIG_USB_GADGET_DUALSPEED, hoping to
	 *	  cause the usb_function's hs_descriptors member to be init'd
	 */
#define CONFIG_USB_MASS_STORAGE
#define CONFIG_ARCH_LF2000 
#define CONFIG_USB_GADGET_VBUS_DRAW 0

/* disabled 3nov11; re-enabled 9nov11; then disabled again * /
#define CONFIG_USE_IRQ 
#define CONFIG_STACKSIZE_IRQ	(4*1024)
#define CONFIG_STACKSIZE_FIQ	(4*1024)
/ * */

/*-----------------------------------------------------------------------
 * Debug message
 */
#define	CONFIG_DISPLAY_CPUINFO			/* print_cpuinfo */
// #define CONFIG_UART_PRE_DEBUG			/* for previous boot message, before board_init */
// #define DEBUG							/* u-boot debug macro, nand, ethernet,... */
// #define CONFIG_PROTO_FUNC_DEBUG		/* nexell prototype debug mode */

#endif /* __CONFIG_H */

