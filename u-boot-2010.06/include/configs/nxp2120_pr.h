/*
 * (C) Copyright 2010 Nexell Co.,
 * jung hyun kim<jhkim@nexell.co.kr>
 *
 * Configuation settings for the Nexell board.
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

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS	   		1				/* nexell soc has 1 bank of dram */

/*
 * Size of malloc() pool
 */
#define CONFIG_SYS_MALLOC_LEN			(512*1024)
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
#define	CONFIG_CMD_IMI				/* image info	*/
#define	CONFIG_CMD_CACHE			/* Cache support: switch on or off, get status	*/
#define	CONFIG_CMD_BOOTD
#define	CONFIG_CMD_DHCP
#define	CONFIG_CMD_LOADB			/* load binary file over serial line	*/
#define	CONFIG_CMD_LOADS			/* load S-Record file over serial line	*/
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
#define CONFIG_BOOTDELAY	   			10
#define CONFIG_ETHADDR		   			12:24:45:4a:59:24:56
#define CONFIG_NETMASK		   			255.255.255.0
#define CONFIG_IPADDR					192.168.0.153
#define CONFIG_SERVERIP					192.168.0.13
#define CONFIG_GATEWAYIP				192.168.0.254
#define CONFIG_BOOTFILE					"uImage"  		/* File to load	*/
#define CONFIG_BOOTCOMMAND				"tftp 0x80200000 uImage; tftp 0x82000000 ramdisk.gz; bootm 0x80200000"

/*-----------------------------------------------------------------------
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_PROMPT				"nxp2120# "     							/* Monitor Command Prompt   */
#define CONFIG_SYS_LONGHELP				       									/* undef to save memory	   */
#define CONFIG_SYS_CBSIZE		   		256		   								/* Console I/O Buffer Size  */
#define CONFIG_SYS_PBSIZE		   		(CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16) /* Print Buffer Size */
#define CONFIG_SYS_MAXARGS			   	16		       							/* max number of command args   */
#define CONFIG_SYS_BARGSIZE			   	CONFIG_SYS_CBSIZE	       				/* Boot Argument Buffer Size    */

#define CONFIG_SYS_LOAD_ADDR			CFG_KERNEL_TEXT_BASE				/* default kernel load address */
#define CONFIG_SYS_MEMTEST_START		CFG_MEM_PHY_SYSTEM_BASE			/* memtest works on */
#define CONFIG_SYS_MEMTEST_END			CFG_MEM_PHY_SYSTEM_SIZE

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
#define CONFIG_DRIVER_DM9000			1
#define CONFIG_DM9000_BASE	   			CFG_EXT_PHY_BASEADDR_ETHER	/* DM9000: 0x10000000(CS4) */
#define DM9000_IO	   					CONFIG_DM9000_BASE
#define DM9000_DATA	   					(CONFIG_DM9000_BASE + 0x4)
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
//#define	CONFIG_CMD_NAND
#if !defined(CONFIG_CMD_NAND)

#define	CONFIG_SYS_NO_FLASH
#undef  CONFIG_CMD_IMLS					/* list all images found in flash	*/

#define CONFIG_ENV_SIZE					0x1000
#define CONFIG_ENV_IS_NOWHERE

#else	/* CONFIG_CMD_NAND */

#define CONFIG_SYS_64BIT_VSPRINTF		/* needed for nand_util.c (u-boot-09.11)*/
#define CONFIG_SYS_MAX_NAND_DEVICE		1
#define	CONFIG_SYS_NAND_BASE			PHY_BASEADDR_NAND		/* Nand conrtoller base	*/

/* etc configure macro */
#undef  CONFIG_CMD_IMLS												/* list all images found in flash	*/
//#define CONFIG_MTD_DEBUG
//#define CONFIG_MTD_DEBUG_VERBOSE		3							/* For nand debug message	*/

/*-----------------------------------------------------------------------
 * NAND FLASH and environment organization
 */
//#define DEBUG_ENV
#define CONFIG_ENV_IS_IN_NAND			/* not support CONFIG_ENV_IS_IN_FLASH in ST Micro, default CONFIG_ENV_IS_NOWHERE */
#define	CONFIG_ENV_OFFSET				512*1024	/* 0x00080000 */
#define CONFIG_ENV_SIZE           		CFG_NAND_BLOCK_SIZE

#endif	/* CONFIG_CMD_NAND */

/*-----------------------------------------------------------------------
 * Board specification configure
 */
#define CONFIG_MACH_NXP2120_PB_PR

// for other nand write function
#define	CFG_NAND_WRITE_YAFFS2
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
#if 0

#define	CONFIG_LCD
#define	CONFIG_LCD_LOGO
#define	CONFIG_LCD_INFO
#define	CONFIG_LCD_INFO_BELOW_LOGO

#define LCD_BPP							LCD_COLOR8	/* BMP file's depth */
#define CONFIG_WHITE_ON_BLACK			1
#define CONFIG_SPLASH_SCREEN			1
#define CONFIG_SYS_CONSOLE_IS_IN_ENV	1			/* set stdout to serial port */

#endif

/*-----------------------------------------------------------------------
 * Debug message
 */
#define	CONFIG_DISPLAY_CPUINFO			/* print_cpuinfo */
//#define CONFIG_UART_PRE_DEBUG			/* for previous boot message, before board_init */
//#define DEBUG							/* u-boot debug macro, nand, ethernet,... */
//#define CONFIG_PROTO_FUNC_DEBUG		/* nexell prototype debug mode */

#endif /* __CONFIG_H */

