/*
 *  armboot - Startup Code for ARMV7 CPU-core
 *
 * Copyright (c) 2010	Nexell
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

#include <config.h>
#include <version.h>



/*
 *************************************************************************
 *
 * low level init (cache clear)
 *
 *************************************************************************
 */

/*
 * define setup registers
 */

.equ	SYSRSTCOFIG,	0xC001E07C	@ yes, spelled this way in databook

.equ	MEMCFG,		0xC0029000
.equ	MEMTIME0,	0xC0029004
.equ	MEMCTRL,	0xC0029008
.equ	MEMTIME1,	0xC0029010

.equ	FASTCH0,	0xC0029020
.equ	FASTCH1,	0xC0029024
.equ	FASTCH2,	0xC0029028

.equ	SLOWCH0,	0xC002902C
.equ	SLOWCH1,	0xC0029030
.equ	SLOWCH2,	0xC0029034

.equ	PHYDELAYCTRL,	0xC0029094
.equ	PHYDLLCTRL0,	0xC0029098
.equ	PHYMODE,	0xC002909C
.equ	PHYDLLCTRL1,	0xC00290A0
.equ	PHYDLLFORCE,	0xC00290A4
.equ	PHYZQCTRL,	0xC00290AC
.equ	PHYZQFORCE,	0xC00290B0
.equ	PHYTERMCTRL,	0xC00290B4
.equ	PHYUPDATE,	0xC00290B8
.equ 	CLKMODEREG0,	0xC001E000
.equ 	CLKMODEREG1,	0xC001E004
.equ 	PLLSETREG0, 	0xC001E008
.equ 	PLLSETREG1, 	0xC001E00C
.equ 	PWRMODE,	0xC001E068
.equ 	CHGPLL,		(1 << 15)


/*
 * Configuration bits in SYSRSTCOFIG register
 */

.equ	DCACHE,		(1 << 6)	@ SA15
.equ	ICACHE,		(1 << 7)	@ SA16

.equ	BOOTMODE0,	(1 << 8)	@ SD0
.equ	BOOTMODE1,	(1 << 9)	@ SD1
.equ	BOOTMODE2,	(1 << 10)	@ SD2
.equ	SELCS,		(1 << 11)	@ SD3
.equ	NANDTYPE0,	(1 << 12)	@ SD4
.equ	NANDTYPE1,	(1 << 13)	@ SD5
.equ	ROMBOOTMODE0,	(1 << 14)	@ SD6
.equ	ROMBOOTMODE1,	(1 << 15)	@ SD7
.equ	ROMBOOTMODE2,	(1 << 16)	@ SD8
.equ	BUSWIDTH,	(1 << 17)	@ SD9
.equ	LATADDR,	(1 << 18)	@ SD10
.equ	DDRTYPE0,	(1 << 19)	@ SD11
.equ	DDRTYPE1,	(1 << 20)	@ SD12
.equ	DDRSIZE0,	(1 << 21)	@ SD13
.equ	DDRSIZE1,	(1 << 22)	@ SD14
.equ	DDRBW,		(1 << 23)	@ SD15
.equ	SELDEX,		(1 << 24)	@ PVD4
.equ	MSBADDR,	(1 << 25)	@ PVD5
.equ	NANDPAGE,	(1 << 26)	@ PVD6
.equ	ADDRMODE,	(1 << 28)	@ PVD3

.equ	BOOTMASK,	(7 << 8)	@ BOOT MODE MASK
.equ	NORBOOT,	(2 << 8)	@ NOR BOOTING

/*
 * MCU-A timings
 */

.equ	MEMCTRL_ALL,		0x00008003

/* 266 MHZ DDR2 */
.equ	MEMCFG_266_MHZ,		0x01890534
.equ	MEMTIME0_266_MHZ,	0x2100B033
.equ	MEMTIME1_266_MHZ,	0x4302040B

/* 275 MHZ DDR2 */
.equ	MEMCFG_275_MHZ,		0x01A90534
.equ	MEMTIME0_275_MHZ,	0x2300C044
.equ	MEMTIME1_275_MHZ,	0x6401042B

/* 333 MHZ DDR2 */
.equ	MEMCFG_333_MHZ,		0x02A90534
.equ	MEMTIME0_333_MHZ,	0x2A00E044
.equ	MEMTIME1_333_MHZ,	0x6401050D

/* 400 MHZ DDR2 */
.equ	MEMCFG_400_MHZ,		0x02C90534
.equ	MEMTIME0_400_MHZ,	0x32011055
.equ	MEMTIME1_400_MHZ,	0x75010612

/*
 * PLL settings
 */

.equ	PLL0_550_MHZ,		0x000D1301

.equ	PLL1_147P5_MHZ,		0x000D2703
.equ	PLL1_266_MHZ,		0x000E1403
.equ	PLL1_275_MHZ,		0x000E2603
.equ	PLL1_333_MHZ,		0x000E9A03
.equ	PLL1_400_MHZ,		0x000CC801

/*
 * register definitions
 */

.SYSRSTCOFIG:
	.word	SYSRSTCOFIG

.DDRBW:
	.word	DDRBW

.BOOTMASK:
	.word	BOOTMASK

.NORBOOT:
	.word	NORBOOT

.MEMCFG:
	.word	MEMCFG

/*
 * memcfg register values, choose based on DDRBW setting
 */

.MEMCFG_8:					@ VTK Board
#if   defined(CONFIG_550_PLL1_147P5_MHZ)
	/* for 550 CPU /275 MHz DDR */
	.word	MEMCFG_275_MHZ			@ 8-bit MEMCFG value
#elif defined(CONFIG_PLL1_266_MHZ)
	/* for 266 MHz DDR */
	.word	MEMCFG_266_MHZ			@ 8-bit MEMCFG value
#elif defined(CONFIG_PLL1_275_MHZ)
	/* for 275 MHz DDR */
	.word	MEMCFG_275_MHZ			@ 8-bit MEMCFG value
#elif defined(CONFIG_PLL1_333_MHZ)
	/* for 333 MHz DDR */
	.word	MEMCFG_333_MHZ			@ 8-bit MEMCFG value
#elif defined(CONFIG_PLL1_400_MHZ)
	/* for 400 MHz DDR */
	.word	MEMCFG_400_MHZ			@ 8-bit MEMCFG value
#else
#error DDR speed not configured, expected 'CONFIG_PLL1_yyy_MHZ' setting
#endif	/* CONFIG_PLL1_yyy_MHZ */

.MEMCFG_16:					@ LF2000 Board
#if   defined(CONFIG_550_PLL1_147P5_MHZ)
	/* for 550 CPU /275 MHz DDR */
	.word	MEMCFG_275_MHZ			@ 16-bit MEMCFG value
#elif defined(CONFIG_PLL1_266_MHZ)
	/* for 266 MHz DDR */
	.word	MEMCFG_266_MHZ			@ 16-bit MEMCFG value
#elif defined(CONFIG_PLL1_275_MHZ)
	/* for 275 MHz DDR */
	.word	MEMCFG_275MHZ			@ 16-bit MEMCFG value
#elif defined(CONFIG_PLL1_333_MHZ)
	/* for 333 MHz DDR */
	.word	MEMCFG_333_MHZ			@ 16-bit MEMCFG value
#elif defined(CONFIG_PLL1_400_MHZ)
	/* for 400 MHz DDR */
	.word	MEMCFG_400MHZ			@ 16-bit MEMCFG value
#else
#error DDR speed not configured, expected 'CONFIG_PLL1_yyy_MHZ' setting
#endif	/* CONFIG_PLL1_yyy_MHZ */

.PWRMODE:
	.word	PWRMODE

/*
 * array of register, value pairs
 *   End of list marked with 0
 */

.REG_VALUE_LIST:
#if   defined(CONFIG_550_PLL1_147P5_MHZ)
	/* for 550/147 MHz (275 MHz DDR) */
	.word	MEMTIME0,     MEMTIME0_275_MHZ	@ MEMTIME0
	.word	MEMCTRL,      MEMCTRL_ALL	@ MEMCTRL
	.word	MEMTIME1,     MEMTIME1_275_MHZ	@ MEMTIME1
#elif defined(CONFIG_PLL1_266_MHZ)
	/* for 266 MHz DDR */
	.word	MEMTIME0,     MEMTIME0_266_MHZ	@ MEMTIME0
	.word	MEMCTRL,      MEMCTRL_ALL	@ MEMCTRL
	.word	MEMTIME1,     MEMTIME1_266_MHZ	@ MEMTIME1
#elif defined(CONFIG_PLL1_275_MHZ)
	/* for 275 MHz DDR */
	.word	MEMTIME0,     MEMTIME0_275_MHZ	@ MEMTIME0
	.word	MEMCTRL,      MEMCTRL_ALL	@ MEMCTRL
	.word	MEMTIME1,     MEMTIME1_275_MHZ	@ MEMTIME1
#elif defined(CONFIG_PLL1_333_MHZ)
	/* for 333 MHz DDR */
	.word	MEMTIME0,     MEMTIME0_333_MHZ	@ MEMTIME0
	.word	MEMCTRL,      MEMCTRL_ALL	@ MEMCTRL
	.word	MEMTIME1,     MEMTIME1_333_MHZ	@ MEMTIME1
#elif defined(CONFIG_PLL1_400_MHZ)
	/* for 400 MHz DDR */
	.word	MEMTIME0,     MEMTIME0_400_MHZ	@ MEMTIME0
	.word	MEMCTRL,      MEMCTRL_ALL	@ MEMCTRL
	.word	MEMTIME1,     MEMTIME1_400_MHZ	@ MEMTIME1
#else
#error DDR speed not configured, expected 'CONFIG_PLL1_yyy_MHZ' setting
#endif	/* CONFIG_PLL1_yyy_MHZ */

	.word	FASTCH0,      0x20184200	@ FASTCH0
	.word	FASTCH1,      0x21080A10	@ FASTCH1
	.word	FASTCH2,      0x00084210	@ FASTCH2

	.word	SLOWCH0,      0x06D1040C	@ SLOWCH0
	.word	SLOWCH1,      0x107338A4	@ SLOWCH1
	.word	SLOWCH2,      0x0005A92F	@ SLOWCH2

	.word	PHYDELAYCTRL, 0x00000000	@ PHYDELAYCTRL
	.word	PHYDLLCTRL0,  0x7000a0a6	@ PHYDLLCTRL0
	.word	PHYMODE,      0x00000000	@ PHYMODE (check)
	.word	PHYDLLCTRL1,  0x00000003	@ PHYDLLCTRL1 (check)
	.word	PHYDLLFORCE,  0x00000014	@ PHYDLLFORCE (check)

	.word	PHYZQCTRL,    0x00000002	@ PHYZQCTRL (check)
	.word	PHYZQFORCE,   0x00000015	@ PHYZQFORCE (check)

# Choose only one PHYTERMCTRL value from the list
# 	.word	PHYTERMCTRL,  0x00000006	@ PHYTERMCTRL ( 300 : 50)
#	.word	PHYTERMCTRL,  0x00000016	@ PHYTERMCTRL ( 150 : 50)
#	.word	PHYTERMCTRL,  0x00000026	@ PHYTERMCTRL ( 100 : 50)
#	.word	PHYTERMCTRL,  0x00000036	@ PHYTERMCTRL (  75 : 50)
#	.word	PHYTERMCTRL,  0x00000046	@ PHYTERMCTRL (  60 : 50)
#	.word	PHYTERMCTRL,  0x00000056	@ PHYTERMCTRL (  50 : 50)
#	.word	PHYTERMCTRL,  0x00000066	@ PHYTERMCTRL (42.8 : 50) 
# 	.word	PHYTERMCTRL,  0x00000076	@ PHYTERMCTRL (37.5 : 50)
#       .word	PHYTERMCTRL,  0x00000034	@ PHYTERMCTRL (  75 : 75)
#       .word   PHYTERMCTRL,  0x00000054        @ PHYTERMCTRL (  50 : 75)
#       .word   PHYTERMCTRL,  0x00000064        @ PHYTERMCTRL (42.8 : 75)
#       .word   PHYTERMCTRL,  0x00000074        @ PHYTERMCTRL (37.5 : 75)
# 	.word	PHYTERMCTRL,  0x00000024	@ PHYTERMCTRL ( 100 : 75)
 	.word	PHYTERMCTRL,  0x00000044	@ PHYTERMCTRL (  60 : 75)

	.word	PHYUPDATE,    0x00000001	@ PHYUPDATE

/*
 * Configure PLL0 / PLL1 speeds and also set CPU and DDR clock sources.
 * Only the PLL1_147_MHZ mode clocks DDR from PLL0.  Future platforms clock DDR
 * from PLL1, allowing the CPU speed to change independent of the DDR speed.
 * PLL0 is set to max of 550 MHz here and may adjust after boosting the CPU
 * core voltage (see lfp100.c).
 */
	.word	CLKMODEREG0,  0x00000300	@ CLKMODEREG0 // CPU PLL0 / 1 / 4 / 2
	.word	PLLSETREG0,   PLL0_550_MHZ	@ PLLSETREG0 @ 550 MHZ

#if   defined(CONFIG_550_PLL1_147P5_MHZ)
	.word	CLKMODEREG1,  0x00001101	@ CLKMODEREG1 // DDR PLL0 / 2 / 2 / 2
#else
	.word	CLKMODEREG1,  0x00001110	@ CLKMODEREG1 // DDR PLL1 / 1 / 2 / 2
#endif

#if   defined(CONFIG_550_PLL1_147P5_MHZ)
	/* for 550 MHz CPU/275 MHz DDR on PLL0 */
	.word	PLLSETREG1,   PLL1_147P5_MHZ	@ PLLSETREG1 @ 147.5 MHZ
#elif defined(CONFIG_PLL1_266_MHZ)
	/* for 266 MHz DDR */
	.word	PLLSETREG1,   PLL1_266_MHZ	@ PLLSETREG1 @ 266 MHZ
#elif defined(CONFIG_PLL1_275_MHZ)
	/* for 275 MHz DDR */
	.word	PLLSETREG1,   PLL1_275_MHZ	@ PLLSETREG1 @ 275 MHZ
#elif defined(CONFIG_PLL1_333_MHZ)
	/* for 333 MHz DDR */
	.word	PLLSETREG1,   PLL1_333_MHZ	@ PLLSETREG1 @ 333 MHZ
#elif defined(CONFIG_PLL1_400_MHZ)
	/* for 400 MHz DDR */
	.word	PLLSETREG1,   PLL1_400_MHZ	@ PLLSETREG1 @ 400 MHZ
#else
#error DDR speed not configured, expected 'CONFIG_PLL1_yyy_MHZ' setting
#endif	/* CONFIG_PLL1_yyy_MHZ */

	.word	PWRMODE,	  CHGPLL
	.word	0				@ marks end of list

.globl	arch_init_crit
arch_init_crit:

	mov	 r1, #0xF0000000
	mov	 r0, #0x80000000		@ CPU0 can access the components
	str	 r0, [r1, #0x40]		@ SCU start address
	mov	 r0, #0xB0000000		@ CPU0 can access the components
	str	 r0, [r1, #0x44]		@ SCU end address

	ldr	r0, .SYSRSTCOFIG		@ get system switch settings
	ldr	r2, [r0]

	ldr	r0, .BOOTMASK			@ mask off boot mode
	and	r1, r0, r2			@ keep boot mode bits
	ldr	r0, .NORBOOT			@ did system NOR boot?
	tst	r0, r1
	bne	check_ddr			@ yes, no NSIH, so setup system
	mov	 pc, lr				@ no, back to caller

check_ddr:
	ldr	r0, .DDRBW			@ 16-bit width DDR switch set?
	tst	r2, r0				@ check switch
	ldreq	r1, .MEMCFG_8			@ 0: use 8-bit DDR
	ldrne	r1, .MEMCFG_16			@ 1: use 16-bit DDR

store_memcfg:
	ldr	r0, .MEMCFG			@ point at MEMCFG register
	str	r1, [r0]			@ save new MEMCFG value

	adr	r0, .REG_VALUE_LIST		@ point at list
reg_load_loop:
	ldr	 r1, [r0]			@ get register
	cmp	 r1, #0				@ at end of list?
	beq	 reg_loop_end			@ yes
	add	 r0, r0, #4			@ point at value
	ldr	 r2, [r0]			@ get value
	add	 r0, r0, #4			@ point at next register
	str	 r2, [r1]			@ place value in this register
	b	 reg_load_loop			@ handle next value

reg_loop_end:
	ldr	 r1, .PWRMODE			@ point to the PWRMODE register
chk_pll_stable:
	ldr	 r0, [r1]
	tst	 r0, #CHGPLL
	bne  chk_pll_stable			@ branch if PWRMODE.CHGPLL is set

	mov	 pc, lr				@ back to caller


.globl	lowlevel_init
lowlevel_init:

	/* Invalidate Data/Unified Caches */
    mrc	 p15, 1, r0, c0, c0, 1   	@ Read CLIDR
    ands r3, r0, #0x7000000
    mov  r3, r3, lsr #23          	@ Total cache levels << 1
    beq  finished
    mov  r10, #0                  	@ R10 holds current cache level << 1

loop1:
	add  r2, r10, r10, lsr #1  		@ R2 holds cache "Set" position
	mov  r1, r0, lsr r2           	@ Bottom 3 bits are the Cache-type for this level
	and  r1, R1, #7              	@ Get those 3 bits alone
	cmp  r1, #2
    blt  skip                  		@ No cache or only instruction cache at this level

    mcr  p15, 2, r10, c0, c0, 0   	@ Write the Cache Size selection register
    mov  r1, #0
    mcr  p15, 0, r1, c7, c5, 4      @ PrefetchFlush to sync the change to the CacheSizeID reg
    mrc  p15, 1, r1, c0, c0, 0      @ Reads current Cache Size ID register
    and  r2, r1, #0x7                @ Extract the line length field
    add  r2, r2, #4                 @ Add 4 for the line length offset (log2 16 bytes)
    ldr  r6, =0x3FF
    ands r6, r6, r1, lsr #3         @ R6 is the max number on the way size (right aligned)
    clz  r5, r6                     @ R5 is the bit position of the way size increment
    ldr  r7, =0x00007FFF
	ands r7, r7, r1, lsr #13        @ R7 is the max number of the index size (right aligned)

loop2:
   	mov  r9, r6                     @ R9 working copy of the max way size (right aligned)

loop3:
   	orr  r11, r10, r9, LSL r5       @ Factor in the Way number and cache number into R11
	orr  r11, r11, r7, LSL r2       @ Factor in the Set number
    mcr  p15, 0, r11, c7, c6, 2     @ Clean and Invalidate by set/way
    subs r9, r9, #1                 @ Decrement the Way number
    bge  loop3
    subs r7, r7, #1                 @ Decrement the Set number
    bge  loop2

skip:
	add  r10, r10, #2               @ increment the cache number
    cmp  r3, r10
    bgt  loop1

finished:
	mov	 pc, lr						@ back to caller

#if 1
	/* This is the assembly language version of */
	/* a routine that's been disabled in        */
	/* u-boot/drivers/usb/gadget/cbf.c          */
	/* The assy language version of the routine */
	/* takes about 16 msec for a 2.2 kernel, and*/
	/* the C version takes about 71 msec.       */
	/* Verification of the u-boot CRC for the   */
	/* same kernel takes about 385 msec.        */
	/* This assembly language version has been  */
	/* copied from emerald-boot's and madrid-   */
	/* boot's startup.S.                        */
.global cbf_checksum
cbf_checksum:
	stmdb		sp!, {r4, r5, r6, r7, r8, r9, lr}    	
 	mov		ip, r1, lsr #2
	mov		r1, r2
 	b	2f

1:	ldmia		r0!, {r2-r9}
	eor		r1, r1, r2
	add		r1, r1, #1
	eor		r1, r1, r3
	add		r1, r1, #1
	eor		r1, r1, r4
	add		r1, r1, #1
	eor		r1, r1, r5
	add		r1, r1, #1
	eor		r1, r1, r6
	add		r1, r1, #1
	eor		r1, r1, r7
	add		r1, r1, #1
	eor		r1, r1, r8
	add		r1, r1, #1
	eor		r1, r1, r9
	add		r1, r1, #1		@ 17 instructions
    	sub		ip, ip, #8	
2:    	cmp		ip, #7	
    	bhi	1b
	b	4f
3:
     	ldr		r3, [r0, #-4]
     	eor		r3, r1, r3
     	add		r1, r3, #1
4:	sub		ip, ip, #1
     	cmn		ip, #1
     	add		r0, r0, #4
     	bne	3b
     	mov		r0, r1
	ldmia	sp!, {r4, r5, r6, r7, r8, r9, pc}

#endif

