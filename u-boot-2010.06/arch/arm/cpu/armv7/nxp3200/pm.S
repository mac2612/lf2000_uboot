/*
 * Copyright (c) Microsoft Corporation.  All rights reserved.
 *
 *
 * Use of this source code is subject to the terms of the Microsoft end-user
 * license agreement (EULA) under which you licensed this SOFTWARE PRODUCT.
 * If you did not accept the terms of the EULA, you are not authorized to use
 * this source code. For a copy of the EULA, please see the LICENSE.RTF on your
 * install media.
 *
 ********************************************************************************
 *
 *  File:  pm.s
 *
 *	CPU power management for Nexell ARM11 SoC.
 *
 ********************************************************************************
 */

#include "cfg_mem.h"

/********************************************************************************
 * 	goto_wakeup_asm
 *  Check memory befor goto wake up routine
 ********************************************************************************/

#define	SLEEP_PHY_BASE 		CFG_SLEEP_DATA_BASE

.globl	goto_wakeup_asm
goto_wakeup_asm:

			push	{r4, lr}					@ save stack

			/* set alive power gating */
			ldr		r0, =0xC0019000				@ power gate address
			ldr		r1, [r0]					@ load power gate value
			orr		r1, r1, #1					@ ( | 0x1)
			str		r1,	[r0]					@ save power gate enable value

			ldr		r0, =SLEEP_PHY_BASE			@ 0x80200000,  set sleep data sum val position

			/* Check scratch register */
			ldr		r1, =0xC0019070
			ldr		r2, [r1]					@ load scratch value
			ldr		r3, =0x50575200				@ check scratch value
			cmp		r2, r3						@ compare

			/* current not sleep mode */
			ldrne	r3, =0x56781234				@ 0x56781234
			strne	r3, [r0]					@ [0x80200000] 0x56781234
			bne		exit_wakeup

			/* check size value */
			ldr		r0, =(SLEEP_PHY_BASE + 4)	@ 0x80200000+4, set sleep data size position
			ldr		r1, [r0]					@ load size value
			cmp		r1, #0						@ if == 0
			ble		fail_size

			cmp		r1, #255					@ 0xff
			movle	r2, #0						@ 0x0
			movle	r1, r2
			ldrle	r0, =(SLEEP_PHY_BASE + 8)	@ data
			ldrle	r7, =(SLEEP_PHY_BASE + 4)	@ size
			ldrle	r6, =(SLEEP_PHY_BASE)		@ sum
			ldrle	r4, [r6]					@ sum
			ldr		r5, [r7]					@ size

			ble		verify_sum

fail_size:
			/* fail check size */
			ldr		r0, =SLEEP_PHY_BASE			@ 0x80200000, set sleep data sum val position
			ldr		r3, =0x12345678
			str		r3, [r0]					@ [0x80200000] 0x12345678
			b		exit_wakeup

			/* verify check sum */
loop:
			ldr		r3, [r0], #4
			eor		r3, r3, #170				@ 0xaa
			eor		r1, r1, r3

verify_sum:
			cmp		r2, r5						@ compare(size) r5 with r2(i)
			add		r2, r2, #1					@ i++ (r2)
			blt		loop

			cmp		r4, r1						@ compare(sum) r4 with r1(xor)

			/*
			 * success verify check sum
			 * goto cpu wakeup routine
			 */
			beq		asm_cpu_wakeup

			/*
			 * fail verify check sum
			 * return boot code
			 */
			ldr		r2, =SLEEP_PHY_BASE			@ set sleep sum position
			ldr		r3, =0x87654321
			str		r3, [r2]

			/*
			 * it's not wakeup routine
			 * return boot code
			 */
exit_wakeup:
			pop		{r4, pc}					@ return


/*******************************************************************************
 * 	Sleep backup data layout
 *******************************************************************************/

#define	SLEEP_DATA_INFO 		(SLEEP_PHY_BASE + 8)

#define	MODE_SVC				(0x13)
#define	NOINT					(0xC0)
#define	R1_VE					(1<<24)
#define	R1_I					(1<<12)
#define	R1_BP					(1<<11)		@ Z bit

#define	WORD_SIZE				(4)
#define	Sleep_Data_Start		(0)
#define	Sleep_WakeAddr			(Sleep_Data_Start)
#define	Sleep_SYSCTL			(Sleep_WakeAddr		+ WORD_SIZE )
#define	Sleep_MMUTTB0			(Sleep_SYSCTL		+ WORD_SIZE )
#define	Sleep_MMUTTB1			(Sleep_MMUTTB0		+ WORD_SIZE )
#define	Sleep_MMUTTBCTL			(Sleep_MMUTTB1		+ WORD_SIZE )
#define	Sleep_MMUDOMAIN			(Sleep_MMUTTBCTL	+ WORD_SIZE )
#define	Sleep_SVC_SP			(Sleep_MMUDOMAIN	+ WORD_SIZE )
#define	Sleep_SVC_SPSR			(Sleep_SVC_SP		+ WORD_SIZE )
#define	Sleep_FIQ_SPSR			(Sleep_SVC_SPSR		+ WORD_SIZE )
#define	Sleep_FIQ_R8			(Sleep_FIQ_SPSR		+ WORD_SIZE )
#define	Sleep_FIQ_R9			(Sleep_FIQ_R8		+ WORD_SIZE )
#define	Sleep_FIQ_R10			(Sleep_FIQ_R9		+ WORD_SIZE )
#define	Sleep_FIQ_R11			(Sleep_FIQ_R10		+ WORD_SIZE )
#define	Sleep_FIQ_R12			(Sleep_FIQ_R11		+ WORD_SIZE )
#define	Sleep_FIQ_SP			(Sleep_FIQ_R12		+ WORD_SIZE )
#define	Sleep_FIQ_LR			(Sleep_FIQ_SP		+ WORD_SIZE )
#define	Sleep_ABT_SPSR			(Sleep_FIQ_LR		+ WORD_SIZE )
#define	Sleep_ABT_SP			(Sleep_ABT_SPSR		+ WORD_SIZE )
#define	Sleep_ABT_LR			(Sleep_ABT_SP		+ WORD_SIZE )
#define	Sleep_IRQ_SPSR			(Sleep_ABT_LR		+ WORD_SIZE )
#define	Sleep_IRQ_SP			(Sleep_IRQ_SPSR		+ WORD_SIZE )
#define	Sleep_IRQ_LR			(Sleep_IRQ_SP		+ WORD_SIZE )
#define	Sleep_UND_SPSR			(Sleep_IRQ_LR		+ WORD_SIZE )
#define	Sleep_UND_SP			(Sleep_UND_SPSR		+ WORD_SIZE )
#define	Sleep_UND_LR			(Sleep_UND_SP		+ WORD_SIZE )
#define	Sleep_SYS_SP			(Sleep_UND_LR		+ WORD_SIZE )
#define	Sleep_SYS_LR			(Sleep_SYS_SP		+ WORD_SIZE )
#define	Sleep_Data_End     		(Sleep_SYS_LR		+ WORD_SIZE )
#define	SLEEPDATA_SIZE		    ((Sleep_Data_End - Sleep_Data_Start) / 4)

@-------------------------------------------------------------------------------
@  Function:  asm_cpu_wakeup
@
@  Restore mmu status before goto cpu power up
@-------------------------------------------------------------------------------

.globl asm_cpu_wakeup
asm_cpu_wakeup:
        @---------------------------------------------------------------
        @	Switch SVC Mode And Disable Irq/Fiq
        @---------------------------------------------------------------
		mov		r0, #MODE_SVC | NOINT			@ SVC mode, no interrupts
		msr     cpsr_c, r0                      @ switch to supervisor mode and fiq/irq disable.

        @---------------------------------------------------------------
		@	Disable MMU
        @---------------------------------------------------------------
		mov		r1, #0
		mcr		p15, 0, r1, c7, c5, 0			@ Invalidate Entire Instruction Cache
		mcr		p15, 0, r1, c7, c14, 0			@ Clean and Invalidate Entire Data Cache

		mrc		p15, 0, r1, c1, c0, 0
		bic		r1, r1, #0x0005					@ Disable MMU and Data Cache
		mcr		p15, 0, r1, c1, c0, 0

        @---------------------------------------------------------------
		@	Enable ICache
        @---------------------------------------------------------------
		mov		r0, #0
		mcr		p15, 0, r0, c7, c7, 0			@ Invalidate Entire I&D Cache

		mrc		p15, 0, r0, c1, c0, 0			@ read CP15 register 1 into r0
		orr		r0, r0, #R1_I					@ enable I Cache
		mcr		p15, 0, r0, c1, c0, 0			@ write cp15 register 1, enable I Cache

        @---------------------------------------------------------------
		@	Peripheral Port Setup
        @---------------------------------------------------------------
		ldr		r0, =0xC0000013					@ Base Addres : 0xC0000000, Size : 256 MB (0x13), 1 GB(0x15)
		mcr		p15, 0, r0, c15, c2, 4

        @---------------------------------------------------------------
		@	Flush TLB, Invalidate ICache, DCache
        @---------------------------------------------------------------
		mov     r0, #0
		mcr     p15, 0, r0, c8, c7, 0           @ flush both TLB
		mcr     p15, 0, r0, c7, c5, 0           @ invalidate instruction cache
		mcr     p15, 0, r0, c7, c6, 0           @ invalidate data cache

        @---------------------------------------------------------------
		@	Disable VIC
        @---------------------------------------------------------------
		mrc		p15, 0, r0, c1, c0, 0
		bic		r0, r0, #R1_VE
		mcr		p15, 0, r0, c1, c0, 0

        @---------------------------------------------------------------
		@	Enable Branch Prediction
        @---------------------------------------------------------------
		mrc		p15, 0, r0, c1, c0, 0
		orr		r0, r0, #R1_BP
		mcr		p15, 0, r0, c1, c0, 0

        @---------------------------------------------------------------
		@ Restore CP15 Register
        @---------------------------------------------------------------
		ldr		r10, =SLEEP_DATA_INFO			@ Sleep mode information data structure
	/*	ldr		r10, [r10]	*/

		ldr		r6,	[r10, #Sleep_MMUDOMAIN]		@ Domain Access Control Register
		ldr		r5,	[r10, #Sleep_MMUTTBCTL]		@ TTB Control Register
		ldr		r4,	[r10, #Sleep_MMUTTB1]		@ TTB Register1
		ldr		r3,	[r10, #Sleep_MMUTTB0]		@ TTB Register0
		ldr		r2,	[r10, #Sleep_SYSCTL]		@ System Control Register
		ldr		r1,	[r10, #Sleep_WakeAddr]		@ Return Address

		mcr		p15, 0, r6, c3, c0, 0			@ Restore Domain Access Control Register
		mcr		p15, 0, r5, c2, c0, 2			@ Restore TTB Control Register
		mcr		p15, 0, r4, c2, c0, 1			@ Restore TTB Register1
		mcr		p15, 0, r3, c2, c0, 0			@ Restore TTB Register0

		mov		r0, #0x0
		mcr		p15, 0, r0, c8, c7, 0	   		@ Invalidate I & D TLB

		mcr		p15, 0, r2, c1, c0, 0			@ Restore System Control Register (MMU Control)

		nop
		nop
		nop
		nop
		nop

        @---------------------------------------------------------------
		@ go to asm_cpu_restore_reg
        @---------------------------------------------------------------
		mov		pc, r1							@ Jump to Virtual Return Address
		b		.
