/*
 * (C) Copyright 2010
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
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
#include <common.h>

/* nexell soc headers */
#include <platform.h>
#include <platfunc.h>

/*----------------------------------------------------------------------------*/
#if defined(CONFIG_PM_WAKEUP) && !defined(CONFIG_PM_WAKEUP_ASM)

#define	SLEEP_PHY_BASE			CFG_SLEEP_DATA_BASE
#define	REG_PWRGATEREG			(0xC0019000)
#define REG_SCRATCHREADREG		(0xC0019070)

int nxp3200_serial_init(void);

static void	goto_wakeup(void)
{
	volatile U32 *pData, *pBase;
	U32 size, sum, val=0, xor=0;
	int i=0;

	pBase =  (volatile U32*)(SLEEP_PHY_BASE);
	pData =  (volatile U32*)(SLEEP_PHY_BASE + 0x8);

	size  = *(U32*)(SLEEP_PHY_BASE + 0x4);
	sum   = *(U32*)pBase;

	/*
	 *	Set alive power gating, NX_ALIVE_SetWriteEnable(CTRUE)
	 */
	WriteIODW((volatile U32*)REG_PWRGATEREG, (REG_PWRGATEREG | (0x1)));

	/*
	 * Check scratch register, NX_ALIVE_GetScratchReg()
	 */
	if (CFG_PWR_SLEEP_SIGNATURE == *(volatile U32*)(REG_SCRATCHREADREG))
	{
		/* Check size value */
		if (size <= 0 || size >= 0x100) {
			*pBase = 0x12345678;
			return;
		}

		/* Verify checksum value */
		for (i = 0; i<size; i++, pData++) {
			xor = (*pData ^ 0xAA);
			val ^= xor;
		}

		/* if memory corrupted, goto boot sequency. */
		if (sum != val) {
			*pBase = 0x87654321;
			return;
		}

		asm_cpu_wakeup();

		/* Fail wake up, so reset */
		nxp3200_serial_init();
		serial_puts("Fail, u-boot wakeup, reset cpu ...\n");

		reset_cpu(0);
	}

	/* Cold boot */
	*pBase = 0x56781234;
}
#endif	/* CONFIG_PM_WAKEUP && ! CONFIG_PM_WAKEUP_ASM */

/*------------------------------------------------------------------------------
 * u-boot cpu wakeup
 */
void cpu_mach_wakeup(void)
{
#if defined(CONFIG_PM_WAKEUP)

#if defined(CONFIG_PM_WAKEUP_ASM)
	goto_wakeup_asm();
#else
	goto_wakeup();
#endif

#endif	/* CONFIG_PM_WAKEUP && ! CONFIG_PM_WAKEUP_ASM */
}
