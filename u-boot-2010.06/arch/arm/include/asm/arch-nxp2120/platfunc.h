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

#ifndef __PLATFUNC_H__
#define __PLATFUNC_H__

extern void	cpu_mach_init(void);
extern void	cpu_mach_wakeup(void);
extern void cpu_pll_info(void);

extern void reset_cpu(ulong ignored);
#if defined(CONFIG_PM_WAKEUP)
extern void	asm_cpu_wakeup(void);
#endif

#if defined(CONFIG_PM_WAKEUP) && defined(CONFIG_PM_WAKEUP_ASM)
extern void	goto_wakeup_asm(void);
#endif

#ifdef CONFIG_LCD
extern void lcd_init(void);
#endif

#ifdef CONFIG_GENERIC_MMC
extern int sdhc_arch_init(void);
#endif

#endif	/* __PLATFUNC_H__ */



