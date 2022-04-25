/*
 * (C) Copyright 2000-2003
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
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

/*
 * Misc boot support
 */
#include <common.h>
#include <command.h>

#include <mach-types.h>

typedef void  (IMAGE)(unsigned long, unsigned long);

int do_zimage (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong addr = 0, mach = MACH_TYPE_NXP3200;
	void (*zImage)(unsigned long, unsigned long) = NULL;

	if (argc < 2) {
		cmd_usage(cmdtp);
		return 1;
	}

	/* get machine type */
	if (argc == 3)
		mach = simple_strtoul(argv[2], NULL, 16);

	/* get start address */
	addr = simple_strtoul(argv[1], NULL, 16);

	printf ("## Starting zImage at 0x%08X with machine type 0x%X ...\n",
		(u_int)addr, (u_int)mach);

	zImage = (IMAGE*)addr;

	zImage(addr, mach);

	return 0;
}

U_BOOT_CMD(
	zimage, CONFIG_SYS_MAXARGS, 1,	do_zimage,
	"start zImage at address 'addr'",
	"addr\n"
	"    - start zImage at address 'addr'\n"
	"type\n"
	"    - machine type number\n"
);

