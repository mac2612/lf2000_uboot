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
#include <nand.h>

/* nexell soc headers */
#include <platform.h>

/* degug print */
#if	(0)
#define DBGOUT(msg...)		{ printf("nand: " msg); }
#define	ERROUT(msg...)		{ printf("nand: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#define	ERROUT(msg...)		do {} while (0)
#endif

/*
 * u-boot nand command
 */
#if defined(CONFIG_CMD_NAND)

static int  nand_hw_ready(struct mtd_info *mtd);
static void nand_hw_select_chip(struct mtd_info *mtd, int chipnr);
static void nand_hw_control(struct mtd_info *mtd, int cmd, unsigned int ctrl);

/*------------------------------------------------------------------------------
 * u-boot nand interface
 */
int board_nand_init(struct nand_chip *nand)
{
	DBGOUT("%s\n", __func__);

	NX_MCUS_SetNFBank(CFG_NAND_BANK_NUM);
	NX_MCUS_SetECCMode(CFG_NAND_ECC_MODE);
	NX_MCUS_SetAutoResetEnable(CFG_NAND_AUTORESET);

	NX_MCUS_ClearInterruptPending(0);			// initial R/B pending clear
	NX_MCUS_SetNFCSEnable(CTRUE);				// nand chip select control enable
	NX_MCUS_SetInterruptEnableAll(CTRUE);

	/* insert callbacks */
	nand->IO_ADDR_R 	= (void __iomem *)CONFIG_SYS_NAND_BASE;
	nand->IO_ADDR_W 	= (void __iomem *)CONFIG_SYS_NAND_BASE;
	nand->cmd_ctrl 		= nand_hw_control;
	nand->dev_ready 	= nand_hw_ready;
	nand->select_chip 	= nand_hw_select_chip;
	nand->chip_delay 	= 15;
	nand->ecc.mode 		= NAND_ECC_SOFT;

	return 0;
}

/*------------------------------------------------------------------------------
 * u-boot hw control
 */
static int nand_hw_ready(struct mtd_info *mtd)
{
	volatile int cnt = 0;
	int ret = 0;

	for(cnt = 0; cnt < 20000; cnt++) {
		if(NX_MCUS_GetInterruptPending(0)) {
			NX_MCUS_ClearInterruptPending(0);	// R/B pending clear
			ret = 1;
			break;
		}
	}

	return ret;
}

static void nand_hw_select_chip(struct mtd_info *mtd, int chipnr)
{
	switch(chipnr) {
	case -1:
		NX_MCUS_SetNFCSEnable(CFALSE);		// nand chip select control disable
		break;
	case 0:
		NX_MCUS_SetNFBank(chipnr);
		NX_MCUS_SetNFCSEnable(CTRUE);
		break;
	case 1:
		DBGOUT("%s chip=%d \n", __func__, chipnr);
		break;
	default:
		break;
	}
}

#define MASK_CLE		0x10
#define MASK_ALE		0x18
#define writeb(d,addr)	*(volatile u_char *)(addr) = (d)

static void nand_hw_control(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = mtd->priv;
	void __iomem* addr = chip->IO_ADDR_W;

	if (cmd == NAND_CMD_NONE)
		return;

	if(ctrl & NAND_CLE)
		addr += MASK_CLE;
	else if(ctrl & NAND_ALE)
		addr += MASK_ALE;

	writeb(cmd, addr);
}

#endif	/* CONFIG_CMD_NAND */



