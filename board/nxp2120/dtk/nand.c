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
int msg_print = 0;
#if	(0)
#define DBGOUT(msg...)		{ if (msg_print) printf("nand: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define ERROUT(msg...)		{ 					\
		printf("ERROR: %s, %s line %d: \n",		\
			__FILE__, __FUNCTION__, __LINE__),	\
		printf(msg); }

/*
 * u-boot nand command
 */
#if defined(CONFIG_CMD_NAND)
/* hw control */
static int  nand_hw_ready  (struct mtd_info *mtd);
static void nand_hw_select (struct mtd_info *mtd, int chipnr);
static void nand_hw_ctrl   (struct mtd_info *mtd, int dat, unsigned int ctrl);

/* hw ecc control */
#ifdef CONFIG_SYS_NAND_HW_ECC
extern int nx_nand_ecc_init(struct nand_chip *chip);
#endif
/*------------------------------------------------------------------------------
 * u-boot nand interface
 */
int board_nand_init(struct nand_chip *chip)
{
	int ret = 0;
	DBGOUT("%s\n", __func__);

	NX_MCUS_SetECCMode(CFG_NAND_ECC_MODE);
	NX_MCUS_SetAutoResetEnable(CFG_NAND_AUTORESET);

	NX_MCUS_ClearInterruptPending(0);			// initial R/B pending clear
	NX_MCUS_SetInterruptEnableAll(CTRUE);		// check for R/B

	NX_MCUS_SetNFBank(0);
	NX_MCUS_SetNFCSEnable(CFALSE);				// nand chip select control enable

	/* insert callbacks */
	chip->IO_ADDR_R 	= (void __iomem *)CONFIG_SYS_NAND_BASE;
	chip->IO_ADDR_W 	= (void __iomem *)CONFIG_SYS_NAND_BASE;
	chip->cmd_ctrl 		= nand_hw_ctrl;
	chip->dev_ready 	= nand_hw_ready;
	chip->select_chip 	= nand_hw_select;
	chip->chip_delay 	= 15;

#ifndef CONFIG_SYS_NAND_HW_ECC
	chip->ecc.mode = NAND_ECC_SOFT;
	printf("Software ecc, ");
#else
	ret = nx_nand_ecc_init(chip);
	printf("Hardware %d bit ecc, ", CFG_NAND_ECC_MODE);
#endif
	return ret;
}

/*------------------------------------------------------------------------------
 * u-boot hw control
 */
#define	WAIT_COUNT	20000	/* 20000 */
static int nand_hw_ready(struct mtd_info *mtd)
{
	volatile int cnt = 0;
	int ret = 0;

	for (cnt = 0; WAIT_COUNT > cnt; cnt++) {
		ret = NX_MCUS_GetInterruptPending(0);
		if (ret) {
			NX_MCUS_ClearInterruptPending(0);	// R/B pending clear
			break;
		}
	}
	DBGOUT("[%s: %d, RnB=%d]\n", ret?"READY":"BUSY", cnt, NX_MCUS_IsNFReady());
	return ret;
}

static void nand_hw_select(struct mtd_info *mtd, int chipnr)
{
	DBGOUT("%s, chipnr=%d\n", __func__, chipnr);

	if (chipnr > 4) {
		ERROUT("not support nand chip index %d\n", chipnr);
		return;
	}

	if (-1 == chipnr) {
		NX_MCUS_SetNFCSEnable(CFALSE);		// nand chip select control disable
	} else {
		NX_MCUS_SetNFBank(chipnr);
		NX_MCUS_SetNFCSEnable(CTRUE);
	}
}

#define MASK_CLE		0x10
#define MASK_ALE		0x18
#define writeb(d,addr)	*(volatile u_char *)(addr) = (d & 0xFF)

static void nand_hw_ctrl(struct mtd_info *mtd, int dat, unsigned int ctrl)
{
	struct nand_chip *chip = mtd->priv;
	void __iomem* addr = chip->IO_ADDR_W;

	if (NAND_CMD_NONE == dat)
		return;

	DBGOUT("%s, [%s], data=0x%02x\n", __func__, ctrl&NAND_CLE?"COMM":"ADDR", dat&0xFF);

	if (ctrl & NAND_CLE) {
		addr += MASK_CLE;
	} else if (ctrl & NAND_ALE) {
		addr += MASK_ALE;
	}

	writeb(dat, addr);
}

#endif	/* CONFIG_CMD_NAND */



