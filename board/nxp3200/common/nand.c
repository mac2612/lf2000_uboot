/*
 * (C) Copyright 2010
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <common.h>
#include <nand.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <linux/compiler.h>
#include <linux/mtd/compat.h>
#include <malloc.h>

/* nexell soc headers */
#include <platform.h>
#include <lf2000.h>
#include <lf2000_board.h>
#include <board_revisions.h>
#include "nxp3200_nand_ecc.h"

#if	(0)
#define DBGOUT(msg...)		{ printk(KERN_INFO "nand: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define ERROUT(msg...)		{ 					\
		printk(KERN_ERR "ERROR: %s, %s line %d: \n",		\
			__FILE__, __FUNCTION__, __LINE__),	\
		printk(KERN_ERR msg); }

#define CLEAR_RnB(r)							\
	r = NX_MCUS_GetInterruptPending(0);			\
	if (r) {									\
		NX_MCUS_ClearInterruptPending(0); 		\
		NX_MCUS_GetInterruptPending  (0); 		\
	}
#define CHECK_RnB()	NX_MCUS_GetInterruptPending(0);

/*
 * u-boot nand command
 */

U32	NX_TIMER_GetTimerCounter( U32 ModuleIndex );
/* Enable the following #define to output timing info to the serial port */
//#define OUTPUT_TIMING 
#ifdef OUTPUT_TIMING
static void output_timing( const char * format) {
	printf(format); 
	printf("; tcount %d\n", NX_TIMER_GetTimerCounter(0));
}
#else
#define output_timing(a) 
#endif

#ifdef CONFIG_NAND_BBT_IN_NOR
/* to collect timing info for copying bbt from NOR to ram, enable this #define */
#define TIME_NOR_TO_RAM	
#ifdef TIME_NOR_TO_RAM
unsigned int nor_to_ram_time;
#endif

/* to collect timing info for scanning NAND to identify bad blocks
 * enable the #define */
#define TIME_SCAN_NAND_FOR_BB	
#ifdef TIME_SCAN_NAND_FOR_BB
unsigned int scan_nand_for_bb_time;
unsigned int scan_nand_for_bb_end_time;
#endif

#ifdef CONFIG_DEBUG_BBT_IN_NOR
#define dbg_bbt_nor(msg...)	{ printf(msg); }
#else
#define dbg_bbt_nor(msg...)	do {} while (0)
//#define dbg_bbt_nor(a,...)	printf(a, ...)
//#else
//#define dbg_bbt_nor(a,...) 
#endif


struct mtd_info * get_nor_mtd(void);

int badblock_table_in_NOR(struct mtd_info * mtd, loff_t * p_data_offset,
						 int    * p_data_size); 
static void copy_bbt_from_ram_to_NOR(struct mtd_info * mtdnor, 
			      struct mtd_info * mtdnand); 
static int mark_block_bad_in_NOR_bbt(struct mtd_info *mtdnor, 
				     struct mtd_info * mtdnand,
				     int block, int mark); 

#endif	/* CONFIG_NAND_BBT_IN_NOR */
extern int nand_create_default_bbt_descr(struct nand_chip *this);


#if defined(CONFIG_CMD_NAND)
/* hw control */
static int  nand_dev_ready  (struct mtd_info *mtd);
static void nand_hw_select (struct mtd_info *mtd, int chipnr);
static void nand_dev_ctrl   (struct mtd_info *mtd, int dat, unsigned int ctrl);
static void nand_hw_read_buf(struct mtd_info *mtd, uint8_t *buf, int len);
static void nand_hw_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len);
int lf2000_nand_init_size(struct mtd_info *mtd, struct nand_chip *chip, u8 *id_data);
static int lf2000_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			   const uint8_t *buf, int page, int cached, int raw);
#endif


#if defined (CONFIG_SYS_NAND_HW_ECC)
extern int nand_hw_ecc_init (struct mtd_info  *mtd, int hw_ecc_mode);
#endif

/* Leapfrog specific stuff */
#define LF2000	1
#ifdef LF2000
#if 0
#include <linux/platform_device.h>
#include <linux/delay.h>
#include "../../ubi/ubi-media.h"
#endif
#include "lf2000.h"
#include "nxp3200_nand_ecc.h"

#if defined(CONFIG_PLAT_NXP3200_L2K) || \
	defined(CONFIG_PLAT_NXP3200_M2K)
U32		NX_CLKPWR_GetSystemResetConfiguration( void );
#endif

/* Here are prototypes of MTD Interface functions defined in nand_base.c 
 * They are called from lf2000_nand_read_oob() and lf2000_nand_do_read_ops()
 */
int  nand_do_read_oob(struct mtd_info *mtd, loff_t from,
                      struct mtd_oob_ops *ops);
uint8_t *nand_transfer_oob(struct nand_chip *chip, uint8_t *oob,
         				   struct mtd_oob_ops *ops, size_t len);

/* These functions are referenced by nand_hw_ecc_init() in nand_ecc.c */
int lf2000_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
		                    size_t *retlen, uint8_t *buf);
int lf2000_nand_read_oob(struct mtd_info *mtd, loff_t from,
		            	 struct mtd_oob_ops *ops);

/*
 * Private device structure
 *
 *  onboard_nand points to info about the base NAND flash
 *  cart_nand    points to info about the cartridge NAND flash
 *  controller is the control structure for the LF2000's NAND controller
 */

struct lf2000_nand_devices lf2000_nand = {
	.mem          = NULL,
	.onboard_nand = NULL,
	.cart_nand    = NULL,
	.pdev	      = NULL,
	.cart_ready   = 0,
	.cart_ubi     = 0,

	.L	      = NULL,
	.u_L	      = NULL,
	.reg	      = NULL,
	.s	      = NULL,
	.elp	      = NULL,
	.desc	      = NULL
};

/* Here are prototypes of MTD Interface functions defined in nand_base.c 
 * They are called from lf2000_nand_read_oob() and lf2000_nand_do_read_ops()
 */
int  nand_do_read_oob(struct mtd_info *mtd, loff_t from,
		      struct mtd_oob_ops *ops);
uint8_t *nand_transfer_oob(struct nand_chip *chip, uint8_t *oob,
					   struct mtd_oob_ops *ops, size_t len);

/* These functions are referenced by nand_hw_ecc_init() in nand_ecc.c */
int lf2000_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
				    size_t *retlen, uint8_t *buf);
int lf2000_nand_read_oob(struct mtd_info *mtd, loff_t from,
				 struct mtd_oob_ops *ops);

/*------------------------------------------------------------------------------
 * u-boot nand interface
 *
 * Support two NAND chips, assigned device 0 and device 1.
 * Furthermore, u-boot expects device 0 to be the default boot device.
 *
 * If we booted from NOR then the on-board NAND is device 0 and any cartridge
 * NAND is device 1.
 *
 * Booting from an SD Cartridge then the system has two devices and
 * assign the on-onboard NAND as device 0 and the cartridge NAND as device 1.
 *
 * First call to board_nand_init assigns device 0, second tries to assign device 1.
 * Use private portion of nand_chip select structure to save value.
 */

static U32 next_chip_select = 0;

int board_nand_init(struct nand_chip *chip)
{
	int phys_chip_select = next_chip_select++;

	/* First configure the LF2000's NAND controller's timing registers */

	switch(get_board_rev()) {
	case LF2000_BOARD_VALENCIA_EP_8:
	case LF2000_BOARD_VALENCIA_FEP_8:
	case LF2000_BOARD_VALENCIA_EP_800_480_8:
	case LF2000_BOARD_VALENCIA_FEP_800_480_8:
	case LF2000_BOARD_VALENCIA_KND_800_480_8:
	case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		if (0 == phys_chip_select) {
			if (CFALSE == NX_MCUS_Set_NFTIMEACS(0x7733))
				printf("ERROR: failed to init NFTIMEACS\n");
			if (CFALSE == NX_MCUS_Set_NFTIMECOS(0x7733))
				printf("ERROR: failed to init NFTIMECOS\n");
			if (CFALSE == NX_MCUS_Set_NFTIMEOCH(0x7733))
				printf("ERROR: failed to init NFTIMEOCH\n");
			if (CFALSE == NX_MCUS_Set_NFTIMECAH(0x7733))
				printf("ERROR: failed to init NFTIMECAH\n");
			if (CFALSE == NX_MCUS_Set_NFTIMEACC(0x0A0A))
				printf("ERROR: failed to init NFTIMEACC\n");
		}
		break;
	default:
		if (0 == phys_chip_select) {
			if (CFALSE == NX_MCUS_Set_NFTIMEACS(0x7711))
				printf("ERROR: failed to init NFTIMEACS\n");
			if (CFALSE == NX_MCUS_Set_NFTIMECOS(0x7711))
				printf("ERROR: failed to init NFTIMECOS\n");
			if (CFALSE == NX_MCUS_Set_NFTIMEOCH(0x7711))
				printf("ERROR: failed to init NFTIMEOCH\n");
			if (CFALSE == NX_MCUS_Set_NFTIMECAH(0x77ff))
				printf("ERROR: failed to init NFTIMECAH\n");
			if (CFALSE == NX_MCUS_Set_NFTIMEACC(0x0505))
				printf("ERROR: failed to init NFTIMEACC\n");
		}
		break;
	}

	if (!lf2000_nand.L) {
		lf2000_nand.L = kzalloc(COMMON_BUFFER_SIZE, GFP_KERNEL);
		if (!lf2000_nand.L) {
			printk(KERN_ERR "failed to allocate common buffers.\n");
			return -ENOMEM;
		}
		lf2000_nand.u_L	 = lf2000_nand.L   + L_ENTRIES;
		lf2000_nand.reg	 = lf2000_nand.L   + uL_ENTRIES;
		lf2000_nand.s	 = lf2000_nand.reg + reg_ENTRIES;
		lf2000_nand.elp	 = lf2000_nand.s   + s_ENTRIES;
		lf2000_nand.desc = lf2000_nand.elp + elp_ENTRIES;
	}

	DBGOUT("%s.%d, next_chip_select =%d\n", __func__, __LINE__, 
		next_chip_select);
	DBGOUT("%s.%d, is_nor_boot()=%d\n", __func__, __LINE__, is_nor_boot());
	DBGOUT("%s.%d, phys_chip_select =%d\n", __func__, __LINE__, 
		phys_chip_select);

	switch(phys_chip_select)
	{
	case 0:
		if (!lf2000_nand.onboard_nand) {
			lf2000_nand.onboard_nand = (struct lf2000_nand_data *)
				kzalloc(sizeof(struct lf2000_nand_data), 
					GFP_KERNEL);
			if (!lf2000_nand.onboard_nand) {
				printk(KERN_ERR "failed to allocate "
					"onboard_nand device structure.\n");
				return -ENOMEM;
			}
		}
		lf2000_nand.onboard_nand->chip	     = *chip;
		lf2000_nand.onboard_nand->nand_bank  = phys_chip_select;
#ifdef CONFIG_NAND_BBT_IN_NOR
		lf2000_nand.onboard_nand->bbt_in_nor = 1;
#else
		lf2000_nand.onboard_nand->bbt_in_nor = 0;
#endif
		chip->priv = lf2000_nand.onboard_nand;
		break;
	case 1:
		if (!lf2000_nand.cart_nand) {
			lf2000_nand.cart_nand = (struct lf2000_nand_data *)
				kzalloc(sizeof(struct lf2000_nand_data), 
					GFP_KERNEL);
			if (!lf2000_nand.cart_nand) {
				printk(KERN_ERR "failed to allocate cart_nand "
						"device structure.\n");
				return -ENOMEM;
			}
		}
		lf2000_nand.cart_nand->chip	  = *chip;
		lf2000_nand.cart_nand->nand_bank  = phys_chip_select;
		lf2000_nand.cart_nand->bbt_in_nor = 0;
		chip->priv = lf2000_nand.cart_nand;
	default:
		printf("board_nand_init: unexpected chip index: %d\n",
			phys_chip_select);
		return -EINVAL;
	}

	NX_MCUS_SetECCMode(CFG_NAND_ECC_MODE);
	NX_MCUS_SetAutoResetEnable(CFG_NAND_AUTORESET);

	NX_MCUS_ClearInterruptPending(0);	// initial R/B pending clear
	NX_MCUS_SetInterruptEnableAll(CTRUE);	// check for R/B

	NX_MCUS_SetNFBank(phys_chip_select);
	NX_MCUS_SetNFCSEnable(CFALSE);	      // nand chip select control enable

	/* insert callbacks */
	chip->IO_ADDR_R 	= (void __iomem *)CONFIG_SYS_NAND_BASE;
	chip->IO_ADDR_W 	= (void __iomem *)CONFIG_SYS_NAND_BASE;
        chip->options           = 0; 		/* 8 bit bus width */
	chip->cmd_ctrl 		= nand_dev_ctrl;
	chip->dev_ready 	= nand_dev_ready;
	chip->select_chip 	= nand_hw_select;
	chip->chip_delay 	= 15;
	chip->read_buf		= nand_hw_read_buf;
	chip->write_buf		= nand_hw_write_buf;
	chip->write_page	= lf2000_nand_write_page;
	chip->init_size		= lf2000_nand_init_size; 
	chip->ecc.mode		= NAND_ECC_SOFT;

	DBGOUT("%s.%d, *nand_chip = 0x%8.8X\n", __func__, __LINE__, 
		(unsigned int)chip);
	DBGOUT("%s.%d, next_chip_select=%d\n", __func__, __LINE__, 
		next_chip_select);
	DBGOUT("%s.%d, chip->priv=%p\n", __func__, __LINE__, chip->priv);

	return 0;
}

/*------------------------------------------------------------------------------
 * u-boot hw control
 */
#define	WAIT_COUNT	20000	/* wait up to 20 ms */
static int nand_dev_ready(struct mtd_info *mtd)
{
	volatile int cnt = 0;
	int ret = 0;

	for (cnt = 0; cnt < (WAIT_COUNT/10); cnt++) {
		ret = NX_MCUS_GetInterruptPending(0);
		if (ret) {
			NX_MCUS_ClearInterruptPending(0);	// R/B pending clear
			break;
		udelay(10);
		}
	}
	DBGOUT("[%s: %d, RnB=%d]\n", ret?"READY":"BUSY", cnt, NX_MCUS_IsNFReady());
	return ret;
}

#if 1	/* 8feb12	Experiment */
#ifndef MASK_CLE
#define MASK_CLE		0x10
#define MASK_ALE		0x18
#endif
#define NAND_CMD_READ	0x00
#define NAND_CMD_READSTART 0x30

void nand_special_read( loff_t offset, int count, unsigned char * p) 
{
	unsigned char c1, c2, r1, r2, r3;

	c1 = offset & 0x00ff;
	c2 = (offset >> 8) & 0x0f;
	c2 |= 0x10;	/* indicate the first byte of the spare area */
	r1 = (offset >> 12) & 0xff;
	r2 = (offset >> 20) & 0xff;
	r3 = (offset >> 28) & 0xff;

	writeb(NAND_CMD_READ, (void __iomem*)CONFIG_SYS_NAND_BASE + MASK_CLE);
	writeb(c1, (void __iomem*)CONFIG_SYS_NAND_BASE + MASK_ALE);
	writeb(c2, (void __iomem*)CONFIG_SYS_NAND_BASE + MASK_ALE);
	writeb(r1, (void __iomem*)CONFIG_SYS_NAND_BASE + MASK_ALE);
	writeb(r2, (void __iomem*)CONFIG_SYS_NAND_BASE + MASK_ALE);
	writeb(r3, (void __iomem*)CONFIG_SYS_NAND_BASE + MASK_ALE);
	writeb(NAND_CMD_READSTART, (void __iomem*)CONFIG_SYS_NAND_BASE + MASK_CLE);

	/* Now wait for Ready */
	{
	volatile int cnt = 0;
	int ret = 0;

	for (cnt = 0; cnt < (WAIT_COUNT/10); cnt++) {
		ret = NX_MCUS_GetInterruptPending(0);
		if (ret) {
			NX_MCUS_ClearInterruptPending(0);	// R/B pending clear
			break;
		udelay(10);
		}
	}
	DBGOUT("[%s: %d, RnB=%d]\n", ret?"READY":"BUSY", cnt, NX_MCUS_IsNFReady());
	}

	/* Now read the bytes */
	while (count-- > 0) {
		*p++ = readb((void __iomem*)CONFIG_SYS_NAND_BASE);
	}
}
#endif


/*
 * nand_hw_select()
 *
 * use chip select associated with mtd if chipnr is not -1
 */
static void nand_hw_select(struct mtd_info *mtd, int chipnr)
{
	struct lf2000_nand_data *lf2000_nand =
		(struct lf2000_nand_data *)((struct nand_chip *)mtd->priv)->priv;
	int nand_bank = -1;

	if (lf2000_nand != NULL)
		nand_bank = lf2000_nand->nand_bank;
		
	if (nand_bank > 4) {
		ERROUT("%s:%s.%d nand_bank not supported (%d)\n", 
			__FILE__, __func__, __LINE__, nand_bank);
		return;
	}

	if (-1 == nand_bank) {
		NX_MCUS_SetNFCSEnable(CFALSE);	// nand chip select control disable
	} else {
		// NX_MCUS_SetNFBank(chipnr);
		NX_MCUS_SetNFBank(nand_bank);	// E2K swaps chip selects when NOR booting
		NX_MCUS_SetNFCSEnable(CTRUE);
	}
}

#define MASK_CLE		0x10
#define MASK_ALE		0x18

static void nand_dev_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = mtd->priv;
	void __iomem* addr = chip->IO_ADDR_W;
	int ret = 0;

	if (cmd == NAND_CMD_NONE)
		return;

	if (ctrl & NAND_CLE)
		writeb(cmd, addr + MASK_CLE);
	else if (ctrl & NAND_ALE)
		writeb(cmd, addr + MASK_ALE);

	if (cmd != NAND_CMD_RESET &&
	    cmd != NAND_CMD_READSTART)
		CLEAR_RnB(ret);
}

static void nand_hw_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
 	readsl(chip->IO_ADDR_R, buf, (len >> 2));
	if (len & 3)
		readsb(chip->IO_ADDR_R, buf + (len & ~0x3), (len & 3));
}

static void nand_hw_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	writesl(chip->IO_ADDR_W, buf, (len >> 2));
	if (len & 3)
		writesb(chip->IO_ADDR_W, buf + (len & ~0x3), (len & 3));
}
/*------------------------------------------------------------------------------
 * u-boot nand module
 */

extern int nand_get_device( struct nand_chip *chip, 
			    struct mtd_info *mtd, int new_state);
extern void nand_release_device(struct mtd_info *mtd);
int nand_do_write_oob(struct mtd_info *mtd, loff_t to,
			     	  struct mtd_oob_ops *ops);

/**
 * nand_block_bad_first_or_last
 * @mtd:	MTD device structure
 * @ofs:	offset from device start
 * @getchip:	0, if the chip is already selected
 *
 * Returns nonzero (TRUE) if block is bad.
 *
 * This is derived from the default implementation (nand_block_bad() in
 * nand_base.c).
 * It considers a block to be bad if it finds a non-FF value in the first oob
 * byte of either the first page or the last page of the block.  This conforms
 * to the method that Samsung and Hynix use for marking bad blocks in their 
 * 2GB sub-50nm MLC NANDs.
 */
static int
nand_block_bad_first_or_last(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	int page, chipnr, res = 0;
	struct nand_chip *chip = mtd->priv;
	u8 bad;

#if 0	/* we hardly ever want/need this output */
	dbg_bbt_nor( "nbbfol: ofs 0x%x, sob 0x%x, page 0x%x, getchip %d\n",
		(unsigned int)ofs, (unsigned int)(ofs & ~(mtd->erasesize -1)),
		(int)(ofs >> chip->page_shift) & chip->pagemask, getchip);
#endif
	page = (int)(ofs >> chip->page_shift) & chip->pagemask;

	if (getchip) {
		chipnr = (int)(ofs >> chip->chip_shift);

		nand_get_device(chip, mtd, FL_READING);

		/* Select the NAND device */
		chip->select_chip(mtd, chipnr);
	}
		/* since the LF2000 doesn't support 16-bit NANDs, we omit the 
		 * check for 16-bit nand bus.
		 */
	chip->cmdfunc(mtd, NAND_CMD_READOOB, chip->badblockpos, page);
	bad = chip->read_byte(mtd);

	res = (bad != 0xFF);
	if (!res) {	/* first page oob does not indicate bad.  Check last page. */
		// TODO: Check if this adjustment is correct.  
		//		 Is ofs initially the offset to the first page of the block?
		ofs += mtd->erasesize - mtd->writesize;
		page = (int)(ofs >> chip->page_shift) & chip->pagemask;
		chip->cmdfunc(mtd, NAND_CMD_READOOB, chip->badblockpos, page);
		bad = chip->read_byte(mtd);
		res = (bad != 0xFF);
	}
	if (getchip)
		nand_release_device(mtd);

	return res;
}



/**
 * nand_block_markbad_first_and_last
 * @mtd:	MTD device structure
 * @ofs:	offset from device start
 *
 * This overrides the default implementation.
 * It stores a non-FF value in the oob of both the first and last pages of
 * the block.  This conforms to the method that Samsung and Hynix use for
 * marking bad blocks in their 2GB sub-50nm MLC NANDs.
 */


static int nand_block_markbad_first_and_last(struct mtd_info *mtd, loff_t ofs)
{
#ifdef CONFIG_NAND_BBT_IN_NOR
	struct lf2000_nand_data * pnd;
#endif
	struct nand_chip *chip = mtd->priv;
	uint8_t buf = 0;
	int block;
	int ret;


	/* Get block number */
	block = (int)(ofs >> chip->bbt_erase_shift);
	printk(KERN_INFO "nbmfal: ofs 0x%x, block 0x%x\n", (unsigned int)ofs, block);

#ifdef CONFIG_NAND_BBT_IN_NOR
	/* If NOR contains a bad block table for this NAND, mark the block
	 * bad in that table.
	 */
	pnd = (struct lf2000_nand_data *)chip->priv;
	if (pnd->bbt_in_nor) {
		ret = mark_block_bad_in_NOR_bbt(get_nor_mtd(), mtd, block, 1);
	}
#if 0	/* 23mar12 For initial testing also mark block bad in NAND */
	else 
#endif
#endif
	/* If the NAND contains a bad block table for itself, update it. */
	if (chip->options & NAND_USE_FLASH_BBT) {
		ret = nand_update_bbt(mtd, ofs);
	}
	else {	/* Try to mark the block bad by writing 0x00 to the first
		 * byte of the oob of its first and last pages.
		 */
		nand_get_device(chip, mtd, FL_WRITING);

		/* Write to first and last page.
		 * If we detect an error, we immediately quit the procedure. 
		 * We write one byte per location because the LF2000 supports
		 * only an 8-bit nand bus.
		 */
		chip->ops.len	  = chip->ops.ooblen = 1;
		chip->ops.datbuf  = NULL;
		chip->ops.oobbuf  = &buf;
		chip->ops.ooboffs = 0;

		ret = nand_do_write_oob(mtd, ofs, &chip->ops);

		if (!ret) {
			ofs += mtd->erasesize - mtd->writesize;
			ret  = nand_do_write_oob(mtd, ofs, &chip->ops);
		}
		nand_release_device(mtd);
	}

	/* If there's a RAM-resident back block table for this NAND, mark
	 * the block bad in that table.
	 * Do this last, in case we decide to erase the block before trying
	 * to write to oob.
	 */
	if (chip->bbt)
		chip->bbt[block >> 2] |= 0x01 << ((block & 0x03) << 1);

	if (!ret)
		mtd->ecc_stats.badblocks++;

	return ret;
}



static inline void mark_block_bad_in_ram_bbt(struct nand_chip * this, 
					 int block_index_x2)
{
	this->bbt[block_index_x2 >> 3] |= 0x03 << (block_index_x2 & 0x6);
}


/* this is derived from create_bbt() in nand_bbt.c */
/**
 * lf_create_bbt - [GENERIC] Create a bad block table by scanning the device
 * @mtd:	MTD device structure
 * @buf:	temporary buffer
 * @bd:		descriptor for the good/bad block search pattern
 * @chip:	create the table for a specific chip, -1 read all chips.
 *		Applies only if NAND_BBT_PERCHIP option is set
 *
 * Create a bad block table by scanning the device
 * for the given good/bad block identify pattern
 */
static int lf_create_bbt(struct mtd_info *mtd,      uint8_t *buf,
			 struct nand_bbt_descr *bd, int chip)
{
	struct lf2000_nand_data * pnd;
	struct nand_chip	* this = mtd->priv;
	int    i;
	int    numblocks;
	int    startblock;
	loff_t from;
	loff_t blocksize;

	printf("Scanning NAND for bad blocks: timer cnt %d\n",
		NX_TIMER_GetTimerCounter(0));
	output_timing("              lf_create_bbt#1");

	blocksize = mtd->erasesize;
		/* Note that startblock and numblocks are 2 * (real numblocks) 
		 * here, see i+=2 below as it makes shifting and masking 
		 * less painful 
		 */
	pnd = (struct lf2000_nand_data *)this->priv;
		/* if there will be a bbt in NOR for this NAND, we will
		 * check the entire NAND for bad blocks.
		 * Otherwise we'll check only the kernel partition.
		 */
	if (pnd->bbt_in_nor) {
		numblocks  = mtd->size >> (this->bbt_erase_shift - 1);
		startblock = 0;
		from	   = 0;
	}
	else {
		from	   = CONFIG_KERNEL_PARTITION_OFFSET;
		startblock = CONFIG_KERNEL_PARTITION_OFFSET 
					>> (this->bbt_erase_shift - 1);
		numblocks  = CONFIG_KERNEL_PARTITION_SIZE 
					>> (this->bbt_erase_shift - 1);
	}

#ifdef TIME_SCAN_NAND_FOR_BB
	scan_nand_for_bb_time = NX_TIMER_GetTimerCounter(0);
#endif
	nand_get_device(this, mtd, FL_READING);

	for (i = 0; i < numblocks; i += 2) {
		int ret;

		ret = this->block_bad(mtd, from, 0);

		if (ret < 0) {
			nand_release_device(mtd);
			printf("lf_create_bbt: ERROR %d from "
				"block_bad on block %d\n",
				ret, (startblock+i)/2);
			return ret;
		}

		if (ret) {
			mark_block_bad_in_ram_bbt(this, startblock + i);
#ifndef TIME_SCAN_NAND_FOR_BB
			printk(KERN_WARNING "Bad eraseblock %d at 0x%012llx"
					    "    %s page oob: 0x%02x\n",
			       (startblock + i) >> 1, (unsigned long long)from,
				   ((ret > 1) ? "last" : "first"), buf[0]);
#endif
			mtd->ecc_stats.badblocks++;
		}
		/* else don't change bbt bits for good blocks */

		from += blocksize;
	}
#ifdef TIME_SCAN_NAND_FOR_BB
	scan_nand_for_bb_end_time = NX_TIMER_GetTimerCounter(0);
#endif
	nand_release_device(mtd);

	printf("              lf_create_bbt#2: timer cnt %d; "
		"starting block %d, # of blocks %d\n", 
		NX_TIMER_GetTimerCounter(0), startblock/2, numblocks/2);
	return 0;
}



static int lf_nand_scan_bbt(struct mtd_info *mtd, struct nand_bbt_descr *bd)
{
	struct nand_chip *this = mtd->priv;
	int len, res = 0;

	output_timing("            lf_nand_scan_bbt#1");
	len = mtd->size >> (this->bbt_erase_shift + 2);
	/* Allocate memory (2bit per block) and clear the memory bad block table */
	this->bbt = (uint8_t *)kzalloc(len, GFP_KERNEL);
	if (!this->bbt) {
		printk(KERN_ERR "lf_nand_scan_bbt: Out of memory\n");
		return -ENOMEM;
	}

	// this code is from inline nand_memory_bbt() in nand_bbt.c
	bd->options &= ~NAND_BBT_SCANEMPTY;
	res = lf_create_bbt(mtd, this->buffers->databuf, bd, -1);
	output_timing("            lf_nand_scan_bbt#2");
	if (res) {
		printk(KERN_ERR "nand_bbt: Can't create the RAM-based BBT\n");
		kfree(this->bbt);
		this->bbt = NULL;
	}
	return res;
}



#ifdef CONFIG_NAND_BBT_IN_NOR
static int lf_nand_bbt_nor(struct mtd_info *mtd, struct nand_chip * chip)
{
	struct mtd_info *mtdnor;
	loff_t bbt_data_offset;
	int    bbt_size;
	int    status;

	output_timing("          lf_nand_bbt_nor#1");

	mtdnor = get_nor_mtd();
	if ( badblock_table_in_NOR(mtdnor, &bbt_data_offset, &bbt_size)) 
	{
		printf("Found %d-byte bbt at offset 0x%x in NOR\n",
			bbt_size, (unsigned int)bbt_data_offset);
		status = copy_bbt_from_NOR_to_ram(mtdnor, mtd, 
						  bbt_data_offset, bbt_size);
#ifdef TIME_NOR_TO_RAM
		printf(" Copy bbt from NOR to RAM took %d ticks (%d usec)\n",
			nor_to_ram_time, (nor_to_ram_time * 434)/1000);
#endif
		output_timing("          lf_nand_bbt_nor#2");
		if (0 == status)
			return 0;
	}
	printf("Found no bbt in NOR\n");

	/* Either no bad block table in NOR or we couldn't read it into
	 *  RAM.  Generate it and then write it to NOR.
	 */
	chip->bbt_td = NULL;
	chip->bbt_md = NULL;
	if (!chip->badblock_pattern)
		nand_create_default_bbt_descr(chip);

	output_timing("          lf_nand_bbt_nor#3");
	status = lf_nand_scan_bbt(mtd, chip->badblock_pattern);

#ifdef TIME_SCAN_NAND_FOR_BB
	if (scan_nand_for_bb_end_time > scan_nand_for_bb_time) {
		scan_nand_for_bb_time = scan_nand_for_bb_end_time
					- scan_nand_for_bb_time;
		printf(" Scan NAND for bbt took %d ticks (%d usec)\n",
			scan_nand_for_bb_time, 
			(scan_nand_for_bb_time * 434)/1000);
	}
	else {
		unsigned int delta;

		printf( " Scan NAND for bbt: start %d (0x%x),"
			" end %d (0x%x) ticks\n",
			scan_nand_for_bb_time, scan_nand_for_bb_time, 
			scan_nand_for_bb_end_time, 
			scan_nand_for_bb_end_time);
		delta = NX_TIMER_GetMatchCounter(0);
		printf("  (rollover at 0x%x\n", delta);
		delta = delta - scan_nand_for_bb_time 
				+ scan_nand_for_bb_end_time;
		printf("  delta: %d (0x%x); %d ticks\n",
			delta, delta, (delta * 434)/1000);
	}
#endif
	if (0 == status) {
		/* we don't check the status returned by 
		 * copy_bbt_from_ram_to_NOR() because that routine 
		 * takes care of error messages, and this routine
		 * returns a status that indicates whether or not
		 * a bbt was constructed in RAM.
		 */
		copy_bbt_from_ram_to_NOR(mtdnor, mtd);
	}
	return status;
}
#endif	/* CONFIG_NAND_BBT_IN_NOR */

int lf_nand_bbt(struct mtd_info *mtd)
{
#ifdef CONFIG_NAND_BBT_IN_NOR
	struct lf2000_nand_data * pnd;
#endif
	struct nand_chip	* chip = mtd->priv;
	int    status;

	output_timing("          lf_nand_bbt#1");

#ifdef CONFIG_NAND_BBT_IN_NOR
	pnd = (struct lf2000_nand_data *)chip->priv;
	/* if this NAND does not have a NOR-resident bbt */
	if ( !pnd->bbt_in_nor) 
#endif
	{
		output_timing("          lf_nand_bbt#2a");
		chip->bbt_td = NULL;
		chip->bbt_md = NULL;
		if (!chip->badblock_pattern)
			nand_create_default_bbt_descr(chip);

		status = lf_nand_scan_bbt(mtd, chip->badblock_pattern);
		output_timing("          lf_nand_bbt#2b");
	}
#ifdef CONFIG_NAND_BBT_IN_NOR
	else {
		status = lf_nand_bbt_nor( mtd, chip);
	}
#endif
	return status;
}









/* This is derived from nand_default_bbt() in nand_bbt.c.
 * It is needed in order to have the bbt creator check both the first and last
 * pages of an eraseblock.  This is needed for the sub-50nm MLC NANDs from
 * Samsung and Hynix.
 */


#ifdef CONFIG_NAND_BBT_IN_NOR

	/* TODO: FIXME: make this more general */
static inline unsigned int bbt_offset_in_nor(struct mtd_info * mtd) {
	return (unsigned int)mtd->size - 3 * 4096;
}

#define BBT_MAGIC1	('L' | ('F' << 8) | ('N' << 16) | ('A' << 24))
#define BBT_MAGIC2	('N' | ('D' << 8) | ('B' << 16) | ('B' << 24))
#define DATA_BYTES_MAGIC	(0xdb)
struct bbt_header {
	unsigned int	bbt_magic1;
	unsigned int	bbt_magic2;
	unsigned char	version;
	unsigned char	data_bytes_magic;
	unsigned short	num_data_bytes;
	unsigned int	num_blocks;
};

struct bbt_trailer {
	unsigned int	be_num_blocks;
	unsigned short	be_num_data_bytes;
	unsigned char	data_bytes_magic;
	unsigned char	version;
	unsigned int	be_bbt_magic2;
	unsigned int	be_bbt_magic1;
};

static unsigned char recognized_bbt_versions[] = {
	1
};

static int bbt_version_ok(unsigned char version) 
{
	int i;
	for (i = 0; i < sizeof(recognized_bbt_versions); ++i)
	{
		if (version == recognized_bbt_versions[i])
			return 1;
	}
	return 0;
}

/* returns nonzero iff valid bbt is in NOR
 * if valid bbt is found,
 *   offset of its data section is stored in p_data_offset 
 *   size (in bytes) of its data section is stored in p_data_size
 *
 * TODO: FIXME: as necessary, add code to check at different offsets, etc.
 */
int badblock_table_in_NOR(struct mtd_info * mtd, loff_t * p_data_offset,
						 int    * p_data_size) 
{ 
	struct bbt_header  header;
	struct bbt_trailer trailer;
	loff_t	offset;
	size_t  retlen;

#if 0	/* enable next line if you want/need to force erasure of NOR's */
	/* sector that contains the bad block table */
	return 0;
#endif
	if ((mtd == NULL) || (p_data_offset == NULL) || (p_data_size == NULL))
		return 0;

	dbg_bbt_nor("badblock_table_in_NOR: mtd->size 0x%x\n", 
		    (unsigned int)mtd->size);
	offset = bbt_offset_in_nor(mtd);
	mtd->read(mtd, offset, sizeof(header), &retlen, (u_char *)&header);
	
	if (retlen != sizeof(header)) {
		dbg_bbt_nor(" bbt_in_NOR failed: retlen %d,"
			    " sizeof(header) %d\n",
			     (unsigned int)retlen, sizeof(header));
		return 0;
	}

	if (   (header.bbt_magic1 != BBT_MAGIC1)
	    || (header.bbt_magic2 != BBT_MAGIC2)
	    || (header.data_bytes_magic != DATA_BYTES_MAGIC)
	    || !bbt_version_ok(header.version) ) {

		dbg_bbt_nor(" hdr.mag1 0x%x, mag2 0x%x,"
			    " dbmag 0x%x, version %d\n",
				header.bbt_magic1, header.bbt_magic2, 
				header.data_bytes_magic, header.version);
		return 0;
	}

	switch (header.version) {
	case 1:	if (header.num_data_bytes == header.num_blocks/4) {
			*p_data_offset = offset + sizeof(header);
			*p_data_size   = header.num_data_bytes;
		}
		break;
	default: printf(" version not 1\n");
		return 0;
	}
	/* header is valid; now check the trailer, which starts on the first 16-byte 
	 * boundary after the end of the bbt data.
	 */
	offset = offset + sizeof(header) 
			+ ((header.num_data_bytes + 15) & ~0x0f);
	mtd->read(mtd, offset, sizeof(trailer), &retlen, (u_char *)&trailer);
	
	if (retlen != sizeof(trailer)) {
		dbg_bbt_nor(" bbt_in_NOR failed: retlen %d,"
			    " sizeof(trailer) %d\n",
				(unsigned int)retlen, sizeof(trailer));
		return 0;
	}

	if (   (trailer.be_bbt_magic1 != cpu_to_be32(BBT_MAGIC1))
	    || (trailer.be_bbt_magic2 != cpu_to_be32(BBT_MAGIC2))
	    || (trailer.data_bytes_magic != DATA_BYTES_MAGIC)
	    || (trailer.version != header.version)) {

		dbg_bbt_nor(" tlr.mag1 0x%x, mag2 0x%x,"
			    " dbmag 0x%x, version %d\n",
				trailer.be_bbt_magic1, trailer.be_bbt_magic2, 
				trailer.data_bytes_magic, trailer.version);
		dbg_bbt_nor(" cpu_to_be32(BBT_MAGIC1) 0x%x,"
			    " ..(BBT_MAGIC2) 0x%x\n",
			     cpu_to_be32(BBT_MAGIC1), cpu_to_be32(BBT_MAGIC2));
		return 0;
	}

	if (   (trailer.be_num_blocks != cpu_to_be32(header.num_blocks))
	    || (trailer.be_num_data_bytes 
				!= cpu_to_be16(header.num_data_bytes))) 
	{
		dbg_bbt_nor( "tlr.be_num_blocks 0x%x, "
			     "cpu_to_be32(header.num_blocks) 0x%x\n",
				trailer.be_num_blocks, 
				cpu_to_be32(header.num_blocks));
		dbg_bbt_nor( "tlr.be_num_data_bytes 0x%x, "
			     "cpu_to_be16(header.num_data_bytes) 0x%x\n",
				trailer.be_num_data_bytes, 
				cpu_to_be16(header.num_data_bytes));
		return 0; 
	}
	printf("  valid bbt in NOR\n");

	/* if we reach here, everything looks ok */
	return 1;
}

int copy_bbt_from_NOR_to_ram(struct mtd_info * mtdnor, 
			     struct mtd_info * mtdnand,
			     loff_t data_offset, int data_size)
{
	size_t retlen;
	struct nand_chip *this;
	int i;
	int num_bad;
	unsigned int * p32;

	if ((mtdnor == NULL) || (mtdnand == NULL))
		return -EINVAL;

	this = mtdnand->priv;
	this->bbt = (uint8_t *)kzalloc(data_size, GFP_KERNEL);
	if (!this->bbt) {
		printk(KERN_ERR "copy_bbt_from_NOR_to_ram: Out of memory\n");
		return -ENOMEM;
	}
#ifdef TIME_NOR_TO_RAM
	nor_to_ram_time = NX_TIMER_GetTimerCounter(0);
#endif
	mtdnor->read(mtdnor, data_offset, data_size, &retlen, (u_char *)this->bbt);

#ifdef TIME_NOR_TO_RAM
	nor_to_ram_time = NX_TIMER_GetTimerCounter(0) - nor_to_ram_time;
#endif

		
	for (i = num_bad = 0, p32 = (unsigned int *)this->bbt; 
	     i < retlen; i += 4, ++p32) {
		unsigned int value;

		value = *p32 ^ 0xFFFFFFFF;
		*p32  = value;
		if (value) 	/* Here bad blocks are indicated by nonzero */
		{		/* pairs of bits.  Count 'em */
			int j;

			for (j = 0; (value != 0) && (j < 16); ++j, value >>= 2) 
			{
				if (0 != (value & 3)) {
					if (0 == num_bad)
						dbg_bbt_nor( 
							"Bad blocks from NOR-"
							"resident table:\n");
					dbg_bbt_nor(
						"  block %d marked %d in NOR\n",
						4 * i + j, (~value & 3));
					++num_bad;
				}
			}
		}
	}
	if (num_bad) {
		printf("Found %d block%s marked bad in NOR\n",
			num_bad, (num_bad > 1) ? "s" : "");
		/* output this info even if CONFIG_DEBUG_BBT_IN_NOR is not
		 * defined. */
	}
	return 0; 
}

int erase_bbt_in_NOR(struct mtd_info * mtd) {
	struct erase_info einfo;
	int status;

	if ((mtd == NULL) || (mtd->size < 0x80000))
		return -1;

	dbg_bbt_nor("  erase_bbt_in_NOR: mtd->size 0x%x\n", 
			(unsigned int)mtd->size);

	einfo.addr     = bbt_offset_in_nor(mtd);
	einfo.len      = 4096;
	einfo.mtd      = mtd;
	einfo.callback = NULL;

	status = mtd->erase(mtd, &einfo);

	dbg_bbt_nor("  erase_bbt_in_NOR: stat %d;"
		    " addr 0x%x, len %d, state %d\n", 
			status, (unsigned int)einfo.addr, 
			(unsigned int)einfo.len, einfo.state);
	if (status != 0) 
		dbg_bbt_nor("  erase_bbt_in_NOR: FAILED to erase NOR sector\n");
	return status;
}

static int write_bbt_header_to_NOR(struct mtd_info *mtdnor, struct mtd_info *mtdnand) {
	int    status;
	size_t num_written;
	struct bbt_header header;
	struct nand_chip *this = mtdnand->priv;

	header.bbt_magic1	= BBT_MAGIC1;
	header.bbt_magic2	= BBT_MAGIC2;
	header.version		= 1;	/* TODO: FIXME: generalize this */
	header.data_bytes_magic = DATA_BYTES_MAGIC;
	header.num_blocks	= mtdnand->size >> (this->bbt_erase_shift);
	header.num_data_bytes	= (header.num_blocks + 3)/4;

	status = mtdnor->write( mtdnor, bbt_offset_in_nor(mtdnor),
				sizeof(header),&num_written, (u_char *)&header);

	dbg_bbt_nor("  write_bbt_header_to_NOR: # blks 0x%x,"
		    " # data bytes 0x%x\n",
			header.num_blocks, header.num_data_bytes);
	dbg_bbt_nor("  write_bbt_header_to_NOR: status %d\n", status);
	return status;
}

static int write_bbt_to_NOR(struct mtd_info *mtdnor, struct mtd_info * mtdnand) 
{
        int len;
	int status;
	size_t num_written;
        struct nand_chip *this = mtdnand->priv;
	int	       num_words;
	int	       i;
	unsigned int * p32;

	if ((NULL == mtdnor) || (NULL == mtdnand))
		return -EINVAL;

        len = mtdnand->size >> (this->bbt_erase_shift + 2); /* # of blks / 4 */

	/* flip the bits so 11 == good */
	for (p32 = (unsigned int *)this->bbt, num_words = (len + 3)/4, i = 0;
	     i < num_words; ++i, ++p32)
	{
		*p32 ^= 0xFFFFFFFF;
	}

	status = mtdnor->write( mtdnor, bbt_offset_in_nor(mtdnor)
					 + sizeof(struct bbt_header), 
				len, &num_written, (u_char *)this->bbt );

	/* flip back so 00 == good */
	for (p32 = (unsigned int *)this->bbt, num_words = (len + 3)/4, i = 0;
	     i < num_words; ++i, ++p32)
	{
		*p32 ^= 0xFFFFFFFF;
	}
	dbg_bbt_nor("  write_bbt_to_NOR wrote %d bytes"
		    " of bbt data, status %d\n",
			(unsigned int)num_written, status);
	return status;
}

static int write_bbt_trailer_to_NOR(struct mtd_info *mtdnor, struct mtd_info *mtdnand) {
	struct bbt_trailer trailer;
	int	status;
	size_t	num_written;
	int	num_blocks;
	int	num_bbt_bytes;
	int	offset;
	struct nand_chip *this = mtdnand->priv;

	num_blocks		 = mtdnand->size >> (this->bbt_erase_shift);
	num_bbt_bytes		 = (num_blocks + 3)/4;	
				    /* 2 bits per block, so 4 blocks per byte */
	offset			 = bbt_offset_in_nor(mtdnor);	
				    /* offset of header */
	offset			 += sizeof(struct bbt_header);	
				    /* offset of bbt data */
	offset			 += (num_bbt_bytes + 15) & ~0x0f; 
				    /* trailer on 16-byte bdry*/
	trailer.be_num_blocks	  = cpu_to_be32(num_blocks);	
	trailer.be_num_data_bytes = cpu_to_be16(num_bbt_bytes);
	trailer.version		  = 1;
	trailer.data_bytes_magic  = DATA_BYTES_MAGIC;
	trailer.be_bbt_magic2	  = cpu_to_be32(BBT_MAGIC2);
	trailer.be_bbt_magic1	  = cpu_to_be32(BBT_MAGIC1);

	status = mtdnor->write( mtdnor, offset, sizeof(trailer),
				&num_written, (u_char *)&trailer);
	dbg_bbt_nor("  write_bbt_trailer_to_NOR:"
		    " # blks 0x%x, # data bytes 0x%x\n",
			trailer.be_num_blocks, trailer.be_num_data_bytes);
	return status;
}

/* @block is index of block in NAND
 * @mark  is value to store in block's spot
 */
static int mark_block_bad_in_NOR_bbt(struct mtd_info * mtdnor, 
				     struct mtd_info * mtdnand,
				     int block, int mark) 
{
	u_char cur_val;
        int    offset;
	int    status;
	size_t num_xferd;
//        struct nand_chip *this = mtdnand->priv;


	if (NULL == mtdnor)
		return -EINVAL;

	offset = bbt_offset_in_nor(mtdnor) + sizeof(struct bbt_header)
					   + (block/4);
	mtdnor->read(mtdnor, offset, 1, &num_xferd, &cur_val);
	cur_val &= ~((mark & 3) << ((block & 3)<<1));

	status = mtdnor->write( mtdnor, offset, 1, &num_xferd, &cur_val );

	dbg_bbt_nor("mark_block_bad_in_NOR_bbt wrote %02x"
		    " @ offset 0x%x, status %d\n",
			cur_val, offset, status);
	return status;
}


static void copy_bbt_from_ram_to_NOR(struct mtd_info * mtdnor, 
				     struct mtd_info * mtdnand) 
{  
	dbg_bbt_nor("copy_bbt_from_ram_to_NOR#1\n");
	if ((mtdnor != NULL) && (mtdnand != NULL) 
			     && (0 == erase_bbt_in_NOR(mtdnor))) 
	{
		dbg_bbt_nor("copy_bbt_from_ram_to_NOR#2\n");
		if (   (0 == write_bbt_header_to_NOR(mtdnor, mtdnand))
		    && (0 == write_bbt_to_NOR(mtdnor, mtdnand)))
		{
			dbg_bbt_nor("copy_bbt_from_ram_to_NOR#3\n");
			write_bbt_trailer_to_NOR(mtdnor, mtdnand);
		}
	}
	dbg_bbt_nor("copy_bbt_from_ram_to_NOR#4\n");
}
#endif	/* ifdef CONFIG_NAND_BBT_IN_NOR */


#if 1	/* 23mar12 Added for testing of NOR-resident bbt */
	/* erase the ram-resident bbt.
	 * if BBT_IN_NOR, erase the bbt sector of NOR
	 */
int force_erase_bbt(struct mtd_info *mtd) {
	struct nand_chip *chip = mtd->priv;
#ifdef CONFIG_NAND_BBT_IN_NOR
	int status = 0;
	struct mtd_info *mtdnor;

	mtdnor = get_nor_mtd();
	if (0 != (status = erase_bbt_in_NOR(mtdnor))) {
		printf("Failed (%d) to erase the bbt sector of NOR\n", status);
		return status;
	}
#endif
	if (chip->bbt) {
		kfree(chip->bbt);
		chip->bbt = NULL;
	}

	return chip->scan_bbt(mtd);
}

void force_good_in_ram_bbt(struct mtd_info *mtd, ulong offset) {
	struct nand_chip *chip = mtd->priv;
	int block;

	block = offset >> chip->bbt_erase_shift;	
	if (chip->bbt)
		chip->bbt[block >> 2] &= ~(0x03 << ((block & 0x03) << 1));
	else
		printf("force_good_in_ram_bbt() found no bbt in RAM\n");
}
 
#endif


/* Deal with Micron NANDs, where 'bad' is indicated by 0x00 in */
/* first byte of spare area of block's first page              */

/**
 * micron_nand_block_bad
 * @mtd:	MTD device structure
 * @ofs:	offset from device start
 * @getchip:	0, if the chip is already selected
 *
 * Returns nonzero (TRUE) if block is bad.
 *
 * This is derived from the default implementation (nand_block_bad() in
 * nand_base.c).
 * It considers a block to be bad if it finds a value of 0x00 in the first oob
 * byte of the first page of the block.
 */
int
micron_nand_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	int page, chipnr, res = 0;
	struct nand_chip *chip = mtd->priv;
	u8 bad;

#if 0	/* we hardly ever want/need this output */
	dbg_bbt_nor("mnbb: ofs 0x%x, sob 0x%x,"
		    " page 0x%x, getchip %d\n",
		(unsigned int)ofs, (unsigned int)(ofs & ~(mtd->erasesize -1)),
		(int)(ofs >> chip->page_shift) & chip->pagemask, getchip);
#endif
	page = (int)(ofs >> chip->page_shift) & chip->pagemask;

	if (getchip) {
		chipnr = (int)(ofs >> chip->chip_shift);

		nand_get_device(chip, mtd, FL_READING);

		/* Select the NAND device */
		chip->select_chip(mtd, chipnr);
	}
	chip->cmdfunc(mtd, NAND_CMD_READOOB, chip->badblockpos, page);
	bad = chip->read_byte(mtd);

	res = (bad == 0x00);
	if (getchip)
		nand_release_device(mtd);

	return res;
}


/* We added this special version for micron NANDs after discovering that
 * it's necessary to erase the block before trying to write 0x00 to its oob.
 */

int micron_nand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
#ifdef CONFIG_NAND_BBT_IN_NOR
	struct lf2000_nand_data * pnd;
#endif
	struct nand_chip *chip = mtd->priv;
	uint8_t buf = 0;
	int block;
	int ret;


	/* Get block number */
	block = (int)(ofs >> chip->bbt_erase_shift);
	printk(KERN_INFO "micron_nand_block_markbad: ofs 0x%x, block 0x%x\n",
		 (unsigned int)ofs, block);


#ifdef CONFIG_NAND_BBT_IN_NOR
	/* If NOR contains a bad block table for this NAND, mark the block
	 * bad in that table.
	 */
	pnd = (struct lf2000_nand_data *)chip->priv;
	if (pnd->bbt_in_nor) {
		ret = mark_block_bad_in_NOR_bbt(get_nor_mtd(), mtd, block, 1);
	}
#if 0	/* 23mar12 For initial testing also mark block bad in NAND */
	else 
#endif
#endif

	/* If the NAND contains a bad block table for itself, update it. */
	if (chip->options & NAND_USE_FLASH_BBT)
		ret = nand_update_bbt(mtd, ofs);
	else {
		struct erase_info ei;

		/* Must erase Micron NAND block before trying to mark it bad */
		memset(&ei, 0, sizeof(struct erase_info));

		ei.mtd      = mtd;
		ei.addr     = ofs;
		ei.len      = mtd->erasesize;
		ei.callback = NULL;
		ei.priv     = 0;
		ret	    = mtd->erase( mtd, &ei);
		if (ret) {
			printk(KERN_INFO "micron_nand_block_markbad():"
					 " mtd->erase() returned %d\n", ret);
		}

		/* Write 0 to the first byte of the oob of the first page. */
		chip->ops.len	  = chip->ops.ooblen = 1;
		chip->ops.datbuf  = NULL;
		chip->ops.oobbuf  = &buf;
		chip->ops.ooboffs = 0;

		nand_get_device(chip, mtd, FL_WRITING);

		ret = nand_do_write_oob(mtd, ofs, &chip->ops);

		nand_release_device(mtd);
	}

	/* If there's a RAM-resident back block table for this NAND, mark
	 * the block bad in that table.
	 * Do this last so it doesn't prevent erasure of the block.
	 */
	if (chip->bbt)
		chip->bbt[block >> 2] |= 0x01 << ((block & 0x03) << 1);

	if (!ret)
		mtd->ecc_stats.badblocks++;

	return ret;
}



/**
 * lf2000_nand_do_read_ops - Leapfrog replacement for nand_do_read_ops()
 *
 * @mtd:	MTD device structure
 * @from:	offset to read from
 * @ops:	oob ops structure
 *
 * Internal function. Called with chip held.
 *
 * Differences from nand_do_read_ops():
 *   code for profiling
 *   code for counting and timing read operations
 *   use of CONFIG_MTD_NAND_LF1000_MLC_SCRUB_THRESHOLD
 *   calls to printk() in order to report errors.
 */
#define MAX_RETRY_CYCLES	4
#define MAX_RETRIES_PER_CYCLE	8

static int lf2000_nand_do_read_ops(struct mtd_info *mtd, loff_t from,
			                       struct mtd_oob_ops *ops)
{
	int chipnr, page, realpage, col, bytes, aligned;
	struct nand_chip *chip = mtd->priv;
	struct mtd_ecc_stats stats;
	struct lf2000_nand_data * pnd;
	int blkcheck = (1 << (chip->phys_erase_shift - chip->page_shift)) - 1;
	int sndcmd = 1;
	int ret = 0;
	uint32_t readlen = ops->len;
	uint32_t oobreadlen = ops->ooblen;
	uint32_t max_oobsize = ops->mode == MTD_OOB_AUTO ?
		mtd->oobavail : mtd->oobsize;

	uint8_t *bufpoi, *oob, *buf;
    uint32_t numCorrected;
#ifdef CONFIG_MTD_NAND_LF2000_PROF
	int block;
#endif
	int	num_retries	 = 0;
	int	num_retry_cycles = 0;

	stats = mtd->ecc_stats;
	numCorrected = stats.corrected;

	chipnr = (int)(from >> chip->chip_shift);
	chip->select_chip(mtd, chipnr);

	realpage = (int)(from >> chip->page_shift);
	page = realpage & chip->pagemask;

	col = (int)(from & (mtd->writesize - 1));

	buf = ops->datbuf;
	oob = ops->oobbuf;

	while(1) {
		bytes = min(mtd->writesize - col, readlen);
		aligned = (bytes == mtd->writesize);

		/* Is the current page in the buffer ? */
		if (realpage != chip->pagebuf || oob) {
			bufpoi = aligned ? buf : chip->buffers->databuf;

#ifdef CONFIG_MTD_NAND_LF2000_PROF
			block = page >> (chip->phys_erase_shift - chip->page_shift); 
			if (aligned || oob || !NAND_SUBPAGE_READ(chip) 
						|| (ops->mode == MTD_OOB_RAW))
				nand_stats_accum (NS_READ_PAGE, 1, block);
#endif
			if (likely(sndcmd)) {
				chip->cmdfunc(mtd, NAND_CMD_READ0, 0x00, page);
				sndcmd = 0;
			}
			/* Now read the page into the buffer */
			if (unlikely(ops->mode == MTD_OOB_RAW))
				ret = chip->ecc.read_page_raw(mtd, chip, bufpoi, page);
			else if (!aligned && NAND_SUBPAGE_READ(chip) && !oob)
				ret = chip->ecc.read_subpage(mtd, chip, col, bytes, bufpoi);
			else {
#ifdef CONFIG_MTD_NAND_LF2000_PROF
				int block;
				block = page >> (chip->phys_erase_shift 
						  - chip->page_shift); 
				nand_stats_accum (NS_READ_AND_ECC, 1, block);
#endif
#ifdef TIME_NAND_READ_ENTIRE
                		timer_start();
#endif
				ret = chip->ecc.read_page(mtd, chip, bufpoi, page);
#ifdef TIME_NAND_READ_ENTIRE
				update_nand_read_time_info();
#endif
#ifdef CONFIG_MTD_NAND_LF2000_PROF
				nand_stats_accum (NS_READ_AND_ECC, 0, block);
#endif
           		}
#ifdef CONFIG_MTD_NAND_LF2000_PROF
			if (aligned || oob || !NAND_SUBPAGE_READ(chip) 
						|| (ops->mode == MTD_OOB_RAW))
				nand_stats_accum (NS_READ_PAGE, 0, block);
#endif
			if (ret < 0) {
#if 0	/* 3may12 Disabled to reduce the amount of serial output */
	/* Ought to check earlier for Verbose/non-verbose, and output only
	 * if Verbose is enabled 
	 */
				printk(KERN_INFO "Error reading %d bytes from page %d\n", 
						bytes, realpage);
#endif
				if (++num_retries < MAX_RETRIES_PER_CYCLE) {
					sndcmd = 1;
					continue;
				}
				else if (++num_retry_cycles < MAX_RETRY_CYCLES){
					num_retries = 0;
					sndcmd = 1;
					continue;
				}
				else {
					printk(KERN_ERR "All retries failed"
						" when reading %d bytes "
						"from page %d\n",
						bytes, realpage);
				break;
				}
            		}
            		else if ( mtd->ecc_stats.corrected != numCorrected ) {
               			numCorrected = mtd->ecc_stats.corrected;
#if 0	/* 3may12 Disabled to reduce the amount of serial output */
	/* Ought to check earlier for Verbose/non-verbose, and output only
	 * if Verbose is enabled 
	 */
#ifndef NAND_ACCESS_TIMING	// we don't want to time serial output
				printk(KERN_INFO "Corrected error while reading %d bytes "
						 "at offset %d from page %d\n",
						   bytes, col, realpage);
#endif
#endif
            		}
			if ((num_retries > 0) || (num_retry_cycles > 0)) {
#if 0	/* 3may12 Disabled to reduce the amount of serial output */
	/* Ought to check earlier for Verbose/non-verbose, and output only
	 * if Verbose is enabled 
	 */
				printk(KERN_INFO "Success on cycle %d, retry %d\n",
					num_retry_cycles, num_retries);
#endif
				num_retries	 = 0;
				num_retry_cycles = 0;
			}

			/* Transfer not aligned data */
			if (!aligned) {
				if (!NAND_SUBPAGE_READ(chip) && !oob &&
				    !(mtd->ecc_stats.failed - stats.failed))
					chip->pagebuf = realpage;
				memcpy(buf, chip->buffers->databuf + col, bytes);
			}

			buf += bytes;

			if (unlikely(oob)) {

				int toread = min(oobreadlen, max_oobsize);

				if (toread) {
					oob = nand_transfer_oob(chip, oob, ops, toread);
					oobreadlen -= toread;
				}
			}

			if (!(chip->options & NAND_NO_READRDY)) {
				/*
				 * Apply delay or wait for ready/busy pin. Do
				 * this before the AUTOINCR check, so no
				 * problems arise if a chip which does auto
				 * increment is marked as NOAUTOINCR by the
				 * board driver.
				 */
				if (!chip->dev_ready)
					udelay(chip->chip_delay);
				else
					nand_wait_ready(mtd);
			}
		} else {
			memcpy(buf, chip->buffers->databuf + col, bytes);
			buf += bytes;
		}

		readlen -= bytes;

		if (!readlen)
			break;

		/* For subsequent reads align to page boundary. */
		col = 0;
		/* Increment page address */
		realpage++;

		page = realpage & chip->pagemask;
		/* Check, if we cross a chip boundary */
		if (!page) {
			chipnr++;
			chip->select_chip(mtd, -1);
			chip->select_chip(mtd, chipnr);
		}

		/* Check, if the chip supports auto page increment
		 * or if we have hit a block boundary.
		 */
		if (!NAND_CANAUTOINCR(chip) || !(page & blkcheck))
			sndcmd = 1;
	}

	ops->retlen = ops->len - (size_t) readlen;
	if (oob)
		ops->oobretlen = ops->ooblen - oobreadlen;

	if (ret)
		return ret;

	if (mtd->ecc_stats.failed - stats.failed) {
#if 0	/* 3may12 Disabled to reduce the amount of serial output */
	/* Ought to check earlier for Verbose/non-verbose, and output only
	 * if Verbose is enabled 
	 */
        	printk(KERN_INFO "nand_do_read_ops: read ok after "
			 "uncorrectable error:\n"
                         "  from %08x %08x; realpage %x; page %x; readlen %d\n",	
                (uint32_t)(from >> 32), (uint32_t)from, 
                realpage, page, ops->len);
#endif
		return -EUCLEAN;
	}

	/* 	chip->priv is the address of a struct lf2000_nand_data which
	 *	contains a pointer to a const struct tag_bch_info whose 't'
	 *  member is the max number of correctable errors.
 	 *  Return -EUCLEAN if the newly corrected # is > 't' - 5.
	 *	(The newly corrected # is the max number of correctable errors
	 *	 that were found and corrected in any ecc-chunk while the page
	 *	 was being read.)
	 *
	 *	If the pointer to a tag_bch_info is NULL (1-bit ECC is used for
	 *	the NAND), return -EUCLEAN if any bitflip was found
	 *
	 * NOTE: this routine will be called for only whose NANDs on which we're
	 *		 using hw ecc, so we don't expect to find a NULL pointer to a
	 *		 tag_bch_info.
	 */
	pnd = (struct lf2000_nand_data *)chip->priv;
	if ((pnd != NULL) && (pnd->bch_info != NULL)) {
		return  (mtd->ecc_stats.corrected - stats.corrected 
					> pnd->bch_info->bch_var_t - 5) 
					? -EUCLEAN 
					: 0;
	}
	else {
		return  mtd->ecc_stats.corrected - stats.corrected ? -EUCLEAN : 0;
	}
}



/**
 * lf2000_nand_read - Leapfrog's replacement for nand_read()
 * @mtd:	MTD device structure
 * @from:	offset to read from
 * @len:	number of bytes to read
 * @retlen:	pointer to variable to store the number of read bytes
 * @buf:	the databuffer to put data
 *
 * Get hold of the chip and call nand_do_read
 *
 * Leapfrog has a special version of this function in order to call
 * lf2000_nand_do_read_ops() instead of the standard nand_do_read_ops().
 * Except for that call, this function is identical to nand_do_read_ops().
 */
int lf2000_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
		                    size_t *retlen, uint8_t *buf)
{
	struct nand_chip *chip = mtd->priv;
	int ret;

printf("lf2000_nand_read(%d)\n", len);
	/* Do not allow reads past end of device */
	if ((from + len) > mtd->size)
		return -EINVAL;
	if (!len)
		return 0;

	nand_get_device(chip, mtd, FL_READING);

	chip->ops.len = len;
	chip->ops.datbuf = buf;
	chip->ops.oobbuf = NULL;

	ret = lf2000_nand_do_read_ops(mtd, from, &chip->ops);

	*retlen = chip->ops.retlen;

	nand_release_device(mtd);

	return ret;
}

/**
 * lf2000_nand_read_oob - Leapfrog replacement for nand_read_oob().
 * @mtd:	MTD device structure
 * @from:	offset to read from
 * @ops:	oob operation description structure
 *
 * NAND read data and/or out-of-band data
 *
 * This function differs from nand_read_oob() only by calling 
 * lf2000_nand_do_read_ops() instead of nand_do_read_ops().
 */
int lf2000_nand_read_oob(struct mtd_info *mtd, loff_t from,
		            	 struct mtd_oob_ops *ops)
{
	struct nand_chip *chip = mtd->priv;
	int ret = -ENOTSUPP;

	ops->retlen = 0;

	/* Do not allow reads past end of device */
	if (ops->datbuf && (from + ops->len) > mtd->size) {
		MTDDEBUG(MTD_DEBUG_LEVEL0, "nand_read_oob: "
		      "Attempt read beyond end of device\n");
		return -EINVAL;
	}
	nand_get_device(chip, mtd, FL_READING);

	switch(ops->mode) {
	case MTD_OOB_PLACE:
	case MTD_OOB_AUTO:
	case MTD_OOB_RAW:
		break;
	default:
		goto out;
	}

	if (!ops->datbuf) {
		ret = nand_do_read_oob(mtd, from, ops);
	}
	else {
		ret = lf2000_nand_do_read_ops(mtd, from, ops);
	}
 out:
	nand_release_device(mtd);
	return ret;
}



/**
 * lf2000_nand_write_page - write one page
 * Leapfrog's replacement for nand_write_page()
 * @mtd:	MTD device structure
 * @chip:	NAND chip descriptor
 * @buf:	the data to write
 * @page:	page number to write
 * @cached:	cached programming
 * @raw:	use _raw version of write_page
 */
static int lf2000_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			   const uint8_t *buf, int page, int cached, int raw)
{
	int status;

#ifdef CONFIG_MTD_NAND_LF2000_PROF
	int block;
	block = page >> (chip->phys_erase_shift - chip->page_shift); 
	nand_stats_accum (NS_WRITE, 1, block);
#endif
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);

	if (unlikely(raw))
		chip->ecc.write_page_raw(mtd, chip, buf);
	else
		chip->ecc.write_page(mtd, chip, buf);

	/*
	 * Cached progamming disabled for now, Not sure if its worth the
	 * trouble. The speed gain is not very impressive. (2.3->2.6Mib/s)
	 */
	cached = 0;

	if (!cached || !(chip->options & NAND_CACHEPRG)) {

		chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
#ifdef CONFIG_MTD_NAND_LF2000_PROF
		nand_stats_accum (NS_WRITE, 0, block);
#endif
		/*
		 * See if operation failed and additional status checks are
		 * available
		 */
		if ((status & NAND_STATUS_FAIL) && (chip->errstat)) 
		{
			status = chip->errstat(mtd, chip, FL_WRITING, status,
					       page);
			printk(KERN_ERR "lf2000_nand_write_page(): after call "
				"to chip->errstat(): status 0x%x\n", status);
		}

		if (status & NAND_STATUS_FAIL)
		{
			printk(KERN_ERR "lf2000_nand_write_page() returns -EIO\n");
			return -EIO;
		}
	} else {
		chip->cmdfunc(mtd, NAND_CMD_CACHEDPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
#ifdef CONFIG_MTD_NAND_LF2000_PROF
		nand_stats_accum (NS_WRITE, 0, block);
#endif
	}

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	/* Send command to read back the data */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);

	if (chip->verify_buf(mtd, buf, mtd->writesize))
		return -EIO;
#endif
	return 0;
}


static void decode_samsung_ecc_blk_oobsize(struct mtd_info  * mtd, 
					   struct nand_chip * chip, 
					   int	  extid, 
					   int	  ecc_level)
{
	/* Calc oobsize */
	/* NOTE: oobsize is indicated by bits 6, 3, and 2 
	 * of the 4th ID byte:
	 * bits 7654.3210   HEX	  Description
	 *		x0xx.00xx  0x00 : reserved
	 *		x0xx.01xx  0x04 : 128 bytes
	 *		x0xx.10xx  0x08 : 218 bytes
	 *		x0xx.11xx  0x0C : 400 bytes
	 *		x1xx.00xx  0x40 : 436 bytes
	 *		x1xx.01xx  0x44 : 512 bytes
	 *		x1xx.10xx  0x48 : 640 bytes
	 *		x1xx.11xx  0x4C : reserved
	 *
	 * clear the unused bits, leaving only 6, 3, and 2
	 */
	switch (extid & 0x4C) {
	case 0x04:	mtd->oobsize = 128;	break;
	case 0x08: 	mtd->oobsize = 218;	break;
	case 0x0C:	mtd->oobsize = 400;	break;
	case 0x40:	mtd->oobsize = 436;	break;
	case 0x44:	mtd->oobsize = 512;	break;
	case 0x48:	mtd->oobsize = 640;	break;
	case 0x00:	/* reserved */
	case 0x4C:	/* reserved */
	default:	mtd->oobsize = 0;	break;
	}
	/* Calc blocksize 
	 * NOTE: blocksize is indicated by bits 7, 5, and 4
	 * of the 4th ID byte:
	 *				000: 128K
	 *				001: 256K
	 *				010: 512K
	 *				011: 1M
	 * 100,101,110, 111: reserved
	 * This code treats all the reserved values as though their
	 * msb is 0.  Not exactly right, but what else to do?
	 */
	mtd->erasesize = (128 * 1024) << ((extid >> 4) & 0x03);

	/* Get the required ecc strength */
	switch (ecc_level) {
	case 0:	// 1 bit / 512 bytes
		// TODO: FIXME: do we need to have a 
		// 		nand_sw_ecc_init(chip) routine
		// to init all the function pointers and other 
		// fields of chip?  
		// NOTE: we'll probably never come through
		// here for a Samsung MLC NAND.
		chip->ecc.mode = NAND_ECC_SOFT;
		printk(KERN_INFO "NAND ecc: Software \n");
		break;
	case 1:		// 2 bits / 512 bytes
		nand_hw_ecc_init(mtd, 4);
		printk(KERN_INFO "NAND ecc: 2/4-bit HW\n");
		break;
	case 2:		// 4 bits / 512 bytes
		nand_hw_ecc_init(mtd, 4);
		printk(KERN_INFO "NAND ecc: 4-bit HW\n");
		break;
	default:	
		// TODO: FIXME: decide what to do for default case
		// 8-bit ecc uses 13 ecc bytes per 512 bytes of data.
		// If the spare area has at least 16 bytes per 512
		// bytes of data, the ecc bytes will fit in the oob.
	case 3:	// 8 bits / 512 bytes
		nand_hw_ecc_init(mtd, 8);
		printk(KERN_INFO "NAND ecc: 8-bit HW\n");
		break;
	case 4:	// 16 bits / 512 bytes
		nand_hw_ecc_init(mtd, 16);
		printk(KERN_INFO "NAND ecc: 16-bit HW\n");
		break;
	case 5:	// 24 bits / 1024 bytes
		nand_hw_ecc_init(mtd, 24);
		printk(KERN_INFO "NAND ecc: 24-bit HW\n");
		break;
	}
}


static void decode_hynix_ecc_blk_oobsize(struct mtd_info  * mtd,
					 struct nand_chip * chip, 
					 int	extid, 
					 int	ecc_level)
{
	/* Calc oobsize */
	/* NOTE: oobsize is indicated by bits 6, 3, and 2 
	 * of the 4th ID byte:
	 * (see the Hynix H27UAG8T2B datasheet, p20)
	 * bits 7654.3210   HEX	  Description
	 *	x0xx.00xx  0x00 : 128 bytes
	 *	x0xx.01xx  0x04 : 224 bytes
	 *	x0xx.10xx  0x08 : 448 bytes
	 *	x0xx.11xx  0x0C : reserved
	 *	x1xx.00xx  0x40 : reserved
	 *	x1xx.01xx  0x44 : reserved
	 *	x1xx.10xx  0x48 : reserved
	 *	x1xx.11xx  0x4C : reserved
	 *
	 * clear the unused bits, leaving only 6, 3, and 2
	 */
	switch (extid & 0x4C) {
	case 0x00:	mtd->oobsize = 128;	break;
	case 0x04:	mtd->oobsize = 224;	break;
	case 0x08: 	mtd->oobsize = 448;	break;
	case 0x0C:	mtd->oobsize = 64;	break;
	case 0x40:	mtd->oobsize = 32;	break;
	case 0x44:	mtd->oobsize = 16;	break;
	case 0x48:	mtd->oobsize = 640;	break;
	case 0x4C:	/* reserved */
	default:	mtd->oobsize = 0;	break;
	}
	/* Calc blocksize */
	/* Mask out all bits except 7, 5, and 4 */
	switch (extid & 0xB0) {
	case 0x00: mtd->erasesize =  128 * 1024; break;
	case 0x10: mtd->erasesize =  256 * 1024; break;
	case 0x20: mtd->erasesize =  512 * 1024; break;
	case 0x30: mtd->erasesize =  768 * 1024; break;
	case 0x80: mtd->erasesize = 1024 * 1024; break;
	case 0x90: mtd->erasesize = 2048 * 1024; break;
	case 0xA0: /* reserved */
	case 0xB0: /* reserved */
		   mtd->erasesize = 0;	break;
	}
	/* Get the required ecc strength */
	switch (ecc_level) {
	case 0:		// 1 bit / 512 bytes
		chip->ecc.mode = NAND_ECC_SOFT;
		printk(KERN_INFO "NAND ecc: Software \n");
		break;
	case 1:		// 2 bits / 512 bytes
		nand_hw_ecc_init(mtd, 4);
		printk(KERN_INFO "NAND ecc: 2/4-bit HW\n");
		break;
	case 2:		// 4 bits / 512 bytes
		nand_hw_ecc_init(mtd, 4);
		printk(KERN_INFO "NAND ecc: 4-bit HW\n");
		break;
	case 3:		// 8 bits / 512 bytes
		nand_hw_ecc_init(mtd, 8);
		printk(KERN_INFO "NAND ecc: 8-bit HW\n");
		break;
	case 4:		// 16 bits / 512 bytes
		nand_hw_ecc_init(mtd, 16);
		printk(KERN_INFO "NAND ecc: 16-bit HW\n");
		break;
	case 5:		// 24 bits / 2048 bytes
	case 6:		// 24 bits / 1024 bytes
	default:	/* reserved; default to 24 bits / 1KB */
		nand_hw_ecc_init(mtd, 24);
		printk(KERN_INFO "NAND ecc: 24-bit HW\n");
		break;
	}
}


/* This routine examines the nand's ID bytes and extracts from them and saves
 * lots of information about the nand's geometry and the amount of error
 * correction that it needs.
 */
int lf2000_nand_init_size(struct mtd_info  * mtd, 
			  struct nand_chip * chip,
			  u8		   * id_data)
{
	int extid;
	int busw;


		/* The 3rd id byte holds MLC / multichip data */
	chip->cellinfo = id_data[2];
		/* The 4th id byte has sizes of page, oob, and eraseblock */
	extid = id_data[3];

		/* Leapfrog uses at least 2 NAND devices that emit 6 ID bytes
		 * which have non-traditional field definitions.  
		 * One of these is a Samsung K9GAG08U0E.
 		 * The other is a Hynix H27UAG8T2B.
		 *
		 * We first check if the 3rd ID byte indicates that a cell contains
		 * more than 1 bit (2 levels), if the 8 ID bytes in id_data[] start to
		 * repeat after 6 bytes (i.e., if id_data[0] == id_data[6] and
		 * id_data[1] == id_data[7]).
		 * If not, or if the manufacturer is not Samsung or Hynix, we interpret
		 * the ID bytes in the traditional way.
		 * If they do, we interpret a Samsung nand's fields one way and Hynix
		 * another way.
		 */
	if (   (chip->cellinfo & NAND_CI_CELLTYPE_MSK) 
		&& (id_data[5] != 0x00) 
		&& ((id_data[0] == id_data[6]) && (id_data[1] == id_data[7]))
		&& (   (id_data[0] == NAND_MFR_SAMSUNG)
			|| (id_data[0] == NAND_MFR_HYNIX)) )
	{
		int ecc_level;

			/* no subpage writes on these MLC NANDs */
		chip->options	   |= NAND_NO_SUBPAGE_WRITE;
		chip->block_bad	    = nand_block_bad_first_or_last;
		chip->block_markbad = nand_block_markbad_first_and_last;
		chip->scan_bbt	    = lf_nand_bbt;

		/* The 5th ecc byte of these devices indicates required ECC level */
		ecc_level = (id_data[4] >> 4) & 0x07;

			/* Calc pagesize */
		mtd->writesize = 2048 << (extid & 0x03);
		if (id_data[0] == NAND_MFR_SAMSUNG)
		{
			decode_samsung_ecc_blk_oobsize(mtd,chip,extid,ecc_level);
		}
		else {	/* Hynix */
			decode_hynix_ecc_blk_oobsize(mtd,chip,extid,ecc_level);
		}
		busw = 0;
	} 
	// NOTE: If we need to deal with other types of non-traditional NANDs,
	// we can insert code here to check for them and to deal with them.

	else {	/* Sometimes we read invalid ID bytes; usually the first one 
		 * is not a recognized manufacturer code.  Do the processing
		 * that's in nand_get_flash_type() only if the first ID byte
		 * is a recognzied mfr code.  This might be wrong sometimes,
		 * but is will catch many of the errors.
		 */
		switch (id_data[0]) {
		case NAND_MFR_TOSHIBA:
		case NAND_MFR_SAMSUNG:
		case NAND_MFR_FUJITSU:
		case NAND_MFR_NATIONAL:
		case NAND_MFR_RENESAS:
		case NAND_MFR_STMICRO:
		case NAND_MFR_HYNIX:
		case NAND_MFR_MICRON:
		case NAND_MFR_AMD:
			/* This processing is identical to code in 
			 * nand_get_flash_type().
			 * First calc pagesize */
		mtd->writesize = 1024 << (extid & 0x03);
		extid >>= 2;
			/* Calc oobsize */
		mtd->oobsize = (8 << (extid & 0x01)) * (mtd->writesize >> 9);
		extid >>= 2;
				/* Calc blocksize (multiples of 64KiB) */
		mtd->erasesize = (64 * 1024) << (extid & 0x03);
		extid >>= 2;
			/* Get buswidth information */
		busw = (extid & 0x01) ? NAND_BUSWIDTH_16 : 0;
			break;
		default:	/* Force an error for unexpected type of NAND */
			printk(KERN_INFO "Non-Samsung, non-Hynix, non-ONFI unit\n");
			busw = -1; /* indicate an error */
			break;
		}
	}
	return busw;
}


#endif	/* LF2000 */
