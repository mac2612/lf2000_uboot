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
#if (1)
#include <common.h>
#include <asm/errno.h>
#include <nand.h>
#include <platform.h>
#else
#include <linux/kernel.h>
#include <linux/mtd/nand.h>
#include <mach/platform.h>
#endif

#include "nand_hw_ecc.h"

#if	(0)
#define DBGOUT(msg...)		{ printk(KERN_INFO "ecc: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define ERROUT(msg...)		{ 					\
		printk(KERN_ERR "ERROR: %s, %s line %d: \n",		\
			__FILE__, __FUNCTION__, __LINE__),	\
		printk(KERN_ERR msg); }

static struct nand_ecclayout nx_nand_oob = {
	.eccbytes 	=   0,
	.eccpos 	= { 0, },
	.oobfree 	= { {.offset = 0, .length = 0} }
};

/*------------------------------------------------------------------------------
 * u-boot nand hw ecc interface
 */
static int nand_hw_ecc_read_page(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int page)
{
	int i, n, ret = 0;

	int eccsteps = chip->ecc.steps;
	int eccbytes = chip->ecc.bytes;
	int eccsize  = chip->ecc.size;
	int ecctotal = chip->ecc.total;

	uint8_t   ecc_data[((CFG_NAND_ECC_BYTES+3)/4) * 4];
	uint8_t  *ecc_code = chip->buffers->ecccode;
	uint8_t  *p = buf;
	uint32_t *oobpoi = (uint32_t *)chip->oob_poi;

	int err = 0;
	int	o_syn[CFG_NAND_ECC_MODE], e_pos[CFG_NAND_ECC_MODE];
	uint32_t *e_dat;

	DBGOUT("%s, page=%d, ecc mode=%d, bytes=%d, total=%d\n",
		__func__, page, CFG_NAND_ECC_MODE, eccbytes, ecctotal);

	/* read oob */
	chip->ecc.read_oob(mtd, chip, page, 1);
	chip->waitfunc(mtd, chip);

	/* get ecc code from oob */
	for (i = 0, n = 2; i < ecctotal; i++, n++) {
		if (5 == n) n += 1;
		ecc_code[i] = chip->oob_poi[n];
	}

	/* read data */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0x00, page);

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		/* get ecc data per 512 byte */
		memcpy(ecc_data, &ecc_code[i], CFG_NAND_ECC_BYTES);

		/* write ecc data to orignal ecc register */
		NX_NAND_SetResetECC(CFG_NAND_ECC_MODE);
		NX_NAND_SetOriECC((uint32_t*)ecc_data, CFG_NAND_ECC_MODE);

		/* read data */
		chip->read_buf(mtd, p, eccsize);

		/* wait decoding  and error correct */
		NX_NAND_WaitForDecoding();

		if (NX_NAND_GetErrorStatus()) {
#if 1
			/* check erase status */
			for (n=0; mtd->oobsize/4 > n; n++) {
				if (-1 != oobpoi[n]) break;
			}
			if (mtd->oobsize/4 == n)
				continue;
#endif
			NX_NAND_GetOddSyndrome(&o_syn[0]);
			ret = NX_NAND_GetErrorLocation(&o_syn[0], &e_pos[0], &err);

			if (0 > ret) {
				ERROUT("nand ecc detect errors, can't correct (page:%d, err:%d)\n",
					page, err);
				ret = -EIO;
				break;
			} else {
				printk(KERN_INFO "nand page %d, ecc %2d err, bit: ", page, err);
				for (n=0 ; err > n; n++) {
					printk(KERN_INFO "%d", e_pos[n]);
					if (n != err-1)
						printk(KERN_INFO ", ");

					if (4096 > e_pos[n]) {
						e_dat = (uint32_t*)p;
						e_dat[e_pos[n] / 32] ^= 1U<<(e_pos[n] % 32);
					}
				}
				ret = err > CFG_NAND_ECC_LIMIT ? -EBADMSG: 0;
				printk(KERN_INFO "\n");
			}
		}
	}

	DBGOUT("DONE %s, ret=%d\n", __func__, ret);
	mtd->ecc_stats.failed    = ret;
	mtd->ecc_stats.corrected = ret;
	return ret;
}

static void nand_hw_ecc_write_page(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf)
{
	int i, n;
	int eccsteps = chip->ecc.steps;
	int eccbytes = chip->ecc.bytes;
	int eccsize  = chip->ecc.size;
	int ecctotal = chip->ecc.total;

	uint8_t *p = (uint8_t *)buf;
	uint8_t  ecc_data[((CFG_NAND_ECC_BYTES+3)/4) * 4];
	uint8_t *ecc_code = chip->buffers->ecccode;

	DBGOUT("%s\n", __func__);

	/* write data and get ecc */
	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {

		NX_NAND_SetResetECC(CFG_NAND_ECC_MODE);

		/* write page */
		chip->write_buf(mtd, p, eccsize);

		/* get ecc code from ecc register */
		NX_NAND_WaitForEncoding();
		NX_NAND_GetGenECC((uint32_t *)ecc_data, CFG_NAND_ECC_MODE);

		memcpy(&ecc_code[i], ecc_data, CFG_NAND_ECC_BYTES);
	}
	memset(chip->oob_poi, 0xff, mtd->oobsize);

	/* set oob with ecc */
	for (i = 0, n = 2; i < ecctotal; i++, n++) {
		if (5 == n) n += 1;
		chip->oob_poi[n] = ecc_code[i];
	}
	/* write oob */
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
}

static int nand_hw_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			   const uint8_t *buf, int page, int cached, int raw)
{
	int i, n, status;
	int eccsteps = chip->ecc.steps;
	int eccbytes = chip->ecc.bytes;
	int eccsize  = chip->ecc.size;
	int ecctotal = chip->ecc.total;

	uint8_t *p = (uint8_t *)buf;
	uint8_t  ecc_data[((CFG_NAND_ECC_BYTES+3)/4) * 4];
	uint8_t *ecc_code = chip->buffers->ecccode;

#if defined (CONFIG_MTD_NAND_VERIFY_WRITE)
	int err, ret = 0;
	int	o_syn[CFG_NAND_ECC_MODE], e_pos[CFG_NAND_ECC_MODE];
#endif
	DBGOUT("%s, page=%d, raw=%d\n", __func__, page, raw);

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);

	if (unlikely(raw)) {
		chip->ecc.write_page_raw(mtd, chip, buf);
		/* write done */
		chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
		if (status & NAND_STATUS_FAIL)
			return -EIO;

	} else {

		/* write data and get ecc */
		for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {

			NX_NAND_SetResetECC(CFG_NAND_ECC_MODE);

			/* write page */
			chip->write_buf(mtd, p, eccsize);

			/* get ecc code from ecc register */
			NX_NAND_WaitForEncoding();
			NX_NAND_GetGenECC((uint32_t *)ecc_data, CFG_NAND_ECC_MODE);

			memcpy(&ecc_code[i], ecc_data, CFG_NAND_ECC_BYTES);
		}

		memset(chip->oob_poi, 0xff, mtd->oobsize);
		/* set oob with ecc */
		for (i = 0, n = 2; i < ecctotal; i++, n++) {
			if (5 == n) n += 1;
			chip->oob_poi[n] = ecc_code[i];
		}

		/* write oob */
		chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);

		/* write done */
		chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
		if (status & NAND_STATUS_FAIL)
			return -EIO;

#if defined (CONFIG_MTD_NAND_VERIFY_WRITE)
		eccsteps = chip->ecc.steps;
		p = (uint8_t *)buf;

		/* read data for verify */
 		chip->cmdfunc(mtd, NAND_CMD_READ0, 0x00, page);

		for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
			/* get ecc data per 512 byte */
			memcpy(ecc_data, &ecc_code[i], CFG_NAND_ECC_BYTES);

			/* set orignal ecc register */
			NX_NAND_SetResetECC(CFG_NAND_ECC_MODE);
			NX_NAND_SetOriECC((uint32_t *)ecc_data, CFG_NAND_ECC_MODE);

			/* read data */
			chip->read_buf(mtd, p, eccsize);

			/* wait decoding  and error correct */
			NX_NAND_WaitForDecoding();

			if (NX_NAND_GetErrorStatus()) {
				NX_NAND_GetOddSyndrome(&o_syn[0]);
				ret = NX_NAND_GetErrorLocation(&o_syn[0], &e_pos[0], &err);
				if (0   > ret ||
					err > CFG_NAND_ECC_LIMIT) {
					ERROUT("nand ecc detect errors, can't correct (page:%d, err:%d)\n",
						page, err);
					return -EIO;
				}
			}
		}
#endif /* CONFIG_MTD_NAND_VERIFY_WRITE */
	}

	/* crash bit */
#if 0
	/* first page each block */
	if (!((page*mtd->writesize)&(mtd->erasesize-1))) {

		int c_bit = 6;	/* crash bit num */
		uint32_t o_dat  = *(uint32_t*)buf;
		uint32_t z_val  = o_dat & ((1 << (32-c_bit))-1);
		uint32_t c_val  = o_dat >> (32-c_bit);

		/* crash data */
		*(uint32_t*)buf = z_val | (~c_val << (32-c_bit));

		printk(KERN_INFO "------------------------------------------------\n");
		printk(KERN_INFO "CRASH %d bit page %d buf 0x%08x=0x%08x->0x%08x\n",
				c_bit, page, (uint32_t)buf, o_dat, *(uint32_t*)buf);
		printk(KERN_INFO "------------------------------------------------\n");

		/* write only data */
		chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);
  		chip->write_buf(mtd, buf, mtd->writesize);
		chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
		chip->waitfunc(mtd, chip);
	}
#endif
	return 0;
}

static int nand_hw_wait(struct mtd_info *mtd, struct nand_chip *this)
{
	unsigned long	timeo;
	int state = this->state;

	if (state == FL_ERASING)
		timeo = (CONFIG_SYS_HZ * 400) / 1000;
	else
		timeo = (CONFIG_SYS_HZ * 20) / 1000;

	if ((state == FL_ERASING) && (this->options & NAND_IS_AND))
		this->cmdfunc(mtd, NAND_CMD_STATUS_MULTI, -1, -1);
	else
		this->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);

	reset_timer();

	while (1) {
		if (get_timer(0) > timeo) {
			printf("Timeout!");
			return 0x01;
		}

		if (this->dev_ready) {
			if (this->dev_ready(mtd))
				break;

			if (this->read_byte(mtd) & NAND_STATUS_READY)
				break;
		}
	}

	return this->read_byte(mtd);
}

int nx_nand_ecc_init(struct nand_chip *chip)
{
	DBGOUT("%s\n", __func__);

	/*
	 * ecc.bytes:
	 *  4 bit ecc need  52 bit (6.5B) ecc code
	 *  8 bit ecc need 104 bit (13 B) ecc code
	 * 16 bit ecc need 208 bit (26 B) ecc code
	 * 24 bit ecc need 336 bit (42 B) ecc code
	 */
	chip->ecc.mode 			= NAND_ECC_HW;
	chip->ecc.read_page 	= nand_hw_ecc_read_page;
	chip->ecc.write_page 	= nand_hw_ecc_write_page;
	chip->ecc.size 			= CFG_NAND_ECC_SIZE;	/* per 512 byte */
	chip->ecc.bytes 		= CFG_NAND_ECC_BYTES;	/* 8 bit ecc */
	chip->write_page		= nand_hw_write_page;
	chip->ecc.layout		= &nx_nand_oob;
	chip->waitfunc			= nand_hw_wait;

	NX_NAND_CreateLookupTable();
	return 0;
}


