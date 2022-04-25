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
#include <linux/mtd/mtd.h>
#include <malloc.h>
#include <command.h>
#include <asm/errno.h>
#include <jffs2/jffs2.h>
#include <nand.h>
#include <platform.h>

#include "nand_hw_ecc.h"

//
//	--------------------------------------------
//	16 bit ECC
//	--------------------------------------------
// 	2K page: data page nr = 60, ecc page nr = 4
// 	4K page: data page nr =120, ecc page nr = 8
//	--------------------------------------------
//
// 	24 bit ECC
//	--------------------------------------------
// 	2K page: data page nr = 58, ecc page nr = 6
// 	4K page: data page nr =116, ecc page nr =12
//	--------------------------------------------
//
// * 24 bit ECC mod
// * mlc nand block map
//
//	--------------------------------- ecc page nr
//	|	DATA ECC page...		|
//	|---------------------------|
//	| 28B | 28B | 28B | ...		|
//	-----------------------------
//	|	DATA ECC page 0			|
//	|---------------------------|
//	| 28B | 28B | 28B | ...		|
//	|---------------------------|---- 1024
//	| info(512B)| info ecc (28B)|
//	--------------------------------- data page nr
//	|	DATA page ...			|
//	-----------------------------
//	|	DATA page 1				|
//	|---------------------------|
//	| 512B | 512B | 512 B | ...	|
//	-----------------------------
//	|	DATA page 0				|
//	|---------------------------|
//	| 512B | 512B | 512 B | ...	|
//	--------------------------------- block offset
//
#if	(0)
#define DBGOUT(msg...)		{ printf("mlc: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define ERROUT(msg...)		{ 					\
		printf("ERROR: %s, %s line %d: \n",		\
			__FILE__, __FUNCTION__, __LINE__),	\
		printf(msg); }

#define	__ALIGN(v, d)		((((uint)v + (uint)d -1)/(uint)d)*d)
#define	__DIVID(v, d)		((((uint)v + (uint)d -1)/(uint)d))

#if defined(CONFIG_CMD_NAND_MLC)

#define	MLC_ECC_MODE			16	/* 16, 24 */
#define	MLC_ECC_LIMIT			14
#define	MLC_ECC_SIZE			512
#define	MLC_ECC_BYTES			26	/* 16bit ecc= 26, 24bit ecc=42 */
#define	MLC_ECC_ALIGN			__ALIGN(MLC_ECC_BYTES, 4)	/* 28 */

#define MLC_ECC_MAGIC			(('M'<<24)|('L'<<16)|('C'<<8)|(0<<0))

struct data_info {
	unsigned int magic;
	unsigned int total;			/* total image size */
	unsigned int index;			/* current image data index in block */
	unsigned int length;		/* current image data length in block */
	unsigned int page_nr;		/* image page count per block */
};

struct mlc_page {
	unsigned int data_page_nr;		/* data page number per block */
	unsigned int ecc_page_nr;		/* ecc page number for data per block */
};

static struct mlc_page mlc_page = {0, };
//static int 	  test_bad_idx = -1;
/*------------------------------------------------------------------------------
 * command util
 */
#include <asm/types.h>
int mtdparts_init(void);
int find_dev_and_part(const char *id, struct mtd_device **dev,
		      u8 *part_num, struct part_info **part);

static inline int str2long(char *p, loff_t *num)
{
	char *endptr;
	*num = simple_strtoull(p, &endptr, 16);
	return (*p != '\0' && *endptr == '\0') ? 1 : 0;
}

static int
arg_off_size(int argc, char *argv[], nand_info_t *nand, loff_t *off, loff_t *size)
{
	int idx = nand_curr_device;
#if defined(CONFIG_CMD_MTDPARTS)
	struct mtd_device *dev;
	struct part_info *part;
	u8 pnum;

	if (argc >= 1 && !(str2long(argv[0], off))) {
		if ((mtdparts_init() == 0) &&
		    (find_dev_and_part(argv[0], &dev, &pnum, &part) == 0)) {
			if (dev->id->type != MTD_DEV_TYPE_NAND) {
				puts("not a NAND device\n");
				return -1;
			}
			*off = part->offset;
			if (argc >= 2) {
				if (!(str2long(argv[1], (loff_t *)size))) {
					printf("'%s' is not a number\n", argv[1]);
					return -1;
				}
				if (*size > part->size)
					*size = part->size;
			} else {
				*size = part->size;
			}
			idx = dev->id->num;
			*nand = nand_info[idx];
			goto out;
		}
	}
#endif

	if (argc >= 1) {
		if (!(str2long(argv[0], off))) {
			printf("'%s' is not a number\n", argv[0]);
			return -1;
		}
	} else {
		*off = 0;
	}

	if (argc >= 2) {
		if (!(str2long(argv[1], (loff_t *)size))) {
			printf("'%s' is not a number\n", argv[1]);
			return -1;
		}
	} else {
		*size = nand->size - *off;
	}

#if defined(CONFIG_CMD_MTDPARTS)
out:
#endif
	printf("device %d ", idx);
	if (*size == nand->size)
		puts("whole chip\n");
	else
		printf("offset 0x%08x%08x, size 0x%08x%08x\n",
			(uint)(*off>>32), (uint)(*off&-1), (uint)(*size>>32), (uint)(*size&-1));
	return 0;
}


static uint64_t get_end_offset(struct mtd_info *mtd, loff_t offset,
				const size_t length)
{
	struct mlc_page *mlp = &mlc_page;
	size_t data_len, nand_len = 0;
	size_t pagesize;

	pagesize = mtd->writesize;
	data_len = mlp->data_page_nr * pagesize;	/* data size per block */

	while (length > nand_len) {
		if (nand_block_isbad(mtd, offset&~(mtd->erasesize-1))) {
			offset += mtd->erasesize;
			continue;
		}

		nand_len += data_len;
		offset   += mtd->erasesize;

	//	DBGOUT("nand:0x%08x, len:0x%08x, offset:0x%08x%08x\n",
	//		nand_len, length, (uint)(offset >>32), (uint)(offset &-1));

		if (offset >= mtd->size)
			return 0;
	}
	return offset;
}

static int nand_check_wp(struct mtd_info *mtd)
{
    struct nand_chip *chip = mtd->priv;
    /* Check the WP bit */
    chip->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);
    return (chip->read_byte(mtd) & NAND_STATUS_WP) ? 0 : 1;
}

static void nand_bad_marker(struct mtd_info *mtd, loff_t offset)
{
	struct nand_chip *chip = mtd->priv;
	int page, chipnr, status;
	uint8_t *buf = chip->oob_poi;
	int length = mtd->oobsize;

	memset(chip->oob_poi, 0xff, mtd->oobsize);

	/* bad block table */
	mtd->block_markbad(mtd, offset);

	page   = (int)(offset >> chip->page_shift);
	chipnr = (int)(offset >> chip->chip_shift);
	DBGOUT("BAD maker %4d\n", page);

	chip->select_chip(mtd, chipnr);

	chip->erase_cmd(mtd, page & chip->pagemask);
	status = chip->waitfunc(mtd, chip);
	if (status & NAND_STATUS_FAIL) {
		ERROUT("fail: erase, page 0x%08x\n", page);
		return;
	}

	/* bad mark */
	buf[0] = 0x00, buf[1] = 0x00;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, mtd->writesize, page);
	chip->write_buf(mtd, buf, length);

	/* Send command to program the OOB data */
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	chip->waitfunc(mtd, chip);

	memset(chip->oob_poi, 0xff, mtd->oobsize);
}

/*------------------------------------------------------------------------------
 * u-boot mlc nand function
 */

static int mlc_nand_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			   const uint8_t *buf, int page, uint8_t *eccbuf, int head)
{
	int i = 0, status, ret = 0;
	int eccsteps, eccsize, pagesize, eccbytes;

	uint8_t *p = (uint8_t *)buf;
	uint8_t eccdata[512] = {0, };

	int err;
	int	o_syn[MLC_ECC_MODE], e_pos[MLC_ECC_MODE];

	pagesize = mtd->writesize;
	eccsize  = MLC_ECC_SIZE;
	eccsteps = pagesize/eccsize;
	eccbytes = MLC_ECC_ALIGN;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		NX_NAND_SetResetECC(MLC_ECC_MODE);

		chip->write_buf(mtd, p, eccsize);

		NX_NAND_WaitForEncoding();
		NX_NAND_GetGenECC((uint32_t*)&eccdata[i], MLC_ECC_MODE);
		if (head && 0 == i) {
			DBGOUT("HEAD page %4d, offset 0x%08x\n", page, page*pagesize);
			memcpy((p+eccsize), eccdata, MLC_ECC_ALIGN);
		}
	}

	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	status = chip->waitfunc(mtd, chip);
	if (status & NAND_STATUS_FAIL)
		return -EIO;

	/*
	* verify write data
	*/
#if 1
	eccsteps = pagesize/eccsize;
	p = (uint8_t *)buf;

	chip->cmdfunc(mtd, NAND_CMD_READ0, 0x00, page);

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		NX_NAND_SetResetECC(MLC_ECC_MODE);
		NX_NAND_SetOriECC((uint32_t*)&eccdata[i], MLC_ECC_MODE);

		chip->read_buf(mtd, p, eccsize);

		NX_NAND_WaitForDecoding();
		if (NX_NAND_GetErrorStatus()) {
			NX_NAND_GetOddSyndrome(&o_syn[0]);
			ret = NX_NAND_GetErrorLocation(&o_syn[0], &e_pos[0], &err);
			if (0   > ret ||
				err > MLC_ECC_LIMIT) {
				ERROUT("nand ecc detect errors, can't correct (page:%d, err:%d)\n",
					page, err);
				return ret > 0 ? -EIO : -EBADMSG;
			}
		}
	}
#endif
	/* copy ecc to ecc buffer */
	if (eccbuf) {
		eccsteps = pagesize/eccsize;
		memcpy(eccbuf, eccdata, (MLC_ECC_ALIGN*eccsteps));
	}
	return 0;
}

static int mlc_nand_write_block(struct mtd_info *mtd, loff_t offset, loff_t length,
				u_char *buffer, struct data_info *info)
{
	struct nand_chip *chip = mtd->priv;
	struct mlc_page  *mlp  = &mlc_page;

	uint8_t *w_buf, *ecc_buf;
	uint32_t w_len;
	int chipnr, page, realpage, endpage, pagesize;
	int eccpos, ecctotal, eccsteps;
	int ret = 0, head = 0, status;

	if (! length)
		return 0;

	pagesize = mtd->writesize;
	eccsteps = pagesize/MLC_ECC_SIZE;
	eccpos   = 1024;	/* info 512 + info ecc 512 */
	ecctotal = 1024 + (MLC_ECC_ALIGN * eccsteps * mlp->data_page_nr);

	ecc_buf  = kzalloc(ecctotal , GFP_KERNEL);
	if (! ecc_buf) {
		ERROUT("fail: out of memory\n");
		return -ENOMEM;
	}

	chipnr = (int)(offset >> chip->chip_shift);
	chip->select_chip(mtd, chipnr);

	if (nand_check_wp(mtd)) {
		ERROUT("nand is write protected\n");
		ret = -EIO;
		goto exit_w;
	}

	/*
	 * write data to page and get ecc for data
	 */
	realpage = ((int)(offset >> chip->page_shift) & chip->pagemask);
	page 	 = realpage;
	endpage  = page + mlp->data_page_nr;
	w_buf	 = buffer;
	w_len	 = length;

	DBGOUT("erase ... %4d\n", page);
	DBGOUT("data page %4d ~ %4d total %4d\n", page, endpage, endpage-page);

	/* erase block */
	chip->erase_cmd(mtd, realpage);
	status = chip->waitfunc(mtd, chip);
	if (status & NAND_STATUS_FAIL) {
		ERROUT("fail: erase, page 0x%08x\n", realpage);
		return -EIO;
	}

	/* write page */
	while(1) {
		ret = mlc_nand_write_page(mtd, chip, w_buf, page, &ecc_buf[eccpos], 0);
		if (ret)
			goto exit_w;

		w_len -= pagesize;
		page++;
		if (page >= endpage || !w_len)
			break;

		w_buf  += pagesize;
		eccpos += (MLC_ECC_ALIGN * eccsteps);
	}

	/*
	 * write ecc to page
	 */
	memcpy(ecc_buf, info, sizeof(struct data_info));

	page 	= realpage + mlp->data_page_nr;
	endpage = page + mlp->ecc_page_nr;
	w_buf	= ecc_buf;
	head    = 1;
	DBGOUT("ecc  page %4d ~ %4d, total %4d, ecc %d\n",
		page, endpage, endpage-page, eccpos);

	while(1) {
		ret = mlc_nand_write_page(mtd, chip, w_buf, page, NULL, head);
		if (ret)
			goto exit_w;

		head   = 0;
		w_buf += pagesize;
		page++;
		if (page >= endpage)
			break;
	}

exit_w:
	kfree(ecc_buf);
	return ret;
}

int mlc_nand_write(struct mtd_info *mtd, loff_t offset, loff_t length,
			u_char *buffer)
{
	int rval = 0, pagesize;
	loff_t left_to_write = length;
	u_char *p_buffer = buffer;

	struct mlc_page  *mlp  = &mlc_page;
	struct data_info  info = {0, };
	size_t data_len;
	uint   percent = 0, prog_unit;

	DBGOUT("%s\n", __func__);

	/* Reject writes, which are not block or page aligned */
	if ((offset & (mtd->erasesize - 1)) != 0) {
		ERROUT("fail: offset 0x%08x%08x is not aligned block 0x%08x\n",
			(uint)(offset>>32), (uint)(offset&-1),mtd->erasesize);
		return -EINVAL;
	}

	if ((length & (mtd->writesize - 1)) != 0) {
		ERROUT("fail: length 0x%08x is not aligned page 0x%08x\n",
			(uint)length, mtd->writesize);
		return -EINVAL;
	}

	if (! get_end_offset(mtd, offset, length)) {
		ERROUT("outside the nand area\n");
		return -EINVAL;
	}

	pagesize = mtd->writesize;
	data_len = mlp->data_page_nr * pagesize;	/* data size per block */

	/* set image info */
	info.magic	 = MLC_ECC_MAGIC;
	info.total 	 = length;
	info.page_nr = data_len;
	prog_unit 	 = (100<<10)/__DIVID(length, data_len);

	printf("\n");
	printf(" write image len :%4d page, 0x%x bytes\n",
		(uint)length/pagesize, (uint)length);
	printf(" nand page  size :%4d kbytes\n", pagesize>>10);
	printf(" nand block size :%4d kbytes, %4d page\n",
		mtd->erasesize>>10, mtd->erasesize/pagesize);
	printf(" data page  size :%4d kbytes, %4d page\n",
		mlp->data_page_nr*pagesize>>10, mlp->data_page_nr);
	printf(" ecc  page  size :%4d kbytes, %4d page\n",
		mlp->ecc_page_nr*pagesize>>10, mlp->ecc_page_nr);
	printf(" erase nand size :%d kbytes\n\n",
		__DIVID(length, data_len)*mtd->erasesize>>10);

	while (left_to_write > 0) {
		size_t write_size;

		if (nand_block_isbad(mtd, offset&~(mtd->erasesize-1))) {
			printf ("Skipping bad block 0x%08x%08x\n",
				(uint)((offset&~(mtd->erasesize-1))>>32),
				(uint)((offset&~(mtd->erasesize-1))&-1));
			offset += mtd->erasesize;
			continue;
		}

		if (data_len > left_to_write)
			write_size = left_to_write;
		else
			write_size = data_len;

		info.length = write_size;

		DBGOUT("write block : 0x%08x%08x\n", (uint)(offset>>32),(uint)(offset & -1));
		rval = mlc_nand_write_block(mtd, offset, data_len, p_buffer, &info);
		if (rval != 0) {
			if (-EBADMSG == rval) {
				nand_bad_marker(mtd, offset);
				offset += mtd->erasesize;
				continue;
			}
			printf("NAND write to offset 0x%08x%08x failed %d\n",
				(uint)(offset>>32), (uint)(offset&-1), rval);
			return rval;
		}

		if (percent >> 10)
			printf("\r :%d%%", percent>>10);

		percent		  += prog_unit;
		left_to_write -= write_size;
		p_buffer      += write_size;
		offset        += mtd->erasesize;
		info.index++;
	}
	printf("\r :100%%\n");

	return 0;
}

static int mlc_nand_read_page(struct mtd_info *mtd, struct nand_chip *chip,
			   const uint8_t *buf, int page, uint8_t *eccbuf, int head)
{
	int i, n, ret = 0;
	int eccsteps, eccsize, pagesize, eccbytes;

	uint8_t *p = (uint8_t *)buf;
	uint8_t eccdata[512] = {0, };

	int err;
	int	o_syn[MLC_ECC_MODE], e_pos[MLC_ECC_MODE];
	uint32_t *e_dat;
	int ecc = 0;

	if (eccbuf)
		ecc = 1;

	pagesize = mtd->writesize;
	eccsize  = MLC_ECC_SIZE;
	eccsteps = pagesize/eccsize;
	eccbytes = MLC_ECC_ALIGN;

	chip->cmdfunc(mtd, NAND_CMD_READ0, 0x00, page);

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {

		if (ecc) {
			memcpy(eccdata, &eccbuf[i], MLC_ECC_ALIGN);
			NX_NAND_SetResetECC(MLC_ECC_MODE);
			NX_NAND_SetOriECC((uint32_t*)eccdata, MLC_ECC_MODE);
		}

		/* read data */
		chip->read_buf(mtd, p, eccsize);

		if (ecc) {
			NX_NAND_WaitForDecoding();
			if (NX_NAND_GetErrorStatus()) {
				NX_NAND_GetOddSyndrome(&o_syn[0]);
				ret = NX_NAND_GetErrorLocation(&o_syn[0], &e_pos[0], &err);
				if (0 > ret) {
					ERROUT("nand ecc detect errors, can't correct (page:%d, err:%d)\n",
						page, err);
					ret = -EIO;
					break;
				} else {
					printf("nand page %4d ecc %2d error found : ", page, err);
					for (n=0 ; err > n; n++) {
						printf("%4d", e_pos[n]);
						if (n != err-1) printf(", ");
						if (4096 > e_pos[n]) {
							e_dat = (uint32_t*)p;
							e_dat[e_pos[n] / 32] ^= 1U<<(e_pos[n] % 32);
						}
					}
					ret = err > MLC_ECC_LIMIT ? -EBADMSG: 0;
					printf("\n");
				}
			}
		}

		if (head)
			ecc = 0;
	}

	return ret;
}

static int mlc_nand_read_block(struct mtd_info *mtd, loff_t offset, loff_t length,
				u_char *buffer, struct data_info *info)
{
	struct nand_chip *chip = mtd->priv;
	struct mlc_page  *mlp  = &mlc_page;

	uint8_t *r_buf, *ecc_buf, *ecc_h;
	uint32_t r_len;
	int chipnr, page, realpage, endpage, pagesize;
	int eccpos, ecctotal, eccsteps;
	int ret = 0, head = 0;

	if (! length)
		return 0;

	pagesize = mtd->writesize;
	eccsteps = pagesize/MLC_ECC_SIZE;
	eccpos   = 1024;	/* info 512 + info ecc 512 */
	ecctotal = 1024 + (MLC_ECC_ALIGN * eccsteps * mlp->data_page_nr);

	ecc_buf  = kzalloc(ecctotal , GFP_KERNEL);
	if (! ecc_buf) {
		ERROUT("fail: out of memory\n");
		return -ENOMEM;
	}
	memset(ecc_buf, 0, ecctotal);

	chipnr = (int)(offset >> chip->chip_shift);
	chip->select_chip(mtd, chipnr);

	/*
	 * read ecc page
	 */
	realpage = ((int)(offset >> chip->page_shift) & chip->pagemask);
	page 	 = realpage + mlp->data_page_nr;
	endpage  = page + mlp->ecc_page_nr;
	r_buf	 = ecc_buf;
	head     = 0;

	/* read info page and info ecc */
	ret = mlc_nand_read_page(mtd, chip, r_buf, page, NULL, head);
	if (ret == -EIO)
		goto exit_r;

	memcpy(info, ecc_buf, sizeof(struct data_info));
	if (MLC_ECC_MAGIC != info->magic) {
		ERROUT("fail, page %4d not mlc image 0x%08x, magic 0x%08x\n",
			page, info->magic, MLC_ECC_MAGIC);
		ret = -EFAULT;
		goto exit_r;
	}

	ecc_h = &ecc_buf[512];
	r_buf = ecc_buf;
	head  = 1;
	DBGOUT("ecc  page %4d ~ %4d total %4d\n", page, endpage, endpage-page);

	/* read ecc page and check info ecc */
	while(1) {
		ret = mlc_nand_read_page(mtd, chip, r_buf, page, ecc_h, head);
		if (ret == -EIO)
			goto exit_r;

		head   = 0;
		ecc_h  = NULL;
		r_buf += pagesize;
		page++;

		if (page >= endpage)
			break;
	}

	/*
	 * read data page and check ecc
	 */
	realpage = ((int)(offset >> chip->page_shift) & chip->pagemask);
	page 	 = realpage;
	endpage  = page + mlp->data_page_nr;
	r_buf 	 = buffer + info->index * info->page_nr;
	r_len	 = length;
	DBGOUT("data page %4d ~ %4d total %4d to 0x%08x (%d)\n",
		page, endpage, endpage-page, (uint32_t)r_buf, info->index);

	while(1) {
		ret = mlc_nand_read_page(mtd, chip, r_buf, page, &ecc_buf[eccpos], 0);
		if (ret == -EIO)
			goto exit_r;

		r_len -= pagesize;
		page++;
		if (page >= endpage || !r_len)
			break;

		r_buf  += pagesize;
		eccpos += (MLC_ECC_ALIGN * eccsteps);
	}

exit_r:
	kfree(ecc_buf);
	return ret;
}

static int mlc_nand_read_copy(struct mtd_info *mtd, loff_t *offset, loff_t length,
				u_char *buffer, struct data_info *info)
{
	struct mlc_page  *mlp  = &mlc_page;
	int 	 pagesize;
	uint32_t data_len;
	u_char * p_buffer;

	/* check bad block */
	while(1) {
		if (! nand_block_isbad(mtd, *offset&~(mtd->erasesize-1)))
			break;
		*offset += mtd->erasesize;
		if (*offset >= mtd->size) {
			ERROUT("outside the nand area\n");
			return -EINVAL;
		}
	}

	pagesize = mtd->writesize;
	data_len = mlp->data_page_nr * pagesize;

	/* write to new block */
	p_buffer = buffer + info->index * info->page_nr;
	return mlc_nand_write_block(mtd, *offset, data_len, p_buffer, info);
}

static int mlc_nand_read(struct mtd_info *mtd, loff_t offset, loff_t length,
		       u_char *buffer)
{
	int rval, pagesize;
	loff_t left_to_read = length;
	loff_t endoff;

	struct mlc_page *mlp  = &mlc_page;
	struct data_info info = {0, };
	size_t data_len;
	uint   percent = 0, prog_unit;

	/* Reject writes, which are not block or page aligned */
	if ((offset & (mtd->erasesize - 1)) != 0) {
		ERROUT("fail: offset 0x%08x%08x is not aligned block 0x%08x\n",
			(uint)(offset>>32), (uint)(offset&-1), mtd->erasesize);
		return -EINVAL;
	}
	if ((length & (mtd->writesize - 1)) != 0) {
		ERROUT("fail: length 0x%08x%08x is not aligned page 0x%08x\n",
			(uint)(length>>32), (uint)(length&-1), mtd->writesize);
		return -EINVAL;
	}

	endoff = get_end_offset(mtd, offset, length);
	if (! endoff) {
		ERROUT("outside the nand area\n");
		return -EINVAL;
	}

	pagesize  = mtd->writesize;
	data_len  = mlp->data_page_nr * pagesize;	/* data size per block */
	prog_unit = (100<<10)/__DIVID(length, data_len);

	while (left_to_read > 0) {
		size_t read_size;

		if (nand_block_isbad(mtd, offset&~(mtd->erasesize-1))) {
			printf ("Skipping bad block 0x%08x%08x\n",
				(uint)((offset&~(mtd->erasesize-1))>>32),
				(uint)((offset&~(mtd->erasesize-1))&-1));
			offset += mtd->erasesize;
			continue;
		}

		if (data_len > left_to_read)
			read_size = left_to_read;
		else
			read_size = data_len;

		DBGOUT("read  block : 0x%08x%08x\n", (uint)(offset>>32),(uint)(offset&-1));
		rval = mlc_nand_read_block(mtd, offset, data_len, buffer, &info);
		if (rval && rval != -EBADMSG) {
			printf ("NAND read from offset 0x%08x%08x failed %d\n",
				(uint)(offset>>32),(uint)(offset&-1), rval);
			return rval;
		}

/*
		if (-1 != test_bad_idx &&
			test_bad_idx == info.index)
			rval = -EBADMSG;
*/
		if (rval == -EBADMSG) {
			printf("mlc: copy back  : 0x%08x%08x (idx:%d) -> 0x%08x%08x\n",
				(uint)(offset>>32),(uint)(offset&-1), info.index,
				(uint)(endoff>>32),(uint)(endoff&-1));

			nand_bad_marker(mtd, offset);
			rval = mlc_nand_read_copy(mtd, &endoff, read_size, buffer, &info);
			if (rval)
				return rval;
		}

		if (percent >> 10)
			printf("\r :%d%%", percent>>10);

		percent		 += prog_unit;
		left_to_read -= read_size;
		offset       += mtd->erasesize;
	}
	printf("\r :100%%\n");
	return 0;
}

static int mlc_nand_erase(struct mtd_info *mtd, loff_t offset, loff_t length)
{
	int page, status = 0, block_page, chipnr, pagesize;
	struct nand_chip *chip = mtd->priv;
	struct mlc_page *mlp = &mlc_page;
	size_t data_len;
	size_t len = length;
	uint   percent = 0, prog_unit;

	if ((offset & (mtd->erasesize - 1)) != 0) {
		ERROUT("fail: offset 0x%08x%08x is not aligned block 0x%08x\n",
			(uint)(offset>>32), (uint)(offset&-1), mtd->erasesize);
		return -EINVAL;
	}

	if ((length & (mtd->erasesize - 1)) != 0) {
		ERROUT("fail: length 0x%08x%08x is not aligned block 0x%08x\n",
			(uint)(length>>32), (uint)(length&-1), mtd->writesize);
		return -EINVAL;
	}

	/* Do not allow erase past end of device */
	if ((length + offset) > mtd->size) {
		ERROUT("fail: Erase past end of device\n");
		return -EINVAL;
	}

	/* Shift to get first page */
	page   = (int)(offset >> chip->page_shift);
	chipnr = (int)(offset >> chip->chip_shift);

	chip->select_chip(mtd, chipnr);
	if (nand_check_wp(mtd)) {
		ERROUT("fail: Device is write protected!!!\n");
		return -EINVAL;
	}

	block_page = 1 << (chip->phys_erase_shift - chip->page_shift);
	pagesize   = mtd->writesize;
	data_len   = mlp->data_page_nr * pagesize;	/* data size per block */
	length 	   = __DIVID(len, data_len) * mtd->erasesize;
	prog_unit  = (100<<10)/__DIVID(length, data_len);

	printf("\n");
	printf(" input erase size:%4d page, 0x%x bytes\n",
		(uint)len/pagesize, (uint)len);
	printf(" nand page  size :%4d kbytes\n", pagesize>>10);
	printf(" nand block size :%4d kbytes, %4d page\n",
		mtd->erasesize>>10, mtd->erasesize/pagesize);
	printf(" data page  size :%4d kbytes, %4d page\n",
		mlp->data_page_nr*pagesize>>10, mlp->data_page_nr);
	printf(" ecc  page  size :%4d kbytes, %4d page\n",
		mlp->ecc_page_nr*pagesize>>10, mlp->ecc_page_nr);
	printf(" real erase size :0x%x bytes (0x%x ~ 0x%x)\n\n",
		(uint)length, (uint)offset, (uint)(offset+length));
	printf("Really erase ? <y/N>\n");

	if (getc() == 'y') {
		puts("y");
		if (getc() != '\r') {
			puts("mlc erase aborted\n");
			return -1;
		}
	} else {
		puts("mlc erase aborted\n");
		return -1;
	}
	puts("\n");

	while (length) {
		if (nand_block_isbad(mtd, offset&~(mtd->erasesize-1))) {
			printf ("Skipping bad block 0x%08x%08x\n",
				(uint)((offset&~(mtd->erasesize-1))>>32),
				(uint)((offset&~(mtd->erasesize-1))&-1));
			offset += mtd->erasesize;
			continue;
		}

		DBGOUT("erase block : 0x%08x\n", page*pagesize);
		chip->erase_cmd(mtd, page & chip->pagemask);
		status = chip->waitfunc(mtd, chip);
		if (status & NAND_STATUS_FAIL) {
			ERROUT("fail: erase, page 0x%08x\n", page);
			return -EIO;
		}

		if (percent >> 10)
			printf("\r :%d%%", percent>>10);

		/* Increment page address and decrement length */
		percent+= prog_unit;
		length -= (1 << chip->phys_erase_shift);
		page   += block_page;

		/* Check, if we cross a chip boundary */
		if (length && !(page & chip->pagemask)) {
			chipnr++;
			chip->select_chip(mtd, -1);
			chip->select_chip(mtd, chipnr);
		}
	}
	printf("\r :100%%\n");
	return 0;
}

/*------------------------------------------------------------------------------
 * u-boot mlc nand command
 */
int do_mlcnand(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	int ret=0, read;
	ulong  addr;
	loff_t off, size;
	char  *cmd;
	nand_info_t *nand;
	struct mlc_page *mlp = &mlc_page;

	int m_nr, data_page_nr, ecc_page_nr;

	cmd = argv[1];

	if (strcmp(cmd, "read") != 0 &&
		strcmp(cmd, "write") != 0 &&
		strcmp(cmd, "erase") != 0)
		goto usage;

	if (nand_curr_device < 0 || nand_curr_device >= CONFIG_SYS_MAX_NAND_DEVICE ||
	    !nand_info[nand_curr_device].name) {
		ERROUT("\nno devices available\n");
		return 1;
	}

	nand = &nand_info[nand_curr_device];
	if (! nand) {
		ERROUT("no nand device: %d\n", nand_curr_device);
		return 1;
	}

	switch (MLC_ECC_MODE) {
	case 16:
		data_page_nr = 60, ecc_page_nr = 4;
		break;
	case 24:
	default:
		data_page_nr = 58, ecc_page_nr = 6;
		break;
	}

	m_nr = (nand->writesize/1024)/2;
	mlp->data_page_nr = data_page_nr * m_nr;
	mlp->ecc_page_nr  = ecc_page_nr  * m_nr;

#ifndef CONFIG_SYS_NAND_HW_ECC
	{
		static int creat_table = 0;
		if (! creat_table)
			NX_NAND_CreateLookupTable();
		creat_table = 1;
	}
#endif

	/* erase */
	if (strcmp(cmd, "erase") == 0) {
		if (3 > argc)
			goto usage;

		if (arg_off_size(argc - 2, argv + 2, nand, &off, &size) != 0)
			return 1;

		ret = mlc_nand_erase(nand, off, size);
	}

	/* read and write */
	if (strncmp(cmd, "read", 4) == 0 || strncmp(cmd, "write", 5) == 0) {
		if (4 > argc)
			goto usage;

		addr = simple_strtoul(argv[2], NULL, 16);

		read = strncmp(cmd, "read", 4) == 0; /* 1 = read, 0 = write */
		printf("\nMLC NAND %s: ", read ? "read" : "write");
		if (arg_off_size(argc - 3, argv + 3, nand, &off, &size) != 0)
			return 1;
/*
		if (argc>5)
			test_bad_idx = simple_strtoul(argv[5], NULL, 10);
		else
			test_bad_idx = -1;
*/
		if (read)
			ret = mlc_nand_read (nand, off, size, (u_char *)addr);
		else
			ret = mlc_nand_write(nand, off, size, (u_char *)addr);

		printf(" %u bytes %s: %s\n", (uint)size,
	    	   read ? "read" : "written", ret ? "ERROR" : "OK");
	}

	return ret == 0 ? 0 : 1;
usage:
	cmd_usage(cmdtp);
	return 1;
}

U_BOOT_CMD(mlcnand, CONFIG_SYS_MAXARGS, 1, do_mlcnand,
	"Multi-Level Cell(MLC) NAND system",
	"mlcnand read - addr off size\n"
	"mlcnand write - addr off size\n"
	"    read/write 'size' bytes starting at offset 'off'\n"
	"    to/from memory address 'addr', erase write block and skipping bad blocks"
/*
	"mlcnand erase - off size\n"
	"	 off size - erase 'size' bytes from off"
*/
);

#endif /* CONFIG_CMD_NAND_MLC */
