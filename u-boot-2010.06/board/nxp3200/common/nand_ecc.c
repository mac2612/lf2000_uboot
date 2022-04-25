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
#if (0)
#include <common.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <malloc.h>
#include <linux/mtd/nand.h>
#include <nand.h>
#include <platform.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <mach/platform.h>
#else
#include <common.h>
#include <asm/errno.h>
#include <linux/compiler.h>
#include <linux/mtd/compat.h>
#include <linux/mtd/nand.h>
#include <asm/io.h>
#include <malloc.h>

#endif

#if defined (CONFIG_MTD_NAND_NEXELL_HWECC) || defined (CONFIG_SYS_NAND_HW_ECC)

#include "nxp3200_nand_ecc.h"
#include <lf2000.h>

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
extern unsigned int nor_to_ram_time;
#endif

/* to collect timing info for scanning NAND to identify bad blocks
 * enable the #define */
#define TIME_SCAN_NAND_FOR_BB	
#ifdef TIME_SCAN_NAND_FOR_BB
extern unsigned int scan_nand_for_bb_time;
extern unsigned int scan_nand_for_bb_end_time;
#endif

struct mtd_info * get_nor_mtd(void);

int badblock_table_in_NOR(struct mtd_info * mtd, loff_t * p_data_offset,
						 int    * p_data_size); 
int copy_bbt_from_NOR_to_ram(struct mtd_info * mtdnor, 
			     struct mtd_info * mtdnand,
			     loff_t data_offset, int data_size);
void copy_bbt_from_ram_to_NOR(struct mtd_info * mtdnor, 
			      struct mtd_info * mtdnand);

#endif


// Now CONFIG_MTD_NAND_LF2000_PROF is #defined (or not) in lf2000.h
// TODO: FIXME:	Decide if we ought to add a new option to Kconfig
#ifdef CONFIG_MTD_NAND_LF2000_PROF
#include "prof.h"
#endif	/* CONFIG_MTD_NAND_LF2000_PROF */

/* NOTE: enable the following #define to use XOR'd ECC bytes.
 *	 When it processes an all-FF packet of bytes, the LF2000's 
 * nand controller's ECC calculator generates a set of ECC bytes that are 
 * not all 0xFF.  Therefore when reading an erased page in which a few bits
 * are not '1', the controller's ECC verifier cannot identify the flipped
 * bits, so the firmware cannot correct them.
 *
 * To avoid this situation, we store in NAND modified ECC bytes.  They are
 * modified by XORing them with the ECC bytes that are generated for a set
 * of all-FF bytes.  When we read from NAND, we read the modified ECC bytes,
 * XOR them again (obtaining the originally generated ECC bytes), and loading
 * the resulting bytes into the LF2000's ECC registers.
 *
 */
#define USE_XORd_ECC	1

static struct tag_bch_arrays bch_13_arrays = {
	13,			/* bch_var_m */
	8191,		/* bch_var_n */
	NULL,		/* bch_AlphaToTable */
	NULL		/* bch_IndexOfTable */
};

static struct tag_bch_arrays bch_14_arrays = {
	14,			/* bch_var_m */
	16383,		/* bch_var_n */
	NULL,		/* bch_AlphaToTable */
	NULL		/* bch_IndexOfTable */
};

#define NUM_4BIT_ECC_BYTES	7
#define NUM_8BIT_ECC_BYTES	13
#define NUM_12BIT_ECC_BYTES	20
#define NUM_16BIT_ECC_BYTES	26
#define NUM_24BIT_ECC_BYTES	42


static const u8 ecc4_xor_factors[NUM_4BIT_ECC_BYTES] = {
	(u8)~0x97, (u8)~0x38, (u8)~0x79, (u8)~0xab, 
	(u8)~0x9d, (u8)~0x49, (u8)~0xd0
};
static const u8 ecc8_xor_factors[NUM_8BIT_ECC_BYTES] = {
	(u8)~0x37, (u8)~0x83, (u8)~0xc7, (u8)~0xcc, 
	(u8)~0x13, (u8)~0x48, (u8)~0xde, (u8)~0xd2, 
	(u8)~0x82, (u8)~0x3d, (u8)~0xfc, (u8)~0x4b, (u8)~0x04 
};
static const u8 ecc12_xor_factors[NUM_12BIT_ECC_BYTES] = {
	(u8)~0x6f, (u8)~0xd7, (u8)~0x44, (u8)~0x62, 
	(u8)~0x24, (u8)~0x4c, (u8)~0xeb, (u8)~0xba,
	(u8)~0xfc, (u8)~0x4b, (u8)~0xdc, (u8)~0xe2, 
	(u8)~0xa5, (u8)~0x1b, (u8)~0x19, (u8)~0x44, 
	(u8)~0x95, (u8)~0xf5, (u8)~0xaa, (u8)~0x80 
};
static const u8 ecc16_xor_factors[NUM_16BIT_ECC_BYTES] = {
	(u8)~0x95, (u8)~0xf2, (u8)~0x8f, (u8)~0x75, 
	(u8)~0xaf, (u8)~0xee, (u8)~0x40, (u8)~0xca,
	(u8)~0xda, (u8)~0x4f, (u8)~0x0d, (u8)~0x62, 
	(u8)~0x5f, (u8)~0x0b, (u8)~0xef, (u8)~0x5d, 
	(u8)~0xbb, (u8)~0xee, (u8)~0xa8, (u8)~0x9b, 
	(u8)~0xf7, (u8)~0x7a, (u8)~0x13, (u8)~0x19,
	(u8)~0x62, (u8)~0xf4
};
static const u8 ecc24_xor_factors[NUM_24BIT_ECC_BYTES] = {
	(u8)~0x7b, (u8)~0x08, (u8)~0x8f, (u8)~0x93, 
	(u8)~0x21, (u8)~0x2f, (u8)~0xd4, (u8)~0xb7, 
	(u8)~0x40, (u8)~0x56, (u8)~0x9f, (u8)~0x43, 
	(u8)~0xee, (u8)~0x9f, (u8)~0xd2, (u8)~0xe0, 
	(u8)~0x84, (u8)~0x0b, (u8)~0x62, (u8)~0x58, 
	(u8)~0xd8, (u8)~0xb1, (u8)~0x5a, (u8)~0x0c, 
	(u8)~0x2c, (u8)~0x6a, (u8)~0x47, (u8)~0xfc, 
	(u8)~0xa5, (u8)~0xaf, (u8)~0x6e, (u8)~0xd7, 
	(u8)~0xde, (u8)~0x73, (u8)~0x4a, (u8)~0x57, 
	(u8)~0xe2, (u8)~0x85, (u8)~0xad, (u8)~0xeb, 
	(u8)~0xc4, (u8)~0x05
};

static const struct tag_bch_info bch_info_4_bits = {
	4, 8 * 512, 13, NUM_4BIT_ECC_BYTES, ecc4_xor_factors, &bch_13_arrays
};
static const struct tag_bch_info bch_info_8_bits = {
	8, 8 * 512, 13, NUM_8BIT_ECC_BYTES, ecc8_xor_factors, &bch_13_arrays
};
static const struct tag_bch_info bch_info_12_bits = {
	12, 8 * 512, 13, NUM_12BIT_ECC_BYTES, ecc12_xor_factors, &bch_13_arrays
};
static const struct tag_bch_info bch_info_16_bits = {
	16, 8 * 512, 13, NUM_16BIT_ECC_BYTES, ecc16_xor_factors, &bch_13_arrays
};
static const struct tag_bch_info bch_info_24_bits = {
	24, 8 * 1024, 14, NUM_24BIT_ECC_BYTES, ecc24_xor_factors, &bch_14_arrays
};

/* The struct nand_chip.priv member points to a struct tag_bch_info */


#if	(0)
#define DBGOUT(msg...)		{ printk(KERN_INFO "ecc: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define ERROUT(msg...)		{ 					\
		printk(KERN_ERR "ERROR: %s, %s line %d: \n",		\
			__FILE__, __FUNCTION__, __LINE__),	\
		printk(KERN_ERR msg); }

void NX_NAND_OutputNandControllerRegs(void);

/*
 * 0x05	= Bad marker (256/512 byte pages)
 * 0x00 = Bad marker (2048/4096   byte pages)
 * 0x01 = Reserved   (2048/4096   byte pages)
 */
static struct nand_ecclayout nx_nand_oob = {
	.eccbytes 	=   0,
	.eccpos 	= { 0, },
	.oobfree 	= { {.offset = 0, .length = 0} }
};

#define	ECC_HW_MAX_BYTES	(44) 	//  4 bit ecc ( 7byte),  8 bit ecc (13byte),
									// 16 bit ecc (26byte), 24 bit ecc (42byte)

/*------------------------------------------------------------------------------
 * u-boot nand hw ecc interface
 */
static int nand_sw_ecc_verify_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	int i;
	struct nand_chip *chip = mtd->priv;

	for (i = 0; len > i; i++)
		if (buf[i] != readb(chip->IO_ADDR_R))
			return -EFAULT;
	return 0;
}


static void FlipErrorBits( int num_errors, int * e_pos, 
			   int eccsize, uint32_t * pdata)
{
	int i;

	for (i=0 ; i < num_errors; i++) {
#ifndef NAND_ACCESS_TIMING	// we don't want to time serial output
		DBGOUT("  error #%d in bit %d (byte 0x%x, mask %x)\n",
			   i, e_pos[i], e_pos[i]/8, 1 << (e_pos[i] & 7));
#endif

		if (   (0   <= e_pos[i])
			&& (e_pos[i] < (8 * eccsize)))
		{
/* 2may12 Commented out #ifndef just for testing; restored 3may12 */
#ifndef NAND_ACCESS_TIMING	// we don't want to time serial output
			uint32_t bad_value, fixed_value;

			bad_value   = pdata[e_pos[i] / 32];
			fixed_value = bad_value ^ 1U<<(e_pos[i] % 32);
			DBGOUT("  change 0x%08x to 0x%08x\n",
					bad_value, fixed_value);
#endif
			pdata[e_pos[i] / 32] ^= 1U << (e_pos[i] & 31);
		}
	}
}


void NX_NAND_StartEccDecoding(int ecccmode, int num_bytes);
void NX_NAND_StartEccEncoding(int ecccmode, int num_bytes);

#define CURRENT_ECC_MODE \
	( ((struct lf2000_nand_data *)chip->priv)->bch_info->bch_var_t)

/**
 * nand_hw_ecc_read_subpage - hardware-ecc-based sub-page read function
 * @mtd:		mtd info structure
 * @chip:		nand chip info structure
 * @data_offs:	offset of requested data within the page
 * @readlen:	data length (in bytes_
 * @bufpoi:		page buffer to store read data
 *
 * Reads enough data from the currently addressed NAND page to ensure that
 * the bytes from data_offs to data_offs + readlen -1 are in the buffer.
 * Reads data in chunks that correspond to the number of bytes protected by a
 * set of ECC bytes.  Stores those complete chunks in the page buffer.
 *
 * Returns:	-EIO if it detected uncorrectable errors
 *				 As soon as it detects an uncorrectable error, it stops reading
 *			 0   if it detected no error or only correctable errors (which it
 *				 corrected).  In this case the routine adds the total number of
 *				 corrected errors to mtd->ecc_stats.corrected.
 */
static int nand_hw_ecc_read_subpage(struct mtd_info   * mtd, 
				    struct nand_chip  * chip,
				    uint32_t		data_offs, 
				    uint32_t		readlen, 
				    uint8_t	      * bufpoi)
{
	int start_step, end_step, num_steps;
	int i, n;
	int pageoffs, eccoffs;

	int eccbytes = chip->ecc.bytes;		/* # of bytes of ECC per chunk */
	int eccsize  = chip->ecc.size;		/* # of bytes per chunk */

	uint32_t  ecc_buff[ECC_HW_MAX_BYTES/4];
	uint8_t  *ecc_code = (uint8_t*)ecc_buff;
	uint32_t *eccpos   = chip->ecc.layout->eccpos;
	uint8_t  *p;

	int ret = 0;
	int err = 0;
	int total_errors = 0;
	int	o_syn[LF2000_BCH_MAX_T], e_pos[LF2000_BCH_MAX_T];
#ifdef USE_XORd_ECC
	const struct tag_bch_info * bch_info;
	const u8 * pXOR;

	bch_info = ((struct lf2000_nand_data *)chip->priv)->bch_info;
	pXOR     = bch_info->pEccX;
#endif

	/* Column address wihin the page aligned to ECC size */
	start_step = data_offs / chip->ecc.size;
	end_step   = (data_offs + readlen - 1) / chip->ecc.size;
	num_steps  = end_step - start_step + 1;

	/* Data size aligned to ECC ecc.size*/
	pageoffs = start_step * eccsize;
	p	 = bufpoi + pageoffs;		/* buffer position */
	n	 = start_step * eccbytes;	/* skip this many ecc bytes */
	eccoffs  = mtd->writesize + eccpos[n];

	DBGOUT("%s, readlen=%6d, start=%4d, ene=%4d, num=%4d, "
			"eccoffs=%4d, pageoffs=%6d, eccpoi=%d\n",
			__func__, readlen, start_step, end_step, num_steps, 
			eccoffs, pageoffs, n);

	/* read only the ecc bytes that are needed */
	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, eccoffs, -1);
	ndelay(200);	/* delay as needed by some NANDs */
	chip->read_buf(mtd, chip->oob_poi + eccpos[n], num_steps * eccbytes);

	/* read data */
	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, pageoffs, -1);
	ndelay(200);	/* delay as needed by some NANDs */

	for ( ; num_steps; num_steps--, p += eccsize) {
#ifdef USE_XORd_ECC
		for (i = 0; i < eccbytes; i++, n++)
			ecc_code[i] = chip->oob_poi[eccpos[n]] ^ pXOR[i];
#else
		for (i = 0; i < eccbytes; i++, n++)
			ecc_code[i] = chip->oob_poi[eccpos[n]];
#endif

		/* Reset the NAND controller's ECC decoder, 
		 * write ecc data to original ecc registers,
		 * and start the controller's ECC decoder */
		NX_NAND_SetResetECC(CURRENT_ECC_MODE);
		NX_NAND_SetOriECC((uint32_t*)ecc_code, CURRENT_ECC_MODE);
		NX_NAND_StartEccDecoding(CURRENT_ECC_MODE, eccsize);

		/* read data */
		chip->read_buf(mtd, p, eccsize);

		/* wait decoding  and error correct */
		NX_NAND_WaitForDecoding();

		if (NX_NAND_GetErrorStatus()) {
#ifndef USE_XORd_ECC
	/* this check is not needed when we XOR the ECC bytes */
			/* An error was detected.
			 * First check if all the ECC bytes are 0xFF.
			 * TODO: FIXME: this is a poor man's check for an erased
			 * subpage.  For completeness it would be better to 
			 * check also that all the subpage data bytes are 0xFF.
			 * Of course this is faster and probably good enough.
			 */
			int eccb = eccbytes >> 2;

			for (i = 0; eccb > i; i++)
				if (0xFFFFFFFF != ecc_buff[i])	break;

			for (i <<= 2 ; eccbytes > i; i++)
				if (0xFF != ecc_code[i]) break;

			if (i >= eccbytes)
				continue;
#endif	/* USE_XORd_ECC */

			/* Try to correct the error */
			NX_NAND_GetOddSyndrome(&o_syn[0], chip);
			ret = NX_NAND_GetErrorLocation(&o_syn[0], &e_pos[0], 
							&err, chip);

			if (0 > ret) {	/* uncorrectable errors; stop reading */
#if 0	/* 3may12 Disabled to reduce the amount of serial output */
	/* Ought to check earlier for Verbose/non-verbose, and output only
	 * if Verbose is enabled 
	 */
				ERROUT( "read_subpage: "
					"nand detect ecc errs, can't correct "
					"(step:%d, pageoffs:%d, errcnt:%d)\n",
					(end_step - start_step + 1 - num_steps),
					 pageoffs, err);
				NX_NAND_OutputNandControllerRegs();
#endif
#if 0	/* 3may12 Disabled to reduce the amount of serial output */
	/* Ought to check earlier for Verbose/non-verbose, and output only
	 * if Verbose is enabled 
	 */
				printk(KERN_INFO "All %d retries failed\n",
					MAX_CHUNK_RETRIES);
#endif
				ret = -EIO;
				break;
			} else { /* Correctable errors; flip the bad bits */
#if 0	/* 3may12 Disabled to reduce the amount of serial output */
	/* Ought to check earlier for Verbose/non-verbose, and output only
	 * if Verbose is enabled 
	 */
				printk(KERN_INFO "Subpage_read correctable"
						 " errors: nand pageoffs %d, "
						 "errors %2d, bit:\n", 
						pageoffs, err);
#endif
				FlipErrorBits( err, e_pos,eccsize,(uint32_t*)p);
				ret = 0;
				total_errors += err;
			}
		}	/* if error */
	}	/* for */

	DBGOUT("DONE %s, ret=%d\n", __func__, ret);
	if (ret < 0) {
		mtd->ecc_stats.failed++;
	} else {
		mtd->ecc_stats.corrected += total_errors;
	}
	return ret;
}


#ifndef NAND_ACCESS_TIMING
static void PrintSubpageErrorInfo(	int eccbytes, int eccsize,
					uint8_t  *ecc_code,
					uint8_t  *p)
{
	int i;

	printk(KERN_INFO "Read these ecc bytes for the subpage:\n");

	if (eccbytes == 42) {
		printk(	"  %02x %02x %02x %02x %02x %02x %02x %02x "
			"%02x %02x %02x %02x %02x %02x %02x %02x\n",
			ecc_code[0], ecc_code[1], ecc_code[2], ecc_code[3],
			ecc_code[4], ecc_code[5], ecc_code[6], ecc_code[7],
			ecc_code[8], ecc_code[9], ecc_code[10], ecc_code[11],
			ecc_code[12], ecc_code[13], ecc_code[14], ecc_code[15]);
		printk(	"  %02x %02x %02x %02x %02x %02x %02x %02x "
			"%02x %02x %02x %02x %02x %02x %02x %02x\n",
			ecc_code[16], ecc_code[17], ecc_code[18], ecc_code[19],
			ecc_code[20], ecc_code[21], ecc_code[22], ecc_code[23],
			ecc_code[24], ecc_code[25], ecc_code[28], ecc_code[27],
			ecc_code[28], ecc_code[29], ecc_code[30], ecc_code[31]);
		printk(	"  %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			ecc_code[32], ecc_code[33], ecc_code[34], ecc_code[35],
			ecc_code[36], ecc_code[37], ecc_code[38], ecc_code[39],
			ecc_code[40], ecc_code[41]);
	}
	else {
		printk(	"  %02x %02x %02x %02x %02x %02x %02x %02x "
			"%02x %02x %02x %02x %02x %02x %02x %02x\n",
			ecc_code[0], ecc_code[1], ecc_code[2], ecc_code[3],
			ecc_code[4], ecc_code[5], ecc_code[6], ecc_code[7],
			ecc_code[8], ecc_code[9], ecc_code[10], ecc_code[11],
			ecc_code[12], ecc_code[13], ecc_code[14], ecc_code[15]);
		printk(	"  %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			ecc_code[16], ecc_code[17], ecc_code[18], ecc_code[19],
			ecc_code[20], ecc_code[21], ecc_code[22], ecc_code[23],
			ecc_code[24], ecc_code[25]);
	}
	printk(KERN_INFO "Read these bytes in subpage:\n");
	for (i = 0; i < eccsize; i += 16) {
		printk(	"%02x %02x %02x %02x %02x %02x %02x %02x "
			"%02x %02x %02x %02x %02x %02x %02x %02x\n",
			*(p+i),   *(p+i+1), *(p+i+2), *(p+i+3),
			*(p+i+4), *(p+i+5), *(p+i+6), *(p+i+7),
			*(p+i+8), *(p+i+9), *(p+i+10), *(p+i+11),
			*(p+i+12), *(p+i+13), *(p+i+14), *(p+i+15));
	}
	NX_NAND_OutputNandControllerRegs();
}
#endif

/**
 * nand_hw_ecc_read_page - hardware-ecc-based page read function
 * @mtd:		mtd info structure
 * @chip:		nand chip info structure
 * @buf:		page buffer to store read data
 * @page:		index of page
 *
 * Reads the contents of the currently addressed NAND page into the 
 * specified buffer.
 *
 * Returns:	-EIO if it detected uncorrectable errors
 *		 As soon as it detects an uncorrectable error, it stops reading
 *		 0   if it detected no error or only correctable errors (which 
 *		 it corrected).  In this case the routine adds the total number 
 *		 of corrected errors to mtd->ecc_stats.corrected.
 */
// enable one of the following
#define Aug4Timing	0
//#define Aug4Timing	1
#if Aug4Timing	// 4aug11
#if 1	/* 5dec11 */
U32	NX_TIMER_GetTimerCounter( U32 ModuleIndex );
int read_current_timer(unsigned long *timer_value) {
        *timer_value = NX_TIMER_GetTimerCounter(0);
        return 1;
}
#else	/* 5dec11 */
int read_current_timer(unsigned long *timer_value);
#endif	/* 5dec11 */
#endif	// 4aug11



static int nand_hw_ecc_read_page(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int page)
{
	int i, n, ret = 0;

	int eccsteps = chip->ecc.steps;
	int eccbytes = chip->ecc.bytes;
	int eccsize  = chip->ecc.size;
static uint32_t  ecc_buff[ECC_HW_MAX_BYTES/4];
	uint8_t  *ecc_code = (uint8_t*)ecc_buff;
	uint32_t *eccpos   = chip->ecc.layout->eccpos;
	uint8_t  *p = buf;

	int err = 0;
	int errors_per_page = 0;
static int	o_syn[LF2000_BCH_MAX_T], e_pos[LF2000_BCH_MAX_T];
#ifdef USE_XORd_ECC
	const struct tag_bch_info * bch_info;
	const u8 * pXOR;

	bch_info = ((struct lf2000_nand_data *)chip->priv)->bch_info;
	pXOR     = bch_info->pEccX;
#endif

	if (512 >= mtd->writesize) {
		/* read oob */
		chip->ecc.read_oob(mtd, chip, page, 1);
		/* read data */
		chip->cmdfunc(mtd, NAND_CMD_READ0, 0x00, page);
	} else {
		/* read ECC bytes from the oob */
		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, 
				mtd->writesize + eccpos[0], -1);
		ndelay(200);	/* needed for some NANDs */
		chip->read_buf(mtd, chip->oob_poi + eccpos[0], 
				eccsteps * eccbytes);

		/* prepare to read data */
		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, 0, -1);
		ndelay(200);	/* needed for some NANDs */
	}

	for (n = 0; eccsteps; eccsteps--, p += eccsize) {
		{
			uint8_t  * dst = &ecc_code[0];
			uint8_t  * src = &chip->oob_poi[eccpos[n]];
			
#ifdef USE_XORd_ECC
			for (i = 0; i < eccbytes; i++) {
				*dst++ = *src++ ^ pXOR[i];
#else
			for (i = 0; i < eccbytes; i++) {
				*dst++ = *src++;
#endif
			}
			n += eccbytes;
		}

		/* write ecc data to orignal ecc register */
		NX_NAND_SetResetECC(CURRENT_ECC_MODE);
		NX_NAND_SetOriECC((uint32_t*)ecc_code, CURRENT_ECC_MODE);
		NX_NAND_StartEccDecoding(CURRENT_ECC_MODE, eccsize);

		/* Now read a chunk of data */
		chip->read_buf(mtd, p, eccsize);

		/* wait for the controller's decoding to finish */
		NX_NAND_WaitForDecoding();

		if (NX_NAND_GetErrorStatus()) {
#ifndef USE_XORd_ECC
			/* First check if all the ECC bytes are 0xFF.
			 * TODO: FIXME: this is a poor man's check for an erased
			 * subpage.  For completeness it would be better to 
			 * check also that all the subpage data bytes are 0xFF.  
			 * Of course this is faster and probably good enough.
			 */
			int eccb = eccbytes >> 2;

			for (i = 0; eccb > i; i++)
				if (0xFFFFFFFF != ecc_buff[i])	break;

			for (i <<= 2 ; eccbytes > i; i++)
				if (0xFF != ecc_code[i]) break;

			if (i >= eccbytes)
				continue;	// if all the ECC bytes are 0xFF
#endif

			/* Try to count errors, find them, and fix them */
			NX_NAND_GetOddSyndrome(&o_syn[0], chip);
			ret = NX_NAND_GetErrorLocation(&o_syn[0], &e_pos[0], &err, chip);

			if (0 > ret) {	/* Uncorrectable errors */
#if 1	/* 13jan12 */
#if 0	/* 3may12 Disabled to reduce the amount of serial output */
	/* Ought to check earlier for Verbose/non-verbose, and output only
	 * if Verbose is enabled 
	 */
				printk(KERN_INFO 
					"read_page: uncorrectable ECC error"
						 " in subpage %d\n", 
					chip->ecc.steps - eccsteps);
#endif
#if 0	/* this might generate lots of output */
				for (i = 0; i < eccsize; i += 16) {
				printk(KERN_INFO "%08x: %08x %08x %08x %08x\n",
					(chip->ecc.steps - eccsteps)*eccsize+i,
					*((unsigned int *)(p+i)),
					*((unsigned int *)(p+i+4)),
					*((unsigned int *)(p+i+8)),
					*((unsigned int *)(p+i+12)));
				}
				printk(KERN_INFO "Here are the ECC bytes:\n");
				for (i = 0; i < eccbytes; i += 8) {
				    printk(KERN_INFO "%02x %02x %02x %02x "
						 "%02x %02x %02x %02x\n",
					ecc_code[i],   ecc_code[i+1],
					ecc_code[i+2], ecc_code[i+3],
					ecc_code[i+4], ecc_code[i+5],
					ecc_code[i+6], ecc_code[i+7]);
				}
#endif
#endif

#if 0	/* 3may12 Disabled to reduce the amount of serial output */
	/* Ought to check earlier for Verbose/non-verbose, and output only
	 * if Verbose is enabled 
	 */
				printk(KERN_INFO "All %d retries failed\n",
					MAX_CHUNK_RETRIES);
#endif
				ret = -EIO;
				break;
			} else {	/* Correctable errors */
				FlipErrorBits( err, e_pos, eccsize, (uint32_t *)p);
				ret = 0;
				errors_per_page += err;
			}
		}	// if (NX_NAND_GetErrorStatus())
	}		// for ( eccsteps )

	if (ret < 0) {
		mtd->ecc_stats.failed++;
	} else {
		mtd->ecc_stats.corrected += errors_per_page;
	}
	return ret;
}


// TODO: FIXME: make this more efficient; see Nexell's code or nand_base.c
//				code.
    /* returns 1 if all 'len' bytes at buf are 0xff;
     *         0 if at least one of the first 'len' bytes at buf is not 0xff
     */
static int all_bytes_ff( const uint8_t * buf, int len)
{
	int rem   = (3 & (unsigned int)buf);
	int allFF = 1;

	if (rem) {
		while ( (len > 0) && (rem < 4)) {
			if (*buf++ != 0xFF) {
				allFF = 0;
				break;
			}
			++rem;
			--len;
		}
	}
	if (allFF) {
		if (0 == (3 & (unsigned int)buf)) {
			u32 * p = (u32 *)buf;

			for ( ; len > 3; len -= 4) {
				if (*p++ != 0xFFFFFFFF) {
					allFF = 0;
					break;
				}
			}
			if (allFF) {
				for (buf = (uint8_t *)p; len > 0; --len) {
					if (*buf++ != 0xFF) {
						allFF = 0;
						break;
					}
				}
			}
		} else {
			//dev_info(&nand.pdev->dev, "!@#$ all_bytes_ff()\n");
			for ( ; len > 0; --len) {
				if (*buf++ != 0xFF) {
					allFF = 0;
					break;
				}
			}
		}
	}
	return allFF;
}


static void nand_hw_ecc_write_page(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf)
{
	int i, n;
	int eccsteps = chip->ecc.steps;
	int eccbytes = chip->ecc.bytes;
	int eccsize  = chip->ecc.size;

static uint32_t  ecc_buff[ECC_HW_MAX_BYTES/4];
	uint8_t  *ecc_code = (uint8_t*)ecc_buff;
	uint32_t *eccpos   = chip->ecc.layout->eccpos;
	uint8_t  *p = (uint8_t *)buf;
#ifdef USE_XORd_ECC
	const struct tag_bch_info * bch_info;
	const u8 * pXOR;

	bch_info = ((struct lf2000_nand_data *)chip->priv)->bch_info;
	pXOR     = bch_info->pEccX;
#endif

	DBGOUT("%s\n", __func__);
#if 1	// 21jul11
	/* First check if the page is all FF; if it is, don't bother writing
	 * to the NAND.  Just verify that the nand page is all FF
	 */
	if ( !all_bytes_ff( buf, mtd->writesize) ) {
#endif	// 21jul11

	/* write data and get ecc */
	for (n = 0; eccsteps; eccsteps--, p += eccsize) {

		NX_NAND_SetResetECC(CURRENT_ECC_MODE);
		NX_NAND_StartEccEncoding(CURRENT_ECC_MODE, eccsize);

		/* write page */
		chip->write_buf(mtd, p, eccsize);

		/* get ecc code from ecc register */
		NX_NAND_WaitForEncoding();
		NX_NAND_GetGenECC((uint32_t *)ecc_code, CURRENT_ECC_MODE);

		/* set oob with ecc */
#ifdef USE_XORd_ECC
		for (i = 0; eccbytes > i; i++, n++)
			chip->oob_poi[eccpos[n]] = ecc_code[i] ^ pXOR[i];
#else
		for (i = 0; eccbytes > i; i++, n++)
			chip->oob_poi[eccpos[n]] = ecc_code[i];
#endif
	}

	/* write oob */
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
#if 1	// 21jul11
	}   /* !all_bytes_ff */
#endif	// 21jul11
// TODO: FIXME: does this routine need to have conditionally compiled code
//				that verifies after writing?
}

static int nand_hw_ecc_verify_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
#if defined (CONFIG_MTD_NAND_VERIFY_WRITE)
	struct nand_chip *chip = mtd->priv;
	int eccsteps = chip->ecc.steps;
	int eccbytes = chip->ecc.bytes;
	int eccsize  = chip->ecc.size;

	uint32_t  ecc_buff[ECC_HW_MAX_BYTES/4];
	uint8_t  *ecc_code = (uint8_t*)ecc_buff;
	uint32_t *eccpos   = chip->ecc.layout->eccpos;
	uint8_t  *p = (uint8_t *)buf;

	int eccoffs;
	int err, ret = 0;
	int	o_syn[LF2000_BCH_MAX_T], e_pos[LF2000_BCH_MAX_T];
	int i, n;
#ifdef USE_XORd_ECC
	const struct tag_bch_info * bch_info;
	const u8 * pXOR;

	bch_info = ((struct lf2000_nand_data *)chip->priv)->bch_info;
	pXOR     = bch_info->pEccX;
#endif

	if (512 >= mtd->writesize)
		return nand_sw_ecc_verify_buf(mtd, buf, len);

	eccoffs  = mtd->writesize;
	eccsteps = chip->ecc.steps;
	p = (uint8_t *)buf;

	/* note> chip->oob_poi is saved when write_page */
#if (1)
	/* read ECC bytes from oob */
	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, eccoffs + eccpos[0], -1);
	ndelay(200);	/* delay as needed by some NANDs */
	chip->read_buf(mtd, chip->oob_poi + eccpos[0], eccsteps * eccbytes);

	/* read data */
	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, 0, -1);
	ndelay(200);	/* delay as needed by some NANDs */
#endif

	for (n = 0; eccsteps; eccsteps--, p += eccsize) {

#ifdef USE_XORd_ECC
		for (i = 0; eccbytes > i; i++, n++)
			ecc_code[i] = chip->oob_poi[eccpos[n]] ^ pXOR[i];
#else
		for (i = 0; eccbytes > i; i++, n++)
			ecc_code[i] = chip->oob_poi[eccpos[n]];
#endif

		/* set orignal ecc register */
		NX_NAND_SetResetECC(CURRENT_ECC_MODE);
		NX_NAND_SetOriECC((uint32_t *)ecc_code, CURRENT_ECC_MODE);

		/* read data */
		chip->read_buf(mtd, p, eccsize);

		/* wait decoding  and error correct */
		NX_NAND_WaitForDecoding();

		if (NX_NAND_GetErrorStatus()) {
			NX_NAND_GetOddSyndrome(&o_syn[0], chip);
			ret = NX_NAND_GetErrorLocation(&o_syn[0], &e_pos[0], &err, chip);
			if (0   > ret) {
				ERROUT("nand detect ecc errs, can't correct "
					"(step:%d, errcnt:%d, ret:%d)\n",
					(chip->ecc.steps - eccsteps), err, ret);
				return -EIO;
			}
			// TODO: Either get rid of this routine or add code 
			//	 here to fix the detected errors
		}
	}
	// TODO: ...and add code here to compare what was read with what was in
	//		 the buffer.  Looks as though this routine now reads from NAND
	// into the buffer, so we'll need to change that.
	// TODO: ... OR -- maybe this routine is ok as it is -- it just checks that
	// the page can be read back without uncorrectable errors.  Not a very
	// thorough check, but maybe it's good enough.
#endif /* CONFIG_MTD_NAND_VERIFY_WRITE */
	return 0;
}

int nand_hw_ecc_check(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_ecclayout *layout = chip->ecc.layout;
	struct nand_oobfree *oobfree  = chip->ecc.layout->oobfree;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	int ecctotal = chip->ecc.total;
	int i = 0, n = 0;
	int ret = 0;

	if (512 > mtd->writesize) {
		printk(KERN_INFO "NAND ecc: page size %d not support hw ecc\n",
			mtd->writesize);
		chip->ecc.mode 		= NAND_ECC_SOFT;
		chip->ecc.read_page 	= NULL;
		chip->ecc.read_subpage 	= NULL;
		chip->ecc.write_page 	= NULL;
		chip->ecc.layout	= NULL;
		chip->verify_buf	= nand_sw_ecc_verify_buf;

		if ( chip->buffers &&
			!(chip->options & NAND_OWN_BUFFERS)) {
			/* XXX U-BOOT XXX */
			#if 0
			kfree(chip->buffers);
			#endif
			chip->buffers = NULL;
		}
		ret = nand_scan_tail(mtd);
		printk(KERN_INFO "NAND ecc: Software \n");
		return ret;
	}

	/*
	 * set ecc layout
	 */
	if (16 >= mtd->oobsize) {
		for (i = 0, n = 0; ecctotal>i; i++, n++) {
			if (5 == n) n += 1;	// Bad marker
			eccpos[i] = n;
		}
		oobfree->offset  = n;
		oobfree->length  = mtd->oobsize - ecctotal - 1;
		layout->oobavail = oobfree->length;
		if (NAND_ECC_NONE == chip->ecc.mode) {
			printk( "No ECC, bad '5', ecc 0~4,6~%d (%d),"
				" free %d~%d (%d)\n",
				ecctotal+1-1, ecctotal, oobfree->offset,
				oobfree->offset+oobfree->length-1, oobfree->length);
		}
		else if (NAND_ECC_SOFT == chip->ecc.mode) {
			printk( "S/W 1-bit ECC, bad '5', ecc 0~4,6~%d (%d),"
				" free %d~%d (%d)\n",
				ecctotal+1-1, ecctotal, oobfree->offset,
				oobfree->offset+oobfree->length-1, oobfree->length);
		}
		else {
			printk( "ECC mode %d, bad '5', ecc 0~4,6~%d (%d),"
				" free %d~%d (%d)\n",
				chip->ecc.mode, ecctotal+1-1, 
				ecctotal, oobfree->offset,
				oobfree->offset + oobfree->length-1, 
				oobfree->length);
		}
	} else {

		oobfree->offset  = 2;
		if (mtd->oobsize < ecctotal + 2) {
			printk( KERN_INFO "oobsize (%d) < ecctotal (%d) + 2\n",
					mtd->oobsize, ecctotal);
			return -1;
		}
		oobfree->length  = mtd->oobsize - ecctotal - 2;
		layout->oobavail = oobfree->length;

		n = oobfree->offset + oobfree->length;
		for (i = 0; i < ecctotal; i++, n++)
			eccpos[i] = n;

		if (NAND_ECC_NONE == chip->ecc.mode) {
			printk(KERN_INFO "No ECC, bblk '0,1', "
					 "ecc %d~%d (%d), free 2~%d (%d)\n",
				oobfree->offset+oobfree->length, n-1,
				ecctotal, oobfree->length+2-1, oobfree->length);
		}
		else if (NAND_ECC_SOFT == chip->ecc.mode) {
			printk(KERN_INFO "S/W 1-bit ECC, bblk '0,1', "
					 "ecc %d~%d (%d), free 2~%d (%d)\n",
				oobfree->offset+oobfree->length, n-1,
				ecctotal, oobfree->length+2-1, oobfree->length);
		}
		else {
			const struct tag_bch_info * bch_info;

			bch_info = ((struct lf2000_nand_data *)chip->priv)->bch_info;
			printk(KERN_INFO "H/W %d-bit ECC, bad '0,1', "
					 "ecc %d~%d (%d), free 2~%d (%d)\n",
				bch_info->bch_var_t, 
				oobfree->offset + oobfree->length, n-1,
				ecctotal, oobfree->length+2-1, oobfree->length);
		}
	}
	return ret;
}


/* This is derived from nand_default_bbt() in nand_bbt.c.
 * It is needed in order to have the bbt creator check for 0x00 in the first 
 * byte of the spare area of the first page of an eraseblock.  
 */
extern int nand_create_default_bbt_descr(struct nand_chip *this);


int lf2000_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
		                    size_t *retlen, uint8_t *buf);
int lf2000_nand_read_oob(struct mtd_info *mtd, loff_t from,
		            	 struct mtd_oob_ops *ops);
int nand_hw_ecc_init(struct mtd_info  *mtd, int hw_ecc_mode)
{
	struct nand_chip *chip = mtd->priv;
	const struct tag_bch_info * bch_info;
	struct lf2000_nand_data   * pnd;

	DBGOUT("%s\n", __func__);

	/*
	 * ecc.bytes:
	 *  4 bit ecc need  52 bit (6.5B) ecc code per 512 Byte
	 *  8 bit ecc need 104 bit (13 B) ecc code per 512 Byte
	 * 16 bit ecc need 208 bit (26 B) ecc code per 512 Byte
	 * 24 bit ecc need 336 bit (42 B) ecc code per 512 Byte
	 */
	switch (hw_ecc_mode) {
	case  4: 	bch_info = &bch_info_4_bits; 	break;
	case  8: 	bch_info = &bch_info_8_bits; 	break;
	case 12: 	bch_info = &bch_info_12_bits; 	break;
	case 16: 	bch_info = &bch_info_16_bits; 	break;
	case 24: 	bch_info = &bch_info_24_bits; 	break;
	default:
		ERROUT("not support ecc %d mode !!!\n", hw_ecc_mode);
		return -1;
	}
	pnd = (struct lf2000_nand_data *)chip->priv;
	pnd->bch_info = bch_info;

	printk(KERN_INFO "NAND ecc: %d-bit Hardware\n", hw_ecc_mode);
	chip->ecc.mode 		= NAND_ECC_HW;
	chip->ecc.read_page 	= nand_hw_ecc_read_page;
	chip->ecc.read_subpage 	= nand_hw_ecc_read_subpage;
	chip->ecc.write_page 	= nand_hw_ecc_write_page;
	chip->ecc.size 		= bch_info->bch_var_k / 8;
					// ecc calc'd over this many bytes
	chip->ecc.bytes 	= bch_info->num_ecc_bytes;
					// and generates this many ecc bytes
	/* It's ok to use static allocation of ecc.layout in u-boot,
	 * because u-boot deals with only one NAND.
	 * But Linux (and u-boot if it ever deals with both NANDs) must use
	 * separate layout structures for base and cart NAND. */
#if 0	/* 17mar12 */
	chip->ecc.layout = (struct nand_ecclayout *)
			     kzalloc(sizeof(struct nand_ecclayout), GFP_KERNEL);
#else
	chip->ecc.layout	= &nx_nand_oob;
#endif
	chip->verify_buf	= nand_hw_ecc_verify_buf;

	NX_MCUS_SetECCMode(bch_info->bch_var_t);
	NX_NAND_CreateLookupTable(chip);
		// NOTE: Even though nand_scan_tail() sometimes undoes these
		// assignments, leave them here because this routine is also 
		// called via the sysfs functions.
	mtd->read     = lf2000_nand_read;
	mtd->read_oob = lf2000_nand_read_oob;
	return 0;
}

int micron_nand_block_markbad(struct mtd_info *mtd, loff_t ofs);
int micron_nand_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip);
int lf_nand_bbt(struct mtd_info *mtd);

void lf2000_onfi(struct mtd_info *mtd, struct nand_chip *chip)
{
	struct nand_onfi_params *p = &chip->onfi_params;

printk(KERN_INFO "nand_flash_detect_onfi: mfr %s, model %s, options 0x%x\n",
		p->manufacturer, p->model, chip->options);
printk(KERN_INFO "    pgsize 0x%x, blksize 0x%x, oobsize 0x%x, chipsize 0x%x\n",
		(unsigned int)mtd->writesize, mtd->erasesize, mtd->oobsize, 
		(unsigned int)chip->chipsize);
printk(KERN_INFO "    bits per cell %d, prgms per page %d, ecc bits %d\n",
		p->bits_per_cell, p->programs_per_page, p->ecc_bits );
	if (1 == p->programs_per_page) {
		chip->options |= NAND_NO_SUBPAGE_WRITE;
	}
	if (p->ecc_bits == 1) {
		chip->ecc.mode = NAND_ECC_SOFT;
		printk(KERN_INFO "ONFI NAND ecc: Software \n");
	}
#if 0	/* to test detection of oob too small for ecc bytes */
	else if (p->ecc_bits <= 4) {
		nand_hw_ecc_init(mtd, 16);
	}
#endif
	else if (p->ecc_bits <= 4) {
		nand_hw_ecc_init(mtd, 4);
	}
	else if (p->ecc_bits <= 8) {
		nand_hw_ecc_init(mtd, 8);
	}
	else if (p->ecc_bits <= 12) {
		nand_hw_ecc_init(mtd, 12);
	}
	else if (p->ecc_bits <= 16) {
		nand_hw_ecc_init(mtd, 16);
	}
	else {
		nand_hw_ecc_init(mtd, 24);
	}
	/* nand_scan() in drivers/mtd/nand/nand_base.c calls nand_hw_ecc_check()
	 * Among other things, it checks if the oob is big enough to hold all
	 * the ECC bytes we think are needed.
	 */
	/* Check that this is a Micron NAND */
	/* TODO: FIXME: if it isn't Micron, we probably ought to specify
	 * other functions for chip->block_bad, etc.
	 */
	if (0 != strcmp("MICRON", p->manufacturer)) {
		printk(KERN_INFO "\nUnexpected non-Micron ONFI NAND\n");
	}
	chip->block_bad	    = micron_nand_block_bad;
	chip->block_markbad = micron_nand_block_markbad;
	chip->scan_bbt	    = lf_nand_bbt;
}


#endif /* CONFIG_MTD_NAND_NEXELL_HWECC */

