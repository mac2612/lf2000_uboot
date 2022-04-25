/*
 *  drivers/mtd/nand/lf2000.h
 *
 * Copyright 2011 LeapFrog Enterprises Inc.
 *
 * Leapfrog Firmware <firmware@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef LF2000_H
#define LF2000_H

/* XXX U-BOOT XXX */
#if 0
#include <linux/semaphore.h>
#include <mach/soc.h>
#endif


// TODO: FIXME:	Decide if we ought to add a new option to Kconfig
/* XXX U-BOOT XXX */
#if 0
#define CONFIG_MTD_NAND_LF2000_PROF	1
#endif

#define LF2000_BASE_NAND_BANK	0
#define LF2000_CART_NAND_BANK	1

#define LF2000_BCH_MAX_T	24


	/* lf2000_nand_data is derived from plat_nand_data, in nand/plat_nand.d */
	/* NOTE: we follow the convention that mtd.priv points to chip, and
	 *		 chip.priv points to the enclosing struct lf2000_nand_data.
	 */
struct lf2000_nand_data {
	struct nand_chip		chip;
	struct mtd_info			mtd;
	int				nand_bank;
	int				bbt_in_nor;
	void __iomem		      * io_base;
	const struct tag_bch_info     * bch_info;
#ifdef CONFIG_MTD_PARTITIONS
	int				nr_parts;
	struct mtd_partition	      * parts;
#endif
};


struct lf2000_nand_devices {
	void __iomem     	* mem;	/* NAND controller IO memory */
	struct lf2000_nand_data * onboard_nand;
	struct lf2000_nand_data * cart_nand;
	struct platform_device	* pdev;

	struct nand_hw_control	controller;
	int                  	cart_ready;
	int                 	cart_ubi;
/* XXX U-BOOT XXX */
#if 0
	struct semaphore    		   sem_hotswap;
#endif

	int	  * L;			/* [bch_max_t * 2 + 3] */
	int	  * u_L;		/* [bch_max_t*2 + 3] */
	int	  * reg;		/* [bch_max_t + 3] */
	int	  * s;			/* [bch_max_t * 2 + 1] */
	int	  * elp;		/* [bch_max_t*2 + 4][bch_max_t*2 + 4] */
	int	  * desc;		/* [bch_max_t*2 + 4] */
	struct dma_trans * tr;
};

extern struct lf2000_nand_devices lf2000_nand;

#define L_ENTRIES	(2*LF2000_BCH_MAX_T + 3)
#define uL_ENTRIES	(2*LF2000_BCH_MAX_T + 3)
#define reg_ENTRIES	(LF2000_BCH_MAX_T + 3)
#define s_ENTRIES	(2*LF2000_BCH_MAX_T + 1)
#define elp_ENTRIES	((2*LF2000_BCH_MAX_T + 4)*(2*LF2000_BCH_MAX_T + 4))
#define desc_ENTRIES	(2*LF2000_BCH_MAX_T + 4)
#define COMMON_BUFFER_SIZE \
		 sizeof(int)*(L_ENTRIES + uL_ENTRIES + reg_ENTRIES \
			      + s_ENTRIES + elp_ENTRIES + desc_ENTRIES)

struct tag_bch_arrays {
	int	bch_var_m;		/* log2(order of the galois field) */
	int	bch_var_n;		/* (2**bch_var_m) - 1 */
	short * bch_AlphaToTable;
	short * bch_IndexOfTable;
	int   * L;			/* [bch_var_t * 2 + 3] */
	int   * u_L;			/* [bch_var_t * 2 + 3] */
	int   * reg;			/* [bch_var_t + 3] */
	int   * s;			/* [bch_var_t * 2 + 1] */
	int   * elp;			/* [bch_var_t*2 + 4][bch_var_t*2 + 4] */
	int   * desc;			/* [bch_var_t*2 + 4] */
};


struct tag_bch_info {
	int bch_var_t;		/* max number of correctable bit errors */
	int bch_var_k;		/* number of bits in which 'bch_var_t' errors 
				 * can be corrected */
	int bch_var_m;		/* log2(order of the galois field) -- smallest X
				 * such that 2**X > 'bch_var_k' */
	int num_ecc_bytes;	/* number of bch ecc bytes needed to correct 
				 * 'bch_var_t' bit errors in 'bch_var_k' bits */
	const u8 * pEccX;	/* address of array of 'num_ecc_bytes' bytes
				 * that are XOR'd with the ECC values 
				 * calc'd by the nand controller; the results
				 * of the XOR are stored in NAND's OOB.  After
				 * reading ECC bytes from NAND's OOB, they are
				 * XOR'd with these values before loading them
				 * into the nand controller's ECC registers for
				 * decoding.
				 */
	struct tag_bch_arrays * p_arrays;
};

#endif /* LF2000_H*/


