/*
 * (C) Copyright 2008 Semihalf
 *
 * Written by: Piotr Ziecik <kosmo@semihalf.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

#include <common.h>
#include <flash.h>
#include <malloc.h>

#include <asm/errno.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/concat.h>

/* use CONFIG_SYS_MAX_FLASH_BANKS_DETECT if defined */
#ifdef CONFIG_SYS_MAX_FLASH_BANKS_DETECT
# define CFI_MAX_FLASH_BANKS	CONFIG_SYS_MAX_FLASH_BANKS_DETECT
#else
# define CFI_MAX_FLASH_BANKS	CONFIG_SYS_MAX_FLASH_BANKS
#endif

extern flash_info_t flash_info[];

static struct mtd_info cfi_mtd_info[CFI_MAX_FLASH_BANKS];
static char cfi_mtd_names[CFI_MAX_FLASH_BANKS][16];
#ifdef CONFIG_MTD_CONCAT
static char c_mtd_name[16];
#endif

static int cfi_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	flash_info_t *fi = mtd->priv;
	size_t a_start = fi->start[0] + instr->addr;
	size_t a_end = a_start + instr->len;
	int s_first = -1;
	int s_last = -1;
	int error, sect;

	for (sect = 0; sect < fi->sector_count; sect++) {
		if (a_start == fi->start[sect])
			s_first = sect;

		if (sect < fi->sector_count - 1) {
			if (a_end == fi->start[sect + 1]) {
				s_last = sect;
				break;
			}
		} else {
			s_last = sect;
			break;
		}
	}

	if (s_first >= 0 && s_first <= s_last) {
		instr->state = MTD_ERASING;

		flash_set_verbose(0);
		error = flash_erase(fi, s_first, s_last);
		flash_set_verbose(1);

		if (error) {
			instr->state = MTD_ERASE_FAILED;
			return -EIO;
		}

		instr->state = MTD_ERASE_DONE;
		mtd_erase_callback(instr);
		return 0;
	}

	return -EINVAL;
}

static int cfi_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	flash_info_t *fi = mtd->priv;
	u_char *f = (u_char*)(fi->start[0]) + from;

	memcpy(buf, f, len);
	*retlen = len;

	return 0;
}

static int cfi_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
	flash_info_t *fi = mtd->priv;
	u_long t = fi->start[0] + to;
	int error;

	flash_set_verbose(0);
	error = write_buff(fi, (u_char*)buf, t, len);
	flash_set_verbose(1);

	if (!error) {
		*retlen = len;
		return 0;
	}

	return -EIO;
}

static void cfi_mtd_sync(struct mtd_info *mtd)
{
	/*
	 * This function should wait until all pending operations
	 * finish. However this driver is fully synchronous, so
	 * this function returns immediately
	 */
}

static int cfi_mtd_lock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	flash_info_t *fi = mtd->priv;

	flash_set_verbose(0);
	flash_protect(FLAG_PROTECT_SET, fi->start[0] + ofs,
					fi->start[0] + ofs + len - 1, fi);
	flash_set_verbose(1);

	return 0;
}

static int cfi_mtd_unlock(struct mtd_info *mtd, loff_t ofs, uint64_t len)
{
	flash_info_t *fi = mtd->priv;

	flash_set_verbose(0);
	flash_protect(FLAG_PROTECT_CLEAR, fi->start[0] + ofs,
					fi->start[0] + ofs + len - 1, fi);
	flash_set_verbose(1);

	return 0;
}

static int cfi_mtd_set_erasesize(struct mtd_info *mtd, flash_info_t *fi)
{
	int sect_size = 0;
	int sect_size_old = 0;
	int sect;
	int regions = 0;
	int numblocks = 0;
	ulong offset;
	ulong base_addr;

	/*
	 * First detect the number of eraseregions so that we can allocate
	 * the array of eraseregions correctly
	 */
	for (sect = 0; sect < fi->sector_count; sect++) {
		if (sect_size_old != flash_sector_size(fi, sect))
			regions++;
		sect_size_old = flash_sector_size(fi, sect);
	}

	switch (regions) {
	case 0:
		return 1;
	case 1:	/* flash has uniform erase size */
		mtd->numeraseregions = 0;
		mtd->erasesize = sect_size_old;
		return 0;
	}

	mtd->numeraseregions = regions;
	mtd->eraseregions = malloc(sizeof(struct mtd_erase_region_info) * regions);

	/*
	 * Now detect the largest sector and fill the eraseregions
	 */
	regions = 0;
	base_addr = offset = fi->start[0];
	sect_size_old = flash_sector_size(fi, 0);
	for (sect = 0; sect < fi->sector_count; sect++) {
		if (sect_size_old != flash_sector_size(fi, sect)) {
			mtd->eraseregions[regions].offset = offset - base_addr;
			mtd->eraseregions[regions].erasesize = sect_size_old;
			mtd->eraseregions[regions].numblocks = numblocks;
			/* Now start counting the next eraseregions */
			numblocks = 0;
			regions++;
			offset = fi->start[sect];
		}
		numblocks++;

		/*
		 * Select the largest sector size as erasesize (e.g. for UBI)
		 */
		if (flash_sector_size(fi, sect) > sect_size)
			sect_size = flash_sector_size(fi, sect);

		sect_size_old = flash_sector_size(fi, sect);
	}

	/*
	 * Set the last region
	 */
	mtd->eraseregions[regions].offset = offset - base_addr;
	mtd->eraseregions[regions].erasesize = sect_size_old;
	mtd->eraseregions[regions].numblocks = numblocks;

	mtd->erasesize = sect_size;

	return 0;
}

#ifdef CONFIG_NAND_BBT_IN_NOR
#if 0	/* 30jan12; disabled  */
static unsigned char nor_buf[512];
static void read_nor_sector(struct mtd_info *mtd) {
	size_t num_read;
	int stat;
	int i;
	stat = mtd->read( mtd, mtd->size - 3 * 4096, sizeof(nor_buf),
			  &num_read, nor_buf);
	printf("mtd->read: stat %d, num_read %d\n", stat, num_read);
	for ( i = 0; i < sizeof(nor_buf); i += 16) {
		printf("%3d: %02x %02x %02x %02x %02x %02x %02x %02x"
			" %02x %02x %02x %02x %02x %02x %02x %02x\n",
			i, 
			nor_buf[i],    nor_buf[i+1],  nor_buf[i+2],  nor_buf[i+3],
			nor_buf[i+4],  nor_buf[i+5],  nor_buf[i+6],  nor_buf[i+7],
			nor_buf[i+8],  nor_buf[i+9],  nor_buf[i+10], nor_buf[i+11],
			nor_buf[i+12], nor_buf[i+13], nor_buf[i+14], nor_buf[i+15]);
	}
}
static void write_nor_sector(struct mtd_info *mtd) {
	size_t num_written;
	int stat;
	int i;

	for (i = 0; i < sizeof(nor_buf); ++i) {
		if (i & 1)
			nor_buf[i] <<= 1;
		else
			nor_buf[i] >>= 1;
	}
	stat = mtd->write( mtd, mtd->size - 3 * 4096, sizeof(nor_buf),
			  &num_written, nor_buf);
	printf("mtd->write: stat %d, num_written %d\n", stat, num_written);
}
static void erase_nor_sector(struct mtd_info *mtd) {
	int stat;
	struct erase_info erase_inst;

/* TODO: FIXME: (31jan12) get size of eraseblock from mtd */
	erase_inst.addr	= mtd->size - 3 * 4096;
	erase_inst.len  = 4096; /* sizeof(nor_buf); gives errors */
	erase_inst.mtd  = mtd;
	erase_inst.callback = NULL;

	stat = mtd->erase(mtd, &erase_inst);
	printf("mtd->erase: stat %d; addr 0x%x, len %d, state %d\n", 
		stat, erase_inst.addr, erase_inst.len, erase_inst.state);
}
#endif

#if 1	/* 20feb12 */
struct mtd_info * get_nor_mtd(void) {
	struct mtd_info *mtd;

	mtd = &cfi_mtd_info[0];
	if (   (mtd->name[0] == 'n')
	    && (mtd->name[1] == 'o')
	    && (mtd->name[2] == 'r')
	    && (mtd->type    == MTD_NORFLASH)
	    && (mtd->flags   == MTD_CAP_NORFLASH)
	    && (mtd->size    != 0))
		return mtd;
	return NULL;
}
#endif	/* 20feb12 */
#endif

int cfi_mtd_init(void)
{
	struct mtd_info *mtd;
	flash_info_t *fi;
	int error, i;
	int devices_found = 0;
	struct mtd_info *mtd_list[CONFIG_SYS_MAX_FLASH_BANKS];

	for (i = 0; i < CONFIG_SYS_MAX_FLASH_BANKS; i++) {
		fi = &flash_info[i];
		mtd = &cfi_mtd_info[i];

		memset(mtd, 0, sizeof(struct mtd_info));

		error = cfi_mtd_set_erasesize(mtd, fi);
		if (error)
			continue;

		sprintf(cfi_mtd_names[i], "nor%d", i);
		mtd->name		= cfi_mtd_names[i];
		mtd->type		= MTD_NORFLASH;
		mtd->flags		= MTD_CAP_NORFLASH;
		mtd->size		= fi->size;
		mtd->writesize		= 1;

		mtd->erase		= cfi_mtd_erase;
		mtd->read		= cfi_mtd_read;
		mtd->write		= cfi_mtd_write;
		mtd->sync		= cfi_mtd_sync;
		mtd->lock		= cfi_mtd_lock;
		mtd->unlock		= cfi_mtd_unlock;
		mtd->priv		= fi;
#ifdef CONFIG_NAND_BBT_IN_NOR
printf("cfi_mtd_init#1; mtd->name: '%s'\n", mtd->name);
printf("  flash_info_t.size %d\n", (unsigned int)fi->size);
printf("  flash_info_t.sector_count %d\n", (unsigned int)fi->sector_count);
printf("  flash_info_t.portwidth %d\n", (unsigned int)fi->portwidth);
printf("  flash_info_t.chipwidth %d\n", (unsigned int)fi->chipwidth);
#endif
#ifdef CONFIG_NAND_BBT_IN_NOR
if (0 == fi->buffer_size) fi->buffer_size = 1;	/* force this */
printf("  flash_info_t.buffer_size %d\n", (unsigned long)fi->buffer_size);
#endif

		if (add_mtd_device(mtd))
			return -ENOMEM;

		mtd_list[devices_found++] = mtd;
	}

#ifdef CONFIG_MTD_CONCAT
	if (devices_found > 1) {
		/*
		 * We detected multiple devices. Concatenate them together.
		 */
		sprintf(c_mtd_name, "nor%d", devices_found);
		mtd = mtd_concat_create(mtd_list, devices_found, c_mtd_name);

		if (mtd == NULL)
			return -ENXIO;

		if (add_mtd_device(mtd))
			return -ENOMEM;
	}
#endif /* CONFIG_MTD_CONCAT */

	return 0;
}
