/*
 * (C) Copyright 2003
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
 * Boot support
 */
#include <common.h>
#include <command.h>

#if defined(CONFIG_ARCH_LF2000)
#include <lf2000_board.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

static void print_num(const char *, ulong);

#if !(defined(CONFIG_ARM) || defined(CONFIG_M68K)) || defined(CONFIG_CMD_NET)
static void print_eth(int idx);
#endif

#ifndef CONFIG_ARM	/* PowerPC and other */
static void print_lnum(const char *, u64);

#ifdef CONFIG_PPC
static void print_str(const char *, const char *);

int do_bdinfo ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	bd_t *bd = gd->bd;
	char buf[32];

#ifdef DEBUG
	print_num ("bd address",    (ulong)bd		);
#endif
	print_num ("memstart",	    bd->bi_memstart	);
	print_lnum ("memsize", 	    bd->bi_memsize	);
	print_num ("flashstart",    bd->bi_flashstart	);
	print_num ("flashsize",	    bd->bi_flashsize	);
	print_num ("flashoffset",   bd->bi_flashoffset	);
	print_num ("sramstart",	    bd->bi_sramstart	);
	print_num ("sramsize",	    bd->bi_sramsize	);
#if defined(CONFIG_5xx)  || defined(CONFIG_8xx) || \
    defined(CONFIG_8260) || defined(CONFIG_E500)
	print_num ("immr_base",	    bd->bi_immr_base	);
#endif
	print_num ("bootflags",	    bd->bi_bootflags	);
#if defined(CONFIG_405GP) || defined(CONFIG_405CR) || \
    defined(CONFIG_405EP) || defined(CONFIG_XILINX_405) || \
    defined(CONFIG_440EP) || defined(CONFIG_440GR) || \
    defined(CONFIG_440EPX) || defined(CONFIG_440GRX) ||	\
    defined(CONFIG_440SP) || defined(CONFIG_440SPE)
	print_str ("procfreq",	    strmhz(buf, bd->bi_procfreq));
	print_str ("plb_busfreq",   strmhz(buf, bd->bi_plb_busfreq));
#if defined(CONFIG_405GP) || defined(CONFIG_405EP) || defined(CONFIG_XILINX_405) || \
    defined(CONFIG_440EP) || defined(CONFIG_440GR) || defined(CONFIG_440SPE) || \
    defined(CONFIG_440EPX) || defined(CONFIG_440GRX)
	print_str ("pci_busfreq",   strmhz(buf, bd->bi_pci_busfreq));
#endif
#else	/* ! CONFIG_405GP, CONFIG_405CR, CONFIG_405EP, CONFIG_XILINX_405, CONFIG_440EP CONFIG_440GR */
#if defined(CONFIG_CPM2)
	print_str ("vco",	    strmhz(buf, bd->bi_vco));
	print_str ("sccfreq",	    strmhz(buf, bd->bi_sccfreq));
	print_str ("brgfreq",	    strmhz(buf, bd->bi_brgfreq));
#endif
	print_str ("intfreq",	    strmhz(buf, bd->bi_intfreq));
#if defined(CONFIG_CPM2)
	print_str ("cpmfreq",	    strmhz(buf, bd->bi_cpmfreq));
#endif
	print_str ("busfreq",	    strmhz(buf, bd->bi_busfreq));
#endif /* CONFIG_405GP, CONFIG_405CR, CONFIG_405EP, CONFIG_XILINX_405, CONFIG_440EP CONFIG_440GR */
#if defined(CONFIG_MPC8220)
	print_str ("inpfreq",	    strmhz(buf, bd->bi_inpfreq));
	print_str ("flbfreq",	    strmhz(buf, bd->bi_flbfreq));
	print_str ("pcifreq",	    strmhz(buf, bd->bi_pcifreq));
	print_str ("vcofreq",	    strmhz(buf, bd->bi_vcofreq));
	print_str ("pevfreq",	    strmhz(buf, bd->bi_pevfreq));
#endif

	print_eth(0);
#if defined(CONFIG_HAS_ETH1)
	print_eth(1);
#endif
#if defined(CONFIG_HAS_ETH2)
	print_eth(2);
#endif
#if defined(CONFIG_HAS_ETH3)
	print_eth(3);
#endif
#if defined(CONFIG_HAS_ETH4)
	print_eth(4);
#endif
#if defined(CONFIG_HAS_ETH5)
	print_eth(5);
#endif

#ifdef CONFIG_HERMES
	print_str ("ethspeed",	    strmhz(buf, bd->bi_ethspeed));
#endif
	printf ("IP addr     = %pI4\n", &bd->bi_ip_addr);
	printf ("baudrate    = %6ld bps\n", bd->bi_baudrate   );
	print_num ("relocaddr", gd->relocaddr);
	return 0;
}

#elif defined(CONFIG_NIOS2) /* Nios-II */

int do_bdinfo ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	bd_t *bd = gd->bd;

	print_num ("mem start",		(ulong)bd->bi_memstart);
	print_lnum ("mem size",		(u64)bd->bi_memsize);
	print_num ("flash start",	(ulong)bd->bi_flashstart);
	print_num ("flash size",	(ulong)bd->bi_flashsize);
	print_num ("flash offset",	(ulong)bd->bi_flashoffset);

#if defined(CONFIG_SYS_SRAM_BASE)
	print_num ("sram start",	(ulong)bd->bi_sramstart);
	print_num ("sram size",		(ulong)bd->bi_sramsize);
#endif

#if defined(CONFIG_CMD_NET)
	print_eth(0);
	printf ("ip_addr     = %pI4\n", &bd->bi_ip_addr);
#endif

	printf ("baudrate    = %ld bps\n", bd->bi_baudrate);

	return 0;
}
#elif defined(CONFIG_MICROBLAZE) /* ! PPC, which leaves Microblaze */

int do_bdinfo ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	bd_t *bd = gd->bd;
	print_num ("mem start      ",	(ulong)bd->bi_memstart);
	print_lnum ("mem size       ",	(u64)bd->bi_memsize);
	print_num ("flash start    ",	(ulong)bd->bi_flashstart);
	print_num ("flash size     ",	(ulong)bd->bi_flashsize);
	print_num ("flash offset   ",	(ulong)bd->bi_flashoffset);
#if defined(CONFIG_SYS_SRAM_BASE)
	print_num ("sram start     ",	(ulong)bd->bi_sramstart);
	print_num ("sram size      ",	(ulong)bd->bi_sramsize);
#endif
#if defined(CONFIG_CMD_NET)
	print_eth(0);
	printf ("ip_addr     = %pI4\n", &bd->bi_ip_addr);
#endif
	printf ("baudrate    = %ld bps\n", (ulong)bd->bi_baudrate);
	return 0;
}

#elif defined(CONFIG_SPARC)	/* SPARC */
int do_bdinfo(cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
	bd_t *bd = gd->bd;

#ifdef DEBUG
	print_num("bd address             ", (ulong) bd);
#endif
	print_num("memstart               ", bd->bi_memstart);
	print_lnum("memsize                ", bd->bi_memsize);
	print_num("flashstart             ", bd->bi_flashstart);
	print_num("CONFIG_SYS_MONITOR_BASE       ", CONFIG_SYS_MONITOR_BASE);
	print_num("CONFIG_ENV_ADDR           ", CONFIG_ENV_ADDR);
	printf("CONFIG_SYS_RELOC_MONITOR_BASE = 0x%lx (%d)\n", CONFIG_SYS_RELOC_MONITOR_BASE,
	       CONFIG_SYS_MONITOR_LEN);
	printf("CONFIG_SYS_MALLOC_BASE        = 0x%lx (%d)\n", CONFIG_SYS_MALLOC_BASE,
	       CONFIG_SYS_MALLOC_LEN);
	printf("CONFIG_SYS_INIT_SP_OFFSET     = 0x%lx (%d)\n", CONFIG_SYS_INIT_SP_OFFSET,
	       CONFIG_SYS_STACK_SIZE);
	printf("CONFIG_SYS_PROM_OFFSET        = 0x%lx (%d)\n", CONFIG_SYS_PROM_OFFSET,
	       CONFIG_SYS_PROM_SIZE);
	printf("CONFIG_SYS_GBL_DATA_OFFSET    = 0x%lx (%d)\n", CONFIG_SYS_GBL_DATA_OFFSET,
	       CONFIG_SYS_GBL_DATA_SIZE);

#if defined(CONFIG_CMD_NET)
	print_eth(0);
	printf("ip_addr     = %pI4\n", &bd->bi_ip_addr);
#endif
	printf("baudrate               = %6ld bps\n", bd->bi_baudrate);
	return 0;
}

#elif defined(CONFIG_M68K) /* M68K */
static void print_str(const char *, const char *);

int do_bdinfo ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	bd_t *bd = gd->bd;
	char buf[32];

	print_num ("memstart",		(ulong)bd->bi_memstart);
	print_lnum ("memsize",		(u64)bd->bi_memsize);
	print_num ("flashstart",	(ulong)bd->bi_flashstart);
	print_num ("flashsize",		(ulong)bd->bi_flashsize);
	print_num ("flashoffset",	(ulong)bd->bi_flashoffset);
#if defined(CONFIG_SYS_INIT_RAM_ADDR)
	print_num ("sramstart",		(ulong)bd->bi_sramstart);
	print_num ("sramsize",		(ulong)bd->bi_sramsize);
#endif
#if defined(CONFIG_SYS_MBAR)
	print_num ("mbar",		bd->bi_mbar_base);
#endif
	print_str ("cpufreq",		strmhz(buf, bd->bi_intfreq));
	print_str ("busfreq",		strmhz(buf, bd->bi_busfreq));
#ifdef CONFIG_PCI
	print_str ("pcifreq",		strmhz(buf, bd->bi_pcifreq));
#endif
#ifdef CONFIG_EXTRA_CLOCK
	print_str ("flbfreq",		strmhz(buf, bd->bi_flbfreq));
	print_str ("inpfreq",		strmhz(buf, bd->bi_inpfreq));
	print_str ("vcofreq",		strmhz(buf, bd->bi_vcofreq));
#endif
#if defined(CONFIG_CMD_NET)
	print_eth(0);
#if defined(CONFIG_HAS_ETH1)
	print_eth(1);
#endif
#if defined(CONFIG_HAS_ETH2)
	print_eth(2);
#endif
#if defined(CONFIG_HAS_ETH3)
	print_eth(3);
#endif

	printf ("ip_addr     = %pI4\n", &bd->bi_ip_addr);
#endif
	printf ("baudrate    = %ld bps\n", bd->bi_baudrate);

	return 0;
}

#elif defined(CONFIG_BLACKFIN)
static void print_str(const char *, const char *);

int do_bdinfo(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	bd_t *bd = gd->bd;
	char buf[32];

	printf("U-Boot      = %s\n", bd->bi_r_version);
	printf("CPU         = %s\n", bd->bi_cpu);
	printf("Board       = %s\n", bd->bi_board_name);
	print_str("VCO",         strmhz(buf, bd->bi_vco));
	print_str("CCLK",        strmhz(buf, bd->bi_cclk));
	print_str("SCLK",        strmhz(buf, bd->bi_sclk));

	print_num("boot_params", (ulong)bd->bi_boot_params);
	print_num("memstart",    (ulong)bd->bi_memstart);
	print_lnum("memsize",    (u64)bd->bi_memsize);
	print_num("flashstart",  (ulong)bd->bi_flashstart);
	print_num("flashsize",   (ulong)bd->bi_flashsize);
	print_num("flashoffset", (ulong)bd->bi_flashoffset);

	print_eth(0);
	printf("ip_addr     = %pI4\n", &bd->bi_ip_addr);
	printf("baudrate    = %d bps\n", bd->bi_baudrate);

	return 0;
}

#else /* ! PPC, which leaves MIPS */

int do_bdinfo ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	bd_t *bd = gd->bd;

	print_num ("boot_params",	(ulong)bd->bi_boot_params);
	print_num ("memstart",		(ulong)bd->bi_memstart);
	print_lnum ("memsize",		(u64)bd->bi_memsize);
	print_num ("flashstart",	(ulong)bd->bi_flashstart);
	print_num ("flashsize",		(ulong)bd->bi_flashsize);
	print_num ("flashoffset",	(ulong)bd->bi_flashoffset);

	print_eth(0);
	printf ("ip_addr     = %pI4\n", &bd->bi_ip_addr);
	printf ("baudrate    = %d bps\n", bd->bi_baudrate);

	return 0;
}
#endif  /* MIPS */

#else	/* ARM */

int do_bdinfo ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int i;
	bd_t *bd = gd->bd;
#if defined(CONFIG_ARCH_LF2000)
	U32	idreg[4];
	char	chipname[49];
	struct {
		U32	data1;
		U16	data2;
		U16	data3;
		U8	data4[8];
	} __attribute__ ((packed)) guid;

	U32	lotID;			// decode ID reg data
	U32	wafer;
	U32	xpos;
	U32	ypos;
	U32	serial;
	U32	usercode;
	U32	fixed1;
	U32	parity;
	U32	invertlo;
	U32	inverthi;
	U32	fixed2;

	NX_ECID_GetECID(idreg);		// get ECID
	NX_ECID_GetChipName(chipname);	// get chip name
	chipname[48]=0;			// ensure zero terminated
	NX_ECID_GetGUID((U32 *)&guid);	// get GUID

	// pick apart ID register
	lotID	=   (idreg[0] >>  0) & 0x1FFFFF;// 21 bits
	wafer	=   (idreg[0] >> 21) & 0x1F;	//  5 bits
	xpos	= (((idreg[1] >>  0) & 0x3) << 6) |
		      ((idreg[0] >> 26) & 0x3F);//  8 bits
	ypos	=   (idreg[1] >>  2) & 0xFF;	//  8 bits
	serial	=   (idreg[1] >> 10) & 0x1FFF;	// 13 bits
	usercode=   (idreg[1] >> 23) & 0x7F;	//  7 bits
	fixed1	=   (idreg[1] >> 30) & 0x1;	//  1 bit
	parity	=   (idreg[1] >> 31) & 0x1;	//  1 bit
	invertlo=    idreg[2];			// 32 bits
	inverthi=   (idreg[3] >>  0) & 0xFFFF;	// 16 bits
	fixed2  =   (idreg[4] >> 16) & 0xFFFF;	// 16 bits

	
#endif

	print_num ("arch_number",	bd->bi_arch_number);
	print_num ("env_t",		(ulong)bd->bi_env);
	print_num ("boot_params",	(ulong)bd->bi_boot_params);

	for (i=0; i<CONFIG_NR_DRAM_BANKS; ++i) {
		print_num("DRAM bank",	i);
		print_num("-> start",	bd->bi_dram[i].start);
		print_num("-> size",	bd->bi_dram[i].size);
	}

#if defined(CONFIG_ARCH_LF2000)
	printf ("%-12s= 0x%8.8X-%8.8X-%8.8X-%8.8X  [127:0]\n",
		"ECID ", idreg[3], idreg[2], idreg[1], idreg[0]);
	printf   ("              xpos:0x%2.2X  ypos:0x%2.2X  wafer:0x%2.2X  lotID:0x%6.6X\n",
		xpos, ypos, wafer, lotID);
	print_num("  serial",	serial);
	print_num("  user",	usercode);
	print_num("  fixed1",	fixed1);
	print_num("  parity",	parity);
	printf   ("%-12s= 0x%4.4X-%4.4X-%4.4X\n", "ECID invert",
			inverthi, (invertlo >> 16) & 0xFFFF, invertlo & 0xFFFF);
	print_num("  fixed2",	fixed2);

	printf ("%-12s= %s\n", "chip name", chipname);
	printf ("%-12s= 0x%8.8X-%4.4X-%4.4X-%2.2X-%2.2X-%2.2X-%2.2X-%2.2X-%2.2X-%2.2X-%2.2X\n",
		"guid", guid.data1, guid.data2, guid.data3,
		guid.data4[0], guid.data4[1], guid.data4[2], guid.data4[3],
		guid.data4[4], guid.data4[5], guid.data4[6], guid.data4[7]);

	print_num ("EC Bonding", 	NX_ECID_GetBondingID());	
	print_num ("board_id",		get_board_rev());
#endif

#if defined(CONFIG_CMD_NET)
	print_eth(0);
	printf ("ip_addr     = %pI4\n", &bd->bi_ip_addr);
#endif
	printf ("baudrate    = %d bps\n", bd->bi_baudrate);

	return 0;
}

#endif /* CONFIG_ARM XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX */

static void print_num(const char *name, ulong value)
{
	printf ("%-12s= 0x%08lX\n", name, value);
}

#if !(defined(CONFIG_ARM) || defined(CONFIG_M68K)) || defined(CONFIG_CMD_NET)
static void print_eth(int idx)
{
	char name[10], *val;
	if (idx)
		sprintf(name, "eth%iaddr", idx);
	else
		strcpy(name, "ethaddr");
	val = getenv(name);
	if (!val)
		val = "(not set)";
	printf("%-12s= %s\n", name, val);
}
#endif

#ifndef CONFIG_ARM
static void print_lnum(const char *name, u64 value)
{
	printf ("%-12s= 0x%.8llX\n", name, value);
}
#endif

#if defined(CONFIG_PPC) || defined(CONFIG_M68K) || defined(CONFIG_BLACKFIN)
static void print_str(const char *name, const char *str)
{
	printf ("%-12s= %6s MHz\n", name, str);
}
#endif	/* CONFIG_PPC */


/* -------------------------------------------------------------------- */

U_BOOT_CMD(
	bdinfo,	1,	1,	do_bdinfo,
	"print Board Info structure",
	""
);
