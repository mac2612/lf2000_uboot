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
#include <config.h>
#include <common.h>

/* nexell soc headers */
#include <platform.h>
#include <platfunc.h>
#include <lf2000_board.h>
#include <mfgdata.h>
#include <lcd.h>
#include <board_revisions.h>
#include "eth.c"
#include <mach/lfp100.h>
#include <screen-list.h>

u32 get_board_rev(void);

/* debug print for hw status */
#ifdef DEBUG_REG
static void dbg_reg_dpc(int dev);
static void dbg_reg_mlc(int dev);
#endif

/* debug macro */
#if 0
#define DBGOUT(msg...)		{ printf(msg); }
#else
#define DBGOUT(msg...)		{ ; }
#endif

#if defined(CONFIG_MACH_NXP3200_RIO_DTK)
/* SP 04-16-13: If you are changing low battery threshold here, 
 * make sure you also change it in these two files to maintain it 
 * across boot, kernel and battery voltage logging.
 * Battery logger - LinuxDist_LF2000/packages/optimization/bat-logger.sh
 * HW Monitor - nxp3200_bsp/kernel/linux-3.4/arch/arm/mach-nxp3200/include/mach/power.h
 */
#define BATTERY_LOW_BOOT	3600		/* low battery - raw ADC reading */ 
#define ADC_SLOPE_256_DEFAULT	266     // (4250 / 4096) * 256

#else

#define BATTERY_LOW_BOOT	4400		/* low battery - raw ADC reading */ 
#define ADC_SLOPE_256_DEFAULT	502		/* (8023 mV / 4096) * 256 */

#endif

#define ADC_BATTERY		2		/* ADC channel connected to battery */
#define ADC_BATTERY_TIMEOUT_US	1000		/* wait up to 1 ms		*/
#define ADC_CONSTANT_DEFAULT	0		/* default ADC constant		*/

/*
 * Lucy buttons
 */

#define A_BTN_L2K		(PAD_GPIO_C +  7)
#define B_BTN_L2K		(PAD_GPIO_A + 30)
#define DPAD_DOWN_L2K		(PAD_GPIO_C + 14)
#define DPAD_LEFT_L2K		(PAD_GPIO_C +  9)
#define DPAD_RIGHT_L2K		(PAD_GPIO_C +  1)
#define DPAD_UP_L2K		(PAD_GPIO_C + 13)
#define HINT_BTN_L2K    	(PAD_GPIO_A + 14)
#define HOME_BTN_L2K		(PAD_GPIO_C + 10)
#define PAUSE_BTN_L2K		(PAD_GPIO_A + 12)
#define SHOULDER_LEFT_L2K       (PAD_GPIO_C +  8)
#define SHOULDER_RIGHT_L2K      (PAD_GPIO_A + 10)
#define VOLUME_DOWN_L2K		(PAD_GPIO_A + 17)
#define VOLUME_UP_L2K		(PAD_GPIO_A + 13)

/*
 * Lucy CIP buttons
 */

#define A_BTN_LUCY_CIP		(PAD_GPIO_C +  7)
#define B_BTN_LUCY_CIP		(PAD_GPIO_A + 30)
#define DPAD_DOWN_LUCY_CIP	(PAD_GPIO_D + 11)
#define DPAD_LEFT_LUCY_CIP	(PAD_GPIO_D + 15)
#define DPAD_RIGHT_LUCY_CIP	(PAD_GPIO_D + 10)
#define DPAD_UP_LUCY_CIP	(PAD_GPIO_D + 12)
#define HINT_BTN_LUCY_CIP    	(PAD_GPIO_A + 14)
#define HOME_BTN_LUCY_CIP	(PAD_GPIO_D + 14)
#define PAUSE_BTN_LUCY_CIP	(PAD_GPIO_A + 12)
#define SHOULDER_LEFT_LUCY_CIP  (PAD_GPIO_D + 18)
#define SHOULDER_RIGHT_LUCY_CIP (PAD_GPIO_A + 10)
#define VOLUME_DOWN_LUCY_CIP	(PAD_GPIO_A + 17)
#define VOLUME_UP_LUCY_CIP	(PAD_GPIO_A + 13)

/*
 * Valencia buttons
 */

#define DPAD_DOWN_M2K		(PAD_GPIO_C + 13)
#define DPAD_LEFT_M2K		(PAD_GPIO_C +  9)
#define DPAD_RIGHT_M2K   	(PAD_GPIO_C + 10)
#define DPAD_UP_M2K		(PAD_GPIO_C +  0)
#define HOME_BTN_M2K    	(PAD_GPIO_C + 11)
#define VOLUME_DOWN_M2K		(PAD_GPIO_A + 13)
#define VOLUME_UP_M2K		(PAD_GPIO_A + 14)

/*
 * Valencia CIP buttons
 */

#define DPAD_DOWN_VAL_CIP	(PAD_GPIO_D + 13)
#define DPAD_LEFT_VAL_CIP	(PAD_GPIO_D + 9)
#define DPAD_RIGHT_VAL_CIP   	(PAD_GPIO_D + 10)
#define DPAD_UP_VAL_CIP		(PAD_GPIO_D + 15)
#define HOME_BTN_VAL_CIP    	(PAD_GPIO_D + 11)
#define VOLUME_DOWN_VAL_CIP	(PAD_GPIO_A + 13)
#define VOLUME_UP_VAL_CIP	(PAD_GPIO_A + 14)

/*
 * Rio Alpha buttons
 */

#define DPAD_DOWN_RIO		(PAD_GPIO_A + 12)
#define DPAD_LEFT_RIO		(PAD_GPIO_A + 10)
#define DPAD_RIGHT_RIO   	(PAD_GPIO_A + 11)
#define DPAD_UP_RIO		(PAD_GPIO_A + 13)
#define HOME_BTN_RIO    	(PAD_GPIO_A + 14)
#define VOLUME_DOWN_RIO		(PAD_GPIO_D + 13)
#define VOLUME_UP_RIO		(PAD_GPIO_A + 17)
#define USB_CHG_DETECT_RIO_ALPHA   (PAD_GPIO_D + 19)

/*
 * Rio Beta buttons
 */

#define DPAD_DOWN_RIO_BETA	(PAD_GPIO_A + 12)
#define DPAD_LEFT_RIO_BETA	(PAD_GPIO_A + 10)
#define DPAD_RIGHT_RIO_BETA   	(PAD_GPIO_A + 11)
#define DPAD_UP_RIO_BETA	(PAD_GPIO_A + 13)
#define HOME_BTN_RIO_BETA    	(PAD_GPIO_A + 14)
#define VOLUME_DOWN_RIO_BETA	(PAD_GPIO_D + 15)
#define VOLUME_UP_RIO_BETA	(PAD_GPIO_A + 19)
#define USB_CHG_DETECT_RIO_BETA   (PAD_GPIO_D + 18)

/*
 * Rio EP buttons
 */

#define DPAD_DOWN_RIO_EP	(PAD_GPIO_A + 12)
#define DPAD_LEFT_RIO_EP	(PAD_GPIO_A + 10)
#define DPAD_RIGHT_RIO_EP   	(PAD_GPIO_A + 11)
#define DPAD_UP_RIO_EP		(PAD_GPIO_A + 13)
#define HOME_BTN_RIO_EP    	(PAD_GPIO_A + 14)
#define VOLUME_DOWN_RIO_EP	(PAD_GPIO_D + 15)
#define VOLUME_UP_RIO_EP	(PAD_GPIO_A + 19)
#define USB_CHG_DETECT_RIO_EP   (PAD_GPIO_D + 18)

/*
 * get_board_info()
 * Isolate board specific differences
 */

struct lf2000_board_info board_info = {
	.is_initialized = 0,
};

void config_board_lucy(void)
{
	board_info.cfg_disp_pri_resol_width	  = CFG_DISP_PRI_RESOL_WIDTH_L2K;
	board_info.cfg_disp_pri_resol_height	  = CFG_DISP_PRI_RESOL_HEIGHT_L2K;
	board_info.cfg_disp_pri_clkgen0_div	  = CFG_DISP_PRI_CLKGEN0_DIV_L2K;
	board_info.cfg_disp_pri_out_clk_invert	  = CFG_DISP_PRI_OUT_CLK_INVERT_L2K;
	board_info.cfg_disp_pri_hsync_front_porch = CFG_DISP_PRI_HSYNC_FRONT_PORCH_L2K;
	board_info.cfg_disp_pri_hsync_back_porch  = CFG_DISP_PRI_HSYNC_BACK_PORCH_L2K;
	board_info.cfg_disp_pri_vsync_front_porch = CFG_DISP_PRI_VSYNC_FRONT_PORCH_L2K;
	board_info.cfg_disp_pri_vsync_back_porch  = CFG_DISP_PRI_VSYNC_BACK_PORCH_L2K;

	board_info.spi_lcd			  = CFG_SPI_LCD_L2K;
}


void config_board_valencia(void)
{
	board_info.cfg_disp_pri_resol_width	  = CFG_DISP_PRI_RESOL_WIDTH_M2K;
	board_info.cfg_disp_pri_resol_height	  = CFG_DISP_PRI_RESOL_HEIGHT_M2K;
	board_info.cfg_disp_pri_clkgen0_div	  = CFG_DISP_PRI_CLKGEN0_DIV_M2K;
	board_info.cfg_disp_pri_out_clk_invert	  = CFG_DISP_PRI_OUT_CLK_INVERT_M2K;
	board_info.cfg_disp_pri_hsync_front_porch = CFG_DISP_PRI_HSYNC_FRONT_PORCH_M2K;
	board_info.cfg_disp_pri_hsync_back_porch  = CFG_DISP_PRI_HSYNC_BACK_PORCH_M2K;
	board_info.cfg_disp_pri_vsync_front_porch = CFG_DISP_PRI_VSYNC_FRONT_PORCH_M2K;
	board_info.cfg_disp_pri_vsync_back_porch  = CFG_DISP_PRI_VSYNC_BACK_PORCH_M2K;

	board_info.spi_lcd			  = CFG_SPI_LCD_M2K;
}

void config_board_valencia_800_480(void)
{
	board_info.cfg_disp_pri_resol_width	  = CFG_DISP_PRI_RESOL_WIDTH_VALENCIA_800_480;
	board_info.cfg_disp_pri_resol_height	  = CFG_DISP_PRI_RESOL_HEIGHT_VALENCIA_800_480;
	board_info.cfg_disp_pri_clkgen0_div	  = CFG_DISP_PRI_CLKGEN0_DIV_VALENCIA_800_480;
	board_info.cfg_disp_pri_out_clk_invert	  = CFG_DISP_PRI_OUT_CLK_INVERT_VALENCIA_800_480;
	board_info.cfg_disp_pri_hsync_front_porch = CFG_DISP_PRI_HSYNC_FRONT_PORCH_VALENCIA_800_480;
	board_info.cfg_disp_pri_hsync_back_porch  = CFG_DISP_PRI_HSYNC_BACK_PORCH_VALENCIA_800_480;
	board_info.cfg_disp_pri_vsync_front_porch = CFG_DISP_PRI_VSYNC_FRONT_PORCH_VALENCIA_800_480;
	board_info.cfg_disp_pri_vsync_back_porch  = CFG_DISP_PRI_VSYNC_BACK_PORCH_VALENCIA_800_480;

	board_info.spi_lcd			  = CFG_SPI_LCD_VALENCIA_800_480;
}

void config_board_valencia_1024_600(void)
{
	board_info.cfg_disp_pri_resol_width	  = 1024;
	board_info.cfg_disp_pri_resol_height	  = 600;
	board_info.cfg_disp_pri_clkgen0_div	  = CFG_DISP_PRI_CLKGEN0_DIV_VALENCIA_800_480;
	board_info.cfg_disp_pri_out_clk_invert	  = 1;
	board_info.cfg_disp_pri_hsync_front_porch = 160;
	board_info.cfg_disp_pri_hsync_back_porch  = 160;
	board_info.cfg_disp_pri_vsync_front_porch = 12;
	board_info.cfg_disp_pri_vsync_back_porch  = 23;

	board_info.spi_lcd			  = CFG_SPI_LCD_VALENCIA_800_480;
}

void config_board_rio(void)
{
	board_info.cfg_disp_pri_resol_width	  = 1024;
	board_info.cfg_disp_pri_resol_height	  = 600;
	board_info.cfg_disp_pri_clkgen0_div	  = CFG_DISP_PRI_CLKGEN0_DIV_RIO;
	board_info.cfg_disp_pri_out_clk_invert	  = 1;
	board_info.cfg_disp_pri_hsync_front_porch = 160;
	board_info.cfg_disp_pri_hsync_back_porch  = 160;
	board_info.cfg_disp_pri_vsync_front_porch = 12;
	board_info.cfg_disp_pri_vsync_back_porch  = 23;

	board_info.spi_lcd			  = CFG_SPI_LCD_RIO;
}

void config_board_vtk(void)
{
	board_info.cfg_disp_pri_resol_width	  = CFG_DISP_PRI_RESOL_WIDTH_VTK;
	board_info.cfg_disp_pri_resol_height	  = CFG_DISP_PRI_RESOL_HEIGHT_VTK;
	board_info.cfg_disp_pri_clkgen0_div	  = CFG_DISP_PRI_CLKGEN0_DIV_VTK;
	board_info.cfg_disp_pri_out_clk_invert	  = CFG_DISP_PRI_OUT_CLK_INVERT_VTK;
	board_info.cfg_disp_pri_hsync_front_porch = CFG_DISP_PRI_HSYNC_FRONT_PORCH_VTK;
	board_info.cfg_disp_pri_hsync_back_porch  = CFG_DISP_PRI_HSYNC_BACK_PORCH_VTK;
	board_info.cfg_disp_pri_vsync_front_porch = CFG_DISP_PRI_VSYNC_FRONT_PORCH_VTK;
	board_info.cfg_disp_pri_vsync_back_porch  = CFG_DISP_PRI_VSYNC_BACK_PORCH_VTK;

	board_info.spi_lcd			  = CFG_SPI_LCD_VTK;
}


struct lf2000_board_info * get_lf2000_board_info(void)
{

	/* initialize structure the first time through */
	if (!board_info.is_initialized) {

		switch(get_board_rev()) {
		case LF2000_BOARD_LUCY:
		case LF2000_BOARD_LUCY_CIP:
		case LF2000_BOARD_LUCY_PP:
			config_board_lucy();
			break;
		case LF2000_BOARD_VALENCIA:
		case LF2000_BOARD_VALENCIA_EP:
		case LF2000_BOARD_VALENCIA_EP_8:
		case LF2000_BOARD_VALENCIA_FEP:
		case LF2000_BOARD_VALENCIA_FEP_8:
		case LF2000_BOARD_VALENCIA_CIP:
			config_board_valencia();
			break;
		case LF2000_BOARD_VALENCIA_EP_800_480:
		case LF2000_BOARD_VALENCIA_EP_800_480_8:
		case LF2000_BOARD_VALENCIA_FEP_800_480:
		case LF2000_BOARD_VALENCIA_FEP_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_800_480:
		case LF2000_BOARD_VALENCIA_KND_800_480_8:
		case LF2000_BOARD_RIO_KND_800_480:
		case LF2000_BOARD_RIO_BETA_800_480:
		case LF2000_BOARD_RIO_BETA_800_480_700_400:
			config_board_valencia_800_480();
			break;
		case LF2000_BOARD_VALENCIA_KND_1024_600:
		case LF2000_BOARD_VALENCIA_KND_1024_600_8:
			config_board_valencia_1024_600();
			break;
		case LF2000_BOARD_RIO:
		case LF2000_BOARD_RIO_BETA_1024_600:
		case LF2000_BOARD_RIO_BETA_1024_600_700_400:
		case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		case LF2000_BOARD_RIO_EP_550_275:
		case LF2000_BOARD_RIO_EP_666_333:
		case LF2000_BOARD_RIO_EP_800_333:
		case LF2000_BOARD_RIO_EP_700_400:
		case LF2000_BOARD_RIO_EP_800_400:
		case LF2000_BOARD_RIO_FEP_800_327P67:
		case LF2000_BOARD_RIO_FEP_800_327P666:
			config_board_rio();
			break;
		case LF2000_BOARD_UNKNOWN:
		case LF2000_BOARD_VTK:
			config_board_vtk();
			break;
		}
		board_info.is_initialized = 1;
	}

	return &board_info;
}

/*------------------------------------------------------------------------------
 * intialize nexell soc and board status.
 */
void init_gpio_pad(void);
static void init_alive_pad(void);
static void init_bus_pad(void);
static void init_display(int dev);

int board_init(void)
{
	int lcd = CFG_DISP_MAIN_SCREEN;

	init_panel_info();	/* dynamically initialize panel_info structure */
	// lf2000 -- moved to mach_init() so we can read NOR early
	// init_gpio_pad();
	init_alive_pad();
	init_bus_pad();

	init_display(lcd);

#ifdef CONFIG_LCD
	lcd_init();
#endif

	DBGOUT("%s : done board initialize ...\n", CFG_SYS_BOARD_NAME);
	return 0;
}

/*
 * look for valid mfgdata area in NOR
 */
#if defined(CONFIG_MACH_NXP3200_VTK_DTK)
void* get_valid_mfgdata(void)
{
	return NULL;
}

#else

struct NX_GPIO_RegisterSet (* const pGPIOReg)[1] = (struct NX_GPIO_RegisterSet (*)[])PHY_BASEADDR_GPIO_MODULE;
struct NX_SSPSPI_RegisterSet volatile * pSPIReg = (struct NX_SSPSPI_RegisterSet *)PHY_BASEADDR_SSPSPI_MODULE;

#define SPI0_GPIO_GROUP	1	// GPIO B
#define SPI0_FRM_GPIO_NUM		12
#define SPI0_CLK_GPIO_NUM		13
#define SPI0_RXD_GPIO_NUM		14
#define SPI0_TXD_GPIO_NUM		15

#define SPI_READ_CMD	( 0x03 )

//FIXME: sesters set _PLL1_333_MHZ_ just for testing.  Should be calculated at run-time.
#define _PLL1_333_MHZ_
/*
 * Read data from SPI Flash -- returns number of words actually read
 */

U32 ReadSPIData(U8 * SPI_source_address, U8 * pDestAddr, U32 uLengthInBytes)
{
	volatile U32 temp;
	U32 uBytesRead = 0;
	register U32	tmp;
	U16 cmd_addr1, cmd_addr2;

	register U32 *pGPIOxReg = (U32 *)&pGPIOReg[SPI0_GPIO_GROUP]->GPIOxALTFN[SPI0_FRM_GPIO_NUM/16];
	*pGPIOxReg = (*pGPIOxReg & ~0xFF000000) | 0x55000000;

	//set GPIOB 12 as GPIO to control chip select independent of SPI
	temp = *((volatile U32*)0xC0014060);
	temp &= ~(0x3 << (12*2));
	*((volatile U32*)0xC0014060) = temp;

	//set GPIOB 12 as output
	temp = *((volatile U32*)0xC0014044);
	temp |= 1 << 12;
	*((volatile U32*)0xC0014044) = temp;

	//set it as high
	temp = *((volatile U32*)0xC0014040);
	temp |= 1 << 12;		//set to high
	*((volatile U32*)0xC0014040) = temp;

	pSPIReg->CLKENB = 0x1<<3;	// pclk mode on but not supply operation clock

#if defined(_PLL1_147P5_MHZ_)
	pSPIReg->CLKGEN = (2-1)<<5 | 0x1<<2;	// select clock source is pll1, 147.5MHz and supply clock is 147.45/2 = 73.725MHz
#elif defined(_PLL1_266_MHZ_)
	pSPIReg->CLKGEN = (3-1)<<5 | 0x1<<2;	// select clock source is pll1, 266.0MHz and supply clock is 266/3 = 88.67MHz
#elif defined(_PLL1_333_MHZ_)
	pSPIReg->CLKGEN = (4-1)<<5 | 0x1<<2;	// select clock source is pll1, 333.0MHz and supply clock is 333/4 = 83.25MHz
#else
#error PLL1_SPEED not set, expected _PLL1_147P5_MHZ_, _PLL1_266_MHZ_, or _PLL1_333_MHZ_, please set PLL1_SPEED
#endif
	pSPIReg->CONT0 =
			0<<12 |			// PIO mode
			0<<11 |			// SPI disable
			1<<10 |			// Reset FIFO
			0<< 9 |			// Internal Clock
		(16-1)<< 5 |		// data bit width 16 bit
		(4-1)<<0;			// 73.725MHz/4 = 18.43MHz

	pSPIReg->CONT0 &= ~(1<<10);	// fifo reset negate
	pSPIReg->CONT1 = 
			1<<5 |			// byte swap to store the uboot code as little endian
			0<<4 |			// master mode
			0<<3 |			// inverse polarity
			1<<2 |			// Format B
			1<<0;			// SPI Mode
	pSPIReg->CLKENB = 0x01<<3 | 0x1<<2;	// supply operation clock

	tmp = pSPIReg->CONT0;
	pSPIReg->CONT0 = tmp | 1<<10;		// reset FIFO
	pSPIReg->CONT0 = tmp;

	//set chip select low , GPIOB12=0
	temp = *((volatile U32*)0xC0014040);
	temp &= ~(1 << 12);  //set to low
	*((volatile U32*)0xC0014040) = temp;

	pSPIReg->CONT0 |= 1<<11;	// spi start (cs will be low)

	cmd_addr1 = (SPI_READ_CMD << 8) | (((U32)SPI_source_address >> 16) & 0xFF);
	cmd_addr2 = (U32)SPI_source_address & 0xFFFF;

	pSPIReg->DATA =	cmd_addr1;		/* cmd and highest address byte */
	pSPIReg->DATA = cmd_addr2;		/* lower two address bytes */


	while(pSPIReg->STATE & 1<<0)  /*NOP*/ ;	/* wait     */
	tmp = pSPIReg->DATA;			/* throw out dummy data */
	while(pSPIReg->STATE & 1<<0)  /*NOP*/ ;	/* wait     */
	tmp = pSPIReg->DATA;			/* throw out dummy data */

	/* read in 16 bits at a time */
	while((uBytesRead + 1) < uLengthInBytes )
	{
		pSPIReg->DATA = 0;

		while(pSPIReg->STATE & 1<<0)  /* NULL */ ;	   /* wait     */
		*((U16 *)pDestAddr) = (volatile U16)pSPIReg->DATA; /* get data */
		pDestAddr += 2;
		uBytesRead += 2;
	}

	/* read last byte if needed */
	if (uBytesRead < uLengthInBytes)
	{
		pSPIReg->DATA = 0;
		while(pSPIReg->STATE & 1<<0)
			/* NULL */;	// wait for data in RX Buffer
		*((U8 *)pDestAddr) =  (pSPIReg->DATA >> 8) & 0xFF; /* get 8 bits of data */
		uBytesRead += 1;
	}

	//set chip select to high
		temp = *((volatile U32*)0xC0014040);
		temp |= 1 << 12;         //set to high
		*((volatile U32*)0xC0014040) = temp;

	pSPIReg->CONT0 &= ~(1<<11);			// SPI Stop
	pSPIReg->CLKENB	= 0;				// PCLKMODE : disable, Clock Gen Disable

	//--------------------------------------------------------------------------

	*pGPIOxReg &= ~0xFF000000;

	//--------------------------------------------------------------------------
	return uBytesRead;
}

/*
 * check for valid MFGDATA area
 */
void * validate_mfgdata(void * base_address)
{
	struct MFGDATA * mfgdata;
	u32 crc;
	int i;

	for (i = 0; i < 2; i++) {
		// point at mfgdata image in NOR
		mfgdata = ((struct MFGDATA *) (base_address
				+ (i * sizeof(struct MFGDATA))));

		// calculate checksum, don't include stored checksum
		crc = crc32(0xFFFFFFFF, (unsigned char *)mfgdata,
				offsetof(struct MFGDATA, Checksum));
		crc ^= 0xFFFFFFFF;
		if (crc == mfgdata->Checksum) {
			return (void*) mfgdata;
		}
	}
	return NULL;	// not found
}

int spi_mfgdata_cache_valid = 0;	/* flag non-zero if mfgdata_cache images loaded */
/* copy both MFGDATA areas from SPI */
struct MFGDATA spi_mfgdata_cache[2];

void* get_valid_mfgdata(void)
{
	void * base_address;

	// Booting from SPI means MFGDATA in SPI NOR
	if (is_spi_boot()) {
		if (spi_mfgdata_cache_valid == 0) {
			/* load SPI data into cache */
			if(ReadSPIData((U8 *)LF2000_NOR_BOOT_SIZE,
				       (U8 *)spi_mfgdata_cache,
				       (U32)sizeof(spi_mfgdata_cache)) == sizeof(spi_mfgdata_cache))
				spi_mfgdata_cache_valid = 1;	/* MFGDATA read from SPI */
		}
		base_address = validate_mfgdata((unsigned int) spi_mfgdata_cache);

	// Booting from PARALLEL NOR means MFGDATA in NOR low memory
	} else	if (is_nor_boot()) {
		base_address = validate_mfgdata(LF2000_NOR_FLASH_BASE_LOW + LF2000_NOR_BOOT_SIZE);

	// Didn't boot from NOR, booted from SD or another method
	} else {
		// Look for mfgdata area in Parallel NOR high memory location
		base_address = validate_mfgdata(LF2000_NOR_FLASH_BASE_HIGH0 + LF2000_NOR_BOOT_SIZE);

		// try to read from SPI NOR
		if (base_address == NULL) {
			/* load SPI data into cache */
			if(ReadSPIData((U8 *)LF2000_NOR_BOOT_SIZE,
							(U8 *)spi_mfgdata_cache,
							(U32)sizeof(spi_mfgdata_cache)) == sizeof(spi_mfgdata_cache))
			spi_mfgdata_cache_valid = 1; /* MFGDATA read from SPI */
			base_address = validate_mfgdata((unsigned int) spi_mfgdata_cache);
		}
	}

// Uncomment to dump MFGDATA contents
#if 0
	/* dump MFGDATA as bytes */
	printf("%s.%d:%s  MFGDATA dump\n", __FILE__, __LINE__, __func__);
	for(i=0; i < sizeof(struct MFGDATA); i += 16)
	{
		printf("  0x%8.8X    %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X - %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X\n",
			i,
			*(U8 *)(base_address + i +  0), *(U8 *)(base_address + i +  1),
			*(U8 *)(base_address + i +  2), *(U8 *)(base_address + i +  3),
			*(U8 *)(base_address + i +  4), *(U8 *)(base_address + i +  5),
			*(U8 *)(base_address + i +  6), *(U8 *)(base_address + i +  7),
			*(U8 *)(base_address + i +  8), *(U8 *)(base_address + i +  9),
			*(U8 *)(base_address + i + 10), *(U8 *)(base_address + i + 11),
			*(U8 *)(base_address + i + 12), *(U8 *)(base_address + i + 13),
			*(U8 *)(base_address + i + 14), *(U8 *)(base_address + i + 15));
	}
#endif

	return base_address;
}
#endif

/*
 * get_board_rev
 *      Read board rev from mfgdata area.  Note that in order
 *	to read the NOR, GPIO port D[19:9] must be setup as ALT1.
 */

#if defined(CONFIG_MACH_NXP3200_VTK_DTK)

u32 get_board_rev(void)
{
	return CONFIG_BOARD_REVISION_DEFAULT;	/* VTK NOR unsupported */
}
#else

u32 board_id = 0;		/* cache board rev */

u32 get_board_rev(void)
{
	struct MFGDATA *pMfgData;

	if (board_id != 0)
		return (board_id);	/* return cached value */

	/* first time, read board id from NOR */
	pMfgData = (struct MFGDATA *)get_valid_mfgdata();
	if (pMfgData == (struct MFGDATA *)NULL) {
		board_id = CONFIG_BOARD_REVISION_DEFAULT;
		printf("  %s:%s.%d  pMfgData not valid\n",
			__FILE__, __func__, __LINE__);
		if (is_show_lcd_info())
			lcd_printf("MfgData not valid\n");
	} else {
		board_id = pMfgData->SoftBoardId;
	}

	/* mfgdata NOR board_id should always be 0x100 or
	 * above and less than 0xFFFFFFFF
	 */

	if (board_id < 0x100 || board_id == 0xFFFFFFFF) {
		// NOR unavailable, use default
		board_id = CONFIG_BOARD_REVISION_DEFAULT;
		printf("  %s:%s.%d  NOR invalid, using board_id=0x%X\n",
			__FILE__, __func__, __LINE__, board_id);
	}
	return(board_id);
}
#endif

/*
 * test for board boot source
 */

/*
 * is_NorBoot()
 *   return true if system configuration shows we booted from NOR
 *   in system configuration register SYSRSTCOFIG (0xC001E07C)
 *   bits [10:8].
 */

#define BOOTMODE_MASK   (7 << 8)
#define NORBOOT_VALUE   (2 << 8)
#define INTBOOT_VALUE	(5 << 8)

#define EXTBOOT_MASK	(7 << 14)
#define EXTBOOT_USB	(0 << 14)
#define EXTBOOT_NANDEDC	(1 << 14)
#define EXTBOOT_NANDECC	(2 << 14)
#define EXTBOOT_SDHC	(3 << 14)
#define EXTBOOT_UART	(4 << 14)
#define EXTBOOT_SPI	(7 << 14)

int is_nor_boot(void)
{
	if ((NX_CLKPWR_GetSystemResetConfiguration() & BOOTMODE_MASK) != NORBOOT_VALUE) {
		DBGOUT("%s:%s.%d return(0)\n", __FILE__, __func__, __LINE__);
		return(0);
	}
	DBGOUT("%s:%s.%d return(1)\n", __FILE__, __func__, __LINE__);
	return(1);
}

int is_spi_boot(void)
{
	if ((NX_CLKPWR_GetSystemResetConfiguration() & BOOTMODE_MASK) != INTBOOT_VALUE) {
		DBGOUT("%s:%s.%d return(0)\n", __FILE__, __func__, __LINE__);
		return (0);
	}

	if ((NX_CLKPWR_GetSystemResetConfiguration() & EXTBOOT_MASK) != EXTBOOT_SPI) {
		DBGOUT("%s:%s.%d return(0)\n", __FILE__, __func__, __LINE__);
		return(0);
	}
	DBGOUT("%s:%s.%d return(1)\n", __FILE__, __func__, __LINE__);
	return(1);
}

/************************************************************************/
/* run_inactive_timer() -- shut down if inactivity timer expires		*/
/*	reset != 0, then reset timer					*/
/************************************************************************/

#define	INACTIVITY_SECONDS	300

unsigned long long inactivity_timer = 0;

void run_inactivity_timer(int reset)
{
	if (reset) {	/* reset timer */
		inactivity_timer =
			get_ticks() + get_tbclk() * INACTIVITY_SECONDS;
		return;
	}

	if (get_ticks() < inactivity_timer)
		return;	/* still have time */

	/* timer expired, shutdown system */
#if defined (CONFIG_SOC_LFP100)
	lfp100_standby();
#endif
}


int	mfgdata_read = 0;		/* 1 = read ADC MFGDATA from NOR*/
int	adc_calibrated = 0;		/* 1 = have NOR calibration data*/

int	adc_constant = 0;		/* ADC calibration constant	*/
int	adc_slope_256 = 0;		/* ADC slope * 256		*/

/************************************************************************/
/* measure_battery							*/
/*	return battery level in millivolts, read NOR if needed		*/
/************************************************************************/
int measure_battery(void)
{
	unsigned int adc_raw;
	
	/* SP 01-29-13: For all Rio units, do not read adc slope
	 * and constant values from the mfgdata. This can be removed later
	 * if mfgdata is programmed with correct calibration values. 
	 */
	switch(get_board_rev()) {
	case LF2000_BOARD_RIO:
	case LF2000_BOARD_RIO_KND_800_480:
	case LF2000_BOARD_RIO_BETA_1024_600:
	case LF2000_BOARD_RIO_BETA_800_480:
	case LF2000_BOARD_RIO_BETA_1024_600_700_400:
	case LF2000_BOARD_RIO_BETA_800_480_700_400:
	case LF2000_BOARD_RIO_BETA_1024_600_550_275:
	case LF2000_BOARD_RIO_EP_550_275:
	case LF2000_BOARD_RIO_EP_666_333:
	case LF2000_BOARD_RIO_EP_800_333:
	case LF2000_BOARD_RIO_EP_700_400:
	case LF2000_BOARD_RIO_EP_800_400:
	case LF2000_BOARD_RIO_FEP_800_327P67:
	case LF2000_BOARD_RIO_FEP_800_327P666:
		mfgdata_read = 1;
		break;
	default:
		break;
	}

	if (!mfgdata_read) {
		struct MFGDATA *pMfgData = get_valid_mfgdata();
		if (pMfgData) {
			adc_constant  = pMfgData->ADCCalData[0];
			adc_slope_256 = pMfgData->ADCCalData[1];
			adc_calibrated = 1;
		} else {
			adc_constant  = ADC_CONSTANT_DEFAULT;
			adc_slope_256 = ADC_SLOPE_256_DEFAULT;
		}
		mfgdata_read = 1;
	}
	
	/* If NOR is not programmed, default to a board specific value */
	if(adc_slope_256 == 0)
		adc_slope_256 = ADC_SLOPE_256_DEFAULT;

#if defined (CONFIG_SOC_LFP100)
	if (lfp100_is_battery()) {
		/* read battery level */
		soc_adc_attach();
		/* get ADC ticks and convert to millivolts */
		adc_raw = soc_adc_read(ADC_BATTERY, ADC_BATTERY_TIMEOUT_US);
		soc_adc_detach();
		return (adc_raw * adc_slope_256)/256 + adc_constant;
	}
#endif
	return -1;
}

/************************************************************************/
/* is_low_battery							*/
/*	return (1) if batteries are low					*/
/************************************************************************/
int is_low_battery(void)
{
	int battery_mv;
	int ret = 0;

	battery_mv = measure_battery();
	if (0 <= battery_mv && battery_mv <= BATTERY_LOW_BOOT)
		ret = 1;
	return ret;
}

int get_screen_res(void)
{
	switch(get_board_rev()) {
	case LF2000_BOARD_LUCY:
	case LF2000_BOARD_LUCY_CIP:
	case LF2000_BOARD_LUCY_PP:
		return SCREEN_320_200;

	case LF2000_BOARD_VALENCIA:
	case LF2000_BOARD_VALENCIA_EP:
	case LF2000_BOARD_VALENCIA_EP_8:
	case LF2000_BOARD_VALENCIA_FEP:
	case LF2000_BOARD_VALENCIA_FEP_8:
	case LF2000_BOARD_VALENCIA_CIP:
		return SCREEN_480_272;

	case LF2000_BOARD_VTK:
	case LF2000_BOARD_VALENCIA_EP_800_480:
	case LF2000_BOARD_VALENCIA_EP_800_480_8:
	case LF2000_BOARD_VALENCIA_KND_800_480:
	case LF2000_BOARD_VALENCIA_KND_800_480_8:
	case LF2000_BOARD_RIO_KND_800_480:
	case LF2000_BOARD_RIO_BETA_800_480:
	case LF2000_BOARD_RIO_BETA_800_480_700_400:
		return SCREEN_800_480;
	
	case LF2000_BOARD_VALENCIA_KND_1024_600:
	case LF2000_BOARD_VALENCIA_KND_1024_600_8:
	case LF2000_BOARD_RIO:
	case LF2000_BOARD_RIO_BETA_1024_600:
	case LF2000_BOARD_RIO_BETA_1024_600_700_400:
	case LF2000_BOARD_RIO_BETA_1024_600_550_275:
	case LF2000_BOARD_RIO_EP_550_275:
	case LF2000_BOARD_RIO_EP_666_333:
	case LF2000_BOARD_RIO_EP_800_333:
	case LF2000_BOARD_RIO_EP_700_400:
	case LF2000_BOARD_RIO_EP_800_400:
	case LF2000_BOARD_RIO_FEP_800_327P67:
	case LF2000_BOARD_RIO_FEP_800_327P666:
		return SCREEN_1024_600;

	case LF2000_BOARD_UNKNOWN:
	default:
		return SCREEN_NO_RES;
	}
}

/*
 * Tune-up boot when buttons are pressed at power-up.
 */

int is_tune_up_boot(void)
{
	int	gpio_button1, gpio_button2;	/* tuneup button sequence */
	int	button1, button2;

	switch(get_board_rev()) {
	case LF2000_BOARD_VALENCIA:
	case LF2000_BOARD_VALENCIA_EP:
	case LF2000_BOARD_VALENCIA_EP_8:
	case LF2000_BOARD_VALENCIA_FEP:
	case LF2000_BOARD_VALENCIA_FEP_8:
	case LF2000_BOARD_VALENCIA_EP_800_480:
	case LF2000_BOARD_VALENCIA_EP_800_480_8:
	case LF2000_BOARD_VALENCIA_FEP_800_480:
	case LF2000_BOARD_VALENCIA_FEP_800_480_8:
	case LF2000_BOARD_VALENCIA_KND_800_480:
	case LF2000_BOARD_VALENCIA_KND_800_480_8:
	case LF2000_BOARD_VALENCIA_KND_1024_600:
	case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		gpio_button1 = DPAD_RIGHT_M2K;
		gpio_button2 = HOME_BTN_M2K;
		break;
	case LF2000_BOARD_VALENCIA_CIP:
		gpio_button1 = DPAD_RIGHT_VAL_CIP;
		gpio_button2 = HOME_BTN_VAL_CIP;
		break;
	case LF2000_BOARD_RIO:
	case LF2000_BOARD_RIO_KND_800_480:
		gpio_button1 = DPAD_RIGHT_RIO;
		gpio_button2 = HOME_BTN_RIO;
		break;
	case LF2000_BOARD_RIO_BETA_1024_600:
	case LF2000_BOARD_RIO_BETA_800_480:
	case LF2000_BOARD_RIO_BETA_1024_600_700_400:
	case LF2000_BOARD_RIO_BETA_800_480_700_400:
	case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		gpio_button1 = DPAD_RIGHT_RIO_BETA;
		gpio_button2 = HOME_BTN_RIO_BETA;
		break;
	case LF2000_BOARD_RIO_EP_550_275:
	case LF2000_BOARD_RIO_EP_666_333:
	case LF2000_BOARD_RIO_EP_800_333:
	case LF2000_BOARD_RIO_EP_700_400:
	case LF2000_BOARD_RIO_EP_800_400:
	case LF2000_BOARD_RIO_FEP_800_327P67:
	case LF2000_BOARD_RIO_FEP_800_327P666:
		gpio_button1 = DPAD_RIGHT_RIO_EP;
		gpio_button2 = HOME_BTN_RIO_EP;
		break;
	case LF2000_BOARD_LUCY:
	case LF2000_BOARD_LUCY_CIP:
	case LF2000_BOARD_LUCY_PP:
		gpio_button1 = SHOULDER_RIGHT_L2K;
		gpio_button2 = SHOULDER_LEFT_L2K;
		break;
	case LF2000_BOARD_VTK:
	case LF2000_BOARD_UNKNOWN:
	default:
		return(0);	/* unknown, never active */
	}

	button1 = NX_GPIO_GetInputValue(gpio_button1 >> 5,
					  gpio_button1 & 0x1F);
	button2 = NX_GPIO_GetInputValue(gpio_button2 >> 5,
					  gpio_button2 & 0x1F);
	return !(button1 | button2);
}


/* in the following macro, 'b' ought to have the form (PAD_GPIO_x +n),
 * where x is A, B, C, or D, and n is in the interval [0,31]
 */
#define button_pressed(b) ( !(NX_GPIO_GetInputValue((b) >> 5, (b) & 0x1F)))

/*
 * return nonzero if any lucy button is pressed
 */

	/*  Hint button pressed AND  */
	/*  NOT ((A+B pressed) OR (VolUp + VolDown pressed)) */
static int is_lucy_serial_output_selected(void)
{
	return (   button_pressed(HINT_BTN_L2K)
		&& !(   (   button_pressed(A_BTN_L2K)
		        && button_pressed(B_BTN_L2K))
		    || (   button_pressed(VOLUME_UP_L2K)
		        && button_pressed(VOLUME_DOWN_L2K))));
}
static int is_lucy_cip_serial_output_selected(void)
{
	return (   button_pressed(HINT_BTN_LUCY_CIP)
		&& !(   (   button_pressed(A_BTN_LUCY_CIP)
		        && button_pressed(B_BTN_LUCY_CIP))
		    || (   button_pressed(VOLUME_UP_LUCY_CIP)
		        && button_pressed(VOLUME_DOWN_LUCY_CIP))));
}
/*
 * return nonzero if any Valencia button is pressed
 */

	/* Home button pressed and not both VolUp and VolDown */
static int is_valencia_serial_output_selected(void)
{
	return (    button_pressed(HOME_BTN_M2K)
		&& !(   button_pressed(VOLUME_UP_M2K)
		     && button_pressed(VOLUME_DOWN_M2K)));
}

/* Home button pressed and not both VolUp and VolDown */
static int is_valencia_cip_serial_output_selected(void)
{
return (    button_pressed(HOME_BTN_VAL_CIP)
	&& !(   button_pressed(VOLUME_UP_VAL_CIP)
	     && button_pressed(VOLUME_DOWN_VAL_CIP)));
}

/*
 * return nonzero if any Rio button is pressed
 */

/* Home button pressed and not both VolUp and VolDown */
static int is_rio_serial_output_selected(void)
{
return (    button_pressed(HOME_BTN_RIO)
	&& !(   button_pressed(VOLUME_UP_RIO)
	     && button_pressed(VOLUME_DOWN_RIO)));
}
/* Home button pressed and not both VolUp and VolDown */
static int is_rio_beta_serial_output_selected(void)
{
return (    button_pressed(HOME_BTN_RIO_BETA)
	&& !(   button_pressed(VOLUME_UP_RIO_BETA)
	     && button_pressed(VOLUME_DOWN_RIO_BETA)));
}
/* Home button pressed and not both VolUp and VolDown */
static int is_rio_ep_serial_output_selected(void)
{
return (    button_pressed(HOME_BTN_RIO_EP)
	&& !(   button_pressed(VOLUME_UP_RIO_EP)
	     && button_pressed(VOLUME_DOWN_RIO_EP)));
}
/*
 * determine if user actions have selected serial output
 * 	For Lucy and Valencia - check the button GPIOs
 *	For others, always return nonzero (true)
 */
int user_selected_serial_output(void)
{
	int	result;

	switch(get_board_rev()) {
	case LF2000_BOARD_RIO:
	case LF2000_BOARD_RIO_KND_800_480:
		result = is_rio_serial_output_selected();
		break;

	case LF2000_BOARD_RIO_BETA_1024_600:
	case LF2000_BOARD_RIO_BETA_800_480:
	case LF2000_BOARD_RIO_BETA_1024_600_700_400:
	case LF2000_BOARD_RIO_BETA_800_480_700_400:
	case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		result = is_rio_beta_serial_output_selected();
		break;

	case LF2000_BOARD_RIO_EP_550_275:
	case LF2000_BOARD_RIO_EP_666_333:
	case LF2000_BOARD_RIO_EP_800_333:
	case LF2000_BOARD_RIO_EP_700_400:
	case LF2000_BOARD_RIO_EP_800_400:
	case LF2000_BOARD_RIO_FEP_800_327P67:
	case LF2000_BOARD_RIO_FEP_800_327P666:
		result = is_rio_ep_serial_output_selected();
		break;
	case LF2000_BOARD_VALENCIA:
	case LF2000_BOARD_VALENCIA_EP:
	case LF2000_BOARD_VALENCIA_EP_8:
	case LF2000_BOARD_VALENCIA_FEP:
	case LF2000_BOARD_VALENCIA_FEP_8:
	case LF2000_BOARD_VALENCIA_EP_800_480:
	case LF2000_BOARD_VALENCIA_EP_800_480_8:
	case LF2000_BOARD_VALENCIA_FEP_800_480:
	case LF2000_BOARD_VALENCIA_FEP_800_480_8:
	case LF2000_BOARD_VALENCIA_KND_800_480:
	case LF2000_BOARD_VALENCIA_KND_800_480_8:
	case LF2000_BOARD_VALENCIA_KND_1024_600:
	case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		result = is_valencia_serial_output_selected();
		break;

	case LF2000_BOARD_VALENCIA_CIP:
		result = is_valencia_cip_serial_output_selected();
		break;

	case LF2000_BOARD_LUCY:
	case LF2000_BOARD_LUCY_PP:
		result = is_lucy_serial_output_selected();
		break;

	case LF2000_BOARD_LUCY_CIP:
		result = is_lucy_cip_serial_output_selected();
		break;

	case LF2000_BOARD_VTK:
	case LF2000_BOARD_UNKNOWN:
	default:
		result = 1;
		break;
	}
	return result;
}


/*
 * show U-Boot Information when the hint/home button is pressed by itself
 */
int is_show_lcd_info(void)
{
	int ret = 0;	// assume don't show LCD info

	switch(get_board_rev()) {
	case LF2000_BOARD_VALENCIA:
	case LF2000_BOARD_VALENCIA_EP:
	case LF2000_BOARD_VALENCIA_EP_8:
	case LF2000_BOARD_VALENCIA_FEP:
	case LF2000_BOARD_VALENCIA_FEP_8:
	case LF2000_BOARD_VALENCIA_EP_800_480:
	case LF2000_BOARD_VALENCIA_EP_800_480_8:
	case LF2000_BOARD_VALENCIA_FEP_800_480:
	case LF2000_BOARD_VALENCIA_FEP_800_480_8:
	case LF2000_BOARD_VALENCIA_KND_800_480:
	case LF2000_BOARD_VALENCIA_KND_800_480_8:
	case LF2000_BOARD_VALENCIA_KND_1024_600:
	case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		ret = button_pressed(HOME_BTN_M2K) && 
			(!(button_pressed(DPAD_DOWN_M2K)   ||
			   button_pressed(DPAD_LEFT_M2K)   ||
			   button_pressed(DPAD_RIGHT_M2K)  ||
			   button_pressed(DPAD_UP_M2K)     ||
			   button_pressed(VOLUME_UP_M2K)   ||
			   button_pressed(VOLUME_DOWN_M2K))
			);
		break;
	case LF2000_BOARD_VALENCIA_CIP:
		ret = button_pressed(HOME_BTN_VAL_CIP) &&
			(!(button_pressed(DPAD_DOWN_VAL_CIP)   ||
			   button_pressed(DPAD_LEFT_VAL_CIP)   ||
			   button_pressed(DPAD_RIGHT_VAL_CIP)  ||
			   button_pressed(DPAD_UP_VAL_CIP)     ||
			   button_pressed(VOLUME_UP_VAL_CIP)   ||
			   button_pressed(VOLUME_DOWN_VAL_CIP))
			);
		break;
	case LF2000_BOARD_RIO:
	case LF2000_BOARD_RIO_KND_800_480:
		ret = button_pressed(HOME_BTN_RIO) &&
			(!(button_pressed(DPAD_DOWN_RIO)    ||
			   button_pressed(DPAD_LEFT_RIO)    ||
			   button_pressed(DPAD_RIGHT_RIO)   ||
			   button_pressed(DPAD_UP_RIO)      ||
			   button_pressed(VOLUME_UP_RIO)    ||
			   button_pressed(VOLUME_DOWN_RIO))
			);
		break;
	case LF2000_BOARD_RIO_BETA_1024_600:
	case LF2000_BOARD_RIO_BETA_800_480:
	case LF2000_BOARD_RIO_BETA_1024_600_700_400:
	case LF2000_BOARD_RIO_BETA_800_480_700_400:
	case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		ret = button_pressed(HOME_BTN_RIO_BETA) &&
			(!(button_pressed(DPAD_DOWN_RIO_BETA)    ||
			   button_pressed(DPAD_LEFT_RIO_BETA)    ||
			   button_pressed(DPAD_RIGHT_RIO_BETA)   ||
			   button_pressed(DPAD_UP_RIO_BETA)      ||
			   button_pressed(VOLUME_UP_RIO_BETA)    ||
			   button_pressed(VOLUME_DOWN_RIO_BETA))
			);
		break;
	case LF2000_BOARD_RIO_EP_550_275:
	case LF2000_BOARD_RIO_EP_666_333:
	case LF2000_BOARD_RIO_EP_800_333:
	case LF2000_BOARD_RIO_EP_700_400:
	case LF2000_BOARD_RIO_EP_800_400:
	case LF2000_BOARD_RIO_FEP_800_327P67:
	case LF2000_BOARD_RIO_FEP_800_327P666:
		ret = button_pressed(HOME_BTN_RIO_EP) &&
			(!(button_pressed(DPAD_DOWN_RIO_EP)    ||
			   button_pressed(DPAD_LEFT_RIO_EP)    ||
			   button_pressed(DPAD_RIGHT_RIO_EP)   ||
			   button_pressed(DPAD_UP_RIO_EP)      ||
			   button_pressed(VOLUME_UP_RIO_EP)    ||
			   button_pressed(VOLUME_DOWN_RIO_EP))
			);
		break;
	case LF2000_BOARD_LUCY:
	case LF2000_BOARD_LUCY_PP:
		ret = button_pressed(HINT_BTN_L2K) && 
			(!(button_pressed(A_BTN_L2K)          ||
			   button_pressed(B_BTN_L2K)          ||
			   button_pressed(DPAD_DOWN_L2K)      ||
			   button_pressed(DPAD_LEFT_L2K)      ||
			   button_pressed(DPAD_RIGHT_L2K)     ||
			   button_pressed(DPAD_UP_L2K)        ||
			   button_pressed(HOME_BTN_L2K)       ||
			   button_pressed(PAUSE_BTN_L2K)      ||
			   button_pressed(SHOULDER_LEFT_L2K)  ||
			   button_pressed(SHOULDER_RIGHT_L2K) ||
			   button_pressed(VOLUME_DOWN_L2K)    ||
			   button_pressed(VOLUME_UP_L2K))
			);
		break;
	case LF2000_BOARD_LUCY_CIP:
		ret = button_pressed(HOME_BTN_LUCY_CIP) &&
			(!(button_pressed(DPAD_DOWN_LUCY_CIP)   ||
			   button_pressed(DPAD_LEFT_LUCY_CIP)   ||
			   button_pressed(DPAD_RIGHT_LUCY_CIP)  ||
			   button_pressed(DPAD_UP_LUCY_CIP)     ||
			   button_pressed(VOLUME_UP_LUCY_CIP)   ||
			   button_pressed(VOLUME_DOWN_LUCY_CIP))
			);
		break;
	case LF2000_BOARD_VTK:
	case LF2000_BOARD_UNKNOWN:
	default:
		break;
	}
	return ret;
}


/*
 * determine if system can be powered from USB device
 */
int have_usb_power_option(void)
{
	switch(get_board_rev()) {
	case LF2000_BOARD_VALENCIA:
	case LF2000_BOARD_VALENCIA_EP:
	case LF2000_BOARD_VALENCIA_EP_8:
	case LF2000_BOARD_VALENCIA_FEP:
	case LF2000_BOARD_VALENCIA_FEP_8:
	case LF2000_BOARD_VALENCIA_CIP:
	case LF2000_BOARD_VALENCIA_EP_800_480:
	case LF2000_BOARD_VALENCIA_EP_800_480_8:
	case LF2000_BOARD_VALENCIA_FEP_800_480:
	case LF2000_BOARD_VALENCIA_FEP_800_480_8:
	case LF2000_BOARD_VALENCIA_KND_800_480:
	case LF2000_BOARD_VALENCIA_KND_800_480_8:
	case LF2000_BOARD_VALENCIA_KND_1024_600:
	case LF2000_BOARD_VALENCIA_KND_1024_600_8:
	case LF2000_BOARD_RIO:
	case LF2000_BOARD_RIO_KND_800_480:
	case LF2000_BOARD_RIO_BETA_1024_600:
	case LF2000_BOARD_RIO_BETA_800_480:
	case LF2000_BOARD_RIO_BETA_1024_600_700_400:
	case LF2000_BOARD_RIO_BETA_800_480_700_400:
	case LF2000_BOARD_RIO_BETA_1024_600_550_275:
	case LF2000_BOARD_RIO_EP_550_275:
	case LF2000_BOARD_RIO_EP_666_333:
	case LF2000_BOARD_RIO_EP_800_333:
	case LF2000_BOARD_RIO_EP_700_400:
	case LF2000_BOARD_RIO_EP_800_400:
	case LF2000_BOARD_RIO_FEP_800_327P67:
	case LF2000_BOARD_RIO_FEP_800_327P666:
		return(1);
		break;

	case LF2000_BOARD_LUCY:
	case LF2000_BOARD_LUCY_CIP:
	case LF2000_BOARD_LUCY_PP:
	case LF2000_BOARD_VTK:
	case LF2000_BOARD_UNKNOWN:
	default:
		break;
	}
	return(0);
}

/*------------------------------------------------------------------------------
 * intialize nexell soc gpio pad func.
 *
 * Note generic pad name ends in '_', while board-specific name
 * ends in '_L2K', '_M2K', and '_VTK'
 *
 */

#define PAD_EMPTY	0xFFFFFFFE	// expected empty configuration value
#define PAD_CUSTOM	0xFFFFFFFF	// placeholder for board-specific config


void init_io_pad_l2k(u32 io_pad[NUMBER_OF_GPIO_MODULE][32])
{
	/* GPIO A*/
	io_pad[0] [0] = PAD_GPIOA00_LUCY; io_pad[0] [1] = PAD_GPIOA01_LUCY;
	io_pad[0] [2] = PAD_GPIOA02_LUCY; io_pad[0] [3] = PAD_GPIOA03_LUCY;
	io_pad[0] [4] = PAD_GPIOA04_LUCY; io_pad[0] [5] = PAD_GPIOA05_LUCY;
	io_pad[0] [6] = PAD_GPIOA06_LUCY; io_pad[0] [7] = PAD_GPIOA07_LUCY;
	io_pad[0] [8] = PAD_GPIOA08_LUCY; io_pad[0] [9] = PAD_GPIOA09_LUCY;
	io_pad[0][10] = PAD_GPIOA10_LUCY; io_pad[0][11] = PAD_GPIOA11_LUCY;
	io_pad[0][12] = PAD_GPIOA12_LUCY; io_pad[0][13] = PAD_GPIOA13_LUCY;
	io_pad[0][14] = PAD_GPIOA14_LUCY; io_pad[0][15] = PAD_GPIOA15_LUCY;
	io_pad[0][16] = PAD_GPIOA16_LUCY; io_pad[0][17] = PAD_GPIOA17_LUCY;
	io_pad[0][18] = PAD_GPIOA18_LUCY; io_pad[0][19] = PAD_GPIOA19_LUCY;
	io_pad[0][20] = PAD_GPIOA20_LUCY; io_pad[0][21] = PAD_GPIOA21_LUCY;
	io_pad[0][22] = PAD_GPIOA22_LUCY; io_pad[0][23] = PAD_GPIOA23_LUCY;
	io_pad[0][24] = PAD_GPIOA24_LUCY; io_pad[0][25] = PAD_GPIOA25_LUCY;
	io_pad[0][26] = PAD_GPIOA26_LUCY; io_pad[0][27] = PAD_GPIOA27_LUCY;
	io_pad[0][28] = PAD_GPIOA28_LUCY; io_pad[0][29] = PAD_GPIOA29_LUCY;
	io_pad[0][30] = PAD_GPIOA30_LUCY; io_pad[0][31] = PAD_GPIOA31_LUCY;

	/* GPIO B*/
	io_pad[1] [0] = PAD_GPIOB00_LUCY; io_pad[1] [1] = PAD_GPIOB01_LUCY;
	io_pad[1] [2] = PAD_GPIOB02_LUCY; io_pad[1] [3] = PAD_GPIOB03_LUCY;
	io_pad[1] [4] = PAD_GPIOB04_LUCY; io_pad[1] [5] = PAD_GPIOB05_LUCY;
	io_pad[1] [6] = PAD_GPIOB06_LUCY; io_pad[1] [7] = PAD_GPIOB07_LUCY;
	io_pad[1] [8] = PAD_GPIOB08_LUCY; io_pad[1] [9] = PAD_GPIOB09_LUCY;
	io_pad[1][10] = PAD_GPIOB10_LUCY; io_pad[1][11] = PAD_GPIOB11_LUCY;
	io_pad[1][12] = PAD_GPIOB12_LUCY; io_pad[1][13] = PAD_GPIOB13_LUCY;
	io_pad[1][14] = PAD_GPIOB14_LUCY; io_pad[1][15] = PAD_GPIOB15_LUCY;
	io_pad[1][16] = PAD_GPIOB16_LUCY; io_pad[1][17] = PAD_GPIOB17_LUCY;
	io_pad[1][18] = PAD_GPIOB18_LUCY; io_pad[1][19] = PAD_GPIOB19_LUCY;
	io_pad[1][20] = PAD_GPIOB20_LUCY; io_pad[1][21] = PAD_GPIOB21_LUCY;
	io_pad[1][22] = PAD_GPIOB22_LUCY; io_pad[1][23] = PAD_GPIOB23_LUCY;
	io_pad[1][24] = PAD_GPIOB24_LUCY; io_pad[1][25] = PAD_GPIOB25_LUCY;
	io_pad[1][26] = PAD_GPIOB26_LUCY; io_pad[1][27] = PAD_GPIOB27_LUCY;
	io_pad[1][28] = PAD_GPIOB28_LUCY; io_pad[1][29] = PAD_GPIOB29_LUCY;
	io_pad[1][30] = PAD_GPIOB30_LUCY; io_pad[1][31] = PAD_GPIOB31_LUCY;

	/* GPIO C*/
	io_pad[2] [0] = PAD_GPIOC00_LUCY; io_pad[2] [1] = PAD_GPIOC01_LUCY;
	io_pad[2] [2] = PAD_GPIOC02_LUCY; io_pad[2] [3] = PAD_GPIOC03_LUCY;
	io_pad[2] [4] = PAD_GPIOC04_LUCY; io_pad[2] [5] = PAD_GPIOC05_LUCY;
	io_pad[2] [6] = PAD_GPIOC06_LUCY; io_pad[2] [7] = PAD_GPIOC07_LUCY;
	io_pad[2] [8] = PAD_GPIOC08_LUCY; io_pad[2] [9] = PAD_GPIOC09_LUCY;
	io_pad[2][10] = PAD_GPIOC10_LUCY; io_pad[2][11] = PAD_GPIOC11_LUCY;
	io_pad[2][12] = PAD_GPIOC12_LUCY; io_pad[2][13] = PAD_GPIOC13_LUCY;
	io_pad[2][14] = PAD_GPIOC14_LUCY; io_pad[2][15] = PAD_GPIOC15_LUCY;
	io_pad[2][16] = PAD_GPIOC16_LUCY; io_pad[2][17] = PAD_GPIOC17_LUCY;
	io_pad[2][18] = PAD_GPIOC18_LUCY; io_pad[2][19] = PAD_GPIOC19_LUCY;
	io_pad[2][20] = PAD_GPIOC20_LUCY; io_pad[2][21] = PAD_GPIOC21_LUCY;
	io_pad[2][22] = PAD_GPIOC22_LUCY; io_pad[2][23] = PAD_GPIOC23_LUCY;
	io_pad[2][24] = PAD_GPIOC24_LUCY; io_pad[2][25] = PAD_GPIOC25_LUCY;
	io_pad[2][26] = PAD_GPIOC26_LUCY; io_pad[2][27] = PAD_GPIOC27_LUCY;
	io_pad[2][28] = PAD_GPIOC28_LUCY; io_pad[2][29] = PAD_GPIOC29_LUCY;
	io_pad[2][30] = PAD_GPIOC30_LUCY; io_pad[2][31] = PAD_GPIOC31_LUCY;

	/* GPIO D*/
	io_pad[3] [0] = PAD_GPIOD00_LUCY; io_pad[3] [1] = PAD_GPIOD01_LUCY;
	io_pad[3] [2] = PAD_GPIOD02_LUCY; io_pad[3] [3] = PAD_GPIOD03_LUCY;
	io_pad[3] [4] = PAD_GPIOD04_LUCY; io_pad[3] [5] = PAD_GPIOD05_LUCY;
	io_pad[3] [6] = PAD_GPIOD06_LUCY; io_pad[3] [7] = PAD_GPIOD07_LUCY;
	io_pad[3] [8] = PAD_GPIOD08_LUCY; io_pad[3] [9] = PAD_GPIOD09_LUCY;
	io_pad[3][10] = PAD_GPIOD10_LUCY; io_pad[3][11] = PAD_GPIOD11_LUCY;
	io_pad[3][12] = PAD_GPIOD12_LUCY; io_pad[3][13] = PAD_GPIOD13_LUCY;
	io_pad[3][14] = PAD_GPIOD14_LUCY; io_pad[3][15] = PAD_GPIOD15_LUCY;
	io_pad[3][16] = PAD_GPIOD16_LUCY; io_pad[3][17] = PAD_GPIOD17_LUCY;
	io_pad[3][18] = PAD_GPIOD18_LUCY; io_pad[3][19] = PAD_GPIOD19_LUCY;
	io_pad[3][20] = PAD_GPIOD20_LUCY; io_pad[3][21] = PAD_GPIOD21_LUCY;
	io_pad[3][22] = PAD_GPIOD22_LUCY; io_pad[3][23] = PAD_GPIOD23_LUCY;
	io_pad[3][24] = PAD_GPIOD24_LUCY;
}

void init_io_pad_lucy_cip(u32 io_pad[NUMBER_OF_GPIO_MODULE][32])
{
	/* GPIO A*/
	io_pad[0] [0] = PAD_GPIOA00_LUCY_CIP; io_pad[0] [1] = PAD_GPIOA01_LUCY_CIP;
	io_pad[0] [2] = PAD_GPIOA02_LUCY_CIP; io_pad[0] [3] = PAD_GPIOA03_LUCY_CIP;
	io_pad[0] [4] = PAD_GPIOA04_LUCY_CIP; io_pad[0] [5] = PAD_GPIOA05_LUCY_CIP;
	io_pad[0] [6] = PAD_GPIOA06_LUCY_CIP; io_pad[0] [7] = PAD_GPIOA07_LUCY_CIP;
	io_pad[0] [8] = PAD_GPIOA08_LUCY_CIP; io_pad[0] [9] = PAD_GPIOA09_LUCY_CIP;
	io_pad[0][10] = PAD_GPIOA10_LUCY_CIP; io_pad[0][11] = PAD_GPIOA11_LUCY_CIP;
	io_pad[0][12] = PAD_GPIOA12_LUCY_CIP; io_pad[0][13] = PAD_GPIOA13_LUCY_CIP;
	io_pad[0][14] = PAD_GPIOA14_LUCY_CIP; io_pad[0][15] = PAD_GPIOA15_LUCY_CIP;
	io_pad[0][16] = PAD_GPIOA16_LUCY_CIP; io_pad[0][17] = PAD_GPIOA17_LUCY_CIP;
	io_pad[0][18] = PAD_GPIOA18_LUCY_CIP; io_pad[0][19] = PAD_GPIOA19_LUCY_CIP;
	io_pad[0][20] = PAD_GPIOA20_LUCY_CIP; io_pad[0][21] = PAD_GPIOA21_LUCY_CIP;
	io_pad[0][22] = PAD_GPIOA22_LUCY_CIP; io_pad[0][23] = PAD_GPIOA23_LUCY_CIP;
	io_pad[0][24] = PAD_GPIOA24_LUCY_CIP; io_pad[0][25] = PAD_GPIOA25_LUCY_CIP;
	io_pad[0][26] = PAD_GPIOA26_LUCY_CIP; io_pad[0][27] = PAD_GPIOA27_LUCY_CIP;
	io_pad[0][28] = PAD_GPIOA28_LUCY_CIP; io_pad[0][29] = PAD_GPIOA29_LUCY_CIP;
	io_pad[0][30] = PAD_GPIOA30_LUCY_CIP; io_pad[0][31] = PAD_GPIOA31_LUCY_CIP;

	/* GPIO B*/
	io_pad[1] [0] = PAD_GPIOB00_LUCY_CIP; io_pad[1] [1] = PAD_GPIOB01_LUCY_CIP;
	io_pad[1] [2] = PAD_GPIOB02_LUCY_CIP; io_pad[1] [3] = PAD_GPIOB03_LUCY_CIP;
	io_pad[1] [4] = PAD_GPIOB04_LUCY_CIP; io_pad[1] [5] = PAD_GPIOB05_LUCY_CIP;
	io_pad[1] [6] = PAD_GPIOB06_LUCY_CIP; io_pad[1] [7] = PAD_GPIOB07_LUCY_CIP;
	io_pad[1] [8] = PAD_GPIOB08_LUCY_CIP; io_pad[1] [9] = PAD_GPIOB09_LUCY_CIP;
	io_pad[1][10] = PAD_GPIOB10_LUCY_CIP; io_pad[1][11] = PAD_GPIOB11_LUCY_CIP;
	io_pad[1][12] = PAD_GPIOB12_LUCY_CIP; io_pad[1][13] = PAD_GPIOB13_LUCY_CIP;
	io_pad[1][14] = PAD_GPIOB14_LUCY_CIP; io_pad[1][15] = PAD_GPIOB15_LUCY_CIP;
	io_pad[1][16] = PAD_GPIOB16_LUCY_CIP; io_pad[1][17] = PAD_GPIOB17_LUCY_CIP;
	io_pad[1][18] = PAD_GPIOB18_LUCY_CIP; io_pad[1][19] = PAD_GPIOB19_LUCY_CIP;
	io_pad[1][20] = PAD_GPIOB20_LUCY_CIP; io_pad[1][21] = PAD_GPIOB21_LUCY_CIP;
	io_pad[1][22] = PAD_GPIOB22_LUCY_CIP; io_pad[1][23] = PAD_GPIOB23_LUCY_CIP;
	io_pad[1][24] = PAD_GPIOB24_LUCY_CIP; io_pad[1][25] = PAD_GPIOB25_LUCY_CIP;
	io_pad[1][26] = PAD_GPIOB26_LUCY_CIP; io_pad[1][27] = PAD_GPIOB27_LUCY_CIP;
	io_pad[1][28] = PAD_GPIOB28_LUCY_CIP; io_pad[1][29] = PAD_GPIOB29_LUCY_CIP;
	io_pad[1][30] = PAD_GPIOB30_LUCY_CIP; io_pad[1][31] = PAD_GPIOB31_LUCY_CIP;

	/* GPIO C*/
	io_pad[2] [0] = PAD_GPIOC00_LUCY_CIP; io_pad[2] [1] = PAD_GPIOC01_LUCY_CIP;
	io_pad[2] [2] = PAD_GPIOC02_LUCY_CIP; io_pad[2] [3] = PAD_GPIOC03_LUCY_CIP;
	io_pad[2] [4] = PAD_GPIOC04_LUCY_CIP; io_pad[2] [5] = PAD_GPIOC05_LUCY_CIP;
	io_pad[2] [6] = PAD_GPIOC06_LUCY_CIP; io_pad[2] [7] = PAD_GPIOC07_LUCY_CIP;
	io_pad[2] [8] = PAD_GPIOC08_LUCY_CIP; io_pad[2] [9] = PAD_GPIOC09_LUCY_CIP;
	io_pad[2][10] = PAD_GPIOC10_LUCY_CIP; io_pad[2][11] = PAD_GPIOC11_LUCY_CIP;
	io_pad[2][12] = PAD_GPIOC12_LUCY_CIP; io_pad[2][13] = PAD_GPIOC13_LUCY_CIP;
	io_pad[2][14] = PAD_GPIOC14_LUCY_CIP; io_pad[2][15] = PAD_GPIOC15_LUCY_CIP;
	io_pad[2][16] = PAD_GPIOC16_LUCY_CIP; io_pad[2][17] = PAD_GPIOC17_LUCY_CIP;
	io_pad[2][18] = PAD_GPIOC18_LUCY_CIP; io_pad[2][19] = PAD_GPIOC19_LUCY_CIP;
	io_pad[2][20] = PAD_GPIOC20_LUCY_CIP; io_pad[2][21] = PAD_GPIOC21_LUCY_CIP;
	io_pad[2][22] = PAD_GPIOC22_LUCY_CIP; io_pad[2][23] = PAD_GPIOC23_LUCY_CIP;
	io_pad[2][24] = PAD_GPIOC24_LUCY_CIP; io_pad[2][25] = PAD_GPIOC25_LUCY_CIP;
	io_pad[2][26] = PAD_GPIOC26_LUCY_CIP; io_pad[2][27] = PAD_GPIOC27_LUCY_CIP;
	io_pad[2][28] = PAD_GPIOC28_LUCY_CIP; io_pad[2][29] = PAD_GPIOC29_LUCY_CIP;
	io_pad[2][30] = PAD_GPIOC30_LUCY_CIP; io_pad[2][31] = PAD_GPIOC31_LUCY_CIP;

	/* GPIO D*/
	io_pad[3] [0] = PAD_GPIOD00_LUCY_CIP; io_pad[3] [1] = PAD_GPIOD01_LUCY_CIP;
	io_pad[3] [2] = PAD_GPIOD02_LUCY_CIP; io_pad[3] [3] = PAD_GPIOD03_LUCY_CIP;
	io_pad[3] [4] = PAD_GPIOD04_LUCY_CIP; io_pad[3] [5] = PAD_GPIOD05_LUCY_CIP;
	io_pad[3] [6] = PAD_GPIOD06_LUCY_CIP; io_pad[3] [7] = PAD_GPIOD07_LUCY_CIP;
	io_pad[3] [8] = PAD_GPIOD08_LUCY_CIP; io_pad[3] [9] = PAD_GPIOD09_LUCY_CIP;
	io_pad[3][10] = PAD_GPIOD10_LUCY_CIP; io_pad[3][11] = PAD_GPIOD11_LUCY_CIP;
	io_pad[3][12] = PAD_GPIOD12_LUCY_CIP; io_pad[3][13] = PAD_GPIOD13_LUCY_CIP;
	io_pad[3][14] = PAD_GPIOD14_LUCY_CIP; io_pad[3][15] = PAD_GPIOD15_LUCY_CIP;
	io_pad[3][16] = PAD_GPIOD16_LUCY_CIP; io_pad[3][17] = PAD_GPIOD17_LUCY_CIP;
	io_pad[3][18] = PAD_GPIOD18_LUCY_CIP; io_pad[3][19] = PAD_GPIOD19_LUCY_CIP;
	io_pad[3][20] = PAD_GPIOD20_LUCY_CIP; io_pad[3][21] = PAD_GPIOD21_LUCY_CIP;
	io_pad[3][22] = PAD_GPIOD22_LUCY_CIP; io_pad[3][23] = PAD_GPIOD23_LUCY_CIP;
	io_pad[3][24] = PAD_GPIOD24_LUCY_CIP;
}

void init_io_pad_m2k(u32 io_pad[NUMBER_OF_GPIO_MODULE][32])
{
	/* GPIO A*/
	io_pad[0] [0] = PAD_GPIOA00_VAL; io_pad[0] [1] = PAD_GPIOA01_VAL;
	io_pad[0] [2] = PAD_GPIOA02_VAL; io_pad[0] [3] = PAD_GPIOA03_VAL;
	io_pad[0] [4] = PAD_GPIOA04_VAL; io_pad[0] [5] = PAD_GPIOA05_VAL;
	io_pad[0] [6] = PAD_GPIOA06_VAL; io_pad[0] [7] = PAD_GPIOA07_VAL;
	io_pad[0] [8] = PAD_GPIOA08_VAL; io_pad[0] [9] = PAD_GPIOA09_VAL;
	io_pad[0][10] = PAD_GPIOA10_VAL; io_pad[0][11] = PAD_GPIOA11_VAL;
	io_pad[0][12] = PAD_GPIOA12_VAL; io_pad[0][13] = PAD_GPIOA13_VAL;
	io_pad[0][14] = PAD_GPIOA14_VAL; io_pad[0][15] = PAD_GPIOA15_VAL;
	io_pad[0][16] = PAD_GPIOA16_VAL; io_pad[0][17] = PAD_GPIOA17_VAL;
	io_pad[0][18] = PAD_GPIOA18_VAL; io_pad[0][19] = PAD_GPIOA19_VAL;
	io_pad[0][20] = PAD_GPIOA20_VAL; io_pad[0][21] = PAD_GPIOA21_VAL;
	io_pad[0][22] = PAD_GPIOA22_VAL; io_pad[0][23] = PAD_GPIOA23_VAL;
	io_pad[0][24] = PAD_GPIOA24_VAL; io_pad[0][25] = PAD_GPIOA25_VAL;
	io_pad[0][26] = PAD_GPIOA26_VAL; io_pad[0][27] = PAD_GPIOA27_VAL;
	io_pad[0][28] = PAD_GPIOA28_VAL; io_pad[0][29] = PAD_GPIOA29_VAL;
	io_pad[0][30] = PAD_GPIOA30_VAL; io_pad[0][31] = PAD_GPIOA31_VAL;

	/* GPIO B*/
	io_pad[1] [0] = PAD_GPIOB00_VAL; io_pad[1] [1] = PAD_GPIOB01_VAL;
	io_pad[1] [2] = PAD_GPIOB02_VAL; io_pad[1] [3] = PAD_GPIOB03_VAL;
	io_pad[1] [4] = PAD_GPIOB04_VAL; io_pad[1] [5] = PAD_GPIOB05_VAL;
	io_pad[1] [6] = PAD_GPIOB06_VAL; io_pad[1] [7] = PAD_GPIOB07_VAL;
	io_pad[1] [8] = PAD_GPIOB08_VAL; io_pad[1] [9] = PAD_GPIOB09_VAL;
	io_pad[1][10] = PAD_GPIOB10_VAL; io_pad[1][11] = PAD_GPIOB11_VAL;
	io_pad[1][12] = PAD_GPIOB12_VAL; io_pad[1][13] = PAD_GPIOB13_VAL;
	io_pad[1][14] = PAD_GPIOB14_VAL; io_pad[1][15] = PAD_GPIOB15_VAL;
	io_pad[1][16] = PAD_GPIOB16_VAL; io_pad[1][17] = PAD_GPIOB17_VAL;
	io_pad[1][18] = PAD_GPIOB18_VAL; io_pad[1][19] = PAD_GPIOB19_VAL;
	io_pad[1][20] = PAD_GPIOB20_VAL; io_pad[1][21] = PAD_GPIOB21_VAL;
	io_pad[1][22] = PAD_GPIOB22_VAL; io_pad[1][23] = PAD_GPIOB23_VAL;
	io_pad[1][24] = PAD_GPIOB24_VAL; io_pad[1][25] = PAD_GPIOB25_VAL;
	io_pad[1][26] = PAD_GPIOB26_VAL; io_pad[1][27] = PAD_GPIOB27_VAL;
	io_pad[1][28] = PAD_GPIOB28_VAL; io_pad[1][29] = PAD_GPIOB29_VAL;
	io_pad[1][30] = PAD_GPIOB30_VAL; io_pad[1][31] = PAD_GPIOB31_VAL;

	/* GPIO C*/
	io_pad[2] [0] = PAD_GPIOC00_VAL; io_pad[2] [1] = PAD_GPIOC01_VAL;
	io_pad[2] [2] = PAD_GPIOC02_VAL; io_pad[2] [3] = PAD_GPIOC03_VAL;
	io_pad[2] [4] = PAD_GPIOC04_VAL; io_pad[2] [5] = PAD_GPIOC05_VAL;
	io_pad[2] [6] = PAD_GPIOC06_VAL; io_pad[2] [7] = PAD_GPIOC07_VAL;
	io_pad[2] [8] = PAD_GPIOC08_VAL; io_pad[2] [9] = PAD_GPIOC09_VAL;
	io_pad[2][10] = PAD_GPIOC10_VAL; io_pad[2][11] = PAD_GPIOC11_VAL;
	io_pad[2][12] = PAD_GPIOC12_VAL; io_pad[2][13] = PAD_GPIOC13_VAL;
	io_pad[2][14] = PAD_GPIOC14_VAL; io_pad[2][15] = PAD_GPIOC15_VAL;
	io_pad[2][16] = PAD_GPIOC16_VAL; io_pad[2][17] = PAD_GPIOC17_VAL;
	io_pad[2][18] = PAD_GPIOC18_VAL; io_pad[2][19] = PAD_GPIOC19_VAL;
	io_pad[2][20] = PAD_GPIOC20_VAL; io_pad[2][21] = PAD_GPIOC21_VAL;
	io_pad[2][22] = PAD_GPIOC22_VAL; io_pad[2][23] = PAD_GPIOC23_VAL;
	io_pad[2][24] = PAD_GPIOC24_VAL; io_pad[2][25] = PAD_GPIOC25_VAL;
	io_pad[2][26] = PAD_GPIOC26_VAL; io_pad[2][27] = PAD_GPIOC27_VAL;
	io_pad[2][28] = PAD_GPIOC28_VAL; io_pad[2][29] = PAD_GPIOC29_VAL;
	io_pad[2][30] = PAD_GPIOC30_VAL; io_pad[2][31] = PAD_GPIOC31_VAL;

	/* GPIO D*/
	io_pad[3] [0] = PAD_GPIOD00_VAL; io_pad[3] [1] = PAD_GPIOD01_VAL;
	io_pad[3] [2] = PAD_GPIOD02_VAL; io_pad[3] [3] = PAD_GPIOD03_VAL;
	io_pad[3] [4] = PAD_GPIOD04_VAL; io_pad[3] [5] = PAD_GPIOD05_VAL;
	io_pad[3] [6] = PAD_GPIOD06_VAL; io_pad[3] [7] = PAD_GPIOD07_VAL;
	io_pad[3] [8] = PAD_GPIOD08_VAL; io_pad[3] [9] = PAD_GPIOD09_VAL;
	io_pad[3][10] = PAD_GPIOD10_VAL; io_pad[3][11] = PAD_GPIOD11_VAL;
	io_pad[3][12] = PAD_GPIOD12_VAL; io_pad[3][13] = PAD_GPIOD13_VAL;
	io_pad[3][14] = PAD_GPIOD14_VAL; io_pad[3][15] = PAD_GPIOD15_VAL;
	io_pad[3][16] = PAD_GPIOD16_VAL; io_pad[3][17] = PAD_GPIOD17_VAL;
	io_pad[3][18] = PAD_GPIOD18_VAL; io_pad[3][19] = PAD_GPIOD19_VAL;
	io_pad[3][20] = PAD_GPIOD20_VAL; io_pad[3][21] = PAD_GPIOD21_VAL;
	io_pad[3][22] = PAD_GPIOD22_VAL; io_pad[3][23] = PAD_GPIOD23_VAL;
	io_pad[3][24] = PAD_GPIOD24_VAL;
}

void init_io_pad_rio(u32 io_pad[NUMBER_OF_GPIO_MODULE][32])
{
	/* GPIO A*/
	io_pad[0] [0] = PAD_GPIOA00_RIO; io_pad[0] [1] = PAD_GPIOA01_RIO;
	io_pad[0] [2] = PAD_GPIOA02_RIO; io_pad[0] [3] = PAD_GPIOA03_RIO;
	io_pad[0] [4] = PAD_GPIOA04_RIO; io_pad[0] [5] = PAD_GPIOA05_RIO;
	io_pad[0] [6] = PAD_GPIOA06_RIO; io_pad[0] [7] = PAD_GPIOA07_RIO;
	io_pad[0] [8] = PAD_GPIOA08_RIO; io_pad[0] [9] = PAD_GPIOA09_RIO;
	io_pad[0][10] = PAD_GPIOA10_RIO; io_pad[0][11] = PAD_GPIOA11_RIO;
	io_pad[0][12] = PAD_GPIOA12_RIO; io_pad[0][13] = PAD_GPIOA13_RIO;
	io_pad[0][14] = PAD_GPIOA14_RIO; io_pad[0][15] = PAD_GPIOA15_RIO;
	io_pad[0][16] = PAD_GPIOA16_RIO; io_pad[0][17] = PAD_GPIOA17_RIO;
	io_pad[0][18] = PAD_GPIOA18_RIO; io_pad[0][19] = PAD_GPIOA19_RIO;
	io_pad[0][20] = PAD_GPIOA20_RIO; io_pad[0][21] = PAD_GPIOA21_RIO;
	io_pad[0][22] = PAD_GPIOA22_RIO; io_pad[0][23] = PAD_GPIOA23_RIO;
	io_pad[0][24] = PAD_GPIOA24_RIO; io_pad[0][25] = PAD_GPIOA25_RIO;
	io_pad[0][26] = PAD_GPIOA26_RIO; io_pad[0][27] = PAD_GPIOA27_RIO;
	io_pad[0][28] = PAD_GPIOA28_RIO; io_pad[0][29] = PAD_GPIOA29_RIO;
	io_pad[0][30] = PAD_GPIOA30_RIO; io_pad[0][31] = PAD_GPIOA31_RIO;

	/* GPIO B*/
	io_pad[1] [0] = PAD_GPIOB00_RIO; io_pad[1] [1] = PAD_GPIOB01_RIO;
	io_pad[1] [2] = PAD_GPIOB02_RIO; io_pad[1] [3] = PAD_GPIOB03_RIO;
	io_pad[1] [4] = PAD_GPIOB04_RIO; io_pad[1] [5] = PAD_GPIOB05_RIO;
	io_pad[1] [6] = PAD_GPIOB06_RIO; io_pad[1] [7] = PAD_GPIOB07_RIO;
	io_pad[1] [8] = PAD_GPIOB08_RIO; io_pad[1] [9] = PAD_GPIOB09_RIO;
	io_pad[1][10] = PAD_GPIOB10_RIO; io_pad[1][11] = PAD_GPIOB11_RIO;
	io_pad[1][12] = PAD_GPIOB12_RIO; io_pad[1][13] = PAD_GPIOB13_RIO;
	io_pad[1][14] = PAD_GPIOB14_RIO; io_pad[1][15] = PAD_GPIOB15_RIO;
	io_pad[1][16] = PAD_GPIOB16_RIO; io_pad[1][17] = PAD_GPIOB17_RIO;
	io_pad[1][18] = PAD_GPIOB18_RIO; io_pad[1][19] = PAD_GPIOB19_RIO;
	io_pad[1][20] = PAD_GPIOB20_RIO; io_pad[1][21] = PAD_GPIOB21_RIO;
	io_pad[1][22] = PAD_GPIOB22_RIO; io_pad[1][23] = PAD_GPIOB23_RIO;
	io_pad[1][24] = PAD_GPIOB24_RIO; io_pad[1][25] = PAD_GPIOB25_RIO;
	io_pad[1][26] = PAD_GPIOB26_RIO; io_pad[1][27] = PAD_GPIOB27_RIO;
	io_pad[1][28] = PAD_GPIOB28_RIO; io_pad[1][29] = PAD_GPIOB29_RIO;
	io_pad[1][30] = PAD_GPIOB30_RIO; io_pad[1][31] = PAD_GPIOB31_RIO;

	/* GPIO C*/
	io_pad[2] [0] = PAD_GPIOC00_RIO; io_pad[2] [1] = PAD_GPIOC01_RIO;
	io_pad[2] [2] = PAD_GPIOC02_RIO; io_pad[2] [3] = PAD_GPIOC03_RIO;
	io_pad[2] [4] = PAD_GPIOC04_RIO; io_pad[2] [5] = PAD_GPIOC05_RIO;
	io_pad[2] [6] = PAD_GPIOC06_RIO; io_pad[2] [7] = PAD_GPIOC07_RIO;
	io_pad[2] [8] = PAD_GPIOC08_RIO; io_pad[2] [9] = PAD_GPIOC09_RIO;
	io_pad[2][10] = PAD_GPIOC10_RIO; io_pad[2][11] = PAD_GPIOC11_RIO;
	io_pad[2][12] = PAD_GPIOC12_RIO; io_pad[2][13] = PAD_GPIOC13_RIO;
	io_pad[2][14] = PAD_GPIOC14_RIO; io_pad[2][15] = PAD_GPIOC15_RIO;
	io_pad[2][16] = PAD_GPIOC16_RIO; io_pad[2][17] = PAD_GPIOC17_RIO;
	io_pad[2][18] = PAD_GPIOC18_RIO; io_pad[2][19] = PAD_GPIOC19_RIO;
	io_pad[2][20] = PAD_GPIOC20_RIO; io_pad[2][21] = PAD_GPIOC21_RIO;
	io_pad[2][22] = PAD_GPIOC22_RIO; io_pad[2][23] = PAD_GPIOC23_RIO;
	io_pad[2][24] = PAD_GPIOC24_RIO; io_pad[2][25] = PAD_GPIOC25_RIO;
	io_pad[2][26] = PAD_GPIOC26_RIO; io_pad[2][27] = PAD_GPIOC27_RIO;
	io_pad[2][28] = PAD_GPIOC28_RIO; io_pad[2][29] = PAD_GPIOC29_RIO;
	io_pad[2][30] = PAD_GPIOC30_RIO; io_pad[2][31] = PAD_GPIOC31_RIO;

	/* GPIO D*/
	io_pad[3] [0] = PAD_GPIOD00_RIO; io_pad[3] [1] = PAD_GPIOD01_RIO;
	io_pad[3] [2] = PAD_GPIOD02_RIO; io_pad[3] [3] = PAD_GPIOD03_RIO;
	io_pad[3] [4] = PAD_GPIOD04_RIO; io_pad[3] [5] = PAD_GPIOD05_RIO;
	io_pad[3] [6] = PAD_GPIOD06_RIO; io_pad[3] [7] = PAD_GPIOD07_RIO;
	io_pad[3] [8] = PAD_GPIOD08_RIO; io_pad[3] [9] = PAD_GPIOD09_RIO;
	io_pad[3][10] = PAD_GPIOD10_RIO; io_pad[3][11] = PAD_GPIOD11_RIO;
	io_pad[3][12] = PAD_GPIOD12_RIO; io_pad[3][13] = PAD_GPIOD13_RIO;
	io_pad[3][14] = PAD_GPIOD14_RIO; io_pad[3][15] = PAD_GPIOD15_RIO;
	io_pad[3][16] = PAD_GPIOD16_RIO; io_pad[3][17] = PAD_GPIOD17_RIO;
	io_pad[3][18] = PAD_GPIOD18_RIO; io_pad[3][19] = PAD_GPIOD19_RIO;
	io_pad[3][20] = PAD_GPIOD20_RIO; io_pad[3][21] = PAD_GPIOD21_RIO;
	io_pad[3][22] = PAD_GPIOD22_RIO; io_pad[3][23] = PAD_GPIOD23_RIO;
	io_pad[3][24] = PAD_GPIOD24_RIO;
}

void init_io_pad_rio_beta(u32 io_pad[NUMBER_OF_GPIO_MODULE][32])
{
	/* GPIO A*/
	io_pad[0] [0] = PAD_GPIOA00_RIO_BETA; io_pad[0] [1] = PAD_GPIOA01_RIO_BETA;
	io_pad[0] [2] = PAD_GPIOA02_RIO_BETA; io_pad[0] [3] = PAD_GPIOA03_RIO_BETA;
	io_pad[0] [4] = PAD_GPIOA04_RIO_BETA; io_pad[0] [5] = PAD_GPIOA05_RIO_BETA;
	io_pad[0] [6] = PAD_GPIOA06_RIO_BETA; io_pad[0] [7] = PAD_GPIOA07_RIO_BETA;
	io_pad[0] [8] = PAD_GPIOA08_RIO_BETA; io_pad[0] [9] = PAD_GPIOA09_RIO_BETA;
	io_pad[0][10] = PAD_GPIOA10_RIO_BETA; io_pad[0][11] = PAD_GPIOA11_RIO_BETA;
	io_pad[0][12] = PAD_GPIOA12_RIO_BETA; io_pad[0][13] = PAD_GPIOA13_RIO_BETA;
	io_pad[0][14] = PAD_GPIOA14_RIO_BETA; io_pad[0][15] = PAD_GPIOA15_RIO_BETA;
	io_pad[0][16] = PAD_GPIOA16_RIO_BETA; io_pad[0][17] = PAD_GPIOA17_RIO_BETA;
	io_pad[0][18] = PAD_GPIOA18_RIO_BETA; io_pad[0][19] = PAD_GPIOA19_RIO_BETA;
	io_pad[0][20] = PAD_GPIOA20_RIO_BETA; io_pad[0][21] = PAD_GPIOA21_RIO_BETA;
	io_pad[0][22] = PAD_GPIOA22_RIO_BETA; io_pad[0][23] = PAD_GPIOA23_RIO_BETA;
	io_pad[0][24] = PAD_GPIOA24_RIO_BETA; io_pad[0][25] = PAD_GPIOA25_RIO_BETA;
	io_pad[0][26] = PAD_GPIOA26_RIO_BETA; io_pad[0][27] = PAD_GPIOA27_RIO_BETA;
	io_pad[0][28] = PAD_GPIOA28_RIO_BETA; io_pad[0][29] = PAD_GPIOA29_RIO_BETA;
	io_pad[0][30] = PAD_GPIOA30_RIO_BETA; io_pad[0][31] = PAD_GPIOA31_RIO_BETA;

	/* GPIO B*/
	io_pad[1] [0] = PAD_GPIOB00_RIO_BETA; io_pad[1] [1] = PAD_GPIOB01_RIO_BETA;
	io_pad[1] [2] = PAD_GPIOB02_RIO_BETA; io_pad[1] [3] = PAD_GPIOB03_RIO_BETA;
	io_pad[1] [4] = PAD_GPIOB04_RIO_BETA; io_pad[1] [5] = PAD_GPIOB05_RIO_BETA;
	io_pad[1] [6] = PAD_GPIOB06_RIO_BETA; io_pad[1] [7] = PAD_GPIOB07_RIO_BETA;
	io_pad[1] [8] = PAD_GPIOB08_RIO_BETA; io_pad[1] [9] = PAD_GPIOB09_RIO_BETA;
	io_pad[1][10] = PAD_GPIOB10_RIO_BETA; io_pad[1][11] = PAD_GPIOB11_RIO_BETA;
	io_pad[1][12] = PAD_GPIOB12_RIO_BETA; io_pad[1][13] = PAD_GPIOB13_RIO_BETA;
	io_pad[1][14] = PAD_GPIOB14_RIO_BETA; io_pad[1][15] = PAD_GPIOB15_RIO_BETA;
	io_pad[1][16] = PAD_GPIOB16_RIO_BETA; io_pad[1][17] = PAD_GPIOB17_RIO_BETA;
	io_pad[1][18] = PAD_GPIOB18_RIO_BETA; io_pad[1][19] = PAD_GPIOB19_RIO_BETA;
	io_pad[1][20] = PAD_GPIOB20_RIO_BETA; io_pad[1][21] = PAD_GPIOB21_RIO_BETA;
	io_pad[1][22] = PAD_GPIOB22_RIO_BETA; io_pad[1][23] = PAD_GPIOB23_RIO_BETA;
	io_pad[1][24] = PAD_GPIOB24_RIO_BETA; io_pad[1][25] = PAD_GPIOB25_RIO_BETA;
	io_pad[1][26] = PAD_GPIOB26_RIO_BETA; io_pad[1][27] = PAD_GPIOB27_RIO_BETA;
	io_pad[1][28] = PAD_GPIOB28_RIO_BETA; io_pad[1][29] = PAD_GPIOB29_RIO_BETA;
	io_pad[1][30] = PAD_GPIOB30_RIO_BETA; io_pad[1][31] = PAD_GPIOB31_RIO_BETA;

	/* GPIO C*/
	io_pad[2] [0] = PAD_GPIOC00_RIO_BETA; io_pad[2] [1] = PAD_GPIOC01_RIO_BETA;
	io_pad[2] [2] = PAD_GPIOC02_RIO_BETA; io_pad[2] [3] = PAD_GPIOC03_RIO_BETA;
	io_pad[2] [4] = PAD_GPIOC04_RIO_BETA; io_pad[2] [5] = PAD_GPIOC05_RIO_BETA;
	io_pad[2] [6] = PAD_GPIOC06_RIO_BETA; io_pad[2] [7] = PAD_GPIOC07_RIO_BETA;
	io_pad[2] [8] = PAD_GPIOC08_RIO_BETA; io_pad[2] [9] = PAD_GPIOC09_RIO_BETA;
	io_pad[2][10] = PAD_GPIOC10_RIO_BETA; io_pad[2][11] = PAD_GPIOC11_RIO_BETA;
	io_pad[2][12] = PAD_GPIOC12_RIO_BETA; io_pad[2][13] = PAD_GPIOC13_RIO_BETA;
	io_pad[2][14] = PAD_GPIOC14_RIO_BETA; io_pad[2][15] = PAD_GPIOC15_RIO_BETA;
	io_pad[2][16] = PAD_GPIOC16_RIO_BETA; io_pad[2][17] = PAD_GPIOC17_RIO_BETA;
	io_pad[2][18] = PAD_GPIOC18_RIO_BETA; io_pad[2][19] = PAD_GPIOC19_RIO_BETA;
	io_pad[2][20] = PAD_GPIOC20_RIO_BETA; io_pad[2][21] = PAD_GPIOC21_RIO_BETA;
	io_pad[2][22] = PAD_GPIOC22_RIO_BETA; io_pad[2][23] = PAD_GPIOC23_RIO_BETA;
	io_pad[2][24] = PAD_GPIOC24_RIO_BETA; io_pad[2][25] = PAD_GPIOC25_RIO_BETA;
	io_pad[2][26] = PAD_GPIOC26_RIO_BETA; io_pad[2][27] = PAD_GPIOC27_RIO_BETA;
	io_pad[2][28] = PAD_GPIOC28_RIO_BETA; io_pad[2][29] = PAD_GPIOC29_RIO_BETA;
	io_pad[2][30] = PAD_GPIOC30_RIO_BETA; io_pad[2][31] = PAD_GPIOC31_RIO_BETA;

	/* GPIO D*/
	io_pad[3] [0] = PAD_GPIOD00_RIO_BETA; io_pad[3] [1] = PAD_GPIOD01_RIO_BETA;
	io_pad[3] [2] = PAD_GPIOD02_RIO_BETA; io_pad[3] [3] = PAD_GPIOD03_RIO_BETA;
	io_pad[3] [4] = PAD_GPIOD04_RIO_BETA; io_pad[3] [5] = PAD_GPIOD05_RIO_BETA;
	io_pad[3] [6] = PAD_GPIOD06_RIO_BETA; io_pad[3] [7] = PAD_GPIOD07_RIO_BETA;
	io_pad[3] [8] = PAD_GPIOD08_RIO_BETA; io_pad[3] [9] = PAD_GPIOD09_RIO_BETA;
	io_pad[3][10] = PAD_GPIOD10_RIO_BETA; io_pad[3][11] = PAD_GPIOD11_RIO_BETA;
	io_pad[3][12] = PAD_GPIOD12_RIO_BETA; io_pad[3][13] = PAD_GPIOD13_RIO_BETA;
	io_pad[3][14] = PAD_GPIOD14_RIO_BETA; io_pad[3][15] = PAD_GPIOD15_RIO_BETA;
	io_pad[3][16] = PAD_GPIOD16_RIO_BETA; io_pad[3][17] = PAD_GPIOD17_RIO_BETA;
	io_pad[3][18] = PAD_GPIOD18_RIO_BETA; io_pad[3][19] = PAD_GPIOD19_RIO_BETA;
	io_pad[3][20] = PAD_GPIOD20_RIO_BETA; io_pad[3][21] = PAD_GPIOD21_RIO_BETA;
	io_pad[3][22] = PAD_GPIOD22_RIO_BETA; io_pad[3][23] = PAD_GPIOD23_RIO_BETA;
	io_pad[3][24] = PAD_GPIOD24_RIO_BETA;
}

void init_io_pad_rio_ep(u32 io_pad[NUMBER_OF_GPIO_MODULE][32])
{
	/* GPIO A*/
	io_pad[0] [0] = PAD_GPIOA00_RIO_EP; io_pad[0] [1] = PAD_GPIOA01_RIO_EP;
	io_pad[0] [2] = PAD_GPIOA02_RIO_EP; io_pad[0] [3] = PAD_GPIOA03_RIO_EP;
	io_pad[0] [4] = PAD_GPIOA04_RIO_EP; io_pad[0] [5] = PAD_GPIOA05_RIO_EP;
	io_pad[0] [6] = PAD_GPIOA06_RIO_EP; io_pad[0] [7] = PAD_GPIOA07_RIO_EP;
	io_pad[0] [8] = PAD_GPIOA08_RIO_EP; io_pad[0] [9] = PAD_GPIOA09_RIO_EP;
	io_pad[0][10] = PAD_GPIOA10_RIO_EP; io_pad[0][11] = PAD_GPIOA11_RIO_EP;
	io_pad[0][12] = PAD_GPIOA12_RIO_EP; io_pad[0][13] = PAD_GPIOA13_RIO_EP;
	io_pad[0][14] = PAD_GPIOA14_RIO_EP; io_pad[0][15] = PAD_GPIOA15_RIO_EP;
	io_pad[0][16] = PAD_GPIOA16_RIO_EP; io_pad[0][17] = PAD_GPIOA17_RIO_EP;
	io_pad[0][18] = PAD_GPIOA18_RIO_EP; io_pad[0][19] = PAD_GPIOA19_RIO_EP;
	io_pad[0][20] = PAD_GPIOA20_RIO_EP; io_pad[0][21] = PAD_GPIOA21_RIO_EP;
	io_pad[0][22] = PAD_GPIOA22_RIO_EP; io_pad[0][23] = PAD_GPIOA23_RIO_EP;
	io_pad[0][24] = PAD_GPIOA24_RIO_EP; io_pad[0][25] = PAD_GPIOA25_RIO_EP;
	io_pad[0][26] = PAD_GPIOA26_RIO_EP; io_pad[0][27] = PAD_GPIOA27_RIO_EP;
	io_pad[0][28] = PAD_GPIOA28_RIO_EP; io_pad[0][29] = PAD_GPIOA29_RIO_EP;
	io_pad[0][30] = PAD_GPIOA30_RIO_EP; io_pad[0][31] = PAD_GPIOA31_RIO_EP;

	/* GPIO B*/
	io_pad[1] [0] = PAD_GPIOB00_RIO_EP; io_pad[1] [1] = PAD_GPIOB01_RIO_EP;
	io_pad[1] [2] = PAD_GPIOB02_RIO_EP; io_pad[1] [3] = PAD_GPIOB03_RIO_EP;
	io_pad[1] [4] = PAD_GPIOB04_RIO_EP; io_pad[1] [5] = PAD_GPIOB05_RIO_EP;
	io_pad[1] [6] = PAD_GPIOB06_RIO_EP; io_pad[1] [7] = PAD_GPIOB07_RIO_EP;
	io_pad[1] [8] = PAD_GPIOB08_RIO_EP; io_pad[1] [9] = PAD_GPIOB09_RIO_EP;
	io_pad[1][10] = PAD_GPIOB10_RIO_EP; io_pad[1][11] = PAD_GPIOB11_RIO_EP;
	io_pad[1][12] = PAD_GPIOB12_RIO_EP; io_pad[1][13] = PAD_GPIOB13_RIO_EP;
	io_pad[1][14] = PAD_GPIOB14_RIO_EP; io_pad[1][15] = PAD_GPIOB15_RIO_EP;
	io_pad[1][16] = PAD_GPIOB16_RIO_EP; io_pad[1][17] = PAD_GPIOB17_RIO_EP;
	io_pad[1][18] = PAD_GPIOB18_RIO_EP; io_pad[1][19] = PAD_GPIOB19_RIO_EP;
	io_pad[1][20] = PAD_GPIOB20_RIO_EP; io_pad[1][21] = PAD_GPIOB21_RIO_EP;
	io_pad[1][22] = PAD_GPIOB22_RIO_EP; io_pad[1][23] = PAD_GPIOB23_RIO_EP;
	io_pad[1][24] = PAD_GPIOB24_RIO_EP; io_pad[1][25] = PAD_GPIOB25_RIO_EP;
	io_pad[1][26] = PAD_GPIOB26_RIO_EP; io_pad[1][27] = PAD_GPIOB27_RIO_EP;
	io_pad[1][28] = PAD_GPIOB28_RIO_EP; io_pad[1][29] = PAD_GPIOB29_RIO_EP;
	io_pad[1][30] = PAD_GPIOB30_RIO_EP; io_pad[1][31] = PAD_GPIOB31_RIO_EP;

	/* GPIO C*/
	io_pad[2] [0] = PAD_GPIOC00_RIO_EP; io_pad[2] [1] = PAD_GPIOC01_RIO_EP;
	io_pad[2] [2] = PAD_GPIOC02_RIO_EP; io_pad[2] [3] = PAD_GPIOC03_RIO_EP;
	io_pad[2] [4] = PAD_GPIOC04_RIO_EP; io_pad[2] [5] = PAD_GPIOC05_RIO_EP;
	io_pad[2] [6] = PAD_GPIOC06_RIO_EP; io_pad[2] [7] = PAD_GPIOC07_RIO_EP;
	io_pad[2] [8] = PAD_GPIOC08_RIO_EP; io_pad[2] [9] = PAD_GPIOC09_RIO_EP;
	io_pad[2][10] = PAD_GPIOC10_RIO_EP; io_pad[2][11] = PAD_GPIOC11_RIO_EP;
	io_pad[2][12] = PAD_GPIOC12_RIO_EP; io_pad[2][13] = PAD_GPIOC13_RIO_EP;
	io_pad[2][14] = PAD_GPIOC14_RIO_EP; io_pad[2][15] = PAD_GPIOC15_RIO_EP;
	io_pad[2][16] = PAD_GPIOC16_RIO_EP; io_pad[2][17] = PAD_GPIOC17_RIO_EP;
	io_pad[2][18] = PAD_GPIOC18_RIO_EP; io_pad[2][19] = PAD_GPIOC19_RIO_EP;
	io_pad[2][20] = PAD_GPIOC20_RIO_EP; io_pad[2][21] = PAD_GPIOC21_RIO_EP;
	io_pad[2][22] = PAD_GPIOC22_RIO_EP; io_pad[2][23] = PAD_GPIOC23_RIO_EP;
	io_pad[2][24] = PAD_GPIOC24_RIO_EP; io_pad[2][25] = PAD_GPIOC25_RIO_EP;
	io_pad[2][26] = PAD_GPIOC26_RIO_EP; io_pad[2][27] = PAD_GPIOC27_RIO_EP;
	io_pad[2][28] = PAD_GPIOC28_RIO_EP; io_pad[2][29] = PAD_GPIOC29_RIO_EP;
	io_pad[2][30] = PAD_GPIOC30_RIO_EP; io_pad[2][31] = PAD_GPIOC31_RIO_EP;

	/* GPIO D*/
	io_pad[3] [0] = PAD_GPIOD00_RIO_EP; io_pad[3] [1] = PAD_GPIOD01_RIO_EP;
	io_pad[3] [2] = PAD_GPIOD02_RIO_EP; io_pad[3] [3] = PAD_GPIOD03_RIO_EP;
	io_pad[3] [4] = PAD_GPIOD04_RIO_EP; io_pad[3] [5] = PAD_GPIOD05_RIO_EP;
	io_pad[3] [6] = PAD_GPIOD06_RIO_EP; io_pad[3] [7] = PAD_GPIOD07_RIO_EP;
	io_pad[3] [8] = PAD_GPIOD08_RIO_EP; io_pad[3] [9] = PAD_GPIOD09_RIO_EP;
	io_pad[3][10] = PAD_GPIOD10_RIO_EP; io_pad[3][11] = PAD_GPIOD11_RIO_EP;
	io_pad[3][12] = PAD_GPIOD12_RIO_EP; io_pad[3][13] = PAD_GPIOD13_RIO_EP;
	io_pad[3][14] = PAD_GPIOD14_RIO_EP; io_pad[3][15] = PAD_GPIOD15_RIO_EP;
	io_pad[3][16] = PAD_GPIOD16_RIO_EP; io_pad[3][17] = PAD_GPIOD17_RIO_EP;
	io_pad[3][18] = PAD_GPIOD18_RIO_EP; io_pad[3][19] = PAD_GPIOD19_RIO_EP;
	io_pad[3][20] = PAD_GPIOD20_RIO_EP; io_pad[3][21] = PAD_GPIOD21_RIO_EP;
	io_pad[3][22] = PAD_GPIOD22_RIO_EP; io_pad[3][23] = PAD_GPIOD23_RIO_EP;
	io_pad[3][24] = PAD_GPIOD24_RIO_EP;
}

void init_io_pad_val_cip(u32 io_pad[NUMBER_OF_GPIO_MODULE][32])
{
	/* GPIO A*/
	io_pad[0] [0] = PAD_GPIOA00_VAL_CIP; io_pad[0] [1] = PAD_GPIOA01_VAL_CIP;
	io_pad[0] [2] = PAD_GPIOA02_VAL_CIP; io_pad[0] [3] = PAD_GPIOA03_VAL_CIP;
	io_pad[0] [4] = PAD_GPIOA04_VAL_CIP; io_pad[0] [5] = PAD_GPIOA05_VAL_CIP;
	io_pad[0] [6] = PAD_GPIOA06_VAL_CIP; io_pad[0] [7] = PAD_GPIOA07_VAL_CIP;
	io_pad[0] [8] = PAD_GPIOA08_VAL_CIP; io_pad[0] [9] = PAD_GPIOA09_VAL_CIP;
	io_pad[0][10] = PAD_GPIOA10_VAL_CIP; io_pad[0][11] = PAD_GPIOA11_VAL_CIP;
	io_pad[0][12] = PAD_GPIOA12_VAL_CIP; io_pad[0][13] = PAD_GPIOA13_VAL_CIP;
	io_pad[0][14] = PAD_GPIOA14_VAL_CIP; io_pad[0][15] = PAD_GPIOA15_VAL_CIP;
	io_pad[0][16] = PAD_GPIOA16_VAL_CIP; io_pad[0][17] = PAD_GPIOA17_VAL_CIP;
	io_pad[0][18] = PAD_GPIOA18_VAL_CIP; io_pad[0][19] = PAD_GPIOA19_VAL_CIP;
	io_pad[0][20] = PAD_GPIOA20_VAL_CIP; io_pad[0][21] = PAD_GPIOA21_VAL_CIP;
	io_pad[0][22] = PAD_GPIOA22_VAL_CIP; io_pad[0][23] = PAD_GPIOA23_VAL_CIP;
	io_pad[0][24] = PAD_GPIOA24_VAL_CIP; io_pad[0][25] = PAD_GPIOA25_VAL_CIP;
	io_pad[0][26] = PAD_GPIOA26_VAL_CIP; io_pad[0][27] = PAD_GPIOA27_VAL_CIP;
	io_pad[0][28] = PAD_GPIOA28_VAL_CIP; io_pad[0][29] = PAD_GPIOA29_VAL_CIP;
	io_pad[0][30] = PAD_GPIOA30_VAL_CIP; io_pad[0][31] = PAD_GPIOA31_VAL_CIP;

	/* GPIO B*/
	io_pad[1] [0] = PAD_GPIOB00_VAL_CIP; io_pad[1] [1] = PAD_GPIOB01_VAL_CIP;
	io_pad[1] [2] = PAD_GPIOB02_VAL_CIP; io_pad[1] [3] = PAD_GPIOB03_VAL_CIP;
	io_pad[1] [4] = PAD_GPIOB04_VAL_CIP; io_pad[1] [5] = PAD_GPIOB05_VAL_CIP;
	io_pad[1] [6] = PAD_GPIOB06_VAL_CIP; io_pad[1] [7] = PAD_GPIOB07_VAL_CIP;
	io_pad[1] [8] = PAD_GPIOB08_VAL_CIP; io_pad[1] [9] = PAD_GPIOB09_VAL_CIP;
	io_pad[1][10] = PAD_GPIOB10_VAL_CIP; io_pad[1][11] = PAD_GPIOB11_VAL_CIP;
	io_pad[1][12] = PAD_GPIOB12_VAL_CIP; io_pad[1][13] = PAD_GPIOB13_VAL_CIP;
	io_pad[1][14] = PAD_GPIOB14_VAL_CIP; io_pad[1][15] = PAD_GPIOB15_VAL_CIP;
	io_pad[1][16] = PAD_GPIOB16_VAL_CIP; io_pad[1][17] = PAD_GPIOB17_VAL_CIP;
	io_pad[1][18] = PAD_GPIOB18_VAL_CIP; io_pad[1][19] = PAD_GPIOB19_VAL_CIP;
	io_pad[1][20] = PAD_GPIOB20_VAL_CIP; io_pad[1][21] = PAD_GPIOB21_VAL_CIP;
	io_pad[1][22] = PAD_GPIOB22_VAL_CIP; io_pad[1][23] = PAD_GPIOB23_VAL_CIP;
	io_pad[1][24] = PAD_GPIOB24_VAL_CIP; io_pad[1][25] = PAD_GPIOB25_VAL_CIP;
	io_pad[1][26] = PAD_GPIOB26_VAL_CIP; io_pad[1][27] = PAD_GPIOB27_VAL_CIP;
	io_pad[1][28] = PAD_GPIOB28_VAL_CIP; io_pad[1][29] = PAD_GPIOB29_VAL_CIP;
	io_pad[1][30] = PAD_GPIOB30_VAL_CIP; io_pad[1][31] = PAD_GPIOB31_VAL_CIP;

	/* GPIO C*/
	io_pad[2] [0] = PAD_GPIOC00_VAL_CIP; io_pad[2] [1] = PAD_GPIOC01_VAL_CIP;
	io_pad[2] [2] = PAD_GPIOC02_VAL_CIP; io_pad[2] [3] = PAD_GPIOC03_VAL_CIP;
	io_pad[2] [4] = PAD_GPIOC04_VAL_CIP; io_pad[2] [5] = PAD_GPIOC05_VAL_CIP;
	io_pad[2] [6] = PAD_GPIOC06_VAL_CIP; io_pad[2] [7] = PAD_GPIOC07_VAL_CIP;
	io_pad[2] [8] = PAD_GPIOC08_VAL_CIP; io_pad[2] [9] = PAD_GPIOC09_VAL_CIP;
	io_pad[2][10] = PAD_GPIOC10_VAL_CIP; io_pad[2][11] = PAD_GPIOC11_VAL_CIP;
	io_pad[2][12] = PAD_GPIOC12_VAL_CIP; io_pad[2][13] = PAD_GPIOC13_VAL_CIP;
	io_pad[2][14] = PAD_GPIOC14_VAL_CIP; io_pad[2][15] = PAD_GPIOC15_VAL_CIP;
	io_pad[2][16] = PAD_GPIOC16_VAL_CIP; io_pad[2][17] = PAD_GPIOC17_VAL_CIP;
	io_pad[2][18] = PAD_GPIOC18_VAL_CIP; io_pad[2][19] = PAD_GPIOC19_VAL_CIP;
	io_pad[2][20] = PAD_GPIOC20_VAL_CIP; io_pad[2][21] = PAD_GPIOC21_VAL_CIP;
	io_pad[2][22] = PAD_GPIOC22_VAL_CIP; io_pad[2][23] = PAD_GPIOC23_VAL_CIP;
	io_pad[2][24] = PAD_GPIOC24_VAL_CIP; io_pad[2][25] = PAD_GPIOC25_VAL_CIP;
	io_pad[2][26] = PAD_GPIOC26_VAL_CIP; io_pad[2][27] = PAD_GPIOC27_VAL_CIP;
	io_pad[2][28] = PAD_GPIOC28_VAL_CIP; io_pad[2][29] = PAD_GPIOC29_VAL_CIP;
	io_pad[2][30] = PAD_GPIOC30_VAL_CIP; io_pad[2][31] = PAD_GPIOC31_VAL_CIP;

	/* GPIO D*/
	io_pad[3] [0] = PAD_GPIOD00_VAL_CIP; io_pad[3] [1] = PAD_GPIOD01_VAL_CIP;
	io_pad[3] [2] = PAD_GPIOD02_VAL_CIP; io_pad[3] [3] = PAD_GPIOD03_VAL_CIP;
	io_pad[3] [4] = PAD_GPIOD04_VAL_CIP; io_pad[3] [5] = PAD_GPIOD05_VAL_CIP;
	io_pad[3] [6] = PAD_GPIOD06_VAL_CIP; io_pad[3] [7] = PAD_GPIOD07_VAL_CIP;
	io_pad[3] [8] = PAD_GPIOD08_VAL_CIP; io_pad[3] [9] = PAD_GPIOD09_VAL_CIP;
	io_pad[3][10] = PAD_GPIOD10_VAL_CIP; io_pad[3][11] = PAD_GPIOD11_VAL_CIP;
	io_pad[3][12] = PAD_GPIOD12_VAL_CIP; io_pad[3][13] = PAD_GPIOD13_VAL_CIP;
	io_pad[3][14] = PAD_GPIOD14_VAL_CIP; io_pad[3][15] = PAD_GPIOD15_VAL_CIP;
	io_pad[3][16] = PAD_GPIOD16_VAL_CIP; io_pad[3][17] = PAD_GPIOD17_VAL_CIP;
	io_pad[3][18] = PAD_GPIOD18_VAL_CIP; io_pad[3][19] = PAD_GPIOD19_VAL_CIP;
	io_pad[3][20] = PAD_GPIOD20_VAL_CIP; io_pad[3][21] = PAD_GPIOD21_VAL_CIP;
	io_pad[3][22] = PAD_GPIOD22_VAL_CIP; io_pad[3][23] = PAD_GPIOD23_VAL_CIP;
	io_pad[3][24] = PAD_GPIOD24_VAL_CIP;
}

void init_io_pad_vtk(u32 io_pad[NUMBER_OF_GPIO_MODULE][32])
{
	/* GPIO A*/
	io_pad[0] [0] = PAD_GPIOA00_VTK; io_pad[0] [1] = PAD_GPIOA01_VTK;
	io_pad[0] [2] = PAD_GPIOA02_VTK; io_pad[0] [3] = PAD_GPIOA03_VTK;
	io_pad[0] [4] = PAD_GPIOA04_VTK; io_pad[0] [5] = PAD_GPIOA05_VTK;
	io_pad[0] [6] = PAD_GPIOA06_VTK; io_pad[0] [7] = PAD_GPIOA07_VTK;
	io_pad[0] [8] = PAD_GPIOA08_VTK; io_pad[0] [9] = PAD_GPIOA09_VTK;
	io_pad[0][10] = PAD_GPIOA10_VTK; io_pad[0][11] = PAD_GPIOA11_VTK;
	io_pad[0][12] = PAD_GPIOA12_VTK; io_pad[0][13] = PAD_GPIOA13_VTK;
	io_pad[0][14] = PAD_GPIOA14_VTK; io_pad[0][15] = PAD_GPIOA15_VTK;
	io_pad[0][16] = PAD_GPIOA16_VTK; io_pad[0][17] = PAD_GPIOA17_VTK;
	io_pad[0][18] = PAD_GPIOA18_VTK; io_pad[0][19] = PAD_GPIOA19_VTK;
	io_pad[0][20] = PAD_GPIOA20_VTK; io_pad[0][21] = PAD_GPIOA21_VTK;
	io_pad[0][22] = PAD_GPIOA22_VTK; io_pad[0][23] = PAD_GPIOA23_VTK;
	io_pad[0][24] = PAD_GPIOA24_VTK; io_pad[0][25] = PAD_GPIOA25_VTK;
	io_pad[0][26] = PAD_GPIOA26_VTK; io_pad[0][27] = PAD_GPIOA27_VTK;
	io_pad[0][28] = PAD_GPIOA28_VTK; io_pad[0][29] = PAD_GPIOA29_VTK;
	io_pad[0][30] = PAD_GPIOA30_VTK; io_pad[0][31] = PAD_GPIOA31_VTK;

	/* GPIO B*/
	io_pad[1] [0] = PAD_GPIOB00_VTK; io_pad[1] [1] = PAD_GPIOB01_VTK;
	io_pad[1] [2] = PAD_GPIOB02_VTK; io_pad[1] [3] = PAD_GPIOB03_VTK;
	io_pad[1] [4] = PAD_GPIOB04_VTK; io_pad[1] [5] = PAD_GPIOB05_VTK;
	io_pad[1] [6] = PAD_GPIOB06_VTK; io_pad[1] [7] = PAD_GPIOB07_VTK;
	io_pad[1] [8] = PAD_GPIOB08_VTK; io_pad[1] [9] = PAD_GPIOB09_VTK;
	io_pad[1][10] = PAD_GPIOB10_VTK; io_pad[1][11] = PAD_GPIOB11_VTK;
	io_pad[1][12] = PAD_GPIOB12_VTK; io_pad[1][13] = PAD_GPIOB13_VTK;
	io_pad[1][14] = PAD_GPIOB14_VTK; io_pad[1][15] = PAD_GPIOB15_VTK;
	io_pad[1][16] = PAD_GPIOB16_VTK; io_pad[1][17] = PAD_GPIOB17_VTK;
	io_pad[1][18] = PAD_GPIOB18_VTK; io_pad[1][19] = PAD_GPIOB19_VTK;
	io_pad[1][20] = PAD_GPIOB20_VTK; io_pad[1][21] = PAD_GPIOB21_VTK;
	io_pad[1][22] = PAD_GPIOB22_VTK; io_pad[1][23] = PAD_GPIOB23_VTK;
	io_pad[1][24] = PAD_GPIOB24_VTK; io_pad[1][25] = PAD_GPIOB25_VTK;
	io_pad[1][26] = PAD_GPIOB26_VTK; io_pad[1][27] = PAD_GPIOB27_VTK;
	io_pad[1][28] = PAD_GPIOB28_VTK; io_pad[1][29] = PAD_GPIOB29_VTK;
	io_pad[1][30] = PAD_GPIOB30_VTK; io_pad[1][31] = PAD_GPIOB31_VTK;

	/* GPIO C*/
	io_pad[2] [0] = PAD_GPIOC00_VTK; io_pad[2] [1] = PAD_GPIOC01_VTK;
	io_pad[2] [2] = PAD_GPIOC02_VTK; io_pad[2] [3] = PAD_GPIOC03_VTK;
	io_pad[2] [4] = PAD_GPIOC04_VTK; io_pad[2] [5] = PAD_GPIOC05_VTK;
	io_pad[2] [6] = PAD_GPIOC06_VTK; io_pad[2] [7] = PAD_GPIOC07_VTK;
	io_pad[2] [8] = PAD_GPIOC08_VTK; io_pad[2] [9] = PAD_GPIOC09_VTK;
	io_pad[2][10] = PAD_GPIOC10_VTK; io_pad[2][11] = PAD_GPIOC11_VTK;
	io_pad[2][12] = PAD_GPIOC12_VTK; io_pad[2][13] = PAD_GPIOC13_VTK;
	io_pad[2][14] = PAD_GPIOC14_VTK; io_pad[2][15] = PAD_GPIOC15_VTK;
	io_pad[2][16] = PAD_GPIOC16_VTK; io_pad[2][17] = PAD_GPIOC17_VTK;
	io_pad[2][18] = PAD_GPIOC18_VTK; io_pad[2][19] = PAD_GPIOC19_VTK;
	io_pad[2][20] = PAD_GPIOC20_VTK; io_pad[2][21] = PAD_GPIOC21_VTK;
	io_pad[2][22] = PAD_GPIOC22_VTK; io_pad[2][23] = PAD_GPIOC23_VTK;
	io_pad[2][24] = PAD_GPIOC24_VTK; io_pad[2][25] = PAD_GPIOC25_VTK;
	io_pad[2][26] = PAD_GPIOC26_VTK; io_pad[2][27] = PAD_GPIOC27_VTK;
	io_pad[2][28] = PAD_GPIOC28_VTK; io_pad[2][29] = PAD_GPIOC29_VTK;
	io_pad[2][30] = PAD_GPIOC30_VTK; io_pad[2][31] = PAD_GPIOC31_VTK;

	/* GPIO D*/
	io_pad[3] [0] = PAD_GPIOD00_VTK; io_pad[3] [1] = PAD_GPIOD01_VTK;
	io_pad[3] [2] = PAD_GPIOD02_VTK; io_pad[3] [3] = PAD_GPIOD03_VTK;
	io_pad[3] [4] = PAD_GPIOD04_VTK; io_pad[3] [5] = PAD_GPIOD05_VTK;
	io_pad[3] [6] = PAD_GPIOD06_VTK; io_pad[3] [7] = PAD_GPIOD07_VTK;
	io_pad[3] [8] = PAD_GPIOD08_VTK; io_pad[3] [9] = PAD_GPIOD09_VTK;
	io_pad[3][10] = PAD_GPIOD10_VTK; io_pad[3][11] = PAD_GPIOD11_VTK;
	io_pad[3][12] = PAD_GPIOD12_VTK; io_pad[3][13] = PAD_GPIOD13_VTK;
	io_pad[3][14] = PAD_GPIOD14_VTK; io_pad[3][15] = PAD_GPIOD15_VTK;
	io_pad[3][16] = PAD_GPIOD16_VTK; io_pad[3][17] = PAD_GPIOD17_VTK;
	io_pad[3][18] = PAD_GPIOD18_VTK; io_pad[3][19] = PAD_GPIOD19_VTK;
	io_pad[3][20] = PAD_GPIOD20_VTK; io_pad[3][21] = PAD_GPIOD21_VTK;
	io_pad[3][22] = PAD_GPIOD22_VTK; io_pad[3][23] = PAD_GPIOD23_VTK;
	io_pad[3][24] = PAD_GPIOD24_VTK;
}


void set_gpio_pad(U32 io_grp, U32 io_bit, U32 io_pad_config)
{
	U32  io_mod, outval, detect, pullup, strength;

	/* print error and skip if not customized */
	if (NUMBER_OF_GPIO_MODULE < io_grp || \
	     31 < io_bit || \
	     io_pad_config == PAD_CUSTOM) {
		printf("%s:%s.%d ERROR: GPIO:%c[%d] pin:%d not set, skipping\n",
			__FILE__, __func__, __LINE__,
			'A' + io_grp, io_grp, io_bit);
		return;
	}

	/* skip if empty definition */
	if (io_pad_config == PAD_EMPTY)
		return;

	io_mod = PAD_GET_PADMODE(io_pad_config);

	switch (io_mod) {
		case PAD_MODE_IN:
			NX_GPIO_SetPadFunction (io_grp, io_bit, NX_GPIO_PADFUNC_GPIO);
			NX_GPIO_SetOutputEnable(io_grp, io_bit, CFALSE);
			break;
		case PAD_MODE_OUT:
			NX_GPIO_SetPadFunction (io_grp, io_bit, NX_GPIO_PADFUNC_GPIO);
			NX_GPIO_SetOutputEnable(io_grp, io_bit, CTRUE);
			break;
		case PAD_MODE_ALT1:
			NX_GPIO_SetPadFunction (io_grp, io_bit, NX_GPIO_PADFUNC_1);
			NX_GPIO_SetOutputEnable(io_grp, io_bit, CFALSE);
			break;
		case PAD_MODE_ALT2:
			NX_GPIO_SetPadFunction (io_grp, io_bit, NX_GPIO_PADFUNC_2);
			NX_GPIO_SetOutputEnable(io_grp, io_bit, CFALSE);
			break;
		case PAD_MODE_ALT3:
			NX_GPIO_SetPadFunction (io_grp, io_bit, NX_GPIO_PADFUNC_3);
			NX_GPIO_SetOutputEnable(io_grp, io_bit, CFALSE);
			break;
		case PAD_MODE_INT:
			detect = PAD_GET_DECTMODE(io_pad_config);
			NX_GPIO_SetPadFunction(io_grp, io_bit, NX_GPIO_PADFUNC_GPIO);
			NX_GPIO_SetOutputEnable(io_grp, io_bit, CFALSE);
			NX_GPIO_SetInterruptMode(io_grp, io_bit, detect);
			break;
		case PAD_MODE_SKIP:
		case PAD_NOTEXIST:
		default:
			return;
	}
	outval   = PAD_GET_OUTLEVEL(io_pad_config);
	pullup   = PAD_GET_PULLUP(io_pad_config);
	strength = PAD_GET_STRENGTH(io_pad_config);

	NX_GPIO_SetOutputValue (io_grp, io_bit, (outval ? CTRUE : CFALSE));
	NX_GPIO_SetPullUpEnable(io_grp, io_bit, (pullup ? CTRUE : CFALSE));
	NX_CLKPWR_SetGPIOPadStrength(io_grp, io_bit, strength);
}


/*
 * init_gpio_pad() --   initialize LF2000 gpio pads as needed
 *			break into two parts.  First initialize
 *			common GPIO ports, read NOR board rev,
 *			then initialize remaining board specific
 *			ports
 */
void init_gpio_pad(void)
{
	int  io_grp, io_bit;

	U32 io_pad[NUMBER_OF_GPIO_MODULE][32] = {
		/* GPIO group A */
		{
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_GPIOA08_, PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		},
		/* GPIO group B */
		{
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		},
		/* GPIO group C */
		{
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		},
		/* GPIO group D */
		{
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_GPIOD09_, PAD_GPIOD10_, PAD_GPIOD11_,
		PAD_GPIOD12_, PAD_GPIOD13_, PAD_GPIOD14_, PAD_GPIOD15_,
		PAD_GPIOD16_, PAD_GPIOD17_, PAD_GPIOD18_, PAD_GPIOD19_,
		PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  , PAD_CUSTOM  ,
		PAD_CUSTOM  , PAD_EMPTY   , PAD_EMPTY   , PAD_EMPTY   ,
		PAD_EMPTY   , PAD_EMPTY   , PAD_EMPTY   , PAD_EMPTY   ,
		}
	};

	/* Determine where to read Board ID from based on Boot, if possible.
	 * Setup GPIO D[19:9] to be ALT1 so Parallel NOR could be read for board_id
	 */

	for (io_grp = 0; io_grp < NUMBER_OF_GPIO_MODULE; io_grp++) {
		for (io_bit = 0; io_bit < 32; io_bit++) {
			if (io_pad[io_grp][io_bit] != PAD_EMPTY &&
			    io_pad[io_grp][io_bit] != PAD_CUSTOM) {
				set_gpio_pad(io_grp, io_bit,
				             io_pad[io_grp][io_bit]);
			}
		}
	}

	printf("Initializing GPIOs\n");

	/* customize the GPIO definitions that differ between boards */
	switch(get_board_rev()) {

	case LF2000_BOARD_LUCY:
	case LF2000_BOARD_LUCY_PP:
		init_io_pad_l2k(io_pad);
		break;

	case LF2000_BOARD_LUCY_CIP:
		init_io_pad_lucy_cip(io_pad);
		break;

	case LF2000_BOARD_VALENCIA:
	case LF2000_BOARD_VALENCIA_EP:
	case LF2000_BOARD_VALENCIA_EP_8:
	case LF2000_BOARD_VALENCIA_FEP:
	case LF2000_BOARD_VALENCIA_FEP_8:
	case LF2000_BOARD_VALENCIA_EP_800_480:
	case LF2000_BOARD_VALENCIA_EP_800_480_8:
	case LF2000_BOARD_VALENCIA_FEP_800_480:
	case LF2000_BOARD_VALENCIA_FEP_800_480_8:
	case LF2000_BOARD_VALENCIA_KND_800_480:
	case LF2000_BOARD_VALENCIA_KND_800_480_8:
	case LF2000_BOARD_VALENCIA_KND_1024_600:
	case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		init_io_pad_m2k(io_pad);
		break;

	case LF2000_BOARD_RIO:
	case LF2000_BOARD_RIO_KND_800_480:
		init_io_pad_rio(io_pad);
		break;

	case LF2000_BOARD_RIO_BETA_1024_600:
	case LF2000_BOARD_RIO_BETA_800_480:
	case LF2000_BOARD_RIO_BETA_1024_600_700_400:
	case LF2000_BOARD_RIO_BETA_800_480_700_400:
	case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		init_io_pad_rio_beta(io_pad);
		break;

	case LF2000_BOARD_RIO_EP_550_275:
	case LF2000_BOARD_RIO_EP_666_333:
	case LF2000_BOARD_RIO_EP_800_333:
	case LF2000_BOARD_RIO_EP_700_400:
	case LF2000_BOARD_RIO_EP_800_400:
	case LF2000_BOARD_RIO_FEP_800_327P67:
	case LF2000_BOARD_RIO_FEP_800_327P666:
		init_io_pad_rio_ep(io_pad);
		break;

	case LF2000_BOARD_VALENCIA_CIP:
		init_io_pad_val_cip(io_pad);
		break;

	case LF2000_BOARD_VTK:
	case LF2000_BOARD_UNKNOWN:
		init_io_pad_vtk(io_pad);
		break;
	}

	/* setup GPIO pads again, this time include custom ones */
	for (io_grp = 0; io_grp < NUMBER_OF_GPIO_MODULE; io_grp++) {
		for (io_bit = 0; io_bit < 32; io_bit++) {
			set_gpio_pad(io_grp, io_bit, io_pad[io_grp][io_bit]);
		}
	}
}

static void init_alive_pad(void)
{
	int  io_bit;
	U32  io_mod, outval, detect, pullup, io_num;

	const U32 alv_pad[] = {
		PAD_GPIOALV0_, PAD_GPIOALV1_,  PAD_GPIOALV2_, PAD_GPIOALV3_,
		PAD_GPIOALV4_, PAD_GPIOALV5_,  PAD_GPIOALV6_, PAD_GPIOALV7_
			};

	io_num = sizeof(alv_pad)/sizeof(alv_pad[0]);

	/* Alive pad function */
	for (io_bit = 0; io_num > io_bit; io_bit++) {

		io_mod = PAD_GET_PADMODE(alv_pad[io_bit]);

		switch (io_mod) {
		case PAD_MODE_IN   :
		case PAD_MODE_DECT :
			NX_ALIVE_SetOutputEnable(io_bit, CFALSE);
			break;
		case PAD_MODE_OUT  :
			NX_ALIVE_SetOutputEnable(io_bit, CTRUE);
			break;
		default :
			DBGOUT("\n Unknown GPIO ALIVE Mode(0x%x)\n", io_mod);
			continue;
		}
		outval = PAD_GET_PADMODE(alv_pad[io_bit]);
		pullup = PAD_GET_PULLUP(alv_pad[io_bit]);
		detect = PAD_GET_DECTMODE(alv_pad[io_bit]);

		NX_ALIVE_SetOutputValue (io_bit, (outval ? CTRUE : CFALSE));
		NX_ALIVE_SetPullUpEnable(io_bit, (pullup ? CTRUE : CFALSE));

		// For wakeup source
		NX_ALIVE_SetDetectMode(detect, io_bit, (io_mod == PAD_MODE_DECT ? CTRUE : CFALSE));
		NX_ALIVE_SetDetectEnable(io_bit, (io_mod == PAD_MODE_DECT ? CTRUE : CFALSE));
	}
}

static void init_bus_pad(void)
{
	int  io_bit;
	U32  strength, buspad, busnum;

	const U32 bus_pad[] = {
		PAD_BUS_STATIC_CNTL_, PAD_BUS_STATIC_ADDR_, PAD_BUS_STATIC_DATA_,
		PAD_BUS_VSYNC_,       PAD_BUS_HSYNC_,       PAD_BUS_DE_
			};

	busnum = sizeof(bus_pad)/sizeof(bus_pad[0]) -1;

	/* BUS pad function */
	for (io_bit = 0; busnum > io_bit; io_bit++) {
		buspad   = PAD_GET_BUS(bus_pad[io_bit]);
		strength = PAD_GET_STRENGTH(bus_pad[io_bit]);
		NX_CLKPWR_SetBusPadStrength(buspad, strength);
	}
}

/*------------------------------------------------------------------------------
 * intialize nexell soc display controller register.
 */

struct pwm_pad{
	unsigned int group;
	unsigned int bit;
	unsigned int func;
};

/* port definitions for PWM Backlight (if LFP100 not used) */
static struct pwm_pad pwm_ch_l2k[] = {
	{ PAD_GET_GRP(PAD_GPIO_A), 30, NX_GPIO_PADFUNC_1 	},
	{ PAD_GET_GRP(PAD_GPIO_C), 11, NX_GPIO_PADFUNC_GPIO }	/* Backlight Enable */
};
static struct pwm_pad pwm_ch_m2k_vtk[] = {
	{ PAD_GET_GRP(PAD_GPIO_A), 30, NX_GPIO_PADFUNC_1 	},
	{ PAD_GET_GRP(PAD_GPIO_A), 31, NX_GPIO_PADFUNC_GPIO }	/* Backlight Enable */
};

static void init_display(int dev)
{
	volatile int delay = 0;

	NX_DPC_DITHER RDither, GDither, BDither;

	U32  DPCFormat = CFG_DISP_PRI_OUT_FORMAT;
  	bool bEmbSync  = CFALSE;
    	U32  XResol    = get_lf2000_board_info()->cfg_disp_pri_resol_width;
    	U32  YResol    = get_lf2000_board_info()->cfg_disp_pri_resol_height;
	U32  LockSize  = CFG_DISP_PRI_MLC_LOCKSIZE;

	U32  PFrameBase= (CFG_MEM_PHY_BLOCK_BASE ? CFG_MEM_PHY_BLOCK_BASE : CFG_MEM_PHY_LINEAR_BASE);
	U32  VFrameBase= (U32)IO_ADDRESS(PFrameBase);

	if ((dev == 0 && CFALSE == CFG_DISP_PRI_BOOT_ENB))
		return;

	//--------------------------------------------------------------------------
	// Turn off LCD; if LFP100 then backlight is already off
	//--------------------------------------------------------------------------

		switch(get_board_rev()) {
		case LF2000_BOARD_LUCY:
		case LF2000_BOARD_LUCY_CIP:
		case LF2000_BOARD_LUCY_PP:
		case LF2000_BOARD_VALENCIA:
		case LF2000_BOARD_VALENCIA_EP:
		case LF2000_BOARD_VALENCIA_EP_8:
		case LF2000_BOARD_VALENCIA_FEP:
		case LF2000_BOARD_VALENCIA_FEP_8:
		case LF2000_BOARD_VALENCIA_CIP:
		case LF2000_BOARD_VALENCIA_EP_800_480:
		case LF2000_BOARD_VALENCIA_EP_800_480_8:
		case LF2000_BOARD_VALENCIA_FEP_800_480:
		case LF2000_BOARD_VALENCIA_FEP_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_800_480:
		case LF2000_BOARD_VALENCIA_KND_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_1024_600:
		case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		case LF2000_BOARD_RIO:
		case LF2000_BOARD_RIO_KND_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600:
		case LF2000_BOARD_RIO_BETA_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600_700_400:
		case LF2000_BOARD_RIO_BETA_800_480_700_400:
		case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		case LF2000_BOARD_RIO_EP_550_275:
		case LF2000_BOARD_RIO_EP_666_333:
		case LF2000_BOARD_RIO_EP_800_333:
		case LF2000_BOARD_RIO_EP_700_400:
		case LF2000_BOARD_RIO_EP_800_400:
		case LF2000_BOARD_RIO_FEP_800_327P67:
		case LF2000_BOARD_RIO_FEP_800_327P666:
			break;
		case LF2000_BOARD_VTK:
		case LF2000_BOARD_UNKNOWN:
		default:
#  if	(CFG_LCD_PRI_LCD_ON)
			if (dev == 0) {
#    ifdef CFG_PIO_LCD_PCI_ENB
				NX_GPIO_SetOutputValue(
					PAD_GET_GRP(CFG_PIO_LCD_PCI_ENB),
					PAD_GET_BIT(CFG_PIO_LCD_PCI_ENB),
					CFALSE);
				for (delay=0; 100 > delay; delay++) { ; }
#    endif
			}
#  endif
			break;
		}

	//--------------------------------------------------------------------------
	// DPC Clock Disable
	//--------------------------------------------------------------------------
	NX_DPC_SetClockPClkMode(dev, NX_PCLKMODE_DYNAMIC);
	NX_DPC_SetDPCEnable(dev, CFALSE);
	NX_DPC_SetClockDivisorEnable(dev, CFALSE);

	//--------------------------------------------------------------------------
	// Horizontla Up scale
	// Note: Only seconary DPC can scale up of horizontal width.
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// Set DPC
	//--------------------------------------------------------------------------
	// Dithering
    if (((U32)NX_DPC_FORMAT_RGB555   == DPCFormat) ||
		((U32)NX_DPC_FORMAT_MRGB555A == DPCFormat) ||
		((U32)NX_DPC_FORMAT_MRGB555B == DPCFormat))
	{
		RDither = GDither = BDither = NX_DPC_DITHER_5BIT;
	}
	else if (((U32)NX_DPC_FORMAT_RGB565  == DPCFormat) ||
			 ((U32)NX_DPC_FORMAT_MRGB565 == DPCFormat))
	{
		RDither = BDither = NX_DPC_DITHER_5BIT;
		GDither           = NX_DPC_DITHER_6BIT;
	}
	else if (((U32)NX_DPC_FORMAT_RGB666  == DPCFormat) ||
			 ((U32)NX_DPC_FORMAT_MRGB666 == DPCFormat))
	{
		RDither = GDither = BDither = NX_DPC_DITHER_6BIT;
	}
	else
	{
		RDither = GDither = BDither = NX_DPC_DITHER_BYPASS;
	}

	//--------------------------------------------------------------------------
	// VCLK2 : CLKGEN0
	NX_DPC_SetClockSource  (dev, 0, CFG_DISP_PRI_CLKGEN0_SOURCE);	// CLKSRCSEL
	NX_DPC_SetClockDivisor (dev, 0, get_lf2000_board_info()->cfg_disp_pri_clkgen0_div);		// CLKDIV
	NX_DPC_SetClockOutDelay(dev, 0, CFG_DISP_PRI_CLKGEN0_DELAY); 	// OUTCLKDELAY

	// VCLK : CLKGEN1
	NX_DPC_SetClockSource  (dev, 1, CFG_DISP_PRI_CLKGEN1_SOURCE);	// CLKSRCSEL  : CLKGEN0's out
	NX_DPC_SetClockDivisor (dev, 1, CFG_DISP_PRI_CLKGEN1_DIV);		// CLKDIV
	NX_DPC_SetClockOutDelay(dev, 1, CFG_DISP_PRI_CLKGEN1_DELAY); 	// OUTCLKDELAY

	//--------------------------------------------------------------------------
	NX_DPC_SetMode( dev,
					  DPCFormat,						// FORMAT
				 	  CFG_DISP_PRI_OUT_INTERLACE,		// SCANMODE
	             	  CFG_DISP_PRI_OUT_POL_INVERT,   	// POLFIELD
	             	  CFG_DISP_PRI_OUT_RGB, 			// RGBMODE
	             	  CFALSE,       					// SWAPRB
	             	  CFG_DISP_PRI_OUT_YCORDER,			// YCORDER
	             	  bEmbSync,							// YCCLIP
	             	  bEmbSync,  						// Embedded sync, Note> have to set to CTRUE for ITU-R BT.656
	             	  CFG_DISP_PRI_PADCLKSEL,			// PADCLKSEL
	             	  get_lf2000_board_info()->cfg_disp_pri_out_clk_invert,		// PADCLKINV
					  CFG_DISP_PRI_OUT_DUAL_VIEW);

	NX_DPC_SetHSync(dev,
					  XResol,
					  CFG_DISP_PRI_HSYNC_SYNC_WIDTH,
					  get_lf2000_board_info()->cfg_disp_pri_hsync_front_porch,
					  get_lf2000_board_info()->cfg_disp_pri_hsync_back_porch,
					  CFG_DISP_PRI_HSYNC_ACTIVE_HIGH);

	NX_DPC_SetVSync(dev,
					  YResol,
					  CFG_DISP_PRI_VSYNC_SYNC_WIDTH,
					  get_lf2000_board_info()->cfg_disp_pri_vsync_front_porch,
					  get_lf2000_board_info()->cfg_disp_pri_vsync_back_porch,
					  CFG_DISP_PRI_VSYNC_ACTIVE_HIGH,
					  CFG_DISP_PRI_EVSYNC_ACTIVE_HEIGHT,
					  CFG_DISP_PRI_EVSYNC_SYNC_WIDTH,
					  CFG_DISP_PRI_EVSYNC_FRONT_PORCH,
					  CFG_DISP_PRI_EVSYNC_BACK_PORCH);

	NX_DPC_SetVSyncOffset(dev,
						CFG_DISP_PRI_VSYNC_START_OFFSET,
						CFG_DISP_PRI_VSYNC_END_OFFSET,
						CFG_DISP_PRI_EVSYNC_START_OFFSET,
						CFG_DISP_PRI_EVSYNC_END_OFFSET);

	NX_DPC_SetDelay(dev,
					  CFG_DISP_PRI_SYNC_DELAY_RGB_PVD,		// DELAYRGB
	             	  CFG_DISP_PRI_SYNC_DELAY_HS_CP1,		// DELAYHS_CP1
	             	  CFG_DISP_PRI_SYNC_DELAY_VS_FRAM,		// DELAYVS_FRAM
	             	  CFG_DISP_PRI_SYNC_DELAY_DE_CP2);		// DELAYDE_CP2

   	NX_DPC_SetDither(dev, RDither, GDither, BDither);

	NX_DPC_SetDPCEnable		 (dev, CTRUE);
	NX_DPC_SetClockDivisorEnable(dev, CTRUE);			// CLKENB : Provides internal operating clock.

	NX_DPC_SetMode( dev,
					  DPCFormat,						// FORMAT
					  CFG_DISP_PRI_OUT_INTERLACE,		// SCANMODE
	             	  CFG_DISP_PRI_OUT_POL_INVERT,   	// POLFIELD
	             	  CFG_DISP_PRI_OUT_RGB, 			// RGBMODE
	             	  CFALSE,       					// SWAPRB
	             	  CFG_DISP_PRI_OUT_YCORDER,			// YCORDER
	             	  bEmbSync,							// YCCLIP
	             	  bEmbSync,  						// Embedded sync, Note> have to set to CTRUE for ITU-R BT.656
	             	  CFG_DISP_PRI_PADCLKSEL,			// PADCLKSEL
	             	  get_lf2000_board_info()->cfg_disp_pri_out_clk_invert, // PADCLKINV
					  CFG_DISP_PRI_OUT_DUAL_VIEW);

	for (delay=0; 100 > delay; delay++) { ; }

	#ifdef DEBUG_REG
	dbg_reg_dpc(dev);
	#endif

	//--------------------------------------------------------------------------
	// Clear frame buffer to black, assume 0 is black in every format
	//--------------------------------------------------------------------------

	//memset((void *)PFrameBase,
		//0xFF,
		//get_lf2000_board_info()->cfg_disp_pri_resol_width * get_lf2000_board_info()->cfg_disp_pri_resol_height * CFG_DISP_SCREEN_PIXEL_BYTE);

	//--------------------------------------------------------------------------
	// LCD On
	//--------------------------------------------------------------------------
#if	(CFG_LCD_PRI_LCD_ON)
#ifdef CFG_PIO_LCD_PCI_ENB
	if (dev == 0)
		NX_GPIO_SetOutputValue(PAD_GET_GRP(CFG_PIO_LCD_PCI_ENB), PAD_GET_BIT(CFG_PIO_LCD_PCI_ENB), CTRUE);
#endif
#endif

	//--------------------------------------------------------------------------
	// LCD Backlight On, if LFP100 not used
	//--------------------------------------------------------------------------
		switch(get_board_rev()) {
		case LF2000_BOARD_LUCY:
		case LF2000_BOARD_LUCY_CIP:
		case LF2000_BOARD_LUCY_PP:
		case LF2000_BOARD_VALENCIA:
		case LF2000_BOARD_VALENCIA_EP:
		case LF2000_BOARD_VALENCIA_EP_8:
		case LF2000_BOARD_VALENCIA_FEP:
		case LF2000_BOARD_VALENCIA_FEP_8:
		case LF2000_BOARD_VALENCIA_CIP:
		case LF2000_BOARD_VALENCIA_EP_800_480:
		case LF2000_BOARD_VALENCIA_EP_800_480_8:
		case LF2000_BOARD_VALENCIA_FEP_800_480:
		case LF2000_BOARD_VALENCIA_FEP_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_800_480:
		case LF2000_BOARD_VALENCIA_KND_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_1024_600:
		case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		case LF2000_BOARD_RIO:
		case LF2000_BOARD_RIO_KND_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600:
		case LF2000_BOARD_RIO_BETA_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600_700_400:
		case LF2000_BOARD_RIO_BETA_800_480_700_400:
		case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		case LF2000_BOARD_RIO_EP_550_275:
		case LF2000_BOARD_RIO_EP_666_333:
		case LF2000_BOARD_RIO_EP_800_333:
		case LF2000_BOARD_RIO_EP_700_400:
		case LF2000_BOARD_RIO_EP_800_400:
		case LF2000_BOARD_RIO_FEP_800_327P67:
		case LF2000_BOARD_RIO_FEP_800_327P666:
			break;

		case LF2000_BOARD_VTK:
		case LF2000_BOARD_UNKNOWN:
		default:
			if (dev == 0 && CFG_LCD_PRI_BLU_ON == CTRUE) {
				NX_PWM_SetClockPClkMode	 (NX_PCLKMODE_ALWAYS);
				NX_PWM_SetClockDivisorEnable(CFALSE);
				NX_PWM_SetClockSource  (0, CFG_PWM_CLK_SOURCE);
				NX_PWM_SetClockDivisor (0, CFG_PWM_CLK_DIV);
				NX_PWM_SetPreScale (CFG_LCD_PRI_PWM_CH,
	CFG_PWM_CLK_FREQ/CFG_PWM_CLK_DIV/CFG_PWM_PERIOD/CFG_LCD_PRI_PWM_FREQ);
				NX_PWM_SetPeriod(CFG_LCD_PRI_PWM_CH,
					CFG_PWM_PERIOD);
				NX_PWM_SetDutyCycle(CFG_LCD_PRI_PWM_CH,
					CFG_LCD_PRI_PWM_DUTYCYCLE);
				NX_PWM_SetClockDivisorEnable(CTRUE);
				NX_GPIO_SetPadFunction(
					pwm_ch_m2k_vtk[CFG_LCD_PRI_PWM_CH].group,
					pwm_ch_m2k_vtk[CFG_LCD_PRI_PWM_CH].bit,
					(NX_GPIO_PADFUNC)pwm_ch_m2k_vtk[CFG_LCD_PRI_PWM_CH].func
				);
			break;
		}

	}

	//--------------------------------------------------------------------------
	// Set DPC
	//--------------------------------------------------------------------------
	// RGB TOP Layer
	//
	NX_MLC_SetClockPClkMode	(dev, NX_PCLKMODE_DYNAMIC);
	NX_MLC_SetClockBClkMode	(dev, NX_BCLKMODE_DYNAMIC);

	NX_MLC_SetLayerPriority	(dev, CFG_DISP_LAYER_VIDEO_PRIORITY);
	NX_MLC_SetBackground   	(dev, CFG_DISP_BACK_GROUND_COLOR);
	NX_MLC_SetFieldEnable		(dev, CFG_DISP_PRI_MLC_INTERLACE);
	NX_MLC_SetScreenSize		(dev, XResol, YResol);

	NX_MLC_SetRGBLayerGamaTablePowerMode 	(dev, CFALSE, CFALSE, CFALSE);
	NX_MLC_SetRGBLayerGamaTableSleepMode 	(dev, CTRUE, CTRUE, CTRUE);
	NX_MLC_SetRGBLayerGammaEnable 			(dev, CFALSE);
	NX_MLC_SetDitherEnableWhenUsingGamma 	(dev, CFALSE);
	NX_MLC_SetGammaPriority 				(dev, CFALSE);

    NX_MLC_SetTopPowerMode		(dev, CTRUE);
    NX_MLC_SetTopSleepMode		(dev, CFALSE);
	NX_MLC_SetMLCEnable		(dev, CTRUE);

	NX_MLC_SetTopDirtyFlag		(dev);

	// RGB Layer
	//
	if (dev == 0 && CFALSE == CFG_DISP_PRI_BOOT_LOGO)
		return;

	NX_MLC_SetLockSize			(dev, CFG_DISP_LAYER_SCREEN, LockSize);

	NX_MLC_SetAlphaBlending 	(dev, CFG_DISP_LAYER_SCREEN, CFALSE, 15);
	NX_MLC_SetTransparency  	(dev, CFG_DISP_LAYER_SCREEN, CFALSE, 0);
	NX_MLC_SetColorInversion	(dev, CFG_DISP_LAYER_SCREEN, CFALSE, 0);

	NX_MLC_SetRGBLayerInvalidPosition(dev, CFG_DISP_LAYER_SCREEN, 0, 0, 0, 0, 0, CFALSE);
	NX_MLC_SetRGBLayerInvalidPosition(dev, CFG_DISP_LAYER_SCREEN, 1, 0, 0, 0, 0, CFALSE);

	NX_MLC_SetFormatRGB 		(dev, CFG_DISP_LAYER_SCREEN, CFG_DISP_SCREEN_RGB_FORMAT);
	NX_MLC_SetPosition 		(dev, CFG_DISP_LAYER_SCREEN, 0, 0, XResol-1, YResol-1);

	if (dev == 1) {
		PFrameBase += (get_lf2000_board_info()->cfg_disp_pri_resol_width * get_lf2000_board_info()->cfg_disp_pri_resol_height*CFG_DISP_SCREEN_PIXEL_BYTE);
		VFrameBase += (get_lf2000_board_info()->cfg_disp_pri_resol_width*get_lf2000_board_info()->cfg_disp_pri_resol_height*CFG_DISP_SCREEN_PIXEL_BYTE);
	}

	NX_MLC_SetRGBLayerStride(dev, CFG_DISP_LAYER_SCREEN, CFG_DISP_SCREEN_PIXEL_BYTE, XResol*CFG_DISP_SCREEN_PIXEL_BYTE);
	NX_MLC_SetRGBLayerAddress(dev, CFG_DISP_LAYER_SCREEN, PFrameBase);

	NX_MLC_SetLayerEnable(dev, CFG_DISP_LAYER_SCREEN, CTRUE);
	//NX_MLC_SetDirtyFlag(dev, CFG_DISP_LAYER_SCREEN);

	#ifdef DEBUG_REG
	dbg_reg_mlc(dev);
	#endif
}

int detect_usb_charger(void)
{
	int detect;
	int wall_usb_charger = 0;
	u32 system_rev = get_board_rev();
	
	switch(system_rev) {
		case LF2000_BOARD_RIO:
		case LF2000_BOARD_RIO_KND_800_480:
			detect = NX_GPIO_GetInputValue(USB_CHG_DETECT_RIO_ALPHA >> 5,
					  USB_CHG_DETECT_RIO_ALPHA & 0x1F);
			if(detect)
			{
				//printf("%s.%d Wall charger detected\n", __FUNCTION__, __LINE__);
				wall_usb_charger = 1;
			}
			else
			{
				//printf("%s.%d No wall charger detected\n", __FUNCTION__, __LINE__);
			}
			break;
		case LF2000_BOARD_RIO_BETA_1024_600:
		case LF2000_BOARD_RIO_BETA_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600_700_400:
		case LF2000_BOARD_RIO_BETA_800_480_700_400:
		case LF2000_BOARD_RIO_BETA_1024_600_550_275:
			detect = NX_GPIO_GetInputValue(USB_CHG_DETECT_RIO_BETA >> 5,
					  USB_CHG_DETECT_RIO_BETA & 0x1F);
			if(detect)
			{
				//printf("%s.%d Wall charger detected\n", __FUNCTION__, __LINE__);
				wall_usb_charger = 1;
			}
			else
			{
				//printf("%s.%d No wall charger detected\n", __FUNCTION__, __LINE__);
			}
			break;
		case LF2000_BOARD_RIO_EP_550_275:
		case LF2000_BOARD_RIO_EP_666_333:
		case LF2000_BOARD_RIO_EP_800_333:
		case LF2000_BOARD_RIO_EP_700_400:
		case LF2000_BOARD_RIO_EP_800_400:
		case LF2000_BOARD_RIO_FEP_800_327P67:
		case LF2000_BOARD_RIO_FEP_800_327P666:
			detect = NX_GPIO_GetInputValue(USB_CHG_DETECT_RIO_EP >> 5,
					  USB_CHG_DETECT_RIO_EP & 0x1F);
			if(!detect)
			{
				//printf("%s.%d Wall charger detected\n", __FUNCTION__, __LINE__);
				wall_usb_charger = 1;
			}
			else
			{
				//printf("%s.%d No wall charger detected\n", __FUNCTION__, __LINE__);
			}
			break;
		case LF2000_BOARD_LUCY:
		case LF2000_BOARD_LUCY_PP:
		case LF2000_BOARD_LUCY_CIP:
		case LF2000_BOARD_VALENCIA:
		case LF2000_BOARD_VALENCIA_EP:
		case LF2000_BOARD_VALENCIA_EP_8:
		case LF2000_BOARD_VALENCIA_FEP:
		case LF2000_BOARD_VALENCIA_FEP_8:
		case LF2000_BOARD_VALENCIA_EP_800_480:
		case LF2000_BOARD_VALENCIA_EP_800_480_8:
		case LF2000_BOARD_VALENCIA_FEP_800_480:
		case LF2000_BOARD_VALENCIA_FEP_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_800_480:
		case LF2000_BOARD_VALENCIA_KND_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_1024_600:
		case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		case LF2000_BOARD_VALENCIA_CIP:
		case LF2000_BOARD_VTK:
			printf("%s: Error: Charging not supported %0x%X \n", 
				__func__, system_rev);
			wall_usb_charger = -1;
			break;
 		default:
			printf("%s: Error: Invalid Board Revision %0x%X \n", 
				__func__, system_rev);
			wall_usb_charger = -1;
			break;
	}
	
	return wall_usb_charger;
}

#ifdef DEBUG_REG
static void
dbg_reg_dpc(int dev)
{
	U32 DPCBASE = IO_ADDRESS(NX_DPC_GetPhysicalAddress(dev));

	DBGOUT(" DPCHTOTAL				= 0x%04x\n", *(U16*)(DPCBASE + 0x07C));
	DBGOUT(" DPCHSWIDTH				= 0x%04x\n", *(U16*)(DPCBASE + 0x07E));
	DBGOUT(" DPCHASTART				= 0x%04x\n", *(U16*)(DPCBASE + 0x080));
	DBGOUT(" DPCHAEND				= 0x%04x\n", *(U16*)(DPCBASE + 0x082));
	DBGOUT(" DPCVTOTAL				= 0x%04x\n", *(U16*)(DPCBASE + 0x084));
	DBGOUT(" DPCVSWIDTH				= 0x%04x\n", *(U16*)(DPCBASE + 0x086));
	DBGOUT(" DPCVASTART				= 0x%04x\n", *(U16*)(DPCBASE + 0x088));
	DBGOUT(" DPCVAEND				= 0x%04x\n", *(U16*)(DPCBASE + 0x08A));
	DBGOUT(" DPCCTRL1				= 0x%04x\n", *(U16*)(DPCBASE + 0x08E));
	DBGOUT(" DPCEVTOTAL				= 0x%04x\n", *(U16*)(DPCBASE + 0x090));
	DBGOUT(" DPCEVSWIDTH			= 0x%04x\n", *(U16*)(DPCBASE + 0x092));
	DBGOUT(" DPCEVASTART			= 0x%04x\n", *(U16*)(DPCBASE + 0x094));
	DBGOUT(" DPCEVAEND				= 0x%04x\n", *(U16*)(DPCBASE + 0x096));
	DBGOUT(" DPCCTRL2				= 0x%04x\n", *(U16*)(DPCBASE + 0x098));
	DBGOUT(" DPCVSEOFFSET			= 0x%04x\n", *(U16*)(DPCBASE + 0x09A));
	DBGOUT(" DPCVSSOFFSET			= 0x%04x\n", *(U16*)(DPCBASE + 0x09C));
	DBGOUT(" DPCEVSEOFFSET			= 0x%04x\n", *(U16*)(DPCBASE + 0x09E));
	DBGOUT(" DPCEVSSOFFSET			= 0x%04x\n", *(U16*)(DPCBASE + 0x0A0));
	DBGOUT(" DPCDELAY0				= 0x%04x\n", *(U16*)(DPCBASE + 0x0A2));
	DBGOUT(" DPCUPSCALECON0 		= 0x%04x\n", *(U16*)(DPCBASE + 0x0A4));
	DBGOUT(" DPCUPSCALECON1 		= 0x%04x\n", *(U16*)(DPCBASE + 0x0A6));
	DBGOUT(" DPCUPSCALECON2 		= 0x%04x\n", *(U16*)(DPCBASE + 0x0A8));
	DBGOUT(" DPCCLKENB       		= 0x%08x\n", *(U32*)(DPCBASE + 0x1C0));
	DBGOUT(" DPCCLKGEN[0][0] 		= 0x%08x\n", *(U32*)(DPCBASE + 0x1C4));
	DBGOUT(" DPCCLKGEN[0][1] 		= 0x%08x\n", *(U32*)(DPCBASE + 0x1C8));
	DBGOUT(" DPCCLKGEN[1][0] 		= 0x%08x\n", *(U32*)(DPCBASE + 0x1CC));
	DBGOUT(" DPCCLKGEN[1][1] 		= 0x%08x\n", *(U32*)(DPCBASE + 0x1D0));
	DBGOUT(" DPCCTRL0				= 0x%04x\n", *(U16*)(DPCBASE + 0x08C));
}

static void dbg_reg_mlc(int dev)
{
	struct NX_MLC_RegisterSet *MLC =
		(struct NX_MLC_RegisterSet*)IO_ADDRESS(NX_MLC_GetPhysicalAddress(dev));

	DBGOUT("MLCCONTROLT			    	=0x%08x\n", MLC->MLCCONTROLT);
	DBGOUT("MLCSCREENSIZE		    	=0x%08x\n", MLC->MLCSCREENSIZE);
	DBGOUT("MLCBGCOLOR		      		=0x%08x\n", MLC->MLCBGCOLOR);

	// MLCRGBLAYER[0]
	DBGOUT("RGB_0 MLCLEFTRIGHT	        =0x%08x\n", MLC->MLCRGBLAYER[0].MLCLEFTRIGHT);
	DBGOUT("RGB_0 MLCTOPBOTTOM 	        =0x%08x\n", MLC->MLCRGBLAYER[0].MLCTOPBOTTOM);
	DBGOUT("RGB_0 MLCINVALIDLEFTRIGHT0  =0x%08x\n", MLC->MLCRGBLAYER[0].MLCINVALIDLEFTRIGHT0);
	DBGOUT("RGB_0 MLCINVALIDTOPBOTTOM0  =0x%08x\n", MLC->MLCRGBLAYER[0].MLCINVALIDTOPBOTTOM0);
	DBGOUT("RGB_0 MLCINVALIDLEFTRIGHT1  =0x%08x\n", MLC->MLCRGBLAYER[0].MLCINVALIDLEFTRIGHT1);
	DBGOUT("RGB_0 MLCINVALIDTOPBOTTOM1  =0x%08x\n", MLC->MLCRGBLAYER[0].MLCINVALIDTOPBOTTOM1);
	DBGOUT("RGB_0 MLCCONTROL	        =0x%08x\n", MLC->MLCRGBLAYER[0].MLCCONTROL);
	DBGOUT("RGB_0 MLCHSTRIDE	        =0x%08x\n", MLC->MLCRGBLAYER[0].MLCHSTRIDE);
	DBGOUT("RGB_0 MLCVSTRIDE	        =0x%08x\n", MLC->MLCRGBLAYER[0].MLCVSTRIDE);
	DBGOUT("RGB_0 MLCTPCOLOR	        =0x%08x\n", MLC->MLCRGBLAYER[0].MLCTPCOLOR);
	DBGOUT("RGB_0 MLCINVCOLOR		    =0x%08x\n", MLC->MLCRGBLAYER[0].MLCINVCOLOR);
	DBGOUT("RGB_0 MLCADDRESS	        =0x%08x\n", MLC->MLCRGBLAYER[0].MLCADDRESS);

	// MLCRGBLAYER[1]
	DBGOUT("RGB_1 MLCLEFTRIGHT	        =0x%08x\n", MLC->MLCRGBLAYER[1].MLCLEFTRIGHT);
	DBGOUT("RGB_1 MLCTOPBOTTOM 	        =0x%08x\n", MLC->MLCRGBLAYER[1].MLCTOPBOTTOM);
	DBGOUT("RGB_1 MLCINVALIDLEFTRIGHT0  =0x%08x\n", MLC->MLCRGBLAYER[1].MLCINVALIDLEFTRIGHT0);
	DBGOUT("RGB_1 MLCINVALIDTOPBOTTOM0  =0x%08x\n", MLC->MLCRGBLAYER[1].MLCINVALIDTOPBOTTOM0);
	DBGOUT("RGB_1 MLCINVALIDLEFTRIGHT1  =0x%08x\n", MLC->MLCRGBLAYER[1].MLCINVALIDLEFTRIGHT1);
	DBGOUT("RGB_1 MLCINVALIDTOPBOTTOM1  =0x%08x\n", MLC->MLCRGBLAYER[1].MLCINVALIDTOPBOTTOM1);
	DBGOUT("RGB_1 MLCCONTROL	        =0x%08x\n", MLC->MLCRGBLAYER[1].MLCCONTROL);
	DBGOUT("RGB_1 MLCHSTRIDE	        =0x%08x\n", MLC->MLCRGBLAYER[1].MLCHSTRIDE);
	DBGOUT("RGB_1 MLCVSTRIDE	        =0x%08x\n", MLC->MLCRGBLAYER[1].MLCVSTRIDE);
	DBGOUT("RGB_1 MLCTPCOLOR	        =0x%08x\n", MLC->MLCRGBLAYER[1].MLCTPCOLOR);
	DBGOUT("RGB_1 MLCINVCOLOR		    =0x%08x\n", MLC->MLCRGBLAYER[1].MLCINVCOLOR);
	DBGOUT("RGB_1 MLCADDRESS	        =0x%08x\n", MLC->MLCRGBLAYER[1].MLCADDRESS);

	// MLCVIDEOLAYER
	DBGOUT("VIDEO MLCLEFTRIGHT          =0x%08x\n", MLC->MLCVIDEOLAYER.MLCLEFTRIGHT);
	DBGOUT("VIDEO MLCTOPBOTTOM			=0x%08x\n", MLC->MLCVIDEOLAYER.MLCTOPBOTTOM);
	DBGOUT("VIDEO MLCCONTROL	        =0x%08x\n", MLC->MLCVIDEOLAYER.MLCCONTROL);
	DBGOUT("VIDEO MLCVSTRIDE	        =0x%08x\n", MLC->MLCVIDEOLAYER.MLCVSTRIDE);
	DBGOUT("VIDEO MLCTPCOLOR	        =0x%08x\n", MLC->MLCVIDEOLAYER.MLCTPCOLOR);
	DBGOUT("VIDEO MLCADDRESS	        =0x%08x\n", MLC->MLCVIDEOLAYER.MLCADDRESS);
	DBGOUT("VIDEO MLCADDRESSCB	        =0x%08x\n", MLC->MLCVIDEOLAYER.MLCADDRESSCB);
	DBGOUT("VIDEO MLCADDRESSCR	        =0x%08x\n", MLC->MLCVIDEOLAYER.MLCADDRESSCR);
	DBGOUT("VIDEO MLCVSTRIDECB	        =0x%08x\n", MLC->MLCVIDEOLAYER.MLCVSTRIDECB);
	DBGOUT("VIDEO MLCVSTRIDECR	        =0x%08x\n", MLC->MLCVIDEOLAYER.MLCVSTRIDECR);
	DBGOUT("VIDEO MLCHSCALE	            =0x%08x\n", MLC->MLCVIDEOLAYER.MLCHSCALE);
	DBGOUT("VIDEO MLCVSCALE	            =0x%08x\n", MLC->MLCVIDEOLAYER.MLCVSCALE);
	DBGOUT("VIDEO MLCLUENH	            =0x%08x\n", MLC->MLCVIDEOLAYER.MLCLUENH);
	DBGOUT("VIDEO MLCCHENH[0]			=0x%08x\n", MLC->MLCVIDEOLAYER.MLCCHENH[0]);
	DBGOUT("VIDEO MLCCHENH[1]			=0x%08x\n", MLC->MLCVIDEOLAYER.MLCCHENH[1]);
	DBGOUT("VIDEO MLCCHENH[2]			=0x%08x\n", MLC->MLCVIDEOLAYER.MLCCHENH[2]);
	DBGOUT("VIDEO MLCCHENH[3]			=0x%08x\n", MLC->MLCVIDEOLAYER.MLCCHENH[3]);

	// MLCRGBLAYER2
	DBGOUT("RGB_2 MLCLEFTRIGHT	        =0x%08x\n", MLC->MLCRGBLAYER2.MLCLEFTRIGHT);
	DBGOUT("RGB_2 MLCTOPBOTTOM 	        =0x%08x\n", MLC->MLCRGBLAYER2.MLCTOPBOTTOM);
	DBGOUT("RGB_2 MLCINVALIDLEFTRIGHT0  =0x%08x\n", MLC->MLCRGBLAYER2.MLCINVALIDLEFTRIGHT0);
	DBGOUT("RGB_2 MLCINVALIDTOPBOTTOM0  =0x%08x\n", MLC->MLCRGBLAYER2.MLCINVALIDTOPBOTTOM0);
	DBGOUT("RGB_2 MLCINVALIDLEFTRIGHT1  =0x%08x\n", MLC->MLCRGBLAYER2.MLCINVALIDLEFTRIGHT1);
	DBGOUT("RGB_2 MLCINVALIDTOPBOTTOM1	=0x%08x\n", MLC->MLCRGBLAYER2.MLCINVALIDTOPBOTTOM1);
	DBGOUT("RGB_2 MLCCONTROL	        =0x%08x\n", MLC->MLCRGBLAYER2.MLCCONTROL);
	DBGOUT("RGB_2 MLCHSTRIDE	        =0x%08x\n", MLC->MLCRGBLAYER2.MLCHSTRIDE);
	DBGOUT("RGB_2 MLCVSTRIDE	        =0x%08x\n", MLC->MLCRGBLAYER2.MLCVSTRIDE);
	DBGOUT("RGB_2 MLCTPCOLOR	        =0x%08x\n", MLC->MLCRGBLAYER2.MLCTPCOLOR);
	DBGOUT("RGB_2 MLCINVCOLOR 	        =0x%08x\n", MLC->MLCRGBLAYER2.MLCINVCOLOR);
	DBGOUT("RGB_2 MLCADDRESS	        =0x%08x\n", MLC->MLCRGBLAYER2.MLCADDRESS);

	DBGOUT("MLCGAMMACONT	        	=0x%08x\n", MLC->MLCGAMMACONT);
	DBGOUT("MLCRGAMMATABLEWRITE			=0x%08x\n", MLC->MLCRGAMMATABLEWRITE);
	DBGOUT("MLCGGAMMATABLEWRITE			=0x%08x\n", MLC->MLCGGAMMATABLEWRITE);
	DBGOUT("MLCBGAMMATABLEWRITE			=0x%08x\n", MLC->MLCBGAMMATABLEWRITE);
	DBGOUT("MLCCLKENB;              	=0x%08x\n", MLC->MLCCLKENB);
}
#endif



