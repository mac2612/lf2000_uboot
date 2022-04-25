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

#include "eth.c"

/* debug print for hw status */
#ifdef DEBUG_REG
static void dbg_reg_dpc(int dev);
static void dbg_reg_mlc(int dev);
#endif

/* debug macro */
#define DBGOUT(msg...)		{ printf(msg); }

/*------------------------------------------------------------------------------
 * intialize nexell soc and board status.
 */
static void init_gpio_pad(void);
static void init_alive_pad(void);
static void init_bus_pad(void);
static void init_display(int dev);

int board_init(void)
{
	int lcd = CFG_DISP_MAIN_SCREEN;

	init_gpio_pad();
	init_alive_pad();
	init_bus_pad();

	init_display(lcd);

#ifdef CONFIG_LCD
	lcd_init();
#endif

	DBGOUT("%s : done board initialize ...\n", CFG_SYS_BOARD_NAME);
	return 0;
}

/*------------------------------------------------------------------------------
 * intialize nexell soc gpio pad func.
 */
static void init_gpio_pad(void)
{
	int  io_grp, io_bit;
	U32  io_mod, outval, detect, pullup, strength;

	const U32 io_pad[NUMBER_OF_GPIO_MODULE][32] = {
		/* GPIO group A */
		{
		PAD_GPIOA0,  PAD_GPIOA1,  PAD_GPIOA2,  PAD_GPIOA3,  PAD_GPIOA4,  PAD_GPIOA5,  PAD_GPIOA6,  PAD_GPIOA7,  PAD_GPIOA8,  PAD_GPIOA9,
		PAD_GPIOA10, PAD_GPIOA11, PAD_GPIOA12, PAD_GPIOA13, PAD_GPIOA14, PAD_GPIOA15, PAD_GPIOA16, PAD_GPIOA17, PAD_GPIOA18, PAD_GPIOA19,
		PAD_GPIOA20, PAD_GPIOA21, PAD_GPIOA22, PAD_GPIOA23, PAD_GPIOA24, PAD_GPIOA25, PAD_GPIOA26, PAD_GPIOA27, PAD_GPIOA28, PAD_GPIOA29,
		PAD_GPIOA30, PAD_GPIOA31
		},
		/* GPIO group B */
		{
		PAD_GPIOB0,  PAD_GPIOB1 , PAD_GPIOB2 , PAD_GPIOB3,  PAD_GPIOB4,  PAD_GPIOB5,  PAD_GPIOB6,  PAD_GPIOB7,  PAD_GPIOB8,  PAD_GPIOB9,
		PAD_GPIOB10, PAD_GPIOB11, PAD_GPIOB12, PAD_GPIOB13, PAD_GPIOB14, PAD_GPIOB15, PAD_GPIOB16, PAD_GPIOB17, PAD_GPIOB18, PAD_GPIOB19,
		PAD_GPIOB20, PAD_GPIOB21, PAD_GPIOB22, PAD_GPIOB23, PAD_GPIOB24, PAD_GPIOB25, PAD_GPIOB26, PAD_GPIOB27, PAD_GPIOB28, PAD_GPIOB29,
		PAD_GPIOB30, PAD_GPIOB31
		},
		/* GPIO group C */
		{
		PAD_GPIOC0,  PAD_GPIOC1,  PAD_GPIOC2,  PAD_GPIOC3,  PAD_GPIOC4,  PAD_GPIOC5,  PAD_GPIOC6,  PAD_GPIOC7,  PAD_GPIOC8,  PAD_GPIOC9,
		PAD_GPIOC10, PAD_GPIOC11, PAD_GPIOC12, PAD_GPIOC13, PAD_GPIOC14, PAD_GPIOC15, PAD_GPIOC16, PAD_GPIOC17, PAD_GPIOC18, PAD_GPIOC19,
		PAD_GPIOC20, 		   0, 		    0,  	     0,           0,           0,           0,           0,           0,           0,
		          0,           0
		}
	};

	/* GPIO pad function */
	for (io_grp = 0; NUMBER_OF_GPIO_MODULE > io_grp; io_grp++) {
		for (io_bit = 0; 32 > io_bit; io_bit++) {

			if (! io_pad[io_grp][io_bit])
				continue;

			io_mod = PAD_GET_PADMODE(io_pad[io_grp][io_bit]);

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
				detect = PAD_GET_DECTMODE(io_pad[io_grp][io_bit]);
				NX_GPIO_SetPadFunction(io_grp, io_bit, NX_GPIO_PADFUNC_GPIO);
				NX_GPIO_SetOutputEnable(io_grp, io_bit, CFALSE);
				NX_GPIO_SetInterruptMode(io_grp, io_bit, detect);
				break;
			case PAD_MODE_SKIP:
			case PAD_NOTEXIST:
			default:
				continue;
			}
			outval   = PAD_GET_OUTLEVEL(io_pad[io_grp][io_bit]);
			pullup   = PAD_GET_PULLUP(io_pad[io_grp][io_bit]);
			strength = PAD_GET_STRENGTH(io_pad[io_grp][io_bit]);

			NX_GPIO_SetOutputValue (io_grp, io_bit, (outval ? CTRUE : CFALSE));
			NX_GPIO_SetPullUpEnable(io_grp, io_bit, (pullup ? CTRUE : CFALSE));
			NX_CLKPWR_SetGPIOPadStrength(io_grp, io_bit, strength);
		}
	}
}

static void init_alive_pad(void)
{
	int  io_bit;
	U32  io_mod, outval, detect, pullup, io_num;

	const U32 alv_pad[] = {
		PAD_GPIOALV0, PAD_GPIOALV1,  PAD_GPIOALV2, PAD_GPIOALV3
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
		PAD_BUS_STATIC_CNTL, PAD_BUS_STATIC_ADDR,  PAD_BUS_STATIC_DATA, PAD_BUS_VSYNC, PAD_BUS_HSYNC, PAD_BUS_DE
			};

	busnum = sizeof(bus_pad)/sizeof(bus_pad[0]);

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
extern void nx_boot_logo(U32 base, int x_resol, int y_resol, U32 pixelbyte);

struct pwm_pad{
	unsigned int group;
	unsigned int bit;
	unsigned int func;
};

static struct pwm_pad pwm_ch[] = {
	{ PAD_GET_GRP(PAD_GPIO_C), 15, NX_GPIO_PADFUNC_1 },
	{ PAD_GET_GRP(PAD_GPIO_C), 16, NX_GPIO_PADFUNC_1 },
	{ PAD_GET_GRP(PAD_GPIO_B),  5, NX_GPIO_PADFUNC_2 },
	{ PAD_GET_GRP(PAD_GPIO_B),  6, NX_GPIO_PADFUNC_2 }
};

static void init_display(int dev)
{
	volatile int delay = 0;

	NX_DPC_DITHER RDither, GDither, BDither;

	U32  DPCFormat = CFG_DISP_PRI_OUT_FORMAT;
  	bool bEmbSync  = CFALSE;
    U32  XResol	   = CFG_DISP_PRI_RESOL_WIDTH;
    U32  YResol	   = CFG_DISP_PRI_RESOL_HEIGHT;
	U32	 LockSize  = CFG_DISP_PRI_MLC_LOCKSIZE;

	U32  PFrameBase= (CFG_MEM_PHY_BLOCK_BASE ? CFG_MEM_PHY_BLOCK_BASE : CFG_MEM_PHY_LINEAR_BASE);
	U32  VFrameBase= (U32)IO_ADDRESS(PFrameBase);

	if ((dev == 0 && CFALSE == CFG_DISP_PRI_BOOT_ENB))
		return;

	//--------------------------------------------------------------------------
	// LCD Off
	//--------------------------------------------------------------------------
#if	(CFG_LCD_PRI_LCD_ON)
	if (dev == 0) {
		NX_GPIO_SetOutputValue(PAD_GET_GRP(CFG_PIO_LCD_PCI_ENB), PAD_GET_BIT(CFG_PIO_LCD_PCI_ENB), CFALSE);
		for (delay=0; 100 > delay; delay++) { ; }
	}
#endif

	//--------------------------------------------------------------------------
	// DPC Clock Disalbe
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
	NX_DPC_SetClockDivisor (dev, 0, CFG_DISP_PRI_CLKGEN0_DIV);		// CLKDIV
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
	             	  CFG_DISP_PRI_OUT_CLK_INVERT,		// PADCLKINV
					  CFG_DISP_PRI_OUT_DUAL_VIEW);

	NX_DPC_SetHSync(dev,
					  XResol,
					  CFG_DISP_PRI_HSYNC_SYNC_WIDTH,
					  CFG_DISP_PRI_HSYNC_FRONT_PORCH,
					  CFG_DISP_PRI_HSYNC_BACK_PORCH,
					  CFG_DISP_PRI_HSYNC_ACTIVE_HIGH);

	NX_DPC_SetVSync(dev,
					  YResol,
					  CFG_DISP_PRI_VSYNC_SYNC_WIDTH,
					  CFG_DISP_PRI_VSYNC_FRONT_PORCH,
					  CFG_DISP_PRI_VSYNC_BACK_PORCH,
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
	             	  CFG_DISP_PRI_OUT_CLK_INVERT,		// PADCLKINV
					  CFG_DISP_PRI_OUT_DUAL_VIEW);

	for (delay=0; 100 > delay; delay++) { ; }

	#ifdef DEBUG_REG
	dbg_reg_dpc(dev);
	#endif

	//--------------------------------------------------------------------------
	// LCD On
	//--------------------------------------------------------------------------
#if	(CFG_LCD_PRI_LCD_ON)
	if (dev == 0)
		NX_GPIO_SetOutputValue(PAD_GET_GRP(CFG_PIO_LCD_PCI_ENB), PAD_GET_BIT(CFG_PIO_LCD_PCI_ENB), CTRUE);
#endif

	//--------------------------------------------------------------------------
	// LCD Backlight On
	//--------------------------------------------------------------------------
	if (dev == 0 && CFG_LCD_PRI_BLU_ON == CTRUE) {
		NX_PWM_SetClockPClkMode	 (NX_PCLKMODE_ALWAYS);
		NX_PWM_SetClockDivisorEnable(CFALSE);
		NX_PWM_SetClockSource		 (0, CFG_PWM_CLK_SOURCE);
		NX_PWM_SetClockDivisor		 (0, CFG_PWM_CLK_DIV);

		NX_PWM_SetPreScale	(CFG_LCD_PRI_PWM_CH, CFG_PWM_CLK_FREQ/CFG_PWM_CLK_DIV/CFG_PWM_PERIOD/CFG_LCD_PRI_PWM_FREQ);
		NX_PWM_SetPeriod	(CFG_LCD_PRI_PWM_CH, CFG_PWM_PERIOD);
		NX_PWM_SetDutyCycle(CFG_LCD_PRI_PWM_CH, CFG_LCD_PRI_PWM_DUTYCYCLE);

		NX_PWM_SetClockDivisorEnable(CTRUE);

		/* Enable pwm out pad*/
		NX_GPIO_SetPadFunction(
			pwm_ch[CFG_LCD_PRI_PWM_CH].group, pwm_ch[CFG_LCD_PRI_PWM_CH].bit, (NX_GPIO_PADFUNC)pwm_ch[CFG_LCD_PRI_PWM_CH].func
				);
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

	if (dev == 0)
		nx_boot_logo(VFrameBase, XResol, YResol, CFG_DISP_SCREEN_PIXEL_BYTE);

	if (dev == 1) {
		PFrameBase += (CFG_DISP_PRI_RESOL_WIDTH*CFG_DISP_PRI_RESOL_HEIGHT*CFG_DISP_SCREEN_PIXEL_BYTE);
		VFrameBase += (CFG_DISP_PRI_RESOL_WIDTH*CFG_DISP_PRI_RESOL_HEIGHT*CFG_DISP_SCREEN_PIXEL_BYTE);
		nx_boot_logo(VFrameBase, XResol, YResol, CFG_DISP_SCREEN_PIXEL_BYTE);
	}

	NX_MLC_SetRGBLayerStride(dev, CFG_DISP_LAYER_SCREEN, CFG_DISP_SCREEN_PIXEL_BYTE, XResol*CFG_DISP_SCREEN_PIXEL_BYTE);
	NX_MLC_SetRGBLayerAddress(dev, CFG_DISP_LAYER_SCREEN, PFrameBase);

	NX_MLC_SetLayerEnable(dev, CFG_DISP_LAYER_SCREEN, CTRUE);
	NX_MLC_SetDirtyFlag(dev, CFG_DISP_LAYER_SCREEN);

	#ifdef DEBUG_REG
	dbg_reg_mlc(dev);
	#endif
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



