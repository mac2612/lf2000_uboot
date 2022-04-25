/*
 * LeapFrog Enterprises (C) 2011
 * Scott Esters <sesters@leapfrog.com>
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
#include <lcd.h>

/* nexell soc headers */
#include <platform.h>
#include <lf2000_board.h>
#include <board_revisions.h>
#include <mach/lfp100.h>

#if	(0)
#define DBGOUT(msg...)		{ printf("lcd: " msg); }
#define	ERROUT(msg...)		{ printf("lcd: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#define	ERROUT(msg...)		do {} while (0)
#endif

DECLARE_GLOBAL_DATA_PTR;

/*------------------------------------------------------------------------------
 * u-boot lcd interface
 */
#ifdef	CONFIG_LCD	// DEBUG_LCD

#define	LCD_BYTEPIXEL 	CFG_DISP_SCREEN_PIXEL_BYTE
#define LCD_BITPIXEL	(LCD_BYTEPIXEL * 8)

vidinfo_t panel_info;

void  * lcd_base;
int 	lcd_line_length;
int 	lcd_color_fg;
int 	lcd_color_bg;

void  * lcd_console_address;	/* Start of console buffer	*/
short 	console_col 	= CONSOLE_CHAR_COLUMN_FIRST;
short 	console_row	= CONSOLE_CHAR_ROW_FIRST;

/* spi_init() routine sets channel info based on board id */
U32	spi_lcd  = -1;
U32	spi_gpio = -1;
U32	spi_cs   = -1;

#define ADC_LCD		0		// ADC channel for LCD ID
#define ADC_TIMEOUT	1000		// ADC timeout msec

#define HX8238		0		// Hymax 320x240 controller
#define ILI9322 	1		// Ilitek 320x240 controller
#define ILI6480G2	2		// Ilitek 480x272 controller

int	lcd_type = HX8238;		// 0 = HX8238, 1 = ILI9322, 2 = ILI6480G2

/* initial LFP100 WLED (reg 0x0A) setting which should match Linux / Brio level 2 */
#define LFP100_WLED_NONE	-1	// don't set brightness

#define LFP100_WLED_GIANT_PLUS_320x240		0x13
#define LFP100_WLED_GOWORLD_320x240		0x18
#define LFP100_WLED_POWERTIP_320x240		0x16

#define LFP100_WLED_GIANT_PLUS_320x240_PP	0x18
#define LFP100_WLED_GOWORLD_320x240_PP		0x15
#define LFP100_WLED_POWERTIP_320x240_PP		0x19

#define LFP100_WLED_GIANT_PLUS_480x272		0x18
#define LFP100_WLED_GOWORLD_480x272		0x18
#define LFP100_WLED_POWERTIP_480x272		0x1b
/*
 * dynamicly setup panel info structure
 */
void init_panel_info(void)
{
	panel_info.vl_col  = get_lf2000_board_info()->cfg_disp_pri_resol_width;
	panel_info.vl_row  = get_lf2000_board_info()->cfg_disp_pri_resol_height;
	panel_info.vl_bpix = LCD_COLOR32;
	panel_info.cmap    = NULL;

}

/*
 * spi_lcd_init()
 *     Based on Linux driver code by dmilici
 */
int get_median3(int a, int b, int c)
{
	if (a<b) {
		// a X Y   or   c a b
		if (a<c)
			return b<c ? b : c;
		else
			// 2 0 1
			return a;
	}
	else {
		// b X Y   or   c b a
		if (b<c)
			return a<c ? a : c;
		else
			return b;
	}
}

#if defined (CONFIG_ARCH_LF2000)

#define ADC_LCD_CHAN            0	/* adc channel */
#define ADC_DELTA_SENSE		118	/* per sense resistor */
#define ADC_TIMEOUT		1000	/* msec */
#define ADC_MIN_SENSE		10	/* bad GP sense resistor */

#define NUM_LCD_ENTRIES         25

#define	MFG_GOWORLD	1
#define MFG_GIANT_PLUS	2
#define	MFG_POWERTIP	3
#define MFG_RESERVED	4

static const int lcd_table_mfg[NUM_LCD_ENTRIES] = {
	MFG_GOWORLD,	// "GoWorld-1",
	MFG_GOWORLD,	// "GoWorld-2",
	MFG_GOWORLD,	// "GoWorld-3",
	MFG_GOWORLD,	// "GoWorld-4",
	MFG_GOWORLD,	// "GoWorld-5",
	MFG_GIANT_PLUS,	// "GiantPlus-1",
	MFG_GIANT_PLUS,	// "GiantPlus-2",
	MFG_GIANT_PLUS,	// "GiantPlus-3",
	MFG_GIANT_PLUS,	// "GiantPlus-4",
	MFG_GIANT_PLUS,	// "GiantPlus-5",
	MFG_POWERTIP,	// "PowerTip-1",
	MFG_POWERTIP,	// "PowerTip-2",
	MFG_POWERTIP,	// "PowerTip-3",
	MFG_POWERTIP,	// "PowerTip-4",
	MFG_POWERTIP,	// "PowerTip-5",
	MFG_RESERVED,	// "Reserved1-1",
	MFG_RESERVED,	// "Reserved1-2",
	MFG_RESERVED,	// "Reserved1-3",
	MFG_RESERVED,	// "Reserved1-4",
	MFG_RESERVED,	// "Reserved1-5",
	MFG_RESERVED,	// "Reserved2-1",
	MFG_RESERVED,	// "Reserved2-2",
	MFG_RESERVED,	// "Reserved2-3",
	MFG_RESERVED,	// "Reserved2-4",
	MFG_RESERVED,	// "Reserved2-5",
};

void adc_lcd_init(void)
{
	unsigned short lcd_sense[4] = {0, 0, 0, 0};
	int lcd_mfg = 0;
	unsigned short lcd_index = 0;

	int	wled_backlight = LFP100_WLED_NONE;	// initial backlight

	soc_adc_attach();
	lcd_sense[0] = soc_adc_read(ADC_LCD, ADC_TIMEOUT);
	lcd_sense[1] = soc_adc_read(ADC_LCD, ADC_TIMEOUT);
	lcd_sense[2] = soc_adc_read(ADC_LCD, ADC_TIMEOUT);
	lcd_sense[3] = soc_adc_read(ADC_LCD, ADC_TIMEOUT);
	soc_adc_detach();

	lcd_index = lcd_sense[0] / ADC_DELTA_SENSE;
	if (lcd_sense[0] < ADC_MIN_SENSE)
		lcd_index = 5;	/* GP hack */
	if (lcd_index < NUM_LCD_ENTRIES)
		lcd_mfg = lcd_table_mfg[lcd_index];

	// get median of 3 samples after discarding 1st one
	DBGOUT("%s: lcd_sense=%d, %d, %d, %d\n", __func__, lcd_sense[0], lcd_sense[1], lcd_sense[2], lcd_sense[3]);
	lcd_sense[0] = get_median3(lcd_sense[1], lcd_sense[2], lcd_sense[3]);

	switch(get_board_rev()) {
	case LF2000_BOARD_VTK:
		lcd_type = ILI6480G2;
		break;

	case LF2000_BOARD_LUCY:
		switch(lcd_mfg) {
		case MFG_GOWORLD:
			lcd_type = HX8238;
			wled_backlight = LFP100_WLED_GOWORLD_320x240;
			break;
		case MFG_GIANT_PLUS:
			lcd_type = ILI9322;
			wled_backlight = LFP100_WLED_GIANT_PLUS_320x240;
			break;
		case MFG_POWERTIP:
			lcd_type = HX8238;
			wled_backlight = LFP100_WLED_POWERTIP_320x240;
			break;
		}
		break;

	case LF2000_BOARD_LUCY_CIP:
	case LF2000_BOARD_LUCY_PP:
		switch(lcd_mfg) {
		case MFG_GOWORLD:
			lcd_type = HX8238;
			wled_backlight = LFP100_WLED_GOWORLD_320x240_PP;
			break;
		case MFG_GIANT_PLUS:
			lcd_type = ILI9322;
			wled_backlight = LFP100_WLED_GIANT_PLUS_320x240_PP;
			break;
		case MFG_POWERTIP:
			lcd_type = HX8238;
			wled_backlight = LFP100_WLED_POWERTIP_320x240_PP;
			break;
		}
		break;

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

		switch(lcd_mfg) {
		case MFG_GOWORLD:
			lcd_type = ILI6480G2;
			wled_backlight = LFP100_WLED_GOWORLD_480x272;
			break;
		case MFG_GIANT_PLUS:
			lcd_type = ILI6480G2;
			wled_backlight = LFP100_WLED_GIANT_PLUS_480x272;
			break;
		case MFG_POWERTIP:
			lcd_type = ILI6480G2;
			wled_backlight = LFP100_WLED_POWERTIP_480x272;
			break;
		}
		break;
	case LF2000_BOARD_UNKNOWN:
	default:
		lcd_type = ILI6480G2;
		wled_backlight = LFP100_WLED_POWERTIP_480x272;
		break;
	}

	if (wled_backlight != LFP100_WLED_NONE)
		lfp100_wled_set(wled_backlight);
	printf("%s: lcd_sense=%d, lcd_mfg=%d lcd_type=%d wled_backlight=0x%X\n",
		__func__, lcd_sense[0], lcd_mfg, lcd_type, wled_backlight);
}

void spi_lcd_init(void)
{
	switch(get_board_rev()) {
	case LF2000_BOARD_LUCY:
	case LF2000_BOARD_LUCY_CIP:
	case LF2000_BOARD_LUCY_PP:
		spi_lcd  = 0;
		spi_gpio = 1;
		spi_cs   = 12;
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
	case LF2000_BOARD_VTK:
	case LF2000_BOARD_UNKNOWN:
	default:
		spi_lcd  = 1;
		spi_gpio = 2;
		spi_cs   = 3;
		break;
	}

	DBGOUT("%s: spi: %d, lcd: %d\n", __func__, spi_lcd, lcd_type);

	// SPI init module
	NX_SSPSPI_SetBaseAddress(spi_lcd, (U32)IO_ADDRESS(NX_SSPSPI_GetPhysicalAddress(spi_lcd)));
	NX_SSPSPI_OpenModule(spi_lcd);

	// SPI init setup
	NX_SSPSPI_SetDividerCount(spi_lcd, 4);		// prescale divider

	switch(get_board_rev()) {
	case LF2000_BOARD_LUCY:
	case LF2000_BOARD_LUCY_CIP:
	case LF2000_BOARD_LUCY_PP:
		NX_SSPSPI_SetBitWidth(spi_lcd, (lcd_type == ILI9322) ? 16 : 8); // 8-bit
		break;

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
	case LF2000_BOARD_VTK:
	case LF2000_BOARD_UNKNOWN:
	default:
		NX_SSPSPI_SetBitWidth(spi_lcd, 16);	// 16-bit
		break;
	}

	NX_SSPSPI_SetEnable(spi_lcd, CTRUE);
	NX_SSPSPI_SetByteSwap(spi_lcd, CFALSE);                                 // no byte swap
	NX_SSPSPI_SetProtocol(spi_lcd, NX_SSPSPI_PROTOCOL_SPI); // SPI
	NX_SSPSPI_SetSPIFormat(spi_lcd, NX_SSPSPI_FORMAT_A);    // format
	NX_SSPSPI_SetSlaveMode(spi_lcd, CFALSE);                                // master
	NX_SSPSPI_SetClockPolarityInvert(spi_lcd, CFALSE);              // normal polarity

	// SPI enable
	NX_SSPSPI_SetEnable(spi_lcd, CTRUE);
	NX_SSPSPI_ResetFIFO(spi_lcd);

	NX_SSPSPI_SetClockPClkMode(spi_lcd, NX_PCLKMODE_ALWAYS);
	NX_SSPSPI_SetClockSource(spi_lcd, 0, 1);                                // PLL1
	NX_SSPSPI_SetClockDivisor(spi_lcd, 0, 0xE);                     // divisor = ?
	NX_SSPSPI_SetClockDivisorEnable(spi_lcd, CTRUE);

	switch(get_board_rev()) {
	case LF2000_BOARD_LUCY:
	case LF2000_BOARD_LUCY_CIP:
	case LF2000_BOARD_LUCY_PP:
		if (lcd_type == HX8238) {
			// SPI CS needs manual assertion for 24-bit transfers
			NX_GPIO_SetPadFunction (spi_gpio, spi_cs, NX_GPIO_PADFUNC_GPIO);
			NX_GPIO_SetOutputEnable(spi_gpio, spi_cs, CTRUE);
			NX_GPIO_SetOutputValue (spi_gpio, spi_cs, 1);
		}
		break;

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
	case LF2000_BOARD_VTK:
	case LF2000_BOARD_UNKNOWN:
	default:
		NX_GPIO_SetPadFunction (spi_gpio, spi_cs, NX_GPIO_PADFUNC_1);
		break;
	}

}

void spi_lcd_write(u8 reg, u8 val)
{
	// SPI LCD 16-bit register write
	NX_SSPSPI_PutHalfWord(spi_lcd, (reg<<10) | (0<<9) | val);
	while (!NX_SSPSPI_IsTxFIFOEmpty(spi_lcd))
		udelay(1);
}

void spi_lcd_write16(u8 reg, u8 val)
{
	// SPI LCD 16-bit register write for ILI9322
	NX_SSPSPI_PutHalfWord(spi_lcd, (reg<<8) | val);
	while (!NX_SSPSPI_IsTxFIFOEmpty(spi_lcd))
		udelay(1);
}

void spi_lcd_write24(u8 reg, u16 val)
{
	// SPI LCD 24-bit register write
	NX_GPIO_SetOutputValue (spi_gpio, spi_cs, 0);

	NX_SSPSPI_PutByte(spi_lcd, 0x70);
	while (!NX_SSPSPI_IsTxFIFOEmpty(spi_lcd)) {
		udelay(0);
	}
	while (NX_SSPSPI_IsRxFIFOEmpty(spi_lcd)) {
		udelay(0);
	}
	NX_SSPSPI_GetByte(spi_lcd);
	NX_SSPSPI_PutByte(spi_lcd, 0x00);
	while (!NX_SSPSPI_IsTxFIFOEmpty(spi_lcd)) {
		udelay(0);
	}
	while (NX_SSPSPI_IsRxFIFOEmpty(spi_lcd)) {
		udelay(0);
	}
	NX_SSPSPI_GetByte(spi_lcd);
	NX_SSPSPI_PutByte(spi_lcd, reg);
	while (!NX_SSPSPI_IsTxFIFOEmpty(spi_lcd)) {
		udelay(0);
	}
	while (NX_SSPSPI_IsRxFIFOEmpty(spi_lcd)) {
		udelay(0);
	}
	NX_SSPSPI_GetByte(spi_lcd);
	while (!NX_SSPSPI_IsTxShiftEmpty(spi_lcd)) {
		udelay(0);
	}

	NX_GPIO_SetOutputValue (spi_gpio, spi_cs, 1);
	NX_GPIO_SetOutputValue (spi_gpio, spi_cs, 0);

	NX_SSPSPI_PutByte(spi_lcd, 0x72);
	while (!NX_SSPSPI_IsTxFIFOEmpty(spi_lcd)) {
		udelay(0);
	}
	while (NX_SSPSPI_IsRxFIFOEmpty(spi_lcd)) {
		udelay(0);
	}
	NX_SSPSPI_GetByte(spi_lcd);
	NX_SSPSPI_PutByte(spi_lcd, val >> 8);
	while (!NX_SSPSPI_IsTxFIFOEmpty(spi_lcd)) {
		udelay(0);
	}
	while (NX_SSPSPI_IsRxFIFOEmpty(spi_lcd)) {
		udelay(0);
	}
	NX_SSPSPI_GetByte(spi_lcd);
	NX_SSPSPI_PutByte(spi_lcd, val & 0xFF);
	while (!NX_SSPSPI_IsTxFIFOEmpty(spi_lcd)) {
		udelay(0);
	}
	while (NX_SSPSPI_IsRxFIFOEmpty(spi_lcd)) {
		udelay(0);
	}
	NX_SSPSPI_GetByte(spi_lcd);
	while (!NX_SSPSPI_IsTxShiftEmpty(spi_lcd)) {
		udelay(0);
	}

	NX_GPIO_SetOutputValue (spi_gpio, spi_cs, 1);
}
#endif

void lcd_init(void)
{
	unsigned int fb_base= (CFG_MEM_PHY_BLOCK_BASE ? CFG_MEM_PHY_BLOCK_BASE : CFG_MEM_PHY_LINEAR_BASE);
	DBGOUT("%s\n", __func__);
	gd->fb_base = (unsigned int)IO_ADDRESS(fb_base);

#if defined (CONFIG_ARCH_LF2000)
	/* init LCD via SPI */
	adc_lcd_init();
	spi_lcd_init();
#endif

	switch(get_board_rev()) {
	case LF2000_BOARD_LUCY:
	case LF2000_BOARD_LUCY_CIP:
	case LF2000_BOARD_LUCY_PP:
		if (lcd_type == ILI9322) {
			spi_lcd_write16(0x06, 0x5C);	// flip
			spi_lcd_write16(0x05, 0x22);	// power
		}
		else {
			spi_lcd_write24(0x04, 0x0447);	// format
			spi_lcd_write24(0x05, 0xBCC4);	// sync
			spi_lcd_write24(0x01, 0x313F);	// flip
		}
		break;

	case LF2000_BOARD_VALENCIA:
	case LF2000_BOARD_VALENCIA_EP:
	case LF2000_BOARD_VALENCIA_EP_8:
	case LF2000_BOARD_VALENCIA_FEP:
	case LF2000_BOARD_VALENCIA_FEP_8:
	case LF2000_BOARD_VALENCIA_CIP:
		spi_lcd_write(0x01, 0x1C);
		break;

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
	case LF2000_BOARD_VTK:
	case LF2000_BOARD_UNKNOWN:
		break;
	}
}

void lcd_ctrl_init (void *lcdbase)
{
	DBGOUT("%s\n", __func__);

	lcd_init();

	lcd_line_length = (panel_info.vl_col * LCD_BYTEPIXEL);
}

void lcd_setcolreg (ushort regno, ushort red, ushort green, ushort blue)
{
	DBGOUT("%s\n", __func__);
}

void lcd_enable (void)
{
	DBGOUT("%s\n", __func__);
}

/*
 * Calculate fb size for VIDEOLFB_ATAG. Size returned contains fb,
 * descriptors and palette areas.
 */
ulong calc_fbsize (void)
{
	ulong size;
	int line_length = (panel_info.vl_col * NBITS (panel_info.vl_bpix)) / 8;

	DBGOUT("%s\n", __func__);

	size = line_length * panel_info.vl_row;
	size += PAGE_SIZE;

	return size;
}

#ifdef CONFIG_LCD_INFO

#include <version.h>
#include <nand.h>
#include <flash.h>

void lcd_show_board_info(void)
{
	ulong dram_size;
	unsigned long long nand_size;
	unsigned int board_rev;
	int i;

	dram_size = 0;
	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++)
		dram_size += gd->bd->bi_dram[i].size;

	nand_size = 0;
	for (i = 0; i < CONFIG_SYS_MAX_NAND_DEVICE; i++)
		nand_size += nand_info[i].size;

	if (is_show_lcd_info()) {
		lcd_printf ("%s\n", U_BOOT_VERSION);
		lcd_printf ("Build: '%s'\n", CONFIG_IDENT_STRING);
		lcd_printf ("Compiled %s %s\n", __DATE__, __TIME__);
		lcd_printf ("(C) 2012 LeapFrog Enterprises\n");
		lcd_printf ("%ld MB SDRAM, %lld MB NAND\n",
				dram_size >> 20,
				nand_size >> 20
			   );
		board_rev = get_board_rev();
		lcd_printf ("Board ID:0x%4.4x\n", board_rev);
	}
}
#endif /* CONFIG_LCD_INFO */

#endif /* CONFIG_LCD */
