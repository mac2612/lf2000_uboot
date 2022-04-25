/*
 * Common LCD routines for supported CPUs
 *
 * (C) Copyright 2001-2002
 * Wolfgang Denk, DENX Software Engineering -- wd@denx.de
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
 */

/************************************************************************/
/* ** HEADER FILES							*/
/************************************************************************/

/* #define DEBUG */

#include <config.h>
#include <common.h>
#include <command.h>
#include <stdarg.h>
#include <linux/types.h>
#include <stdio_dev.h>
#if defined(CONFIG_POST)
#include <post.h>
#endif
#include <lcd.h>
#include <watchdog.h>

#if defined CONFIG_VIDEO_BMP_GZIP
#include <malloc.h>
#endif

#if defined(CONFIG_PXA250)
#include <asm/byteorder.h>
#endif

#if defined(CONFIG_MPC823)
#include <lcdvideo.h>
#endif

#if defined(CONFIG_ATMEL_LCD)
#include <atmel_lcdc.h>
#endif

#if defined(CONFIG_ARCH_LF2000)
#include <lf2000_board.h>
#endif

/************************************************************************/
/* ** FONT DATA								*/
/************************************************************************/
#include <video_font.h>		/* Get font data, width and height	*/

/************************************************************************/
/* ** LOGO DATA								*/
/************************************************************************/
#ifdef CONFIG_LCD_LOGO
# include <bmp_logo.h>		/* Get logo data, width and height	*/
# if (CONSOLE_COLOR_WHITE >= BMP_LOGO_OFFSET) && \
	(LCD_BPP != LCD_COLOR16) && \
	(LCD_BPP != LCD_COLOR32)
#  error Default Color Map overlaps with Logo Color Map
# endif
#endif

/************************************************************************/
/* ** SCREEN DATA							*/
/************************************************************************/
#ifdef CONFIG_LCD_SCREENS
#include <screen-list.h>	/* generic names for each screen	*/
#endif

DECLARE_GLOBAL_DATA_PTR;

ulong lcd_setmem (ulong addr);

static void lcd_drawchars (ushort x, ushort y, uchar *str, int count);
static inline void lcd_puts_xy (ushort x, ushort y, uchar *s);
static inline void lcd_putc_xy (ushort x, ushort y, uchar  c);

static int lcd_init (void *lcdbase);

static int lcd_clear (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[]);
static void *lcd_logo (void);

static int lcd_getbgcolor (void);
static void lcd_setfgcolor (int color);
static void lcd_setbgcolor (int color);

char lcd_is_enabled = 0;

#ifdef	NOT_USED_SO_FAR
static void lcd_getcolreg (ushort regno,
				ushort *red, ushort *green, ushort *blue);
static int lcd_getfgcolor (void);
#endif	/* NOT_USED_SO_FAR */

/************************************************************************/

/*----------------------------------------------------------------------*/

static void console_new_screen(void)
{
	console_col = CONSOLE_CHAR_COLUMN_FIRST ;
	console_row = CONSOLE_CHAR_ROW_FIRST;
}

/*----------------------------------------------------------------------*/

static void console_scrollup (void)
{
	/* Copy up rows ignoring the first one */
	memcpy (CONSOLE_BIT_ROW_FIRST, CONSOLE_BIT_ROW_SECOND, CONSOLE_BIT_SCROLL_SIZE);

	/* Clear the last one */
	memset (CONSOLE_BIT_ROW_LAST, COLOR_MASK(lcd_color_bg), CONSOLE_BIT_ROW_SIZE);
}

/*----------------------------------------------------------------------*/

static inline void console_back (void)
{
	if (--console_col < CONSOLE_CHAR_COLUMN_FIRST) {
		console_col = CONSOLE_CHAR_COLUMN_FIRST ;
		if (--console_row < CONSOLE_CHAR_ROW_FIRST) {
			console_row = CONSOLE_CHAR_ROW_FIRST;
		}
	}

	lcd_putc_xy (console_col * VIDEO_FONT_WIDTH,
		     console_row * VIDEO_FONT_HEIGHT,
		     ' ');
}

/*----------------------------------------------------------------------*/

static inline void console_newline (void)
{
	++console_row;
	console_col = CONSOLE_CHAR_COLUMN_FIRST;

	/* Check if we need to scroll the terminal */
	if (console_row >= CONSOLE_CHAR_ROWS) {
		/* Scroll everything up */
		console_scrollup () ;
		console_row = CONSOLE_CHAR_ROWS - 1;
	}
}

/*----------------------------------------------------------------------*/

void lcd_putc (const char c)
{
	if (!lcd_is_enabled) {
		serial_putc(c);
		return;
	}

	switch (c) {
	case '\r':	console_col = CONSOLE_CHAR_COLUMN_FIRST;
			return;

	case '\n':	console_newline();
			return;

	case '\t':	/* Tab (8 chars alignment) */
			console_col +=  8;
			console_col &= ~7;

			if (console_col >= CONSOLE_CHAR_COLS) {
				console_newline();
			}
			return;

	case '\b':	console_back();
			return;

	default:	lcd_putc_xy (console_col * VIDEO_FONT_WIDTH,
				     console_row * VIDEO_FONT_HEIGHT,
				     c);
			if (++console_col >= CONSOLE_CHAR_COLS) {
				console_newline();
			}
			return;
	}
	/* NOTREACHED */
}

/*----------------------------------------------------------------------*/

void lcd_puts (const char *s)
{
	if (!lcd_is_enabled) {
		serial_puts (s);
		return;
	}

	while (*s) {
		lcd_putc (*s++);
	}
}

/*----------------------------------------------------------------------*/

void lcd_printf(const char *fmt, ...)
{
	va_list args;
	char buf[CONFIG_SYS_PBSIZE];

	va_start(args, fmt);
	vsprintf(buf, fmt, args);
	va_end(args);

	lcd_puts(buf);
}

/************************************************************************/
/* ** Low-Level Graphics Routines					*/
/************************************************************************/

static void lcd_drawchars (ushort x, ushort y, uchar *str, int count)
{
	uchar *dest;
	ushort off, row;

	dest = (uchar *)(lcd_base + y * lcd_line_length + x * (1 << LCD_BPP) / 8);
	off  = x * (1 << LCD_BPP) % 8;

	for (row=0;  row < VIDEO_FONT_HEIGHT;  ++row, dest += lcd_line_length)  {
		uchar *s = str;
		int i;
#if LCD_BPP == LCD_COLOR16
		ushort *d = (ushort *)dest;
#elif LCD_BPP == LCD_COLOR32
		uint *d = (uint *)dest;
#endif

#if LCD_BPP == LCD_MONOCHROME
		uchar rest = *d & -(1 << (8-off));
		uchar sym;
#endif
		for (i=0; i<count; ++i) {
			uchar c, bits;

			c = *s++;
			bits = video_fontdata[c * VIDEO_FONT_HEIGHT + row];

#if LCD_BPP == LCD_MONOCHROME
			sym  = (COLOR_MASK(lcd_color_fg) & bits) |
			       (COLOR_MASK(lcd_color_bg) & ~bits);

			*d++ = rest | (sym >> off);
			rest = sym << (8-off);
#elif LCD_BPP == LCD_COLOR8
			for (c=0; c<8; ++c) {
				*d++ = (bits & 0x80) ?
						lcd_color_fg : lcd_color_bg;
				bits <<= 1;
			}
#elif LCD_BPP == LCD_COLOR16
			for (c=0; c<8; ++c) {
				*d++ = (bits & 0x80) ?
						lcd_color_fg : lcd_color_bg;
				bits <<= 1;
			}
#elif LCD_BPP == LCD_COLOR32
			for (c=0; c<8; ++c) {
				*d++ = (bits & 0x80) ?
						lcd_color_fg : lcd_color_bg;
				bits <<= 1;
			}
#endif
		}
#if LCD_BPP == LCD_MONOCHROME
		*d  = rest | (*d & ((1 << (8-off)) - 1));
#endif
	}
}

/*----------------------------------------------------------------------*/

static inline void lcd_puts_xy (ushort x, ushort y, uchar *s)
{
#if defined(CONFIG_LCD_LOGO) && !defined(CONFIG_LCD_INFO_BELOW_LOGO)
	lcd_drawchars (x, y+BMP_LOGO_HEIGHT, s, strlen ((char *)s));
#else
	lcd_drawchars (x, y, s, strlen ((char *)s));
#endif
}

/*----------------------------------------------------------------------*/

static inline void lcd_putc_xy (ushort x, ushort y, uchar c)
{
#if defined(CONFIG_LCD_LOGO) && !defined(CONFIG_LCD_INFO_BELOW_LOGO)
	lcd_drawchars (x, y+BMP_LOGO_HEIGHT, &c, 1);
#else
	lcd_drawchars (x, y, &c, 1);
#endif
}

/************************************************************************/
/**  Small utility to check that you got the colours right		*/
/************************************************************************/
#ifdef LCD_TEST_PATTERN

#define	N_BLK_VERT	2
#define	N_BLK_HOR	3

static int test_colors[N_BLK_HOR*N_BLK_VERT] = {
	CONSOLE_COLOR_RED,	CONSOLE_COLOR_GREEN,	CONSOLE_COLOR_YELLOW,
	CONSOLE_COLOR_BLUE,	CONSOLE_COLOR_MAGENTA,	CONSOLE_COLOR_CYAN,
};

static void test_pattern (void)
{
	ushort v_max  = LCD_HEIGHT;
	ushort h_max  = LCD_WIDTH;
	ushort v_step = (v_max + N_BLK_VERT - 1) / N_BLK_VERT;
	ushort h_step = (h_max + N_BLK_HOR  - 1) / N_BLK_HOR;
	ushort v, h;
	uchar *pix = (uchar *)lcd_base;

	printf ("[LCD] Test Pattern: %d x %d [%d x %d]\n",
		h_max, v_max, h_step, v_step);

	/* WARNING: Code silently assumes 8bit/pixel */
	for (v=0; v<v_max; ++v) {
		uchar iy = v / v_step;
		for (h=0; h<h_max; ++h) {
			uchar ix = N_BLK_HOR * iy + (h/h_step);
			*pix++ = test_colors[ix];
		}
	}
}
#endif /* LCD_TEST_PATTERN */


/************************************************************************/
/* ** GENERIC Initialization Routines					*/
/************************************************************************/

int drv_lcd_init (void)
{
	struct stdio_dev lcddev;
	int rc;

	lcd_base = (void *)(gd->fb_base);

	lcd_line_length = (panel_info.vl_col * NBITS (panel_info.vl_bpix)) / 8;

	lcd_init (lcd_base);		/* LCD initialization */

	/* Device initialization */
	memset (&lcddev, 0, sizeof (lcddev));

	strcpy (lcddev.name, "lcd");
	lcddev.ext   = 0;			/* No extensions */
	lcddev.flags = DEV_FLAGS_OUTPUT;	/* Output only */
	lcddev.putc  = lcd_putc;		/* 'putc' function */
	lcddev.puts  = lcd_puts;		/* 'puts' function */

	rc = stdio_register (&lcddev);

	return (rc == 0) ? 1 : rc;
}

/*----------------------------------------------------------------------*/
static int lcd_clear (cmd_tbl_t * cmdtp, int flag, int argc, char *argv[])
{
#if LCD_BPP == LCD_MONOCHROME
	/* Setting the palette */
	lcd_initcolregs();

#elif LCD_BPP == LCD_COLOR8
	/* Setting the palette */
	lcd_setcolreg  (CONSOLE_COLOR_BLACK,       0,    0,    0);
	lcd_setcolreg  (CONSOLE_COLOR_RED,	0xFF,    0,    0);
	lcd_setcolreg  (CONSOLE_COLOR_GREEN,       0, 0xFF,    0);
	lcd_setcolreg  (CONSOLE_COLOR_YELLOW,	0xFF, 0xFF,    0);
	lcd_setcolreg  (CONSOLE_COLOR_BLUE,        0,    0, 0xFF);
	lcd_setcolreg  (CONSOLE_COLOR_MAGENTA,	0xFF,    0, 0xFF);
	lcd_setcolreg  (CONSOLE_COLOR_CYAN,	   0, 0xFF, 0xFF);
	lcd_setcolreg  (CONSOLE_COLOR_GREY,	0xAA, 0xAA, 0xAA);
	lcd_setcolreg  (CONSOLE_COLOR_WHITE,	0xFF, 0xFF, 0xFF);
#endif

#ifndef CONFIG_SYS_WHITE_ON_BLACK
	lcd_setfgcolor (CONSOLE_COLOR_BLACK);
	lcd_setbgcolor (CONSOLE_COLOR_WHITE);
#else
	lcd_setfgcolor (CONSOLE_COLOR_WHITE);
	lcd_setbgcolor (CONSOLE_COLOR_BLACK);
#endif	/* CONFIG_SYS_WHITE_ON_BLACK */

#ifdef	LCD_TEST_PATTERN
	test_pattern();
#else
	/* set framebuffer to background color */
	memset ((char *)lcd_base,
		COLOR_MASK(lcd_getbgcolor()),
		lcd_line_length*panel_info.vl_row);
#endif
	/* Paint the logo and retrieve LCD base address */
	debug ("[LCD] Drawing the logo...\n");
	lcd_console_address = lcd_logo ();

	return (0);
}

U_BOOT_CMD(
	cls,	1,	1,	lcd_clear,
	"clear screen",
	""
);

/*----------------------------------------------------------------------*/

static int lcd_init (void *lcdbase)
{
	/* Initialize the lcd controller */
	debug ("[LCD] Initializing LCD frambuffer at %p\n", lcdbase);

	lcd_ctrl_init (lcdbase);
	lcd_is_enabled = 1;
	lcd_clear (NULL, 1, 1, NULL);	/* dummy args */
	lcd_enable ();

	/* ensure console line is on screen */
	if (CONSOLE_CHAR_COLS <= console_col)
		console_col = CONSOLE_CHAR_COLUMN_FIRST;
	if (CONSOLE_CHAR_ROWS <= console_row) {
		debug ("[LCD] ERR: Console row %d is off screen.\n", console_row);
		console_row =  CONSOLE_CHAR_ROWS - 1;
		debug ("[LCD] ERR: console_row reset to %d\n", console_row);
	}

	debug ("[LCD] console_row:%d  console_col:%d\n", console_row, console_col);

	/* Initialize the lcd controller */
	debug ("[LCD] Initializing LCD frambuffer at %p\n", lcdbase);

	return 0;
}


/************************************************************************/
/* ** ROM capable initialization part - needed to reserve FB memory	*/
/************************************************************************/
/*
 * This is called early in the system initialization to grab memory
 * for the LCD controller.
 * Returns new address for monitor, after reserving LCD buffer memory
 *
 * Note that this is running from ROM, so no write access to global data.
 */
ulong lcd_setmem (ulong addr)
{
	ulong size;
	int line_length = (panel_info.vl_col * NBITS (panel_info.vl_bpix)) / 8;

	debug ("LCD panel info: %d x %d, %d bit/pix\n",
		panel_info.vl_col, panel_info.vl_row, NBITS (panel_info.vl_bpix) );

	size = line_length * panel_info.vl_row;

	/* Round up to nearest full page */
	size = (size + (PAGE_SIZE - 1)) & ~(PAGE_SIZE - 1);

	/* Allocate pages for the frame buffer. */
	addr -= size;

	debug ("Reserving %ldk for LCD Framebuffer at: %08lx\n", size>>10, addr);

	return (addr);
}

/*----------------------------------------------------------------------*/

static void lcd_setfgcolor (int color)
{
	lcd_color_fg = color;
}

/*----------------------------------------------------------------------*/

static void lcd_setbgcolor (int color)
{
	lcd_color_bg = color;
}

/*----------------------------------------------------------------------*/

#ifdef	NOT_USED_SO_FAR
static int lcd_getfgcolor (void)
{
	return lcd_color_fg;
}
#endif	/* NOT_USED_SO_FAR */

/*----------------------------------------------------------------------*/

static int lcd_getbgcolor (void)
{
	return lcd_color_bg;
}

/*----------------------------------------------------------------------*/

/************************************************************************/
/* ** Chipset depending Bitmap / Logo stuff...                          */
/************************************************************************/
#ifdef CONFIG_LCD_LOGO
void bitmap_plot (int x, int y)
{
#ifdef CONFIG_ATMEL_LCD
	uint *cmap;
#else
	ushort *cmap;
#endif
	ushort i, j;
	uchar *bmap;
	uchar *fb;
	ushort *fb16;
	ulong *fb32;
#if defined(CONFIG_PXA250)
	struct pxafb_info *fbi = &panel_info.pxa;
#elif defined(CONFIG_MPC823)
	volatile immap_t *immr = (immap_t *) CONFIG_SYS_IMMR;
	volatile cpm8xx_t *cp = &(immr->im_cpm);
#endif

	debug ("Logo: width %d  height %d  colors %d  cmap %d\n",
		BMP_LOGO_WIDTH, BMP_LOGO_HEIGHT, BMP_LOGO_COLORS,
		(int)(sizeof(bmp_logo_palette)/(sizeof(ushort))));

	bmap = &bmp_logo_bitmap[0];
	fb   = (uchar *)(lcd_base + y * lcd_line_length + x);

	if (NBITS(panel_info.vl_bpix) < 12) {
		/* Leave room for default color map */
#if defined(CONFIG_PXA250)
		cmap = (ushort *)fbi->palette;
#elif defined(CONFIG_MPC823)
		cmap = (ushort *)&(cp->lcd_cmap[BMP_LOGO_OFFSET*sizeof(ushort)]);
#elif defined(CONFIG_ATMEL_LCD)
		cmap = (uint *) (panel_info.mmio + ATMEL_LCDC_LUT(0));
#else
		/*
		 * default case: generic system with no cmap (most likely 16bpp)
		 * We set cmap to the source palette, so no change is done.
		 * This avoids even more ifdef in the next stanza
		 */
		cmap = bmp_logo_palette;
#endif

		WATCHDOG_RESET();

		/* Set color map */
		for (i=0; i<(sizeof(bmp_logo_palette)/(sizeof(ushort))); ++i) {
			ushort colreg = bmp_logo_palette[i];
#ifdef CONFIG_ATMEL_LCD
			uint lut_entry;
#ifdef CONFIG_ATMEL_LCD_BGR555
			lut_entry = ((colreg & 0x000F) << 11) |
				    ((colreg & 0x00F0) <<  2) |
				    ((colreg & 0x0F00) >>  7);
#else /* CONFIG_ATMEL_LCD_RGB565 */
			lut_entry = ((colreg & 0x000F) << 1) |
				    ((colreg & 0x00F0) << 3) |
				    ((colreg & 0x0F00) << 4);
#endif
			*(cmap + BMP_LOGO_OFFSET) = lut_entry;
			cmap++;
#else /* !CONFIG_ATMEL_LCD */
#ifdef  CONFIG_SYS_INVERT_COLORS
			*cmap++ = 0xffff - colreg;
#else
			*cmap++ = colreg;
#endif
#endif /* CONFIG_ATMEL_LCD */
		}

		WATCHDOG_RESET();

		for (i=0; i<BMP_LOGO_HEIGHT; ++i) {
			memcpy (fb, bmap, BMP_LOGO_WIDTH);
			bmap += BMP_LOGO_WIDTH;
			fb   += panel_info.vl_col;
		}
	} else if (panel_info.vl_bpix == LCD_COLOR16) { /* true color mode */
		u16 col16;
		fb16 = (ushort *)(lcd_base + y * lcd_line_length + x);
		for (i=0; i<BMP_LOGO_HEIGHT; ++i) {
			for (j=0; j<BMP_LOGO_WIDTH; j++) {
				col16 = bmp_logo_palette[(bmap[j] - BMP_LOGO_OFFSET)];
				fb16[j] =
					((col16 & 0x000F) << 1) |
					((col16 & 0x00F0) << 3) |
					((col16 & 0x0F00) << 4);
				}
			bmap += BMP_LOGO_WIDTH;
			fb16 += panel_info.vl_col;
		}
	} else if (panel_info.vl_bpix == LCD_COLOR32) { /* true color mode */
		u16 col16;
		fb32 = (ulong *)(lcd_base + y * lcd_line_length + x);
		debug("%s:%s.%d panel_info.vl_bpix=0x%X (LCD_COLOR32)\n",
			__FILE__, __func__, __LINE__, panel_info.vl_bpix);

		for (i=0; i<BMP_LOGO_HEIGHT; ++i) {
			for (j=0; j<BMP_LOGO_WIDTH; j++) {
				/* lookup in color table */
				col16 = bmp_logo_palette[(bmap[j] - BMP_LOGO_OFFSET)];
				/* convert pixel from R4:G4:B4 to R8:G8:B8 format */
				/* so [11-8:7-4:3-0] to [23-20:15-12:7-4] */
				fb32[j] =
					((col16 & 0x000F) <<   4) |	/* B4 */
					((col16 & 0x00F0) <<   8) |	/* G4 */
					((col16 & 0x0F00) <<  12);	/* R4 */
			}
			bmap += BMP_LOGO_WIDTH;
			fb32 += panel_info.vl_col;
		}
	} else {
		debug("%s:%s.%d panel_info.vl_bpix=0x%X not recognised\n",
			__FILE__, __func__, __LINE__, panel_info.vl_bpix);
	}
		

	WATCHDOG_RESET();
}
#endif /* CONFIG_LCD_LOGO */

/*----------------------------------------------------------------------*/
#if defined(CONFIG_CMD_BMP) || defined(CONFIG_SPLASH_SCREEN)
/*
 * Display the BMP file located at address bmp_image.
 * Only uncompressed.
 */

#ifdef CONFIG_SPLASH_SCREEN_ALIGN
#define BMP_ALIGN_CENTER	0x7FFF
#endif

int lcd_display_bitmap(unsigned long bitmap_image, int x, int y)
{
#if !defined(CONFIG_MCC200)
	ushort *cmap = NULL;
#endif
	ushort *cmap_base = NULL;
	ushort colreg = 0;
	ushort i, j;
	uchar *fb;
	bmp_image_t *bmp=(bmp_image_t *)bitmap_image;
	uchar *bmap;
	ushort padded_line;
	unsigned long width, height, byte_width;
	unsigned long pwidth = panel_info.vl_col;
	unsigned colors, bpix, bmp_bpix;
	unsigned long compression;
//#if defined(CONFIG_PXA250)
	//struct pxafb_info *fbi = &panel_info.pxa;
//#elif defined(CONFIG_MPC823)
	//volatile immap_t *immr = (immap_t *) CONFIG_SYS_IMMR;
	//volatile cpm8xx_t *cp = &(immr->im_cpm);
//#endif

#ifdef CONFIG_VIDEO_BMP_GZIP
	unsigned long len;

	printf("%s:%s.%d\n", __FILE__, __func__, __LINE__);

	/* unzip if signature found */
	if (((bmp->header.signature[0]== 0x1F) &&
	     (bmp->header.signature[1]== 0x8B) &&
	     (bmp->header.signature[2]== 0x08))) {
		bmp = gunzip_bmp((ulong)bmp, &len);
	}
#endif

	if (bmp == NULL || !((bmp->header.signature[0]=='B') &&
		(bmp->header.signature[1]=='M'))) {
		printf ("Error: no valid bmp image at %lx\n", bitmap_image);
		return 1;
	}

	width = le32_to_cpu (bmp->header.width);
	height = le32_to_cpu (bmp->header.height);
	bmp_bpix = le16_to_cpu(bmp->header.bit_count);
	colors = 1 << bmp_bpix;
	compression = le32_to_cpu (bmp->header.compression);

	debug ("%s:%s.%d  width %lu  height %lu  colors %u\n",
		__FILE__, __func__, __LINE__, width, height, colors);

	bpix = NBITS(panel_info.vl_bpix);

	if ((bpix != 1) &&
		(bpix != 8) && (bpix != 16) && (bpix != 32)) {
		printf ("Error: %d bit/pixel mode, but BMP has %d bit/pixel\n",
			bpix, bmp_bpix);
		if (bmp != (bmp_image_t *)bitmap_image)
			free(bmp);	/* free gunzip memory */
		return 1;
	}

	debug ("%s:%s.%d bmp_bpix = %d, bpix = %d\n",
		__FILE__, __func__, __LINE__, bmp_bpix, bpix);

	/* We support displaying 8bpp BMPs on 16bpp and 32bpp LCDs */
	if ((bpix != bmp_bpix) &&
		(bmp_bpix != 8 || (bpix != 16 &&  bpix != 32))) {
		printf ("Error: %d bit/pixel mode, but BMP has %d bit/pixel\n",
			bpix,
			le16_to_cpu(bmp->header.bit_count));
		if (bmp != (bmp_image_t *)bitmap_image)
			free(bmp);	/* free gunzip memory */
		return 1;
	}

	debug ("Display-bmp: %d x %d  with %d colors\n",
		(int)width, (int)height, (int)colors);

//#if !defined(CONFIG_MCC200)
	///* MCC200 LCD doesn't need CMAP, supports 1bpp b&w only */
	//if ((bmp_bpix == 8) && (panel_info.cmap != NULL)) {
//#if defined(CONFIG_PXA250)
		//cmap = (ushort *)fbi->palette;
//#elif defined(CONFIG_MPC823)
		//cmap = (ushort *)&(cp->lcd_cmap[255*sizeof(ushort)]);
//#elif !defined(CONFIG_ATMEL_LCD)
		//cmap = panel_info.cmap;
//#endif

		//cmap_base = cmap;

		//printf("%s:%s.%d cmap = 0x%p, cmap_base = 0x%p\n",
			//__FILE__, __func__, __LINE__, cmap, cmap_base);

		//for (i=0; i<colors; ++i) {
			//bmp_color_table_entry_t cte = bmp->color_table[i];
			//printf("cte.red = 0x%2.2X, cte.green = 0x%2.2X, cte.blue = 0x%2.2X\n",
				//cte.red, cte.green, cte.blue);
//#if !defined(CONFIG_ATMEL_LCD)
			//if (panel_info.vl_bpix == LCD_COLOR16) {
				//colreg =
					//( ((cte.red)   << 8) & 0xf800) |
					//( ((cte.green) << 3) & 0x07e0) |
					//( ((cte.blue)  >> 3) & 0x001f) ;
			//} else if (panel_info.vl_bpix == LCD_COLOR32) {
				//colreg =
					//( ((cte.red)   << 16) & 0x00ff0000) |
					//( ((cte.green) <<  8) & 0x0000ff00) |
					//( ((cte.blue)  <<  0) & 0x000000ff) ;
			//}
//#ifdef CONFIG_SYS_INVERT_COLORS
			//*cmap = 0xffff - colreg;
//#else
			//*cmap = colreg;
//#endif
//#if defined(CONFIG_MPC823)
			//cmap--;
//#else
			//cmap++;
//#endif
//#else /* CONFIG_ATMEL_LCD */
			//lcd_setcolreg(i, cte.red, cte.green, cte.blue);
//#endif
		//}
	//}
//#endif

	/*
	 *  BMP format for Monochrome assumes that the state of a
	 * pixel is described on a per Bit basis, not per Byte.
	 *  So, in case of Monochrome BMP we should align widths
	 * on a byte boundary and convert them from Bit to Byte
	 * units.
	 *  Probably, PXA250 and MPC823 process 1bpp BMP images in
	 * their own ways, so make the converting to be MCC200
	 * specific.
	 */
//#if defined(CONFIG_MCC200)
	//if (bpix==1)
	//{
		//width = ((width + 7) & ~7) >> 3;
		//x     = ((x + 7) & ~7) >> 3;
		//pwidth= ((pwidth + 7) & ~7) >> 3;
	//}
//#endif

/* round up line width, if needed, to word boundry */
	switch(bpix) {
	case 1:
	case 8:
		padded_line = (width&0x3) ? ((width&~0x3)+4) : (width);
		byte_width = width;
		break;
	case 16:
		padded_line = (width&0x1) ? ((width&~0x1)+2) : (width);
		byte_width = width * 2;
		break;
	case 32:
		padded_line = width;
		byte_width = width * 4;
		break;
	default:
		padded_line = (width&0x3) ? ((width&~0x3)+4) : (width);
		byte_width = width;
		break;
	}

//#ifdef CONFIG_SPLASH_SCREEN_ALIGN
	//if (x == BMP_ALIGN_CENTER)
		//x = max(0, (pwidth - width) / 2);
	//else if (x < 0)
		//x = max(0, pwidth - width + x + 1);

	//if (y == BMP_ALIGN_CENTER)
		//y = max(0, (panel_info.vl_row - height) / 2);
	//else if (y < 0)
		//y = max(0, panel_info.vl_row - height + y + 1);
//#endif /* CONFIG_SPLASH_SCREEN_ALIGN */

	/* clip image width and height to screen */
	if ((x + width)>pwidth)
		width = pwidth - x;
	if ((y + height)>panel_info.vl_row)
		height = panel_info.vl_row - y;

	debug("%s:%s.%d screen_width = %lu, screen_height = %lu\n",
		__FILE__, __func__, __LINE__, width, height);

	/* point at image data in bitmap image file */
	bmap = (uchar *)bmp + le32_to_cpu (bmp->header.data_offset);

	/* point frame buffer at lower left corner of destination */
	fb   = (uchar *) (lcd_base + (y + height - 1) * lcd_line_length + x);
	//printf("%s:%s.%d  lcd_base = 0x%p, y = %d x = %d lcd_line_length = %d\n",
				//__FILE__, __func__, __LINE__, lcd_base, x, y, lcd_line_length);

	debug("%s:%s.%d fb = 0x%p, bmap = 0x%p\n",
		__FILE__, __func__, __LINE__, fb, bmap);

	switch (bmp_bpix) {
	case 1: /* pass through */
	case 8:
		/* loop for each row displayed on screen */
		for (i = 0; i < height; ++i) {
			WATCHDOG_RESET();
			/* loop for each horizontal pixel displayed on screen */
			for (j = 0; j < width; j++) {
				if (bpix == 8) {
#if defined(CONFIG_PXA250) || defined(CONFIG_ATMEL_LCD)
					*(fb++) = *(bmap++);
#elif defined(CONFIG_MPC823) || defined(CONFIG_MCC200)
					*(fb++) = 255 - *(bmap++);
#endif
				} else if (bpix == 16) {
					*(uint16_t *)fb = cmap_base[*(bmap++)];
					fb += sizeof(uint16_t) / sizeof(*fb);
				} else if (bpix == 32) { /* no colormap */
					*(uint32_t *)fb = 
					    bmp->color_table[*bmap].red  << 16 |
					    bmp->color_table[*bmap].green << 8 |
					    bmp->color_table[*bmap].blue;
					bmap++;
					fb += sizeof(uint32_t) / sizeof(*fb);
				}
			}

			/* skip to next line in bitmap */
			bmap += (padded_line - width);
			/* back up to beginning of line, then up one row on screen */
			switch(bpix) {
			case 1:
			case 8:
				fb -= (byte_width + lcd_line_length);
				break;
			case 16:
				fb -= (byte_width + lcd_line_length);
				break;
			case 32:
				fb -= (byte_width + lcd_line_length);
				break;
			default:	
				fb -= (byte_width + lcd_line_length);
				break;
			}
			debug("\n%s:%s.%d start-of-line  fb = 0x%p, bmap = 0x%p\n\n",
				__FILE__, __func__, __LINE__, fb, bmap);
		}
		break;

#if defined(CONFIG_BMP_16BPP)
	case 16:
		for (i = 0; i < height; ++i) {
			WATCHDOG_RESET();
			for (j = 0; j < width; j++) {
#if defined(CONFIG_ATMEL_LCD_BGR555)
				*(fb++) = ((bmap[0] & 0x1f) << 2) |
					(bmap[1] & 0x03);
				*(fb++) = (bmap[0] & 0xe0) |
					((bmap[1] & 0x7c) >> 2);
				bmap += 2;
#else
				*(fb++) = *(bmap++);
				*(fb++) = *(bmap++);
#endif
			}
			bmap += (padded_line - width) * 2;
			fb   -= (width * 2 + lcd_line_length);
		}
		break;
#endif /* CONFIG_BMP_16BPP */

#if defined (CONFIG_BMP_32BPP)
	case 32:
		for (i = 0; i < height; ++i) {
			WATCHDOG_RESET();
			for (j = 0; j < width; j++) {
				*(fb++) = *(bmap++);
				*(fb++) = *(bmap++);
			}
			bmap += (padded_line - width) * 4;
			fb   -= (width * 4 + lcd_line_length);
		}
		break;
#endif	/* CONFIG_BMP_32BPP */
	default:
		break;
	};

	if (bmp != (bmp_image_t *)bitmap_image)
		free(bmp);	/* free gunzip memory */
	console_new_screen();	/* reset console position */
	return (0);
}
#endif

static void *lcd_logo (void)
{
#ifdef CONFIG_SPLASH_SCREEN
	char *s;
	ulong addr;
	static int do_splash = 1;

	if (do_splash && (s = getenv("splashimage")) != NULL) {
		int x = 0, y = 0;
		do_splash = 0;

		addr = simple_strtoul (s, NULL, 16);
#ifdef CONFIG_SPLASH_SCREEN_ALIGN
		if ((s = getenv ("splashpos")) != NULL) {
			if (s[0] == 'm')
				x = BMP_ALIGN_CENTER;
			else
				x = simple_strtol (s, NULL, 0);

			if ((s = strchr (s + 1, ',')) != NULL) {
				if (s[1] == 'm')
					y = BMP_ALIGN_CENTER;
				else
					y = simple_strtol (s + 1, NULL, 0);
			}
		}
#endif /* CONFIG_SPLASH_SCREEN_ALIGN */

#ifdef CONFIG_VIDEO_BMP_GZIP
		bmp_image_t *bmp = (bmp_image_t *)addr;
		unsigned long len;

		if (!((bmp->header.signature[0]=='B') &&
		      (bmp->header.signature[1]=='M'))) {
			addr = (ulong)gunzip_bmp(addr, &len);
		}
#endif

		if (lcd_display_bitmap ((ulong)addr, x, y) == 0) {
			return ((void *)lcd_base);
		}
	}
#endif /* CONFIG_SPLASH_SCREEN */

#ifdef CONFIG_LCD_LOGO
	bitmap_plot (0, 0);
#endif /* CONFIG_LCD_LOGO */

//#ifdef CONFIG_LCD_SCREENS
	//run_screen_state(SCREEN_NORMAL);
//#endif

	debug("%s:%s.%d\n", __FILE__, __func__, __LINE__);

#ifdef CONFIG_LCD_INFO
	console_col = CONSOLE_CHAR_COLUMN_FIRST;
	console_row = CONSOLE_CHAR_ROW_FIRST;

	if (CONSOLE_CHAR_COLS <= console_col) {
		debug ("%s:%s.%d ERR: Console col %d is off screen, resetting to 0.\n",
			__FILE__, __func__, __LINE__, console_col);
		console_col = CONSOLE_CHAR_COLUMN_FIRST;
	}

	if (CONSOLE_CHAR_ROWS <= console_row) {
		debug ("%s:%s.%d ERR: Console row %d is off screen.\n",
			__FILE__, __func__, __LINE__, console_row);
		console_row =  CONSOLE_CHAR_ROWS - 1;
		debug ("    ERR: console_row reset to %d\n", console_row);
	}

#if defined(CONFIG_LCD_LOGO) && !defined(CONFIG_LCD_INFO_BELOW_LOGO)
	lcd_console_address =  ((void *)((ulong)lcd_base + BMP_LOGO_HEIGHT * lcd_line_length));
#else
	lcd_console_address = ((void *)lcd_base);

#endif /* CONFIG_LCD_LOGO && !CONFIG_LCD_INFO_BELOW_LOGO */
	// lcd_show_board_info();
#endif /* CONFIG_LCD_INFO */

#if defined(CONFIG_LCD_LOGO) && !defined(CONFIG_LCD_INFO_BELOW_LOGO)
	return ((void *)((ulong)lcd_base + BMP_LOGO_HEIGHT * lcd_line_length));
#else
	return ((void *)lcd_base);
#endif /* CONFIG_LCD_LOGO && !CONFIG_LCD_INFO_BELOW_LOGO */
}

/************************************************************************/
/************************************************************************/
