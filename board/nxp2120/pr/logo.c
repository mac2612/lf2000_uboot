//------------------------------------------------------------------------------
//	Module     : bootloader logo
//	File       : logo.c
//	Description: Draw logo image on screen
//	Author     : jhkim@nexell.co.kr
//	Export     :
//	History    :
//------------------------------------------------------------------------------
#include <config.h>
#include <common.h>

/* nexell soc headers */
#include <platform.h>

//------------------------------------------------------------------------------
#define	FLASH_OPT_BOOT_LOGO			(4)
#define BOOT_LOGO_BUFFER_OFFSET		\
	((CFG_MEM_PHY_BLOCK_BASE ? CFG_MEM_PHY_BLOCK_BASE : CFG_MEM_PHY_LINEAR_BASE) + (2<<20))		// add 2MB to frame buffer
#define BOOT_LOGO_BGCOL				0x00	// 0x88

#ifndef BITMAPINFOHEADER
typedef struct tagBITMAPINFOHEADER {
  unsigned int 		biSize;
  unsigned int 		biWidth;
  unsigned int 		biHeight;
  unsigned short 	biPlanes;
  unsigned short	biBitCount;
  unsigned int 		biCompression;
  unsigned int 		biSizeImage;
  unsigned int 		biXPelsPerMeter;
  unsigned int 		biYPelsPerMeter;
  unsigned int 		biClrUsed;
  unsigned int 		biClrImportant;
} BITMAPINFOHEADER;
#endif

#ifndef BITMAPFILEHEADER
typedef struct tagBITMAPFILEHEADER {
  unsigned short 	bfType;
  unsigned int   	bfSize;
  unsigned short 	bfReserved1;
  unsigned short 	bfReserved2;
  unsigned int 		bfOffBits;
} BITMAPFILEHEADER, *PBITMAPFILEHEADER;
#endif
#ifndef EdbgOutputDebugString
#define	EdbgOutputDebugString	printf
#endif

#ifndef	BOOT_CFG
typedef struct tagBOOT_CFG {
	unsigned int 	dwLogoLength;
} BOOT_CFG, *PBOOT_CFG;
BOOT_CFG  g_BootCfg = { 0, };
#endif

#ifndef	ReadFlash
#define	ReadFlash
#endif
//------------------------------------------------------------------------------
#define	DEBUGMSG_LOGO		0

extern BOOT_CFG  g_BootCfg;			// Boot configuration info

//------------------------------------------------------------------------------
#define	RGB888TO565(col) 	((((col>>16)&0xFF)&0xF8)<<8) | ((((col>>8)&0xFF)&0xFC)<<3) | ((((col>>0 )&0xFF)&0xF8)>>3)
void
PutPixel888To565(
		U32  Base,
		int  xpos,
		int  ypos,
		int  width,
		int  height,
		U32  Color
		)
{
//	if(xpos<width && ypos<height)
	{
		*(unsigned short*)(Base + (ypos * width + xpos) * 2) = (unsigned short)RGB888TO565(Color);
	}
}

void
PutPixel565To565(
		U32  Base,
		int  xpos,
		int  ypos,
		int  width,
		int  height,
		U32  Color
		)
{
//	if(xpos<width && ypos<height)
	{
		*(unsigned short*)(Base + (ypos * width + xpos) * 2) = (unsigned short)Color;
	}
}

void
PutPixel888To888(
		U32  Base,
		int  xpos,
		int  ypos,
		int  width,
		int  height,
		U32  Color
		)
{
//	if(xpos<width && ypos<height)
	{
		*(unsigned char*)(Base + (ypos * width + xpos) * 3 + 0) = ((Color>> 0)&0xFF);	// B
		*(unsigned char*)(Base + (ypos * width + xpos) * 3 + 1) = ((Color>> 8)&0xFF);	// G
		*(unsigned char*)(Base + (ypos * width + xpos) * 3 + 2) = ((Color>>16)&0xFF);	// R
	}
}

void
PutPixel565To888(
		U32  Base,
		int  xpos,
		int  ypos,
		int  width,
		int  height,
		U32  Color
		)
{
//	if(xpos<width && ypos<height)
	{
		*(unsigned char*)(Base + (ypos * width + xpos) * 3 + 0) = (((Color >> 0 ) << 3) & 0xf8) | (((Color >> 0 ) >> 2) & 0x7);	// B
		*(unsigned char*)(Base + (ypos * width + xpos) * 3 + 1) = (((Color >> 5 ) << 2) & 0xfc) | (((Color >> 5 ) >> 4) & 0x3);	// G
		*(unsigned char*)(Base + (ypos * width + xpos) * 3 + 2) = (((Color >> 11) << 3) & 0xf8) | (((Color >> 11) >> 2) & 0x7);	// R
	}
}

void
PutPixel888To8888(
		U32  Base,
		int  xpos,
		int  ypos,
		int  width,
		int  height,
		U32  Color
		)
{
//	if(xpos<width && ypos<height)
	{
		*(unsigned char*)(Base + (ypos * width + xpos) * 4 + 0) = ((Color>> 0)&0xFF);	// B
		*(unsigned char*)(Base + (ypos * width + xpos) * 4 + 1) = ((Color>> 8)&0xFF);	// G
		*(unsigned char*)(Base + (ypos * width + xpos) * 4 + 2) = ((Color>>16)&0xFF);	// R
		*(unsigned char*)(Base + (ypos * width + xpos) * 4 + 3) = 0;					// Alpha
	}
}

void
PutPixel565To8888(
		U32  Base,
		int  xpos,
		int  ypos,
		int  width,
		int  height,
		U32  Color
		)
{
//	if(xpos<width && ypos<height)
	{
		*(unsigned char*)(Base + (ypos * width + xpos) * 4 + 0) = (((Color >> 0 ) << 3) & 0xf8) | (((Color >> 0 ) >> 2) & 0x7);	// B
		*(unsigned char*)(Base + (ypos * width + xpos) * 4 + 1) = (((Color >> 5 ) << 2) & 0xfc) | (((Color >> 5 ) >> 4) & 0x3);	// G
		*(unsigned char*)(Base + (ypos * width + xpos) * 4 + 2) = (((Color >> 11) << 3) & 0xf8) | (((Color >> 11) >> 2) & 0x7);	// R
		*(unsigned char*)(Base + (ypos * width + xpos) * 4 + 3) = 0;	// Alpha
	}
}

void (*PUTPIXELTABLE[])(U32, int, int, int, int, U32) =
{
	PutPixel565To565,
	PutPixel565To888,
	PutPixel565To8888,
	PutPixel888To565,
	PutPixel888To888,
	PutPixel888To8888,
};

//------------------------------------------------------------------------------
static void NX_Logo(U32 FrameBase, int XResol, int YResol, U32 PixelByte);

void
nx_boot_logo(
		U32  FrameBase,
		int  XResol,
		int  YResol,
		U32  PixelByte
		)
{
	U32 			* pBMPBase = (U32*)IO_ADDRESS(BOOT_LOGO_BUFFER_OFFSET);
	PBITMAPFILEHEADER pBMPFile = (PBITMAPFILEHEADER)pBMPBase;
	BITMAPINFOHEADER  BMPInfo;
	unsigned char   * pBitMap = NULL;
	int BMPPixelByte;

	int lcdsx, lcdsy, lcdex, lcdey;
	int bmpsx, bmpsy, bmpex, bmpey;
	int lx, ly, bx, by, align=0;
	BOOL balign = FALSE;

	U8 *pPixel;
	U32 Color;

	void (*PutPixel)(U32, int, int, int, int, U32) = NULL;

	PBOOT_CFG  pBootCfg = &g_BootCfg;

	// Read boot logo from flash device.
	//
//	if(0 != pBootCfg->dwLogoLength)
//		ReadFlash(FLASH_OPT_BOOT_LOGO, CFG_NAND_BOOTLD_LOGO_START, (U32)BOOT_LOGO_BUFFER_OFFSET, pBootCfg->dwLogoLength);

	// Check logo file type.
	//
	if(pBMPFile->bfType != 0x4D42)
	{
		EdbgOutputDebugString("\nCan't find logo image at 0x%x (Type:0x%x), base:0x%x...\r\n",
			(uint)pBMPFile, (uint)pBMPFile->bfType, FrameBase);
		NX_Logo(FrameBase, XResol, YResol, PixelByte);
		return;
	}

#if (DEBUGMSG_LOGO == 1)
	EdbgOutputDebugString("\nBMP File Header Base 0x%x, Size %d \r\n",
		pBMPFile, sizeof(BITMAPFILEHEADER));
	EdbgOutputDebugString("Type	: 0x%x \r\n", pBMPFile->bfType);
	EdbgOutputDebugString("Size	: %d   \r\n", pBMPFile->bfSize);
	EdbgOutputDebugString("Offs	: %d   \r\n", pBMPFile->bfOffBits);
#endif

	// Set logo length
	//
	pBootCfg->dwLogoLength = pBMPFile->bfSize;

	// Get BMP info
	//
	memcpy((void*)&BMPInfo, ((char*)pBMPBase+sizeof(BITMAPFILEHEADER)), sizeof(BITMAPINFOHEADER));
	BMPPixelByte = BMPInfo.biBitCount/8;

#if (DEBUGMSG_LOGO == 1)
	EdbgOutputDebugString("\nBMP Info Header Base 0x%x, Size %d \r\n",
		((char*)pBMPBase+sizeof(BITMAPFILEHEADER)), sizeof(BITMAPINFOHEADER));
	EdbgOutputDebugString("Size		: %d\r\n", BMPInfo.biSize);
	EdbgOutputDebugString("Width	: %d\r\n", BMPInfo.biWidth);
	EdbgOutputDebugString("Height	: %d\r\n", BMPInfo.biHeight);
	EdbgOutputDebugString("Planes	: %d\r\n", BMPInfo.biPlanes);
	EdbgOutputDebugString("BitCount	: %d\r\n", BMPInfo.biBitCount);
	EdbgOutputDebugString("Compress	: %d\r\n", BMPInfo.biCompression);
	EdbgOutputDebugString("SizeImage: %d\r\n", BMPInfo.biSizeImage);
	EdbgOutputDebugString("XPels	: %d\r\n", BMPInfo.biXPelsPerMeter);
	EdbgOutputDebugString("YPels	: %d\r\n", BMPInfo.biYPelsPerMeter);
	EdbgOutputDebugString("ClrUsed	: %d\r\n", BMPInfo.biClrUsed);
	EdbgOutputDebugString("ClrImport: %d\r\n", BMPInfo.biClrImportant);
	EdbgOutputDebugString("\r\n");
#endif
	// Select put pixel function
	//
	switch(BMPPixelByte)
	{
	case 2:	PutPixel = PUTPIXELTABLE[0 + PixelByte-2];	break;	// 565 To 565/888
	case 3: PutPixel = PUTPIXELTABLE[3 + PixelByte-2];	break;	// 888 To 565/888
	default:
		EdbgOutputDebugString("\nNot support BitPerPixel (%d) ...\r\n", BMPPixelByte);
		NX_Logo(FrameBase, XResol, YResol, PixelByte);
		return;
	}

	// Clear frame buffer
	//
#if 0
	for(ly=0; ly<YResol; ly++)
	for(lx=0; lx<XResol; lx++)
	{
		Color = (U32)(BOOT_LOGO_BGCOL<<16 | BOOT_LOGO_BGCOL<<8 | BOOT_LOGO_BGCOL);	// RGB888
		PutPixel(FrameBase, lx, ly, XResol, YResol, Color);
	}
#else
	memset((void*)FrameBase, BOOT_LOGO_BGCOL, (XResol*YResol*PixelByte));
#endif

	lcdsx = 0, lcdsy = 0, lcdex = XResol, lcdey = YResol;
	bmpsx = 0, bmpsy = 0, bmpex = BMPInfo.biWidth, bmpey = BMPInfo.biHeight;
	pBitMap = (unsigned char*)pBMPBase + pBMPFile->bfSize;	// BMP file end point.

	if((bmpex * bmpey)%BMPPixelByte)
		balign = TRUE;

	EdbgOutputDebugString("DONE: Logo BMP %d by %d (%dBPP), Len=%d \r\n",
		BMPInfo.biWidth, BMPInfo.biHeight, BMPPixelByte, pBootCfg->dwLogoLength);

	if(BMPInfo.biWidth  > XResol)
	{
		bmpsx = (BMPInfo.biWidth - XResol)/2;
		bmpex = bmpsx + XResol;
	}
	else if(BMPInfo.biWidth  < XResol)
	{
		lcdsx += (XResol- BMPInfo.biWidth)/2;
		lcdex  = lcdsx + BMPInfo.biWidth;
	}

	if(BMPInfo.biHeight > YResol)
	{
		bmpsy = (BMPInfo.biHeight - YResol)/2;
		bmpey = bmpsy + YResol;
	}
	else if(BMPInfo.biHeight < YResol)
	{
		lcdsy += (YResol- BMPInfo.biHeight)/2;
		lcdey  = lcdsy + BMPInfo.biHeight;
	}

	// Draw 16 BitperPixel image on the frame buffer base.
	//
	if(BMPPixelByte == 2)
	{
		for(ly = lcdsy, by = bmpsy; by<bmpey; ly++, by++)
		{
			for(lx = lcdsx, bx = bmpex; bx>bmpsx; lx++, bx--)
			{
#if 0
			Color = *(U16*)(pBitMap - (by * BMPInfo.biWidth+ bx) * BMPPixelByte);	// Occur Data abort
#else
			Color = *(U16*)(pBitMap - (by * BMPInfo.biWidth+ bx) * BMPPixelByte);
#endif
			PutPixel(FrameBase, lx, ly, XResol, YResol, Color);
			}
		}
	}

	// Draw 24 BitperPixel image on the frame buffer base.
	//
	if(BMPPixelByte == 3)
	{
		for(ly = lcdsy, by = bmpsy; by<bmpey; ly++, by++)
		{
			for(lx = lcdsx, bx = bmpex; bx>bmpsx; lx++, bx--)
			{
			pPixel = (U8*)(pBitMap - (by * BMPInfo.biWidth + bx + align) * BMPPixelByte);
			Color  = (U32)(*(pPixel+2)<<16 | *(pPixel+1)<<8 | *(pPixel));
			PutPixel(FrameBase, lx, ly, XResol, YResol, Color);
			}

			if(balign && by%3)
				align++;
		}
	}
}

// Draw gratation color bar
//
static void
NX_Logo(
		U32 FrameBase,
		int XResol,
		int YResol,
		U32 PixelByte
		)
{
	void (*PutPixel)(U32, int, int, int, int, U32) = NULL;

	int sx, sy, ex, ey, x, y, slope = 0;
	int pxl, num, col, div, dep, dec;
	U8  R0, G0, B0, R, G, B;
	U32 RGB;

	col = 8;		// colorbar count.
	dep = 256;		// gratation depth.

	div = (YResol/col);
	dec = 256/dep;

	sx  = (XResol%dep)/2;
	ex  = (XResol/dep)*dep + sx;

	sy  = (YResol%col)/2;
	ey  = (YResol/col)*col + sy;

	pxl = (XResol/dep);

	// 888To565 or 888To888
	PutPixel = PUTPIXELTABLE[3 + PixelByte - 2];

	// Clear frame buffer
	//
#if 0
	for(y=0; y<YResol; y++)
	for(x=0; x<XResol; x++)
	{
		RGB = (U32)(BOOT_LOGO_BGCOL<<16 | BOOT_LOGO_BGCOL<<8 | BOOT_LOGO_BGCOL);	// RGB888
		PutPixel(FrameBase, x, y, XResol, YResol, RGB);
	}
#else
	memset((void*)FrameBase, BOOT_LOGO_BGCOL, (XResol*YResol*PixelByte));
#endif

	for(y=sy; y<ey; y++)
	{
		switch(y/div)
		{
		case 0:	R = 0xFF, G = 0xFF, B = 0xFF; break;	// White
		case 1:	R = 0xFF, G = 0x00, B = 0x00; break;	// Red
		case 2:	R = 0x00, G = 0xFF, B = 0x00; break;	// Green
		case 3:	R = 0x00, G = 0x00, B = 0xFF; break;	// Blue
		case 4:	R = 0xFF, G = 0xFF, B = 0x00; break;	// RG
		case 5:	R = 0x00, G = 0xFF, B = 0xFF; break;	// GB
		case 6:	R = 0xFF, G = 0x00, B = 0xFF; break;	// RB
		case 7:	R = 0x00, G = 0x00, B = 0x00; break;	// Black
		default:
			return;
		}

		// Separate line
		//
		if(0 == y%div || 1 == y%div)
		{
			for(x=0 ; x<XResol; x++)
			{
			RGB = (U32)(BOOT_LOGO_BGCOL<<16 | BOOT_LOGO_BGCOL<<8 | BOOT_LOGO_BGCOL);	// RGB888
			PutPixel(FrameBase, x, y+slope, XResol, YResol, RGB);
			}
			continue;
		}

		R0  = R;
		G0  = G;
		B0  = B;
		num = pxl;

		slope = 0;

		// Gratation color bar
		//
		for(x=sx ; x<ex; x++, num--)
		{
			if(0 == num)
			{
				if(R0==0xFF) R -= dec;
				if(G0==0xFF) G -= dec;
				if(B0==0xFF) B -= dec;
				num = pxl;

#if	0	// slope
				slope++;
#endif
			}

			RGB = (U32)(R<<16 | G<<8 | B);	// RGB888
			PutPixel(FrameBase, x, y+slope, XResol, YResol, RGB);
		}
	}
}


//------------------------------------------------------------------------------
// Progress Bar
//
//------------------------------------------------------------------------------
#ifdef BOOT_CFG_PROGRESS_BAR

static U32 ProgHead[8][8] =
{
	0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF,
	0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0x52586B,
	0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0x254889, 0x0039A1,
	0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0x577799, 0x0056AF, 0x0056AF,
	0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0x3E80AE, 0x006EBA, 0x006EBA,
	0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0x6BA7C8, 0x007EC2, 0x007EC2,
	0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xCCD3D9, 0x1C77BC, 0x0069B8,
	0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xD3D3DB, 0x7577B7
};

static U32 ProgTail[8][8] =
{
	0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF,
	0x52586B, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF,
	0x0039A1, 0x254889, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF,
	0x0056AF, 0x0056AF, 0x3D5D7F, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF,
	0x006EBA, 0x006EBA, 0x276997, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF,
	0x007EC2, 0x007EC2, 0x337090, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF,
	0x0069B8, 0x1570B5, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF,
	0x7E80BF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF, 0xFF00FF
};

static U32 ProgBody[] =
{
	0x006164B3,
	0x000069B8,
	0x00007EC2,
	0x00006EBA,
	0x000056AF,
	0x000039A1,
	0x003B4977,
	0x00939393
};

#define BAR_POS_SX			20
#define BAR_LENGTH			(CFG_DISP_PRI_RESOL_WIDTH - (2*BAR_POS_SX))
#define BAR_POS_EX			(BAR_POS_SX + BAR_LENGTH)
#define BAR_EDGE_X			8
#define BAR_EDGE_Y			8

static struct
{
	U32	FrameBase;
	int	LcdWidth;
	int	LcdHeight;
	int	PixelByte;

} g_ScreenInfo =
{
	(CFG_MEM_PHY_BLOCK_BASE ? CFG_MEM_PHY_BLOCK_BASE : CFG_MEM_PHY_LINEAR_BASE)
		 + 0x20000000,	// Non cache zone.
	CFG_DISP_PRI_RESOL_WIDTH,
	CFG_DISP_PRI_RESOL_HEIGHT,
	CFG_DISP_SCREEN_PIXEL_BYTE,
};

static BOOL g_bProgInit  = FALSE;
static BOOL g_bProgHead  = FALSE;
static BOOL g_bProgTail  = FALSE;

static int	g_ProgPixel  = 0;
static int	g_ProgLength = 0;
static int	g_ProgCount  = 0;
static int	g_ProgPoint  = 0;

void
InitProgressBar(
		U32  Length,
		U32  Unit
		)
{
	U32 ObjLength = 0;

	if(0 == Length || 0 == Unit)
		return;

	ObjLength = (Length/Unit);

	if(ObjLength > BAR_LENGTH)
	{
 		g_ProgLength = (ObjLength / BAR_LENGTH);
 		g_ProgPixel  = 1;
 	}
 	else
 	{
 		g_ProgLength = 1;
 		g_ProgPixel  = (BAR_LENGTH / ObjLength);
 	}

	g_ProgCount = 0;
	g_bProgInit = TRUE;

	EdbgOutputDebugString("DONE: Progress Bar Image Len=%d (Unit:%d)\n", Length, Unit);
}

void
ShowProgressBar(void)
{
	int x = 0, y = 0;
	int sx  = BAR_POS_SX + g_ProgPoint;
	int sy  = (g_ScreenInfo.LcdHeight/10)*9;
	int wx  = g_ScreenInfo.LcdWidth;
	int wy  = g_ScreenInfo.LcdHeight;
	int bpp = g_ScreenInfo.PixelByte;
	U32 fb  = IO_ADDRESS(g_ScreenInfo.FrameBase);
	int ex  = 0;

	void (*PutPixel)(U32, int, int, int, int, U32) = PUTPIXELTABLE[2 + bpp - 2];

	if(FALSE == g_bProgInit)
		return;

	// Complete progress bar
	//
	if(TRUE == g_bProgTail)
		return;

	// Once draw head bar
	//
	if(FALSE == g_bProgHead)
	{
		for(x=0; x<BAR_EDGE_X; x++)
		{
			for(y=0; y<BAR_EDGE_Y; y++)
			{
				if(ProgHead[BAR_EDGE_Y-y-1][x] != 0xFF00FF)
				{
					PutPixel(fb, (BAR_POS_SX + x - BAR_EDGE_X + 1), (sy + y + 1), wx, wy, ProgHead[BAR_EDGE_Y-y-1][x]);
				}
			}
		}

		g_bProgHead = TRUE;
	}

	g_ProgCount++;

	if(g_ProgCount != g_ProgLength)
		return;

	g_ProgCount = 0;

	// Get End position
	ex = sx + g_ProgPixel;
	if(ex > BAR_POS_EX)
		ex = BAR_POS_EX;

 	// Next start position
	g_ProgPoint += g_ProgPixel;

	// Draw pregress bar
	//
	for(x=sx; x<ex; x++)
	{
		for(y=0; y<BAR_EDGE_Y; y++)
		{
			PutPixel(fb, x, (sy + y + 1), wx, wy, ProgBody[y]);
		}
	}

	// Check tail bar
	if(ex < BAR_POS_EX)
		return;

	// Tail start position
	sx = ex;

	// Once draw tail bar
	//
	if(FALSE == g_bProgTail)
	{
		for(x=0; x<BAR_EDGE_X; x++)
		{
			for(y=0; y<BAR_EDGE_Y; y++)
			{
				if(ProgTail[BAR_EDGE_Y-y-1][x]!=0xFF00FF)
				{
					PutPixel(fb, (sx + x + 1), (sy + y + 1), wx, wy, ProgTail[BAR_EDGE_Y-y-1][x]);
				}
			}
		}

		g_bProgTail = TRUE;
	}
}

#endif	// BOOT_CFG_PROGRESS_BAR
