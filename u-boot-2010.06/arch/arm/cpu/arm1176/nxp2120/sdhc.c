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
#include <config.h>
#include <malloc.h>
#include <part.h>
#include <mmc.h>
#include <asm/errno.h>

#include <platform.h>

/*----------------------------------------------------------------------------
 * Debug
 *----------------------------------------------------------------------------*/
#if (0)
#define DBGOUT(msg...)		{ printf("sdhc: " msg); }
#else
#define DBGOUT(msg...)
#endif

#define ERROUT(msg...)		{ printf("sdhc: line=%d ", __LINE__); printf(msg); }

#define	DEBUG_RESP			(1)
#define	DEBUG_DATA			(1)

#define SD_ASSERT(expr)	{										\
	if (! expr) {                                  				\
		printf("%s, %d = %s \r\n", __func__, __LINE__, #expr);	\
	}															\
}

/*----------------------------------------------------------------------------
 * Error
 *----------------------------------------------------------------------------*/
#define	NX_SDCARD_STATUS_NOERROR		0
#define	NX_SDCARD_STATUS_ERROR			(1UL<<31)
#define	NX_SDCARD_STATUS_CMDBUSY		(NX_SDCARD_STATUS_ERROR | (1UL<<0))
#define	NX_SDCARD_STATUS_CMDTOUT		(NX_SDCARD_STATUS_ERROR | (1UL<<1))
#define	NX_SDCARD_STATUS_RESCRCFAIL		(NX_SDCARD_STATUS_ERROR | (1UL<<2))
#define	NX_SDCARD_STATUS_RESERROR		(NX_SDCARD_STATUS_ERROR | (1UL<<3))
#define	NX_SDCARD_STATUS_RESTOUT		(NX_SDCARD_STATUS_ERROR | (1UL<<4))
#define NX_SDCARD_STATUS_UNKNOWNCMD		(NX_SDCARD_STATUS_ERROR | (1UL<<5))
#define NX_SDCARD_STATUS_NOCARD			(1UL<<6)

/*----------------------------------------------------------------------------
 * Command
 *----------------------------------------------------------------------------*/
#define GO_IDLE_STATE					(0)
#define ALL_SEND_CID					(2)
#define SEND_RELATIVE_ADDR				(3)
#define IO_SEND_OP_COND					(5)
#define SELECT_CARD						(7)
#define SEND_IF_COND					(8)
#define SEND_CSD						(9)
#define SEND_CID						(10)
#define STOP_TRANSMISSION				(12)
#define SEND_STATUS						(13)
#define	SWITCH_FUNC						(6)
#define SET_BLOCKLEN					(16)
#define READ_SINGLE_BLOCK				(17)
#define READ_MULTIPLE_BLOCK				(18)
#define WRITE_BLOCK						(24)
#define WRITE_MULTIPLE_BLOCK			(25)

#define SET_BUS_WIDTH					(6)		// APP_CMD
#define SD_STATUS						(13)	// APP_CMD
#define	SD_SEND_OP_COND					(41) 	// APP_CMD
#define	SET_CLR_CARD_DETECT				(42) 	// APP_CMD
#define SEND_SCR						(51) 	// APP_CMD

#define	MMC_SEND_OP_COND				(1)		// MMC_CMD
#define	MMC_SEND_EXT_CSD				(8)		// MMC_CMD
#define	MMC_SWITCH						(6)		// MMC_CMD
#define	MMC_SEND_BUSTEST_R				(14)	// MMC_CMD
#define	MMC_SEND_BUSTEST_W				(19)	// MMC_CMD

/*----------------------------------------------------------------------------
 * Data/Response
 *----------------------------------------------------------------------------*/
#define	SDHC_DATA_NONE					(0)
#define	SDHC_DATA_READ					(1)
#define	SDHC_DATA_WRITE					(2)

#define SDHC_RESP_NONE    				(0x00)
#define SDHC_RESP_R1      				(0x01)
#define SDHC_RESP_R1b 	 				(0x1b)
#define SDHC_RESP_R2      				(0x02)
#define SDHC_RESP_R3      				(0x03)
#define SDHC_RESP_R4      				(0x04)
#define SDHC_RESP_R5      				(0x05)
#define SDHC_RESP_R6      				(0x06)
#define SDHC_RESP_R7      				(0x07)

/*----------------------------------------------------------------------------
 * Config
 *----------------------------------------------------------------------------*/
#define SDHC_BLOCK_LENGTH				(512)
#define SDHC_CLOCK_MAX					(50000000)			/*  50 MHZ */
#define SDHC_CLOCK_MIN					(400000)			/* 400 KHZ */
#define	CFG_SDHC_CLK_SRC				NX_SDHC_CLKSRC_PLL1 /* NX_SDHC_CLKSRC_PLL0, NX_SDHC_CLKSRC_PLL1, NX_SDHC_CLKSRC_PCLK */
#define	MAX_WAIT_TIME					(50000)

/*----------------------------------------------------------------------------*/
struct sdhc {
	unsigned int clk_max;
	unsigned int clk_min;
	unsigned int clk_cur;
	int			 bus_width;		/* 1 or 4 */
};

static struct sdhc _sdhc;

#ifndef msleep
#define	msleep(_n)		udelay(_n*1000)
#endif	/* msleep */

/*----------------------------------------------------------------------------*/
#if 0
static void	dump_register(int index)
{
	struct  NX_SDHC_RegisterSet *reg;
	unsigned int temp, temp1, temp2;
	int	i;

	reg = (struct NX_SDHC_RegisterSet *)NX_SDHC_GetBaseAddress(index);

	static const char cmd_fsm_str[16][21] =
	{
		"Idle",                 	"Send init",       	"Tx cmd start bit",  	"Tx cmd tx",
		"Tx cmd index+arg",     	"Tx cmd crc7",     	"Tx cmd end bit",    	"Rx resp start bit",
		"Rx resp IRQ response", 	"Rx resp tx bit",  	"Rx resp cmd idx",   	"Rx resp data",
		"Rx resp crc7",         	"Rx resp end bit", 	"Cmd path with NCC", 	"Wait"
	};

	printf("Base address = %08Xh\n", (unsigned int)reg);

	temp = reg->CTRL;
	printf("CTRL    = %08Xh\n", temp);
	printf("-> [ 8] ABORT_RDATA   = %d\n", (temp >> 8) & 1);
	printf("-> [ 7] SEND_IRQ_RESP = %d\n", (temp >> 7) & 1);
	printf("-> [ 6] READ_WAIT     = %d\n", (temp >> 6) & 1);
	printf("-> [ 5] DMA_ENA       = %d\n", (temp >> 5) & 1);
	printf("-> [ 4] INT_ENA       = %d\n", (temp >> 4) & 1);
	printf("-> [ 2] DMARST        = %d\n", (temp >> 2) & 1);
	printf("-> [ 1] FIFORST       = %d\n", (temp >> 1) & 1);
	printf("-> [ 0] CTRLRST       = %d\n", (temp >> 0) & 1);

	temp = reg->CLKDIV;
	printf("CLKDIV  = %08Xh (%d)\n", temp, temp);

	temp = reg->CLKENA;
	printf("CLKENA  = %08Xh\n", temp);
	printf("-> [16] LOWPWR        = %d\n", (temp >>16) & 1);
	printf("-> [ 0] CLKENA        = %d\n", (temp >> 0) & 1);

	temp = reg->TMOUT;
	printf("TMOUT   = %08Xh\n", temp);
	printf("-> [31: 8] DTMOUT      = %d\n", (temp >> 8));
	printf("-> [ 7: 0] RSPTMOUT    = %d\n", (temp >> 0) & 0xFF);

	temp = reg->CTYPE;
	printf("CTYPE   = %08Xh\n", temp);
	printf("-> Bus width = %d\n", (temp & (1<<16)) ? 8 : ((temp&1) ? 4 : 1));

	temp = reg->BLKSIZ;
	printf("BLKSIZ  = %08Xh (%d)\n", temp, temp & 0xFFFF);

	temp = reg->BYTCNT;
	printf("BYTCNT  = %08Xh (%d)\n", temp, temp);

	temp 	= reg->INTMASK;
	temp1 	= reg->MINTSTS;
	temp2	= reg->RINTSTS;
	printf("INTMASK = %08Xh\n", temp  );
	printf("MINTSTS = %08Xh\n", temp1 );
	printf("RINTSTS = %08Xh\n", temp2 );
	printf("          1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16 \n");
	printf("         RE   CD   DTO TXDR RXDR RCRC DCRC  RTO DRTO  HTO FRUN  HLE  SBE  ACD  EBE SDIO\n");
	printf("---------------------------------------------------------------------------------------\n");
	printf("Enb : ");
	for( i=1 ; i<=16 ; i++ )	printf("    %d", (temp >>i) & 1);
	printf("\n");
	printf("Raw : ");
	for( i=1 ; i<=16 ; i++ )	printf("    %d", (temp2>>i) & 1);
	printf("\n");
	printf("Msk : ");
	for( i=1 ; i<=16 ; i++ )	printf("    %d", (temp1>>i) & 1);
	printf("\n");

	printf("CMDARG  = %08Xh\n", reg->CMDARG);

	temp = reg->CMD;
	printf("CMD     = %08Xh\n", temp);
	printf("-> [31]    STARTCMD       = %d\n",  (temp >> 31) & 1 );
	printf("-> [21]    UPDATECLKONLY  = %d\n",  (temp >> 21) & 1 );
	printf("-> [20:16] CARDNUM        = %d\n",  (temp >> 16) & 0x1F);
	printf("-> [15]    SENDINIT       = %d\n",  (temp >> 15) & 1 );
	printf("-> [14]    STOPABORT      = %d\n",  (temp >> 14) & 1 );
	printf("-> [13]    WAITPRVDAT     = %d\n",  (temp >> 13) & 1 );
	printf("-> [12]    SENDAUTOSTOP   = %d\n",  (temp >> 12) & 1 );
	printf("-> [11]    TRMODE         = %s\n", ((temp >> 11) & 1) ? "Stream": "Block");
	printf("-> [10]    RW             = %s\n", ((temp >> 10) & 1) ? "Write" : "Read");
	printf("-> [ 9]    DATEXP         = %s\n", ((temp >>  9) & 1) ? "Data transfer" : "No data transfer");
	printf("-> [ 8]    CHKRSPCRC      = %d\n",  (temp >>  8) & 1);
	printf("-> [ 7]    RSPLEN         = %s\n", ((temp >>  7) & 1) ? "Long response" : "Short response");
	printf("-> [ 6]    RSPEXP         = %s\n", ((temp >>  6) & 1) ? "Response expected": "No response");
	printf("-> [ 5: 0] CMDINDEX       = %d\n",  (temp >>  0) & 0x3F);

	printf("RESP0   = %08Xh\n", reg->RESP0);
	printf("RESP1   = %08Xh\n", reg->RESP1);
	printf("RESP2   = %08Xh\n", reg->RESP2);
	printf("RESP3   = %08Xh\n", reg->RESP3);

	temp = 	reg->STATUS;
	printf("STATUS  = %08Xh\n", temp);
	printf("-> [31]    DMAREQ         = %d\n",  (temp >> 31) & 1 );
	printf("-> [30]    DMAACK         = %d\n",  (temp >> 30) & 1 );
	printf("-> [29:17] FIFOCOUNT      = %d\n",  (temp >> 17) & 0x1FFF);
	printf("-> [16:11] RSPINDEX       = %d\n",  (temp >> 11) & 0x3F);
	printf("-> [10]    FSMBUSY        = %d\n",  (temp >> 10) & 1 );
	printf("-> [ 9]    DATBUSY (~D[0])= %d\n",  (temp >>  9) & 1 );
	printf("-> [ 8]    CPRESENT( D[3])= %d\n",  (temp >>  8) & 1 );
	printf("-> [ 7: 4] CMDFSM         = %d (%s)\n",  (temp >>  4) & 0xF, cmd_fsm_str[(temp >>  4) & 0xF]);
	printf("-> [ 3]    FIFOFULL       = %d\n",  (temp >>  3) & 1 );
	printf("-> [ 2]    FIFOEMPTY      = %d\n",  (temp >>  2) & 1 );
	printf("-> [ 1]    TXWMARK        = %d\n",  (temp >>  1) & 1 );
	printf("-> [ 0]    RXWMARK        = %d\n",  (temp >>  0) & 1 );

	temp = reg->FIFOTH;
	printf("FIFOTH  = %08Xh\n", temp);
	printf("-> [19:16] RXTH           = %d\n",  (temp >> 16) & 0xF);
	printf("-> [ 3: 0] TXTH           = %d\n",  (temp >>  0) & 0xF);

	temp = reg->TCBCNT;
	printf("TCBCNT  = %08Xh (%d)\n", temp, temp);

	temp = reg->TBBCNT;
	printf("TBBCNT  = %08Xh (%d)\n", temp, temp);

	//printf("DATA    = %08Xh\n", reg->DATA   );

	temp = reg->CLKENB;
	printf("CLKENB  = %08Xh\n", temp);
	printf("-> [ 3]    PCLKMODE       = %s\n",  ((temp >>  3) & 1) ? "Enable" : "Disable");
	printf("-> [ 2]    CLKGENENB      = %s\n",  ((temp >>  2) & 1) ? "Enable" : "Disable");

	temp = reg->CLKGEN;
	printf("CLKGEN  = %08Xh\n", temp);
	printf("-> [ 9: 4] CLKDIV         = %d\n",   (temp >>  4) & 0x3F);
	printf("-> [ 3: 1] CLKSRCSEL      = %d\n",   (temp >>  1) & 0x7 );
};
#endif

static int check_disk(int index)
{
	/*
	 * if pad is low, Disk is attached,
	 * else is detached.
	 */
	int det = NX_GPIO_GetInputValue(CFG_PIO_SDHC_0_DETECT >> 5, CFG_PIO_SDHC_0_DETECT & 0x1F);

	return det ? NX_SDCARD_STATUS_NOCARD : NX_SDCARD_STATUS_NOERROR;
}

static void set_bus_w(int index, int bus_w)
{
	struct sdhc *host = &_sdhc;
	DBGOUT("Bus width : %d\n", bus_w);

	if (host->bus_width != bus_w) {
		NX_SDHC_SetDataBusWidth(index, bus_w);
		msleep(10);
	}
	host->bus_width = bus_w;
}

static void set_clock(int index, unsigned int clock)
{
	struct sdhc *host = &_sdhc;
	unsigned int f_clk = 0;
	int c_div  = 0;

	DBGOUT("Bus clock : %d\n", clock);

	if (! clock)
		return;

	c_div  = host->clk_max / clock;
	c_div += host->clk_max % clock ? 1 : 0;
	f_clk  = host->clk_max / (c_div);

	if (host->clk_cur != f_clk) {

		// 1. Confirm that no card is engaged in any transaction.
		//	If there's a transaction, wait until it finishes.
		while (NX_SDHC_IsDataTransferBusy(index));

		// 2. Disable the output clock.
		NX_SDHC_SetOutputClockEnable(index, CFALSE);

		// 3. Program the clock divider as required.
		NX_SDHC_SetOutputClockDivider(index, c_div);

		// 4. Start a command with NX_SDHC_CMDFLAG_UPDATECLKONLY flag.
	repeat_4 :
		NX_SDHC_SetCommand(index, 0,
			NX_SDHC_CMDFLAG_STARTCMD | NX_SDHC_CMDFLAG_UPDATECLKONLY | NX_SDHC_CMDFLAG_WAITPRVDAT);

		// 5. Wait until a update clock command is taken by the SDHC module.
		//	If a HLE is occurred, repeat 4.
		while(NX_SDHC_IsCommandBusy(index))	{ ; }

		if (NX_SDHC_GetInterruptPending(index, NX_SDHC_INT_HLE)) {
			NX_SDHC_ClearInterruptPending(index, NX_SDHC_INT_HLE);
			goto repeat_4;
		}

		// 6. Enable the output clock.
		NX_SDHC_SetOutputClockEnable(index, CTRUE);

		// 7. Start a command with NX_SDHC_CMDFLAG_UPDATECLKONLY flag.
	repeat_7 :
		NX_SDHC_SetCommand(index, 0,
			NX_SDHC_CMDFLAG_STARTCMD | NX_SDHC_CMDFLAG_UPDATECLKONLY | NX_SDHC_CMDFLAG_WAITPRVDAT);

		// 8. Wait until a update clock command is taken by the SDHC module.
		//	If a HLE is occurred, repeat 7.
		while (NX_SDHC_IsCommandBusy(index)) { ; }

		if (NX_SDHC_GetInterruptPending(index, NX_SDHC_INT_HLE)) {
			NX_SDHC_ClearInterruptPending(index, NX_SDHC_INT_HLE);
			goto repeat_7;
		}
		msleep(10);
	}

	host->clk_cur = f_clk;
	DBGOUT("Bus clock : %d (%d)\n", clock, host->clk_cur);
}

static int wait_cmd(int index)
{
	volatile int count = 0;

	/* Check unmasked interrupt status (MINTSTS) */
	while (CFALSE == NX_SDHC_GetInterruptPendingAll(index) &&
		   MAX_WAIT_TIME > count++) { ; }

	return count >= MAX_WAIT_TIME ? -1 : 0;
}

static int wait_data(int index)
{
	volatile int count = 0;
	
	/* Check raw interrupt status (RINTSTS) */
	while (CFALSE == NX_SDHC_GetInterruptPending32(index) &&
		   MAX_WAIT_TIME > count++) { ; }

	NX_SDHC_ClearInterruptPending(index, NX_SDHC_INT_SDIO);

	return count >= MAX_WAIT_TIME ? -1 : 0;
}

static int wait_busy(int index)
{
	volatile int count = 0;

	while (CTRUE == NX_SDHC_IsCardDataBusy(index) &&
		   MAX_WAIT_TIME > count++) { ; }

	count = 0;

	while (CTRUE == NX_SDHC_IsDataTransferBusy(index) &&
		   MAX_WAIT_TIME > count++) { ; }

	return count >= MAX_WAIT_TIME ? -1 : 0;
}

static void setup_hc(int index, int datalen)
{
	wait_busy(index);

	NX_SDHC_ResetFIFO(index);               	// Reset the FIFO.
	while (NX_SDHC_IsResetFIFO(index)) { ; }   	// Wait until the FIFO reset is completed.

	/*	For next data tranfer */
	if (datalen) {
		NX_SDHC_SetFIFORxThreshold(index, (1-1));
   		NX_SDHC_SetDMAMode(index, CFALSE);
   		NX_SDHC_SetBlockSize(index, datalen > SDHC_BLOCK_LENGTH ? SDHC_BLOCK_LENGTH : datalen);
		NX_SDHC_SetByteCount(index, datalen);
		NX_SDHC_SetInterruptEnable(index, NX_SDHC_INT_SDIO, CTRUE);
	}

	NX_SDHC_ClearInterruptPendingAll(index);
}

static int send_cmd(int index, unsigned int cmd, unsigned int argc,
					unsigned int resptype, unsigned int *response, unsigned int datatype)
{
	unsigned int flags  = 0;
	unsigned int result = 0;
	unsigned int status = 0;
	int  retry = 4;

	DBGOUT("%s, mmc%d, cmd=%02d, argc=0x%08x, resp type=%02x, data type=%02x\n",
		__func__, index, (cmd & 0xFF), argc, resptype, datatype);

	/* Check card status. */
	if (NX_SDCARD_STATUS_NOCARD == check_disk(index)) {
		ERROUT("Error, mmc%d no disk !!!\n", index);
		return NX_SDCARD_STATUS_NOCARD;
	}

	switch(resptype) {
	case SDHC_RESP_NONE:
		flags = (NX_SDHC_CMDFLAG_STARTCMD | NX_SDHC_CMDFLAG_SENDINIT | NX_SDHC_CMDFLAG_WAITPRVDAT);
		break;

	case SDHC_RESP_R1:
	case SDHC_RESP_R1b:
		flags |= (NX_SDHC_CMDFLAG_STARTCMD | NX_SDHC_CMDFLAG_WAITPRVDAT | NX_SDHC_CMDFLAG_CHKRSPCRC
			  | NX_SDHC_CMDFLAG_SHORTRSP);
		if (cmd == SEND_STATUS)					// R1
			flags &= (~NX_SDHC_CMDFLAG_WAITPRVDAT);
		else if (cmd == STOP_TRANSMISSION)		// R1b
			flags |= (NX_SDHC_CMDFLAG_STOPABORT);
		break;

	case SDHC_RESP_R2:
		flags |= (NX_SDHC_CMDFLAG_STARTCMD | NX_SDHC_CMDFLAG_WAITPRVDAT | NX_SDHC_CMDFLAG_CHKRSPCRC
			  | NX_SDHC_CMDFLAG_LONGRSP);
		break;

	case SDHC_RESP_R3:
	case SDHC_RESP_R4:
			flags |= (NX_SDHC_CMDFLAG_STARTCMD | NX_SDHC_CMDFLAG_WAITPRVDAT | NX_SDHC_CMDFLAG_SHORTRSP);
			if (cmd == IO_SEND_OP_COND)
				flags |= (NX_SDHC_CMDFLAG_SENDINIT);
		break;

	case SDHC_RESP_R5:
	case SDHC_RESP_R6:
	case SDHC_RESP_R7:
		flags |= (NX_SDHC_CMDFLAG_STARTCMD | NX_SDHC_CMDFLAG_WAITPRVDAT | NX_SDHC_CMDFLAG_CHKRSPCRC
			  | NX_SDHC_CMDFLAG_SHORTRSP);
		break;

	default:
		ERROUT("Error, invalid response type (0x%x) !!!\n", resptype);
		result |= NX_SDCARD_STATUS_UNKNOWNCMD;
		break;
	}

	switch (datatype) {
	case SDHC_DATA_NONE:
			break;
	case SDHC_DATA_READ:
			flags |= (NX_SDHC_CMDFLAG_BLOCK | NX_SDHC_CMDFLAG_RXDATA);
			if (cmd == READ_MULTIPLE_BLOCK)
				flags |= (NX_SDHC_CMDFLAG_SENDAUTOSTOP);
			break;
	case SDHC_DATA_WRITE:
			flags |= (NX_SDHC_CMDFLAG_BLOCK | NX_SDHC_CMDFLAG_TXDATA);
			if (cmd == WRITE_MULTIPLE_BLOCK)
				flags |= (NX_SDHC_CMDFLAG_SENDAUTOSTOP);
			break;
	default:
		ERROUT("Error, invalid data type (0x%x) !!!\n", datatype);
		result |= NX_SDCARD_STATUS_UNKNOWNCMD;
		break;
	}

	if (result != NX_SDCARD_STATUS_NOERROR)
		goto __exit_cmd;

	DBGOUT("CMD=%02d, ARG=0x%08x, FLAGS=0x%08x\n", (cmd & 0xFF), argc, flags);

	/*
	 * Send cmd and Get response.
	 */
	while (retry--) {
		NX_SDHC_ClearInterruptPendingAll(index);
		NX_SDHC_SetInterruptEnable(index, NX_SDHC_INT_CD , CTRUE);
		NX_SDHC_SetInterruptEnable(index, NX_SDHC_INT_HLE, CTRUE);

		result = NX_SDCARD_STATUS_NOERROR;

		NX_SDHC_SetCommandArgument(index, argc);
		NX_SDHC_SetCommand(index, (cmd & 0xFF), flags);

		/* Wait for interrupt to get response. */
		if (0 > wait_cmd(index)) {
			ERROUT("Error, Wait timeout, CMD=%02d !!!\n", (cmd & 0xFF));
			result |= NX_SDCARD_STATUS_CMDTOUT;
			break;
		}

		status = NX_SDHC_GetInterruptPending32(index);

		if (! (status & (1<<NX_SDHC_INT_CD))) {
			result |= NX_SDCARD_STATUS_CMDTOUT;
			ERROUT("Error, command done, CMD=%02d !!! \n", (cmd&0xFF));
			break;
		}

		if (status & (1<<NX_SDHC_INT_HLE)) {
			result |= NX_SDCARD_STATUS_ERROR;
			ERROUT("Error, hardware locked, CMD=%02d !!! \n", (cmd&0xFF));
			break;
		}

		/* retry error */
		if ( status &
			((1<<NX_SDHC_INT_RE) 	|
			 (1<<NX_SDHC_INT_RTO) 	|
			 (1<<NX_SDHC_INT_RCRC))	) {
			if(status & (1<<NX_SDHC_INT_RE)) {
				result |= NX_SDCARD_STATUS_RESERROR;
			//	ERROUT("Error, reponse, CMD=%02d !!! \n", (cmd&0xFF));
			}
			if(status & (1<<NX_SDHC_INT_RTO)) {
				result |= NX_SDCARD_STATUS_RESTOUT;
			//	ERROUT("Error, response timeout, CMD=%02d !!! \n", (cmd&0xFF));
			}
			if(status & (1<<NX_SDHC_INT_RCRC)) {
				result |= NX_SDCARD_STATUS_RESCRCFAIL;
			//	ERROUT("Error, response crc, CMD=%02d !!! \n", (cmd&0xFF));
			}
		}

		/* Get response and Exit */
		if ((result == NX_SDCARD_STATUS_NOERROR) && (flags & NX_SDHC_CMDFLAG_SHORTRSP)) {
			if ((flags & NX_SDHC_CMDFLAG_LONGRSP) == NX_SDHC_CMDFLAG_LONGRSP)
				NX_SDHC_GetLongResponse(index, response);			/* long response */
			else
				response[0] = NX_SDHC_GetShortResponse(index);		/* short response */
			break;
		}
		msleep(10);
	}

	if (flags & NX_SDHC_CMDFLAG_STOPABORT)
		NX_SDHC_ResetFIFO(index);

__exit_cmd:
	DBGOUT("CMD=%02d, RET=0x%08x\n", (cmd & 0xFF), result);

	return result;
}

static int transfer_cmd(struct mmc *mmc, struct mmc_cmd *mmc_cmd, struct mmc_data *mmc_dat)
{
	int  index = mmc->block_dev.dev;
	uint cmd   = mmc_cmd->cmdidx;
	uint arg   = mmc_cmd->cmdarg;

	uint resptype = SDHC_RESP_NONE;
	uint response[4];

	uint datatype = SDHC_DATA_NONE;
	int  datalen  = 0;
	int  ret = 0;

	switch (mmc_cmd->resp_type) {
		case MMC_RSP_NONE:	resptype = SDHC_RESP_NONE;
							break;
	//	case MMC_RSP_R5	 :
	//	case MMC_RSP_R6	 :
	//	case MMC_RSP_R7	 :
		case MMC_RSP_R1	 :	resptype = SDHC_RESP_R1;
							break;
		case MMC_RSP_R1b :	resptype = SDHC_RESP_R1b;
							break;
		case MMC_RSP_R2	 :	resptype = SDHC_RESP_R2;
							break;
	//	case MMC_RSP_R3	 :
		case MMC_RSP_R4	 :	resptype = SDHC_RESP_R3;
							break;
		default 		 :	resptype = SDHC_RESP_NONE;
							break;
	}

	/* data status */
	if (mmc_dat) {
		datatype = mmc_dat->flags;
		datalen  = mmc_dat->blocks * mmc_dat->blocksize;
	}

	setup_hc(index, datalen);

	/* send sd command */
	ret = send_cmd(index, cmd, arg, resptype, response, datatype);

	/* check error */
	if (NX_SDCARD_STATUS_NOERROR != ret) {
		switch (ret) {
		case NX_SDCARD_STATUS_RESTOUT	:	ret = TIMEOUT;		break;
		case NX_SDCARD_STATUS_NOCARD	:	ret = NO_CARD_ERR;	break;
		case NX_SDCARD_STATUS_CMDBUSY	:
		case NX_SDCARD_STATUS_CMDTOUT	:
		case NX_SDCARD_STATUS_RESCRCFAIL:
		case NX_SDCARD_STATUS_RESERROR	:
		case NX_SDCARD_STATUS_UNKNOWNCMD:	ret = UNUSABLE_ERR;	break;
		default							:	ret = UNUSABLE_ERR;	break;
		}
		return ret;
	}

	/* copy response */
	if (MMC_RSP_PRESENT & mmc_cmd->resp_type) {
		mmc_cmd->response[0] = response[0];
		if (mmc_cmd->resp_type & MMC_RSP_136) {
			if (IS_SD(mmc)) {
			DBGOUT("This is SD\n");
			mmc_cmd->response[0] = response[3];
			mmc_cmd->response[1] = response[2];
			mmc_cmd->response[2] = response[1];
			mmc_cmd->response[3] = response[0];
			} else {
			DBGOUT("This is MMC\n");
			mmc_cmd->response[1] = response[1];
			mmc_cmd->response[2] = response[2];
			mmc_cmd->response[3] = response[3];
			}
		}
	}

	return ret;
}

static int read_data(int index, char *addr, int length)
{
	unsigned int status = 0;
	unsigned int * destaddr = (unsigned int *)addr;
	int translen = length;

	DBGOUT("read to addr=0x%08x, length=%d\r\n", (unsigned int)addr, length);

	if ((unsigned int)addr & 0x3)
		ERROUT("Warn, read buffer is not aligned 4 = 0x%08x ...\n", (unsigned int)addr);

	while(translen > 0)	{

		if (0 > wait_data(index)) {
			ERROUT("Error, Wait timeout for read !!!\n");
			break;
		}

		status = NX_SDHC_GetInterruptPending32(index);

		if ( status &
		    ((1<<NX_SDHC_INT_DRTO)|
		     (1<<NX_SDHC_INT_FRUN)|
			 (1<<NX_SDHC_INT_SBE) |
			 (1<<NX_SDHC_INT_EBE) |
			 (1<<NX_SDHC_INT_DCRC))	) {
			if (status & (1<<NX_SDHC_INT_DRTO))
				ERROUT("Error, read timeout	!!! \n");
			if (status & (1<<NX_SDHC_INT_FRUN))
				ERROUT("Error, FIFO run		!!! \n");
			if(status & (1<<NX_SDHC_INT_SBE))
				ERROUT("Error, Start bit 	!!! \n");
			if(status & (1<<NX_SDHC_INT_EBE))
				ERROUT("Error, End bit 		!!! \n");
			if(status & (1<<NX_SDHC_INT_DCRC))
				ERROUT("Error, Data crc 	!!! \n");
			break;
		}

		if ( ( status & (1 << NX_SDHC_INT_RXDR)) ||
			 ((status & (1 << NX_SDHC_INT_DTO)) && translen > 0) ) {
			int fifo_cnt = NX_SDHC_GetFIFOCount(index);
			if (fifo_cnt > 0) {
				unsigned int data = NX_SDHC_GetData(index);
				if (fifo_cnt == 1)
					NX_SDHC_ClearInterruptPending(index, NX_SDHC_INT_RXDR);

				/* refer to fifo threshold */
				*destaddr++ = data;
				translen   -= 4;
			}
		}
	}

	if (0 != NX_SDHC_GetFIFOCount(index))
		ERROUT("Fail, FIFO is not empty, FIFO = %d, Remain = %d (%d)\n\n\n",
			NX_SDHC_GetFIFOCount(index), translen, length);

	NX_SDHC_SetInterruptEnable(index, NX_SDHC_INT_DTO, CFALSE);
	NX_SDHC_ClearInterruptPendingAll(index);

	DBGOUT("read - %s(%d byte)\n", translen ? "not complete":"complete", length-translen);

	return (length - translen);
}

/* set data for single block transfer */
static int transfer_data(struct mmc *mmc, struct mmc_cmd *cmd, struct mmc_data *dat)
{
	int   index = mmc->block_dev.dev;
	int   length = dat->blocks * dat->blocksize;
	char *dest   = dat->dest;
	int   ret = 0;

	if (dat->flags & MMC_DATA_READ) {
		if (dat->blocksize != read_data(index, dest, length))
			ret = COMM_ERR;
	} else {
		printf("******** NOT IMPLEMENT SDHC WRITE ********\n");
		ret = UNUSABLE_ERR;
	}

	return ret;
}

int sdhc_arch_init(void)
{
	struct sdhc *host = &_sdhc;

	int index = 0;
	unsigned int f_src, f_max, f_min;

	int c_src = CFG_SDHC_CLK_SRC;
	int c_div = 0, f_cur = 0;

	/* get sdhc clock */
	switch (c_src) {
	case NX_SDHC_CLKSRC_PLL0: f_src = CFG_SYS_PLL0_FREQ; break;
	case NX_SDHC_CLKSRC_PLL1: f_src = CFG_SYS_PLL1_FREQ; break;
	case NX_SDHC_CLKSRC_PCLK: f_src = CFG_SYS_PCLK_FREQ; break;
	default:  c_src = NX_SDHC_CLKSRC_PLL1, f_src = CFG_SYS_PLL1_FREQ; break;
	}

	c_div  = f_src / SDHC_CLOCK_MAX;
	c_div += f_src % SDHC_CLOCK_MAX ? 1 : 0;

	f_max = f_src / c_div;
	f_min = f_src / c_div / SDHC_CLOCK_MIN;

	f_cur  = f_max / SDHC_CLOCK_MIN;
	f_cur += f_max % SDHC_CLOCK_MIN ? 1 : 0;

	/* sd host controller */
	NX_SDHC_Initialize();
	NX_SDHC_SetBaseAddress(index, NX_SDHC_GetPhysicalAddress(index));

	/* Enable */
	NX_SDHC_SetClockSource (index, 0, c_src);
	NX_SDHC_SetClockDivisor(index, 0, c_div);
	NX_SDHC_SetClockPClkMode(index, NX_PCLKMODE_ALWAYS);

	NX_SDHC_SetOutputClockEnable (index, CTRUE);
	NX_SDHC_SetClockDivisorEnable(index, CTRUE);

	NX_SDHC_SetOutputClockDivider(index, f_cur);
	NX_SDHC_OpenModule(index);

	NX_SDHC_ResetController(index);        			// Reset the controller.
	while (NX_SDHC_IsResetController(index)) { ; } 	// Wait until the controller reset is completed.

	NX_SDHC_ResetDMA(index);        				// Reset the DMA interface.
	while (NX_SDHC_IsResetDMA(index)) { ; }			// Wait until the DMA reset is completed.

	NX_SDHC_ResetFIFO(index);        				// Reset the FIFO.
	while(NX_SDHC_IsResetFIFO(index)) { ; }    		// Wait until the FIFO reset is completed.

	NX_SDHC_SetDMAMode(index, CFALSE);
	NX_SDHC_SetLowPowerClockMode(index, CFALSE);
	NX_SDHC_SetDataTimeOut(index, 0xFFFFFF);
	NX_SDHC_SetResponseTimeOut(index, 0xff);		//0x64;
	NX_SDHC_SetDataBusWidth(index, 1);

	NX_SDHC_SetBlockSize(index, SDHC_BLOCK_LENGTH);
	NX_SDHC_SetFIFORxThreshold(index, (1-1));		// Issue when RX FIFO Count >= 1 x 4 bytes
	NX_SDHC_SetFIFOTxThreshold(index, 8); 			// Issue when TX FIFO Count <= 8 x 4 bytes

	NX_SDHC_SetInterruptEnableAll(index, CFALSE);
	NX_SDHC_ClearInterruptPendingAll(index);

	/* sdhc parameters */
	host->clk_max   = f_max;
	host->clk_min   = f_min;
	host->clk_cur   = 0;
	host->bus_width = 1;

	DBGOUT("mmc%d done, clock %d, %d ~ %d\n",
		index, host->clk_cur, host->clk_min, host->clk_max);

	return 0;
}

static int sdhc_request(struct mmc *mmc, struct mmc_cmd *cmd, struct mmc_data *dat)
{
	int ret = transfer_cmd(mmc, cmd, dat);
	if (0 > ret)
		return ret;

	if (dat)
		ret = transfer_data(mmc, cmd, dat);

	return ret;
}

static void sdhc_set_ios(struct mmc *mmc)
{
	int index = mmc->block_dev.dev;

	set_bus_w(index, mmc->bus_width);
	set_clock(index, mmc->clock);
}

static int sdhc_init(struct mmc *mmc)
{
	DBGOUT("%s pass\n", __func__);
	return 0;
}

int board_mmc_init(bd_t *bis)
{
	struct mmc *mmc = NULL;

	mmc = malloc(sizeof(struct mmc));
	if (!mmc)
		return -ENOMEM;

	sprintf(mmc->name, "Nexell SDHC");

	mmc->send_cmd 	= sdhc_request;
	mmc->set_ios 	= sdhc_set_ios;
	mmc->init 		= sdhc_init;
	mmc->host_caps 	= MMC_MODE_4BIT;
	mmc->voltages 	= MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->f_max 		= SDHC_CLOCK_MAX;
	mmc->f_min 		= SDHC_CLOCK_MIN;

	mmc->block_dev.part_type = PART_TYPE_DOS;

	mmc_register(mmc);

	mmc->block_dev.dev = 0;

	return 0;
}
