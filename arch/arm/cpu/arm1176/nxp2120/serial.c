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

/* nexell soc headers */
#include <platform.h>

DECLARE_GLOBAL_DATA_PTR;

static void uart_tx_byte(unsigned char c);
static int  uart_rx_byte(void);
static int  uart_rx_count(void);

static int	g_inituart = 0;
/*------------------------------------------------------------------------------
 * u-boot serial interface
 */
int serial_init(void)
{
	int baudrate = CFG_UART_DEBUG_BAUDRATE;

	if (g_inituart)
		return 0;

	g_inituart = 1;

	NX_UART_Initialize();
	NX_UART_SetBaseAddress(CFG_UART_DEBUG_CH, (U32)NX_UART_GetPhysicalAddress(CFG_UART_DEBUG_CH));
	NX_UART_OpenModule(CFG_UART_DEBUG_CH);

	NX_UART_SetClockDivisorEnable(CFG_UART_DEBUG_CH, CFALSE);
	NX_UART_SetClockPClkMode(CFG_UART_DEBUG_CH, NX_PCLKMODE_DYNAMIC);

	// UART Mode : Tx, Rx Only
	NX_UART_SetSIRMode(CFG_UART_DEBUG_CH, CFALSE);
	NX_UART_SetLoopBackMode(CFG_UART_DEBUG_CH, CFALSE);
	NX_UART_SetAutoFlowControl(CFG_UART_DEBUG_CH, CFALSE);
	NX_UART_SetHalfChannelEnable(CFG_UART_DEBUG_CH, CTRUE);	// Full or Half

    NX_UART_SetSCTxEnb(CFG_UART_DEBUG_CH, CFALSE);
    NX_UART_SetSCRxEnb(CFG_UART_DEBUG_CH, CTRUE);

	// Frame Configuration : Data 8 - Parity 0 - Stop 1
	NX_UART_SetFrameConfiguration(CFG_UART_DEBUG_CH, NX_UART_PARITYMODE_NONE, 8, 1);

	// Tx Rx Operation : Polling
	NX_UART_SetInterruptEnableAll(CFG_UART_DEBUG_CH, CFALSE);
	NX_UART_ClearInterruptPendingAll(CFG_UART_DEBUG_CH);
    NX_UART_SetTxIRQType(CFG_UART_DEBUG_CH, NX_UART_IRQTYPE_PULSE);
    NX_UART_SetTxOperationMode(CFG_UART_DEBUG_CH, NX_UART_OPERMODE_NORMAL);

    NX_UART_SetRxIRQType(CFG_UART_DEBUG_CH, NX_UART_IRQTYPE_PULSE);
    NX_UART_SetRxOperationMode(CFG_UART_DEBUG_CH, NX_UART_OPERMODE_NORMAL);
    NX_UART_SetRxTimeOutEnb(CFG_UART_DEBUG_CH, CFALSE);

    NX_UART_SetSYNCPendClear(CFG_UART_DEBUG_CH);

	// FIFO Control
    NX_UART_SetFIFOEnb(CFG_UART_DEBUG_CH, CTRUE);
    NX_UART_ResetTxFIFO(CFG_UART_DEBUG_CH);
    NX_UART_ResetRxFIFO(CFG_UART_DEBUG_CH);

	// UART clock
	NX_UART_SetClockSource(CFG_UART_DEBUG_CH, 0, CFG_UART_DEBUG_CLKSRC);
	NX_UART_SetClockDivisor(CFG_UART_DEBUG_CH, 0, CFG_UART_DEBUG_CLKDIV);

    NX_UART_SetBRD(CFG_UART_DEBUG_CH, NX_UART_MakeBRD(baudrate, CFG_UART_DEBUG_CLKFREQ));

	NX_UART_SetClockDivisorEnable(CFG_UART_DEBUG_CH, CTRUE);
	NX_UART_SetClockPClkMode(CFG_UART_DEBUG_CH, NX_PCLKMODE_ALWAYS);

	//--------------------------------------------------------------------------
	// Some delay to update baud rate on pc program. : 19200 -> 115200
	{
	volatile U32 temp;
	for (temp=0 ; temp < 0x0000FFFF ; temp++);
	}

	return 0;
}

void serial_putc(const char c)
{
#if 0
	if (c == '\r') return;
#endif
	/* If \n, also do \r */
    if (c == '\n')
		uart_tx_byte('\r');

	uart_tx_byte(c);
}

int serial_tstc(void)
{
	return uart_rx_count();
}

int serial_getc(void)
{
	int c = 0;

	do {
		c = uart_rx_byte();;
	} while (!c);

	return c;
}

void serial_puts(const char *s)
{
	while (*s)
		serial_putc(*s++);
}

void serial_setbrg(void)
{
	return;
}

/*------------------------------------------------------------------------------
 * u-boot serial hw configure.
 */
static void uart_tx_byte(unsigned char ch)
{
	while (!(NX_UART_GetTxRxStatus(CFG_UART_DEBUG_CH) &
			NX_UART_TX_BUFFER_EMPTY) ) { ; }
	NX_UART_SendByte(CFG_UART_DEBUG_CH, (U8)ch);

#if (0)
	while (!(NX_UART_GetTxRxStatus(CFG_UART_DEBUG_CH) &
			NX_UART_TX_BUFFER_EMPTY) ) { ; }
#endif
}

static int uart_rx_byte(void)
{
	char data = 0;

	if (NX_UART_GetRxFIFOCount(CFG_UART_DEBUG_CH))
		data = (char)NX_UART_GetByte(CFG_UART_DEBUG_CH);

	return (int)data;
}

static int uart_rx_count(void)
{
	return (int)NX_UART_GetRxFIFOCount(CFG_UART_DEBUG_CH);
}

