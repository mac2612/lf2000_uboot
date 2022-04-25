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
extern u_int cpu_get_clock_hz(int clk);

DECLARE_GLOBAL_DATA_PTR;

static void uart_tx_byte(unsigned char c);
static int  uart_rx_byte(void);
static int  uart_rx_count(void);

int	g_inituart = 0;

/*
 * u_int serial_calc_divisor(uart_channel,baud)
 * calculate divisor for given uart channel
 */

u_int calc_uart_divisor(u_int uart_channel, u_int baud)
{
	/* get UART information */
	u_int	clock_source = NX_UART_GetClockSource(uart_channel, 0);
	u_int	clock_divisor = NX_UART_GetClockDivisor(uart_channel, 0);

	/* get PLL frequency */
	u_int	clock_freq_in_hz = cpu_get_clock_hz(clock_source);

	/* calculate divisor */
	u_int	divisor = clock_freq_in_hz;
	divisor /= clock_divisor;
	divisor /= baud;
	divisor += 16 / 2;	/* round number */
	divisor /= 16;
	divisor-= 1;		/* one less for divisor */

	return divisor;
}

/*------------------------------------------------------------------------------
 * u-boot serial interface
 */
void set_gpio_pad(U32 io_grp, U32 io_bit, U32 io_pad_config);

int nxp3200_serial_init(void)
{
	int baudrate = CFG_UART_DEBUG_BAUDRATE;

	if (g_inituart)
		return 0;

	g_inituart = 1;

	/* enable transmitter function */
	set_gpio_pad(PAD_GET_GRP(PAD_GPIO_A), 8, PAD_GPIOA08_);


	NX_UART_Initialize();
	NX_UART_SetBaseAddress(CFG_UART_DEBUG_CH, (U32)NX_UART_GetPhysicalAddress(CFG_UART_DEBUG_CH));
	NX_UART_OpenModule(CFG_UART_DEBUG_CH);

	/* reset FIFO */
	NX_UART_SetFIFOConfig(CFG_UART_DEBUG_CH, CFALSE, CTRUE, CTRUE, 8, 8);

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

	NX_UART_SetBRD(CFG_UART_DEBUG_CH, calc_uart_divisor(CFG_UART_DEBUG_CH, baudrate));

	NX_UART_SetClockDivisorEnable(CFG_UART_DEBUG_CH, CTRUE);
	NX_UART_SetClockPClkMode(CFG_UART_DEBUG_CH, NX_PCLKMODE_ALWAYS);
	printf("NX_UART_GetClockDivisor: 0x%X\n", NX_UART_GetClockDivisor(CFG_UART_DEBUG_CH, 0));
	printf("NX_UART_GetBRD:          0x%X\n", NX_UART_GetBRD(CFG_UART_DEBUG_CH));

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

void lfp100_monitor_power_button(void);
int serial_getc(void)
{
	int c = 0;

	do {
#if defined (CONFIG_SOC_LFP100)
		lfp100_monitor_power_button();
#endif
		c = uart_rx_byte();;
	} while (!c);

	return c;
}


extern int disable_serial_output;

void serial_puts(const char *s)
{
	if (!disable_serial_output) {
		while (*s)
			serial_putc(*s++);
	}
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
	if (! g_inituart)
		return;

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

	if (! g_inituart)
		return 0;

	if (NX_UART_GetRxFIFOCount(CFG_UART_DEBUG_CH))
		data = (char)NX_UART_GetByte(CFG_UART_DEBUG_CH);

	return (int)data;
}

static int uart_rx_count(void)
{
	if (! g_inituart)
		return 0;

	return (int)NX_UART_GetRxFIFOCount(CFG_UART_DEBUG_CH);
}

