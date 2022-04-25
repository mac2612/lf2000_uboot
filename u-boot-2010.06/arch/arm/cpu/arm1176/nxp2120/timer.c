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

/* degug print */
#if	(0)
#define DBGOUT(msg...)		{ printf("timer: " msg); }
#define	ERROUT(msg...)		{ printf("timer: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#define	ERROUT(msg...)		do {} while (0)
#endif

/* global variables to save timer count */
static ulong timestamp;
static ulong lastdec;

/* macro to read from the 32 bit hw timer */
#define R_TIMER_COUNT		NX_TIMER_GetTimerCounter(CFG_TIMER_SYS_TICK_CH)

/* macro to hw timer tick config */
#define	TIMER_FREQ			(CFG_TIMER_SYS_TICK_CLKFREQ)
#define	TIMER_HZ			(CFG_TIMER_SYS_TICK_CLKFREQ / CONFIG_SYS_HZ)

/*------------------------------------------------------------------------------
 * u-boot timer interface
 */
int timer_init(void)
{
	NX_TIMER_SetClockDivisorEnable(CFG_TIMER_SYS_TICK_CH, CFALSE);
	NX_TIMER_SetClockSource(CFG_TIMER_SYS_TICK_CH, 0, CFG_TIMER_SYS_TICK_CLKSRC);
	NX_TIMER_SetClockDivisor(CFG_TIMER_SYS_TICK_CH, 0, CFG_TIMER_SYS_TICK_CLKDIV);
	NX_TIMER_SetClockPClkMode(CFG_TIMER_SYS_TICK_CH, NX_PCLKMODE_ALWAYS);
	NX_TIMER_SetClockDivisorEnable(CFG_TIMER_SYS_TICK_CH, CTRUE);
	NX_TIMER_Stop(CFG_TIMER_SYS_TICK_CH);

	NX_TIMER_SetWatchDogEnable(CFG_TIMER_SYS_TICK_CH, CFALSE);
	NX_TIMER_SetInterruptEnableAll(CFG_TIMER_SYS_TICK_CH, CFALSE);
	NX_TIMER_ClearInterruptPendingAll(CFG_TIMER_SYS_TICK_CH);

	NX_TIMER_SetTClkDivider(CFG_TIMER_SYS_TICK_CH, NX_TIMER_CLOCK_TCLK);
	NX_TIMER_SetTimerCounter(CFG_TIMER_SYS_TICK_CH, 0);
	NX_TIMER_SetMatchCounter(CFG_TIMER_SYS_TICK_CH, CFG_TIMER_SYS_TICK_CLKFREQ);
	NX_TIMER_Run(CFG_TIMER_SYS_TICK_CH);

	reset_timer_masked();

	return 0;
}

void reset_timer(void)
{
	reset_timer_masked();
}

ulong get_timer(ulong base)
{
	ulong time = get_timer_masked();
	ulong hz   = TIMER_HZ;

	return (time/hz - base);
}

void set_timer(ulong t)
{
	timestamp = (ulong)t;
}

#if 0
void udelay(unsigned long usec)
#else
void __udelay(unsigned long usec)
#endif
{
	ulong tmo, tmp;

	DBGOUT("+udelay=%d\n", usec);

	if (usec >= 1000) {			// if "big" number, spread normalization to seconds //
		tmo = usec / 1000;		// start to normalize for usec to ticks per sec //
		tmo *= TIMER_FREQ;		// find number of "ticks" to wait to achieve target //
		tmo /= 1000;			// finish normalize. //
	} else {						// else small number, don't kill it prior to HZ multiply //
		tmo = usec * TIMER_FREQ;
		tmo /= (1000*1000);
	}

	tmp = get_ticks();			// get current timestamp //

	DBGOUT("tmo=%d, tmp=%d\n", tmo, tmp);

	if ( tmp > (tmo + tmp + 1) )	// if setting this fordward will roll time stamp //
		reset_timer_masked();	// reset "advancing" timestamp to 0, set lastdec value //
	else
		tmo += tmp;				// else, set advancing stamp wake up time //

	while (tmo > get_timer_masked ())// loop till event //
	{
		//*NOP*/;
	}

	DBGOUT("-udelay=%d\n", usec);

	return;
}

void reset_timer_masked(void)
{
	/* reset time */
	lastdec = R_TIMER_COUNT;  	/* capure current decrementer value time */
	timestamp = 0;	       		/* start "advancing" time stamp from 0 */
}

ulong get_timer_masked(void)
{
	ulong now = R_TIMER_COUNT;		/* current tick value */

	if (now > lastdec) {			/* normal mode (non roll) */
		timestamp += now - lastdec; /* move stamp fordward with absoulte diff ticks */
	} else {						/* we have overflow of the count down timer */
		/* nts = ts + ld + (TLV - now)
		 * ts=old stamp, ld=time that passed before passing through -1
		 * (TLV-now) amount of time after passing though -1
		 * nts = new "advancing time stamp"...it could also roll and cause problems.
		 */
		timestamp += now + TIMER_FREQ - lastdec;
	}

	DBGOUT("now=%d, last=%d, timestamp=%d\n", now, lastdec, timestamp);

	lastdec = now;

	return (ulong)timestamp;
}

void udelay_masked(unsigned long usec)
{
	ulong tmo;
	ulong endtime;
	signed long diff;

	if (usec >= 1000) {		/* if "big" number, spread normalization to seconds */
		tmo = usec / 1000;	/* start to normalize for usec to ticks per sec */
		tmo *= TIMER_FREQ;		/* find number of "ticks" to wait to achieve target */
		tmo /= 1000;		/* finish normalize. */
	} else {			/* else small number, don't kill it prior to HZ multiply */
		tmo = usec * TIMER_FREQ;
		tmo /= (1000*1000);
	}

	endtime = get_timer_masked() + tmo;

	do {
		ulong now = get_timer_masked();
		diff = endtime - now;
	} while (diff >= 0);
}

unsigned long long get_ticks(void)
{
	return get_timer_masked();
}

ulong get_tbclk(void)
{
	ulong tbclk;

	tbclk = TIMER_FREQ;

	return tbclk;
}
