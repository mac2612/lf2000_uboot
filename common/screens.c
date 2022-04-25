/*
 * Screen State Machine
 *
 * (C) Copyright 2012
 * Scott Esters, LeapFrog <sesters@leapfrog.com>
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
#include <stdarg.h>
#include <linux/types.h>
#include <lcd.h>

#if defined(CONFIG_ARCH_LF2000)
#include <board_revisions.h>
#include <mfgdata.h>
#include <lf2000_board.h>
#endif

/************************************************************************/
/* ** SCREEN DATA							*/
/************************************************************************/
#include <screen-list.h>	/* generic names for each screen	*/
#include <screens.h>		/* Get screen data, maybe gzipped	*/

#ifndef msleep
#define	msleep(_n)		__udelay(_n*1000)
#endif	/* msleep */


#define BATTERY_SHOW		2	/* seconds to show battery for	*/
#define BATTERY_NO_SHOW		10	/* seconds to leave msg off */
#define CHARGING_SHOW 		6 /* seconds to show charging screen for */

screen_state old_screen_state = SCREEN_NONE; /* no prior screen state   */
unsigned char *old_screen = 0;		/* no prior screen		*/
unsigned long long idle_time_end = 0;	/* ticks until idle screen	*/
unsigned long long charging_time_end = 0; /* ticks till charging screen */
unsigned long long next_low_battery_check = 0;/* check battery timer in ticks*/
unsigned long long show_low_battery_end = 0; /* show battery until this time */
unsigned long long last_second = 0;	/* last second shown on screen  */
int	draw_lcd_info = 0;		/* 1 = draw LCD version info	*/


/************************************************************************/
/* run_screen_state							*/
/*	Show screens based on current system conditions.		*/
/*									*/
/*	This cooperative process runs occasionally when called from	*/
/*	inside u-boot 'busy' loops.					*/
/************************************************************************/
void run_screen_state ( screen_state new_screen_state )
{
	unsigned char *new_screen = NULL;
	char          *state_name = "";
	int	      battery = 0;
	int	      draw_lcd_info_enable = 0;
	int       i;

	/* Continuously check for low battery.
	 * When found show low battery screen for BATTERY_SHOW seconds;
	 * then leave off for BATTERY_NO_SHOW seconds.
	 * Note this may interfere with an active USB connection.
	 */

	if (new_screen_state == SCREEN_REFRESH)
		new_screen_state = old_screen_state;
	else
		old_screen_state = new_screen_state;

	/* if low battery is on screen then leave it up until
	 * timer expires
	 */
	if (show_low_battery_end != 0) {	/* timer running */
		if (get_ticks() < show_low_battery_end)
			return;
		else
			show_low_battery_end = 0;
	}

	/* time to check battery level?  Check each time
	 * until a low battery condition is found
	 */
	if (next_low_battery_check < get_ticks()) {
		if (is_low_battery()) {
			new_screen_state = SCREEN_LOW_BATTERY;
			/* calculate low_battery screen end time */
			show_low_battery_end =
				get_ticks() + get_tbclk() * BATTERY_SHOW;
			/* low battery found, don't check for a while */
			next_low_battery_check = show_low_battery_end +
				get_tbclk() * BATTERY_NO_SHOW;
		}
	}

	/* reset idle connect timer unless showing a connect screen */
	if (new_screen_state != SCREEN_TUNEUP_VBUS)
		idle_time_end = 0;

	/* choose a possible screen */
	switch(new_screen_state) {

	case SCREEN_TUNEUP_NO_VBUS:
		new_screen = SYSTEM_ATTENTION_SCREEN;
		state_name = "Waiting for Tuneup";
		draw_lcd_info_enable = 1;	/* show LCD info if button pressed */
		break;

	case SCREEN_TUNEUP_VBUS:
		/* if first time, then calculate idle timeout */
		if (!idle_time_end) {
			idle_time_end = get_ticks() + get_tbclk() * 45;
		}

		if (get_ticks() < idle_time_end) 
		{
			new_screen = CONNECTED_SCREEN;
			state_name = "Connected";
		} 
		else 
		{
			new_screen = DOWNLOAD_TIMEOUT_SCREEN;
			state_name = "Timeout";
		}
		draw_lcd_info_enable = 1;	/* show LCD info if button pressed */
		break;
		
	case SCREEN_LOW_BATTERY:
		new_screen = LOW_BATTERY_SCREEN;
		state_name = "Low Battery";
		break;

	/* should never get here, but included for completeness */
	case SCREEN_REFRESH:
		break;
	
#if (defined(CONFIG_MACH_NXP3200_RIO_DTK))
	case SCREEN_TUNEUP_VBUS_WALL:
		new_screen = CONNECTED_WALL_SCREEN;
		state_name = "Wall Charger";
		break;
		
	case SCREEN_CHARGING:
		if(!charging_time_end) {
			charging_time_end = get_ticks() + get_tbclk() * CHARGING_SHOW;
		}
		new_screen = CHARGING_SCREEN;
		state_name = "Charging";
		break;
		
	case SCREEN_DISCONNECT_ERROR:
		new_screen = DISCONNECT_ERROR_SCREEN;
		state_name = "Surgeon failure";
#endif
		
	case SCREEN_NONE:
	case SCREEN_NORMAL:
	default:
		new_screen = LOGO_SCREEN;
		state_name = "Booting";
		break;
	}


	/* update screen if needed */
	if ((old_screen != new_screen) || (draw_lcd_info != is_show_lcd_info())) {
		old_screen = new_screen;
		draw_lcd_info = is_show_lcd_info();
		if (new_screen != NULL)
		{
			lcd_display_bitmap((unsigned long)new_screen, 0, 0);
			NX_MLC_SetDirtyFlag(CFG_DISP_MAIN_SCREEN, CFG_DISP_LAYER_SCREEN); // Enable layer only when the image has been completely drawn.
		}
#if (defined(CONFIG_MACH_NXP3200_RIO_DTK))
		if (new_screen == CHARGING_SCREEN)
		{
			while (get_ticks() < charging_time_end); /* display CHARGING_SCREEN for a minimum time */
			//for(i = 15; i >= 0 ; i--)
			//{
				//NX_MLC_SetAlphaBlending(CFG_DISP_MAIN_SCREEN, CFG_DISP_LAYER_SCREEN, CTRUE, i);
				//NX_MLC_SetDirtyFlag(CFG_DISP_MAIN_SCREEN, CFG_DISP_LAYER_SCREEN);
				//msleep(1000);
				////printf("%s.%d:%s alpha = %d \n", __FILE__, __LINE__, __func__, i);
			//}
		}
#endif
		battery = measure_battery();
		printf("%s.%d:%s Report Battery %d \n", __FILE__, __LINE__, __func__, battery);
			
		if (draw_lcd_info_enable && draw_lcd_info) {
			lcd_show_board_info();	/* redisplay info */
			lcd_printf("state:%s\n", state_name);
			lcd_printf("time:%llu\n", get_ticks()/get_tbclk());
			if (battery < 0)
				lcd_printf("battery:  NONE\n");
			else
				lcd_printf("battery: %d mv\n", battery);
		}
	}
}
