/*
 * include/lf2000_board.h 
 *
 * Copyright 2012 LeapFrog Enterprises Inc.
 *
 * Leapfrog Firmware <firmware@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef _LF2000_BOARD_H_
#define _LF2000_BOARD_H_

u32 get_board_rev(void);
void  bus_init(void);
int init_baudrate (void);
void init_gpio_pad(void);
void init_panel_info(void);

int is_low_battery(void);
int is_lucy(void);
int is_nor_boot(void);
int is_spi_boot(void);
int is_sdhc_boot(void);
int is_show_lcd_info(void);
int is_tune_up_boot(void);
int is_valencia(void);
int have_usb_power_option(void);

int measure_battery(void);
void run_inactivity_timer(int reset);
void set_gpio_pad(U32 io_grp, U32 io_bit, U32 io_pad_config);
void soc_adc_attach(void);
void soc_adc_detach(void);
unsigned short soc_adc_read(int ch, uint timeout);
int get_screen_res(void);

int detect_usb_charger(void);

struct lf2000_board_info {
	int	is_initialized;			/* structure is initialized	*/
	/* display variables */
	u32	cfg_disp_pri_resol_width;
	u32	cfg_disp_pri_resol_height;
	u32	cfg_disp_pri_clkgen0_div;
	u32	cfg_disp_pri_out_clk_invert;
	u32	cfg_disp_pri_hsync_front_porch;
	u32	cfg_disp_pri_hsync_back_porch;
	u32	cfg_disp_pri_vsync_front_porch;
	u32	cfg_disp_pri_vsync_back_porch;
	u32	spi_lcd;
}__attribute__ ((packed));

struct lf2000_board_info * get_lf2000_board_info(void);

#endif /* _LF2000_BOARD_H_*/

