/* linux/drivers/usb/gadget/lf2000-hsotg.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * S3C USB2.0 High-speed / OtG driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
/*------------------------------------------------------------------------------
 * This file is derived from lf2000-hsotg.c in Leapfrog's LF2000 Linux
 * drivers/usb/gadget directory.  It has been modified for use in u-boot.
 * 
 * Most code that depends on linux header files has been disabled by putting
 * it inside #ifndef U_BOOT ... #endif brackets.  You'll note that U_Boot is
 * #defined near the beginning of this file.  The linux-dependent code was
 * disabled rather than deleted because its presence might simplify the process
 * of porting this file's changes to new versions of Leapfrog's linux usb
 * gadget driver.
 *
 * Most code that has been added for the u-boot environment is inside
 * #ifdef U_BOOT ... #endif brackets.
 *
 * The linux gadget driver comprises initialization code, interrupt service
 * code, and code that is part of a kernel thread.  Usually the kernel thread
 * sleeps.  It is awakened by signals (e.g., terminate, exit) and by events
 * that are triggered by the interrupt service code.
 *
 * Since u-boot doesn't support threads, and since our implementation of u-boot
 * is not using interrupts, we needed to reorganize the linux gadget driver for
 * use in u-boot.  The reorganization was also affected by the fact that u-boot
 * usually does not need to communicate via usb.  It uses the gadget driver 
 * only when it needs to download a new version of the kernel.  This happens
 * when a kernel has not yet been loaded into the system's NAND.  After that
 * u-boot typically uses the gadget driver only when it finds that the kernel
 * has been corrupted and can no longer run.  There might also be ways for a
 * user to force u-boot to download a newer kernel and store it in NAND.
 *
 * While u-boot uses the gadget driver, it no longer needs to perform u-boot's
 * usual functions.  Therefore control remains in the gadget driver.  In effect
 * it becomes u-boot's single thread.  At places where the linux gadget driver
 * sleeps, the u-boot gadget driver simply polls the LF2000's OTG controller's
 * status registers.  When something happens that would trigger an event in 
 * the linux driver, the u-boot driver simply stops polling and responds to
 * the event.
 *
 * The u-boot gadget driver's entry point and main routine is udc_connect().
 * You'll find it near the end of this file.  After initializing the driver's
 * control variables and data structures, it configures the OTG controller,
 * initializes the mass storage class part of the driver, and then enters its
 * main loop:
 *	while (1) {
 *		if (wait_and_respond(0))
 *		{
 *			fsg_main_thread(&common);
 * #ifndef RAMDISK
 *			check_if_download_verified();
 * #endif
 *		}	
 *	}
 *
 * When check_if_download_verified() finds that the download has been 
 * completed successfully, it jumps to the start of the downloaded code
 * (in RAM).  We don't expect the downloaded code to return control to 
 * this loop.
 *
 *
 * The gadget driver comprises two compilation units.  This file is one of them.
 * The other comprises mass_storage.c and the files it includes.
 *
 * This file contains the code that deals with the LF2000's OTG controller.
 * You can find some information about the controller in the LF2000 Data Book.
 * More complete information is in Synopsys's "DesignWare Cores USB 2.0 Hi-Speed
 * On-the-Go (OTG) Databook."
 *----------------------------------------------------------------------------*/
#define U_BOOT 

#ifndef U_BOOT
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/input.h>
#include <linux/timer.h>
#else
#include <common.h>
#include "linux_to_uboot.h"
#include <linux/compiler.h>
#include <linux/err.h>
#include <linux/mtd/compat.h>
#include <asm/io.h>
#include <mach/lfp100.h>
#include <lf2000_board.h>

#if 1	/* Disable this #if to get lots of debug output */
#ifdef DBGOUT
#undef DBGOUT
#endif
#ifdef dev_dbg
#undef dev_dbg
#endif
#define DBGOUT(a...)
#define dev_dbg(a...)
#else
#define DBGOUT(a...)	{ printf(a); } 
#define dev_dbg(a,...)	{ printf(__VA_ARGS__); } 
#endif

#ifndef RAMDISK
void	check_if_download_verified(void);
#endif	/* RAMDISK */

#endif	/* U_BOOT */

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#ifndef U_BOOT
#include <mach/map.h>
#endif	/* U_BOOT */

#include <plat/regs-usb-hsotg-phy.h>
#include <plat/regs-usb-hsotg.h>

#if defined(CONFIG_ARCH_LF2000)
#include <mach/reg_otg.h>
#include <mach/devices.h>
#include <lf2000_board.h>
#endif

#include <plat/udc-hs.h>

#ifdef CONFIG_LCD_SCREENS
#include <screen-list.h>
#endif

/* Define a 'jiffie' to be one millisecond long
 * This choice simplifies the next two defines.
 * 'jiffies' returns the current jiffie count
 * msecs_to_jiffies(x) then needs to return just 'x'
 */
#define jiffies ( get_ticks() / ( get_tbclk() / 1000) )
#define	msecs_to_jiffies(x) (x)

#define time_after(a,b) ((long)(b) - (long)(a) < 0)

#define DMA_ADDR_INVALID (~((dma_addr_t)0))

/* Enable the #define of DEBUG to get output from s3c_hsotg_dump() */
/*
#define DEBUG 0
 */

#ifdef U_BOOT
/* otg_clk_enable() is identical to otg_clk_enable() in
 * kernel/linux/arch/arm/mach-nxp3200/device.c
 */
static void otg_clk_enable(void)
{
    writel(0x0B, LF2000_USBOTG_CLKENB);
    writel(0x0C, LF2000_USBOTG_CLKGEN);
    writel(0x0F, LF2000_USBOTG_CLKENB);
}

static void otg_phy_init(void)
{
        volatile int i;

	// This code is similar to iRomBoot's USBBOOT() code

	for(i = 10; i>0; i--);

        writel(0x0001, LF2000_USBOTG_TESTPARM7); /* Test Mode Off; 
						   force Device Mode */
        writel(0x0430, LF2000_USBOTG_TESTPARM4); /* TESTPARM4 = 0x0430 */
        writel(0x0237, LF2000_USBOTG_PHYPOR);	 /* PHY POR on (1->0)(auto clear) */

	udelay(230);		// 220us delay need. 230us delayed
	for(i = 600; i>0; i--);

        writel(0x4000, LF2000_USBOTG_LINKCTL);  
				/* LINK - prst_n (0->1)(auto clear) */
        for(i = 6; i>0; i--);

        writel(0xC000, LF2000_USBOTG_LINKCTL);  
				/* LINK - hreset_n (0->1)(auto clear) */
	/* usb core soft reset */
        writel(0x0001, S3C_UDC_OTG_GRSTCTL);  /* Core Soft Reset ->GRSTCTL */
	while ( !(0x80000000 & S3C_UDC_OTG_GRSTCTL))
		;	/* Wait for AHB Master Idle bit to be set */

}
#else
extern void otg_clk_enable(void);
extern void otg_phy_init(void);
#endif

/* patterned after the reconfig_usbd() function in linux s3c_udc_otg.c driver */
static void reconfig_usbd(void)
{

	/* init_core */
#if 1	/* 3nov11 */
        writel(0x0001, S3C_UDC_OTG_GAHBCFG);  /*  ->GAHBCFG */
        writel(0x540F, S3C_UDC_OTG_GUSBCFG);  /*  ->GUSBCFG */
#else	/* iRomBoot.c's USBBOOT() */
	pUOReg->GCSR.GAHBCFG = PTXFE_HALF|NPTXFE_HALF|MODE_SLAVE|BURST_SINGLE|GBL_INT_UNMASK;
	pUOReg->GCSR.GUSBCFG = 
		 0<<15		/* PHY Low Power Clock sel */
		|1<<14		/* Non-Periodic TxFIFO Rewind Enable */
		|5<<10		/* Turnaround time */
		|0<<9		/* 0:HNP disable, 1:HNP enable */
		|0<<8		/* 0:SRP disable, 1:SRP enable */
		|0<<7		/* ULPI DDR sel */
		|0<<6		/* 0: high speed utmi+, 1: full speed serial */
		|0<<4		/* 0: utmi+, 1:ulpi */
		|1<<3		/* phy i/f  0:8bit, 1:16bit */
		|7<<0;		/* HS/FS Timeout**/
#endif
	/* check current mode */
#if 1	/* 3nov11 */
	if (0 == (1 & readl(S3C_UDC_OTG_GINTSTS))) /* if Device Mode */
	{
		unsigned int curval;

		curval = readl(S3C_UDC_OTG_DCTL); /* DCSR.DCTL */
		/* soft disconnect on */
		writel( curval | SOFT_DISCONNECT, S3C_UDC_OTG_DCTL);
		udelay(10);
		/* soft disconnect off */
		curval = readl(S3C_UDC_OTG_DCTL); /* DCSR.DCTL */
		writel( curval & ~SOFT_DISCONNECT, S3C_UDC_OTG_DCTL);

		/* usb init device */
		writel( 1 << S3C_DCFG_EPMisCnt_SHIFT, S3C_UDC_OTG_DCFG); /* DCSR.DCFG */
		writel( INT_RESUME | INT_OUT_EP  | INT_IN_EP | INT_ENUMDONE |
			INT_RESET  | INT_SUSPEND | INT_RX_FIFO_NOT_EMPTY,
			S3C_UDC_OTG_GINTMSK); /* GCSR.GINTMSK */

	}
#else	/* iRomBoot.c's USBBOOT() */
	if ((pUOReg->GCSR.GINTSTS & 0x1) == 0)	/* if Device Mode */
	{
		/* soft disconnect on */
		pUOReg->DCSR.DCTL |= SOFT_DISCONNECT;
		udelay(10);
		/* soft disconnect off */
		pUOReg->DCSR.DCTL &= ~SOFT_DISCONNECT;

		/* usb init device */
		pUOReg->DCSR.DCFG = 1<<18;// | pUSBBootStatus->speed<<0; /* [][1: full speed(30Mhz) 0:high speed]*/
		pUOReg->GCSR.GINTMSK = INT_RESUME|INT_OUT_EP|INT_IN_EP|INT_ENUMDONE|INT_RESET|INT_SUSPEND|INT_RX_FIFO_NOT_EMPTY;
	}
#endif
}

/* EP0_MPS_LIMIT
 *
 * Unfortunately there seems to be a limit of the amount of data that can
 * be transferred by IN transactions on EP0. This is either 127 bytes or 3
 * packets (which practically means 1 packet and 63 bytes of data) when the
 * MPS is set to 64.
 *
 * This means if we are wanting to move >127 bytes of data, we need to
 * split the transactions up, but just doing one packet at a time does
 * not work (this may be an implicit DATA0 PID on first packet of the
 * transaction) and doing 2 packets is outside the controller's limits.
 *
 * If we try to lower the MPS size for EP0, then no transfers work properly
 * for EP0, and the system will fail basic enumeration. As no cause for this
 * has currently been found, we cannot support any large IN transfers for
 * EP0.
 */

/* LF2000 has a 0x900 word (32-bits each) SPRAM (single-port FIFO RAM)	*/
/* carve up into these suggested Max Packet Sizes		*/

#define EP0_MPS_LIMIT	64
#define EP1_MPS_LIMIT	512
#define EP2_MPS_LIMIT	512
#define EP3_MPS_LIMIT	1024
#define EP4_MPS_LIMIT	1024
#define EP5_MPS_LIMIT	64
#define EP6_MPS_LIMIT	64
#define EP7_MPS_LIMIT	64

/* LF2000 SPRAM data area size in 32 bit words	*/
#define	SPRAM_RX_FIFO_SIZE	541
#define	EP0_FIFO_SIZE		32
#define	EP1_FIFO_SIZE		256
#define	EP2_FIFO_SIZE		256
#define	EP3_FIFO_SIZE		512
#define	EP4_FIFO_SIZE		512
#define	EP5_FIFO_SIZE		32
#define	EP6_FIFO_SIZE		32
#define	EP7_FIFO_SIZE		32

/* LF2000 SPRAM base addresses (32 bits per word */
#define	RX_FIFO_BASE_ADDRESS	0
#define	EP0_FIFO_BASE_ADDRESS	( RX_FIFO_BASE_ADDRESS + SPRAM_RX_FIFO_SIZE)
#define	EP1_FIFO_BASE_ADDRESS	(EP0_FIFO_BASE_ADDRESS + EP0_FIFO_SIZE)
#define	EP2_FIFO_BASE_ADDRESS	(EP1_FIFO_BASE_ADDRESS + EP1_FIFO_SIZE)
#define	EP3_FIFO_BASE_ADDRESS	(EP2_FIFO_BASE_ADDRESS + EP2_FIFO_SIZE)
#define	EP4_FIFO_BASE_ADDRESS	(EP3_FIFO_BASE_ADDRESS + EP3_FIFO_SIZE)
#define	EP5_FIFO_BASE_ADDRESS	(EP4_FIFO_BASE_ADDRESS + EP4_FIFO_SIZE)
#define	EP6_FIFO_BASE_ADDRESS	(EP5_FIFO_BASE_ADDRESS + EP5_FIFO_SIZE)
#define	EP7_FIFO_BASE_ADDRESS	(EP6_FIFO_BASE_ADDRESS + EP6_FIFO_SIZE)

struct s3c_hsotg;
struct s3c_hsotg_req;

/**
 * struct s3c_hsotg_ep - driver endpoint definition.
 * @ep: The gadget layer representation of the endpoint.
 * @name: The driver generated name for the endpoint.
 * @queue: Queue of requests for this endpoint.
 * @parent: Reference back to the parent device structure.
 * @req: The current request that the endpoint is processing. This is
 *       used to indicate an request has been loaded onto the endpoint
 *       and has yet to be completed (maybe due to data move, or simply
 *	 awaiting an ack from the core all the data has been completed).
 * @debugfs: File entry for debugfs file for this endpoint.
 * @lock: State lock to protect contents of endpoint.
 * @dir_in: Set to true if this endpoint is of the IN direction, which
 *	    means that it is sending data to the Host.
 * @index: The index for the endpoint registers.
 * @name: The name array passed to the USB core.
 * @halted: Set if the endpoint has been halted.
 * @periodic: Set if this is a periodic ep, such as Interrupt
 * @sent_zlp: Set if we've sent a zero-length packet.
 * @total_data: The total number of data bytes done.
 * @fifo_size: The size of the FIFO (for periodic IN endpoints)
 * @fifo_load: The amount of data loaded into the FIFO (periodic IN)
 * @last_load: The offset of data for the last start of request.
 * @size_loaded: The last loaded size for DxEPTSIZE for periodic IN
 *
 * This is the driver's state for each registered endpoint, allowing it
 * to keep track of transactions that need doing. Each endpoint has a
 * lock to protect the state, to try and avoid using an overall lock
 * for the host controller as much as possible.
 *
 * For periodic IN endpoints, we have fifo_size and fifo_load to try
 * and keep track of the amount of data in the periodic FIFO for each
 * of these as we don't have a status register that tells us how much
 * is in each of them. (note, this may actually be useless information
 * as in shared-fifo mode periodic in acts like a single-frame packet
 * buffer than a fifo)
 */
struct s3c_hsotg_ep {
	struct usb_ep		ep;
	struct list_head	queue;
	struct s3c_hsotg	*parent;
	struct s3c_hsotg_req	*req;
	struct dentry		*debugfs;

#ifndef U_BOOT
	spinlock_t		lock;
#endif

	unsigned long		total_data;
	unsigned int		size_loaded;
	unsigned int		last_load;
	unsigned int		fifo_load;
	unsigned short		fifo_size;

	unsigned char		dir_in;
	unsigned char		index;

	unsigned int		halted:1;
	unsigned int		periodic:1;
	unsigned int		sent_zlp:1;

	char			name[10];
};

/* number of endpoints on LF2000 */
#define S3C_HSOTG_EPS	(7+1)	/* limit to 8 (7 + control) for the moment */

/**
 * struct s3c_hsotg - driver state.
 * @dev: The parent device supplied to the probe function
 * @driver: USB gadget driver
 * @plat: The platform specific configuration data.
 * @regs: The memory area mapped for accessing registers.
 * @regs_res: The resource that was allocated when claiming register space.
 * @irq: The IRQ number we are using
 * @dedicated_fifos: Set if the hardware has dedicated IN-EP fifos.
 * @debug_root: root directrory for debugfs.
 * @debug_file: main status file for debugfs.
 * @debug_fifo: FIFO status file for debugfs.
 * @ep0_reply: Request used for ep0 reply.
 * @ep0_buff: Buffer for EP0 reply data, if needed.
 * @ctrl_buff: Buffer for EP0 control requests.
 * @ctrl_req: Request for EP0 control packets.
 * @setup: NAK management for EP0 SETUP
 * @eps: The endpoints being supplied to the gadget framework
 */
struct s3c_hsotg {
	struct device		 *dev;
	struct usb_gadget_driver *driver;
	struct s3c_hsotg_plat	 *plat;
#ifndef U_BOOT
	struct input_dev	 *input;
#endif

	void __iomem		*regs;
	struct resource		*regs_res;
	int			irq;
	struct clk		*clk;

	unsigned int		dedicated_fifos:1;

	struct dentry		*debug_root;
	struct dentry		*debug_file;
	struct dentry		*debug_fifo;

	struct usb_request	*ep0_reply;
	struct usb_request	*ctrl_req;
	u8			ep0_buff[8];
	u8			ctrl_buff[8];

	bool			conn_state;
#ifndef U_BOOT
	struct timer_list	conn_state_timer;
#else
	u32			conn_event_timer;
	bool        enum_done;
	bool        was_wall_connected;
#endif

	struct usb_gadget	gadget;
	unsigned int		setup;
	struct s3c_hsotg_ep	eps[];
};

/**
 * struct s3c_hsotg_req - data transfer request
 * @req: The USB gadget request
 * @queue: The list of requests for the endpoint this is queued for.
 * @in_progress: Has already had size/packets written to core
 * @mapped: DMA buffer for this request has been mapped via dma_map_single().
 */
struct s3c_hsotg_req {
	struct usb_request	req;
	struct list_head	queue;
	unsigned char		in_progress;
	unsigned char		mapped;
};

/* conversion functions */
static inline struct s3c_hsotg_req *our_req(struct usb_request *req)
{
	return container_of(req, struct s3c_hsotg_req, req);
}

static inline struct s3c_hsotg_ep *our_ep(struct usb_ep *ep)
{
	return container_of(ep, struct s3c_hsotg_ep, ep);
}

static inline struct s3c_hsotg *to_hsotg(struct usb_gadget *gadget)
{
	return container_of(gadget, struct s3c_hsotg, gadget);
}

static inline void __orr32(void __iomem *ptr, u32 val)
{
	writel(readl(ptr) | val, ptr);
}

static inline void __bic32(void __iomem *ptr, u32 val)
{
	writel(readl(ptr) & ~val, ptr);
}

/* forward decleration of functions */
static void s3c_hsotg_dump(struct s3c_hsotg *hsotg);

/**
 * using_dma - return the DMA status of the driver.
 * @hsotg: The driver state.
 *
 * Return true if we're using DMA.
 *
 * Currently, we have the DMA support code worked into everywhere
 * that needs it, but the AMBA DMA implementation in the hardware can
 * only DMA from 32bit aligned addresses. This means that gadgets such
 * as the CDC Ethernet cannot work as they often pass packets which are
 * not 32bit aligned.
 *
 * Unfortunately the choice to use DMA or not is global to the controller
 * and seems to be only settable when the controller is being put through
 * a core reset. This means we either need to fix the gadgets to take
 * account of DMA alignment, or add bounce buffers (yuerk).
 *
 * Until this issue is sorted out, we always return 'false'.
 */
static inline bool using_dma(struct s3c_hsotg *hsotg)
{
	return false;	/* support is not complete */
}

/**
 * s3c_hsotg_en_gsint - enable one or more of the general interrupt
 * @hsotg: The device state
 * @ints: A bitmask of the interrupts to enable
 */
static void s3c_hsotg_en_gsint(struct s3c_hsotg *hsotg, u32 ints)
{
	u32 gsintmsk = readl(hsotg->regs + S3C_GINTMSK);
	u32 new_gsintmsk;

	new_gsintmsk = gsintmsk | ints;

	if (new_gsintmsk != gsintmsk) {
		dev_dbg(hsotg->dev, "gsintmsk now 0x%08x\n", new_gsintmsk);
		writel(new_gsintmsk, hsotg->regs + S3C_GINTMSK);
	}
}

/**
 * s3c_hsotg_disable_gsint - disable one or more of the general interrupt
 * @hsotg: The device state
 * @ints: A bitmask of the interrupts to enable
 */
static void s3c_hsotg_disable_gsint(struct s3c_hsotg *hsotg, u32 ints)
{
	u32 gsintmsk = readl(hsotg->regs + S3C_GINTMSK);
	u32 new_gsintmsk;

	new_gsintmsk = gsintmsk & ~ints;

	if (new_gsintmsk != gsintmsk)
		writel(new_gsintmsk, hsotg->regs + S3C_GINTMSK);
}

/**
 * s3c_hsotg_ctrl_epint - enable/disable an endpoint irq
 * @hsotg: The device state
 * @ep: The endpoint index
 * @dir_in: True if direction is in.
 * @en: The enable value, true to enable
 *
 * Set or clear the mask for an individual endpoint's interrupt
 * request.
 */
static void s3c_hsotg_ctrl_epint(struct s3c_hsotg *hsotg,
				 unsigned int ep, unsigned int dir_in,
				 unsigned int en)
{
	unsigned long flags;
	u32 bit = 1 << ep;
	u32 daint;

	if (!dir_in)
		bit <<= 16;

	local_irq_save(flags);
	daint = readl(hsotg->regs + S3C_DAINTMSK);
	if (en)
		daint |= bit;
	else
		daint &= ~bit;
	writel(daint, hsotg->regs + S3C_DAINTMSK);
	local_irq_restore(flags);
}

/**
 * s3c_hsotg_init_fifo - initialise non-periodic FIFOs
 * @hsotg: The device instance.
 */
static void s3c_hsotg_init_fifo(struct s3c_hsotg *hsotg)
{
	int timeout;
	u32 val;

	/* the ryu 2.6.24 release ahs
	   writel(0x1C0, hsotg->regs + S3C_GRXFSIZ);
	   writel(S3C_GNPTXFSIZ_NPTxFStAddr(0x200) |
		S3C_GNPTXFSIZ_NPTxFDep(0x1C0),
		hsotg->regs + S3C_GNPTXFSIZ);
	*/

	/* set LF2000 FIFO sizes */

	writel(SPRAM_RX_FIFO_SIZE, hsotg->regs + S3C_GRXFSIZ);
	writel(S3C_GNPTXFSIZ_NPTxFStAddr(EP0_FIFO_BASE_ADDRESS) |
	       S3C_GNPTXFSIZ_NPTxFDep(EP0_FIFO_SIZE),
	       hsotg->regs + S3C_GNPTXFSIZ);

	/* setup TX fifos 1 through 7 */
	writel(S3C_DPTXFSIZn_DPTxFSize(EP1_FIFO_SIZE) |
		S3C_DPTXFSIZn_DPTxFStAddr(EP1_FIFO_BASE_ADDRESS),
		hsotg->regs + S3C_DPTXFSIZn(1));

	writel(S3C_DPTXFSIZn_DPTxFSize(EP2_FIFO_SIZE) |
		S3C_DPTXFSIZn_DPTxFStAddr(EP2_FIFO_BASE_ADDRESS),
		hsotg->regs + S3C_DPTXFSIZn(2));

	writel(S3C_DPTXFSIZn_DPTxFSize(EP3_FIFO_SIZE) |
		S3C_DPTXFSIZn_DPTxFStAddr(EP3_FIFO_BASE_ADDRESS),
		hsotg->regs + S3C_DPTXFSIZn(3));

	writel(S3C_DPTXFSIZn_DPTxFSize(EP4_FIFO_SIZE) |
		S3C_DPTXFSIZn_DPTxFStAddr(EP4_FIFO_BASE_ADDRESS),
		hsotg->regs + S3C_DPTXFSIZn(4));

	writel(S3C_DPTXFSIZn_DPTxFSize(EP5_FIFO_SIZE) |
		S3C_DPTXFSIZn_DPTxFStAddr(EP5_FIFO_BASE_ADDRESS),
		hsotg->regs + S3C_DPTXFSIZn(5));

	writel(S3C_DPTXFSIZn_DPTxFSize(EP6_FIFO_SIZE) |
		S3C_DPTXFSIZn_DPTxFStAddr(EP6_FIFO_BASE_ADDRESS),
		hsotg->regs + S3C_DPTXFSIZn(6));

	writel(S3C_DPTXFSIZn_DPTxFSize(EP7_FIFO_SIZE) |
		S3C_DPTXFSIZn_DPTxFStAddr(EP7_FIFO_BASE_ADDRESS),
		hsotg->regs + S3C_DPTXFSIZn(7));

	/* according to p428 of the design guide, we need to ensure that
	 * all fifos are flushed before continuing */

	writel(S3C_GRSTCTL_TxFNum(0x10) | S3C_GRSTCTL_TxFFlsh |
	       S3C_GRSTCTL_RxFFlsh, hsotg->regs + S3C_GRSTCTL);

	/* wait until the fifos are both flushed */
	timeout = 100;
	while (1) {
		val = readl(hsotg->regs + S3C_GRSTCTL);

		if ((val & (S3C_GRSTCTL_TxFFlsh | S3C_GRSTCTL_RxFFlsh)) == 0)
			break;

		if (--timeout == 0) {
			dev_err(hsotg->dev,
				"%s: timeout flushing fifos (GRSTCTL=%08x)\n",
				__func__, val);
		}

		udelay(1);
	}

	dev_dbg(hsotg->dev, "FIFOs reset, timeout at %d\n", timeout);
}

/**
 * @ep: USB endpoint to allocate request for.
 * @flags: Allocation flags
 *
 * Allocate a new USB request structure appropriate for the specified endpoint
 */
static struct usb_request *s3c_hsotg_ep_alloc_request(struct usb_ep *ep,
						      gfp_t flags)
{
	struct s3c_hsotg_req *req;

	req = kzalloc(sizeof(struct s3c_hsotg_req), flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);

	req->req.dma = DMA_ADDR_INVALID;
	return &req->req;
}

/**
 * is_ep_periodic - return true if the endpoint is in periodic mode.
 * @hs_ep: The endpoint to query.
 *
 * Returns true if the endpoint is in periodic mode, meaning it is being
 * used for an Interrupt or ISO transfer.
 */
static inline int is_ep_periodic(struct s3c_hsotg_ep *hs_ep)
{
	return hs_ep->periodic;
}

/**
 * s3c_hsotg_unmap_dma - unmap the DMA memory being used for the request
 * @hsotg: The device state.
 * @hs_ep: The endpoint for the request
 * @hs_req: The request being processed.
 *
 * This is the reverse of s3c_hsotg_map_dma(), called for the completion
 * of a request to ensure the buffer is ready for access by the caller.
*/
static void s3c_hsotg_unmap_dma(struct s3c_hsotg *hsotg,
				struct s3c_hsotg_ep *hs_ep,
				struct s3c_hsotg_req *hs_req)
{
#ifndef U_BOOT
	struct usb_request *req = &hs_req->req;
#endif
	enum dma_data_direction dir;

	dir = hs_ep->dir_in ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

	/* ignore this if we're not moving any data */
	if (hs_req->req.length == 0)
		return;

#ifndef U_BOOT
	if (hs_req->mapped) {
		/* we mapped this, so unmap and remove the dma */

		dma_unmap_single(hsotg->dev, req->dma, req->length, dir);

		req->dma = DMA_ADDR_INVALID;
		hs_req->mapped = 0;
	} else {
		dma_sync_single_for_cpu(hsotg->dev, req->dma, req->length, dir);
	}
#endif
}

/**
 * s3c_hsotg_write_fifo - write packet Data to the TxFIFO
 * @hsotg: The controller state.
 * @hs_ep: The endpoint we're going to write for.
 * @hs_req: The request to write data for.
 *
 * This is called when the TxFIFO has some space in it to hold a new
 * transmission and we have something to give it. The actual setup of
 * the data size is done elsewhere, so all we have to do is to actually
 * write the data.
 *
 * The return value is zero if there is more space (or nothing was done)
 * otherwise -ENOSPC is returned if the FIFO space was used up.
 *
 * This routine is only needed for PIO
*/
static int s3c_hsotg_write_fifo(struct s3c_hsotg *hsotg,
				struct s3c_hsotg_ep *hs_ep,
				struct s3c_hsotg_req *hs_req)
{
	bool periodic = is_ep_periodic(hs_ep);
	u32 gnptxsts = readl(hsotg->regs + S3C_GNPTXSTS);
	int buf_pos = hs_req->req.actual;
	int to_write = hs_ep->size_loaded;
	void *data;
	int can_write;
	int pkt_round;

	// FIXME: sesters debugging
	dev_dbg(hsotg->dev, "%s.%d: left=%d, load=%d, fifo=%d, size %d\n",
			__func__, __LINE__,
			S3C_DxEPTSIZ_XferSize_GET(readl(hsotg->regs + S3C_DIEPTSIZ(hs_ep->index))),
			hs_ep->size_loaded, hs_ep->fifo_load, hs_ep->fifo_size);

	to_write -= (buf_pos - hs_ep->last_load);


	/* if there's nothing to write, get out early */
	if (to_write == 0)
		return 0;

	if (periodic && !hsotg->dedicated_fifos) {
		u32 epsize = readl(hsotg->regs + S3C_DIEPTSIZ(hs_ep->index));
		int size_left;
		int size_done;

		/* work out how much data was loaded so we can calculate
		 * how much data is left in the fifo. */

		size_left = S3C_DxEPTSIZ_XferSize_GET(epsize);

		/* if shared fifo, we cannot write anything until the
		 * previous data has been completely sent.
		 */
		if (hs_ep->fifo_load != 0) {
			s3c_hsotg_en_gsint(hsotg, S3C_GINTSTS_PTxFEmp);
			return -ENOSPC;
		}

		dev_dbg(hsotg->dev, "%s.%d: left=%d, load=%d, fifo=%d, size %d\n",
			__func__, __LINE__,size_left,
			hs_ep->size_loaded, hs_ep->fifo_load, hs_ep->fifo_size);

		/* how much of the data has moved */
		size_done = hs_ep->size_loaded - size_left;

		/* how much data is left in the fifo */
		can_write = hs_ep->fifo_load - size_done;
		dev_dbg(hsotg->dev, "%s: => can_write1=%d\n",
			__func__, can_write);

		can_write = hs_ep->fifo_size - can_write;
		dev_dbg(hsotg->dev, "%s: => can_write2=%d\n",
			__func__, can_write);

		if (can_write <= 0) {
			s3c_hsotg_en_gsint(hsotg, S3C_GINTSTS_PTxFEmp);
			return -ENOSPC;
		}
	} else if (hsotg->dedicated_fifos && hs_ep->index != 0) {
		can_write = readl(hsotg->regs + S3C_DTXFSTS(hs_ep->index));

		can_write &= 0xffff;
		can_write *= 4;
	} else {
		if (S3C_GNPTXSTS_NPTxQSpcAvail_GET(gnptxsts) == 0) {
			dev_dbg(hsotg->dev,
				"%s: no queue slots available (0x%08x)\n",
				__func__, gnptxsts);

			s3c_hsotg_en_gsint(hsotg, S3C_GINTSTS_NPTxFEmp);
			return -ENOSPC;
		}

		can_write = S3C_GNPTXSTS_NPTxFSpcAvail_GET(gnptxsts);
		can_write *= 4;	/* fifo size is in 32bit quantities. */
	}

	dev_dbg(hsotg->dev, "%s: GNPTXSTS=%08x, can=%d, to=%d, mps %d\n",
		 __func__, gnptxsts, can_write, to_write, hs_ep->ep.maxpacket);

	/* limit to 512 bytes of data, it seems at least on the non-periodic
	 * FIFO, requests of >512 cause the endpoint to get stuck with a
	 * fragment of the end of the transfer in it.
	 */
	if (can_write > 512)
		can_write = 512;

	/* limit the write to one max-packet size worth of data, but allow
	 * the transfer to return that it did not run out of fifo space
	 * doing it. */
	if (to_write > hs_ep->ep.maxpacket) {
		to_write = hs_ep->ep.maxpacket;

		s3c_hsotg_en_gsint(hsotg,
				   periodic ? S3C_GINTSTS_PTxFEmp :
				   S3C_GINTSTS_NPTxFEmp);
	}

	/* see if we can write data */

	if (to_write > can_write) {
		to_write = can_write;
		pkt_round = to_write % hs_ep->ep.maxpacket;

		/* Not sure, but we probably shouldn't be writing partial
		 * packets into the FIFO, so round the write down to an
		 * exact number of packets.
		 *
		 * Note, we do not currently check to see if we can ever
		 * write a full packet or not to the FIFO.
		 */

		if (pkt_round)
			to_write -= pkt_round;

		/* enable correct FIFO interrupt to alert us when there
		 * is more room left. */

		s3c_hsotg_en_gsint(hsotg,
				   periodic ? S3C_GINTSTS_PTxFEmp :
				   S3C_GINTSTS_NPTxFEmp);
	}

	dev_dbg(hsotg->dev, "write %d/%d, can_write %d, done %d\n",
		 to_write, hs_req->req.length, can_write, buf_pos);

	if (to_write <= 0)
		return -ENOSPC;

	hs_req->req.actual = buf_pos + to_write;
	hs_ep->total_data += to_write;

	if (periodic)
		hs_ep->fifo_load += to_write;

	to_write = DIV_ROUND_UP(to_write, 4);
	data = hs_req->req.buf + buf_pos;

	writesl(hsotg->regs + S3C_EPFIFO(hs_ep->index), data, to_write);

	return (to_write >= can_write) ? -ENOSPC : 0;
}

/**
 * get_ep_limit - get the maximum data legnth for this endpoint
 * @hs_ep: The endpoint
 *
 * Return the maximum data that can be queued in one go on a given endpoint
 * so that transfers that are too long can be split.
 */
static unsigned get_ep_limit(struct s3c_hsotg_ep *hs_ep)
{
	int index = hs_ep->index;
	unsigned maxsize;
	unsigned maxpkt;

	if (index != 0) {
		maxsize = S3C_DxEPTSIZ_XferSize_LIMIT + 1;
		maxpkt = S3C_DxEPTSIZ_PktCnt_LIMIT + 1;
	} else {
		maxsize = 64+64;
		if (hs_ep->dir_in)
			maxpkt = S3C_DIEPTSIZ0_PktCnt_LIMIT + 1;
		else
			maxpkt = 2;
	}

	/* we made the constant loading easier above by using +1 */
	maxpkt--;
	maxsize--;

	/* constrain by packet count if maxpkts*pktsize is greater
	 * than the length register size. */

	if ((maxpkt * hs_ep->ep.maxpacket) < maxsize)
		maxsize = maxpkt * hs_ep->ep.maxpacket;

	return maxsize;
}

/**
 * s3c_hsotg_start_req - start a USB request from an endpoint's queue
 * @hsotg: The controller state.
 * @hs_ep: The endpoint to process a request for
 * @hs_req: The request to start.
 * @continuing: True if we are doing more for the current request.
 *
 * Start the given request running by setting the endpoint registers
 * appropriately, and writing any data to the FIFOs.
 */
static void s3c_hsotg_start_req(struct s3c_hsotg *hsotg,
				struct s3c_hsotg_ep *hs_ep,
				struct s3c_hsotg_req *hs_req,
				bool continuing)
{
	struct usb_request *ureq = &hs_req->req;
	int index = hs_ep->index;
	int dir_in = hs_ep->dir_in;
	u32 epctrl_reg;
	u32 epsize_reg;
	u32 epsize;
	u32 ctrl;
	unsigned length;
	unsigned packets;
	unsigned maxreq;

	if (index != 0) {
		if (hs_ep->req && !continuing) {
			dev_err(hsotg->dev, "%s: active request\n", __func__);
			WARN_ON(1);
			return;
		} else if (hs_ep->req != hs_req && continuing) {
			dev_err(hsotg->dev,
				"%s: continue different req\n", __func__);
			WARN_ON(1);
			return;
		}
	}

	epctrl_reg = dir_in ? S3C_DIEPCTL(index) : S3C_DOEPCTL(index);
	epsize_reg = dir_in ? S3C_DIEPTSIZ(index) : S3C_DOEPTSIZ(index);

	dev_dbg(hsotg->dev, "%s.%d: DxEPCTL=0x%08x, ep %d, dir %s\n",
		__func__, __LINE__, readl(hsotg->regs + epctrl_reg), index,
		hs_ep->dir_in ? "in" : "out");

	/* If endpoint is stalled, we will restart request later */
	ctrl = readl(hsotg->regs + epctrl_reg);

	if (ctrl & S3C_DxEPCTL_Stall) {
		dev_warn(hsotg->dev, "%s: ep%d is stalled\n", __func__, index);
		return;
	}

	length = ureq->length - ureq->actual;

	dev_dbg(hsotg->dev, "ureq->length:%d ureq->actual:%d\n",
		ureq->length, ureq->actual);
	if (0)
		dev_dbg(hsotg->dev,
			"REQ buf %p len %d dma 0x%08x noi=%d zp=%d snok=%d\n",
			ureq->buf, length, ureq->dma,
			ureq->no_interrupt, ureq->zero, ureq->short_not_ok);

	maxreq = get_ep_limit(hs_ep);
	if (length > maxreq) {
		int round = maxreq % hs_ep->ep.maxpacket;

		dev_dbg(hsotg->dev, "%s: length %d, max-req %d, r %d\n",
			__func__, length, maxreq, round);

		/* round down to multiple of packets */
		if (round)
			maxreq -= round;

		length = maxreq;
	}

	if (length)
		packets = DIV_ROUND_UP(length, hs_ep->ep.maxpacket);
	else
		packets = 1;	/* send one packet if length is zero. */

	if (dir_in && index != 0)
		epsize = S3C_DxEPTSIZ_MC(1);
	else
		epsize = 0;

/* DL 2/7/2012 ZLP on ep1+ is now handled in s3c_hsotg_ep_queue */
#ifdef U_BOOT
	if (index != 0 && ureq->zero) {
		/* test for the packets being exactly right for the
		 * transfer */

		if (length == (packets * hs_ep->ep.maxpacket))
			packets++;
	}
#endif

	epsize |= S3C_DxEPTSIZ_PktCnt(packets);
	epsize |= S3C_DxEPTSIZ_XferSize(length);

	dev_dbg(hsotg->dev, "%s: %d@%d/%d, 0x%08x => 0x%08x\n",
		__func__, packets, length, ureq->length, epsize, epsize_reg);

	/* store the request as the current one we're doing */
	hs_ep->req = hs_req;

	/* write size / packets */
	writel(epsize, hsotg->regs + epsize_reg);

	if (using_dma(hsotg)) {
		unsigned int dma_reg;

		/* write DMA address to control register, buffer already
		 * synced by s3c_hsotg_ep_queue().  */

		dma_reg = dir_in ? S3C_DIEPDMA(index) : S3C_DOEPDMA(index);
		writel(ureq->dma, hsotg->regs + dma_reg);

		dev_dbg(hsotg->dev, "%s: 0x%08x => 0x%08x\n",
			__func__, ureq->dma, dma_reg);
	}

	ctrl |= S3C_DxEPCTL_EPEna;	/* ensure ep enabled */
	ctrl |= S3C_DxEPCTL_USBActEp;
	ctrl |= S3C_DxEPCTL_CNAK;	/* clear NAK set by core */

	dev_dbg(hsotg->dev, "%s: DxEPCTL=0x%08x\n", __func__, ctrl);
	writel(ctrl, hsotg->regs + epctrl_reg);

	/* set these, it seems that DMA support increments past the end
	 * of the packet buffer so we need to calculate the length from
	 * this information. */
	hs_ep->size_loaded = length;
	hs_ep->last_load = ureq->actual;

	if (dir_in && !using_dma(hsotg)) {
		/* set these anyway, we may need them for non-periodic in */
		hs_ep->fifo_load = 0;

		s3c_hsotg_write_fifo(hsotg, hs_ep, hs_req);
	}

	/* clear the INTknTXFEmpMsk when we start request, more as a aide
	 * to debugging to see what is going on. */
	if (dir_in)
		writel(S3C_DIEPMSK_INTknTXFEmpMsk,
		       hsotg->regs + S3C_DIEPINT(index));

	/* Note, trying to clear the NAK here causes problems with transmit
	 * on the S3C6400 ending up with the TXFIFO becoming full. */

	/* check ep is enabled */
	if (!(readl(hsotg->regs + epctrl_reg) & S3C_DxEPCTL_EPEna))
		dev_warn(hsotg->dev,
			 "ep%d: failed to become enabled (DxEPCTL=0x%08x)?\n",
			 index, readl(hsotg->regs + epctrl_reg));

	dev_dbg(hsotg->dev, "%s: DxEPCTL=0x%08x\n",
		__func__, readl(hsotg->regs + epctrl_reg));
}

/**
 * s3c_hsotg_map_dma - map the DMA memory being used for the request
 * @hsotg: The device state.
 * @hs_ep: The endpoint the request is on.
 * @req: The request being processed.
 *
 * We've been asked to queue a request, so ensure that the memory buffer
 * is correctly setup for DMA. If we've been passed an extant DMA address
 * then ensure the buffer has been synced to memory. If our buffer has no
 * DMA memory, then we map the memory and mark our request to allow us to
 * cleanup on completion.
*/
static int s3c_hsotg_map_dma(struct s3c_hsotg *hsotg,
			     struct s3c_hsotg_ep *hs_ep,
			     struct usb_request *req)
{
	enum dma_data_direction dir;
	struct s3c_hsotg_req *hs_req = our_req(req);

	dir = hs_ep->dir_in ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

	/* if the length is zero, ignore the DMA data */
	if (hs_req->req.length == 0)
		return 0;

#ifndef U_BOOT
	if (req->dma == DMA_ADDR_INVALID) {
		dma_addr_t dma;

		dma = dma_map_single(hsotg->dev, req->buf, req->length, dir);

		if (unlikely(dma_mapping_error(hsotg->dev, dma)))
			goto dma_error;

		if (dma & 3) {
			dev_err(hsotg->dev, "%s: unaligned dma buffer\n",
				__func__);

			dma_unmap_single(hsotg->dev, dma, req->length, dir);
			return -EINVAL;
		}

		hs_req->mapped = 1;
		req->dma = dma;
	} else {
		dma_sync_single_for_cpu(hsotg->dev, req->dma, req->length, dir);
		hs_req->mapped = 0;
	}

#endif
	return 0;

#ifndef U_BOOT
dma_error:
	dev_err(hsotg->dev, "%s: failed to map buffer %p, %d bytes\n",
		__func__, req->buf, req->length);

	return -EIO;
#endif
}

static int s3c_hsotg_ep_queue(struct usb_ep *ep, struct usb_request *req,
			      gfp_t gfp_flags)
{
	struct s3c_hsotg_req *hs_req = our_req(req);
	struct s3c_hsotg_ep *hs_ep = our_ep(ep);
	struct s3c_hsotg *hs = hs_ep->parent;
#ifndef U_BOOT
	unsigned long irqflags;
	struct usb_request* zlp_req;
#endif
	bool first;

	dev_dbg(hs->dev, "%s: req %p: %d@%p, noi=%d, zero=%d, snok=%d\n",
		ep->name, req, req->length, req->buf, req->no_interrupt,
		req->zero, req->short_not_ok);

	/* initialise status of the request */
	INIT_LIST_HEAD(&hs_req->queue);
	req->actual = 0;
	req->status = -EINPROGRESS;

	/* if we're using DMA, sync the buffers as necessary */
	if (using_dma(hs)) {
		int ret = s3c_hsotg_map_dma(hs, hs_ep, req);
		if (ret)
			return ret;
	}

	spin_lock_irqsave(&hs_ep->lock, irqflags);

	first = list_empty(&hs_ep->queue);
	list_add_tail(&hs_req->queue, &hs_ep->queue);

#ifndef U_BOOT	
	/* If we need a zlp on a non-control endpoint, queue it */
	if(hs_ep->index > 0 && hs_req->req.zero)
	{
		if( (hs_req->req.length % hs_ep->ep.maxpacket) == 0 )
		{
			/* Queue up a new zlp */
			zlp_req = s3c_hsotg_ep_alloc_request(ep, gfp_flags);
			zlp_req->buf = NULL;
			zlp_req->length = 0;
			zlp_req->zero = 0;
			zlp_req->short_not_ok = 0;
			s3c_hsotg_ep_queue(ep, zlp_req, gfp_flags);
		}
	}
#endif
	if (first)
		s3c_hsotg_start_req(hs, hs_ep, hs_req, false);

	spin_unlock_irqrestore(&hs_ep->lock, irqflags);

	return 0;
}

static void s3c_hsotg_ep_free_request(struct usb_ep *ep,
				      struct usb_request *req)
{
	struct s3c_hsotg_req *hs_req = our_req(req);

	kfree(hs_req);
}

/**
 * s3c_hsotg_complete_oursetup - setup completion callback
 * @ep: The endpoint the request was on.
 * @req: The request completed.
 *
 * Called on completion of any requests the driver itself
 * submitted that need cleaning up.
 */
static void s3c_hsotg_complete_oursetup(struct usb_ep *ep,
					struct usb_request *req)
{
#ifndef U_BOOT
	struct s3c_hsotg_ep *hs_ep = our_ep(ep);
	struct s3c_hsotg *hsotg = hs_ep->parent;
#endif

	dev_dbg(s3c_hsotg->dev, "%s: ep %p, req %p\n", __func__, ep, req);

	s3c_hsotg_ep_free_request(ep, req);
}

/**
 * ep_from_windex - convert control wIndex value to endpoint
 * @hsotg: The driver state.
 * @windex: The control request wIndex field (in host order).
 *
 * Convert the given wIndex into a pointer to an driver endpoint
 * structure, or return NULL if it is not a valid endpoint.
*/
static struct s3c_hsotg_ep *ep_from_windex(struct s3c_hsotg *hsotg,
					   u32 windex)
{
	struct s3c_hsotg_ep *ep = &hsotg->eps[windex & 0x7F];
	int dir = (windex & USB_DIR_IN) ? 1 : 0;
	int idx = windex & 0x7F;

	if (windex >= 0x100)
		return NULL;

	if (idx >= S3C_HSOTG_EPS)
		return NULL;

	if (idx && ep->dir_in != dir)
		return NULL;

	return ep;
}

/**
 * s3c_hsotg_send_reply - send reply to control request
 * @hsotg: The device state
 * @ep: Endpoint 0
 * @buff: Buffer for request
 * @length: Length of reply.
 *
 * Create a request and queue it on the given endpoint. This is useful as
 * an internal method of sending replies to certain control requests, etc.
 */
static int s3c_hsotg_send_reply(struct s3c_hsotg *hsotg,
				struct s3c_hsotg_ep *ep,
				void *buff,
				int length)
{
	struct usb_request *req;
	int ret;

	dev_dbg(hsotg->dev, "%s: buff %p, len %d\n", __func__, buff, length);

	req = s3c_hsotg_ep_alloc_request(&ep->ep, GFP_ATOMIC);
	hsotg->ep0_reply = req;
	if (!req) {
		dev_warn(hsotg->dev, "%s: cannot alloc req\n", __func__);
		return -ENOMEM;
	}

	req->buf = hsotg->ep0_buff;
	req->length = length;
	req->zero = 1; /* always do zero-length final transfer */
	req->complete = s3c_hsotg_complete_oursetup;

	if (length)
		memcpy(req->buf, buff, length);
	else
		ep->sent_zlp = 1;

	ret = s3c_hsotg_ep_queue(&ep->ep, req, GFP_ATOMIC);
	if (ret) {
		dev_warn(hsotg->dev, "%s: cannot queue req\n", __func__);
		return ret;
	}

	return 0;
}

/**
 * s3c_hsotg_process_req_status - process request GET_STATUS
 * @hsotg: The device state
 * @ctrl: USB control request
 */
static int s3c_hsotg_process_req_status(struct s3c_hsotg *hsotg,
					struct usb_ctrlrequest *ctrl)
{
	struct s3c_hsotg_ep *ep0 = &hsotg->eps[0];
	struct s3c_hsotg_ep *ep;
	__le16 reply;
	int ret;

	dev_dbg(hsotg->dev, "%s: USB_REQ_GET_STATUS\n", __func__);

	if (!ep0->dir_in) {
		dev_warn(hsotg->dev, "%s: direction out?\n", __func__);
		return -EINVAL;
	}

	switch (ctrl->bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_DEVICE:
#if 1		/* report if powered from bus or self-powered */
		if (have_usb_power_option())    /* bit 0 => self powered  */
			reply = cpu_to_le16(0); /* bit 1 => remote wakeup */
		else
			reply = cpu_to_le16(1);
#else
		reply = cpu_to_le16(0); /* bit 0 => self powered,
					 * bit 1 => remote wakeup */
#endif
		break;

	case USB_RECIP_INTERFACE:
		/* currently, the data result should be zero */
		reply = cpu_to_le16(0);
		break;

	case USB_RECIP_ENDPOINT:
		ep = ep_from_windex(hsotg, le16_to_cpu(ctrl->wIndex));
		if (!ep)
			return -ENOENT;

		reply = cpu_to_le16(ep->halted ? 1 : 0);
		break;

	default:
		return 0;
	}

	if (le16_to_cpu(ctrl->wLength) != 2)
		return -EINVAL;

	ret = s3c_hsotg_send_reply(hsotg, ep0, &reply, 2);
	if (ret) {
		dev_err(hsotg->dev, "%s: failed to send reply\n", __func__);
		return ret;
	}

	return 1;
}

static int s3c_hsotg_ep_sethalt(struct usb_ep *ep, int value);

/**
 * get_ep_head - return the first request on the endpoint
 * @hs_ep: The controller endpoint to get
 *
 * Get the first request on the endpoint.
 */
static struct s3c_hsotg_req *get_ep_head(struct s3c_hsotg_ep *hs_ep)
{
	if (list_empty(&hs_ep->queue))
		return NULL;

	return list_first_entry(&hs_ep->queue, struct s3c_hsotg_req, queue);
}

/**
 * s3c_hsotg_process_req_feature - process request {SET,CLEAR}_FEATURE
 * @hsotg: The device state
 * @ctrl: USB control request
 */
static int s3c_hsotg_process_req_feature(struct s3c_hsotg *hsotg,
					 struct usb_ctrlrequest *ctrl)
{
	struct s3c_hsotg_ep *ep0 = &hsotg->eps[0];
	struct s3c_hsotg_req *hs_req;
	bool restart;
	bool set = (ctrl->bRequest == USB_REQ_SET_FEATURE);
	struct s3c_hsotg_ep *ep;
	int ret;

	dev_dbg(hsotg->dev, "%s: %s_FEATURE\n",
		__func__, set ? "SET" : "CLEAR");

	if (ctrl->bRequestType == USB_RECIP_ENDPOINT) {
		ep = ep_from_windex(hsotg, le16_to_cpu(ctrl->wIndex));
		if (!ep) {
			dev_dbg(hsotg->dev, "%s: no endpoint for 0x%04x\n",
				__func__, le16_to_cpu(ctrl->wIndex));
			return -ENOENT;
		}

		switch (le16_to_cpu(ctrl->wValue)) {
		case USB_ENDPOINT_HALT:
			s3c_hsotg_ep_sethalt(&ep->ep, set);
#if 1	/* 16feb12 */
			ep->halted = (set ? 1 : 0);
#endif

			ret = s3c_hsotg_send_reply(hsotg, ep0, NULL, 0);
			if (ret) {
				dev_err(hsotg->dev,
					"%s: failed to send reply\n", __func__);
				return ret;
			}

			if (!set) {
				/*
				 * If we have request in progress,
				 * then complete it
				 */
				if (ep->req) {
					hs_req = ep->req;
					ep->req = NULL;
					list_del_init(&hs_req->queue);
					hs_req->req.complete(&ep->ep,
							     &hs_req->req);
				}

				/* If we have pending request, then start it */
				restart = !list_empty(&ep->queue);
				if (restart) {
					hs_req = get_ep_head(ep);
					s3c_hsotg_start_req(hsotg, ep,
							    hs_req, false);
				}
			}

			break;

		default:
			return -ENOENT;
		}
	} else
		return -ENOENT;  /* currently only deal with endpoint */

	return 1;
}

static void s3c_hsotg_enqueue_setup(struct s3c_hsotg *hsotg);

/**
 * s3c_hsotg_process_control - process a control request
 * @hsotg: The device state
 * @ctrl: The control request received
 *
 * The controller has received the SETUP phase of a control request, and
 * needs to work out what to do next (and whether to pass it on to the
 * gadget driver).
 */
static void s3c_hsotg_process_control(struct s3c_hsotg *hsotg,
				      struct usb_ctrlrequest *ctrl)
{
	struct s3c_hsotg_ep *ep0 = &hsotg->eps[0];
	int ret = 0;
	u32 dcfg;

	ep0->sent_zlp = 0;

	dev_dbg(hsotg->dev, "ctrl Req=%02x, Type=%02x, V=%04x, L=%04x\n",
		 ctrl->bRequest, ctrl->bRequestType,
		 ctrl->wValue, ctrl->wLength);

	/* record the direction of the request, for later use when enquing
	 * packets onto EP0. */

	ep0->dir_in = (ctrl->bRequestType & USB_DIR_IN) ? 1 : 0;
	dev_dbg(hsotg->dev, "ctrl: dir_in=%d\n", ep0->dir_in);

	/* if we've no data with this request, then the last part of the
	 * transaction is going to implicitly be IN. */
	if (ctrl->wLength == 0)
		ep0->dir_in = 1;

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD) {
		switch (ctrl->bRequest) {
		case USB_REQ_SET_ADDRESS:
			dcfg = readl(hsotg->regs + S3C_DCFG);
			dcfg &= ~S3C_DCFG_DevAddr_MASK;
			dcfg |= ctrl->wValue << S3C_DCFG_DevAddr_SHIFT;
			writel(dcfg, hsotg->regs + S3C_DCFG);

			dev_info(hsotg->dev, "new address %d\n", ctrl->wValue);

//FIXME;  sesters  remove for testing
//#ifdef U_BOOT
#if 0
			/* show connected; stop connect bounce and not use timer */
			hsotg->conn_state = true;
#endif

			ret = s3c_hsotg_send_reply(hsotg, ep0, NULL, 0);
			return;

		case USB_REQ_GET_STATUS:
			ret = s3c_hsotg_process_req_status(hsotg, ctrl);
			break;

		case USB_REQ_CLEAR_FEATURE:
		case USB_REQ_SET_FEATURE:
			ret = s3c_hsotg_process_req_feature(hsotg, ctrl);
			break;
		}
	}

	/* as a fallback, try delivering it to the driver to deal with */

	if (ret == 0 && hsotg->driver) {
		ret = hsotg->driver->setup(&hsotg->gadget, ctrl);
		if (ret < 0)
			dev_dbg(hsotg->dev, "driver->setup() ret %d\n", ret);
	}

	/* the request is either unhandlable, or is not formatted correctly
	 * so respond with a STALL for the status stage to indicate failure.
	 */

	if (ret < 0) {
		u32 reg;
		u32 ctrl;

		dev_dbg(hsotg->dev, "ep0 stall (dir=%d)\n", ep0->dir_in);
		reg = (ep0->dir_in) ? S3C_DIEPCTL0 : S3C_DOEPCTL0;

		/* S3C_DxEPCTL_Stall will be cleared by EP once it has
		 * taken effect, so no need to clear later. */

		ctrl = readl(hsotg->regs + reg);
		ctrl |= S3C_DxEPCTL_Stall;
		ctrl |= S3C_DxEPCTL_CNAK;
		writel(ctrl, hsotg->regs + reg);

		dev_dbg(hsotg->dev,
			"written DxEPCTL=0x%08x to %08x (DxEPCTL=0x%08x)\n",
			ctrl, reg, readl(hsotg->regs + reg));

		/* don't believe we need to anything more to get the EP
		 * to reply with a STALL packet */
#if 1	/* 16feb12 After a stall, we must queue a request for next packet */
	/*         since ep0 must always be ready to accept a SETUP */
                s3c_hsotg_enqueue_setup(hsotg);
#endif
	}
}


/**
 * s3c_hsotg_complete_setup - completion of a setup transfer
 * @ep: The endpoint the request was on.
 * @req: The request completed.
 *
 * Called on completion of any requests the driver itself submitted for
 * EP0 setup packets
 */
static void s3c_hsotg_complete_setup(struct usb_ep *ep,
				     struct usb_request *req)
{
	struct s3c_hsotg_ep *hs_ep = our_ep(ep);
	struct s3c_hsotg *hsotg = hs_ep->parent;

	if (req->status < 0) {
		dev_dbg(hsotg->dev, "%s: failed %d\n", __func__, req->status);
		return;
	}

	if (req->actual == 0)
		s3c_hsotg_enqueue_setup(hsotg);
	else
		s3c_hsotg_process_control(hsotg, req->buf);
}

/**
 * s3c_hsotg_enqueue_setup - start a request for EP0 packets
 * @hsotg: The device state.
 *
 * Enqueue a request on EP0 if necessary to received any SETUP packets
 * received from the host.
 */
static void s3c_hsotg_enqueue_setup(struct s3c_hsotg *hsotg)
{
	struct usb_request *req = hsotg->ctrl_req;
	struct s3c_hsotg_req *hs_req = our_req(req);
	int ret;

	dev_dbg(hsotg->dev, "%s: queueing setup request\n", __func__);

	req->zero = 0;
	req->length = 8;
	req->buf = hsotg->ctrl_buff;
	req->complete = s3c_hsotg_complete_setup;

	if (!list_empty(&hs_req->queue)) {
		dev_dbg(hsotg->dev, "%s already queued???\n", __func__);
		return;
	}

	hsotg->eps[0].dir_in = 0;

	ret = s3c_hsotg_ep_queue(&hsotg->eps[0].ep, req, GFP_ATOMIC);
	if (ret < 0) {
		dev_err(hsotg->dev, "%s: failed queue (%d)\n", __func__, ret);
		/* Don't think there's much we can do other than watch the
		 * driver fail. */
	}
}

/**
 * s3c_hsotg_complete_request - complete a request given to us
 * @hsotg: The device state.
 * @hs_ep: The endpoint the request was on.
 * @hs_req: The request to complete.
 * @result: The result code (0 => Ok, otherwise errno)
 *
 * The given request has finished, so call the necessary completion
 * if it has one and then look to see if we can start a new request
 * on the endpoint.
 *
 * Note, expects the ep to already be locked as appropriate.
*/
static void s3c_hsotg_complete_request(struct s3c_hsotg *hsotg,
				       struct s3c_hsotg_ep *hs_ep,
				       struct s3c_hsotg_req *hs_req,
				       int result)
{
	bool restart;

	if (!hs_req) {
		dev_dbg(hsotg->dev, "%s: nothing to complete?\n", __func__);
		return;
	}

	dev_dbg(hsotg->dev, "complete: ep %p %s, req %p, %d => %p\n",
		hs_ep, hs_ep->ep.name, hs_req, result, hs_req->req.complete);

	/* only replace the status if we've not already set an error
	 * from a previous transaction */

	if (hs_req->req.status == -EINPROGRESS)
		hs_req->req.status = result;

	hs_ep->req = NULL;
	list_del_init(&hs_req->queue);

	if (using_dma(hsotg))
		s3c_hsotg_unmap_dma(hsotg, hs_ep, hs_req);

	/* call the complete request with the locks off, just in case the
	 * request tries to queue more work for this endpoint. */

	if (hs_req->req.complete) {
		spin_unlock(&hs_ep->lock);
		hs_req->req.complete(&hs_ep->ep, &hs_req->req);
		spin_lock(&hs_ep->lock);
	}

	/* Look to see if there is anything else to do. Note, the completion
	 * of the previous request may have caused a new request to be started
	 * so be careful when doing this. */

	if (!hs_ep->req && result >= 0) {
		restart = !list_empty(&hs_ep->queue);
		if (restart) {
			hs_req = get_ep_head(hs_ep);
			s3c_hsotg_start_req(hsotg, hs_ep, hs_req, false);
		}
	}
}

/**
 * s3c_hsotg_complete_request_lock - complete a request given to us (locked)
 * @hsotg: The device state.
 * @hs_ep: The endpoint the request was on.
 * @hs_req: The request to complete.
 * @result: The result code (0 => Ok, otherwise errno)
 *
 * See s3c_hsotg_complete_request(), but called with the endpoint's
 * lock held.
*/
static void s3c_hsotg_complete_request_lock(struct s3c_hsotg *hsotg,
					    struct s3c_hsotg_ep *hs_ep,
					    struct s3c_hsotg_req *hs_req,
					    int result)
{
#ifndef U_BOOT
	unsigned long flags;
#endif

	spin_lock_irqsave(&hs_ep->lock, flags);
	s3c_hsotg_complete_request(hsotg, hs_ep, hs_req, result);
	spin_unlock_irqrestore(&hs_ep->lock, flags);
}

/**
 * s3c_hsotg_rx_data - receive data from the FIFO for an endpoint
 * @hsotg: The device state.
 * @ep_idx: The endpoint index for the data
 * @size: The size of data in the fifo, in bytes
 *
 * The FIFO status shows there is data to read from the FIFO for a given
 * endpoint, so sort out whether we need to read the data into a request
 * that has been made for that endpoint.
 */
static void s3c_hsotg_rx_data(struct s3c_hsotg *hsotg, int ep_idx, int size)
{
	struct s3c_hsotg_ep *hs_ep = &hsotg->eps[ep_idx];
	struct s3c_hsotg_req *hs_req = hs_ep->req;
	void __iomem *fifo = hsotg->regs + S3C_EPFIFO(ep_idx);
	int to_read;
	int max_req;
	int read_ptr;

	if (!hs_req) {
		u32 epctl = readl(hsotg->regs + S3C_DOEPCTL(ep_idx));
		int ptr;

		dev_warn(hsotg->dev,
			 "%s: FIFO %d bytes on ep%d but no req (DxEPCTl=0x%08x)\n",
			 __func__, size, ep_idx, epctl);

		/* dump the data from the FIFO, we've nothing we can do */
		for (ptr = 0; ptr < size; ptr += 4) {
			dev_warn(hsotg->dev,"%s: BYTES: 0x%8.8X ",
			__func__, readl(fifo));
		}
		dev_warn(hsotg->dev, "\n");

		return;
	}

	spin_lock(&hs_ep->lock);

	to_read = size;
	read_ptr = hs_req->req.actual;
	max_req = hs_req->req.length - read_ptr;

	dev_dbg(hsotg->dev, "%s: read %d/%d, done %d/%d\n",
		__func__, to_read, max_req, read_ptr, hs_req->req.length);

	if (to_read > max_req) {
		/* more data appeared than we where willing
		 * to deal with in this request.
		 */

		/* currently we don't deal this */
		WARN_ON_ONCE(1);
	}

	hs_ep->total_data += to_read;
	hs_req->req.actual += to_read;
	to_read = DIV_ROUND_UP(to_read, 4);

	/* note, we might over-write the buffer end by 3 bytes depending on
	 * alignment of the data. */
	readsl(fifo, hs_req->req.buf + read_ptr, to_read);

	spin_unlock(&hs_ep->lock);
}

/**
 * s3c_hsotg_send_zlp - send zero-length packet on control endpoint
 * @hsotg: The device instance
 * @req: The request currently on this endpoint
 *
 * Generate a zero-length IN packet request for terminating a SETUP
 * transaction.
 *
 * Note, since we don't write any data to the TxFIFO, then it is
 * currently believed that we do not need to wait for any space in
 * the TxFIFO.
 */
static void s3c_hsotg_send_zlp(struct s3c_hsotg *hsotg,
			       struct s3c_hsotg_req *req)
{
	u32 ctrl;

	if (!req) {
		dev_warn(hsotg->dev, "%s: no request?\n", __func__);
		return;
	}

	if (req->req.length == 0) {
		hsotg->eps[0].sent_zlp = 1;
		s3c_hsotg_enqueue_setup(hsotg);
		return;
	}

	hsotg->eps[0].dir_in = 1;
	hsotg->eps[0].sent_zlp = 1;

	dev_dbg(hsotg->dev, "sending zero-length packet\n");

	/* issue a zero-sized packet to terminate this */
	writel(S3C_DxEPTSIZ_MC(1) | S3C_DxEPTSIZ_PktCnt(1) |
	       S3C_DxEPTSIZ_XferSize(0), hsotg->regs + S3C_DIEPTSIZ(0));

	ctrl = readl(hsotg->regs + S3C_DIEPCTL0);
	ctrl |= S3C_DxEPCTL_CNAK;  /* clear NAK set by core */
	ctrl |= S3C_DxEPCTL_EPEna; /* ensure ep enabled */
	ctrl |= S3C_DxEPCTL_USBActEp;
	writel(ctrl, hsotg->regs + S3C_DIEPCTL0);
}

/**
 * s3c_hsotg_handle_outdone - handle receiving OutDone/SetupDone from RXFIFO
 * @hsotg: The device instance
 * @epnum: The endpoint received from
 * @was_setup: Set if processing a SetupDone event.
 *
 * The RXFIFO has delivered an OutDone event, which means that the data
 * transfer for an OUT endpoint has been completed, either by a short
 * packet or by the finish of a transfer.
*/
static void s3c_hsotg_handle_outdone(struct s3c_hsotg *hsotg,
				     int epnum, bool was_setup)
{
	u32 epsize = readl(hsotg->regs + S3C_DOEPTSIZ(epnum));
	struct s3c_hsotg_ep *hs_ep = &hsotg->eps[epnum];
	struct s3c_hsotg_req *hs_req = hs_ep->req;
	struct usb_request *req = &hs_req->req;
	unsigned size_left = S3C_DxEPTSIZ_XferSize_GET(epsize);
	int result = 0;

	if (!hs_req) {
		dev_dbg(hsotg->dev, "%s: no request active\n", __func__);
		return;
	}

	if (using_dma(hsotg)) {
		unsigned size_done;

		/* Calculate the size of the transfer by checking how much
		 * is left in the endpoint size register and then working it
		 * out from the amount we loaded for the transfer.
		 *
		 * We need to do this as DMA pointers are always 32bit aligned
		 * so may overshoot/undershoot the transfer.
		 */

		size_done = hs_ep->size_loaded - size_left;
		size_done += hs_ep->last_load;

		req->actual = size_done;
	}
	run_inactivity_timer(1);	/* re-initialize inactivity timer */

	/* if there is more request to do, schedule new transfer */
	if (req->actual < req->length && size_left == 0) {
		s3c_hsotg_start_req(hsotg, hs_ep, hs_req, true);
		return;
	}

	if (req->actual < req->length && req->short_not_ok) {
		dev_dbg(hsotg->dev, "%s: got %d/%d (short not ok) => error\n",
			__func__, req->actual, req->length);

		/* todo - what should we return here? there's no one else
		 * even bothering to check the status. */
	}

	if (epnum == 0) {
		/* Condition req->complete != s3c_hsotg_complete_setup says: */
		/* send ZLP when we have an asynchronous request from gadget */
		if (!was_setup && req->complete != s3c_hsotg_complete_setup)
			s3c_hsotg_send_zlp(hsotg, hs_req);
	}

	s3c_hsotg_complete_request_lock(hsotg, hs_ep, hs_req, result);
}

/**
 * s3c_hsotg_read_frameno - read current frame number
 * @hsotg: The device instance
 *
 * Return the current frame number
*/
static u32 s3c_hsotg_read_frameno(struct s3c_hsotg *hsotg)
{
	u32 dsts;

	dsts = readl(hsotg->regs + S3C_DSTS);
	dsts &= S3C_DSTS_SOFFN_MASK;
	dsts >>= S3C_DSTS_SOFFN_SHIFT;

	return dsts;
}

/**
 * s3c_hsotg_handle_rx - RX FIFO has data
 * @hsotg: The device instance
 *
 * The IRQ handler has detected that the RX FIFO has some data in it
 * that requires processing, so find out what is in there and do the
 * appropriate read.
 *
 * The RXFIFO is a true FIFO, the packets coming out are still in packet
 * chunks, so if you have x packets received on an endpoint you'll get x
 * FIFO events delivered, each with a packet's worth of data in it.
 *
 * When using DMA, we should not be processing events from the RXFIFO
 * as the actual data should be sent to the memory directly and we turn
 * on the completion interrupts to get notifications of transfer completion.
 */
static void s3c_hsotg_handle_rx(struct s3c_hsotg *hsotg)
{
	u32 grxstsr = readl(hsotg->regs + S3C_GRXSTSP);
	u32 epnum, status, size;

	WARN_ON(using_dma(hsotg));

	epnum = grxstsr & S3C_GRXSTS_EPNum_MASK;
	status = grxstsr & S3C_GRXSTS_PktSts_MASK;

	size = grxstsr & S3C_GRXSTS_ByteCnt_MASK;
	size >>= S3C_GRXSTS_ByteCnt_SHIFT;

	if (1)
		dev_dbg(hsotg->dev, "%s: GRXSTSP=0x%08x (%d@%d)\n",
			__func__, grxstsr, size, epnum);

#define __status(x) ((x) >> S3C_GRXSTS_PktSts_SHIFT)

	switch (status >> S3C_GRXSTS_PktSts_SHIFT) {
	case __status(S3C_GRXSTS_PktSts_GlobalOutNAK):
		dev_dbg(hsotg->dev, "GlobalOutNAK\n");
		break;

	case __status(S3C_GRXSTS_PktSts_OutDone):
		dev_dbg(hsotg->dev, "OutDone (Frame=0x%08x)\n",
			s3c_hsotg_read_frameno(hsotg));

		if (!using_dma(hsotg))
			s3c_hsotg_handle_outdone(hsotg, epnum, false);
		break;

	case __status(S3C_GRXSTS_PktSts_SetupDone):
		dev_dbg(hsotg->dev,
			"SetupDone (Frame=0x%08x, DOPEPCTL=0x%08x)\n",
			s3c_hsotg_read_frameno(hsotg),
			readl(hsotg->regs + S3C_DOEPCTL(0)));

		s3c_hsotg_handle_outdone(hsotg, epnum, true);
		break;

	case __status(S3C_GRXSTS_PktSts_OutRX):
		s3c_hsotg_rx_data(hsotg, epnum, size);
		break;

	case __status(S3C_GRXSTS_PktSts_SetupRX):
		dev_dbg(hsotg->dev,
			"SetupRX (Frame=0x%08x, DOPEPCTL=0x%08x)\n",
			s3c_hsotg_read_frameno(hsotg),
			readl(hsotg->regs + S3C_DOEPCTL(0)));

		s3c_hsotg_rx_data(hsotg, epnum, size);
		break;

	default:
		dev_warn(hsotg->dev, "%s: unknown status %08x\n",
			 __func__, grxstsr);

		s3c_hsotg_dump(hsotg);
		break;
	}
}

/**
 * s3c_hsotg_ep0_mps - turn max packet size into register setting
 * @mps: The maximum packet size in bytes.
*/
static u32 s3c_hsotg_ep0_mps(unsigned int mps)
{
	switch (mps) {
	case 64:
		return S3C_D0EPCTL_MPS_64;
	case 32:
		return S3C_D0EPCTL_MPS_32;
	case 16:
		return S3C_D0EPCTL_MPS_16;
	case 8:
		return S3C_D0EPCTL_MPS_8;
	}

	/* bad max packet size, warn and return invalid result */
	WARN_ON(1);
	return (u32)-1;
}

/**
 * s3c_hsotg_set_ep_maxpacket - set endpoint's max-packet field
 * @hsotg: The driver state.
 * @ep: The index number of the endpoint
 * @mps: The maximum packet size in bytes
 *
 * Configure the maximum packet size for the given endpoint, updating
 * the hardware control registers to reflect this.
 */
static void s3c_hsotg_set_ep_maxpacket(struct s3c_hsotg *hsotg,
				       unsigned int ep, unsigned int mps)
{
	struct s3c_hsotg_ep *hs_ep = &hsotg->eps[ep];
	void __iomem *regs = hsotg->regs;
	u32 mpsval;
	u32 reg;

	if (ep == 0) {
		/* EP0 is a special case */
		mpsval = s3c_hsotg_ep0_mps(mps);
		if (mpsval > 3)
			goto bad_mps;
	} else {
		if (mps >= S3C_DxEPCTL_MPS_LIMIT+1)
			goto bad_mps;

		mpsval = mps;
	}

	hs_ep->ep.maxpacket = mps;

	/* update both the in and out endpoint controldir_ registers, even
	 * if one of the directions may not be in use. */

	reg = readl(regs + S3C_DIEPCTL(ep));
	reg &= ~S3C_DxEPCTL_MPS_MASK;
	reg |= mpsval;
	writel(reg, regs + S3C_DIEPCTL(ep));

	reg = readl(regs + S3C_DOEPCTL(ep));
	reg &= ~S3C_DxEPCTL_MPS_MASK;
	reg |= mpsval;
	writel(reg, regs + S3C_DOEPCTL(ep));

	return;

bad_mps:
	dev_err(hsotg->dev, "ep%d: bad mps of %d\n", ep, mps);
}

/**
 * s3c_hsotg_txfifo_flush - flush Tx FIFO
 * @hsotg: The driver state
 * @idx: The index for the endpoint (0..(S3C_HSOTG_EPS-1))
 */
static void s3c_hsotg_txfifo_flush(struct s3c_hsotg *hsotg, unsigned int idx)
{
	int timeout;
	int val;

	writel(S3C_GRSTCTL_TxFNum(idx) | S3C_GRSTCTL_TxFFlsh,
		hsotg->regs + S3C_GRSTCTL);

	/* wait until the fifo is flushed */
	timeout = 100;

	while (1) {
		val = readl(hsotg->regs + S3C_GRSTCTL);

		if ((val & (S3C_GRSTCTL_TxFFlsh)) == 0)
			break;

		if (--timeout == 0) {
			dev_err(hsotg->dev,
				"%s: timeout flushing fifo (GRSTCTL=%08x)\n",
				__func__, val);
		}

		udelay(1);
	}
}

/**
 * s3c_hsotg_trytx - check to see if anything needs transmitting
 * @hsotg: The driver state
 * @hs_ep: The driver endpoint to check.
 *
 * Check to see if there is a request that has data to send, and if so
 * make an attempt to write data into the FIFO.
 */
static int s3c_hsotg_trytx(struct s3c_hsotg *hsotg,
			   struct s3c_hsotg_ep *hs_ep)
{
	struct s3c_hsotg_req *hs_req = hs_ep->req;

	if (!hs_ep->dir_in || !hs_req)
		return 0;

	if (hs_req->req.actual < hs_req->req.length) {
		dev_dbg(hsotg->dev, "trying to write more for ep%d\n",
			hs_ep->index);
		return s3c_hsotg_write_fifo(hsotg, hs_ep, hs_req);
	}

	return 0;
}

/**
 * s3c_hsotg_complete_in - complete IN transfer
 * @hsotg: The device state.
 * @hs_ep: The endpoint that has just completed.
 *
 * An IN transfer has been completed, update the transfer's state and then
 * call the relevant completion routines.
 */
static void s3c_hsotg_complete_in(struct s3c_hsotg *hsotg,
				  struct s3c_hsotg_ep *hs_ep)
{
	struct s3c_hsotg_req *hs_req = hs_ep->req;
	u32 epsize = readl(hsotg->regs + S3C_DIEPTSIZ(hs_ep->index));
	int size_left, size_done;

	if (!hs_req) {
		dev_dbg(hsotg->dev, "XferCompl but no req\n");
		return;
	}

	/* Calculate the size of the transfer by checking how much is left
	 * in the endpoint size register and then working it out from
	 * the amount we loaded for the transfer.
	 *
	 * We do this even for DMA, as the transfer may have incremented
	 * past the end of the buffer (DMA transfers are always 32bit
	 * aligned).
	 */

	size_left = S3C_DxEPTSIZ_XferSize_GET(epsize);

	size_done = hs_ep->size_loaded - size_left;
	size_done += hs_ep->last_load;

	if (hs_req->req.actual != size_done)
		dev_dbg(hsotg->dev, "%s: adjusting size done %d => %d\n",
			__func__, hs_req->req.actual, size_done);

	hs_req->req.actual = size_done;

	/* if we did all of the transfer, and there is more data left
	 * around, then try restarting the rest of the request */

	if (!size_left && hs_req->req.actual < hs_req->req.length) {
		dev_dbg(hsotg->dev, "%s trying more for req...\n", __func__);
		s3c_hsotg_start_req(hsotg, hs_ep, hs_req, true);
	} else
		s3c_hsotg_complete_request_lock(hsotg, hs_ep, hs_req, 0);
}

/**
 * s3c_hsotg_epint - handle an in/out endpoint interrupt
 * @hsotg: The driver state
 * @idx: The index for the endpoint (0..(S3C_HSOTG_EPS-1))
 * @dir_in: Set if this is an IN endpoint
 *
 * Process and clear any interrupt pending for an individual endpoint
*/
static void s3c_hsotg_epint(struct s3c_hsotg *hsotg, unsigned int idx,
			    int dir_in)
{
	struct s3c_hsotg_ep *hs_ep = &hsotg->eps[idx];
	u32 epint_reg = dir_in ? S3C_DIEPINT(idx) : S3C_DOEPINT(idx);
	u32 epctl_reg = dir_in ? S3C_DIEPCTL(idx) : S3C_DOEPCTL(idx);
	u32 epsiz_reg = dir_in ? S3C_DIEPTSIZ(idx) : S3C_DOEPTSIZ(idx);
	u32 ints;

	ints = readl(hsotg->regs + epint_reg);

	/* Clear endpoint interrupts */
	writel(ints, hsotg->regs + epint_reg);

	dev_dbg(hsotg->dev, "%s: ep%d(%s) DxEPINT=0x%08x\n",
		__func__, idx, dir_in ? "in" : "out", ints);

	if (ints & S3C_DxEPINT_XferCompl) {
		dev_dbg(hsotg->dev,
			"%s: XferCompl: DxEPCTL=0x%08x, DxEPTSIZ=%08x\n",
			__func__, readl(hsotg->regs + epctl_reg),
			readl(hsotg->regs + epsiz_reg));

		/* we get OutDone from the FIFO, so we only need to look
		 * at completing IN requests here */
		if (dir_in) {
			s3c_hsotg_complete_in(hsotg, hs_ep);

			if (idx == 0 && !hs_ep->req)
				s3c_hsotg_enqueue_setup(hsotg);
		} else if (using_dma(hsotg)) {
			/* We're using DMA, we need to fire an OutDone here
			 * as we ignore the RXFIFO. */

			s3c_hsotg_handle_outdone(hsotg, idx, false);
		}
	}

	if (ints & S3C_DxEPINT_EPDisbld) {
		dev_dbg(hsotg->dev, "%s: EPDisbld\n", __func__);

		if (dir_in) {
			int epctl = readl(hsotg->regs + epctl_reg);

			s3c_hsotg_txfifo_flush(hsotg, idx);

			if ((epctl & S3C_DxEPCTL_Stall) &&
				(epctl & S3C_DxEPCTL_EPType_Bulk)) {
				int dctl = readl(hsotg->regs + S3C_DCTL);

				dctl |= S3C_DCTL_CGNPInNAK;
				writel(dctl, hsotg->regs + S3C_DCTL);
			}
		}
	}

	if (ints & S3C_DxEPINT_AHBErr)
		dev_dbg(hsotg->dev, "%s: AHBErr\n", __func__);

	if (ints & S3C_DxEPINT_Setup) {  /* Setup or Timeout */
		dev_dbg(hsotg->dev, "%s: Setup/Timeout\n",  __func__);

		if (using_dma(hsotg) && idx == 0) {
			/* this is the notification we've received a
			 * setup packet. In non-DMA mode we'd get this
			 * from the RXFIFO, instead we need to process
			 * the setup here. */

			if (dir_in)
				WARN_ON_ONCE(1);
			else
				s3c_hsotg_handle_outdone(hsotg, 0, true);
		}
	}

	if (ints & S3C_DxEPINT_Back2BackSetup)
		dev_dbg(hsotg->dev, "%s: B2BSetup/INEPNakEff\n", __func__);

	if (dir_in) {
		/* not sure if this is important, but we'll clear it anyway
		 */
		if (ints & S3C_DIEPMSK_INTknTXFEmpMsk) {
			dev_dbg(hsotg->dev, "%s: ep%d: INTknTXFEmpMsk\n",
				__func__, idx);
		}

		/* this probably means something bad is happening */
		if (ints & S3C_DIEPMSK_INTknEPMisMsk) {
			dev_warn(hsotg->dev, "%s: ep%d: INTknEP\n",
				 __func__, idx);
		}

		/* FIFO has space or is empty (see GAHBCFG) */
		if (hsotg->dedicated_fifos &&
		    ints & S3C_DIEPMSK_TxFIFOEmpty) {
			dev_dbg(hsotg->dev, "%s: ep%d: TxFIFOEmpty\n",
				__func__, idx);
			s3c_hsotg_trytx(hsotg, hs_ep);
		}
	}
}

/**
 * s3c_hsotg_irq_enumdone - Handle EnumDone interrupt (enumeration done)
 * @hsotg: The device state.
 *
 * Handle updating the device settings after the enumeration phase has
 * been completed.
*/
static void s3c_hsotg_irq_enumdone(struct s3c_hsotg *hsotg)
{
	u32 dsts = readl(hsotg->regs + S3C_DSTS);
	int ep0_mps = 0, ep_mps;

	/* This should signal the finish of the enumeration phase
	 * of the USB handshaking, so we should now know what rate
	 * we connected at. */

	dev_dbg(hsotg->dev, "EnumDone (DSTS=0x%08x)\n", dsts);

	/* note, since we're limited by the size of transfer on EP0, and
	 * it seems IN transfers must be a even number of packets we do
	 * not advertise a 64byte MPS on EP0. */

	/* catch both EnumSpd_FS and EnumSpd_FS48 */
	switch (dsts & S3C_DSTS_EnumSpd_MASK) {
	case S3C_DSTS_EnumSpd_FS:
	case S3C_DSTS_EnumSpd_FS48:
		hsotg->gadget.speed = USB_SPEED_FULL;
		dev_info(hsotg->dev, "new device is full-speed\n");

		ep0_mps = EP0_MPS_LIMIT;
		ep_mps = 64;
		break;

	case S3C_DSTS_EnumSpd_HS:
		dev_info(hsotg->dev, "new device is high-speed\n");
		hsotg->gadget.speed = USB_SPEED_HIGH;

		ep0_mps = EP0_MPS_LIMIT;
		ep_mps = 512;
		break;

	case S3C_DSTS_EnumSpd_LS:
		hsotg->gadget.speed = USB_SPEED_LOW;
		dev_info(hsotg->dev, "new device is low-speed\n");

		/* note, we don't actually support LS in this driver at the
		 * moment, and the documentation seems to imply that it isn't
		 * supported by the PHYs on some of the devices.
		 */
		break;
	}

	/* we should now know the maximum packet size for an
	 * endpoint, so set the endpoints to a default value. */

	if (ep0_mps) {
		int i;
		s3c_hsotg_set_ep_maxpacket(hsotg, 0, ep0_mps);
		for (i = 1; i < S3C_HSOTG_EPS; i++)
			s3c_hsotg_set_ep_maxpacket(hsotg, i, ep_mps);
	}

	/* ensure after enumeration our EP0 is active */

	s3c_hsotg_enqueue_setup(hsotg);

	dev_dbg(hsotg->dev, "EP0: DIEPCTL0=0x%08x, DOEPCTL0=0x%08x\n",
		readl(hsotg->regs + S3C_DIEPCTL0),
		readl(hsotg->regs + S3C_DOEPCTL0));
}

/**
 * kill_all_requests - remove all requests from the endpoint's queue
 * @hsotg: The device state.
 * @ep: The endpoint the requests may be on.
 * @result: The result code to use.
 * @force: Force removal of any current requests
 *
 * Go through the requests on the given endpoint and mark them
 * completed with the given result code.
 */
static void kill_all_requests(struct s3c_hsotg *hsotg,
			      struct s3c_hsotg_ep *ep,
			      int result, bool force)
{
	struct s3c_hsotg_req *req, *treq;
	unsigned long flags;

	spin_lock_irqsave(&ep->lock, flags);

	list_for_each_entry_safe(req, treq, &ep->queue, queue) {
		/* currently, we can't do much about an already
		 * running request on an in endpoint */

		if (ep->req == req && ep->dir_in && !force)
			continue;

		s3c_hsotg_complete_request(hsotg, ep, req,
					   result);
	}

	spin_unlock_irqrestore(&ep->lock, flags);
}

#define call_gadget(_hs, _entry) \
	if ((_hs)->gadget.speed != USB_SPEED_UNKNOWN &&	\
	    (_hs)->driver && (_hs)->driver->_entry)	\
		(_hs)->driver->_entry(&(_hs)->gadget);

/**
 * s3c_hsotg_disconnect_irq - disconnect irq service
 * @hsotg: The device state.
 *
 * A disconnect IRQ has been received, meaning that the host has
 * lost contact with the bus. Remove all current transactions
 * and signal the gadget driver that this has happened.
*/
static void s3c_hsotg_disconnect_irq(struct s3c_hsotg *hsotg)
{
	unsigned ep;

	for (ep = 0; ep < S3C_HSOTG_EPS; ep++)
		kill_all_requests(hsotg, &hsotg->eps[ep], -ESHUTDOWN, true);

	call_gadget(hsotg, disconnect);
}

/**
 * s3c_hsotg_irq_fifoempty - TX FIFO empty interrupt handler
 * @hsotg: The device state:
 * @periodic: True if this is a periodic FIFO interrupt
 */
static void s3c_hsotg_irq_fifoempty(struct s3c_hsotg *hsotg, bool periodic)
{
	struct s3c_hsotg_ep *ep;
	int epno, ret;

	/* look through for any more data to transmit */

	for (epno = 0; epno < S3C_HSOTG_EPS; epno++) {
		ep = &hsotg->eps[epno];

		if (!ep->dir_in)
			continue;

		if ((periodic && !ep->periodic) ||
		    (!periodic && ep->periodic))
			continue;

		ret = s3c_hsotg_trytx(hsotg, ep);
		if (ret < 0)
			break;
	}
}

static struct s3c_hsotg *our_hsotg;

/* Sends the cable connected state to an input event */
static void s3c_hsotg_update_connected_state(unsigned long data)
{
	struct s3c_hsotg* hsotg;
	hsotg = (struct s3c_hsotg*)data;
	
	if(!hsotg->conn_state)
		s3c_hsotg_disconnect_irq(hsotg);
#ifndef U_BOOT
	input_report_switch(hsotg->input, SW_LID, hsotg->conn_state);
#endif
}

/* IRQ flags which will trigger a retry around the IRQ loop */
#define IRQ_RETRY_MASK (S3C_GINTSTS_NPTxFEmp | \
			S3C_GINTSTS_PTxFEmp |  \
			S3C_GINTSTS_RxFLvl)

/**
 * s3c_hsotg_irq - handle device interrupt
 * @irq: The IRQ number triggered
 * @pw: The pw value when registered the handler.
 */
static irqreturn_t s3c_hsotg_irq(int irq, void *pw)
{
	struct s3c_hsotg *hsotg = pw;
	int retry_count = 8;
	u32 gintsts;
	u32 gintmsk;
	u32 current_timer = 0;
	int detect = detect_usb_charger();
	static int counter = 0;

irq_retry:
	
	counter = counter + 1;
	
	// Deal with the wall charger first
	if(detect == 1)
	{
		run_screen_state(SCREEN_TUNEUP_VBUS_WALL);
		hsotg->was_wall_connected = true;
		hsotg->enum_done = false;
		hsotg->conn_event_timer = 0;
		s3c_hsotg_disconnect_irq(hsotg);
	}
	else 
	{
		if(hsotg->was_wall_connected && !hsotg->enum_done)
		{
			run_screen_state(SCREEN_TUNEUP_NO_VBUS);
			hsotg->was_wall_connected = false;
		}
	}

	//Debounce timer handling
	if( hsotg->conn_event_timer )
	{
		//Only fire 200ms after the timer is created
		current_timer = get_timer(0);
		if( current_timer - hsotg->conn_event_timer >= 200 )
		{
			//We've been disconnected! Show screen and run disconnect handling
			dev_dbg(hsotg->dev, "current_timer: %u, hsotg->conn_event_timer: %u, dcon\n",
				current_timer,
				hsotg->conn_event_timer);
				
#if defined(CONFIG_LCD_SCREENS)
		if(!hsotg->enum_done || !hsotg->conn_state)
		{
			run_screen_state(SCREEN_TUNEUP_NO_VBUS);
		}
#endif
			hsotg->conn_event_timer = 0;
			s3c_hsotg_disconnect_irq(hsotg);
		}
	}
		

	gintsts = readl(hsotg->regs + S3C_GINTSTS);
	gintmsk = readl(hsotg->regs + S3C_GINTMSK);

	dev_dbg(hsotg->dev, "%s: %08x %08x (%08x) retry %d\n",
		__func__, gintsts, gintsts & gintmsk, gintmsk, retry_count);

	gintsts &= gintmsk;

	if (gintsts & S3C_GINTSTS_OTGInt) {
		u32 otgint = readl(hsotg->regs + S3C_GOTGINT);

		dev_dbg(hsotg->dev, "OTGInt: %08x\n", otgint);

		writel(otgint, hsotg->regs + S3C_GOTGINT);
	}

	if (gintsts & S3C_GINTSTS_DisconnInt) {
		dev_dbg(hsotg->dev, "%s: DisconnInt\n", __func__);
		writel(S3C_GINTSTS_DisconnInt, hsotg->regs + S3C_GINTSTS);
	
		s3c_hsotg_disconnect_irq(hsotg);
	}

	if (gintsts & S3C_GINTSTS_SessReqInt) {
		dev_dbg(hsotg->dev, "%s: SessReqInt\n", __func__);
		writel(S3C_GINTSTS_SessReqInt, hsotg->regs + S3C_GINTSTS);
	
	}

	if (gintsts & S3C_GINTSTS_EnumDone) {
		writel(S3C_GINTSTS_EnumDone, hsotg->regs + S3C_GINTSTS);
	
		s3c_hsotg_irq_enumdone(hsotg);
		
		hsotg->conn_state = true;
		if(!hsotg->enum_done)
			hsotg->enum_done = true;
		
		run_screen_state(SCREEN_TUNEUP_VBUS);	/* update screen */

#ifndef U_BOOT
		/* Set a debounce timer for connection state */
		mod_timer( &(hsotg->conn_state_timer), jiffies + msecs_to_jiffies(100) );
#else
		//Disable disconnect timer
		hsotg->conn_event_timer = 0;
#endif
	}

	if (gintsts & S3C_GINTSTS_ConIDStsChng) {
		dev_dbg(hsotg->dev, "ConIDStsChg (DSTS=0x%08x, GOTCTL=%08x)\n",
			readl(hsotg->regs + S3C_DSTS),
			readl(hsotg->regs + S3C_GOTGCTL));

		writel(S3C_GINTSTS_ConIDStsChng, hsotg->regs + S3C_GINTSTS);
		
	}

	if (gintsts & (S3C_GINTSTS_OEPInt | S3C_GINTSTS_IEPInt)) {
		u32 daint = readl(hsotg->regs + S3C_DAINT);
		u32 daint_out = daint >> S3C_DAINT_OutEP_SHIFT;
		u32 daint_in = daint & ~(daint_out << S3C_DAINT_OutEP_SHIFT);
		int ep;

		dev_dbg(hsotg->dev, "%s: daint=%08x\n", __func__, daint);
	

		for (ep = 0; ep < S3C_HSOTG_EPS && daint_out; ep++, daint_out >>= 1) {
			if (daint_out & 1)
				s3c_hsotg_epint(hsotg, ep, 0);
		}

		for (ep = 0; ep < S3C_HSOTG_EPS && daint_in; ep++, daint_in >>= 1) {
			if (daint_in & 1)
				s3c_hsotg_epint(hsotg, ep, 1);
		}
	}

	if (gintsts & S3C_GINTSTS_USBRst) {
		u32 dcfg;
		dev_dbg(hsotg->dev, "%s: USBRst\n", __func__);
		dev_dbg(hsotg->dev, "GNPTXSTS=%08x\n",
			readl(hsotg->regs + S3C_GNPTXSTS));

		writel(S3C_GINTSTS_USBRst, hsotg->regs + S3C_GINTSTS);
		
		kill_all_requests(hsotg, &hsotg->eps[0], -ECONNRESET, true);

		/* Reset device address */
		dcfg = readl(hsotg->regs + S3C_DCFG);
		dcfg &= ~S3C_DCFG_DevAddr_MASK;
		writel(dcfg, hsotg->regs + S3C_DCFG);

		/* it seems after a reset we can end up with a situation
		 * where the TXFIFO still has data in it... the docs
		 * suggest resetting all the fifos, so use the init_fifo
		 * code to relayout and flush the fifos.
		 */

		s3c_hsotg_init_fifo(hsotg);

		s3c_hsotg_enqueue_setup(hsotg);
		
		/* Set a debounce timer for connection state */
		hsotg->conn_state = false;
#ifdef U_BOOT
		/* Set disconnect debounce timer, change screen when timer expires */
		hsotg->conn_event_timer = get_timer(0);
		dev_dbg( hsotg->dev, "set conn_event_timer to %u\n", hsotg->conn_event_timer);
#else
		mod_timer( &(hsotg->conn_state_timer), jiffies + msecs_to_jiffies(100) );
#endif
	}

	/* check both FIFOs */

	if (gintsts & S3C_GINTSTS_NPTxFEmp) {
		dev_dbg(hsotg->dev, "NPTxFEmp\n");

		/* Disable the interrupt to stop it happening again
		 * unless one of these endpoint routines decides that
		 * it needs re-enabling */

		s3c_hsotg_disable_gsint(hsotg, S3C_GINTSTS_NPTxFEmp);
		s3c_hsotg_irq_fifoempty(hsotg, false);
	}

	if (gintsts & S3C_GINTSTS_PTxFEmp) {
		dev_dbg(hsotg->dev, "PTxFEmp\n");

		/* See note in S3C_GINTSTS_NPTxFEmp */

		s3c_hsotg_disable_gsint(hsotg, S3C_GINTSTS_PTxFEmp);
		s3c_hsotg_irq_fifoempty(hsotg, true);
	}

	if (gintsts & S3C_GINTSTS_RxFLvl) {
		/* note, since GINTSTS_RxFLvl doubles as FIFO-not-empty,
		 * we need to retry s3c_hsotg_handle_rx if this is still
		 * set. */

		s3c_hsotg_handle_rx(hsotg);
	}

	if (gintsts & S3C_GINTSTS_ModeMis) {
		dev_warn(hsotg->dev, "warning, mode mismatch triggered\n");
		writel(S3C_GINTSTS_ModeMis, hsotg->regs + S3C_GINTSTS);
	}

	if (gintsts & S3C_GINTSTS_USBSusp) {
		dev_dbg(hsotg->dev, "S3C_GINTSTS_USBSusp\n");
	
		writel(S3C_GINTSTS_USBSusp, hsotg->regs + S3C_GINTSTS);

		call_gadget(hsotg, suspend);
	}

	if (gintsts & S3C_GINTSTS_WkUpInt) {
		dev_dbg(hsotg->dev, "S3C_GINTSTS_WkUpIn\n");
	
		writel(S3C_GINTSTS_WkUpInt, hsotg->regs + S3C_GINTSTS);

		call_gadget(hsotg, resume);
	}

	if (gintsts & S3C_GINTSTS_ErlySusp) {
		dev_dbg(hsotg->dev, "S3C_GINTSTS_ErlySusp\n");
	
		writel(S3C_GINTSTS_ErlySusp, hsotg->regs + S3C_GINTSTS);
	}

	/* these next two seem to crop-up occasionally causing the core
	 * to shutdown the USB transfer, so try clearing them and logging
	 * the occurrence. */

	if (gintsts & S3C_GINTSTS_GOUTNakEff) {
		dev_dbg(hsotg->dev, "GOUTNakEff triggered\n");
	
		writel(S3C_DCTL_CGOUTNak, hsotg->regs + S3C_DCTL);

		s3c_hsotg_dump(hsotg);
	}

	if (gintsts & S3C_GINTSTS_GINNakEff) {
		dev_dbg(hsotg->dev, "GINNakEff triggered\n");

		writel(S3C_DCTL_CGNPInNAK, hsotg->regs + S3C_DCTL);

		s3c_hsotg_dump(hsotg);
	}

	/* if we've had fifo events, we should try and go around the
	 * loop again to see if there's any point in returning yet. */

	if (gintsts & IRQ_RETRY_MASK && --retry_count > 0)
			goto irq_retry;

//irq_end:
	return IRQ_HANDLED;
}

/**
 * s3c_hsotg_ep_enable - enable the given endpoint
 * @ep: The USB endpint to configure
 * @desc: The USB endpoint descriptor to configure with.
 *
 * This is called from the USB gadget code's usb_ep_enable().
*/
static int s3c_hsotg_ep_enable(struct usb_ep *ep,
			       const struct usb_endpoint_descriptor *desc)
{
	struct s3c_hsotg_ep *hs_ep = our_ep(ep);
	struct s3c_hsotg *hsotg = hs_ep->parent;
#ifndef U_BOOT
	unsigned long flags;
#endif
	int index = hs_ep->index;
	u32 epctrl_reg;
	u32 epctrl;
	u32 mps;
	int dir_in;
	int ret = 0;

	dev_dbg(hsotg->dev,
		"%s: ep %s: a 0x%02x, attr 0x%02x, mps 0x%04x, intr %d\n",
		__func__, ep->name, desc->bEndpointAddress, desc->bmAttributes,
		desc->wMaxPacketSize, desc->bInterval);

	/* not to be called for EP0 */
	WARN_ON(index == 0);

	dir_in = (desc->bEndpointAddress & USB_ENDPOINT_DIR_MASK) ? 1 : 0;
	if (dir_in != hs_ep->dir_in) {
		dev_err(hsotg->dev, "%s: direction mismatch!\n", __func__);
		return -EINVAL;
	}

	mps = le16_to_cpu(desc->wMaxPacketSize);

	/* note, we handle this here instead of s3c_hsotg_set_ep_maxpacket */

	epctrl_reg = dir_in ? S3C_DIEPCTL(index) : S3C_DOEPCTL(index);
	epctrl = readl(hsotg->regs + epctrl_reg);

	dev_dbg(hsotg->dev, "%s: read DxEPCTL=0x%08x from 0x%08x\n",
		__func__, epctrl, epctrl_reg);

	spin_lock_irqsave(&hs_ep->lock, flags);

	epctrl &= ~(S3C_DxEPCTL_EPType_MASK | S3C_DxEPCTL_MPS_MASK);
	epctrl |= S3C_DxEPCTL_MPS(mps);

	/* mark the endpoint as active, otherwise the core may ignore
	 * transactions entirely for this endpoint */
	epctrl |= S3C_DxEPCTL_USBActEp;

	/* set the NAK status on the endpoint, otherwise we might try and
	 * do something with data that we've yet got a request to process
	 * since the RXFIFO will take data for an endpoint even if the
	 * size register hasn't been set.
	 */

	epctrl |= S3C_DxEPCTL_SNAK;

	/* update the endpoint state */
	hs_ep->ep.maxpacket = mps;

	/* default, set to non-periodic */
	hs_ep->periodic = 0;

	switch (desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
	case USB_ENDPOINT_XFER_ISOC:
		dev_err(hsotg->dev, "no current ISOC support\n");
		ret = -EINVAL;
		goto out;

	case USB_ENDPOINT_XFER_BULK:
		epctrl |= S3C_DxEPCTL_EPType_Bulk;
		break;

	case USB_ENDPOINT_XFER_INT:
		if (dir_in) {
			/* Allocate our TxFNum by simply using the index
			 * of the endpoint for the moment. We could do
			 * something better if the host indicates how
			 * many FIFOs we are expecting to use. */

			hs_ep->periodic = 0;	/* LF2000 has no periodic IN Endpoints */
			epctrl |= S3C_DxEPCTL_TxFNum(index);
		}

		epctrl |= S3C_DxEPCTL_EPType_Intterupt;
		break;

	case USB_ENDPOINT_XFER_CONTROL:
		epctrl |= S3C_DxEPCTL_EPType_Control;
		break;
	}

	/* if the hardware has dedicated fifos, we must give each IN EP
	 * a unique tx-fifo even if it is non-periodic.
	 */
	if (dir_in && hsotg->dedicated_fifos)
		epctrl |= S3C_DxEPCTL_TxFNum(index);

	/* for non control endpoints, set PID to D0 */
	if (index)
		epctrl |= S3C_DxEPCTL_SetD0PID;

	dev_dbg(hsotg->dev, "%s: write DxEPCTL=0x%08x\n",
		__func__, epctrl);

	writel(epctrl, hsotg->regs + epctrl_reg);
	dev_dbg(hsotg->dev, "%s: read DxEPCTL=0x%08x\n",
		__func__, readl(hsotg->regs + epctrl_reg));

	/* enable the endpoint interrupt */
	s3c_hsotg_ctrl_epint(hsotg, index, dir_in, 1);

out:
	spin_unlock_irqrestore(&hs_ep->lock, flags);
	return ret;
}

static int s3c_hsotg_ep_disable(struct usb_ep *ep)
{
	struct s3c_hsotg_ep *hs_ep = our_ep(ep);
	struct s3c_hsotg *hsotg = hs_ep->parent;
	int dir_in = hs_ep->dir_in;
	int index = hs_ep->index;
#ifndef U_BOOT
	unsigned long flags;
#endif
	u32 epctrl_reg;
	u32 ctrl;

	dev_dbg(hsotg->dev, "%s (ep%d-%s %p)\n", __func__, index, hs_ep->dir_in ? "in" : "out", ep);

	if (ep == &hsotg->eps[0].ep) {
		dev_err(hsotg->dev, "%s: called for ep0\n", __func__);
		return -EINVAL;
	}

	epctrl_reg = dir_in ? S3C_DIEPCTL(index) : S3C_DOEPCTL(index);

	/* terminate all requests with shutdown */
	kill_all_requests(hsotg, hs_ep, -ESHUTDOWN, false);

	spin_lock_irqsave(&hs_ep->lock, flags);

	ctrl = readl(hsotg->regs + epctrl_reg);
	ctrl &= ~S3C_DxEPCTL_EPEna;
	ctrl &= ~S3C_DxEPCTL_USBActEp;
	ctrl |= S3C_DxEPCTL_SNAK;

	dev_dbg(hsotg->dev, "%s: DxEPCTL=0x%08x\n", __func__, ctrl);
	writel(ctrl, hsotg->regs + epctrl_reg);

	/* disable endpoint interrupts */
	s3c_hsotg_ctrl_epint(hsotg, hs_ep->index, hs_ep->dir_in, 0);

	spin_unlock_irqrestore(&hs_ep->lock, flags);
	return 0;
}

/**
 * on_list - check request is on the given endpoint
 * @ep: The endpoint to check.
 * @test: The request to test if it is on the endpoint.
*/
static bool on_list(struct s3c_hsotg_ep *ep, struct s3c_hsotg_req *test)
{
	struct s3c_hsotg_req *req, *treq;

	list_for_each_entry_safe(req, treq, &ep->queue, queue) {
		if (req == test)
			return true;
	}

	return false;
}

static int s3c_hsotg_ep_dequeue(struct usb_ep *ep, struct usb_request *req)
{
	struct s3c_hsotg_req *hs_req = our_req(req);
	struct s3c_hsotg_ep *hs_ep = our_ep(ep);
	struct s3c_hsotg *hs = hs_ep->parent;
#ifndef U_BOOT
	unsigned long flags;
#endif

	dev_dbg(hs->dev, "ep_dequeue(%p,%p)\n", ep, req);

	spin_lock_irqsave(&hs_ep->lock, flags);

	if (!on_list(hs_ep, hs_req)) {
		spin_unlock_irqrestore(&hs_ep->lock, flags);
		return -EINVAL;
	}

	s3c_hsotg_complete_request(hs, hs_ep, hs_req, -ECONNRESET);
	spin_unlock_irqrestore(&hs_ep->lock, flags);

	return 0;
}

#if 1	/* 29nov11 Try calling this in response to a Bulk-only Reset */
void s3c_hsotg_clear_bulk_stall(struct usb_ep *ep) 
{
	struct s3c_hsotg_ep *hs_ep = our_ep(ep);
	struct s3c_hsotg    *hs    = hs_ep->parent;
	int index = hs_ep->index;
	u32 epreg;
	u32 epctl;

	epreg  = S3C_DIEPCTL(index);
	epctl  = readl(hs->regs + epreg);
	epctl &= ~S3C_DxEPCTL_Stall;
	writel(epctl, hs->regs + epreg);

	epreg  = S3C_DOEPCTL(index);
	epctl  = readl(hs->regs + epreg);
	epctl &= ~S3C_DxEPCTL_Stall;
	writel(epctl, hs->regs + epreg);
}
#endif	/* 29nov11 */

static int s3c_hsotg_ep_sethalt(struct usb_ep *ep, int value)
{
	struct s3c_hsotg_ep *hs_ep = our_ep(ep);
	struct s3c_hsotg *hs = hs_ep->parent;
	int index = hs_ep->index;
#ifndef U_BOOT
	unsigned long irqflags;
#endif
	u32 epreg;
	u32 epctl;
	u32 xfertype;

	dev_dbg(hs->dev, "%s(ep %p %s, %d)\n", __func__, ep, ep->name, value);

	spin_lock_irqsave(&hs_ep->lock, irqflags);

	/* write both IN and OUT control registers */

	epreg = S3C_DIEPCTL(index);
	epctl = readl(hs->regs + epreg);

	if (value) {
		epctl |= S3C_DxEPCTL_Stall + S3C_DxEPCTL_SNAK;
		if (epctl & S3C_DxEPCTL_EPEna)
			epctl |= S3C_DxEPCTL_EPDis;
	} else {
		epctl &= ~S3C_DxEPCTL_Stall;
		xfertype = epctl & S3C_DxEPCTL_EPType_MASK;
		if (xfertype == S3C_DxEPCTL_EPType_Bulk ||
			xfertype == S3C_DxEPCTL_EPType_Intterupt)
				epctl |= S3C_DxEPCTL_SetD0PID;
	}

	writel(epctl, hs->regs + epreg);

	epreg = S3C_DOEPCTL(index);
	epctl = readl(hs->regs + epreg);

	if (value)
		epctl |= S3C_DxEPCTL_Stall;
	else {
		epctl &= ~S3C_DxEPCTL_Stall;
		xfertype = epctl & S3C_DxEPCTL_EPType_MASK;
		if (xfertype == S3C_DxEPCTL_EPType_Bulk ||
			xfertype == S3C_DxEPCTL_EPType_Intterupt)
				epctl |= S3C_DxEPCTL_SetD0PID;
	}

	writel(epctl, hs->regs + epreg);

	spin_unlock_irqrestore(&hs_ep->lock, irqflags);

	return 0;
}

static struct usb_ep_ops s3c_hsotg_ep_ops = {
	.enable		= s3c_hsotg_ep_enable,
	.disable	= s3c_hsotg_ep_disable,
	.alloc_request	= s3c_hsotg_ep_alloc_request,
	.free_request	= s3c_hsotg_ep_free_request,
	.queue		= s3c_hsotg_ep_queue,
	.dequeue	= s3c_hsotg_ep_dequeue,
	.set_halt	= s3c_hsotg_ep_sethalt,
	/* note, don't believe we have any call for the fifo routines */
};

/**
 * s3c_hsotg_corereset - issue softreset to the core
 * @hsotg: The device state
 *
 * Issue a soft reset to the core, and await the core finishing it.
*/
static int s3c_hsotg_corereset(struct s3c_hsotg *hsotg)
{
	int timeout;
	u32 grstctl;

	dev_dbg(hsotg->dev, "resetting core\n");

	/* issue soft reset */
	writel(S3C_GRSTCTL_CSftRst, hsotg->regs + S3C_GRSTCTL);

	timeout = 1000;
	do {
		grstctl = readl(hsotg->regs + S3C_GRSTCTL);
	} while ((grstctl & S3C_GRSTCTL_CSftRst) && timeout-- > 0);

	if (grstctl & S3C_GRSTCTL_CSftRst) {
		dev_err(hsotg->dev, "Failed to get CSftRst asserted\n");
		return -EINVAL;
	}

	timeout = 1000;

	while (1) {
		u32 grstctl = readl(hsotg->regs + S3C_GRSTCTL);

		if (timeout-- < 0) {
			dev_info(hsotg->dev,
				 "%s: reset failed, GRSTCTL=%08x\n",
				 __func__, grstctl);
			return -ETIMEDOUT;
		}

		if (!(grstctl & S3C_GRSTCTL_AHBIdle))
			continue;

		break;		/* reset done */
	}

	dev_dbg(hsotg->dev, "reset successful\n");
	return 0;
}

int usb_gadget_probe_driver(struct usb_gadget_driver *driver,
		int (*bind)(struct usb_gadget *))
{
	struct s3c_hsotg *hsotg = our_hsotg;
	int ret;

	printk(KERN_ERR "%s.%d  %s() called\n", __FILE__, __LINE__, __func__);

	if (!hsotg) {
		printk(KERN_ERR "%s: called with no device\n", __func__);
		return -ENODEV;
	}

	if (!driver) {
		dev_err(hsotg->dev, "%s: no driver\n", __func__);
		return -EINVAL;
	}

	if (driver->speed != USB_SPEED_HIGH &&
	    driver->speed != USB_SPEED_FULL) {
		dev_err(hsotg->dev, "%s: bad speed\n", __func__);
	}

	if (!bind || !driver->setup) {
		dev_err(hsotg->dev, "%s: missing entry points\n", __func__);
		return -EINVAL;
	}

	WARN_ON(hsotg->driver);

	driver->driver.bus = NULL;
	hsotg->driver = driver;
	hsotg->gadget.dev.driver = &driver->driver;
	hsotg->gadget.dev.dma_mask = hsotg->dev->dma_mask;
	hsotg->gadget.speed = USB_SPEED_UNKNOWN;

#ifndef U_BOOT
	ret = device_add(&hsotg->gadget.dev);
	if (ret) {
		dev_err(hsotg->dev, "failed to register gadget device\n");
		goto err;
	}
#endif

	ret = bind(&hsotg->gadget);
	if (ret) {
		dev_err(hsotg->dev, "failed bind %s\n", driver->driver.name);

#ifndef U_BOOT
		hsotg->gadget.dev.driver = NULL;
#endif
		hsotg->driver = NULL;
		goto err;
	}

	/* we must now enable ep0 ready for host detection and then
	 * set configuration. */

	s3c_hsotg_corereset(hsotg);

	/* set the PLL on, remove the HNP/SRP and set the PHY */
	writel(S3C_GUSBCFG_PHYIf16 | S3C_GUSBCFG_TOutCal(7) |
	       (0x5 << 10), hsotg->regs + S3C_GUSBCFG);

	/* looks like soft-reset changes state of FIFOs */
	s3c_hsotg_init_fifo(hsotg);

	__orr32(hsotg->regs + S3C_DCTL, S3C_DCTL_SftDiscon);

	writel(1 << 18 | S3C_DCFG_DevSpd_HS,  hsotg->regs + S3C_DCFG);

	/* Clear any pending OTG interrupts */
	writel(0xffffffff, hsotg->regs + S3C_GOTGINT);

	/* Clear any pending interrupts */
	writel(0xffffffff, hsotg->regs + S3C_GINTSTS);

	writel(S3C_GINTSTS_DisconnInt | S3C_GINTSTS_SessReqInt |
	       S3C_GINTSTS_ConIDStsChng | S3C_GINTSTS_USBRst |
	       S3C_GINTSTS_EnumDone | S3C_GINTSTS_OTGInt |
	       S3C_GINTSTS_USBSusp | S3C_GINTSTS_WkUpInt |
	       S3C_GINTSTS_GOUTNakEff | S3C_GINTSTS_GINNakEff |
	       S3C_GINTSTS_ErlySusp,
	       hsotg->regs + S3C_GINTMSK);

	if (using_dma(hsotg))
		writel(S3C_GAHBCFG_GlblIntrEn | S3C_GAHBCFG_DMAEn |
		       S3C_GAHBCFG_HBstLen_Incr4,
		       hsotg->regs + S3C_GAHBCFG);
	else
		writel(S3C_GAHBCFG_GlblIntrEn, hsotg->regs + S3C_GAHBCFG);

	/* Enabling INTknTXFEmpMsk here seems to be a big mistake, we end
	 * up being flooded with interrupts if the host is polling the
	 * endpoint to try and read data. */

	writel(S3C_DIEPMSK_TimeOUTMsk | S3C_DIEPMSK_AHBErrMsk |
	       S3C_DIEPMSK_INTknEPMisMsk |
	       S3C_DIEPMSK_EPDisbldMsk | S3C_DIEPMSK_XferComplMsk |
	       ((hsotg->dedicated_fifos) ? S3C_DIEPMSK_TxFIFOEmpty : 0),
	       hsotg->regs + S3C_DIEPMSK);

	/* don't need XferCompl, we get that from RXFIFO in slave mode. In
	 * DMA mode we may need this. */
	writel(S3C_DOEPMSK_SetupMsk | S3C_DOEPMSK_AHBErrMsk |
	       S3C_DOEPMSK_EPDisbldMsk |
	       (using_dma(hsotg) ? S3C_DOEPMSK_XferComplMsk  : 0),
	       hsotg->regs + S3C_DOEPMSK);

	writel(0, hsotg->regs + S3C_DAINTMSK);

	dev_dbg(hsotg->dev, "EP0: DIEPCTL0=0x%08x, DOEPCTL0=0x%08x\n",
		readl(hsotg->regs + S3C_DIEPCTL0),
		readl(hsotg->regs + S3C_DOEPCTL0));

	/* enable in and out endpoint interrupts */
	s3c_hsotg_en_gsint(hsotg, S3C_GINTSTS_OEPInt | S3C_GINTSTS_IEPInt);

	/* Enable the RXFIFO when in slave mode, as this is how we collect
	 * the data. In DMA mode, we get events from the FIFO but also
	 * things we cannot process, so do not use it. */
	if (!using_dma(hsotg))
		s3c_hsotg_en_gsint(hsotg, S3C_GINTSTS_RxFLvl);

	/* Enable interrupts for EP0 in and out */
	s3c_hsotg_ctrl_epint(hsotg, 0, 0, 1);
	s3c_hsotg_ctrl_epint(hsotg, 0, 1, 1);

	__orr32(hsotg->regs + S3C_DCTL, S3C_DCTL_PWROnPrgDone);
	udelay(10);  /* see openiboot */
	__bic32(hsotg->regs + S3C_DCTL, S3C_DCTL_PWROnPrgDone);

	dev_dbg(hsotg->dev, "DCTL=0x%08x\n", readl(hsotg->regs + S3C_DCTL));

	/* S3C_DxEPCTL_USBActEp says RO in manual, but seems to be set by
	   writing to the EPCTL register.. */

	/* set to read 1 8byte packet */
	writel(S3C_DxEPTSIZ_MC(1) | S3C_DxEPTSIZ_PktCnt(1) |
	       S3C_DxEPTSIZ_XferSize(8), hsotg->regs + DOEPTSIZ0);

	writel(s3c_hsotg_ep0_mps(hsotg->eps[0].ep.maxpacket) |
	       S3C_DxEPCTL_CNAK | S3C_DxEPCTL_EPEna |
	       S3C_DxEPCTL_USBActEp,
	       hsotg->regs + S3C_DOEPCTL0);

	/* enable, but don't activate EP0in */
	writel(s3c_hsotg_ep0_mps(hsotg->eps[0].ep.maxpacket) |
	       S3C_DxEPCTL_USBActEp, hsotg->regs + S3C_DIEPCTL0);

	s3c_hsotg_enqueue_setup(hsotg);

	dev_dbg(hsotg->dev, "EP0: DIEPCTL0=0x%08x, DOEPCTL0=0x%08x\n",
		readl(hsotg->regs + S3C_DIEPCTL0),
		readl(hsotg->regs + S3C_DOEPCTL0));

	/* clear global NAKs */
	writel(S3C_DCTL_CGOUTNak | S3C_DCTL_CGNPInNAK,
	       hsotg->regs + S3C_DCTL);

	/* must be at-least 3ms to allow bus to see disconnect */
	msleep(3);

	/* remove the soft-disconnect and let's go */
	__bic32(hsotg->regs + S3C_DCTL, S3C_DCTL_SftDiscon);

	/* report to the user, and return */

	dev_info(hsotg->dev, "bound driver %s\n", driver->driver.name);
	return 0;

err:
	hsotg->driver = NULL;
#ifndef U_BOOT
	hsotg->gadget.dev.driver = NULL;
#endif
	return ret;
}
EXPORT_SYMBOL(usb_gadget_probe_driver);

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct s3c_hsotg *hsotg = our_hsotg;
	int ep;

	if (!hsotg)
		return -ENODEV;

	if (!driver || driver != hsotg->driver || !driver->unbind)
		return -EINVAL;

	/* all endpoints should be shutdown */
	for (ep = 0; ep < S3C_HSOTG_EPS; ep++)
		s3c_hsotg_ep_disable(&hsotg->eps[ep].ep);

	call_gadget(hsotg, disconnect);

	driver->unbind(&hsotg->gadget);
	hsotg->driver = NULL;
	hsotg->gadget.speed = USB_SPEED_UNKNOWN;

#ifndef U_BOOT
	device_del(&hsotg->gadget.dev);
#endif

	dev_info(hsotg->dev, "unregistered gadget driver '%s'\n",
		 driver->driver.name);

	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

static int s3c_hsotg_gadget_getframe(struct usb_gadget *gadget)
{
	return s3c_hsotg_read_frameno(to_hsotg(gadget));
}

static int s3c_hsotg_vbus_draw(struct usb_gadget *gadget, unsigned mA)
{
	if (lfp100_have_lfp100())
		return lfp100_set_vbus_draw(mA);
	else
		return -ENOTSUPP;
}

static struct usb_gadget_ops s3c_hsotg_gadget_ops = {
	.get_frame	= s3c_hsotg_gadget_getframe,
	.vbus_draw	= s3c_hsotg_vbus_draw,
};

/**
 * s3c_hsotg_initep - initialise a single endpoint
 * @hsotg: The device state.
 * @hs_ep: The endpoint to be initialised.
 * @epnum: The endpoint number
 *
 * Initialise the given endpoint (as part of the probe and device state
 * creation) to give to the gadget driver. Setup the endpoint name, any
 * direction information and other state that may be required.
 */
static void __devinit s3c_hsotg_initep(struct s3c_hsotg *hsotg,
				       struct s3c_hsotg_ep *hs_ep,
				       int epnum)
{
	u32 ptxfifo;
#ifndef U_BOOT
	char *dir;

	if (epnum == 0)
		dir = "";
	else if ((epnum % 2) == 0) {
		dir = "out";
	} else {
		dir = "in";
		hs_ep->dir_in = 1;
	}
	snDBGOUT(hs_ep->name, sizeof(hs_ep->name), "ep%d%s", epnum, dir);
#else	/* #ifdef U_BOOT */
	hs_ep->name[0] = 'e';
	hs_ep->name[1] = 'p';
	hs_ep->name[2] = '0' + epnum;
	if (epnum == 0)
		hs_ep->name[3] = '\0';
	else if ((epnum % 2) == 0) {
		hs_ep->name[3] = 'o';
		hs_ep->name[4] = 'u';
		hs_ep->name[5] = 't';
		hs_ep->name[6] = '\0';
	} else {
		hs_ep->name[3] = 'i';
		hs_ep->name[4] = 'n';
		hs_ep->name[5] = '\0';
		hs_ep->dir_in  = 1;
	}
#endif
	hs_ep->index = epnum;

	INIT_LIST_HEAD(&hs_ep->queue);
	INIT_LIST_HEAD(&hs_ep->ep.ep_list);

	spin_lock_init(&hs_ep->lock);

	/* add to the list of endpoints known by the gadget driver */
	if (epnum)
		list_add_tail(&hs_ep->ep.ep_list, &hsotg->gadget.ep_list);

	hs_ep->parent = hsotg;
	hs_ep->ep.name = hs_ep->name;
	/*LF2000 endpoint sizes are 64, 512, and 1024		*/
	/* documentation shows 0x0900 32 bit word Data FIFO RAM	*/
	switch(hs_ep->index) {
		case 0: hs_ep->ep.maxpacket = EP0_MPS_LIMIT; break;
		case 1: hs_ep->ep.maxpacket = EP1_MPS_LIMIT; break;
		case 2: hs_ep->ep.maxpacket = EP2_MPS_LIMIT; break;
		case 3: hs_ep->ep.maxpacket = EP3_MPS_LIMIT; break;
		case 4: hs_ep->ep.maxpacket = EP4_MPS_LIMIT; break;
		case 5: hs_ep->ep.maxpacket = EP5_MPS_LIMIT; break;
		case 6: hs_ep->ep.maxpacket = EP6_MPS_LIMIT; break;
		case 7: hs_ep->ep.maxpacket = EP7_MPS_LIMIT; break;
	}
	hs_ep->ep.ops = &s3c_hsotg_ep_ops;

	/* Read the FIFO size for the Periodic TX FIFO, even if we're
	 * an OUT endpoint, we may as well do this if in future the
	 * code is changed to make each endpoint's direction changeable.
	 */

	ptxfifo = readl(hsotg->regs + S3C_DPTXFSIZn(epnum));
	hs_ep->fifo_size = S3C_DPTXFSIZn_DPTxFSize_GET(ptxfifo) * 4;

	/* if we're using dma, we need to set the next-endpoint pointer
	 * to be something valid.
	 */

	if (using_dma(hsotg)) {
		u32 next = S3C_DxEPCTL_NextEp((epnum + 1) % S3C_HSOTG_EPS);
		writel(next, hsotg->regs + S3C_DIEPCTL(epnum));
		writel(next, hsotg->regs + S3C_DOEPCTL(epnum));
	}
}

static void s3c_hsotg_init(struct s3c_hsotg *hsotg)
{
	u32 cfg4;

	/* unmask subset of endpoint interrupts */

	writel(S3C_DIEPMSK_TimeOUTMsk | S3C_DIEPMSK_AHBErrMsk |
	       S3C_DIEPMSK_EPDisbldMsk | S3C_DIEPMSK_XferComplMsk,
	       hsotg->regs + S3C_DIEPMSK);

	writel(S3C_DOEPMSK_SetupMsk | S3C_DOEPMSK_AHBErrMsk |
	       S3C_DOEPMSK_EPDisbldMsk | S3C_DOEPMSK_XferComplMsk,
	       hsotg->regs + S3C_DOEPMSK);

	writel(0, hsotg->regs + S3C_DAINTMSK);

	/* Be in disconnected state until gadget is registered */
	__orr32(hsotg->regs + S3C_DCTL, S3C_DCTL_SftDiscon);

	if (0) {
		/* post global nak until we're ready */
		writel(S3C_DCTL_SGNPInNAK | S3C_DCTL_SGOUTNak,
		       hsotg->regs + S3C_DCTL);
	}

	/* setup fifos */

	dev_dbg(hsotg->dev, "GRXFSIZ=0x%08x, GNPTXFSIZ=0x%08x\n",
		readl(hsotg->regs + S3C_GRXFSIZ),
		readl(hsotg->regs + S3C_GNPTXFSIZ));

	s3c_hsotg_init_fifo(hsotg);

	/* set the PLL on, remove the HNP/SRP and set the PHY */
	writel(S3C_GUSBCFG_PHYIf16 | S3C_GUSBCFG_TOutCal(7) | (0x5 << 10),
	       hsotg->regs + S3C_GUSBCFG);

	writel(using_dma(hsotg) ? S3C_GAHBCFG_DMAEn : 0x0,
	       hsotg->regs + S3C_GAHBCFG);

	/* check hardware configuration */

	cfg4 = readl(hsotg->regs + S3C_GHWCFG4);
	hsotg->dedicated_fifos = (cfg4 >> 25) & 1;

	dev_info(hsotg->dev, "%s fifos\n",
		 hsotg->dedicated_fifos ? "dedicated" : "shared");
}

static void s3c_hsotg_dump(struct s3c_hsotg *hsotg)
{
#ifdef DEBUG
#ifndef U_BOOT
	struct device *dev = hsotg->dev;
#endif
	void __iomem *regs = hsotg->regs;
	u32 val;
	int idx;

	dev_info(dev, "DCFG=0x%08x, DCTL=0x%08x, DIEPMSK=%08x\n",
		 readl(regs + S3C_DCFG), readl(regs + S3C_DCTL),
		 readl(regs + S3C_DIEPMSK));

	dev_info(dev, "GAHBCFG=0x%08x, GHWCFG1=0x%08x,  GINTSTS=0x%08x\n",
		 readl(regs + S3C_GAHBCFG), readl(regs + S3C_GHWCFG1),
		 readl(regs + S3C_GINTSTS));

	dev_info(dev, "GRXFSIZ=0x%08x, GNPTXFSIZ=0x%08x\n",
		 readl(regs + S3C_GRXFSIZ), readl(regs + S3C_GNPTXFSIZ));

	/* show periodic fifo settings */

	for (idx = 1; idx < S3C_HSOTG_EPS; idx++) {
		val = readl(regs + S3C_DPTXFSIZn(idx));
		dev_info(dev, "DPTx[%d] FSize=%d, StAddr=0x%08x\n", idx,
			 val >> S3C_DPTXFSIZn_DPTxFSize_SHIFT,
			 val & S3C_DPTXFSIZn_DPTxFStAddr_MASK);
	}

	for (idx = 0; idx < S3C_HSOTG_EPS; idx++) {
		dev_info(dev,
			 "ep%d-in: EPCTL=0x%08x, SIZ=0x%08x, DMA=0x%08x\n", idx,
			 readl(regs + S3C_DIEPCTL(idx)),
			 readl(regs + S3C_DIEPTSIZ(idx)),
			 readl(regs + S3C_DIEPDMA(idx)));

		val = readl(regs + S3C_DOEPCTL(idx));
		dev_info(dev,
			 "ep%d-out: EPCTL=0x%08x, SIZ=0x%08x, DMA=0x%08x\n",
			 idx, readl(regs + S3C_DOEPCTL(idx)),
			 readl(regs + S3C_DOEPTSIZ(idx)),
			 readl(regs + S3C_DOEPDMA(idx)));

	}

	dev_info(dev, "DVBUSDIS=0x%08x, DVBUSPULSE=%08x\n",
		 readl(regs + S3C_DVBUSDIS), readl(regs + S3C_DVBUSPULSE));
#endif
}


#ifndef U_BOOT	/* removed on the way to an error-free compilation */
/**
 * state_show - debugfs: show overall driver and device state.
 * @seq: The seq file to write to.
 * @v: Unused parameter.
 *
 * This debugfs entry shows the overall state of the hardware and
 * some general information about each of the endpoints available
 * to the system.
 */
static int state_show(struct seq_file *seq, void *v)
{
	struct s3c_hsotg *hsotg = seq->private;
	void __iomem *regs = hsotg->regs;
	int idx;

	seq_DBGOUT(seq, "GOTGCTL  =0x%08x, GOTGINT  =0x%08x, GAHBCFG       =0x%08x, GUSBCFG       =0x%08x\n",
		 readl(regs + S3C_GOTGCTL),
		 readl(regs + S3C_GOTGINT),
		 readl(regs + S3C_GAHBCFG),
		 readl(regs + S3C_GUSBCFG));

	seq_DBGOUT(seq, "GRSTCTL  =0x%08x, GINTSTS  =0x%08x, GINTMSK       =0x%08x, GRXSTSR       =0x%08x\n",
		 readl(regs + S3C_GRSTCTL),
		 readl(regs + S3C_GINTSTS),
		 readl(regs + S3C_GINTMSK),
		 readl(regs + S3C_GRXSTSR));

	seq_DBGOUT(seq, "GRXSTSP  =0x%08x, GRXFSIZ  =0x%08x, GNPTXFSIZ     =0x%08x, GNPTXSTS      =0x%08x\n",
		 readl(regs + S3C_GRXSTSP),
		 readl(regs + S3C_GRXFSIZ),
		 readl(regs + S3C_GNPTXFSIZ),
		 readl(regs + S3C_GNPTXSTS));

	seq_DBGOUT(seq, "GI2CCTL  =0x%08x, GPVNDCTL =0x%08x, GGPIO         =0x%08x, GUID          =0x%08x\n",
		 readl(regs + S3C_GI2CCTL),
		 readl(regs + S3C_GPVNDCTL),
		 readl(regs + S3C_GGPIO),
		 readl(regs + S3C_GUID));

	seq_DBGOUT(seq, "GSNPSID  =0x%08x, GHWCFG1  =0x%08x, GHWCFG2       =0x%08x, GHWCFG3       =0x%08x\n",
		 readl(regs + S3C_GSNPSID),
		 readl(regs + S3C_GHWCFG1),
		 readl(regs + S3C_GHWCFG2),
		 readl(regs + S3C_GHWCFG3));

	seq_DBGOUT(seq, "GHWCFG4  =0x%08x, GLPMCFG  =0x%08x, PHYPOR        =0x%08x, TESTPARM3     =0x%08x\n",
		 readl(regs + S3C_GHWCFG4),
		 readl(regs + S3C_GLPMCFG),
		 readl(regs + S3C_PHYPOR),
		 readl(regs + S3C_TESTPARM3));

	seq_DBGOUT(seq, "TESTPARM4=0x%08x, LINCTRL  =0x%08x, TESTPARM6     =0x%08x, TESTPARM7     =0x%08x\n",
		 readl(regs + S3C_TESTPARM4),
		 readl(regs + S3C_LINCTRL),
		 readl(regs + S3C_TESTPARM6),
		 readl(regs + S3C_TESTPARM7));

	seq_DBGOUT(seq, "TESTPARM8=0x%08x, TESTPARM9=0x%08x, USB_OTG_CLKENB=0x%08x, USB_OTG_CLKGEN=0x%08x\n",
		 readl(regs + S3C_TESTPARM8),
		 readl(regs + S3C_TESTPARM9),
		 readl(regs + S3C_USB_OTG_CLKENB),
		 readl(regs + S3C_USB_OTG_CLKGEN));

	seq_DBGOUT(seq, "DCFG=0x%08x, DCTL=0x%08x, DSTS=0x%08x\n",
		 readl(regs + S3C_DCFG),
		 readl(regs + S3C_DCTL),
		 readl(regs + S3C_DSTS));

	seq_DBGOUT(seq, "DIEPMSK=0x%08x, DOEPMASK=0x%08x\n",
		   readl(regs + S3C_DIEPMSK), readl(regs + S3C_DOEPMSK));

	seq_DBGOUT(seq, "DAINTMSK=0x%08x, DAINT=0x%08x\n",
		   readl(regs + S3C_DAINTMSK),
		   readl(regs + S3C_DAINT));

	seq_DBGOUT(seq, "\nEndpoint status:\n");

	for (idx = 0; idx < S3C_HSOTG_EPS; idx++) {
		u32 in, out;

		in = readl(regs + S3C_DIEPCTL(idx));
		out = readl(regs + S3C_DOEPCTL(idx));

		seq_DBGOUT(seq, "ep%d: DIEPCTL=0x%08x, DOEPCTL=0x%08x",
			   idx, in, out);

		in = readl(regs + S3C_DIEPTSIZ(idx));
		out = readl(regs + S3C_DOEPTSIZ(idx));

		seq_DBGOUT(seq, ", DIEPTSIZ=0x%08x, DOEPTSIZ=0x%08x",
			   in, out);

		seq_DBGOUT(seq, "\n");
	}

	return 0;
}

static int state_open(struct inode *inode, struct file *file)
{
	return single_open(file, state_show, inode->i_private);
}

static const struct file_operations state_fops = {
	.owner		= THIS_MODULE,
	.open		= state_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/**
 * fifo_show - debugfs: show the fifo information
 * @seq: The seq_file to write data to.
 * @v: Unused parameter.
 *
 * Show the FIFO information for the overall fifo and all the
 * periodic transmission FIFOs.
*/
static int fifo_show(struct seq_file *seq, void *v)
{
	struct s3c_hsotg *hsotg = seq->private;
	void __iomem *regs = hsotg->regs;
	u32 val;
	int idx;

	seq_DBGOUT(seq, "Non-periodic FIFOs:\n");
	seq_DBGOUT(seq, "RXFIFO: Size %d\n", readl(regs + S3C_GRXFSIZ));

	val = readl(regs + S3C_GNPTXFSIZ);
	seq_DBGOUT(seq, "NPTXFIFO: Size %d, Start 0x%08x\n",
		   val >> S3C_GNPTXFSIZ_NPTxFDep_SHIFT,
		   val & S3C_GNPTXFSIZ_NPTxFStAddr_MASK);

	seq_DBGOUT(seq, "\nPeriodic TXFIFOs:\n");

	for (idx = 1; idx < S3C_HSOTG_EPS; idx++) {
		val = readl(regs + S3C_DPTXFSIZn(idx));

		seq_DBGOUT(seq, "\tDPTXFIFO%2d: Size %d, Start 0x%08x\n", idx,
			   val >> S3C_DPTXFSIZn_DPTxFSize_SHIFT,
			   val & S3C_DPTXFSIZn_DPTxFStAddr_MASK);
	}

	return 0;
}

static int fifo_open(struct inode *inode, struct file *file)
{
	return single_open(file, fifo_show, inode->i_private);
}

static const struct file_operations fifo_fops = {
	.owner		= THIS_MODULE,
	.open		= fifo_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static const char *decode_direction(int is_in)
{
	return is_in ? "in" : "out";
}

/**
 * ep_show - debugfs: show the state of an endpoint.
 * @seq: The seq_file to write data to.
 * @v: Unused parameter.
 *
 * This debugfs entry shows the state of the given endpoint (one is
 * registered for each available).
*/
static int ep_show(struct seq_file *seq, void *v)
{
	struct s3c_hsotg_ep *ep = seq->private;
	struct s3c_hsotg *hsotg = ep->parent;
	struct s3c_hsotg_req *req;
	void __iomem *regs = hsotg->regs;
	int index = ep->index;
	int show_limit = S3C_HSOTG_EPS;
	unsigned long flags;

	seq_DBGOUT(seq, "Endpoint index %d, named %s,  dir %s:\n",
		   ep->index, ep->ep.name, decode_direction(ep->dir_in));

	/* first show the register state */

	seq_DBGOUT(seq, "\tDIEPCTL=0x%08x, DOEPCTL=0x%08x\n",
		   readl(regs + S3C_DIEPCTL(index)),
		   readl(regs + S3C_DOEPCTL(index)));

	seq_DBGOUT(seq, "\tDIEPDMA=0x%08x, DOEPDMA=0x%08x\n",
		   readl(regs + S3C_DIEPDMA(index)),
		   readl(regs + S3C_DOEPDMA(index)));

	seq_DBGOUT(seq, "\tDIEPINT=0x%08x, DOEPINT=0x%08x\n",
		   readl(regs + S3C_DIEPINT(index)),
		   readl(regs + S3C_DOEPINT(index)));

	seq_DBGOUT(seq, "\tDIEPTSIZ=0x%08x, DOEPTSIZ=0x%08x\n",
		   readl(regs + S3C_DIEPTSIZ(index)),
		   readl(regs + S3C_DOEPTSIZ(index)));

	seq_DBGOUT(seq, "\n");
	seq_DBGOUT(seq, "mps %d\n", ep->ep.maxpacket);
	seq_DBGOUT(seq, "total_data=%ld\n", ep->total_data);

	seq_DBGOUT(seq, "request list (%p,%p):\n",
		   ep->queue.next, ep->queue.prev);

	spin_lock_irqsave(&ep->lock, flags);

	list_for_each_entry(req, &ep->queue, queue) {
		if (--show_limit < 0) {
			seq_DBGOUT(seq, "not showing more requests...\n");
			break;
		}

		seq_DBGOUT(seq, "%c req %p: %d bytes @%p, ",
			   req == ep->req ? '*' : ' ',
			   req, req->req.length, req->req.buf);
		seq_DBGOUT(seq, "%d done, res %d\n",
			   req->req.actual, req->req.status);
	}

	spin_unlock_irqrestore(&ep->lock, flags);

	return 0;
}

static int ep_open(struct inode *inode, struct file *file)
{
	return single_open(file, ep_show, inode->i_private);
}

static const struct file_operations ep_fops = {
	.owner		= THIS_MODULE,
	.open		= ep_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/**
 * s3c_hsotg_create_debug - create debugfs directory and files
 * @hsotg: The driver state
 *
 * Create the debugfs files to allow the user to get information
 * about the state of the system. The directory name is created
 * with the same name as the device itself, in case we end up
 * with multiple blocks in future systems.
*/
static void __devinit s3c_hsotg_create_debug(struct s3c_hsotg *hsotg)
{
	struct dentry *root;
	unsigned epidx;

	root = debugfs_create_dir(dev_name(hsotg->dev), NULL);
	hsotg->debug_root = root;
	if (IS_ERR(root)) {
		dev_err(hsotg->dev, "cannot create debug root\n");
		return;
	}

	/* create general state file */

	hsotg->debug_file = debugfs_create_file("state", 0444, root,
						hsotg, &state_fops);

	if (IS_ERR(hsotg->debug_file))
		dev_err(hsotg->dev, "%s: failed to create state\n", __func__);

	hsotg->debug_fifo = debugfs_create_file("fifo", 0444, root,
						hsotg, &fifo_fops);

	if (IS_ERR(hsotg->debug_fifo))
		dev_err(hsotg->dev, "%s: failed to create fifo\n", __func__);

	/* create one file for each endpoint */

	for (epidx = 0; epidx < S3C_HSOTG_EPS; epidx++) {
		struct s3c_hsotg_ep *ep = &hsotg->eps[epidx];

		ep->debugfs = debugfs_create_file(ep->name, 0444,
						  root, ep, &ep_fops);

		if (IS_ERR(ep->debugfs))
			dev_err(hsotg->dev, "failed to create %s debug file\n",
				ep->name);
	}
}

/**
 * s3c_hsotg_delete_debug - cleanup debugfs entries
 * @hsotg: The driver state
 *
 * Cleanup (remove) the debugfs files for use on module exit.
*/
static void __devexit s3c_hsotg_delete_debug(struct s3c_hsotg *hsotg)
{
	unsigned epidx;

	for (epidx = 0; epidx < S3C_HSOTG_EPS; epidx++) {
		struct s3c_hsotg_ep *ep = &hsotg->eps[epidx];
		debugfs_remove(ep->debugfs);
	}

	debugfs_remove(hsotg->debug_file);
	debugfs_remove(hsotg->debug_fifo);
	debugfs_remove(hsotg->debug_root);
}
#endif	/* U_BOOT */

#ifndef U_BOOT
static struct s3c_hsotg_plat s3c_hsotg_default_pdata;

static int __devinit s3c_hsotg_probe(struct platform_device *pdev)
{
	struct s3c_hsotg_plat *plat = pdev->dev.platform_data;
	struct device *dev = &pdev->dev;
	struct s3c_hsotg *hsotg;
	struct resource *res;
	int epnum;
	int ret;

//	plat = pdev->dev.platform_data;
	if (!plat)
		plat = &s3c_hsotg_default_pdata;

	hsotg = kzalloc(sizeof(struct s3c_hsotg) +
			sizeof(struct s3c_hsotg_ep) * S3C_HSOTG_EPS,
			GFP_KERNEL);
	if (!hsotg) {
		dev_err(dev, "cannot get memory\n");
		return -ENOMEM;
	}

	hsotg->dev = dev;
	hsotg->plat = plat;

	/* FIXME: sdesters leave OTG clock always enabled in core */
#if 0
	hsotg->clk = clk_get(&pdev->dev, "otg");
	if (IS_ERR(hsotg->clk)) {
		dev_err(dev, "cannot get otg clock\n");
		ret = PTR_ERR(hsotg->clk);
		goto err_mem;
	}
#endif
	/* FIXME: sesters call nxp3200 specific setup routines */
	otg_clk_enable();
	otg_phy_init();

	platform_set_drvdata(pdev, hsotg);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "cannot find register resource 0\n");
		ret = -EINVAL;
		goto err_clk;
	}

	hsotg->regs_res = request_mem_region(res->start, resource_size(res),
					     dev_name(dev));
	if (!hsotg->regs_res) {
		dev_err(dev, "cannot reserve registers\n");
		ret = -ENOENT;
		goto err_clk;
	}

	hsotg->regs = ioremap(res->start, resource_size(res));
	if (!hsotg->regs) {
		dev_err(dev, "cannot map registers\n");
		ret = -ENXIO;
		goto err_regs_res;
	}

	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(dev, "cannot find IRQ\n");
		goto err_regs;
	}

	hsotg->irq = ret;

	ret = request_irq(ret, s3c_hsotg_irq, 0, dev_name(dev), hsotg);
	if (ret < 0) {
		dev_err(dev, "cannot claim IRQ\n");
		goto err_regs;
	}

	dev_info(dev, "regs %p, irq %d\n", hsotg->regs, hsotg->irq);

	/* set up input device for reporting VBUS */
	hsotg->input = input_allocate_device();
	if(!hsotg->input) {
		dev_err(dev, "can't get device for vbus key hack\n");
		ret = -ENOMEM;
		goto err_regs;
	}
	hsotg->input->name = "LF2000 USB";
	hsotg->input->phys = "lf2000/usb";
	hsotg->input->id.bustype = BUS_HOST;
	hsotg->input->id.vendor = 0x0001;
	hsotg->input->id.product = 0x0001;
	hsotg->input->id.version = 0x0001;

	/* we only support a 'switch' event */
	hsotg->input->evbit[0] = BIT(EV_SW);
	/* we don't offer any keys */
	hsotg->input->keycode = NULL;
	hsotg->input->keycodesize = 0;
	hsotg->input->keycodemax = 0;
	
	/* reusing 'lid' for our switch */
	set_bit(SW_LID, hsotg->input->swbit);

	ret = input_register_device(hsotg->input);
	if(ret) {
		dev_err(dev, "can't register dev for vbus key hack\n");
		goto err_regs;
	}

	/* set up connection state timer */
	setup_timer( &(hsotg->conn_state_timer), s3c_hsotg_update_connected_state, (unsigned long)hsotg );

	device_initialize(&hsotg->gadget.dev);

	dev_set_name(&hsotg->gadget.dev, "gadget");

	hsotg->gadget.is_dualspeed = 1;
	hsotg->gadget.ops = &s3c_hsotg_gadget_ops;
	hsotg->gadget.name = dev_name(dev);

	hsotg->gadget.dev.parent = dev;
	hsotg->gadget.dev.dma_mask = dev->dma_mask;

	/* setup endpoint information */

	INIT_LIST_HEAD(&hsotg->gadget.ep_list);
	hsotg->gadget.ep0 = &hsotg->eps[0].ep;

	/* allocate EP0 request */

	hsotg->ctrl_req = s3c_hsotg_ep_alloc_request(&hsotg->eps[0].ep,
						     GFP_KERNEL);
	if (!hsotg->ctrl_req) {
		dev_err(dev, "failed to allocate ctrl req\n");
		goto err_regs;
	}

	/* reset the system */

	/* FIXME: sesters maybe we're already setup with otg_clk_enable() call */
#if 0
	clk_enable(hsotg->clk);
#endif
#if 0	if (plat->phy_init)
		plat->phy_init(pdev, S5P_USB_PHY_DEVICE);
#endif
	s3c_hsotg_corereset(hsotg);
	s3c_hsotg_init(hsotg);

	/* initialise the endpoints now the core has been initialised */
	for (epnum = 0; epnum < S3C_HSOTG_EPS; epnum++)
		s3c_hsotg_initep(hsotg, &hsotg->eps[epnum], epnum);

	s3c_hsotg_create_debug(hsotg);

	s3c_hsotg_dump(hsotg);

	our_hsotg = hsotg;
	return 0;

err_regs:
	iounmap(hsotg->regs);

err_regs_res:
	release_resource(hsotg->regs_res);
	kfree(hsotg->regs_res);
err_clk:
	clk_put(hsotg->clk);
//err_mem:
	kfree(hsotg);
	return ret;
}

static int __devexit s3c_hsotg_remove(struct platform_device *pdev)
{
	struct s3c_hsotg *hsotg = platform_get_drvdata(pdev);

	s3c_hsotg_delete_debug(hsotg);

	usb_gadget_unregister_driver(hsotg->driver);

	free_irq(hsotg->irq, hsotg);
	iounmap(hsotg->regs);

	release_resource(hsotg->regs_res);
	kfree(hsotg->regs_res);
	
	if(hsotg->input > 0)
		input_unregister_device(hsotg->input);


	clk_disable(hsotg->clk);
	clk_put(hsotg->clk);

	kfree(hsotg);
	return 0;
}

#if 1
#define s3c_hsotg_suspend NULL
#define s3c_hsotg_resume NULL
#endif

static struct platform_driver s3c_hsotg_driver = {
	.driver		= {
		.name	= OTG_GADGET_DEV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= s3c_hsotg_probe,
	.remove		= __devexit_p(s3c_hsotg_remove),
	.suspend	= s3c_hsotg_suspend,
	.resume		= s3c_hsotg_resume,
};

static int __init s3c_hsotg_modinit(void)
{
	return platform_driver_register(&s3c_hsotg_driver);
}

static void __exit s3c_hsotg_modexit(void)
{
	platform_driver_unregister(&s3c_hsotg_driver);
}

module_init(s3c_hsotg_modinit);
module_exit(s3c_hsotg_modexit);

#endif	/* ifndef U_BOOT */


#ifdef U_BOOT

#define	S3C_GINTSTS_WAIT      ( S3C_GINTSTS_WkUpInt	  | \
				S3C_GINTSTS_SessReqInt    | \
				S3C_GINTSTS_DisconnInt    | \
				S3C_GINTSTS_ConIDStsChng  | \
				S3C_GINTSTS_PTxFEmp	  | \
				S3C_GINTSTS_OEPInt	  | \
				S3C_GINTSTS_IEPInt	  | \
				S3C_GINTSTS_EnumDone	  | \
				S3C_GINTSTS_USBRst	  | \
				S3C_GINTSTS_USBSusp	  | \
				S3C_GINTSTS_ErlySusp	  | \
				S3C_GINTSTS_GOUTNakEff	  | \
				S3C_GINTSTS_GINNakEff	  | \
				S3C_GINTSTS_NPTxFEmp	  | \
				S3C_GINTSTS_RxFLvl	  | \
				S3C_GINTSTS_OTGInt	  | \
				S3C_GINTSTS_ModeMis )



/* returns 0 if didn't call s3c_hsotg_irq(); 
 * returns -1 if it detects disconnection from usb
 * otherwise returns 1 */
int wait_and_respond(int callerID)
{
	int ret = 0;
	int last_conn_state = our_hsotg->conn_state;

	/* wait for GINTSTS bits to be set */
	if (readl(S3C_UDC_OTG_GINTSTS) & S3C_GINTSTS_WAIT )
	{
		DBGOUT("\nwait_and_respond(%d)\n", callerID);
		//printk(KERN_ALERT "%s:%s.%d\n", __FILE__, __func__, __LINE__);

		s3c_hsotg_irq(56, our_hsotg);

		ret = 1;
	}

#if defined (CONFIG_SOC_LFP100)
	lfp100_monitor_power_button();
#endif

	run_inactivity_timer(0);	/* shutdown if no traffic */

#if defined(CONFIG_LCD_SCREENS)
	/* update screen if needed */
	run_screen_state(SCREEN_REFRESH);
#endif
	return ret;
}

int interrupt_init (void);
void enable_interrupts (void);
int lf_uboot_msg_init(void);
extern struct fsg_common common;
int fsg_main_thread(void *);
void start_fsg_thread(void);
int  get_cpsr(void);
extern unsigned int current_psr;
int get_irq_jump(void);
extern unsigned int irq_jmp_addr;
extern unsigned int LF_HSOTG_IRQ_;
extern unsigned int HIGH_BIT_MASK_;

#define INTMODEL	0xC0000808
#define INTMODEH	0xC000080C
#define INTMASKL	0xC0000810
#define INTMASKH	0xC0000814
#define INTPRIORDER	0xC0000818
#define INTPENDL	0xC0000820
#define INTPENDH	0xC0000824
#define USB_OTG_MASK	0x01000000

int arch_interrupt_init(void) 
{
	/* make all interrupts Normal (not FIQ)
	 * mask all interrupts except OTG
	 */
	writel (0x00000000, INTMODEL);
	writel (0x00000000, INTMODEH);
#if 1	/* 10nov11 */
	writel (0xFFFFFFFF, INTPENDL);	/* clear any pending interrupts */
	writel (0xFFFFFFFF, INTPENDH);	/* clear any pending interrupts */
	writel (0x00000000, INTMASKL);	/* un-mask all */
	writel (0x00000000, INTMASKH);	/* un-mask all */
#else
	writel (0xEF60FCBF, INTMASKL);	/* mask all that aren't reserved */
	writel (0x00247FFF, INTMASKH);	/* mask all but USB OTG */
#endif
	return 0;
}

/* Turn on the USB connection by enabling the pullup resistor */
void udc_connect (void)
{
	/* this is from omap code */
	//UDCDBG ("connect, enable Pullup");
	//outl (0x00000018, FUNC_MUX_CTRL_D);

/* this is from s3c_hsotg_probe() */	
	struct s3c_hsotg *hsotg;
	int epnum;


#if 1	/* 10feb12 */
	//nx_i2c_init();
	//init_lfp100();


#endif
	hsotg = kzalloc(sizeof(struct s3c_hsotg) +
			sizeof(struct s3c_hsotg_ep) * S3C_HSOTG_EPS,
			GFP_KERNEL);
	if (!hsotg) {
		dev_err(dev, "cannot get memory\n");
		return /*-ENOMEM*/;
	}
	our_hsotg = hsotg;

	run_inactivity_timer(1);	/* initialize inactivity timer */
	hsotg->conn_event_timer = 0;
	hsotg->enum_done = false;

	/* FIXME: sesters call nxp3200 specific setup routines */
	otg_clk_enable();
	otg_phy_init();

	DBGOUT("%s:%s.%d\n", __FILE__, __func__, __LINE__);
	/* FIXME: sesters The USB initialization could be simplified,
	 *                but keep this iROMBOOT form for now
	 */
	reconfig_usbd();

	DBGOUT("%s:%s.%d\n", __FILE__, __func__, __LINE__);
	/* TODO: change this to hard-code an address */
	hsotg->regs = (void __iomem *)SOC_PA_OTG;

	DBGOUT("%s:%s.%d\n", __FILE__, __func__, __LINE__);
	DBGOUT("\nRegs after otg_phy_init() ------------------\n");
	s3c_hsotg_dump(hsotg);
	DBGOUT("%s:%s.%d\n", __FILE__, __func__, __LINE__);
	DBGOUT("\n--------------------------------------------\n\n");

	/* We'll define our own do_irq() and have it call s3c_hsotg_irq() */
	hsotg->irq = 56;
	dev_info(dev, "regs %p, irq %d\n", hsotg->regs, hsotg->irq);

	hsotg->gadget.is_dualspeed = 1;
	hsotg->gadget.ops = &s3c_hsotg_gadget_ops;
	hsotg->gadget.name = "lf2000-hsotg";

	/* setup endpoint information */

	DBGOUT("%s:%s.%d\n", __FILE__, __func__, __LINE__);
	INIT_LIST_HEAD(&hsotg->gadget.ep_list);
	hsotg->gadget.ep0 = &hsotg->eps[0].ep;

	/* allocate EP0 request */

	hsotg->ctrl_req = s3c_hsotg_ep_alloc_request(&hsotg->eps[0].ep,
						     GFP_KERNEL);
	if (!hsotg->ctrl_req) {
		dev_err(dev, "failed to allocate ctrl req\n");
		goto err_regs;
	}

	/* reset the system */

	/* FIXME: sesters maybe we're already setup with otg_clk_enable() call */
#if 0
	clk_enable(hsotg->clk);
#endif
	DBGOUT("%s:%s.%d\n", __FILE__, __func__, __LINE__);
	s3c_hsotg_corereset(hsotg);
	s3c_hsotg_init(hsotg);

	/* initialise the endpoints now the core has been initialised */
	for (epnum = 0; epnum < S3C_HSOTG_EPS; epnum++)
		s3c_hsotg_initep(hsotg, &hsotg->eps[epnum], epnum);

	s3c_hsotg_dump(hsotg);

	DBGOUT("%s:%s.%d\n", __FILE__, __func__, __LINE__);
	lf_uboot_msg_init();

	/* poll the GINTSTS register, like USBBOOT() */

	DBGOUT("%s:%s.%d\n", __FILE__, __func__, __LINE__);

	while (1) {
#if defined (CONFIG_SOC_LFP100)
		lfp100_monitor_power_button();
#endif

		run_inactivity_timer(0);	/* shutdown if no traffic */

		/* process download packet */
		DBGOUT("%s:%s.%d\n", __FILE__, __func__, __LINE__);
		if (wait_and_respond(0) > 0)
		{
			DBGOUT("%s:%s.%d\n", __FILE__, __func__, __LINE__);
			fsg_main_thread(&common);
#ifndef RAMDISK
			DBGOUT("%s:%s.%d\n", __FILE__, __func__, __LINE__);
			check_if_download_verified();
#endif	/* RAMDISK */
		}
	}

	return;

err_regs:
	kfree(hsotg);
	return;
}

/* Turn off the USB connection by disabling the pullup resistor */
void udc_disconnect (void)
{
	//UDCDBG ("disconnect, disable Pullup");
	//outl (0x00000000, FUNC_MUX_CTRL_D);
}


#endif	/* ifdef U_BOOT */

#ifndef U_BOOT
MODULE_DESCRIPTION("Samsung S3C USB High-speed/OtG device");
MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lf2000-hsotg");
MODULE_ALIAS("platform:"OTG_GADGET_DEV_NAME);
#endif

