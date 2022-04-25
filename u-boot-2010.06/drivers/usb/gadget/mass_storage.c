/*
 * mass_storage.c -- Mass Storage USB Gadget
 *
 * Copyright (C) 2003-2008 Alan Stern
 * Copyright (C) 2009 Samsung Electronics
 *                    Author: Michal Nazarewicz <m.nazarewicz@samsung.com>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


/*
 * The Mass Storage Gadget acts as a USB Mass Storage device,
 * appearing to the host as a disk drive or as a CD-ROM drive.  In
 * addition to providing an example of a genuinely useful gadget
 * driver for a USB device, it also illustrates a technique of
 * double-buffering for increased throughput.  Last but not least, it
 * gives an easy way to probe the behavior of the Mass Storage drivers
 * in a USB host.
 *
 * Since this file serves only administrative purposes and all the
 * business logic is implemented in f_mass_storage.* file.  Read
 * comments in this file for more detailed description.
 */

/*------------------------------------------------------------------------------
 * This file is derived from mass_storage.c in Leapfrog's LF2000 Linux
 * drivers/usb/gadget directory.  It has been modified for use in u-boot's
 * gadget driver.  The gadget driver comprises two compilation units.  This
 * file is one of them.  The other is lf2000-hsotg.c.
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
 *----------------------------------------------------------------------------*/


#define U_BOOT 

#ifndef U_BOOT
#include <linux/kernel.h>
#include <linux/utsname.h>
#endif
#include <linux/usb/ch9.h>

#ifdef U_BOOT
#include <common.h>
#include "linux_to_uboot.h"
#include <linux/compiler.h>
#include <linux/err.h>
#include <linux/mtd/compat.h>
#include <lf2000_board.h>

#ifdef WARN_ON
#undef WARN_ON
#define WARN_ON(a) 0 
#endif

#endif	/* U_BOOT */

/*-------------------------------------------------------------------------*/

#define DRIVER_DESC		"Mass Storage Gadget"
#define DRIVER_VERSION		"2009/09/11"

/*-------------------------------------------------------------------------*/

/*
 * kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */

#include "composite.c"
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#ifdef U_BOOT
#ifdef RAMDISK
#include "ramdisk.c"
#else
#include "cbf.c"
#endif	/* RAMDISK */
#endif	/* U_BOOT */
#include "f_mass_storage.c"

/*-------------------------------------------------------------------------*/

static struct usb_device_descriptor msg_device_desc = {
	.bLength =		sizeof msg_device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	.bcdUSB =		cpu_to_le16(0x0200),
	.bDeviceClass =		USB_CLASS_PER_INTERFACE,

	/* Vendor and product id can be overridden by module parameters.  */
	.idVendor =		cpu_to_le16(FSG_VENDOR_ID),
	.idProduct =		cpu_to_le16(FSG_PRODUCT_ID),
	.bNumConfigurations =	1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,

	/*
	 * REVISIT SRP-only hardware is possible, although
	 * it would not be called "OTG" ...
	 */
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};


/****************************** Configurations ******************************/

static struct fsg_module_parameters mod_data = {
	.stall = 1
};
#ifndef U_BOOT
FSG_MODULE_PARAMETERS(/* no prefix */, mod_data);
#endif

static unsigned long msg_registered = 0;
static void msg_cleanup(void);

static int msg_thread_exits(struct fsg_common *common)
{
	msg_cleanup();
	return 0;
}

#ifdef U_BOOT
struct fsg_common common;
#endif
static int __init msg_do_config(struct usb_configuration *c)
{
	static const struct fsg_operations ops = {
		.thread_exits = msg_thread_exits,
	};
#ifndef U_BOOT
	static struct fsg_common common;
#endif

	struct fsg_common *retp;
	struct fsg_config config;
	int ret;

	if (gadget_is_otg(c->cdev->gadget)) {
		c->descriptors = otg_desc;
		c->bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	fsg_config_from_params(&config, &mod_data);
	config.ops = &ops;

	retp = fsg_common_init(&common, c->cdev, &config);
	if (IS_ERR(retp))
		return PTR_ERR(retp);

	ret = fsg_bind_config(c->cdev, c, &common);
#ifndef U_BOOT
	fsg_common_put(&common);
#endif
	return ret;
}

static struct usb_configuration msg_config_driver = {
	.label			= "Linux File-Backed Storage",
	.bConfigurationValue	= 1,
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
};


/****************************** Gadget Bind ******************************/

static int __init msg_bind(struct usb_composite_dev *cdev)
{
	int status;

	status = usb_add_config(cdev, &msg_config_driver, msg_do_config);
	if (status < 0)
		return status;

	dev_info(&cdev->gadget->dev,
		 DRIVER_DESC ", version: " DRIVER_VERSION "\n");
	set_bit(0, &msg_registered);
	return 0;
}


/****************************** Some noise ******************************/

struct usb_composite_driver msg_driver = {
	.name		= "g_mass_storage",
	.dev		= &msg_device_desc,
	.iProduct	= DRIVER_DESC,
	.needs_serial	= 1,
};

#ifndef U_BOOT
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Michal Nazarewicz");
MODULE_LICENSE("GPL");

static int __init msg_init(void)
{
	return usb_composite_probe(&msg_driver, msg_bind);
}
module_init(msg_init);
#else

int lf_uboot_msg_init(void)
{
	switch(get_board_rev()) {
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
#ifdef ENUMERATE_AS_MADRID
		msg_driver.dev->idProduct =
			cpu_to_le16(LF_EMERALD_USB_PRODUCT_ID);
#else
		msg_driver.dev->idProduct =
			cpu_to_le16(LF_VALENCIA_USB_PRODUCT_ID);
#endif
		break;
	case LF2000_BOARD_LUCY:
	case LF2000_BOARD_LUCY_CIP:
	case LF2000_BOARD_LUCY_PP:
#ifdef ENUMERATE_AS_EMERALD
		msg_driver.dev->idProduct =
			cpu_to_le16(LF_EMERALD_USB_PRODUCT_ID);
#else
		msg_driver.dev->idProduct =
			cpu_to_le16(LF_LUCY_USB_PRODUCT_ID);
#endif
		break;
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
#ifdef ENUMERATE_AS_VALENCIA
		msg_driver.dev->idProduct =
			cpu_to_le16(LF_VALENCIA_USB_PRODUCT_ID);
#else
		msg_driver.dev->idProduct =
			cpu_to_le16(LF_RIO_USB_PRODUCT_ID);
#endif
		break;
	case LF2000_BOARD_VTK:
	case LF2000_BOARD_UNKNOWN:
	default:
		msg_driver.dev->idProduct =
			cpu_to_le16(LF_EMERALD_USB_PRODUCT_ID);
	}
	return usb_composite_probe(&msg_driver, msg_bind);
}
#endif

static void msg_cleanup(void)
{
#ifndef U_BOOT
	if (test_and_clear_bit(0, &msg_registered))
		usb_composite_unregister(&msg_driver);
#endif
}
#ifndef U_BOOT
module_exit(msg_cleanup);
#endif

