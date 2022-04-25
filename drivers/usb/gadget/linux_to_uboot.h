/* linux_to_uboot.h -- defines that faciliate moving linux driver to u-boot
 *
 * Copyright 2011 LeapFrog Enterprises Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_TO_UBOOT__H__
#define __LINUX_TO_UBOOT__H__

#ifdef U_BOOT

/* Enable the #define of RAMDISK if you want a ramdisk.
 * If the #define is disabled, the CBF-processing code will be enabled.
 */
//#define RAMDISK 

#define __init 
#define __iomem  
#define dev_dbg(a,...) printf(__VA_ARGS__)
#define dev_err(a,...) printf(__VA_ARGS__)
#define dev_info(a,...) printf(__VA_ARGS__)
#define dev_warn(a,...) printf(__VA_ARGS__)
#define dev_vdbg(a,...) printf(__VA_ARGS__)
#define WARN_ON(a) 
#define WARN_ON_ONCE(a) 
#define GFP_ATOMIC 0
#define GFP_KERNEL 1
#define spin_lock_init(a) 
#define spin_lock(a) 
#define spin_unlock(a) 
#define spin_lock_irq(a) 
#define spin_unlock_irq(a) 
#define	spin_lock_irqsave(a,b) 
#define	spin_unlock_irqrestore(a,b) 

#define init_rwsem(a) 
#define up_read(a) 
#define down_read(a) 
#define down_write(a) 
#define up_write(a) 
#define smp_rmb() 
#define smp_wmb() 
#define signal_pending(current) 0
#define PAGE_CACHE_SIZE	4096
#define PATH_MAX 256


typedef int irqreturn_t;
#define IRQ_HANDLED 0

#define	msleep(_n)		udelay(_n*1000)


enum dma_data_direction {
        DMA_BIDIRECTIONAL = 0,
        DMA_TO_DEVICE = 1,
        DMA_FROM_DEVICE = 2,
        DMA_NONE = 3,
};

	/* next is from include/linux/kernel.h */
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

	/* next two are from include/linux/bitops.h */
#define BITS_PER_BYTE           8
#define BITS_TO_LONGS(nr)       DIV_ROUND_UP(nr, BITS_PER_BYTE * sizeof(long))

	/* next is from include/linux/types.h */
#define DECLARE_BITMAP(name,bits) \
         unsigned long name[BITS_TO_LONGS(bits)]

/* #defines used in our u-boot usb mass storage class gadget driver */
#define LF_USB_VENDOR_ID		0x0f63

#define LF_EMERALD_USB_PRODUCT_ID	0x0016
#define LF_MADRID_USB_PRODUCT_ID	0x001B
#define LF_LUCY_USB_PRODUCT_ID		0x001D
#define LF_VALENCIA_USB_PRODUCT_ID	0x001F
#define LF_RIO_USB_PRODUCT_ID		0x0025

#define LF_USB_GADGET_VBUS_NO_POWER	0	// No USB Power
#define LF_USB_GADGET_VBUS_POWER	500	// USB Power requested

/* Enable the next #define to enumerate Lucy as an Emerald */
/* #define ENUMERATE_AS_EMERALD  */
/* Enable the next #define to enumerate Valencia as a Madrid */
/* #define ENUMERATE_AS_MADRID */

#if   defined(CONFIG_MACH_NXP3200_L2K_DTK) || defined(CONFIG_MACH_NXP3200_LUCY_CIP_DTK)
/* Enable the next #define to enumerate as an Emerald */
#ifdef ENUMERATE_AS_EMERALD
#define LF_USB_PRODUCT_ID	LF_EMERALD_USB_PRODUCT_ID
#define LF_PRODUCT_ID_VOL	"LeapsterExplorer"
#define LF_PRODUCT_STRING	"Leapster Explorer"
#warning "Using Emerald's USB PID"

#else
#define LF_USB_PRODUCT_ID	LF_LUCY_USB_PRODUCT_ID
#define LF_PRODUCT_ID_VOL	"LGS Explorer"
#define LF_PRODUCT_STRING	"Leapster GS Explorer"
#warning "Using Lucy's USB PID"
#endif

#elif defined(CONFIG_MACH_NXP3200_M2K_DTK) || defined(CONFIG_MACH_NXP3200_VAL_CIP_DTK)
/* Enable the next #define to enumerate as a Madrid */
#ifdef ENUMERATE_AS_MADRID
#define LF_USB_PRODUCT_ID	LF_MADRID_USB_PRODUCT_ID
#define LF_PRODUCT_ID_VOL	"LeapPad Explorer"
#define LF_PRODUCT_STRING	"LeapPad Explorer"
#warning "Using Madrid's USB PID"

#else
#define LF_USB_PRODUCT_ID	LF_VALENCIA_USB_PRODUCT_ID
#define LF_PRODUCT_ID_VOL	"LeapPad2"
#define LF_PRODUCT_STRING	"LeapPad2 Explorer"
#warning "Using Valencia's USB PID"
#endif

#elif defined(CONFIG_MACH_NXP3200_RIO_DTK)
/* Enable the next #define to enumerate as a Valencia */
#ifdef ENUMERATE_AS_VALENCIA
#define LF_USB_PRODUCT_ID	LF_VALENCIA_USB_PRODUCT_ID
#define LF_PRODUCT_ID_VOL	"LeapPad2"
#define LF_PRODUCT_STRING	"LeapPad2 Explorer"
#warning "Using Valencia's USB PID"

#else
#define LF_USB_PRODUCT_ID	LF_RIO_USB_PRODUCT_ID
#define LF_PRODUCT_ID_VOL	"LeapPad Ultra"
#define LF_PRODUCT_STRING	"LeapPad Ultra"
#warning "Using Rio's USB PID"
#endif

#elif defined(CONFIG_MACH_NXP3200_VTK_DTK)
#define LF_USB_PRODUCT_ID	LF_EMERALD_USB_PRODUCT_ID
#define LF_PRODUCT_ID_VOL	"LeapsterExplorer"
#define LF_PRODUCT_STRING	"Leapster Explorer"
#warning "Using Emerald's USB PID"

#else
#error "platform in not e2k, l2k, m2k, rio or vtk"
#define LF_USB_PRODUCT_ID	LF_EMERALD_USB_PRODUCT_ID
#define LF_PRODUCT_ID_VOL	"LeapsterExplorer"
#define LF_PRODUCT_STRING	"Leapster Explorer"
#endif


#endif /* U_BOOT */

#endif /* __LINUX_TO_UBOOT__H__ */

