#
# Scott Esters <sesters@leapfrog.com>
# (C) Copyright 2006 LeapFrog Enterprises
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

include $(TOPDIR)/config.mk

ifneq ($(OBJTREE),$(SRCTREE))
$(shell mkdir -p $(obj)board/$(VENDOR)/common)
endif

LIB	= $(obj)lib$(VENDOR).a $(obj)lib$(BOARD).a $(obj)lib$(SOC).a
#LIB	= $(obj)lib$(VENDOR).a
#LIB	= $(obj)lib$(BOARD).a

COBJS-y := board.o logo.o

COBJS-$(CONFIG_CMD_NAND)	+= nand.o
COBJS-$(CONFIG_CMD_NAND_MLC)	+= cmd_mlcnand.o
COBJS-$(CONFIG_LCD)		+= lcd.o
COBJS-$(CONFIG_SOC_LFP100)	+= lfp100.o
COBJS-$(CONFIG_SYS_NAND_HW_ECC)	+= nand_hw_ecc.o nand_ecc.o



SRCS    := $(SOBJS-y:.o=.S) $(COBJS-y:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS-y) $(SOBJS-y))
#SOBJS	:= $(addprefix $(obj),$(SOBJS-y))

$(LIB):	$(obj).depend $(OBJS)
	$(AR) $(ARFLAGS) $@ $(OBJS)

clean:
	rm -f $(SOBJS) $(OBJS)

distclean:	clean
	rm -f $(LIB) core *.bak $(obj).depend

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
