#
# (C) Copyright 2010
# jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
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

#########################################################################
#	Build options
#########################################################################
NX_CFLAGS += -I$(TOPDIR)/arch/arm/cpu/$(CPU)/$(SOC)/prototype/module  \
			 -I$(TOPDIR)/arch/arm/cpu/$(CPU)/$(SOC)/prototype/nx_base \
			 -I$(TOPDIR)/arch/arm/include/asm/arch-$(SOC)

ifeq ($(CONFIG_PROTO_FUNC_DEBUG),y)
NX_CFLAGS += -D__LINUX__ -DNX_DEBUG
else
NX_CFLAGS += -D__LINUX__ -DNX_RELEASE
endif

#########################################################################
#	Build options for platform CC
#########################################################################
PLATFORM_RELFLAGS += $(NX_CFLAGS) -fno-strict-aliasing

#########################################################################
#	Build options for HOSTCC
#########################################################################
HOSTCFLAGS += $(NX_CFLAGS)