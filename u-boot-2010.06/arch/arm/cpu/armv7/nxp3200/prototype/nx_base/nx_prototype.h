//------------------------------------------------------------------------------
//
//	Copyright (C) 2009 Nexell Co., All Rights Reserved
//	Nexell Co. Proprietary & Confidential
//
//	NEXELL INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
//	AND	WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
//	BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
//	FOR A PARTICULAR PURPOSE.
//
//	Module		: nx_base 
//	File		: nx_prototype.h
//	Description	: include header files for nx_base.
//	Author		: Goofy
//	Export		: 
//	History		: 
//		2010.10.05	Hans	add gloval io access function
//		2010.04.27	Hans
//		2007.04.04	Goofy	First draft
//------------------------------------------------------------------------------
#ifndef __NX_PROTOTYPE_H__
#define __NX_PROTOTYPE_H__

#include "nx_type.h"
#include "nx_debug.h"
#include "nx_chip.h"
#include "nx_clockcontrol.h"

#ifdef __cplusplus
extern "C" {
#endif
#define ReadIODW(Addr) (*(volatile unsigned int*)Addr)
#define ReadIOW(Addr) (*(volatile unsigned short*)Addr)
#define ReadIOB(Addr) (*(volatile unsigned char*)Addr)
#define WriteIODW(Addr,Data) (*(volatile unsigned int*)Addr)=((unsigned int)Data)
#define WriteIOW(Addr,Data) (*(volatile unsigned short*)Addr)=((unsigned short)Data)
#define WriteIOB(Addr,Data) (*(volatile unsigned char*)Addr)=((unsigned char)Data)
#ifdef __cplusplus
}
#endif

#endif	// __NX_PROTOTYPE_H__
