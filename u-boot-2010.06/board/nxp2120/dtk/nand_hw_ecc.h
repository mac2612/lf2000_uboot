//------------------------------------------------------------------------------
//
//  Copyright (C) 2009 Nexell Co., All Rights Reserved
//  Nexell Co. Proprietary & Confidential
//
//	MAGICEYES INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
//  AND WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
//  FOR A PARTICULAR PURPOSE.
//
//	Module     :
//	File       :
//	Description:
//	Author     : Goofy
//	History    :
//------------------------------------------------------------------------------
#ifndef __NAND_HW_ECC_H__
#define __NAND_HW_ECC_H__

void 	NX_NAND_CreateLookupTable(void);
void	NX_NAND_WaitForDecoding(void);
void	NX_NAND_WaitForEncoding(void);
int		NX_NAND_GetErrorStatus(void);
int		NX_NAND_GetErrorLocation(int *pOddSyn, int *pLocation, int *ErrCnt);
void	NX_NAND_SetResetECC(int EccMode);
void 	NX_NAND_GetGenECC(unsigned int *pECC, int EccMode);
void 	NX_NAND_SetOriECC(unsigned int *pECC, int EccMode);
void 	NX_NAND_GetOddSyndrome(int *pSyndrome);

#endif /* __NAND_HW_ECC_H__ */