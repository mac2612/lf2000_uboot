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
#if (1)
#include <common.h>
#include <asm/errno.h>
#include <nand.h>
#include <platform.h>
#else
#include <linux/kernel.h>
#include <mach/platform.h>
#endif

#if	(1)
#define DBGOUT(msg...)		{ printk(KERN_INFO "ecc_func: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define ERROUT(msg...)		{ 					\
		printk(KERN_ERR "ERROR: %s, %s line %d: \n",		\
			__FILE__, __FUNCTION__, __LINE__),	\
		printk(KERN_ERR msg); }


//------------------------------------------------------------------------------
// NAND CTRL REGISGER:
//------------------------------------------------------------------------------
#define BASEADDR_NFREG			IO_ADDRESS(PHY_BASEADDR_NAND_MODULE)
#define BASEADDR_NFCTRL			(BASEADDR_NFREG)			// 0xC001587C
#define BASEADDR_NFECC			(BASEADDR_NFREG + 0x04)		// 0xC0015880
#define BASEADDR_NFORGECC		(BASEADDR_NFREG + 0x20)		// 0xC001589C
#define BASEADDR_NFECCSTATUS	(BASEADDR_NFREG + 0x40)		// 0xC00158BC
#define BASEADDR_NFSYNDROME		(BASEADDR_NFREG + 0x44)		// 0xC00158C0

#define REG_NFCTRL				(*(volatile unsigned int *)(BASEADDR_NFREG))
#define REG_NFECCSTATUS			(*(volatile unsigned int *)(BASEADDR_NFECCSTATUS))

#define NX_NFECCSTATUS_ERROR	(1U<< 2)
#define NX_NFECCSTATUS_DECDONE	(1U<< 1)
#define NX_NFECCSTATUS_ENCDONE	(1U<< 0)
#define NX_NFCTRL_ECCRST		(1U<<11)

static char BASEADDR_POLY[64*1024] = { 0, };
//------------------------------------------------------------------------------
// BCH variables:
//------------------------------------------------------------------------------
//	k : number of information
//	m : dimension of Galois field.
//	t : number of error that can be corrected.
//	n : length of codeword = 2^m - 1
//	r : number of parity bit = m * t
//------------------------------------------------------------------------------
#define NX_BCH_VAR_K		(512 * 8)
#define NX_BCH_VAR_M		(13)
#define NX_BCH_VAR_T		(CFG_NAND_ECC_MODE)		// 4 or 8 or 16

#define NX_BCH_VAR_N		(((1<<NX_BCH_VAR_M)-1))
#define NX_BCH_VAR_R		(NX_BCH_VAR_M * NX_BCH_VAR_T)

#define NX_BCH_VAR_TMAX		(16)
#define NX_BCH_VAR_RMAX		(NX_BCH_VAR_M * NX_BCH_VAR_TMAX)

#define NX_BCH_VAR_R32		((NX_BCH_VAR_R   +31)/32)
#define NX_BCH_VAR_RMAX32	((NX_BCH_VAR_RMAX+31)/32)


typedef struct tag_POLYNOMIALS
{
	short BCH_AlphaToTable[8192];
	short BCH_IndexOfTable[8192];
	int elp[(NX_BCH_VAR_TMAX*2)+1][(NX_BCH_VAR_TMAX*2)*2]; 	// Error locator polynomial (ELP)
	int B  [(NX_BCH_VAR_TMAX*2)+1][(NX_BCH_VAR_TMAX*2)*2];	// Scratch polynomial
} POLYNOMIALS;


//------------------------------------------------------------------------------
// Generate GF(2**NX_BCH_VAR_M) from the primitive polynomial p(X) in p[0]..p[NX_BCH_VAR_M]
// The lookup table looks like:
// index -> polynomial form:   pAlphaTo[ ] contains j = alpha**i;
// polynomial form -> index form:  pIndexOf[j = alpha**i] = i
// pAlphaTo[1] = 2 is the primitive element of GF(2**NX_BCH_VAR_M)
//------------------------------------------------------------------------------
void NX_NAND_CreateLookupTable(void)
{
	int i;
	int mask;	// Register states
	unsigned int p = 0x25AF;	// Primitive polynomials

	POLYNOMIALS *pPoly = (POLYNOMIALS *)BASEADDR_POLY;
	short *   pAlphaTo = &(pPoly->BCH_AlphaToTable[0]);
	short *   pIndexOf = &(pPoly->BCH_IndexOfTable[0]);

	// Galois field implementation with shift registers
	// Ref: L&C, Chapter 6.7, pp. 217
	mask = 1;
	pAlphaTo[NX_BCH_VAR_M] = 0;
	for ( i=0 ; i<NX_BCH_VAR_M ; i++ )	{
		pAlphaTo[ i ] = mask;
		pIndexOf[ pAlphaTo[i] ] = i;

		if ( p & (1U<<i) )
			pAlphaTo[NX_BCH_VAR_M] ^= mask;

		mask <<= 1 ;
	}

	pIndexOf[ pAlphaTo[NX_BCH_VAR_M] ] = NX_BCH_VAR_M;
	mask >>= 1;
	for ( i=NX_BCH_VAR_M+1 ; i<NX_BCH_VAR_N ; i++ ) {
		if ( pAlphaTo[i-1] >= mask )
			pAlphaTo[i] = pAlphaTo[NX_BCH_VAR_M] ^ ((pAlphaTo[i-1] ^ mask) << 1);
		else
			pAlphaTo[i] = pAlphaTo[i-1] << 1;

		pIndexOf[pAlphaTo[i]] = i;
	}
	pIndexOf[0] = -1;
}

//------------------------------------------------------------------------------
#define NX_BCH_AlphaTo( _i_ )		((int)(pPoly->BCH_AlphaToTable[ (_i_) ]))
#define NX_BCH_IndexOf( _i_ )		((int)(pPoly->BCH_IndexOfTable[ (_i_) ]))

static int	NX_BCH_MODULAR(int index)
{
	register int modular = NX_BCH_VAR_N;

	while ( index >= modular )
		index -= modular;

	return index;
}

//------------------------------------------------------------------------------
void NX_NAND_SetResetECC(int EccMode)
{
	const U32 BIT_SIZE	= 2;
	const U32 BIT_POS	= 28;
	const U32 BIT_MASK	= ((1<<BIT_SIZE)-1) << BIT_POS;

	register U32 regval;

	EccMode /= 8;	// NFECCMODE[1:0] = 0(4), 1(8), 2(16)

	regval  = REG_NFCTRL;
	regval &= ~(BIT_MASK);	// Unmask bits.
	regval |= (EccMode << BIT_POS);

	// Reset H/W BCH decoder.
	REG_NFCTRL = regval | NX_NFCTRL_ECCRST;
}

//------------------------------------------------------------------------------
void NX_NAND_WaitForDecoding(void)
{
	while (0==(REG_NFECCSTATUS & NX_NFECCSTATUS_DECDONE)) { ; }
}

//------------------------------------------------------------------------------
void NX_NAND_WaitForEncoding(void)
{
	while (0==(REG_NFECCSTATUS & NX_NFECCSTATUS_ENCDONE)) { ; }
}

//------------------------------------------------------------------------------
int	NX_NAND_GetErrorStatus(void)
{
	if (REG_NFECCSTATUS & NX_NFECCSTATUS_ERROR)
		return 1;
	return 0;
}
//------------------------------------------------------------------------------
int	NX_NAND_GetErrorLocation(int *pOddSyn, int *pLocation, int *ErrCnt)
{
	register int i, j, elp_sum ;
	int count;
	int r;				// Iteration steps
	int Delta; 			// Discrepancy value

//	int elp[(NX_BCH_VAR_TMAX*2)+1][(NX_BCH_VAR_TMAX*2)+2]; 	// Error locator polynomial (ELP)
//	int B  [(NX_BCH_VAR_TMAX*2)+1][(NX_BCH_VAR_TMAX*2)+2];	// Scratch polynomial
	POLYNOMIALS *pPoly = (POLYNOMIALS *)BASEADDR_POLY;

	int L[(NX_BCH_VAR_TMAX*2)+1];		// Degree of ELP
	int reg[(NX_BCH_VAR_TMAX*1)+1];	// Register state
	int	s[(NX_BCH_VAR_TMAX*2)];


	for( i=0 ; i<NX_BCH_VAR_T ; i++ )
		s[i*2] = pOddSyn[i];

	// Even syndrome = (Odd syndrome) ** 2
	for( i=1,j=0 ; i<(NX_BCH_VAR_T*2) ; i+=2, j++ )
	{
		if( s[j] == 0 )		s[i] = 0;
		else				s[i] = NX_BCH_AlphaTo( NX_BCH_MODULAR( 2 * NX_BCH_IndexOf(s[j]) ) );
	}

	// Initialization of pPoly->elp, pPoly->B and register
	for( i=0 ; i<=(NX_BCH_VAR_T*2) ; i++ )
	{
		L[i] = 0 ;
		for( j=0 ; j<=(NX_BCH_VAR_T*2) ; j++ )
		{
			pPoly->elp[i][j] = 0 ;
			pPoly->B[i][j] = 0 ;
		}
	}

	for( i=0 ; i<=NX_BCH_VAR_T ; i++ )
	{
		reg[i] = 0 ;
	}

	pPoly->elp[1][0] = 1 ;
	pPoly->elp[1][1] = s[0] ;

	L[1] = 1 ;
	if( s[0] != 0 )
		pPoly->B[1][0] = NX_BCH_AlphaTo( NX_BCH_MODULAR( NX_BCH_VAR_N - NX_BCH_IndexOf(s[0]) ) );
	else
		pPoly->B[1][0] = 0;

	for( r=3 ; r<=(NX_BCH_VAR_T*2)-1 ; r=r+2 )
	{
		// Compute discrepancy
		Delta = s[r-1] ;
		for( i=1 ; i<=L[r-2] ; i++ )
		{
			if( (s[r-i-1] != 0) && (pPoly->elp[r-2][i] != 0) )
				Delta ^= NX_BCH_AlphaTo( NX_BCH_MODULAR( NX_BCH_IndexOf(s[r-i-1]) + NX_BCH_IndexOf(pPoly->elp[r-2][i]) ) );
		}

		if( Delta == 0 )
		{
			L[r] = L[r-2] ;
			for( i=0 ; i<=L[r-2] ; i++ )
			{
				pPoly->elp[r][i] = pPoly->elp[r-2][i];
				pPoly->B[r][i+2] = pPoly->B[r-2][i] ;
			}
		}
		else
		{
			// Form new error locator polynomial
			for( i=0 ; i<=L[r-2] ; i++ )
			{
				pPoly->elp[r][i] = pPoly->elp[r-2][i] ;
			}

			for( i=0 ; i<=L[r-2] ; i++ )
			{
				if( pPoly->B[r-2][i] != 0 )
					pPoly->elp[r][i+2] ^= NX_BCH_AlphaTo( NX_BCH_MODULAR( NX_BCH_IndexOf(Delta) + NX_BCH_IndexOf(pPoly->B[r-2][i]) ) );
			}

			// Form new scratch polynomial and register length
			if( 2 * L[r-2] >= r )
			{
				L[r] = L[r-2] ;
				for( i=0 ; i<=L[r-2] ; i++ )
				{
					pPoly->B[r][i+2] = pPoly->B[r-2][i];
				}
			}
			else
			{
				L[r] = r - L[r-2];
				for( i=0 ; i<=L[r-2] ; i++ )
				{
					if( pPoly->elp[r-2][i] != 0 )
						pPoly->B[r][i] = NX_BCH_AlphaTo( NX_BCH_MODULAR( NX_BCH_IndexOf(pPoly->elp[r-2][i]) + NX_BCH_VAR_N - NX_BCH_IndexOf(Delta) ) );
					else
						pPoly->B[r][i] = 0;
				}
			}
		}
	}

	if( L[(NX_BCH_VAR_T*2)-1] > NX_BCH_VAR_T )
	{
		if(ErrCnt)
			*ErrCnt = L[(NX_BCH_VAR_T*2)-1];
		return -1;
	}
	else
	{
		// Chien's search to find roots of the error location polynomial
		// Ref: L&C pp.216, Fig.6.1
		for( i=1 ; i<=L[(NX_BCH_VAR_T*2)-1] ; i++ )
			reg[i] = pPoly->elp[(NX_BCH_VAR_T*2)-1][i];

		count = 0;
		for( i=1 ; i<=NX_BCH_VAR_N ; i++ )
		{
			elp_sum = 1;
			for( j=1 ; j<=L[(NX_BCH_VAR_T*2)-1] ; j++ )
			{
				if( reg[j] != 0 )
				{
					reg[j] = NX_BCH_AlphaTo( NX_BCH_MODULAR( NX_BCH_IndexOf(reg[j]) + j ) );
					elp_sum ^= reg[j] ;
				}
			}

			if( !elp_sum )		// store root and error location number indices
			{
				// Convert error location from systematic form to storage form
				pLocation[count] = NX_BCH_VAR_N - i;

				if (pLocation[count] >= NX_BCH_VAR_R)
				{
					// Data Bit Error
					pLocation[count] = pLocation[count] - NX_BCH_VAR_R;
					pLocation[count] = (NX_BCH_VAR_K-1) - pLocation[count];
				}
				else
				{
					// ECC Error
					pLocation[count] = pLocation[count] + NX_BCH_VAR_K;
				}

				if( pLocation[count] < 0 ) {
					if(ErrCnt)
						*ErrCnt = L[(NX_BCH_VAR_T*2)-1];
					return -1;
				}
				//if( pLocation[count] >= 0 )
				count++;
			}
		}


		if( count == L[(NX_BCH_VAR_T*2)-1] )	// Number of roots = degree of pPoly->elp hence <= NX_BCH_VAR_T errors
		{
			return count;
		}
		else	// Number of roots != degree of ELP => >NX_BCH_VAR_T errors and cannot solve
		{
			if(ErrCnt)
				*ErrCnt = L[(NX_BCH_VAR_T*2)-1];
			return -1;
		}

		/*
		if( count != L[(NX_BCH_VAR_T*2)-1] )
		{
			NX_DEBUG_MSG( "\n\n\t\t ERROR -> count = " );
			NX_DEBUG_DEC( count );
			NX_DEBUG_MSG( ", L = " );
			NX_DEBUG_DEC( L[(NX_BCH_VAR_T*2)-1] );
			NX_DEBUG_MSG( "\n\n" );

			if( count < 4 )		return -1;
		}

		return count;
		*/
	}
}

//------------------------------------------------------------------------------
void NX_NAND_GetGenECC(unsigned int *pECC, int EccMode)
{
	int i, num;
	volatile U32 *pRegECC = (volatile U32 *)BASEADDR_NFECC;

	switch (EccMode) {
	case  4: num = 2;	break;
	case  8: num = 4;	break;
	case 16: num = 7;	break;
	case 24:
	default:
		ERROUT("not support ECC %d bit\n", EccMode);
		return;
	}

	for (i=0 ; i<num ; i++)
		*pECC++ = *pRegECC++;
}

void NX_NAND_SetOriECC(unsigned int *pECC, int EccMode)
{
	int i, num;
	volatile U32 *pRegOrgECC = (volatile U32 *)BASEADDR_NFORGECC;

	switch (EccMode) {
	case  4: num = 2;	break;
	case  8: num = 4;	break;
	case 16: num = 7;	break;
	case 24:
	default:
		ERROUT("not support ECC %d bit\n", EccMode);
		return;
	}

	for (i=0 ; num > i; i++)
		*pRegOrgECC++ = *pECC++;
}


//------------------------------------------------------------------------------
void NX_NAND_GetOddSyndrome(int *pSyndrome)
{
	const U32 BIT_SIZE	= 13;
	const U32 BIT_POS	= 13;
	const U32 BIT_MASK	= ((1UL<<BIT_SIZE)-1);

	register volatile U32 *pReg;
	register U32 regval;
	int i;

	NX_ASSERT( CNULL != pSyndrome );

	pReg = (volatile U32 *)BASEADDR_NFSYNDROME;

	for ( i=0 ; i<(NX_BCH_VAR_T/2) ; i++ ) {
		regval = *pReg++;
		*pSyndrome++ = (int)(regval & BIT_MASK);		// Syndrome <= NFSYNDROME[i][12: 0]
		*pSyndrome++ = (int)(regval >> BIT_POS);		// Syndrome <= NFSYNDROME[i][25:13]
	}
}

