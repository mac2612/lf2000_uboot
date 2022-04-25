;-------------------------------------------------------------------------------
;
;	Copyright (C) 2009 Nexell Co., All Rights Reserved
;	Nexell Co. Proprietary & Confidential
;
;	NEXELL INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
;	AND WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
;	BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
;	FOR A PARTICULAR PURPOSE.
;
;	Module		: Bug turn around code
;	File		: TurnArround.s
;	Description	: ARM assembly functions
;	Author		: Hans
;	History		:
;		2010-10-05	Hans	create
;-------------------------------------------------------------------------------

		INCLUDE kxarm.h

;-------------------------------------------------------------------------------
;	current program status register mode definition
;-------------------------------------------------------------------------------
I_Bit			EQU		(0x80)			; when I bit is set, IRQ is disabled
ST_MEM_OFFS		EQU		(0xBC000000)	; SRAM Phys = 0x18000000, Virt = 0xBC000000

	TEXTAREA

;-------------------------------------------------------------------------------
;	void	WriteIODW( Addr, Data )
;-------------------------------------------------------------------------------

	LEAF_ENTRY	WriteIODW

	;R0:IOA, R1:IOD, R2:UCA, R3:CPSR, R4:TMP
	STMFD 	SP!, {R2-R4}

	MOV		R2, #ST_MEM_OFFS;

	MRS		R3, CPSR			; read interrupt enable status
	ORR		R4, R3, #I_Bit		; clear interrupt enable bit
	MSR		CPSR_cxsf, R4		; save and interrupt disable

	LDRB	R4, [R2]			; before memory bus read
	STRB	R4, [R2]			; before memory bus write

	STR		R1, [R0]			; write io data

	LDRB	R4, [R2]			; after memory bus read

	MSR		CPSR_cxsf, R3		; restore CPSR

	LDMFD 	SP!, {R2-R4}

	BX		LR

	ENTRY_END

;-------------------------------------------------------------------------------
;	void	WriteIOW( Addr, Data )
;-------------------------------------------------------------------------------

	LEAF_ENTRY	WriteIOW

	;R0:IOA, R1:IOD, R2:UCA, R3:CPSR, R4:TMP
	STMFD 	SP!, {R2-R4}

	MOV		R2, #ST_MEM_OFFS;

	MRS		R3, CPSR			; read interrupt enable status
	ORR		R4, R3, #I_Bit		; clear interrupt enable bit
	MSR		CPSR_cxsf, R4		; save and interrupt disable

	LDRB	R4, [R2]			; before memory bus read
	STRB	R4, [R2]			; before memory bus write

	STRH	R1, [R0]			; write io data

	LDRB	R4, [R2]			; after memory bus read

	MSR		CPSR_cxsf, R3		; restore CPSR

	LDMFD 	SP!, {R2-R4}

	BX		LR

	ENTRY_END

;-------------------------------------------------------------------------------
;	void	WriteIOB( Addr, Data )
;-------------------------------------------------------------------------------

	LEAF_ENTRY	WriteIOB

	;R0:IOA, R1:IOD, R2:UCA, R3:CPSR, R4:TMP
	STMFD 	SP!, {R2-R4}

	MOV		R2, #ST_MEM_OFFS;

	MRS		R3, CPSR			; read interrupt enable status
	ORR		R4, R3, #I_Bit		; clear interrupt enable bit
	MSR		CPSR_cxsf, R4		; save and interrupt disable

	LDRB	R4, [R2]			; before memory bus read
	STRB	R4, [R2]			; before memory bus write

	STRB	R1, [R0]			; write io data

	LDRB	R4, [R2]			; after memory bus read

	MSR		CPSR_cxsf, R3		; restore CPSR

	LDMFD 	SP!, {R2-R4}

	BX		LR

	ENTRY_END

;-------------------------------------------------------------------------------
;	U32		ReadIODW( Addr )
;-------------------------------------------------------------------------------
	LEAF_ENTRY ReadIODW

	LDR		R0, [R0]
	BX		LR

	ENTRY_END

;-------------------------------------------------------------------------------
;	U16		ReadIOW( Addr )
;-------------------------------------------------------------------------------
	LEAF_ENTRY ReadIOW

	LDRH	R0, [R0]
	BX		LR

	ENTRY_END

;-------------------------------------------------------------------------------
;	U8		ReadIOB( Addr )
;-------------------------------------------------------------------------------
	LEAF_ENTRY ReadIOB

	LDRB	R0, [R0]
	BX		LR

	ENTRY_END
;-------------------------------------------------------------------------------
	END
