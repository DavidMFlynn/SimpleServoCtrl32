;==========================================================================
;
;    Filename:	ServoF1847.inc
;    Date:	11/22/2014                                   
;    File Version:	1.0a1
;
;    Author:	David M. Flynn
;    Company:	Oxford V.U.E., Inc.
;    E-Mail:	dflynn@oxfordvue.com
;
;==========================================================================
;    Servo control module for 12F1847
;
;    History:
; 1.0d1    5/1/2015	Copied from ServoF1822.inc
; 1.0a1    11/22/2014	First Code, Copied from R/C Governor
;
;==========================================================================
; What happens next:
;
;
;==========================================================================
; Notes:
;
;====================================================================================================
;
;   Pin 1 (RA2/AN2) Address A2 (output)
;   Pin 2 (RA3/AN3) Enable Servos 0..7 (active low output)
;   Pin 3 (RA4/AN4) Enable Servos 8..15 (active low output)
;   Pin 4 (RA5/MCLR*) N.C.
;   Pin 5 (GND) Ground
;   Pin 6 (RB0) SW3/LED3 (Active Low Input/Output)
;   Pin 7 (RB1/AN11/SDA1) I2C Data
;   Pin 8 (RB2/AN10/RX) SW2/LED2 (Active Low Input/Output)
;   Pin 9 (RB3/CCP1) Pulse output for Servos 0..7
;
;   Pin 10 (RB4/AN8/SLC1) I2C Clock
;   Pin 11 (RB5/AN7)  SW1/LED1 (Active Low Input/Output)(System LED)
;   Pin 12 (RB6/AN5/CCP2) N.C.
;   Pin 13 (RB7/AN6) N.C.
;   Pin 14 (Vcc) +5 volts
;   Pin 15 (RA6) N.C.
;   Pin 16 (RA7/CCP2) Pulse output for Servos 0..7
;   Pin 17 (RA0) Address A0 (output)
;   Pin 18 (RA1) Address A1 (output)
;
;==========================================================================
;==========================================================================
;
;Constants for conditional assembly
;
	constant	oldcode=0
;
;
; 0.5uS res counter from 8MHz OSC
T1CON_Val	EQU	b'00000001'	;PreScale=1 32.7675mS
CCPCON_Clr	EQU	b'00001001'	;Clear output on match
CCPCON_Set	EQU	b'00001000'	;Set output on match
;
;==============================================================================================
;
;    Port A bits
;
;
;PortADDRBits	EQU	b'01100000'
;PortAValue	EQU	b'00011000'
;
;    Port B bits
;
#Define	CCP1_OutBit	PORTB,3	;Output Active High
#Define	CCP1_TrisBit	TRISB,3
#Define	CCP2_OutBit	PORTA,7	;Output Active High
#Define	CCP2_TrisBit	TRISA,7
;
;
;Constants
;
kMinPulseWidth	EQU	d'1800'	;900uS
kCenterPulseWidth	EQU	d'3000'	;1500uS
kMaxPulseWidth	EQU	d'4200'	;2100uS
kServoDwellTime	EQU	d'50000'	;2.5mS/Channel
;
;
;==============================================================================================
;***** VARIABLE DEFINITIONS
;
;
SigOutTime	EQU	0xF0	;2 bytes F0,F1
;
Flags	EQU	0xF2
; Flags bits
#Define	ServoOff	Flags,4
;
;
;==================================================================================
;
; Handle CCP1 Interupt Flag, Enter w/ bank 0 selected
;
IRQ_Servo1	BTFSS	PIR1,CCP1IF
	GOTO	IRQ_Servo1_End
;
	BTFSS	ServoOff	;Are we sending a pulse?
	GOTO	IRQ_Servo1_1	; Yes
;
;Oops, how did we get here???
	MOVLB	0x05
	CLRF	CCP1CON
	GOTO	IRQ_Servo1_X
;
IRQ_Servo1_1	MOVLB	0x05
	BTFSC	CCP1CON,CCP1M0	;Set output on match?
	GOTO	IRQ_Servo1_OL	; No
; An output just went high
;
	MOVF	SigOutTime,W	;Put the pulse into the CCP reg.
	ADDWF	CCPR1L,F
	MOVF	SigOutTime+1,W
	ADDWFC	CCPR1H,F
	MOVLW	CCP1CON_Clr	;Clear output on match
	MOVWF	CCP1CON	;CCP1 clr on match
	GOTO	IRQ_Servo1_X
;
; output went low so this cycle is done
IRQ_Servo1_OL	MOVLW	LOW kServoDwellTime
	ADDWF	CCPR1L,F
	MOVLW	HIGH kServoDwellTime
	ADDWFC	CCPR1H,F
;
	MOVLW	CCP1CON_Set	;Set output on match
	MOVWF	CCP1CON
;
IRQ_Servo1_X	MOVLB	0x00
	BCF	PIR1,CCP1IF
IRQ_Servo1_End
;--------------------------------------------------------------------
;
; Setup
	MOVLB	0x01
	BCF	CCP1_TrisBit	;Output active
	BSF	PIE1,CCP1IE
	MOVLB	0x00
;
;==============================================================================================
; Set CCP1 to go high is 0x100 clocks
;
StartServo	BTFSS	ServoOff
	RETURN
	BCF	ServoOff
;
	MOVLW	0x00	;start in 0x100 clocks
	MOVWF	TMR1L
	MOVLW	0xFF
	MOVWF	TMR1H
;
	MOVLB	0x05
	CLRF	CCPR1H
	CLRF	CCPR1L
	MOVLW	CCP1CON_Set
	MOVWF	CCP1CON	;go high on match
	RETURN
;
;=========================================================================================
;=========================================================================================
; Setup all data to usable values
;
DefaultMaxSpeed	EQU	0x02	;1uS/20mS^2
DefaultAccel	EQU	0x01
DefaultActive	EQU	0x00	;Off, no pulse
DefaultSFlags	EQU	b'00100010'	;In Position
;
ServoInit16	CLRW
	LOADFSR0W	ServoMaxSpeed0_7
;
	MOVLW	DefaultMaxSpeed
	CALL	Store16Times
;
	MOVLW	DefaultAccel
	CALL	Store16Times
;
	MOVLW	0x00
	CALL	Store16Times	;CurSpeed=stopped
;
	MOVLW	DefaultActive
	CALL	Store8Times
;
; Set Command position to center
	CLRW
	LOADFSR0W	CMDSigTime0_7
	MOVLW	LOW kCenterPulseWidth
	MOVWF	Param78
	MOVLW	HIGH kCenterPulseWidth
	MOVWF	Param79
	CALL	StoreWord16Times
; Set minimum time MinTime0_7,MinTime8_15
	MOVLW	LOW kMinPulseWidth
	MOVWF	Param78
	MOVLW	HIGH kMinPulseWidth
	MOVWF	Param79
	CALL	StoreWord16Times
; Set ServoFlags
	CLRW
	LOADFSR0W	ServoFlags
;
	MOVLW	DefaultSFlags
	CALL	Store8Times
;
; Set current position to center 
	CLRW
	LOADFSR0W	CMDSigTime0_7
; SigOutTime0_7
	MOVLW	LOW kCenterPulseWidth
	MOVWF	Param78
	MOVLW	HIGH kCenterPulseWidth
	MOVWF	Param79
	CALL	StoreWord8Times
; DwellTime0_7
	MOVLW	LOW kServoDwellTime
	MOVWF	Param78
	MOVLW	HIGH kServoDwellTime
	MOVWF	Param79
	CALL	StoreWord8Times
; SigOutTime8_15
	MOVLW	LOW kCenterPulseWidth
	MOVWF	Param78
	MOVLW	HIGH kCenterPulseWidth
	MOVWF	Param79
	CALL	StoreWord8Times
; DwellTime8_15
	MOVLW	LOW kServoDwellTime
	MOVWF	Param78
	MOVLW	HIGH kServoDwellTime
	MOVWF	Param79
	CALL	StoreWord8Times
;MaxTime0_7,MaxTime8_15
	CLRW
	LOADFSR0W	MaxTime0_7
	MOVLW	LOW kMaxPulseWidth
	MOVWF	Param78
	MOVLW	HIGH kMaxPulseWidth
	MOVWF	Param79
	CALL	StoreWord16Times
;
	RETURN
;
StoreWord8Times	MOVLW	.8
	GOTO	StoreWordXTimes
StoreWord16Times	MOVLW	.16
StoreWordXTimes	MOVWF	Param7A
StoreWord16Times_L1	MOVF	Param78,F
	MOVWI	FSR0++
	MOVF	Param79,F
	MOVWI	FSR0++
	DECFSZ	Param7A,F
	GOTO	StoreWord16Times_L1
	RETURN
;
Store16Times	MOVWF	Param78
	MOVLW	.16
StoreXTimes	MOVWF	Param79
	MOVF	Param78,W
Store16Times_L1	MOVWI	FSR0++
	DECFSZ	Param79,F
	GOTO	Store16Times_L1
	RETURN
;
Store8Times	MOVWF	Param78
	MOVLW	.8
	GOTO	StoreXTimes
;
;=========================================================================================
; ClampInt(Param7D:Param7C,kMinPulseWidth,kMaxPulseWidth)
;
; Entry: Param7D:Param7C
; Exit: Param7D:Param7C=ClampInt(Param7D:Param7C,kMinPulseWidth,kMaxPulseWidth)
;
ClampInt	MOVLW	high kMaxPulseWidth
	SUBWF	Param7D,W	;7D-kMaxPulseWidth
	SKPNB		;7D<Max?
	GOTO	ClampInt_1	; Yes
	SKPZ		;7D=Max?
	GOTO	ClampInt_tooHigh	; No, its greater.
	MOVLW	low kMaxPulseWidth	; Yes, MSB was equal check LSB
	SUBWF	Param7C,W	;7C-kMaxPulseWidth
	SKPNZ		;=kMaxPulseWidth
	RETURN		;Yes
	SKPB		;7C<Max?
	GOTO	ClampInt_tooHigh	; No
	RETURN		; Yes
;
ClampInt_1	MOVLW	high kMinPulseWidth
	SUBWF	Param7D,W	;7D-kMinPulseWidth
	SKPNB		;7D<Min?
	GOTO	ClampInt_tooLow	; Yes
	SKPZ		;=Min?
	RETURN		; No, 7D>kMinPulseWidth
	MOVLW	low kMinPulseWidth	; Yes, MSB is a match
	SUBWF	Param7C,W	;7C-kMinPulseWidth
	SKPB		;7C>=Min?
	RETURN		; Yes
;	
ClampInt_tooLow	MOVLW	low kMinPulseWidth
	MOVWF	Param7C
	MOVLW	high kMinPulseWidth
	MOVWF	Param7D
	RETURN
;
ClampInt_tooHigh	MOVLW	low kMaxPulseWidth
	MOVWF	Param7C
	MOVLW	high kMaxPulseWidth
	MOVWF	Param7D
	RETURN
;
;=========================================================================================
;=========================================================================================
;
;
	movlw	PortADDRBits
	movwf	TRISA
;
;
	CLRWDT
;
	MOVLW	LEDTIME
	MOVWF	LEDRate
;
	MOVLW	low d'1500'
	MOVWF	SigOutTime
	MOVLW	high d'1500'
	MOVWF	SigOutTime+1
;
;
	bsf	_RP0	; select bank 1
	bsf	PIE1,CCP1IE
;
;
;
;
;==========================================================================
;  Main Loop
;
;==========================================================================
;
MainLoop	CLRWDT
;
;
;
;

	CALL	StartServo
	GOTO	MainLoop	
;
;=====================================================================================
;=====================================================================================
;
MoveTo78	MOVWF	FSR
	MOVF	INDF,W
	MOVWF	Param78
	INCF	FSR,F
	MOVF	INDF,W
	MOVWF	Param79
	RETURN
;
;=====================================================================================
;
MoveTo7C	MOVWF	FSR
	MOVF	INDF,W
	MOVWF	Param7C
	INCF	FSR,F
	MOVF	INDF,W
	MOVWF	Param7D
	RETURN
;
;=====================================================================================
;
Move78To7C	MOVF	Param78,W
	MOVWF	Param7C
	MOVF	Param79,W
	MOVWF	Param7D
	RETURN
;
;=====================================================================================
;
MoveFrom7C	MOVWF	FSR
	MOVF	Param7C,W
	MOVWF	INDF
	INCF	FSR,F
	MOVF	Param7D,W
	MOVWF	INDF
	RETURN
;
;=====================================================================================
; Less or Equal
;
; Entry: Param7D:Param7C, Param79:Param78
; Exit: Param77:0=Param7D:Param7C<=Param79:Param78
;
Param7D_LE_Param79	CLRF	Param77	;default to >
	MOVF	Param79,W
	SUBWF	Param7D,W	;Param7D-Param79
	SKPNB		;Param7D<Param79?
	GOTO	SetTrue	; Yes
	SKPZ		;Param7D>Param79?
	RETURN		; Yes
	MOVF	Param78,W	; No, MSB is a match
	SUBWF	Param7C,W	;Param7C-Param78
	SKPNB		;Param7C<Param78?
	GOTO	SetTrue	; Yes
	SKPZ		;LSBs then same?
	RETURN		; No
;
SetTrue	BSF	Param77,0
	RETURN
;
;=====================================================================================
; Greater or Equal
;
; Entry: Param7D:Param7C, Param79:Param78
; Exit: Param77:0=Param7D:Param7C>=Param79:Param78
;
Param7D_GE_Param79	CLRF	Param77	;default to <
	MOVF	Param79,W
	SUBWF	Param7D,W	;Param7D-Param79
	SKPNB		;Param7D<Param79?
	RETURN		; Yes
	SKPZ		;Param7D>Param79?
	GOTO	SetTrue	; Yes
Param7D_GE_Param79_1	MOVF	Param78,W	; No, MSB is a match
	SUBWF	Param7C,W	;Param7C-Param78
	SKPNB		;Param7C<Param78?
	RETURN		; Yes
	GOTO	SetTrue	; No
;
;======================================================================================
;
EqualMin	CLRF	Param77
	MOVLW	high kMinPulseWidth
	SUBWF	Param7D,W
	SKPZ
	RETURN
	MOVLW	low kMinPulseWidth
	SUBWF	Param7C,W
	SKPNZ
	BSF	Param77,0
	RETURN
	
;
Subtract1000	MOVLW	low kMinPulseWidth
	SUBWF	Param7C,F
	SUBBF	Param7D,F
	MOVLW	high kMinPulseWidth
	SUBWF	Param7D,F
	RETURN
;
Subtract1500	MOVLW	low d'1500'
	SUBWF	Param7C,F
	SUBBF	Param7D,F
	MOVLW	high d'1500'
	SUBWF	Param7D,F
	RETURN
;
X2	CLRC
	RLF	Param7C,F
	RLF	Param7D,F
	RETURN
;
Add1000	MOVLW	low kMinPulseWidth
	ADDWF	Param7C,F
	ADDCF	Param7D,F
	MOVLW	high kMinPulseWidth
	ADDWF	Param7D,F
	RETURN
;
;=============================================================================================

