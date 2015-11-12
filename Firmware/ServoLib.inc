;====================================================================================================
; PIC16F1847 on SimpleServo16 PCB
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
;   Pin 16 (RA7/CCP2) Pulse output for Servos 8..15
;   Pin 17 (RA0) Address A0 (output)
;   Pin 18 (RA1) Address A1 (output)
;
;=========================================================================================
; include ServoLib.h
;=========================================================================================
; Constants:
;
;DefaultMaxSpeed	EQU	0x30	;1uS/20mS^2
;DefaultAccel	EQU	0x01
;DefaultSFlags	EQU	b'00100010'	;In Position
;
;CCPCON_Clr	EQU	b'00001001'	;Clear output on match
;CCPCON_Set	EQU	b'00001000'	;Set output on match
;CCPCON_Int	EQU	b'00001010'	;Interupt only on match
;
;kCenterPulseWidth	EQU	d'3000'	;1500uS
;kMinPulseWidth	EQU	d'1800'	;900uS
;kMaxPulseWidth	EQU	d'4200'	;2100uS
;kServoDwellTimeA	EQU	d'4500'	;Address Change time 2250uS
;kServoAddrTime	EQU	d'500'	;250uS
;kServoDwellTime	EQU	d'5000'	;2.5mS/Channel
;
;================================================================================================
;  Bank3 Ram 1A0h-1EFh 80 Bytes
;
;Bank3_Vars	udata	0x1A0
;ServoMaxSpeed0_7	res	.8	;0=no Accel, 1..255 counts/20mS
;ServoMaxSpeed8_15	res	.8
;ServoAccelValue0_7	res	.8	;1..8 counts/20mS squared
;ServoAccelValue8_15	res	.8
;ServoCurSpeed0_7	res	.8	;0=Stopped, MSb=Direction, 1..127
;ServoCurSpeed8_15	res	.8
;
;================================================================================================
;  Bank4 Ram 220h-26Fh 80 Bytes
;
;Bank4_Vars	udata	0x220
;CMDServoIDX	res	1	;0..15, used by IdleServos
;CMDSigTime0_7	res	.16	;Commanded position
;CMDSigTime8_15	res	.16
;MinTime0_7	res	.16	;Minimum pulse time (900uS=1800)
;MinTime8_15	res	.16	;Minimum pulse time (900uS=1800)
;
;================================================================================================
;  Bank5 Ram 2A0h-2EFh 80 Bytes
;
;Bank5_Vars	udata	0x2A0
;ServoIDX	res	1	;Index 0..7
;ServoCtlFlags	res	1
; Flag bits, ToDo at Next ISR:
;CyclePulseStart	EQU	0	;Start cycle banks 1 and 2
;CyclePulseEnd1	EQU	2	;End pulse, begin dwell
;CyclePulseEnd2	EQU	3	; Set when a pulse is started.
;AddrChngDwell	EQU	4	;dwell 100uS , change address
;
;ServoFlags	res	.8	;4 bits per servo
; Flag bits
;ValueSentFlag0_7	EQU	0
;InPositionFlag0_7	EQU	1
;ServoOnBit0_7	EQU	2
;MovingFWD0_7	EQU	3
;ValueSentFlag8_15	EQU	4
;InPositionFlag8_15	EQU	5
;ServoOnBit8_15	EQU	6
;MovingFWD8_15	EQU	7
;
;CalcdDwell	res	1	;scratch var
;CalcdDwellH	res	1
;SigOutTime0_7	res	.16	;Current position
;SigOutTime8_15	res	.16
;DwellTime0_7	res	.16	;Next dwell time
;DwellTime8_15	res	.16
;
;================================================================================================
;  Bank6 Ram 320h-26Fh 80 Bytes
;
;Bank6_Vars	udata	0x320
;ServoFlags2	res	.8	;4 bits per servo
; Flag bits
;AccelComplete0_7	EQU	0
;AccelComplete8_15	EQU	4
;MaxTime0_7	res	.16	;Maximum pulse time (2100uS=4200)
;MaxTime8_15	res	.16	;Maximum pulse time (2100uS=4200)
;AccelRampLen0_7	res	.16
;AccelRampLen8_15	res	.16
;
;=========================================================================================
; Init Code:
;
;	CALL	ServoInit16
;	CALL	StartServos
;
;=========================================================================================
; ISR Code:
;
;	MOVLB	0	;Bank0
;	BTFSC	PIR1,CCP1IF
;	CALL	ISR_ServoCCP1
;
;	BTFSC	PIR2,CCP2IF
;	CALL	ISR_Servo2
;
;=========================================================================================
; Main Loop Code:
;
;	CALL	IdleServos
;	CALL	TestServoLib	;Optional Test Routine
;
;=========================================================================================
; Routines:
;
; ServoInit16	Call once to init vars
; StartServos	Call once to start servo system
;
; StartMotion	Call to move a servo,  W=Servo# 0..15, Param7D:Param7C = Dest
; IdleServos	Call often from main loop
;
; TestServoLib	Call from MainLoop to test, "this is only a test"
;
; ISR's
;  ISR_ServoCCP1
;  ISR_ServoCCP2
;
;=========================================================================================
; Interupt Service Routine for CCP1
;
ISR_ServoCCP1	MOVLB	0x05	; bank 5
;
	BTFSC	ServoCtlFlags,AddrChngDwell	;Is it time to change the address?
	GOTO	IRQ_Servo_Next
;
	BTFSC	ServoCtlFlags,CyclePulseStart	;Is it time to start a new pulse/dwell?
	GOTO	IRQ_Servo1_PulseOrDwell
;
	BTFSC	ServoCtlFlags,CyclePulseEnd1	;Is the current pulse ending?
	GOTO	IRQ_Servo1_OL
;
;	MOVF	ServoCtlFlags,F
;	SKPZ
	GOTO	IRQ_Servo1_X	;tc
; if not set to do anything fall thru and start a cycle
IRQ_Servo1_PulseOrDwell:
	BCF	ServoCtlFlags,CyclePulseStart
; FSR1 = ServoFlags+ServoIDX
	MOVF	ServoIDX,W
	ADDLW	LOW ServoFlags
	MOVWF	FSR1L
	MOVLW	HIGH ServoFlags
	MOVWF	FSR1H
;
	BTFSS	INDF1,ServoOnBit0_7	;Is this servo active?
	GOTO	IRQ_Servo1_InAct	; No
; FSR0 = SigOutTime0_7 + ServoIDX * 2			
	LSLF	ServoIDX,W	;Servo Idx*2
	ADDLW	LOW SigOutTime0_7
	MOVWF	FSR0L
	MOVLW	HIGH SigOutTime0_7
	MOVWF	FSR0H
; Set time to end pulse
	MOVIW	FSR0++	;Put the pulse into the CCP reg.
	ADDWF	CCPR1L,F
	MOVIW	FSR0--
	ADDWFC	CCPR1H,F
	MOVLW	CCPCON_Clr	;Clear output on match
	MOVWF	CCP1CON	;CCP1 clr on match
;Calculate dwell time
	MOVLW	LOW kServoDwellTimeA	;2.25mS
	MOVWF	CalcdDwell
	MOVLW	HIGH kServoDwellTimeA
	MOVWF	CalcdDwellH
	MOVIW	FSR0++	;SigOutTime0_7
	SUBWF	CalcdDwell,F
	MOVIW	FSR0++	;SigOutTime0_7+1
	SUBWFB	CalcdDwellH,F
; Store calc'd dwell for later
	LSLF	ServoIDX,W	;FSR0 = DwellTime0_7 + ServoIDX * 2
	ADDLW	LOW DwellTime0_7
	MOVWF	FSR0L
	MOVF	CalcdDwell,W
	MOVWI	FSR0++
	MOVF	CalcdDwellH,W
	MOVWI	FSR0++
;Set Value Sent Flag
	BSF	INDF1,ValueSentFlag0_7
	BSF	ServoCtlFlags,CyclePulseEnd1
	MOVLB	0	; bank 0
	BCF	PIR1,CCP1IF
	GOTO	IRQ_Servo2_PulseOrDwell
;
;This servo was not active, see if the next one is active
IRQ_Servo1_InAct	MOVLW	LOW kServoDwellTimeA
	ADDWF	CCPR1L,F
	MOVLW	HIGH kServoDwellTimeA
	ADDWFC	CCPR1H,F
	BSF	ServoCtlFlags,AddrChngDwell
	MOVLW	CCPCON_Int	; No, Interrupt only
	MOVWF	CCP1CON
;	
	MOVLB	0	; bank 0
	BCF	PIR1,CCP1IF
	GOTO	IRQ_Servo2_PulseOrDwell
;
; output went low so this cycle is done, set dwell time
IRQ_Servo1_OL	LSLF	ServoIDX,W	; FSR0 = DwellTime0_7 + ServoIDX * 2
	ADDLW	LOW DwellTime0_7
	MOVWF	FSR0L
	MOVLW	HIGH DwellTime0_7
	MOVWF	FSR0H
;  Copy dwell to ccpr
	MOVIW	FSR0++
	ADDWF	CCPR1L,F
	MOVIW	FSR0++
	ADDWFC	CCPR1H,F
;
	BCF	ServoCtlFlags,CyclePulseEnd1
	BSF	ServoCtlFlags,AddrChngDwell
	GOTO	IRQ_Servo1_IntOnly
;
; Next servo, If next servo is active set CCP to go high else interrupt only.
IRQ_Servo_Next	BCF	ServoCtlFlags,AddrChngDwell
	BSF	ServoCtlFlags,CyclePulseStart
	MOVLW	LOW kServoAddrTime
	ADDWF	CCPR1L,W
	MOVWF	CCPR1L
	MOVWF	CCPR2L
	MOVLW	HIGH kServoAddrTime
	ADDWFC	CCPR1H,W
	MOVWF	CCPR1H
	MOVWF	CCPR2H
; Next servo index
	INCF	ServoIDX,W
	ANDLW	0x07
	MOVWF	ServoIDX
	ADDLW	LOW ServoFlags
	MOVWF	FSR1L
	MOVLW	HIGH ServoFlags
	MOVWF	FSR1H
; Change Address
	MOVLB	0x02
	MOVF	LATA,W
	ANDLW	b'11111000'
	MOVLB	0x05
	IORWF	ServoIDX,W
	MOVLB	0x02
	MOVWF	LATA
	MOVLB	0x05
;
	MOVLW	CCPCON_Set	;Clr CCP1 to go high on next match
	BTFSS	INDF1,ServoOnBit0_7	;Is the next servo active?
	MOVLW	CCPCON_Int	; No, Interrupt only
	MOVWF	CCP1CON
;
	MOVLW	CCPCON_Set
	BTFSS	INDF1,ServoOnBit8_15	;Is the next servo active?
	MOVLW	CCPCON_Int	; No, Interrupt only
	MOVWF	CCP2CON
;	
	MOVLB	0	; bank 0
	BCF	PIR1,CCP1IF
	BCF	PIR2,CCP2IF
	RETURN
;
IRQ_Servo1_IntOnly	MOVLW	CCPCON_Int	; No, Interrupt only
	MOVWF	CCP1CON
;	
IRQ_Servo1_X	MOVLB	0	; bank 0
	BCF	PIR1,CCP1IF
	RETURN
;
;===========================================
; Interupt Service Routine for CCP2
; Servos 8..15
;
ISR_ServoCCP2	MOVLB	0x05	; bank 5
;
	BTFSC	ServoCtlFlags,CyclePulseEnd2	;Is the current pulse ending?
	GOTO	IRQ_Servo2_OL
;
	GOTO	IRQ_Servo2_X	;tc
; if not set to do anything fall thru and start a cycle
IRQ_Servo2_PulseOrDwell:
; FSR1 = ServoFlags+ServoIDX
	MOVLB	0x05	; bank 5
	MOVF	ServoIDX,W
	ADDLW	LOW ServoFlags
	MOVWF	FSR1L
	MOVLW	HIGH ServoFlags
	MOVWF	FSR1H
;
	BTFSS	INDF1,ServoOnBit8_15	;Is this servo active?
	GOTO	IRQ_Servo2_InAct	; No
; FSR0 = SigOutTime8_15 + ServoIDX * 2		
	LSLF	ServoIDX,W
	ADDLW	LOW SigOutTime8_15
	MOVWF	FSR0L
	MOVLW	HIGH SigOutTime8_15
	MOVWF	FSR0H
; Set time to end pulse
	MOVIW	FSR0++	;Put the pulse into the CCP reg.
	ADDWF	CCPR2L,F
	MOVIW	FSR0--
	ADDWFC	CCPR2H,F
	MOVLW	CCPCON_Clr	;Clear output on match
	MOVWF	CCP2CON	;CCP1 clr on match
;Calculate dwell time
	MOVLW	LOW kServoDwellTimeA
	MOVWF	CalcdDwell
	MOVLW	HIGH kServoDwellTimeA
	MOVWF	CalcdDwellH
	MOVIW	FSR0++	;SigOutTime8_15
	SUBWF	CalcdDwell,F
	MOVIW	FSR0++	;SigOutTime8_15+1
	SUBWFB	CalcdDwellH,F
; Store calc'd dwell for later
	LSLF	ServoIDX,W	;FSR0 = DwellTime8_15 + ServoIDX * 2
	ADDLW	LOW DwellTime8_15
	MOVWF	FSR0L
	MOVF	CalcdDwell,W
	MOVWI	FSR0++
	MOVF	CalcdDwellH,W
	MOVWI	FSR0++
;Set Value Sent Flag
	BSF	INDF1,ValueSentFlag8_15
	BSF	ServoCtlFlags,CyclePulseEnd2
	GOTO	IRQ_Servo2_X
;
;This servo was not active, see if the next one is active
IRQ_Servo2_InAct	MOVLW	LOW kServoDwellTimeA
	ADDWF	CCPR2L,F
	MOVLW	HIGH kServoDwellTimeA
	ADDWFC	CCPR2H,F
	GOTO	IRQ_Servo2_IntOnly
;
; output went low so this cycle is done, set dwell time
IRQ_Servo2_OL	LSLF	ServoIDX,W	; FSR0 = DwellTime8_15 + ServoIDX * 2
	ADDLW	LOW DwellTime8_15
	MOVWF	FSR0L
	MOVLW	HIGH DwellTime8_15
	MOVWF	FSR0H
; Copy dwell to ccpr
	MOVIW	FSR0++
	ADDWF	CCPR2L,F
	MOVIW	FSR0++
	ADDWFC	CCPR2H,F
;
	BCF	ServoCtlFlags,CyclePulseEnd2
;
IRQ_Servo2_IntOnly	MOVLW	CCPCON_Int	; No, Interrupt only
	MOVWF	CCP2CON
;
IRQ_Servo2_X	MOVLB	0	; bank 0
	BCF	PIR2,CCP2IF
IRQ_Servo2_End	RETURN
;=========================================================================================
;=========================================================================================
; Servo motion control
; Call often from main loop
;
IdleServos	MOVLB	0x04	; bank 4
	CALL	SetFSR0ToServoFlags
	BTFSC	CMDServoIDX,3
	GOTO	IdleServos8_15
;
	BTFSS	INDF0,ServoOnBit0_7	;Is this servo active?
	GOTO	IdleServos_Next	; No
	BTFSS	INDF0,ValueSentFlag0_7	;Ready for an update?
	GOTO	IdleServos_Next	; No
	GOTO	IdleServos_Update
;
IdleServos8_15	BTFSS	INDF0,ServoOnBit8_15	;Is this servo active?
	GOTO	IdleServos_Next	; No
	BTFSS	INDF0,ValueSentFlag8_15	;Ready for an update?
	GOTO	IdleServos_Next	; No
;
IdleServos_Update	BTFSS	CMDServoIDX,3
	BCF	INDF0,ValueSentFlag0_7	;Mark as updated
	BTFSC	CMDServoIDX,3
	BCF	INDF0,ValueSentFlag8_15	;Mark as updated
;
;FSR0 = Command Position
	LSLF	CMDServoIDX,W
	ADDLW	LOW CMDSigTime0_7
	MOVWF	FSR0L
	MOVLW	HIGH CMDSigTime0_7
	MOVWF	FSR0H
;FSR1 = Current Position
	LSLF	CMDServoIDX,W
	ADDLW	LOW SigOutTime0_7
	MOVWF	FSR1L
	MOVLW	HIGH SigOutTime0_7
	MOVWF	FSR1H
; Param7D:Param7C = (FSR0)-(FSR1) Cmd-Cur
	MOVF	INDF1,W	;Cur, SigOutTime
	SUBWF	INDF0,W	;Cmd
	MOVWF	Param7C
	INCF	FSR0L,F
	INCF	FSR1L,F
	MOVF	INDF1,W	;SigOutTimeH
	SUBWFB	INDF0,W
	MOVWF	Param7D
	DECF	FSR1L,F	;FSR1=SigOutTime+Servo#x2
;(FSR0)=ServoFlags
; if Param7D:Param7C = 0 then we are In Position
	MOVF	Param7C,W
	IORWF	Param7D,W
	SKPNZ		;Command = Current?
	GOTO	IdleServos_InPos	; Yes, we are in-position
;
	CALL	IdleServos_Accel
; Clear the In-Position bit
	CALL	SetFSR0ToServoFlags
	BTFSS	CMDServoIDX,3
	BCF	INDF0,InPositionFlag0_7
	BTFSC	CMDServoIDX,3
	BCF	INDF0,InPositionFlag8_15
;
	BTFSS	Param7D,7	;Cmd<Cur? Set if Cur>Cmd
	GOTO	IdleServos_MovPlus	; Yes
;Move minus
	BTFSS	CMDServoIDX,3
	BCF	INDF0,MovingFWD0_7
	BTFSC	CMDServoIDX,3
	BCF	INDF0,MovingFWD8_15
; Get Speed
	CALL	GetSpeed
	MOVWF	Param78
	INCFSZ	Param7D,W	;Dist=0xFFxx?
	GOTO	IdleServos_Minus	; No
IdleServos_L1	MOVF	Param78,W
	ADDWF	Param7C,W
	BTFSS	_C	;Dist<Param78?
	GOTO	IdleServos_Minus	; No
	DECFSZ	Param78,W	;Speed>1?
	GOTO	IdleServos_DecAcc	; Yes
	GOTO	IdleServos_Minus
IdleServos_DecAcc	DECF	Param78,F
	GOTO	IdleServos_L1	
;
; Subtract speed from current position
IdleServos_Minus	MOVF	Param78,W
	SUBWF	INDF1,W	;SigOutTime
	MOVWF	Param7C
	INCF	FSR1L,F
	MOVLW	0x00
	SUBWFB	INDF1,W	;SigOutTimeH
	MOVWF	Param7D
;
;FSR1 = SigOutTime0_7+1 + Servo# x 2
IdleServos_Clamp	LSLF	CMDServoIDX,W	
;	CALL	ClampMinMax
;
IdleServos_DisInt	bcf	INTCON,GIE
	BTFSC	INTCON,GIE
	GOTO	IdleServos_DisInt
;
;FSR1 = Current Position
	MOVLB	0x04
	LSLF	CMDServoIDX,W
	ADDLW	LOW SigOutTime0_7
	MOVWF	FSR1L
	MOVLW	HIGH SigOutTime0_7
	MOVWF	FSR1H
	MOVF	Param7C,W	;SigOutTime
	MOVWI	FSR1++
	MOVF	Param7D,W
	MOVWI	FSR1++
;
	bsf	INTCON,GIE
;
	GOTO	IdleServos_Next
;
;=============================
; 7D:7C = distance to go
; FSR1=SigOutTime0_7 + Servo#x2
; FSR0=ServoFlags + Servo# & 0x07
;
IdleServos_MovPlus	BTFSS	CMDServoIDX,3
	BSF	INDF0,MovingFWD0_7
	BTFSC	CMDServoIDX,3
	BSF	INDF0,MovingFWD8_15
; Get Speed
	CALL	GetSpeed
	MOVWF	Param78
	MOVF	Param7D,F
	SKPZ		;>255 to go?
	GOTO	IdleServos_Plus	; Yes
IdleServos_L2	MOVF	Param78,W
	SUBWF	Param7C,W	;Dist-Speed
	SKPB		;Speed>Dist?
	GOTO	IdleServos_Plus	; No
	DECFSZ	Param78,W
	GOTO	IdleServos_IncAcc
	GOTO	IdleServos_Plus
IdleServos_IncAcc	DECF	Param78,F
	GOTO	IdleServos_L2	
;
IdleServos_Plus	MOVF	Param78,W	;7D:7C = CurPos + Speed
	ADDWF	INDF1,W	;SigOutTime
	MOVWF	Param7C
	INCF	FSR1L,F
	CLRW
	ADDWFC	INDF1,W	;SigOutTimeH
	MOVWF	Param7D
	GOTO	IdleServos_Clamp
;
;========================================
;
IdleServos_InPos	CALL	SetFSR0ToServoFlags
	BTFSS	CMDServoIDX,3
	BSF	INDF0,InPositionFlag0_7
	BTFSC	CMDServoIDX,3
	BSF	INDF0,InPositionFlag8_15
;	
IdleServos_Next	INCF	CMDServoIDX,W
	ANDLW	0x0F
	MOVWF	CMDServoIDX
	RETURN
;
;=========================================================================================
;Do Accel
;
; Entry: Bank 4, CMDServoIDX
; Exit:
; Ram Used: Param78, Param79, FSR0
;
IdleServos_Accel	CALL	SetFSR0ToServoFlags2
	BTFSC	CMDServoIDX,3
	GOTO	IdleServos_Accel_1
;
	BTFSC	INDF0,AccelComplete0_7	;Done with accel?
	RETURN		; Yes
	GOTO	IdleServos_Accel_2
;
IdleServos_Accel_1	BTFSC	INDF0,AccelComplete8_15	;Done with accel?
	RETURN		; Yes
;
IdleServos_Accel_2	CALL	GetSpeed
	MOVWF	Param78	;Save Cur Speed in 78
	CALL	GetMaxSpeed
	MOVWF	Param79	;Save Max Speed in 79
	SUBWF	Param78,W	;W=CurSpd-MaxSpd
	BTFSC	_Z	;Max=CurSpd?
	GOTO	AccelDone	; Yes
	SKPB		;Max>CurSpd?
	GOTO	AccelDone	; No
	CALL	GetAccelValue
	ADDWF	Param78,F	;CurSpd=CurSpd+Accel
	MOVF	Param78,W	;new CurSpd
	SUBWF	Param79,W	;Max-new Cur
	MOVF	Param78,W	;new Cur
	SKPB		;new Cur>Max?
	MOVF	Param79,W	; Yes, use max instead
	MOVWF	Param78	;New Speed
;FSR0 = CMDServoIDX*2 + AccelRampLen0_7
	LSLF	CMDServoIDX,W	
	ADDLW	LOW AccelRampLen0_7
	MOVWF	FSR0L
	MOVLW	HIGH AccelRampLen0_7
	MOVWF	FSR0H
	MOVF	Param78,W
	ADDWF	INDF0,F
	INCF	FSR0L,F
	CLRW
	ADDWFC	INDF0,F
	GOTO	SaveSpeed
;
;========================================
; Set AccelComplete flag for servo (CMDServoIDX)
;
; Entry: Bank 4
; Exit: FSR0 >> ServoFlags2
;
AccelDone	CALL	SetFSR0ToServoFlags2
	BTFSS	CMDServoIDX,3
	BSF	INDF0,AccelComplete0_7
	BTFSC	CMDServoIDX,3
	BSF	INDF0,AccelComplete8_15
	RETURN
;
;==========================================
; Save speed Param78 to ServoCurSpeed for servo (CMDServoIDX)
;
; Entry: Bank 4, Param78 = Speed
; Exit: FSR0 >> ServoCurSpeed
;
SaveSpeed	MOVF	CMDServoIDX,W
	ADDLW	LOW ServoCurSpeed0_7
	MOVWF	FSR0L
	MOVLW	HIGH ServoCurSpeed0_7
	MOVWF	FSR0H
	MOVF	Param78,W
	MOVWF	INDF0
	RETURN
;
;==========================================
; Get ServoCurSpeed for servo (CMDServoIDX)
;
; Entry: Bank 4
; Exit: FSR0 >> ServoCurSpeed, W=ServoCurSpeed
;
GetSpeed	MOVF	CMDServoIDX,W
	ADDLW	LOW ServoCurSpeed0_7
	MOVWF	FSR0L
	MOVLW	HIGH ServoCurSpeed0_7
	MOVWF	FSR0H
	MOVF	INDF0,W
	RETURN
;
;==========================================
; Get ServoMaxSpeed for servo (CMDServoIDX)
;
; Entry: Bank 4
; Exit: FSR0 >> ServoMaxSpeed, W=ServoMaxSpeed
;
GetMaxSpeed	MOVF	CMDServoIDX,W
	ADDLW	LOW ServoMaxSpeed0_7
	MOVWF	FSR0L
	MOVLW	HIGH ServoMaxSpeed0_7
	MOVWF	FSR0H
	MOVF	INDF0,W
	RETURN
;
;==========================================
; Get ServoAccelValue for servo (CMDServoIDX)
;
; Entry: Bank 4
; Exit: FSR0 >> ServoAccelValue, W=ServoAccelValue
;
GetAccelValue	MOVF	CMDServoIDX,W
	ADDLW	LOW ServoAccelValue0_7
	MOVWF	FSR0L
	MOVLW	HIGH ServoAccelValue0_7
	MOVWF	FSR0H
	MOVF	INDF0,W
	RETURN
;
;=========================================================================================
; Point FRS0 at ServoFlags2
;
; Entry: Bank 4
; Exit: FSR0 >> ServoFlags2
;
SetFSR0ToServoFlags2	MOVF	CMDServoIDX,W
	ANDLW	0x07
	ADDLW	LOW ServoFlags2
	MOVWF	FSR0L
	MOVLW	HIGH ServoFlags2
	MOVWF	FSR0H
	RETURN
;
;=========================================================================================
; Point FRS0 at ServoFlags
;
; Entry: Bank 4
; Exit: FSR0 >> ServoFlags
;
;
SetFSR0ToServoFlags	MOVF	CMDServoIDX,W
	ANDLW	0x07
	ADDLW	LOW ServoFlags
	MOVWF	FSR0L
	MOVLW	HIGH ServoFlags
	MOVWF	FSR0H
	RETURN
;
;=========================================================================================
; Set CCP1/2 to cause an interrupt the next time T1 matches CCPRn
; Call Once to activate servos
;
StartServos	MOVLB	0x02	; bank 2
	BCF	Enable0_7	; Active Low Output
	BCF	Enable8_15
	MOVLB	0x05
	CLRF	CCPR1L
	CLRF	CCPR1H
	CLRF	CCPR2L
	CLRF	CCPR2H
	MOVLW	CCPCON_Int	;interrupt on match
	MOVWF	CCP1CON
	MOVWF	CCP2CON
	BSF	ServoCtlFlags,CyclePulseStart
	MOVLB	0x01
	BSF	PIE1,CCP1IE
	BSF	PIE2,CCP2IE
	MOVLB	0x00	;Bank 0
	RETURN
;
;=========================================================================================
; Setup all data to usable values
; Call once before mainloop begins
;
; RAM Used:Param78, Param79, Param7A, FSR0
;
;
ServoInit16	CLRW
	LOADFSR0W	ServoMaxSpeed0_7
;ServoMaxSpeed 0..15
	MOVLW	DefaultMaxSpeed
	CALL	Store16Times
;ServoAccelValue 0..15
	MOVLW	DefaultAccel
	CALL	Store16Times
;ServoCurSpeed 0..15
	MOVLW	0x01
	CALL	Store16Times	;CurSpeed=min
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
	LOADFSR0W	SigOutTime0_7
; SigOutTime 0..15
	MOVLW	LOW kCenterPulseWidth
	MOVWF	Param78
	MOVLW	HIGH kCenterPulseWidth
	MOVWF	Param79
	CALL	StoreWord16Times
; DwellTime 0..15
	MOVLW	LOW kServoDwellTime
	MOVWF	Param78
	MOVLW	HIGH kServoDwellTime
	MOVWF	Param79
	CALL	StoreWord16Times
;MaxTime 0..15
	CLRW
	LOADFSR0W	MaxTime0_7
	MOVLW	LOW kMaxPulseWidth
	MOVWF	Param78
	MOVLW	HIGH kMaxPulseWidth
	MOVWF	Param79
; Fall thu to StoreWord16Times
;
StoreWord16Times	MOVLW	.16
	MOVWF	Param7A
StoreWord16Times_L1	MOVF	Param78,W
	MOVWI	FSR0++
	MOVF	Param79,W
	MOVWI	FSR0++
	DECFSZ	Param7A,F
	GOTO	StoreWord16Times_L1
	RETURN
;
Store16Times	MOVWF	Param78
	MOVLW	.16
StoreXTimes	MOVWF	Param79
	MOVF	Param78,W
StoreXTimes_L1	MOVWI	FSR0++
	DECFSZ	Param79,F
	GOTO	StoreXTimes_L1
	RETURN
;
Store8Times	MOVWF	Param78
	MOVLW	.8
	GOTO	StoreXTimes
;
;=========================================================================================
; Entry: W=Servo# 0..15, Param7D:Param7C = Dest
; Exit: W=unchanged
; Ram used: Param78, Param7C, Param7D FSR0
; Calls: none
;
StartMotion	ANDLW	0x0F
	MOVWF	Param78
; Copy 7D:7C to CMDSigTime
	LSLF	Param78,W	;x2, 0,2,4..30
	ADDLW	LOW CMDSigTime0_7
	MOVWF	FSR0L
	MOVLW	HIGH CMDSigTime0_7
	MOVWF	FSR0H
	MOVF	Param7C,W
	MOVWI	FSR0++
	MOVF	Param7D,W
	MOVWI	FSR0++
; Set ServoCurSpeed to 1 (minimum)
	MOVF	Param78,W
	ADDLW	LOW ServoCurSpeed0_7
	MOVWF	FSR0L
	MOVLW	HIGH ServoCurSpeed0_7
	MOVWF	FSR0H
	MOVLW	0x01
	MOVWF	INDF0
; Clear the In-Position Flag
	MOVF	Param78,W
	ANDLW	0x07
	ADDLW	LOW ServoFlags
	MOVWF	FSR0L
	MOVLW	HIGH ServoFlags
	MOVWF	FSR0H
	BTFSS	Param78,3
	BCF	INDF0,InPositionFlag0_7
	BTFSC	Param78,3
	BCF	INDF0,InPositionFlag8_15
; Set ServoFlags,ServoOnBit
	BTFSS	Param78,3
	BSF	INDF0,ServoOnBit0_7
	BTFSC	Param78,3
	BSF	INDF0,ServoOnBit8_15
; Clear the AccelComplete Flag
	MOVF	Param78,W
	ANDLW	0x07
	ADDLW	LOW ServoFlags2
	MOVWF	FSR0L
	MOVLW	HIGH ServoFlags2
	MOVWF	FSR0H
	BTFSS	Param78,3
	BCF	INDF0,AccelComplete0_7
	BTFSC	Param78,3
	BCF	INDF0,AccelComplete8_15
; Clear AccelRampLen
	LSLF	Param78,W	;x2, 0,2,4..30
	ADDLW	LOW AccelRampLen0_7
	MOVWF	FSR0L
	MOVLW	HIGH AccelRampLen0_7
	MOVWF	FSR0H
	CLRW
	MOVWI	FSR0++
	MOVWI	FSR0++
	RETURN
;
;=========================================================================================
; Servo test
;	if oldCode
TestServoLib	MOVLB	0x05
	BTFSS	ServoFlags,InPositionFlag0_7 ;Servo 0 in position?
	GOTO	S_Text_End	; No
;
	MOVLB	0x04
	MOVLW	HIGH d'2200'	;1100uS
	SUBWF	CMDSigTime0_7+1,W
	SKPZ		;At 1100 end?
	GOTO	STest_1	; No
;
	MOVLW	HIGH d'3800'	;1900 uS
	MOVWF	Param7D
	MOVLW	LOW d'3800'
	MOVWF	Param7C
	MOVLW	0x00	;Servo# 0
	CALL	StartMotion
	GOTO	S_Text_End
;
STest_1	MOVLW	HIGH d'2200'	;1900 uS
	MOVWF	Param7D
	MOVLW	LOW d'2200'
	MOVWF	Param7C
	MOVLW	0x00	;Servo# 0
	CALL	StartMotion
;
S_Text_End:
; Servo test3
	MOVLB	0x05
	BTFSS	ServoFlags+1,InPositionFlag0_7 ;Servo 1 in position?
	GOTO	S_Text_End3	; No
;
	MOVLB	0x04
	MOVLW	HIGH d'2200'	;1100uS
	SUBWF	CMDSigTime0_7+3,W
	SKPZ		;At 1100 end?
	GOTO	STest3_1	; No
;
	MOVLW	HIGH d'3800'	;1900 uS
	MOVWF	Param7D
	MOVLW	LOW d'3800'
	MOVWF	Param7C
	MOVLW	0x01	;Servo# 8
	CALL	StartMotion
	GOTO	S_Text_End3
;
STest3_1	MOVLW	HIGH d'2200'	;1900 uS
	MOVWF	Param7D
	MOVLW	LOW d'2200'
	MOVWF	Param7C
	MOVLW	0x01	;Servo# 8
	CALL	StartMotion
;
S_Text_End3:
;	endif
;
;	if oldCode
; Servo test2
	MOVLB	0x05
	BTFSS	ServoFlags,InPositionFlag8_15 ;Servo 8 in position?
	GOTO	S_Text_End2	; No
;
	MOVLB	0x04
	MOVLW	HIGH d'2200'	;1100uS
	SUBWF	CMDSigTime8_15+1,W
	SKPZ		;At 1100 end?
	GOTO	STest2_1	; No
;
	MOVLW	HIGH d'3800'	;1900 uS
	MOVWF	Param7D
	MOVLW	LOW d'3800'
	MOVWF	Param7C
	MOVLW	0x08	;Servo# 8
	CALL	StartMotion
	GOTO	S_Text_End2
;
STest2_1	MOVLW	HIGH d'2200'	;1900 uS
	MOVWF	Param7D
	MOVLW	LOW d'2200'
	MOVWF	Param7C
	MOVLW	0x08	;Servo# 8
	CALL	StartMotion
;
S_Text_End2:
; Servo test4
	MOVLB	0x05
	BTFSS	ServoFlags+1,InPositionFlag8_15 ;Servo 9 in position?
	GOTO	S_Text_End4	; No
;
	MOVLB	0x04
	MOVLW	HIGH d'2200'	;1100uS
	SUBWF	CMDSigTime8_15+3,W
	SKPZ		;At 1100 end?
	GOTO	STest4_1	; No
;
	MOVLW	HIGH d'3800'	;1900 uS
	MOVWF	Param7D
	MOVLW	LOW d'3800'
	MOVWF	Param7C
	MOVLW	0x09	;Servo# 8
	CALL	StartMotion
	GOTO	S_Text_End4
;
STest4_1	MOVLW	HIGH d'2200'	;1900 uS
	MOVWF	Param7D
	MOVLW	LOW d'2200'
	MOVWF	Param7C
	MOVLW	0x09	;Servo# 8
	CALL	StartMotion
;
S_Text_End4:
;	endif
	RETURN
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
; ClampMinMax(Param7D:Param7C,W = offset servo# * 2 (0,2,4..30)
;
; Entry: Param7D:Param7C, W=servo# x 2
; Exit: Param7D:Param7C=ClampInt(Param7D:Param7C,(MinTime0_7+Param78),(MaxTime0_7+Param78))
; Ram used: Param78, Param7C, Param7D, FSR0
;
ClampMinMax	MOVWF	Param78
	INCF	Param78,W
	ADDLW	LOW MaxTime0_7
	MOVWF	FSR0L
	MOVLW	HIGH MaxTime0_7
	MOVWF	FSR0H
	MOVIW	FSR0--	;MaxTime0_7+Offset+1
	SUBWF	Param7D,W	;7D-kMaxPulseWidth
	SKPNB		;7D<Max?
	GOTO	ClampMinMax_1	; Yes
	SKPZ		;7D=Max?
	GOTO	ClampMinMax_too	; No, its greater.
	MOVF	INDF0,W	; Yes, MSB was equal check LSB
	SUBWF	Param7C,W	;7C-kMaxPulseWidth
	SKPNZ		;=kMaxPulseWidth
	RETURN		;Yes
	SKPB		;7C<Max?
	GOTO	ClampMinMax_too	; No
	RETURN		; Yes
;
ClampMinMax_1	INCF	Param78,W
	ADDLW	LOW MinTime0_7
	MOVWF	FSR0L
	MOVLW	HIGH MinTime0_7
	MOVWF	FSR0H
	MOVIW	FSR0--	;MinTime0_7+Offset+1
	SUBWF	Param7D,W	;7D-kMinPulseWidth
	SKPNB		;7D<Min?
	GOTO	ClampMinMax_too	; Yes
	SKPZ		;=Min?
	RETURN		; No, 7D>kMinPulseWidth
	MOVF	INDF0,W	; Yes, MSB is a match
	SUBWF	Param7C,W	;7C-kMinPulseWidth
	SKPB		;7C>=Min?
	RETURN		; Yes
;	
ClampMinMax_too	MOVIW	FSR0++
	MOVWF	Param7C
	MOVF	INDF0,W
	MOVWF	Param7D
	RETURN
;
	if oldCode
;=========================================================================================
;=====================================================================================
;
MoveTo78	MOVWF	FSR0L
	MOVF	INDF0,W
	MOVWF	Param78
	INCF	FSR0L,F
	MOVF	INDF0,W
	MOVWF	Param79
	RETURN
;
;=====================================================================================
;
MoveTo7C	MOVWF	FSR0L
	MOVF	INDF0,W
	MOVWF	Param7C
	INCF	FSR0L,F
	MOVF	INDF0,W
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
MoveFrom7C	MOVWF	FSR0L
	MOVF	Param7C,W
	MOVWF	INDF0
	INCF	FSR0L,F
	MOVF	Param7D,W
	MOVWF	INDF0
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
	endif
;


























