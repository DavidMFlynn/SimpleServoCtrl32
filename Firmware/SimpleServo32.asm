;====================================================================================================
;
;    Filename:      SimpleServos.asm
;    Date:          5/25/2015
;    File Version:  1.0d2
;    
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;====================================================================================================
;    SimpleServos is a 16 servo controller with speed, accel and position control.
;
;
;    History:
;
; 1.0d2   5/25/2015	Incomplete, ISR is almost there.
; 1.0d1   4/4/2015	First code. Copied from StepperTest.
;
;====================================================================================================
; Options
;
;====================================================================================================
;====================================================================================================
; What happens next:
;   At power up the system LED will blink.
;
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
;   Pin 16 (RA7/CCP2) Pulse output for Servos 8..15
;   Pin 17 (RA0) Address A0 (output)
;   Pin 18 (RA1) Address A1 (output)
;
;====================================================================================================
;
;
	list	p=16f1847,r=hex,W=0	; list directive to define processor
	nolist
	include	p16f1847.inc	; processor specific variable definitions
	list
;
	__CONFIG _CONFIG1,_FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF & _IESO_OFF
;
;
; INTOSC oscillator: I/O function on CLKIN pin
; WDT disabled
; PWRT disabled
; MCLR/VPP pin function is digital input
; Program memory code protection is disabled
; Data memory code protection is disabled
; Brown-out Reset enabled
; CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
; Internal/External Switchover mode is disabled
; Fail-Safe Clock Monitor is enabled
;
	__CONFIG _CONFIG2,_WRT_OFF & _PLLEN_OFF & _LVP_OFF
;
; Write protection off
; 4x PLL disabled
; Stack Overflow or Underflow will cause a Reset
; Brown-out Reset Voltage (Vbor), low trip point selected.
; Low-voltage programming enabled
;
; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.
;
	constant	oldCode=0
	constant	useRS232=0
;
#Define	_C	STATUS,C
#Define	_Z	STATUS,Z
;
;====================================================================================================
	nolist
	include	F1847_Macros.inc
	list
;
;    Port A bits
PortADDRBits	EQU	b'01100000'
PortAValue	EQU	b'00011000'
;
#Define	Servo_A0	LATA,0	;Output
#Define	Servo_A1	LATA,1	;Output
#Define	Servo_A2	LATA,2	;Output
#Define	Enable0_7	LATA,3	;Output RA3
#Define	Enable8_15	LATA,4	;Output RA4
#Define	RA5_In	PORTA,5	;unused
#Define	RA6_In	PORTA,6	;unused
#Define	RA7_Out	PORTA,7	;CCP2 Output
;
Servo_AddrDataMask	EQU	0xF8
;
;
;    Port B bits
PortBDDRBits	EQU	b'11110111'	;LEDs Out Others In
PortBValue	EQU	b'00000000'
;
#Define	SW3_In	PORTB,0	;SW3/LED3
#Define	RB1_In	PORTB,1	;I2C Data
#Define	SW2_In	PORTB,2	;SW2/LED2
#Define	RB3_Out	PORTB,3	;CCP1 Output
#Define	RB4_In	PORTB,4	;I2C Clock
#Define	SW1_In	PORTB,5	;SW1/LED1
#Define	RB6_In	PORTB,6	;N.C.
#Define	RB7_In	PORTB,7	;N.C.
LED1_Bit	EQU	5	;LED1 (Active Low Output)
LED2_Bit	EQU	2	;LED2 (Active Low Output)
LED3_Bit	EQU	0	;LED3 (Active Low Output)
#Define	LED1_Tris	TRISB,LED1_Bit	;LED1 (Active Low Output)
#Define	LED2_Tris	TRISB,LED2_Bit	;LED2 (Active Low Output)
#Define	LED3_Tris	TRISB,LED3_Bit	;LED3 (Active Low Output)
;
;
;========================================================================================
;========================================================================================
;
;Constants
All_In	EQU	0xFF
All_Out	EQU	0x00
;
TMR0Val	EQU	0xB2	;0xB2=100Hz, 0.000128S/Count
LEDTIME	EQU	d'100'	;1.00 seconds
LEDErrorTime	EQU	d'10'
;
T1CON_Val	EQU	b'00000001'	;PreScale=1,Fosc/4,Timer ON
TMR1L_Val	EQU	0x3C	; -2500 = 2.5 mS, 400 steps/sec
TMR1H_Val	EQU	0xF6
;
;TMR1L_Val	EQU	0x1E	; -1250 = 1.25 mS, 800 steps/sec
;TMR1H_Val	EQU	0xFB
;
;TMR1L_Val	EQU	0x8F	; -625 = 0.625 mS, 1600 steps/sec
;TMR1H_Val	EQU	0xFD
;
TXSTA_Value	EQU	b'00100000'	;8 bit, TX enabled, Async, low speed
RCSTA_Value	EQU	b'10010000'	;RX enabled, 8 bit, Continious receive
; 8MHz clock low speed (BRGH=0,BRG16=1)
Baud_300	EQU	d'1666'	;0.299, -0.02%
Baud_1200	EQU	d'416'	;1.199, -0.08%
Baud_2400	EQU	d'207'	;2.404, +0.16%
Baud_9600	EQU	d'51'	;9.615, +0.16%
BaudRate	EQU	Baud_9600
;
;
DebounceTime	EQU	d'10'
;
;================================================================================================
;***** VARIABLE DEFINITIONS
; there are 256 bytes of ram, Bank0 0x20..0x7F, Bank1 0xA0..0xEF, Bank2 0x120..0x16F
; there are 256 bytes of EEPROM starting at 0x00 the EEPROM is not mapped into memory but
;  accessed through the EEADR and EEDATA registers
;================================================================================================
;  Bank0 Ram 020h-06Fh 80 Bytes
;
Bank0_Vars	udata	0x20 
;
LED_Time	RES	1
lastc	RES	1	;part of tickcount timmer
tickcount	RES	1	;Timer tick count
;
StatLED_Time	RES	1
Stat_Count	RES	1
;
	if useRS232
TXByte	RES	1	;Next byte to send
RXByte	RES	1	;Last byte received
WorkingRXByte	RES	1
RS232Flags	RES	1
#Define	DataSentFlag	RS232Flags,0
#Define	DataReceivedFlag	RS232Flags,1
	endif
;
;
;
EEAddrTemp	RES	1	;EEProm address to read or write
EEDataTemp	RES	1	;Data to be writen to EEProm
;
;
Timer1Lo	RES	1	;1st 16 bit timer
Timer1Hi	RES	1	; one second RX timeiout
;
Timer2Lo	RES	1	;2nd 16 bit timer
Timer2Hi	RES	1	;
;
Timer3Lo	RES	1	;3rd 16 bit timer
Timer3Hi	RES	1	;GP wait timer
;
Timer4Lo	RES	1	;4th 16 bit timer
Timer4Hi	RES	1	; debounce timer
;
;
SysFlags	RES	1
#Define	SW1_Flag	SysFlags,0
#Define	SW2_Flag	SysFlags,1
#Define	SW3_Flag	SysFlags,2
#Define	LED2_Flag	SysFlags,3
#Define	LED3_Flag	SysFlags,4
;
;#Define	FirstRAMParam	MinSpdLo
;#Define	LastRAMParam	SysFlags
;
;
;================================================================================================
;  Bank2 Ram 120h-16Fh 80 Bytes
;
; I2C Stuff is here
;Note: only upper 7 bits of address are used
I2C_ADDRESS	EQU	0x30	; Slave address
RX_ELEMENTS	EQU	.32	; number of allowable array elements, in this case 16
TX_ELEMENTS	EQU	.8	; Status nibble for each servo
I2C_TX_Init_Val	EQU	0xAA	; value to load into transmit array to send to master
I2C_RX_Init_Val	EQU	0xAA	; value to load into received data array
;
Bank2_Vars	udata	0x120   
I2C_ARRAY_TX	res	RX_ELEMENTS	; array to transmit to master
I2C_ARRAY_RX 	res	TX_ELEMENTS 	; array to receive from master
;
;ContigMem	udata	0x2100
;BigBuffer	EQU	0x2100
	cblock	0x2100
	BigBuffer:0x80
	endc
;
	include <ServoLib32.h>
;
;=======================================================================================================
;  Common Ram 70-7F same for all banks
;      except for ISR_W_Temp these are used for paramiter passing and temp vars
;=======================================================================================================
;
	cblock	0x70
	Param70
	Param71
	Param72
	Param73
	Param74
	Param75
	Param76
	Param77
	Param78
	Param79
	Param7A
	Param7B
	Param7C
	Param7D
	Param7E
	Param7F
	endc
;
#Define	INDEX_I2C	Param70	;I2C Data Pointer
#Define	GFlags	Param71
#Define	I2C_TXLocked	Param71,0	; Set/cleared by ISR, data is being sent
#Define	I2C_RXLocked	Param71,1	; Set/cleared by ISR, data is being received
#Define	I2C_NewRXData	Param71,2	; Set by ISR, The new data is here!
;
;=========================================================================================
;Conditions
HasISR	EQU	0x80	;used to enable interupts 0x80=true 0x00=false
;
;=========================================================================================
;==============================================================================================
; ID Locations
;	ORG	0x2000
;	DE	'1','.','0','0'
;
;==============================================================================================
; EEPROM locations (NV-RAM) 0x00..0x7F (offsets)
	cblock	0x0000
;
nvMinSpdLo;	RES	1	;0x1E; -1250 = 1.25 mS, 60RPM
nvMinSpdHi;	RES	1	;0xFB
nvMaxSpdLo;	RES	1	;0x8F; -625 = 0.625 mS, 120RPM
nvMaxSpdHi;	RES	1	;0xFD
;
nvSysFlags;	RES	1
	endc
;
#Define	nvFirstParamByte	nvMinSpdLo
#Define	nvLastParamByte	nvSysFlags
;
;
;==============================================================================================
;============================================================================================
;
;
	ORG	0x000	; processor reset vector
	CLRF	STATUS
	CLRF	PCLATH
  	goto	start	; go to beginning of program
;
;===============================================================================================
; Interupt Service Routine
;
; we loop through the interupt service routing every 0.008192 seconds
;
;
	ORG	0x004	; interrupt vector location
	CLRF	BSR	; bank0
;
;
	btfss	INTCON,T0IF
	goto	SystemBlink_end
;
	movlw	TMR0Val	;256x39+16 cycles (10,000uS)
	addwf	TMR0,F	; reload TMR0 with -40
	bcf	INTCON,T0IF	; reset interupt flag bit
;------------------
; These routines run 100 times per second
;------------------
;Decrement timers until they are zero
; 
	call	DecTimer1	;if timer 1 is not zero decrement
	call	DecTimer2
	call	DecTimer3
	call	DecTimer4
;
;-----------------------------------------------------------------
; blink LEDs
	MOVLW	LOW TRISB
	MOVWF	FSR0L
	MOVLW	HIGH TRISB
	MOVWF	FSR0H
; All LEDs off
	BSF	INDF0,LED1_Bit
	BSF	INDF0,LED2_Bit
	BSF	INDF0,LED3_Bit
;
; Read SW's
	BCF	SW1_Flag
	BCF	SW2_Flag
	BCF	SW3_Flag
	BTFSS	SW1_In
	BSF	SW1_Flag
	BTFSS	SW2_In
	BSF	SW2_Flag
	BTFSS	SW3_In
	BSF	SW3_Flag
; Dec LED time
	DECFSZ	tickcount,F	;Is it time?
	GOTO	SystemBlink_end	; No, not yet
;
	MOVF	LED_Time,W
	MOVWF	tickcount
; Flash LEDs
	BCF	INDF0,LED1_Bit
	BTFSC	LED2_Flag
	BCF	INDF0,LED2_Bit
	BTFSC	LED3_Flag
	BCF	INDF0,LED3_Bit
;
;
SystemBlink_end	
;
;=========================================================================================
	MOVLB	0	;Bank0
	BTFSC	PIR1,CCP1IF
	CALL	ISR_ServoCCP1
;
	BTFSC	PIR2,CCP2IF
	CALL	ISR_ServoCCP2
;
;=========================================================================================
;-----------------------------------------------------------------------------------------
; I2C Com
IRQ_4	MOVLB	0x00
	btfss 	PIR1,SSP1IF 	; Is this a SSP interrupt?
	goto 	IRQ_4_End 	; if not, bus collision int occurred
	banksel	SSP1STAT						
	btfsc	SSP1STAT,R_NOT_W	; is it a master read:
	goto	I2C_READ	; if so go here
	goto	I2C_WRITE	; if not, go here
I2C_READ_Return:
I2C_WRITE_Return	movlb	0x00
	bcf 	PIR1,SSP1IF	; clear the SSP interrupt flag
IRQ_4_End
;-----------------------------------------------------------------------------------------
; I2C Bus Collision
IRQ_5	MOVLB	0x00
	btfss	PIR2,BCL1IF
	goto	IRQ_5_End
	banksel	SSPBUF						
	clrf	SSPBUF	; clear the SSP buffer
	movlb	0x00	;banksel PIR2
	bcf	PIR2,BCL1IF	; clear the SSP interrupt flag	
	banksel	SSPCON1
	bsf	SSPCON1,CKP	; release clock stretch
	movlb	0x00
;
IRQ_5_End:
;
;--------------------------------------------------------------------
;
	retfie		; return from interrupt
;
;
;==============================================================================================
;==============================================================================================
;
	include	F1847_Common.inc
	include	I2C_SLAVE.inc
;
;==============================================================================================
;
start	MOVLB	0x01	; select bank 1
	bsf	OPTION_REG,NOT_WPUEN	; disable pullups on port B
	bcf	OPTION_REG,TMR0CS	; TMR0 clock Fosc/4
	bcf	OPTION_REG,PSA	; prescaler assigned to TMR0
	bsf	OPTION_REG,PS0	;111 8mhz/4/256=7812.5hz=128uS/Ct=0.032768S/ISR
	bsf	OPTION_REG,PS1	;101 8mhz/4/64=31250hz=32uS/Ct=0.008192S/ISR
	bsf	OPTION_REG,PS2
;
	MOVLB	0x01	; bank 1
	MOVLW	b'01110010'	; 8 MHz
	MOVWF	OSCCON
	movlw	b'00010111'	; WDT prescaler 1:65536 period is 2 sec (RESET value)
	movwf	WDTCON 	
;	
	MOVLB	0x03	; bank 3
	CLRF	ANSELA
	CLRF	ANSELB	;Digital I/O
;
; setup timer 1 for 0.5uS/count
;
	MOVLB	0x00	; bank 0
	MOVLW	T1CON_Val
	MOVWF	T1CON
	bcf	T1GCON,TMR1GE	;always count
;
;
	MOVLB	0x00	;Bank 0
; setup data ports
	movlw	PortBValue
	movwf	PORTB	;init port B
	movlw	PortAValue
	movwf	PORTA
	MOVLB	0x01	; bank 1
	movlw	PortADDRBits
	movwf	TRISA
	movlw	PortBDDRBits	;setup for programer
	movwf	TRISB
;
	if useRS232
; setup serial I/O
	MOVLW	TXSTA_Value
	MOVWF	TXSTA
	MOVLW	BaudRate
	MOVWF	SPBRG
	MOVLB	0x00	; bank 0
	MOVLW	RCSTA_Value
	MOVWF	RCSTA
	endif
;
	CLRWDT
; clear memory to zero
	CALL	ClearRam
;-----------------------
; big buffer test
	MOVLW	LOW BigBuffer
	MOVWF	FSR1L
	MOVLW	HIGH BigBuffer
	MOVWF	FSR1H
	MOVLW	0x22
	MOVWF	INDF1
	SUBWF	INDF1,W
	SKPZ
	GOTO	$
;-----------------------
; Setup CCP1 & CCP2
	MOVLB	0x02	; bank 2
	BSF	APFCON0,CCP2SEL
;
	MOVLB	0x00
	MOVLW	LEDTIME
	MOVWF	LED_Time
;
	CLRWDT
	MOVLB	0x00
	call	Init_I2C	;setup I2C
;
	bsf	INTCON,PEIE	; enable periferal interupts
	bsf	INTCON,T0IE	; enable TMR0 interupt
	bsf	INTCON,GIE	; enable interupts
;
;=========================================================================================
; Setup default servo data
;
	CALL	ServoInit16
	CALL	StartServos
;
; tc Test servo code
;	MOVLB	0x05
;	BSF	ServoFlags,ServoOnBit0_7 ;Servo 0 on
;	BSF	ServoFlags,ServoOnBit8_15 ;Servo 8 on
;	BSF	ServoFlags+1,ServoOnBit0_7 ;Servo 1 on
;	MOVLB	0x03
;	MOVLW	0x01
;	MOVWF	ServoCurSpeed0_7	;speed 0
;	MOVWF	ServoCurSpeed8_15	;speed 8
;	MOVWF	ServoCurSpeed0_7+1	;speed 1
	MOVLB	0x03
	MOVLW	0x03
	MOVWF	ServoMaxSpeed0_7+1
	MOVLW	HIGH d'3000'	;1900 uS
	MOVWF	Param7D
	MOVLW	LOW d'3000'
	MOVWF	Param7C
	MOVLW	0x02	;Servo# 0
	CALL	StartMotion
;
;=========================================================================================
;=========================================================================================
;  Main Loop
;
;=========================================================================================
MainLoop	CLRWDT
;
;	CALL	I2C_DataInturp
;
;	CALL	I2C_DataSender
;
	CALL	IdleServos
	CALL	TestServoLib
;
	MOVLB	0x00
	BTFSC	SW2_Flag
	GOTO	start
;
	goto	MainLoop
;
	include <ServoLib32.inc>
;
;=========================================================================================
;=========================================================================================
; Parse the incoming data and put it where it belongs
; Even byte: data type nibble, 4 MSb
; Odd byte: data
kServoPosCmd	EQU	0x80	;Position Command CMDSigTime
kServoMaxSpd	EQU	0x90	;LSB is ServoMaxSpeed
kServoAccel	EQU	0xA0	;LSB is ServoAccelValue
kServoON	EQU	0xB0	;Set ServoActive
kServoOFF	EQU	0xC0	;Clr ServoActive
kServoMinTime	EQU	0xD0	;Minimum pulse time (900uS=1800)
kServoMaxTime	EQU	0xE0	;Maximum pulse time (2100uS=4200)
;
;
I2C_DataInturp	BTFSC	I2C_RXLocked
	RETURN
	BTFSS	I2C_NewRXData	;Data is new?
	RETURN		; No
	BCF	I2C_NewRXData
	CLRF	Param79	;offset
I2C_DataInturp_L1	LOADFSR0	I2C_ARRAY_RX,Param79
	MOVIW	FSR0++
	MOVWF	Param78
	ANDLW	0xF0
	MOVWF	Param7A
	MOVLW	0x0F
	ANDWF	Param78,F
; *** kServoPosCmd ***
	MOVF	Param7A,W
	SUBLW	kServoPosCmd
	SKPZ
	GOTO	I2C_DataInturp_1
	LOADFSR1	CMDSigTime0_7,Param79
	GOTO	I2C_DI_Mov2
; *** kServoMaxSpd ***
I2C_DataInturp_1	MOVF	Param7A,W
	SUBLW	kServoMaxSpd
	SKPZ
	GOTO	I2C_DataInturp_2
	LSRF	Param79,W
	LOADFSR1W	ServoMaxSpeed0_7
I2C_DI_Mov1	MOVIW	FSR0++
	MOVWI	FSR1++
	GOTO	I2C_DataInturp_Next
; *** kServoAccel ***
I2C_DataInturp_2	MOVF	Param7A,W
	SUBLW	kServoAccel
	SKPZ
	GOTO	I2C_DataInturp_3
	LSRF	Param79,W
	LOADFSR1	ServoAccelValue0_7,WREG
	GOTO	I2C_DI_Mov1
; *** kServoON ***
I2C_DataInturp_3	MOVF	Param7A,W
	SUBLW	kServoON
	SKPZ
	GOTO	I2C_DataInturp_4
;
	LSRF	Param79,W
	LOADFSR1W	ServoFlags
	BTFSS	Param79,4
	BSF	INDF1,ServoOnBit0_7
	BTFSC	Param79,4
	BSF	INDF1,ServoOnBit8_15
	GOTO	I2C_DataInturp_Next
; *** kServoOFF ***
I2C_DataInturp_4	MOVF	Param7A,W
	SUBLW	kServoOFF
	SKPZ
	GOTO	I2C_DataInturp_5
;
	LSRF	Param79,W
	LOADFSR1W	ServoFlags
	BTFSS	Param79,4
	BCF	INDF1,ServoOnBit0_7
	BTFSC	Param79,4
	BCF	INDF1,ServoOnBit8_15	
	GOTO	I2C_DataInturp_Next
; *** kServoMinTime ***
I2C_DataInturp_5	MOVF	Param7A,W
	SUBLW	kServoMinTime
	SKPZ
	GOTO	I2C_DataInturp_6
	LOADFSR1	MinTime0_7,Param79
	GOTO	I2C_DI_Mov2
; *** kServoMaxTime ***
I2C_DataInturp_6	MOVF	Param7A,W
	SUBLW	kServoMaxTime
	SKPZ
	GOTO	I2C_DataInturp_7
	LOADFSR1	MaxTime0_7,Param79
	GOTO	I2C_DI_Mov2
;	
I2C_DataInturp_7:
;
I2C_DI_Mov2	MOVF	Param78,W
	MOVWI	FSR1++
	MOVIW	FSR0++
	MOVWI	FSR1++
;
I2C_DataInturp_Next	INCF	Param79,F
	INCF	Param79,F
	MOVLW	.32
	SUBWF	Param79,W
	SKPZ
	GOTO	I2C_DataInturp_L1
	MOVLB	0x00
	RETURN
;
;==============================================================
;
I2C_DataSender	BTFSC	I2C_TXLocked
	RETURN
;
	CLRF	Param78
;	BTFSS	SW1BtnBit
;	BSF	Param78,0
;
	CLRF	Param79	;offset
	LOADFSR0	I2C_ARRAY_TX,Param79
	MOVF	Param78,W
	MOVWF	INDF0
;
	RETURN
;
;=========================================================================================
;=========================================================================================
;
;
;
;
;
	END
;
