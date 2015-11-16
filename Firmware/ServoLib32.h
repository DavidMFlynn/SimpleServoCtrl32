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
;====================================================================================================
; Constants:
;
DefaultMaxSpeed	EQU	0x30	;1uS/20mS^2
DefaultAccel	EQU	0x01
DefaultSFlags	EQU	b'00100010'	;In Position
;
CCPCON_Clr	EQU	b'00001001'	;Clear output on match
CCPCON_Set	EQU	b'00001000'	;Set output on match
CCPCON_Int	EQU	b'00001010'	;Interupt only on match
;
kCenterPulseWidth	EQU	d'3000'	;1500uS
kMinPulseWidth	EQU	d'1800'	;900uS
kMaxPulseWidth	EQU	d'4200'	;2100uS
kServoDwellTimeA	EQU	d'4500'	;Address Change time 2250uS
kServoAddrTime	EQU	d'500'	;250uS
kServoDwellTime	EQU	d'5000'	;2.5mS/Channel
;
;================================================================================================
;  Bank3 Ram 1A0h-1EFh 80 Bytes
;
Bank3_Vars	udata	0x1A0
ServoMaxSpeed0_7	res	.8	;0=no Accel, 1..255 counts/20mS
ServoMaxSpeed8_15	res	.8
ServoAccelValue0_7	res	.8	;1..8 counts/20mS squared
ServoAccelValue8_15	res	.8
ServoCurSpeed0_7	res	.8	;0=Stopped, MSb=Direction, 1..127
ServoCurSpeed8_15	res	.8
;
;================================================================================================
;  Bank4 Ram 220h-26Fh 80 Bytes
;
Bank4_Vars	udata	0x220
CMDServoIDX	res	1	;0..15, used by IdleServos
CMDSigTime0_7	res	.16	;Commanded position
CMDSigTime8_15	res	.16
MinTime0_7	res	.16	;Minimum pulse time (900uS=1800)
MinTime8_15	res	.16	;Minimum pulse time (900uS=1800)
;
;================================================================================================
;  Bank5 Ram 2A0h-2EFh 80 Bytes
;
Bank5_Vars	udata	0x2A0
ServoIDX	res	1	;Index 0..7
ServoCtlFlags	res	1
; Flag bits, ToDo at Next ISR:
CyclePulseStart	EQU	0	;Start cycle banks 1 and 2
CyclePulseEnd1	EQU	2	;End pulse, begin dwell
CyclePulseEnd2	EQU	3	; Set when a pulse is started.
AddrChngDwell	EQU	4	;dwell 100uS , change address
;
ServoFlags	res	.8	;4 bits per servo
; Flag bits
ValueSentFlag0_7	EQU	0
InPositionFlag0_7	EQU	1
ServoOnBit0_7	EQU	2
MovingFWD0_7	EQU	3
ValueSentFlag8_15	EQU	4
InPositionFlag8_15	EQU	5
ServoOnBit8_15	EQU	6
MovingFWD8_15	EQU	7
;
CalcdDwell	res	1	;scratch var
CalcdDwellH	res	1
SigOutTime0_7	res	.16	;Current position
SigOutTime8_15	res	.16
DwellTime0_7	res	.16	;Next dwell time
DwellTime8_15	res	.16
;
;================================================================================================
;  Bank6 Ram 320h-26Fh 80 Bytes
;
Bank6_Vars	udata	0x320
ServoFlags2	res	.8	;4 bits per servo
; Flag bits
AccelComplete0_7	EQU	0
AccelComplete8_15	EQU	4
MaxTime0_7	res	.16	;Maximum pulse time (2100uS=4200)
MaxTime8_15	res	.16	;Maximum pulse time (2100uS=4200)
AccelRampLen0_7	res	.16
AccelRampLen8_15	res	.16
;
;=========================================================================================
;