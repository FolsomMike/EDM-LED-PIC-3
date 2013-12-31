;--------------------------------------------------------------------------------------------------
; Project:  OPT EDM Notch Cutter -- LED PIC software
; Date:     12/25/13
; Revision: See Revision History notes below.
;
;
; Overview:
;
; This program provides the notch cutting current control pulse and drives two LED arrays as
; voltage and current monitors for inputs on two A/D convertor channels.
; 
;--------------------------------------------------------------------------------------------------
; Notes on PCLATH
;
; The program counter (PC) is 13 bits. The lower 8 bits can be read and written as register PCL.
; The upper bits cannot be directly read or written.
;
; When the PCL register is written, PCLATH<4:0> is copied at the same time to the upper 5 bits of
; PC.
;
; When a goto is executed, 11 bits embedded into the goto instruction are loaded into the PC<10:0>
; while bits PCLATH<4:3> are copied to bits PC<12:11>
;
; Changing PCLATH does NOT instantly change the PC register. The PCLATH will be used the next time
; a goto is executed (or similar opcode) or the PCL register is written to. Thus, to jump farther
; than the 11 bits (2047 bytes) in the goto opcode will allow, the PCLATH register is adjusted
; first and then the goto executed.
;
;--------------------------------------------------------------------------------------------------
;
; Revision History:
;
; 1.0   Code copied from "OPT EDM Main PIC 2" project.
;
;
;--------------------------------------------------------------------------------------------------
; Miscellaneous Notes
;
; incf vs decf rollover
;
; When incrementing multi-byte values, incf can be used because it sets the Z flag - then the Z
; flag is set, the next byte up should then be incremented.
; When decrementing multi-byte values, decf CANNOT be used because it sets the Z flag but NOT the
; C flag.  The next byte up is not decremented when the lower byte reaches zero, but when it rolls
; under zero.  This can be caught by loading w with 1 and then using subwf and catching the C flag
; cleared. (C flag is set for a roll-over with addwf, cleared for roll-under for subwf.
; For a quickie but not perfect count down of a two byte variable, decf and the Z flag can be used
; but the upper byte will be decremented one count too early.
;
;--------------------------------------------------------------------------------------------------
; Operational Notes
;
;
;--------------------------------------------------------------------------------------------------
; Hardware Control Description
;
; Function by Pin
;
; Port A
;
; RA0   In  - 
; RA1   In  - 
; RA2   xxx - 
; RA3   In  - 
; RA4   In  - 
; RA5   Out - 
; RA6   xxx - 
; RA7   xxx - 
;
; Port B
;
; RB0   xxx - 
; RB1   xxx - 
; RB2   xxx - 
; RB3   xxx - 
; RB4   I/O - 
; RB5   In  - 
; RB6   Out - 
; RB7   Out - 
;
; Port C
;
; RC0   Out - LED0
; RC1   Out - LED1
; RC2   Out - LED2
; RC3   Out - LED3
; RC4   Out - LED4
; RC5   Out - PWM out ~ cutting current pulse control
; RC6   Out - Current LED Display latch signal
; RC7   Out  - Current LED Display latch signal
;
;end of Hardware Control Description
;--------------------------------------------------------------------------------------------------
;
; User Inputs
;
;
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Defines
;

; COMMENT OUT "#define debug" line before using code in system.
; Defining debug will insert code which simplifies simulation by skipping code which waits on
; stimulus and performing various other actions which make the simulation run properly.
; Search for "ifdef debug" to find all examples of such code.

;#define debug 1     ; set debug testing "on"


; Cutting Current Pulse Controller Values (cycle width and duty cycle)
;
; with Timer 2 prescaler set to 4, each count of PWM_PERIOD equals 1uS of period

PWM_PERIOD EQU  .218

; debug mks -- needs to be high byte and low byte and then shift right to place in the
; timer 2 reload register

PWM_DUTY_CYLE_HIGH_BITS     EQU     0x2c
PWM_DUTY_CYLE_LOW_BITS      EQU     0x80


; end of Defines
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Configurations, etc. for the Assembler Tools and the PIC

;	LIST p = PIC16F648a	;select the processor

    errorlevel  -306 ; Suppresses Message[306] Crossing page boundary -- ensure page bits are set.

    errorLevel  -302 ; Suppresses Message[302] Register in operand not in bank 0.

	errorLevel	-202 ; Suppresses Message[205] Argument out of range. Least significant bits used.
					 ;	(this is displayed when a RAM address above bank 1 is used -- it is
					 ;	 expected that the lower bits will be used as the lower address bits)

#INCLUDE <p16f1459.inc> 		; Microchip Device Header File


;#include <xc.h>

#INCLUDE <STANDARD 2.MAC>     	; include standard macros

; Specify Device Configuration Bits

; CONFIG1
; __config 0xF9E4
 __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _WRT_ALL & _CPUDIV_NOCLKDIV & _USBLSCLK_48MHz & _PLLMULT_4x & _PLLEN_DISABLED & _STVREN_ON & _BORV_LO & _LPBOR_OFF & _LVP_OFF

; _FOSC_INTOSC -> internal oscillator, I/O function on CLKIN pin
; _WDTE_OFF -> watch dog timer disabled
; _PWRTE_OFF -> Power Up Timer disabled
; _MCLRE_OFF -> MCLR/VPP pin is digital input
; _CP_OFF -> Flash Program Memory Code Protection off
; _BOREN_OFF -> Power Brown-out Reset off
; _CLKOUTEN_OFF -> CLKOUT function off, I/O or oscillator function on CLKOUT pin
; _IESO_OFF -> Internal/External Oscillator Switchover off
;   (not used for this application since there is no external clock)
; _FCMEN_OFF -> Fail-Safe Clock Monitor off
;   (not used for this application since there is no external clock)
; _WRT_ALL -> Flash Memory Self-Write Protection on -- no writing to flash
;
; _CPUDIV_NOCLKDIV -> CPU clock not divided
; _USBLSCLK_48MHz -> only used for USB operation
; _PLLMULT_4x -> sets PLL (if enabled) multiplier -- 4x allows software override
; _PLLEN_DISABLED -> the clock frequency multiplier is not used
;
; _STVREN_ON -> Stack Overflow/Underflow Reset on
; _BORV_LO -> Brown-out Reset Voltage Selection -- low trip point
; _LPBOR_OFF -> Low-Power Brown-out Reset Off
; _LVP_OFF -> Low Voltage Programming off
;
; end of configurations
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Hardware Definitions

; Port A

;SERIAL_IN_P     EQU     PORTA
;SERIAL_IN       EQU     RA0         ; input ~ RA0 can only be input on PIC16f1459

; Port B

;I2CSDA_LINE     EQU     RB4
;JOG_DWN_SW_P    EQU     PORTB

; Port C

LED0_P              EQU     LATC
LED0                EQU     RC0
LED1_P              EQU     LATC
LED1                EQU     RC1
LED2_P              EQU     LATC
LED2                EQU     RC2
LED3_P              EQU     LATC
LED3                EQU     RC3
LED4_P              EQU     LATC
LED4                EQU     RC4

CURRENT_LED_LATCH_P EQU     LATC
CURRENT_LED_LATCH   EQU     RC6

VOLTAGE_LED_LATCH_P EQU     LATC
VOLTAGE_LED_LATCH   EQU     RC7


; end of Hardware Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Software Definitions

; bits in flags variable

;EXTENDED_MODE   EQU     0x0
;CUT_STARTED     EQU     0x1

; end of Software Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Variables in RAM
;
; Note that you cannot use a lot of the data definition directives for RAM space (such as DB)
; unless you are compiling object files and using a linker command file.  The cblock directive is
; commonly used to reserve RAM space or Code space when producing "absolute" code, as is done here.
; 

; Assign variables in RAM - Bank 0 - must set BSR to 0 to access
; Bank 0 has 80 bytes of free space

 cblock 0x20                ; starting address

    flags                   ; bit 0: 0 = ?? 1 = ??
                            ; bit 1: 0 = 
                            ; bit 2: 0 = 
                            ; bit 3: 0 = 
                            ; bit 4: 0 = 
                            ; bit 5: 0 = 
							; bit 6: 0 = 
							; bit 7: 0 = 

    scratch0                ; these can be used by any function
    scratch1
    scratch2
    scratch3
    scratch4
    scratch5
    scratch6
    scratch7
    scratch8
    scratch9
    scratch10

 endc

;-----------------

; Assign variables in RAM - Bank 1 - must set BSR to 1 to access
; Bank 1 has 80 bytes of free space

 cblock 0xa0                ; starting address


 endc

;-----------------

; Assign variables in RAM - Bank 3 - must set BSR to 3 to access
; Bank 2 has 80 bytes of free space

 cblock 0x120                ; starting address

	block1PlaceHolder

 endc

;-----------------
 
; Define variables in the memory which is mirrored in all 4 RAM banks.  This area is usually used
; by the interrupt routine for saving register states because there is no need to worry about
; which bank is current when the interrupt is invoked.
; On the PIC16F628A, 0x70 thru 0x7f is mirrored in all 4 RAM banks.

; NOTE:
; This block cannot be used in ANY bank other than by the interrupt routine.
; The mirrored sections:
;
;	Bank 0		Bank 1		Bank 2		Bank3
;	70h-7fh		f0h-ffh		170h-17fh	1f0h-1ffh
;

 cblock	0x70
    W_TEMP
    FSR0L_TEMP
    FSR0H_TEMP
    STATUS_TEMP
    PCLATH_TEMP	
 endc

; end of Variables in RAM
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Power On and Reset Vectors
;

	org 0x00                ; Start of Program Memory

	goto start              ; jump to main code section
	nop			            ; Pad out so interrupt
	nop			            ; service routine gets
	nop			            ; put at address 0x0004.


;wip -- PIC16f1459 now saves important registers automatically on interrupt
; keep PUSH and POP macros for possible future use -- just empty them for now

; interrupt vector at 0x0004

    goto 	handleInterrupt	; points to interrupt service routine

; end of Reset Vectors
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; start
;

start:

    call    setup           ; preset variables and configure hardware

mainLoop:

    bcf     LED3_P,LED3
    nop
    nop
    nop
    bsf     LED3_P,LED3
    nop
    nop
    nop

    goto    mainLoop
    
; end of start
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setup
;
; Presets variables and configures hardware.
;

setup:

    call    setupClock      ; set system clock source and frequency

    call    setupPortA      ; prepare Port A for I/O

    call    setupPortB      ; prepare Port B for I/O

    call    setupPortC      ; prepare Port C  for I/O

    call    initializeOutputs

    call    setupCuttingCurrentPWM

;start of hardware configuration

    clrf   FSR0H            ;high byte of indirect addressing pointers -> 0
    clrf   FSR1H

    clrf    INTCON          ; disable all interrupts

    banksel OPTION_REG
    movlw   0x58
    movwf   OPTION_REG      ; Option Register = 0x58   0101 1000 b
                            ; bit 7 = 0 : weak pull-ups are enabled by individual port latch values
                            ; bit 6 = 1 : interrupt on rising edge
                            ; bit 5 = 0 : TOCS ~ Timer 0 run by internal instruction cycle clock (CLKOUT ~ Fosc/4)
                            ; bit 4 = 1 : TOSE ~ Timer 0 increment on high-to-low transition on RA4/T0CKI/CMP2 pin (not used here)
							; bit 3 = 1 : PSA ~ Prescaler assigned to WatchDog; Timer0 will be 1:1 with Fosc/4
                            ; bit 2 = 0 : Bits 2:0 control prescaler:
                            ; bit 1 = 0 :    000 = 1:2 scaling for Timer0 (if assigned to Timer0)
                            ; bit 0 = 0 :
    
;end of hardware configuration

    banksel flags

    clrf    flags
	
; enable the interrupts

;	bsf	    INTCON,PEIE	    ; enable peripheral interrupts (Timer0 is a peripheral)
;    bsf     INTCON,T0IE     ; enable TMR0 interrupts
;    bsf     INTCON,GIE      ; enable all interrupts

    return

; end of setup
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; initializeOutputs
;
; Initializes all outputs to known values.
;

initializeOutputs:

    banksel LATC

    ; when LATCH ENABLE input is HIGH, the Q outputs
    ; will follow the D inputs. When the LATCH ENABLE goes
    ; LOW, data at the D inputs will be retained

    ; latch high (outputs follow inputs)

    bsf     CURRENT_LED_LATCH_P, CURRENT_LED_LATCH
    bsf     VOLTAGE_LED_LATCH_P, VOLTAGE_LED_LATCH

    ; turn on lower LEDs

    bcf     LED0_P,LED0
    bsf     LED1_P,LED1
    bsf     LED2_P,LED2
    bsf     LED3_P,LED3
    bsf     LED4_P,LED4

    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop


    ; latches low to lock in data

    bcf     CURRENT_LED_LATCH_P, CURRENT_LED_LATCH
    bcf     VOLTAGE_LED_LATCH_P, VOLTAGE_LED_LATCH

    return

; end of initializeOutpus
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupClock
;
; Saves the flags value to eeprom.
;
; Sets up the system clock source and frequency.
;
; Assumes clock related configuration bits are set as follows:
;
;   _FOSC_INTOSC,  _CPUDIV_NOCLKDIV, _PLLMULT_4x, _PLLEN_DISABLED
;
; Assumes all programmable clock related options are at Reset default values.
;

setupClock:

    ; choose internal clock frequency of 16 Mhz

    banksel OSCCON

    bsf     OSCCON, IRCF0
    bsf     OSCCON, IRCF1
    bsf     OSCCON, IRCF2
    bsf     OSCCON, IRCF3

    return

; end of setupClock
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupCuttingCurrentPWM
;
; Sets up the Pulse Width Modulator output to control the cutting current pulse.
;
;

setupCuttingCurrentPWM:

    banksel PWM1CON
    clrf    PWM1CON

    banksel PR2             ; period of the cycle
    movlw   PWM_PERIOD
    movwf   PR2
       
    ; upper byte plus lower two bits = 46 decimal

    banksel PWM1DCH
    movlw   PWM_DUTY_CYLE_HIGH_BITS
    movwf   PWM1DCH

    banksel PWM1DCL
    movlw   PWM_DUTY_CYLE_LOW_BITS
    movwf   PWM1DCL

    banksel PIR1
    bcf     PIR1, TMR2IF

    ;T2CON:T2CKPS<1:0>: Timer2 Clock Prescale Select bits
    ;    11 = Prescaler is 64
    ;    10 = Prescaler is 16
    ;    01 = Prescaler is 4
    ;    00 = Prescaler is 1

    banksel T2CON                   ; use prescaler of 4
    bsf     T2CON, T2CKPS0
    bcf     T2CON, T2CKPS1

    bsf     T2CON, TMR2ON

    banksel PWM1CON
    bsf     PWM1CON, PWM1EN         ; enable PWM module
    bsf     PWM1CON, PWM1OE         ; enable PWM 1 output pin (RC5)

    banksel TRISC
    bcf     TRISC, TRISC5

    return

; end of setupCuttingCurrentPWM
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortA
;
; Sets up Port A for I/O operation.
;
; NOTE: Writing to PORTA is same as writing to LATA for PIC16f1459. The code example from the
; data manual writes to both -- probably to be compatible with other PIC chips.
;
; NOTE: RA0, RA1 and RA3 can only be inputs on the PIC16f1459 device. 
;       RA2, RA6, RA7 are not implemented.
;

setupPortA:

    banksel WPUA
    movlw   b'00000000'                 ; disable weak pull-ups
    movwf   WPUA

    banksel PORTA
    clrf    PORTA                       ; init port value

    banksel LATA                        ; init port data latch
    clrf    LATA

    banksel ANSELA
    clrf    ANSELA                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISA
    movlw   b'11111111'                 ; first set all to inputs
    movwf   TRISA

    ; set direction for each pin used

    ;bsf     TRISA, ???                  ; input
    ;bcf     TRISA, ???                  ; output

    return

; end of setupPortA
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortB
;
; Sets up Port B for I/O operation.
;
; NOTE: Writing to PORTB is same as writing to LATB for PIC16f1459. The code example from the
; data manual writes to both -- probably to be compatible with other PIC chips.
;
; NOTE: RB0, RB1, RB2, RB3 are not implemented on the PIC16f1459 device.
;

setupPortB:

    banksel WPUB
    movlw   b'00000000'                 ; disable weak pull-ups
    movwf   WPUB

    banksel PORTB
    clrf    PORTB                       ; init port value

    banksel LATB                        ; init port data latch
    clrf    LATB

    banksel ANSELB
    clrf    ANSELB                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISB
    movlw   b'11111111'                 ; first set all to inputs
    movwf   TRISB

    ;bsf     TRISB, ???           ; input
    ;bcf     TRISB, ???           ; output

    return

; end of setupPortB
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupPortC
;
; Sets up Port C for I/O operation.
;
; NOTE: Writing to PORTC is same as writing to LATC for PIC16f1459. The code example from the
; data manual writes to both -- probably to be compatible with other PIC chips.
;

setupPortC:

    ; Port C does not have a weak pull-up register

    banksel PORTC                       ; init port value
    clrf    PORTC

    banksel LATC                        ; init port data latch
    clrf    LATC

    banksel ANSELC
    clrf    ANSELC                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISC
    movlw   b'11111111'                 ; first set all to inputs
    movwf   TRISC

    bcf     TRISC, LED0                 ; output
    bcf     TRISC, LED1                 ; output
    bcf     TRISC, LED2                 ; output
    bcf     TRISC, LED3                 ; output
    bcf     TRISC, LED4                 ; output
    bcf     TRISC, CURRENT_LED_LATCH    ; output
    bcf     TRISC, VOLTAGE_LED_LATCH    ; output

    return

; end of setupPortC
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleInterrupt
;
; All interrupts call this function.  The interrupt flags must be polled to determine which
; interrupts actually need servicing.
;
; Note that after each interrupt type is handled, the interrupt handler returns without checking
; for other types.  If another type has been set, then it will immediately force a new call
; to the interrupt handler so that it will be handled.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 8 deep and
; it is very bad for the interrupt routine to use it.
;

handleInterrupt:

	btfsc 	INTCON,T0IF     		; Timer0 overflow interrupt?
	goto 	handleTimer0Interrupt	; YES, so process Timer0
           
; Not used at this time to make interrupt handler as small as possible.
;	btfsc 	INTCON, RBIF      		; NO, Change on PORTB interrupt?
;	goto 	portB_interrupt       	; YES, Do PortB Change thing

INT_ERROR_LP1:		        		; NO, do error recovery
	;GOTO INT_ERROR_LP1      		; This is the trap if you enter the ISR
                               		; but there were no expected interrupts

endISR:

	retfie                  	; Return and enable interrupts

; end of handleInterrupt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleTimer0Interrupt
;
; This function is called when the Timer0 register overflows.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 8 deep and
; it is very bad for the interrupt routine to use it.
;

handleTimer0Interrupt:

	bcf 	INTCON,T0IF     ; clear the Timer0 overflow interrupt flag


    goto    endISR

; end of handleTimer0Interrupt
;--------------------------------------------------------------------------------------------------

    END
