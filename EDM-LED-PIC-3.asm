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
; with oscillator frequency of 16 Mhz and Timer 2 prescaler set to 4:
;
; each count of PWM_PERIOD_DEFAULT equals 1uS of period with an offset of +1
;
;    period uS = (PWM_PERIOD_DEFAULT + 1) * 1 uS
;
; each count of PWM_DUTY_CYCLE_LO_BYTE_DEFAULT:PWM_DUTY_CYCLE_LO_BYTE_DEFAULT equals 0.25 uS
;  for the high pulse width -- note that signal is inverted before it reaches testpoing J23 so this
;  is actually a low pulse at that point
;
;   pulse width uS = PWM_DUTY_CYCLE_LO_BYTE_DEFAULT:PWM_DUTY_CYCLE_LO_BYTE_DEFAULT * 0.25 uS
;

I2C_SLAVE_ADDR                      EQU     b'10100100'

PWM_PERIOD_DEFAULT                  EQU  .217       ; gives 218 uS PWM period
                                                    ; period uS = (PWM_PERIOD + 1) * 1 uS

PWM_DUTY_CYCLE_HI_BYTE_DEFAULT      EQU     0x00    ; 00:b8 gives 46 uS PWM high pulse width
PWM_DUTY_CYCLE_LO_BYTE_DEFAULT      EQU     0xb8    ; (high:lo) * 0.25 uS = high pulse width
                                                    ; note that signal is inverted befor it reaches
                                                    ; testpoint J23

PWM_POLARITY_DEFAULT                EQU 0x00

; LED PIC Commands

LEDPIC_SET_LEDS                 EQU 0x00    ; sets the on/off states of the LED arrays
LEDPIC_SET_PWM                  EQU 0x01    ; sets the PWM values
LEDPIC_START                    EQU 0x02    ; starts normal operation
LEDPIC_SET_RESET                EQU 0xff    ; resets to a known state

; end of Defines
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Configurations, etc. for the Assembler Tools and the PIC

	LIST p = PIC16F1459	;select the processor

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

CURRENT_MONITOR_INPUT   EQU     RA4

; Port B

VOLTAGE_MONITOR_INPUT   EQU     RB5

; pin used for debugging -- sometimes used as an output or input
DEBUG_IO_P      EQU     LATB
DEBUG_IO        EQU     RB7

; Port C
;
; NOTE: For all write operations, the port's latch is written to in order to avoid
; read-modify-write issues sometimes caused by writing to the port's pins.

LEDS                EQU     LATC

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

HANDLE_LED_ARRAYS   EQU     0x00     ; if set, A/D inputs are displayed on the LED arrays
CURRENT_VOLTAGE_AD  EQU     0x01     ; specifies which input is to be converted next

; values for A/D register ADCON0 to select channels and turn A/D on

; Current Monitor input channel/pin: AN3/RA4
SAMPLE_CURRENT_INPUT    EQU     b'00001101'

; Voltage Monitor input channel/pin: AN11/RB5
SAMPLE_VOLTAGE_INPUT    EQU     b'00101101'

; Specify the subrange increment for each additional LED displayed in the arrays.
; For example, if voltage input of 3.8V is to be full scale, this equates to
; A/D value of 194 decimal when Vref if +5V.
; Dividing 194 by 6 (5 possible LEDs on plus case of no LEDs on) gives
; increment of 32.3 rounded down to 32.

CURRENT_RANGE_INCREMENT    EQU     .32
VOLTAGE_RANGE_INCREMENT    EQU     .32

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

    flags                   ; bit 0: 0 = skip code; 1 = display A/D inputs on LED arrays
                            ; bit 1: 0 = current A/D input; 1 = voltage A/D input
                            ; bit 2: 0 = 
                            ; bit 3: 0 = 
                            ; bit 4: 0 = 
                            ; bit 5: 0 = 
							; bit 6: 0 = 
							; bit 7: 0 = 

    adTrigger               ; 0 = not ready; 1 = handle A/D input
                            ; set by Timer0 interrupt to trigger the next A/D conversion
                            ; cleared by handleADToLEDArrays
                            ; an entire byte is used to avoid read-modify-write issues between the
                            ; main thread and the interrupt thread

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

    call    setup               ; preset variables and configure hardware

    call    flashLEDsUntilMasterPICConnects

mainLoop:

    call    handleI2CCommand    ; checks for incoming command on I2C bus

    banksel flags

    btfsc   flags,HANDLE_LED_ARRAYS ; display A/D inputs on LED arrays
    call    handleADToLEDArrays

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

    call    setupI2CSlave7BitMode ; prepare the I2C serial bus for use

    call    setupADConverter ; prepare A/D converter for use

    call    setupCuttingCurrentPWM

;start of hardware configuration

    clrf   FSR0H            ;high byte of indirect addressing pointers -> 0
    clrf   FSR1H

    clrf    INTCON          ; disable all interrupts

    banksel OPTION_REG
    movlw   0x57
    movwf   OPTION_REG      ; Option Register = 0x57   0101 0111 b
                            ; bit 7 = 0 : weak pull-ups are enabled by individual port latch values
                            ; bit 6 = 1 : interrupt on rising edge
                            ; bit 5 = 0 : TOCS ~ Timer 0 run by internal instruction cycle clock (CLKOUT ~ Fosc/4)
                            ; bit 4 = 1 : TOSE ~ Timer 0 increment on high-to-low transition on RA4/T0CKI/CMP2 pin (not applicable here)
							; bit 3 = 0 : PSA ~ Prescaler assigned to Timer0
                            ; bit 2 = 1 : Bits 2:0 control prescaler:
                            ; bit 1 = 1 :    111 = 1:256 scaling for Timer0 (if assigned to Timer0)
                            ; bit 0 = 1 :
    
;end of hardware configuration

    banksel flags

    clrf    flags
	
; enable the interrupts

    bsf     INTCON,PEIE     ; enable peripheral interrupts (Timer0 is a peripheral)
    bsf     INTCON,T0IE     ; enable TMR0 interrupts
    bsf     INTCON,GIE      ; enable all interrupts

    return

; end of setup
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; initializeOutputs
;
; Initializes all outputs to known values.
;

initializeOutputs:

    ;turn off all LEDs

    movlw   0xff
    call    setCurrentLEDArray

    movlw   0xff
    call    setVoltageLEDArray

    return

; end of initializeOutpus
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupClock
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

setupCuttingCurrentPWM:

    banksel PWM1CON
    clrf    PWM1CON

    banksel PR2             ; period of the cycle
    movlw   PWM_PERIOD_DEFAULT
    movwf   PR2

    call    setPWM1DCToDefaults     ; set value of PWM1DCH and PWM1DCH duty cycle time registers

    banksel PIR1
    bcf     PIR1, TMR2IF

    ;T2CON:T2CKPS<1:0>: Timer2 Clock Prescale Select bits
    ;    11 = Prescaler is 64
    ;    10 = Prescaler is 16
    ;    01 = Prescaler is 4
    ;    00 = Prescaler is 1

    banksel T2CON                   ; use prescaler of 4 for Timer 2
    bsf     T2CON, T2CKPS0
    bcf     T2CON, T2CKPS1

    bsf     T2CON, TMR2ON           ; turn on Timer 2

    banksel PWM1CON
    bsf     PWM1CON, PWM1EN         ; enable PWM module
    bsf     PWM1CON, PWM1OE         ; enable PWM 1 output pin (RC5)

    banksel TRISC                   ; make Port C bit 5 an output -- this is the PWM pulse
    bcf     TRISC, TRISC5

    return

; end of setupCuttingCurrentPWM
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setPWM1DCToDefaults
;
; Sets the value of the PWM module's duty cycle time registers to default values.
;

setPWM1DCToDefaults:

    banksel scratch0

    movlw   PWM_DUTY_CYCLE_LO_BYTE_DEFAULT
    movwf   scratch1

    movlw   PWM_DUTY_CYCLE_HI_BYTE_DEFAULT
    movwf   scratch2

    goto    setPWM1DC

; end of setPWM1DCToDefaults
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setPWM1DC
;
; Sets the value of the PWM module's duty cycle time registers.
;
; PWM1DCH<7:0> are the upper 8 upper bits of the value and PWM1DCL<7:6> are the 2 lower bits. The
; value to be stored is in the more common format of high byte and low byte, with the high byte
; only having the two lower bits applicable. The value is rotated to place the two lower bits
; in a byte and the upper 8 bits in another byte ready for the PWM1DC registers.
;
; original value in two bytes:      xx xxxxxxxx
; modified value in two bytes:      xxxxxxxx xx
;

setPWM1DC:

    clrf    scratch0

    ; shift value right by two bits so lower two bits end up in upper bits of scratch0
    ;       original value:         scratch2:scratch1
    ;                                     xx:xxxxxxxx
    ;       shifted value:          scratch1:scratch0
    ;                               xxxxxxxx:xx

    bcf     STATUS,C

    rrf     scratch2,F          ; rotate first time through all three bytes
    rrf     scratch1,F
    rrf     scratch0,F

    bcf     STATUS,C

    rrf     scratch2,F          ; rotate second time through all three bytes
    rrf     scratch1,F
    rrf     scratch0,F

    banksel scratch0            ; lower 2 bits -> PWM1DCL
    movf    scratch0,W
    banksel PWM1DCL
    movwf   PWM1DCL

    banksel scratch1            ; upper 8 bits -> PWM1DCH
    movf    scratch1,W
    banksel PWM1DCH
    movwf   PWM1DCH

    return

; end of setPWM1DC
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

    bcf     TRISB,DEBUG_IO              ; debug pin

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
; setupI2CSlave7BitMode
;
; Sets the MASTER SYNCHRONOUS SERIAL PORT (MSSP) MODULE to the I2C Slave mode using the 7 bit
; address mode.
;
; NOTE: RB4 and RB6 must have been configured elswhere as inputs for this mode.
;

setupI2CSlave7BitMode:

    movlw   I2C_SLAVE_ADDR  ; set the I2C slave device address
    banksel SSPADD
    movwf   SSPADD
    
    movlw   0xff            ; bits <7:1> of SSPADD are used to match address
    banksel SSPMSK          ; (bit <0> is ignored in 7 bit address mode)
    movwf   SSPMSK

    banksel SSPCON2         ; enable clock stretching -- upon receiving a byte this PIC will
    bsf     SSPCON2,SEN     ; hold clock and halt further transmissions until CKP bit cleared

    banksel SSPCON1
    bcf     SSPCON1,SSP1M0		; SSPM = b0110 ~ I2C Slave mode
    bsf     SSPCON1,SSP1M1
    bsf     SSPCON1,SSP1M2
    bcf     SSPCON1,SSP1M3

    bsf	SSPCON1,SSPEN		;enables the MSSP module

    return

; end setupI2CSlave7BitMode
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupADConverter
;
; Sets up the A/D converter to convert Current and Voltage monitor inputs so they can be
; represented on the LED arrays.
;

setupADConverter:

    ;This code block configures the ADC for polling, Vdd and Vss references,
    ; FOSC/16 clock

    ; configure the A/D converter
    ; bit 7 = 0 : left justify the result in ADRESH:ADRESL
    ; bit 6 = 1 : bits 6-4 : A/D Conversion Clock Select bits
    ; bit 5 = 0 :    101 -> FOSC/16
    ; bit 4 = 1 : 
    ; bit 3 = 0 : unused
    ; bit 2 = 0 : unused
    ; bit 1 = 0 : bits 1-0: A/D voltage reference source
    ; bit 0 = 0 :    00 -> VREF+ connected to VDD

    banksel ADCON1
    movlw   b'01010000'         ;left justify, FOSC/16 clock
    movwf   ADCON1              ;Vdd is Vref+

    ; Current Monitor input channel/pin: AN3/RA4

    banksel TRISA
    bsf     TRISA,CURRENT_MONITOR_INPUT     ;set I/O pin to input

    banksel ANSELA
    bsf     ANSELA,CURRENT_MONITOR_INPUT    ;set I/O pin to analog

    ; Voltage Monitor input channel/pin: AN11/RB5

    banksel TRISB
    bsf     TRISB,VOLTAGE_MONITOR_INPUT     ;set I/O pin to input

    banksel ANSELB
    bsf     ANSELB,VOLTAGE_MONITOR_INPUT    ;set I/O pin to analog

    ; turn on A/D module and begin sampling Current Monitor input

    banksel ADCON0
    movlw   SAMPLE_CURRENT_INPUT
    movwf   ADCON0

    return

; end of setupADConverter
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleADToLEDArrays
;
; Reads the A/D inputs for the current and voltage levels and displays them on the appropriate
; LED arrays.
;
; If the adTrigger flag has not been set by a Timerx interrupt, function returns immediately.
; If flag is set, it is cleared and the A/D processed.
;
; Each time the A/D is processed, it is for a different input channel / LED array:
;  One pass handles the Current Monitor input and drives the Current LED array.
;  The next pass handles the Voltage Monitor input and drives the Voltage LED array.
;  The cycle repeats.
;
; Flag CURRENT_VOLTAGE_AD in flags determines which input is to be processed:
;   0: current, 1: voltage
;
; If Current input is handled on a pass, then the A/D is set up to start sampling Voltage input
; before exit and vice versa. This ensures the sampling circuit has ample time to settle before
; the next call.
;
; The frequency of the adTrigger flag being set should be slow enough to allow the sampling circuit
; to settle between processing.
;

handleADToLEDArrays:

    ; do nothing until adTrigger bit 0 is set

    banksel adTrigger
    btfss   adTrigger,0
    return

    clrf    adTrigger           ; clear flag so interrupt routine can set it

    ; if flag is 0, handle Current input else handle voltage input

    banksel flags

    btfss   flags,CURRENT_VOLTAGE_AD
    goto    handleInvVoltageToCurrentLED

    goto    handleVoltageADToLED

    return

; end of handleADToLEDArrays
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleCurrentADToLED
;
; Converts the Current Monitor input voltage and represents its value on the Current LED Array.
;
; Sets up the A/D to begin sampling the Voltage input before exiting so it will be ready for
; conversion on the next trigger.
;

handleCurrentADToLED:

    bsf     flags,CURRENT_VOLTAGE_AD    ; handle Voltage input next time

    banksel ADCON0
    bsf     ADCON0,ADGO                 ;start conversion

hcatl1:
    btfsc   ADCON0,ADGO                 ;loop until conversion done
    goto    hcatl1

    ; turn on A/D module and begin sampling Voltage Monitor input so it will be
    ; ready to convert on next call

    banksel ADCON0
    movlw   SAMPLE_VOLTAGE_INPUT
    movwf   ADCON0

   ; set the Current Array LEDs to reflect the value

    goto    setCurrentLEDArrayFromADValue

; end of handleCurrentADToLED
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleVoltageADToLED
;
; Converts the Voltage Monitor input voltage and represents its value on the Voltage LED Array.
;
; Sets up the A/D to begin sampling the Current input before exiting so it will be ready for
; conversion on the next trigger.
;

handleVoltageADToLED:

    bcf     flags,CURRENT_VOLTAGE_AD    ; handle Current input next time

    banksel ADCON0
    bsf     ADCON0,ADGO                 ;start conversion

hvatl1:
    btfsc   ADCON0,ADGO                 ;loop until conversion done
    goto    hvatl1

    ; turn on A/D module and begin sampling Current Monitor input so it will be
    ; ready to convert on next call

    banksel ADCON0
    movlw   SAMPLE_CURRENT_INPUT
    movwf   ADCON0

   ; set the Voltage Array LEDs to reflect the value

    goto    setVoltageLEDArrayFromADValue

; end of handleVoltageADToLED
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setCurrentLEDArrayFromADValue
;
; Sets the LEDs in the Current Monitor array to reflect the value in ADRESH.
; It is assumed that the A/D value is left justified, thus the upper 8 bits will be used
; and the lower 2 bits will be discarded.
;

setCurrentLEDArrayFromADValue:

    banksel ADRESH              ;read upper 8 bits of result
    movf    ADRESH,W            ; (result is left justified so this gets
                                ;  the upper 8 bits, ignoring the lsbs)

    banksel scratch0
    movwf   scratch0            ; store the A/D value

    ; compare the A/D value with different percentages of the max display value
    ; the total range is divided into 6 possible subranges to account for 5 LEDs on for max and
    ; 0 LEDs on for minimum
    ;
    ; all values rounded down:
    ; 0  - 16%  : 0 LEDs on
    ; 17 - 33%  : 1 LEDs on
    ; 34 - 50%  : 2 LEDs on
    ; 51 - 66%  : 3 LEDs on
    ; 67 - 83%  : 4 LEDs on
    ; 84 - 100% : 5 LEDs on
    ;
    ; sublw : k-(W)->(W)     C = 1 W <= k

    movf    scratch0,W
    sublw   (CURRENT_RANGE_INCREMENT * 1)
    btfss   STATUS,C                            ; C = 1 W <= k
    goto    criTest2

    movlw   0xff                                ; all LEDs off
    goto    setCurrentLEDArray

criTest2:

    movf    scratch0,W
    sublw   (CURRENT_RANGE_INCREMENT * 2)
    btfss   STATUS,C                            ; C = 1 W <= k
    goto    criTest3

    movlw   0xfe                                ; 1 LED(s) on
    goto    setCurrentLEDArray

criTest3:

    movf    scratch0,W
    sublw   (CURRENT_RANGE_INCREMENT * 3)
    btfss   STATUS,C                            ; C = 1 W <= k
    goto    criTest4

    movlw   0xfc                                ; 2 LED(s) on
    goto    setCurrentLEDArray

criTest4:

    movf    scratch0,W
    sublw   (CURRENT_RANGE_INCREMENT * 4)
    btfss   STATUS,C                            ; C = 1 W <= k
    goto    criTest5

    movlw   0xf8                                ; 3 LED(s) on
    goto    setCurrentLEDArray

criTest5:

    movf    scratch0,W
    sublw   (CURRENT_RANGE_INCREMENT * 5)
    btfss   STATUS,C                            ; C = 1 W <= k
    goto    criTest6

    movlw   0xf0                                ; 4 LED(s) on
    goto    setCurrentLEDArray

criTest6:

    movlw   0xe0                                ; 5 LED(s) on
    goto    setCurrentLEDArray

; end of setCurrentLEDArrayFromADValue
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleInvVoltageToCurrentLED
;
; Sets Current LED array to the inverse of the Voltage LED array.
; See setCurrentLEDArrayInverseToLATC for more details.
;
; Sets up the A/D to begin sampling the Voltage input before exiting so it will be ready for
; conversion on the next trigger.
;

handleInvVoltageToCurrentLED:

    bsf     flags,CURRENT_VOLTAGE_AD    ; handle Voltage input next time

    ; turn on A/D module and begin sampling Voltage Monitor input so it will be
    ; ready to convert on next call

    banksel ADCON0
    movlw   SAMPLE_VOLTAGE_INPUT
    movwf   ADCON0

    ; set Current LEDs to the opposite of the Voltage LEDs

    goto    setCurrentLEDArrayInverseToLATC

; end of handleInvVoltageToCurrentLED
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setCurrentLEDArrayInverseToLATC
;
; Sets the LEDs in the Current Monitor array to the opposite of whatever is currently
; in the LATC register.
;
; It is assumed that the LATC register contains the value previously written to the voltage
; LEDs. Whatever LEDs were turned on for that array will be turned off for the Current array and
; vice versa.
;
; This is used to mimic actually reading the current monitor voltage input. It is assumed that the
; cutting current flow will be inverse to the voltage level. This is done because the boards
; could not read very low current voltage input at the digital pot -- it had to be scaled low to
; protect the input from cases where the high voltage side was shorted to the electrode when the
; electrode was not being pulsed (due to bad MosFets, etc.). This would have driven the input to
; nearly 100 volts and blown up the digital pot. The very first boards were able to read the very
; voltage because they used a different chip to translate the voltage to LED display.
;
; UPDATE: The board might CAN read the values -- the lowest 2 bits are being discarded. Haven't
; changed the code to test this yet! It would only give 47 counts for max value, but might still
; work if noise isn't a problem. On the other hand, using the inverse value eliminates a
; calibration step required by the current input. Also, using the FVR as +ADRef with the FVR set to
; 1.024V would increase the counts to 200 -- VERY close to that already used for the Voltage
; LED array. Would noise be a problem? Would have to switch back and forth between FVR and
; VDD for the +ADRef between conversions?
;

setCurrentLEDArrayInverseToLATC:

    banksel LEDS            ; Port C latch

    clrw                    ; start with all cleared in new value

    btfss   LEDS,LED0       ; set opposite bit for each set in scratch0
    bsf     WREG, LED4
    btfss   LEDS,LED1
    bsf     WREG, LED3
    btfss   LEDS,LED2
    bsf     WREG, LED2
    btfss   LEDS,LED3
    bsf     WREG, LED1
    btfss   LEDS,LED4
    bsf     WREG, LED0

    goto    setCurrentLEDArray

; end of setCurrentLEDArrayInverseToLATC
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setVoltageLEDArrayFromADValue
;
; Sets the LEDs in the Voltage Monitor array to reflect the value in ADRESH.
; It is assumed that the A/D value is left justified, thus the upper 8 bits will be used
; and the lower 2 bits will be discarded.
;

setVoltageLEDArrayFromADValue:

    banksel ADRESH              ;read upper 8 bits of result
    movf    ADRESH,W            ; (result is left justified so this gets
                                ;  the upper 8 bits, ignoring the lsbs)

    banksel scratch0
    movwf   scratch0            ; store the A/D value

    ; compare the A/D value with different percentages of the max display value
    ; the total range is divided into 6 possible subranges to account for 5 LEDs on for max and
    ; 0 LEDs on for minimum
    ;
    ; all values rounded down:
    ; 0  - 16%  : 0 LEDs on
    ; 17 - 33%  : 1 LEDs on
    ; 34 - 50%  : 2 LEDs on
    ; 51 - 66%  : 3 LEDs on
    ; 67 - 83%  : 4 LEDs on
    ; 84 - 100% : 5 LEDs on
    ;
    ; sublw : k-(W)->(W)     C = 1 W <= k

    movf    scratch0,W
    sublw   (VOLTAGE_RANGE_INCREMENT * 1)
    btfss   STATUS,C                            ; C = 1 W <= k
    goto    vriTest2

    movlw   0xff                                ; all LEDs off
    goto    setVoltageLEDArray

vriTest2:

    movf    scratch0,W
    sublw   (VOLTAGE_RANGE_INCREMENT * 2)
    btfss   STATUS,C                            ; C = 1 W <= k
    goto    vriTest3

    movlw   0xfe                                ; 1 LED(s) on
    goto    setVoltageLEDArray

vriTest3:

    movf    scratch0,W
    sublw   (VOLTAGE_RANGE_INCREMENT * 3)
    btfss   STATUS,C                            ; C = 1 W <= k
    goto    vriTest4

    movlw   0xfc                                ; 2 LED(s) on
    goto    setVoltageLEDArray

vriTest4:

    movf    scratch0,W
    sublw   (VOLTAGE_RANGE_INCREMENT * 4)
    btfss   STATUS,C                            ; C = 1 W <= k
    goto    vriTest5

    movlw   0xf8                                ; 3 LED(s) on
    goto    setVoltageLEDArray

vriTest5:

    movf    scratch0,W
    sublw   (VOLTAGE_RANGE_INCREMENT * 5)
    btfss   STATUS,C                            ; C = 1 W <= k
    goto    vriTest6

    movlw   0xf0                                ; 4 LED(s) on
    goto    setVoltageLEDArray

vriTest6:

    movlw   0xe0                                ; 5 LED(s) on
    goto    setVoltageLEDArray

; end of setVoltageLEDArrayFromADValue
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setCurrentLEDArray
;
; Sets the on/off states of the LEDs in the "Current" monitor array according to the value in
; the W register.
;
; On entry:
;
; W register: on/off states of the LEDs in the array, 5 lsbs, 0=on;1=0ff
;

setCurrentLEDArray:

    banksel LEDS

    ; apply appropriate bits in WREG to port latch
    ; cannot write entire WREG to LATC as only some of the bits are LEDs
    ; and others have different functions

    ; clear all LED outputs

    bcf     LED0_P, LED0
    bcf     LED1_P, LED1
    bcf     LED2_P, LED2
    bcf     LED3_P, LED3
    bcf     LED4_P, LED4

    ;set each LED output which has corresponding bit set in WREG

    btfsc   WREG,LED0
    bsf     LED0_P, LED0
    btfsc   WREG,LED1
    bsf     LED1_P, LED1
    btfsc   WREG,LED2
    bsf     LED2_P, LED2
    btfsc   WREG,LED3
    bsf     LED3_P, LED3
    btfsc   WREG,LED4
    bsf     LED4_P, LED4

    ; when LATCH ENABLE input is HIGH, the Q outputs
    ; will follow the D inputs. When the LATCH ENABLE goes
    ; LOW, data at the D inputs will be retained

    ; latch high (outputs follow inputs)
    bsf     CURRENT_LED_LATCH_P, CURRENT_LED_LATCH

    ; latch low to lock in data
    bcf     CURRENT_LED_LATCH_P, CURRENT_LED_LATCH

    return

; end of setCurrentLEDArray
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setVoltageLEDArray
;
; Sets the on/off states of the LEDs in the "Voltage" monitor array according to the value in
; the W register.
;
; On entry:
;
; W register: on/off states of the LEDs in the array, 5 lsbs, 0=on;1=0ff
;

setVoltageLEDArray:

    banksel LEDS

    ; apply appropriate bits in WREG to port latch
    ; cannot write entire WREG to LATC as only some of the bits are LEDs
    ; and others have different functions

    ; clear all LED outputs

    bcf     LED0_P, LED0
    bcf     LED1_P, LED1
    bcf     LED2_P, LED2
    bcf     LED3_P, LED3
    bcf     LED4_P, LED4

    ;set each LED output which has corresponding bit set in WREG

    btfsc   WREG,LED0
    bsf     LED0_P, LED0
    btfsc   WREG,LED1
    bsf     LED1_P, LED1
    btfsc   WREG,LED2
    bsf     LED2_P, LED2
    btfsc   WREG,LED3
    bsf     LED3_P, LED3
    btfsc   WREG,LED4
    bsf     LED4_P, LED4

    ; when LATCH ENABLE input is HIGH, the Q outputs
    ; will follow the D inputs. When the LATCH ENABLE goes
    ; LOW, data at the D inputs will be retained

    ; latch high (outputs follow inputs)
    bsf     VOLTAGE_LED_LATCH_P, VOLTAGE_LED_LATCH

    ; latch low to lock in data
    bcf     VOLTAGE_LED_LATCH_P, VOLTAGE_LED_LATCH

    return

; end of setVoltageLEDArray
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleI2CCommand
;
; Checks to see if this PIC has received an ID byte via the I2C bus which matches its address.
;
; The I2C bus is set up to hold the clock and delay transmission by the master after each byte
; is received. That allows this method to be checked in the normal thread instead of by an
; interrupt as any response delays are not a problem as the transmission is halted.
;

handleI2CCommand:

    ; check if SSP1IF flag set -- if so, matching address byte has been received
    
    ifndef debug               ; pretend flag set if in debug mode
    banksel PIR1
    btfss   PIR1, SSP1IF
    return
    endif

    banksel SSPBUF
    movf    SSPBUF,W            ; get incoming value; clears BF flag

    call    clearSSP1IF         ; clear the I2C interrupt flag
    call    setCKP              ; release the I2C clock line so master can send next byte

; jump to handle receive or transmit request
; if bit 0 of the address byte is 0, the master is sending and this PIC is receiving
; if bit 0 is 1, the master is receiving and this PIC is sending

    btfss   WREG,0
    goto    handleI2CReceive
    goto    handleI2CTransmit

    return

; end handleI2CCommand
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleI2CReceive
;
; Handles the receipt of data from the master on the I2C bus.
;
; The next byte read should be the command byte which determines the total number of bytes
; expected.
;

handleI2CReceive:

    call    waitForSSP1IFHighOrStop     ; wait for byte or stop condition to be received

    btfss   STATUS,Z
    goto    cleanUpAndReturn            ; bail out when stop condition received

    call    readI2CByteAndPrepForNext   ; get the byte just received

    banksel scratch0
    movwf   scratch0                    ; store the command byte

; parse the command byte by comparing with each command

    sublw   LEDPIC_SET_LEDS
    btfsc   STATUS,Z
    goto    setLEDsFromI2C

    movf    scratch0,W
    sublw   LEDPIC_SET_PWM
    btfsc   STATUS,Z
    goto    setPWMFromI2C

    movf    scratch0,W
    sublw   LEDPIC_START
    btfsc   STATUS,Z
    goto    doStartFromI2C

    movf    scratch0,W
    sublw   LEDPIC_SET_RESET
    btfsc   STATUS,Z
    goto    doReset

    return

; end handleI2CReceive
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleI2CTransmit
;
; Handles the transmission of data to the master on the I2C bus.
;
; The command byte specifying what data should be sent should have already been received in the
; previous transmission from the master and stored in the masterCommand variable.
;
; I2C Read Protocol
;
; When reading some devices, the Master sends commands and/or address bytes with a write operation,
; performs a restart, sends a read command, then reads the data from the device. The only thing
; a restart does over a stop and start operation is the restart allows the Master to maintain
; control of the bus...another device cannot take it over.
;
; For the Multi-IO Board, there is only one master so it does not need to retain control. In order
; to read data, the Master writes a command and/or further addressing information to the device in
; the normal write protocol, ending with a stop signal. The Slave will save the information and
; will expect that information to apply to the next read operation. The Master will then send a
; normal read command to read the desired data. The Slave will know what information to send based
; on the data sent in the previous write command.
;

handleI2CTransmit:




    return

; end handleI2CTransmit
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setLEDsFromI2C
;
; Sets the on/off state of the LEDs based on the values received via the I2C bus.
;

setLEDsFromI2C:

    ; receive 2 data bytes from I2C -- one for each LED array

    movlw   .2                      ; store 2 bytes from I2C
    call    receiveBytesFromI2C

    ; set state of "Current" red LED array to first received byte

    banksel scratch3
    movf    scratch3,W
    call    setCurrentLEDArray

    ; set state of "Voltage" green LED array to second received byte

    banksel scratch4
    movf    scratch4,W
    call    setVoltageLEDArray

    return

; end setLEDsFromI2C
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setPWMFromI2C
;
; Sets the registers controlling the PWM cutting current pulse control with values received via
; the I2C bus.
;
; The data bytes from the master should be:
;
;    pwmDutyCycleHiByte      ; cutting current pulse controller duty cycle time
;    pwmDutyCycleLoByte
;    pwmPeriod               ; cutting current pulse controller period time
;    pwmPolarity             ; polarity of the PWM output -- only lsb used
;    pwmCheckSum             ; used to verify PWM values read from eeprom
;
; If the checksum does not validate, the PWM registers are not changed.
;
; The value of 1 is added to the checksum to prevent cases where the values are read as all zeroes
; in error and would match the checksum of zero.
;

setPWMFromI2C:

    movlw   .5                      ; store 2 bytes from I2C
    call    receiveBytesFromI2C

    ; validate the checksum

    banksel scratch3

    clrw                            ; calculate the checksum for all PWM values
    addwf   scratch3,W
    addwf   scratch4,W
    addwf   scratch5,W
    addwf   scratch6,W
    addlw   1                       ; see note in function header

    subwf   scratch7,W              ; compare calculated checksum with that read from I2C
    btfss   STATUS,Z
    return                          ; zero flag clear, checksum NOT matched, exit

    ; checksum matched -- apply values

    ; set the polarity of the PWM output

    banksel scratch6
    movf    scratch6,W          ; byte read from I2C bus

    banksel PWM1CON
    bcf     PWM1CON,PWM1POL     ; clear polarity bit
    btfsc   WREG,0
    bsf     PWM1CON,PWM1POL     ; set polarity bit if bit 0 of polarity byte is set

    ; set the PWM cycle period

    banksel scratch5
    movf    scratch5,W          ; byte read from I2C bus
    banksel PR2                 ; period of the cycle
    movwf   PR2

    ; set the PWM duty cycle

    banksel scratch0

    movf    scratch4,W          ; set up low byte of value
    movwf   scratch1

    movf    scratch3,W          ; set up high byte of value
    movwf   scratch2

    call    setPWM1DC           ; set value of PWM1DCH and PWM1DCH duty cycle time registers

    return

; end setPWMFromI2C
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; doStartFromI2C
;
; Sets flag to trigger execution of the normal operation code, such as displaying the A/D
; inputs on the LED arrays, etc.
;

doStartFromI2C:

    banksel flags

    bsf     flags,HANDLE_LED_ARRAYS

    return

; end doStartFromI2C
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; doReset
;
; Places the PIC in known state.
;

doReset:

    return

; end doReset
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; receiveBytesFromI2C
;
; Receives a specified number of bytes from the I2C bus.
; Normally, this number is equal to the number of bytes read unless the master
; sends too few or too many.
;
; On entry:
;  W register: the number of bytes to be stored
;
; Bytes will be read until a stop condition is received from the master -- only the number of bytes
; specified in W will actually be stored.
;
; On exit:
;  scratch1: The number of bytes actually stored (may be less than number of bytes specified to be
;   read if master sends too few and may be less than the number of bytes actually read if master
;   sends too many).
;
; scratch3-scratchx: the received bytes
;
; Other variable usage:
;
; Variable scratch0 is used to count down the number of bytes stored.
; Variable scratch2 is used for control flags.
;

receiveBytesFromI2C:

    banksel scratch0
    movwf   scratch0                ; store number of bytes to record

    clrf    scratch1                ; track number of bytes recorded
    clrf    scratch2                ; clear control flags

    movlw   scratch3                ; point to first byte of receive buffer
    movwf   FSR0L

rBFILoop1:

    call    waitForSSP1IFHighOrStop     ; wait for byte or stop condition to be received

    btfss   STATUS,Z
    goto    cleanUpAndReturn            ; bail out when stop condition received

    call    readI2CByteAndPrepForNext   ; get the byte just received

    banksel scratch0

    btfsc   scratch2,0                  ; if bit 0 set, expected number of bytes already stored
    goto    skipByteStore               ;  more bytes will be read until stop condition received
                                        ;  but they are not stored

    ; store the byte in the buffer

    movwi   FSR0++                      ; store in next buffer position

    incf    scratch1,F                  ; count number of bytes stored

    decfsz  scratch0,F                  ; check number of bytes to record
    goto    rBFILoop1                   ; loop to read more bytes

    bsf     scratch2,0                  ; set flag to prevent storing any more bytes
                                        ; function will read and toss bytes until stop condition

skipByteStore:

    goto    rBFILoop1                   ; loop to read more bytes

cleanUpAndReturn:

    ; makes sure all flags are set/reset to enable data to be received

    call    clearSSP1IF         ; clear the I2C interrupt flag
    call    setCKP              ; release the I2C clock line so master can send next byte
    call    clearSSPOV          ; clears the overflow bit to allow new data to be read

    return

; end receiveBytesFromI2C
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; readI2CByteAndPrepForNext
;
; Reads value from SSPBUF into W register, resets SSP1IF, and sets CKP to release I2C clock so
; master can start sending the next byte.
;
; The last byte received is returned in W.
;

readI2CByteAndPrepForNext:

    banksel SSPBUF
    movf    SSPBUF,W            ; get incoming value; clears BF flag

    call    clearSSP1IF         ; clear the I2C interrupt flag
    call    setCKP              ; release the I2C clock line so master can send next byte
    call    clearSSPOV          ; clears the overflow bit to allow new data to be read

    return

; end of readI2CByteAndPrepForNext
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearSSP1IF
;
; Sets the SSP1IF bit in register PIR1 to 0.
;

clearSSP1IF:

    banksel PIR1
    bcf     PIR1, SSP1IF

    return

; end of clearSSP1IF
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitForSSP1IFHigh
;
; Waits in a loop for SSP1IF bit in register PIR1 to go high.
;

waitForSSP1IFHigh:

    ifdef debug       ; if debugging, don't wait for interrupt to be set high as the MSSP is not
    return            ; simulated by the IDE
    endif

    banksel PIR1

wfsh1:
    btfss   PIR1, SSP1IF
    goto    wfsh1

    return

; end of waitForSSP1IFHigh
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitForSSP1IFHighOrStop
;
; Waits in a loop for SSP1IF bit in register PIR1 to go high or P (stop) bit in SSP1STAT to go
; high.
;
; On exit:
;
; If SSP1IF went high, W will be 0 and Z flag set.
; If P (stop condition) went high, W will be 1 and Z flag cleared.
;

waitForSSP1IFHighOrStop:

    ifdef debug       ; if debugging, don't wait for interrupt to be set high as the MSSP is not
    goto  wfshosByteReceived    ; simulated by the IDE
    endif


wfshos1:

    banksel PIR1
    btfsc   PIR1, SSP1IF        ; check interrupt flag
    goto    wfshosByteReceived  ; return if flag set

    banksel SSP1STAT
    btfsc   SSP1STAT, P         ; check stop condition received flag
    goto    wfshosStopReceived  ; return if flag set

    goto    wfshos1

wfshosByteReceived:

    clrw                        ; signal that byte received
    addlw   0x00                ; (must use addlw to set Z bit)
    return

wfshosStopReceived:

    clrw                        ; signal that stop condition received
    addlw   0x01                ; (must use addlw to clear Z bit)
    return

; end of waitForSSP1IFHighOrStop
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitForSSP1IFHighThenClearIt
;
; Waits in a loop for SSP1IF bit in register PIR1 to go high and then clears that bit.
;
; The bit must be cleared at some point after it is set before performing most actions with the I2C
; but. In most cases, it can be cleared immediately for which this function is useful.
;
; If the bit is not cleared before an operation, then checking the bit immediately after the
; operation will make it appear that the operation completed immediately and the code will not
; wait until the MSSP module sets the bit after actual completion.
;

waitForSSP1IFHighThenClearIt:

    call    waitForSSP1IFHigh

    call    clearSSP1IF

    return

; end of waitForSSP1IFHighThenClearIt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setCKP
;
; Sets the CKP bit in register SSPCON1 to 1.
;
; This will release the I2C bus clock so the master can transmit the next byte.
;

setCKP:

    banksel SSPCON1
    bsf     SSPCON1, CKP

    return

; end of setCKP
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearSSPOV
;
; Clears the MSSP overflow bit to allow new bytes to be read.
;
; The bit is set if a byte was received before the previous byte was read from the buffer.
;

clearSSPOV:

    banksel SSP1CON1
    bcf     SSP1CON1,SSPOV

    return

; end of clearSSPOV
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
; It is important to use no (or very few) subroutine calls.  The stack is only 16 deep and
; it is very bad for the interrupt routine to use it.
;

handleInterrupt:

	btfsc 	INTCON,T0IF     		; Timer0 overflow interrupt?
	goto 	handleTimer0Interrupt	; YES, so process Timer0

INT_ERROR_LP1:		        		; NO, do error recovery
	;GOTO INT_ERROR_LP1      		; This is the trap if you enter the ISR
                               		; but there were no expected interrupts

	retfie                  ; return and enable interrupts

; end of handleInterrupt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleTimer0Interrupt
;
; This function is called when the Timer 0 register overflows.
;
; The prescaler is set to 1:256.
; 16 Mhz Fosc = 4 Mhz instruction clock (CLKOUT)
; 4,000,000 Hz / 256 = 15,625 Hz;  15,625 Hz / 156 = 100 Hz
; Interrupt needed every 156 counts of TMR0 -- set to 255-156.
;
; Interrupt triggered when 8 bit TMR0 register overflows, so subtract desired number of increments
; between interrupts from 255 for value to store in register.
;
; NOTE NOTE NOTE
; It is important to use no (or very few) subroutine calls.  The stack is only 8 deep and
; it is very bad for the interrupt routine to use it.
;

handleTimer0Interrupt:

	bcf 	INTCON,TMR0IF     ; clear the Timer0 overflow interrupt flag

    ; reload the timer -- see notes in function header

    movlw   (.255 - .156)
    banksel TMR0
    movwf   TMR0

;debug mks -- output a pulse to verify the timer0 period
    banksel DEBUG_IO_P
    bsf     DEBUG_IO_P,DEBUG_IO
    bcf     DEBUG_IO_P,DEBUG_IO
;debug mks end

    ; trigger the next A/D conversion

    movlw   0x01
    banksel adTrigger
    movwf   adTrigger

	retfie                  ; return and enable interrupts

; end of handleTimer0Interrupt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; flashLEDsUntilMasterPICConnects
;
; Flashes the LEDs until the Master PIC first communicates.
;

flashLEDsUntilMasterPICConnects:

flumpc1:

    ;set LEDs

    movlw   0xaa
    call    setCurrentLEDArray

    movlw   0x55
    call    setVoltageLEDArray

    ; delay

    banksel scratch6
    movlw   .5
    movwf   scratch6
    call    rbd1

    ;set LEDs

    movlw   0x55
    call    setCurrentLEDArray

    movlw   0xaa
    call    setVoltageLEDArray

    ; delay

    banksel scratch6
    movlw   .5
    movwf   scratch6
    call    rbd1

    ; check if SSP1IF flag set -- if so, Master PIC has sent a command

    ifndef debug               ; pretend flag set if in debug mode
    banksel PIR1
    btfss   PIR1, SSP1IF
    goto    flumpc1
    endif

    return

; end of flashLEDsUntilMasterPICConnects
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; reallyBigDelay
;
; Delays for a second or two.
;
; To adjust the delay, load scratch6 with any value and enter at rbd1.
; In that case, use "banksel scratch6" before entering.
;

reallyBigDelay:

    banksel scratch6

    movlw   .50
    movwf   scratch6

rbd1:

    movlw   .255
    movwf   scratch7

rbd2:

    movlw   .255
    movwf   scratch8

rbd3:

    decfsz  scratch8,F
    goto    rbd3

    decfsz  scratch7,F
    goto    rbd2

    decfsz  scratch6,F
    goto    rbd1

    return

; end of reallyBigDelay
;--------------------------------------------------------------------------------------------------

    END
