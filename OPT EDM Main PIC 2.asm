;--------------------------------------------------------------------------------------------------
; Project:  OPT EDM Notch Cutter -- Main PIC software
; Date:     2/29/12
; Revision: See Revision History notes below.
;
; IMPORTANT: When programming the PIC in the notch cutter, turn the Electrode Current switch to
; Off and the Electrode Motion switch to Setup.
;
; Normally, the programming header for the LCD PIC is not installed on the board.  It can be
; installed in the Main PIC socket, programmed, and then moved to the LCD PIC socket.
;
; Overview:
;
; This program controls an EDM cutting device by manipulating a motor which moves the head up and
; down and a high current power supply which provides voltage to the cutting blade.  The current
; output of the electrode cutting blade is monitored to adjust the height of the blade above the
; material being cut so that the current maintains optimum value.
;
; The program monitors several button inputs and displays data on an LCD display.
;
; There are two PIC controllers on the board -- the Main PIC and the LCD PIC.  This code is
; for the Main PIC.  The Main PIC sends data to the LCD PIC via a serial data line for display
; on the LCD.
;
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
; 1.0   Some code and concepts used from source code disassembled from hex object code version 6.4 
;       from original author.
; 7.6e	Version distributed.  Has aggression control.
; 7.7a	Fixed bug: asterisk wasn't displaying by the "Up" label when doing fast retract from
;		 over current condition in auto cut mode.
;		Increased responsiveness in auto cut mode.
;		Changed motor step size back to Full.  This was the original setting used by the designer.
; 7.7b  Motor direction reversed so motor wiring makes more sense.
;		The "R" prefix removed from the displayed version so that the letter suffix can fit.
; 7.7c	Added "repeat cycle" test function for testing for proper operation.  The head will be
;		driven down until the low current input signal is cleared and then retracted quickly
;		back to the starting position.  The cycle is repeated until the user exits.
; 7.7d	Fixed bug in "repeat cycle" test where it was locking up in the retreat mode because
;		the power supply wasn't coming up fast enough at the start.
;		Improved the input button handling - better debounce, faster response.
;		Improved the multiple page menu handling - cursor starts on bottom option when moving
;		back to a previous menu page.
; 7.7e  Fixed incrementing/decrementing of multibyte variables -- decrementing was skipping a
;		count when crossing the zero thresold of an upper byte.
; 7.7f	Fixed comments explaining commands to the LCD screen. Cleaned up superfluous code.
; 7.7g	Refactoring for clarity.
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
; Standard Head
;
; The cam should turn clockwise when the head is being driven down.
; The cam should make a full rotation in 65 seconds while jogging up or down in Setup mode.
;
; The motor used for the OPT standard reach head is:
;
; manufactured by Faulhaber @ http://www.faulhaber.com/
; sold by www.micromo.com
;
; Printed on motor: 
;
;	ARSAPE
;	swiss made
;	AM1524-A-025-125-71
;
; 	AM1524 	  ~ AM motor design, 15 mm diameter, 24 steps per revolution
;	A-025-125 ~ -A-0,25-12,5 Winding Type
;	71	  ~ 71 (-73)  ~ Plain shaft, L=4,3mm (Designation for assembly with gearhead 15A)
;
; A planetary gearhead is attached: Faulhaber Series 15A.  There are no identifying numbers for
; the gearhead printed on the assembly.
; 
; Using Full Step mode, the motor has 24 steps per revolution.
;
;--------------------------------------------------------------------------------------------------
;
; J8 on the schematics is implemented as switches labeled "microstep set" on the board.  The
; step size inputs to the motor controller chip are defined as follows:
;
; MS1 MS2
;  L   L  = Full Step
;  H   L  = Half
;  L   H  = Quarter
;  H   H =  Eighth
;
; Typical J8 switch settings on the board:
; 1 (MS1) = Off  2 (MS2) = On
;
; When a J8 switch is off, it forces the associated input (MS1 or MS2) to be low.
; When a J8 switch is on, it allows the associated input to be controlled by RB2.
;
; Thus, with MS1 Off, it is always low.
; With MS1 On, it is set by RB2.
; If RB2 is set high, MS2 = high - step size will be 1/4.  If set low, step size will be full.
;
; The full step size is currently used - as of version 7.7a
;
; On the standard reach head: 1 degree rotation of the motor = 0.001" of blade travel
; On the extended reach head: 360 degree rotation = 1mm of blade travel
;
; 1 meter = 39.3700787 inches, 1mm = 0.0393700787"
;
; Standard reach : 1 revolution = .360" blade travel
; Extended reach : 1 revolution = .039" blade travel
; Standard Ratio is 9.2 times Extended Ratio
;
;
; The internal comparators are NOT used - see note in header of "setup" function.
;
;--------------------------------------------------------------------------------------------------
; Hardware Control Description
;
; Function by Pin
;
; Port A
;
; RA0   In  - serial data in line -- com link to other devices
; RA1   In  - unused
; RA2   xxx - not implemented in PIC16f1459
; RA3   In  - short detect
; RA4   In  - cutting current high limit trigger
; RA5   Out - cutting current power supply on/off
; RA6   xxx - not implemented in PIC16f1459
; RA7   xxx - not implemented in PIC16f1459
;
; Port B
;
; RB0   xxx - not implemented in PIC16f1459
; RB1   xxx - not implemented in PIC16f1459
; RB2   xxx - not implemented in PIC16f1459
; RB3   xxx - not implemented in PIC16f1459
; RB4   I/O - I2CSDA
; RB5   In  - Jog Down Switch
; RB6   Out - I2CSCL
; RB7   Out - serial data out line -- com link to other devices
;
; Port C
;
; RC0   Out - motor enable
; RC1   In  - Mode Switch
; RC2   In  - Jog Up Switch
; RC3   Out - motor direction
; RC4   Out - motor step
; RC5   In  - cutting current low limit trigger
; RC6   Out - Motor Mode Step Size - (Microstep Select Full/Half/Quarter/Eighth)
; RC7   In  - Select Switch
;
;end of Hardware Control Description
;--------------------------------------------------------------------------------------------------
;
; User Inputs
;
; On startup, Mode button used to select between Standard Cut Depth and Extended Cut Depth
;
; Mode button used during operation to select Cut, Jog, & Setup modes.
;
; Jog Up and Jog Down buttons used to move cutting head up & down.
;
; Select button used to accept an entry or selection, stop a function, or return to a menu.
;  (also referred to as "Reset" on the schematic and "Zero Reset" on wiring harness"
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


; Values for the digital pot settings.
;
; Each count equals approximately 0.0196V when power is at +5V
;

HI_CURRENT_LIMIT_POT    EQU     .104    ; tweak for 2.0 Volts
LO_CURRENT_LIMIT_POT    EQU     .94     ; tweak for 1.8 Volts


VOLTAGE_MONITOR_POT     EQU     .127    ; not currently used -- manual pot is used instead
CURRENT_MONITOR_POT     EQU     .127    ; not currently used -- manual pot is used instead

; end of Defines
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Step Ratio Constants
;
; Sets the number of steps per 0.001" for different types of cutting heads.
;

; Extended Ratio is 9.2 times Standard ratio.

; Standard Ratio
; suggested setup is 17% blade erosion
; was 0x1a for 1/4 motor step

STANDARD_RATIO1 EQU     0x0      
STANDARD_RATIO0 EQU     0x06
                            
; Extended Ratio
; suggested setup is ??% blade erosion for wall reduction?
; was 0xdc for 1/4 motor step

EXTENDED_RATIO1  EQU     0x0
EXTENDED_RATIO0  EQU     0x37

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

SERIAL_IN_P     EQU     PORTA
SERIAL_IN       EQU     RA0         ; input ~ RA0 can only be input on PIC16f1459
UNUSED_RA1_P    EQU     PORTA
UNUSED_RA1      EQU     RA1         ; input ~ RA1 can only be input on PIC16f1459
;NA_RA2         EQU     RA2         ; RA2 not implemented on PIC16f1459
SHORT_DETECT_P  EQU     PORTA
SHORT_DETECT    EQU     RA3         ; input ~ RA3 can only be input on PIC16f1459
HI_LIMIT_P      EQU     PORTA
HI_LIMIT        EQU     RA4         ; input ~ cutting current hight limit
POWER_ON_P      EQU     PORTA
POWER_ON        EQU     RA5         ; output
;NA_RA6         EQU     RA6         ; RA6 not implemented on PIC16f1459
;NA_RA7         EQU     RA7         ; RA7 not implemented on PIC16f1459

; Port B

I2CSDA_LINE     EQU     RB4
JOG_DWN_SW_P    EQU     PORTB
JOG_DWN_SW      EQU     RB5         ; input
I2CSCL_LINE     EQU     RB6
SERIAL_OUT_P    EQU     PORTB
SERIAL_OUT      EQU     RB7         ; output ~ serial data out to other devices

; Port C

MOTOR_ENABLE_P  EQU     PORTC
MOTOR_ENABLE    EQU     RC0         ; output
MODE_SW_P       EQU     PORTC
MODE_SW         EQU     RC1         ; input
JOG_UP_SW_P     EQU     PORTC
JOG_UP_SW       EQU     RC2         ; input
MOTOR_DIR_P     EQU     PORTC
MOTOR_DIR       EQU     RC3         ; output
MOTOR_STEP_P    EQU     PORTC
MOTOR_STEP      EQU     RC4         ; output
LO_LIMIT_P      EQU     PORTC
LO_LIMIT		EQU     RC5         ; input ~ cutting current low limit
MOTOR_MODE_P    EQU     PORTC
MOTOR_MODE      EQU     RC6         ; output ~ motor step size selection
SELECT_SW_P     EQU     PORTC
SELECT_SW       EQU     RC7         ; input ~ select switch

; Button State Flags

SELECT_SW_STATE     EQU     0
JOG_UP_SW_STATE     EQU     1
JOG_DWN_SW_STATE    EQU     2
MODE_SW_STATE       EQU     3

; I2C bus ID byte for writing to digital pot 1
; upper nibble = 1010 (bits 7-4)
; chip A2-A0 inputs = 001 (bits 3-1)
; R/W bit set to 0 (bit 0)

DIGITAL_POT1_WRITE_ID       EQU     b'10100010'

HI_LIMIT_POT_ADDR           EQU     0x0
LO_LIMIT_POT_ADDR           EQU     0x1
VOLTAGE_MONITOR_POT_ADDR    EQU     0x2
CURRENT_MONITOR_POT_ADDR    EQU     0x3

; I2C bus ID bytes for writing and reading to EEprom 1
; upper nibble = 1010 (bits 7-4)
; chip A2-A0 inputs = 000 (bits 3-1)
; R/W bit set to 0 (bit 0) for writing
; R/W bit set to 1 (bit 0) for reading

EEPROM1_WRITE_ID            EQU     b'10100000'
EEPROM1_READ_ID             EQU     b'10100001'


; end of Hardware Definitions
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; Software Definitions

; bits in flags variable

EXTENDED_MODE   EQU     0x0
CUT_STARTED     EQU     0x1
AT_DEPTH        EQU     0x2
WALL_MODE       EQU     0x3
DATA_MODIFIED   EQU     0x4
UPDATE_DISPLAY  EQU     0x5
UPDATE_DIR_SYM  EQU     0x6
MOTOR_DIR_MODE	EQU     0x7

; bits in LCDFlags

LCDBusy         EQU     0x00
startBit        EQU     0x01
stopBit         EQU     0x02
endBuffer       EQU     0x03
inDelay         EQU     0x04

; LCD Display Commands

CLEAR_SCREEN_CMD	EQU		0x01

; LCD Display On/Off Command bits

;  bit 3: specifies that this is a display on/off command if 1
;  bit 2: 0 = display off, 1 = display on
;  bit 1: 0 = cursor off, 1 = cursor on
;  bit 0: 0 = character blink off, 1 = blink on

DISPLAY_ONOFF_CMD_FLAG	EQU		0x08
DISPLAY_ON_FLAG			EQU		0x04
CURSOR_ON_FLAG			EQU		0x02
BLINK_ON_FLAG			EQU		0x01

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

    flags                   ; bit 0: 0 = standard mode, 1 = extended cut depth mode
                            ; bit 1: 0 = cut not started, 1 = cut started
                            ; bit 2: 0 = depth not reached, 1 = depth reached
                            ; bit 3: 0 = notch mode, 1 = wall reduction mode
                            ; bit 4: 0 = data not modified, 1 = data modified (used by various functions)
                            ; bit 5: 0 = no display update, 1 = display update (used by various functions)
							; bit 6: 0 = no update direction symbol, 1 = update (used by various functions)
							; bit 7: 0 = normal motor rotation, 1 = reverse motor direction

    menuOption              ; tracks which menu option is currently selected

    buttonState                  
                            ; bit 0: 0 = Select/Reset/Zero/Enter button pressed
                            ; bit 1: 0 = Jog Up button pressed
                            ; bit 2: 0 = Jog Down button pressed
                            ; bit 3: 0 = Mode button switched to Setup mode

    buttonPrev              ; state of buttons the last time they were scanned
                            ; bit assignments same as for buttonState

    eepromAddressL		    ; use to specify address to read or write from EEprom
    eepromAddressH          ; high byte
    eepromCount	        	; use to specify number of bytes to read or write from EEprom

    speedValue              ; stores sparkLevel converted to a single digit

    sparkLevel              ; current value being used - copy from sparkLevelNotch or sparkLevelWall
                            ; depending on flags.WALL_MODE

    sparkLevelNotch         ; specifies the low amount of spark to trigger an advance in notch mode, smaller number
                            ; makes advance more aggressive  
                            ; NOTE: keep eeSparkLevelNotch and eeSparkLevelWall contiguous
    sparkLevelWall          ; same as above, but for the wall reduction mode
                            ; NOTE: keep eeSparkLevelNotch and eeSparkLevelWall contiguous

    sparkTimer1             ; tracks time between sparks (sensed by voltage change on comparator input)
    sparkTimer0

    overCurrentTimer1	    ; tracks time between over current spikes (sensed by voltage change on comparator input)
    overCurrentTimer0

    ratio1                  ; stores the number of motor steps per 0.001"
    ratio0                  ; this is used to reload preScaler
                            
    ratio_neg1              ; negated value of ratio
    ratio_neg0                  

    preScaler1              ; scales the change of position variable to match actual actual movement
    preScaler0

    debounce1               ; switch debounce timer decremented by the interrupt routine
    debounce0

    normDelay               ; delay between motor pulses for normal mode (slow)
    setupDelay              ; delay between motor pulses for setup mode (fast)

    cursorPos               ; contains the location of the cursor
                            ; NOTE: LCD addressing is screwy - the lines are not in sequential order:
                            ; line 1 column 1 = 0x80  	(actually address 0x00)
                            ; line 2 column 1 = 0xc0	(actually address 0x40)
                            ; line 3 column 1 = 0x94	(actually address 0x14)
                            ; line 4 column 1 = 0xd4	(actually address 0x54)
							;
							; To address the second column in each line, use 81, C1, 95, d5, etc.
							;
							; The two different columns of values listed above are due to the fact that the address
							; is in bits 6:0 and control bit 7 must be set to signal that the byte is an address
							; byte.  Thus, 0x00 byte with the control bit set is 0x80.  The 0x80 value is what is
							; actually sent to the LCD to set address 0x00.
							;
							;  Line 3 is actually the continuation in memory at the end of line 1
							;    (0x94 - 0x80 = 0x14 which is 20 decimal -- the character width of the display)
							;  Line 4 is a similar extension of line 2.
                            ;
							; Note that the user manual offered by Optrex shows the line addresses
							; for 20 character wide displays at the bottom of page 20.

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

	debug0					; debug mks - use a scratch variable instead?
	debug1					; debug mks - use a scratch variable instead?

    position3               ; MSByte position of the electrode in BCD digits
    position2
    position1
    position0               ; LSB    
    positionSign            ; sign for position variable

    zero3                   ; MSByte zero location for the electrode in BCD digits
    zero2
    zero1
    zero0                   ; LSB    

    depth3                  ; MSByte target cut depth for the electrode in BCD digits
    depth2
    depth1
    depth0                  ; LSB    

 endc

;-----------------

; Assign variables in RAM - Bank 1 - must set BSR to 1 to access
; Bank 1 has 80 bytes of free space

 cblock 0xa0                ; starting address

    LCDFlags                ; bit 0: 0 = LCD buffer not busy, 1 = buffer busy
                            ; bit 1: 0 = start bit not due, 1 = transmit start bit next
                            ; bit 2: 0 = stop bit not due,  1 = transmit stop bit next
                            ; bit 3: 0 = not buffer end,  1 = buffer end reached
                            ; bit 4: 0 = not delaying, 1 = delaying
    LCDScratch0             ; scratch pad variable

    LCDBitCount             ; tracks bits of byte being transmitted to LCD
    LCDBufferCnt            ; number of characters in the buffer
    LCDBufferPtr            ; points to next byte in buffer to be transmitted to LCD 

    LCDDelay1               ; delay counter for providing necessary time delay between
    LCDDelay0               ;    chars

    LCDBuffer0              ; LCD display buffer - holds string being transmitted to the LCD
    LCDBuffer1
    LCDBuffer2
    LCDBuffer3
    LCDBuffer4
    LCDBuffer5
    LCDBuffer6
    LCDBuffer7
    LCDBuffer8
    LCDBuffer9
    LCDBuffera
    LCDBufferb
    LCDBufferc
    LCDBufferd
    LCDBuffere
    LCDBufferf
    LCDBuffer10
    LCDBuffer11
    LCDBuffer12
    LCDBuffer13
    LCDBuffer14
    LCDBuffer15
    LCDBuffer16
    LCDBuffer17
    LCDBuffer18
    LCDBuffer19
    LCDBuffer1a
    LCDBuffer1b
    LCDBuffer1c
    LCDBuffer1d
    LCDBuffer1e
    LCDBuffer1f
    LCDBuffer20
    LCDBuffer21
    LCDBuffer22
    LCDBuffer23
    LCDBuffer24
    LCDBuffer25
    LCDBuffer26
    LCDBuffer27
    LCDBuffer28
    LCDBuffer29
    LCDBuffer2a
    LCDBuffer2b
    LCDBuffer2c
    LCDBuffer2d
    LCDBuffer2e
    LCDBuffer2f
    LCDBuffer30
    LCDBuffer31
    LCDBuffer32
    LCDBuffer33
    LCDBuffer34
    LCDBuffer35
    LCDBuffer36
    LCDBuffer37
    LCDBuffer38
    LCDBuffer39
    LCDBuffer3a
    LCDBuffer3b
    LCDBuffer3c
    LCDBuffer3d
    LCDBuffer3e
    LCDBuffer3f

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
; Variables in EEprom
;
; Assign variables in EEprom
;

 cblock 	0x0      	; Variables start in RAM at 0x0
	
	eeDepth3                ; storage for depth value
    eeDepth2
    eeDepth1
    eeDepth0
    eeFlags
    eeSparkLevelNotch       ; NOTE: keep eeSparkLevelNotch and eeSparkLevelWall contiguous
    eeSparkLevelWall

 endc

; end of Variables in EEprom
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
; NOTE: You must save values (PUSH_MACRO) and clear PCLATH before jumping to the interrupt
; routine - if PCLATH has bits set it will cause a jump into an unexpected program memory
; bank.

	PUSH_MACRO              ; MACRO that saves required context registers
	clrf	STATUS          ; set to known state
    clrf    PCLATH          ; set to bank 0 where the ISR is located
    goto 	handleInterrupt	; points to interrupt service routine

; end of Reset Vectors
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; start
;

start:

    call    setup           ; preset variables and configure hardware

menuLoop:

    call    doExtModeMenu   ; display and handle the Standard / Extended Mode menu

    call    doMainMenu      ; display and handle the main menu

    goto    menuLoop
    
; end of start
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setup
;
; Presets variables and configures hardware.
;
; NOTE: The system does not use the internal comparators even though it appears there are analog
; inputs to some of the comparator pins.  Those inputs are the output of external op-amps
; acting as comparators - their outputs are read as digital inputs on RA3 and RA4
;

setup:

    call    setupClock      ; set system clock source and frequency

    call    setupPortA      ; prepare Port A for I/O

    call    setupPortB      ; prepare Port B for I/O

    call    setupPortC      ; prepare Port C  for I/O

    call    initializeOutputs

    call    setupI2CMaster7BitTransmitMode ; prepare the I2C serial bus for use

    call    setDigitalPots  ; set digital pot values to stored values

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

    movlw   0x3
    movwf   scratch1
    movlw   0xe8
    call    bigDelayA       ; delay
    
    movlw   position3
    call    zeroQuad        ; clear the position variable
    clrf    positionSign    ; set sign positive

    movlw   zero3
    call    zeroQuad        ; clear the zero variable

    clrf    buttonState     ; zero various variables
    clrf    preScaler1
    clrf    preScaler0
    clrf    ratio1
    clrf    ratio0
    clrf    ratio_neg1
    clrf    ratio_neg0

    call    readFlagsFromEEprom     ; read the value stored for flags from the EEProm

    ; reset some values to a default state
    ; leave WALL_MODE as read from eeprom - this state is saved
	; leave MOTOR_DIR_MODE as read from eeprom - this state is saved

    bcf     flags,EXTENDED_MODE
    bcf     flags,CUT_STARTED
    bcf     flags,AT_DEPTH
    bcf     flags,DATA_MODIFIED
    bcf     flags,UPDATE_DISPLAY

    call    readSparkLevelsFromEEprom

    ; set sparkLevel to value loaded for Notch or Wall mode depending on current mode

    movf    sparkLevelNotch,W   ; use Notch mode value if in Wall Reduction mode
    btfsc   flags,WALL_MODE
    movf    sparkLevelWall,W    ; use Wall mode value if in Wall Reduction mode
    movwf   sparkLevel          ; save the selected value

    call    readDepthValueFromEEprom    ; read the value stored for depth from the EEProm

    ; set up the LCD buffer variables

    banksel LCDFlags

    clrf    LCDFlags        
    movlw   LCDBuffer0
    movwf   LCDBufferPtr
    clrf    LCDBufferCnt    ; no characters in the buffer
	
; enable the interrupts

	bsf	    INTCON,PEIE	    ; enable peripheral interrupts (Timer0 is a peripheral)
    bsf     INTCON,T0IE     ; enable TMR0 interrupts
    bsf     INTCON,GIE      ; enable all interrupts

    banksel MOTOR_ENABLE_P
    bcf     MOTOR_ENABLE_P, MOTOR_ENABLE    ; enable the motor

    call    resetLCD        ; resets the LCD PIC and positions at line 1 column 1

    return

; end of setup
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; initializeOutputs
;
; Initializes all outputs to known values.
;

initializeOutputs:

    banksel SERIAL_OUT_P
    bsf     SERIAL_OUT_P,SERIAL_OUT

    banksel POWER_ON_P
    bcf     POWER_ON_P, POWER_ON

    banksel MOTOR_STEP_P
    bsf     MOTOR_STEP_P, MOTOR_STEP

    banksel MOTOR_DIR_P
    bsf     MOTOR_DIR_P, MOTOR_DIR

    banksel MOTOR_ENABLE_P              ; disable the motor
    bsf     MOTOR_ENABLE_P, MOTOR_ENABLE

    banksel MOTOR_MODE_P
	bcf     MOTOR_MODE_P, MOTOR_MODE    ; choose full step if J8-1 (MS1) = Off and J8-2 (MS2) = On
                                        ; see notes at top of page for more info

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

    bsf     TRISA, SERIAL_IN            ; input
    bsf     TRISA, SHORT_DETECT         ; input
    bsf     TRISA, HI_LIMIT             ; input
    bcf     TRISA, POWER_ON             ; output

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

    bsf     TRISB, JOG_DWN_SW           ; input
    bcf     TRISB, SERIAL_OUT           ; output

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

    banksel PORTC
    clrf    PORTC                       ; init port value

    banksel LATC                        ; init port data latch
    clrf    LATC

    banksel ANSELC
    clrf    ANSELC                      ; setup port for all digital I/O

    ; set I/O directions

    banksel TRISC
    movlw   b'11111111'                 ; first set all to inputs
    movwf   TRISC

    bcf     TRISC, MOTOR_ENABLE         ; output
    bsf     TRISC, MODE_SW              ; input
    bsf     TRISC, JOG_UP_SW            ; input
    bcf     TRISC, MOTOR_DIR            ; output
    bcf     TRISC, MOTOR_STEP           ; output
    bsf     TRISC, LO_LIMIT             ; input
    bcf     TRISC, MOTOR_MODE           ; output
    bsf     TRISC, SELECT_SW            ; input

    return

; end of setupPortC
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setDigitalPots
;
; Sets the digital pot values to their stored settings.
;

setDigitalPots:

    banksel scratch0
    movlw   HI_LIMIT_POT_ADDR
    movwf   scratch0
    movlw   HI_CURRENT_LIMIT_POT
    movwf   scratch1
    call    setDigitalPotInChip1

    banksel scratch0
    movlw   LO_LIMIT_POT_ADDR
    movwf   scratch0
    movlw   LO_CURRENT_LIMIT_POT
    movwf   scratch1
    call    setDigitalPotInChip1

    banksel scratch0
    movlw   VOLTAGE_MONITOR_POT_ADDR
    movwf   scratch0
    movlw   VOLTAGE_MONITOR_POT
    movwf   scratch1
    call    setDigitalPotInChip1

    banksel scratch0
    movlw   CURRENT_MONITOR_POT_ADDR
    movwf   scratch0
    movlw   CURRENT_MONITOR_POT
    movwf   scratch1
    call    setDigitalPotInChip1

    return

; end of setDigitalPots
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; readDepthValueFromEEprom
;
; Reads the user set cut depth value from eeprom. These are unpacked BCD digits.
;
; Each digit is checked for valid range of 0-9 and force to that range.
;
; Note: these variables must be kept contiguous in memory and eeprom.
;

readDepthValueFromEEprom:

    banksel depth3

    movlw   depth3          ; address in RAM
    movwf   FSR0L
    clrf    eepromAddressH
    movlw   eeDepth3        ; address in EEprom
    movwf   eepromAddressL
    movlw   .4
    movwf   eepromCount     ; read 4 bytes
    call    readFromEEprom

    ; check each digit for illegal BCD value (0-9)

    movf    depth3,W
    movwf   scratch0
    call    applyBCDDigitLimits
    movf    scratch0,W
    movwf   depth3

    movf    depth2,W
    movwf   scratch0
    call    applyBCDDigitLimits
    movf    scratch0,W
    movwf   depth2

    movf    depth1,W
    movwf   scratch0
    call    applyBCDDigitLimits
    movf    scratch0,W
    movwf   depth1

    movf    depth0,W
    movwf   scratch0
    call    applyBCDDigitLimits
    movf    scratch0,W
    movwf   depth0

    return

; end of readDepthValueFromEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; saveDepthValueToEEprom
;
; Saves the user set cut depth value from eeprom. These are BCD digits.
;
; Note: these variables must be kept contiguous in memory and eeprom.
;

saveDepthValueToEEprom:

    banksel depth3

    movlw   depth3          ; address in RAM
    movwf   FSR0L
    clrf    eepromAddressH
    movlw   eeDepth3        ; address in EEprom
    movwf   eepromAddressL
    movlw   .4
    movwf   eepromCount     ; write 4 bytes
    call    writeToEEprom

    return

; end of saveDepthValueToEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; readSparkLevelsFromEEprom
;
; Reads the spark Wall/Notch aggression/speed values from eeprom.
;
; Note: these variables must be kept contiguous in memory and eeprom.
;

readSparkLevelsFromEEprom:

    banksel sparkLevelNotch

    movlw   sparkLevelNotch     ; address in RAM
    movwf   FSR0L
    clrf    eepromAddressH
    movlw   eeSparkLevelNotch   ; address in EEprom
    movwf   eepromAddressL
    movlw   .2
    movwf   eepromCount         ; read 2 bytes
    call    readFromEEprom

    ; apply limits to the values to prevent illegal values read from eeprom

    banksel scratch0

    movlw   0x01                        ; lower limit
    movwf   scratch1
    movlw   0x81                        ; upper limit
    movwf   scratch2

    movf    sparkLevelNotch,W           ; limit notch spark level value
    movwf   scratch0
    call    applyLimitsToByteValue
    movf    scratch0,W                  ; update variable with clipped value
    movwf   sparkLevelNotch

    movf    sparkLevelWall,W            ; limit wall spark level value
    movwf   scratch0
    call    applyLimitsToByteValue
    movf    scratch0,W                  ; update variable with clipped value
    movwf   sparkLevelWall

    return

; end of readSparkLevelsFromEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; saveSparkLevelsToEEprom
;
; Saves the spark Wall/Notch aggression/speed values to eeprom.
;
; Note: these variables must be kept contiguous in memory and eeprom.

saveSparkLevelsToEEprom:

    banksel sparkLevelNotch

    movlw   sparkLevelNotch     ; address in RAM
    movwf   FSR0L
    clrf    eepromAddressH
    movlw   eeSparkLevelNotch   ; address in EEprom
    movwf   eepromAddressL
    movlw   .2
    movwf   eepromCount         ; write 2 bytes
    call    writeToEEprom

    return

; end of saveSparkLevelsToEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; readFlagsFromEEprom
;
; Reads the flags value from eeprom.
;

readFlagsFromEEprom:

    banksel flags

    movlw   flags           ; address in RAM
    movwf   FSR0L
    clrf    eepromAddressH
    movlw   eeFlags         ; address in EEprom
    movwf   eepromAddressL
    movlw   .1
    movwf   eepromCount     ; read 1 byte
    call    readFromEEprom

    return

; end of readFlagsFromEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; saveFlagsToEEprom
;
; Saves the flags value to eeprom.
;

saveFlagsToEEprom:

    banksel flags

    movlw   flags           ; address in RAM
    movwf   FSR0L
    clrf    eepromAddressH
    movlw   eeFlags         ; address in EEprom
    movwf   eepromAddressL
    movlw   .1
    movwf   eepromCount     ; write 1 byte
    call    writeToEEprom

    return

; end of saveFlagsToEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; resetLCD
;
; Resets the LCD screen.
;

resetLCD:

    movlw   CLEAR_SCREEN_CMD
    call    writeControl    ; send Clear Display control code to the LCD

    movlw   0x80
    call    writeControl    ; position at line 1 column 1

    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

    return
    
; end of resetLCD
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

	POP_MACRO               	; MACRO that restores required registers

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

    banksel debounce0

    movf    debounce0,W		; if debounce counter is zero, don't decrement it
    iorwf   debounce1,W
    btfsc   STATUS,Z
    goto    doLCD

    decf	debounce0,F     ; count down debounce timer
    btfsc   STATUS,Z		; not perfect count down - Z flag set one count before roll-under
    decf    debounce1,F

doLCD:

    banksel LCDFlags        ; select data bank 1 to access LCD buffer variables

    btfss   LCDFlags,LCDBusy    
    goto    endISR          ; if nothing in buffer, exit

    btfss   LCDFlags,inDelay    ; if in delay phase, waste time until counter is zero
    goto    startBitCheck       ;  (this delay is necessary between words, but is used
                                ;    between every byte for simplicity)

    decfsz  LCDDelay0,F         ; decrement (actually decrements upper byte when lower byte reaches 0
    goto    endISR              ;   instead of just past 0, but accurate enough for this purpose)
    decfsz  LCDDelay1,F
    goto    endISR

    bcf     LCDFlags,inDelay    ; delay ended

    bsf     LCDFlags,startBit   ; transmit start bit on next interrupt
    
    btfss   LCDFlags,endBuffer  ; buffer empty?
    goto    endISR

    clrf    LCDFlags            ; if end of buffer reached, set LCD not busy
    movlw   LCDBuffer0
    movwf   LCDBufferPtr        ; reset pointer to beginning of the buffer
    clrf    LCDBufferCnt        ; no characters in the buffer

    goto    endISR

startBitCheck:

    btfss   LCDFlags,startBit   ; if set, initiate a startbit and exit    
    goto    stopBitCheck

    banksel SERIAL_OUT_P

    bcf     SERIAL_OUT_P,SERIAL_OUT ; transmit start bit (low)

    banksel LCDFlags

    bcf     LCDFlags,startBit   ; start bit done
    movlw   .8
    movwf   LCDBitCount         ; prepare to send 8 bits starting with next interrupt
    goto    endISR    

stopBitCheck:

    btfss   LCDFlags,stopBit    ; if set, initiate a stopbit and exit
    goto    transmitByteT0I

    banksel SERIAL_OUT_P

    bsf     SERIAL_OUT_P, SERIAL_OUT           ; transmit stop bit (high)

    banksel LCDFlags
    
    bcf     LCDFlags,stopBit    ; stop bit done
    
    movlw   0x30                ; don't use less than 1 here - will count from 0xff
    movwf   LCDDelay0
    movlw   0x01                ; don't use less than 1 here - will count from 0xff
    movwf   LCDDelay1           ; setup delay
    bsf     LCDFlags,inDelay    ; start delay on next interrupt
    
    goto    endISR    

transmitByteT0I:

    movf    LCDBufferPtr,W  ; get pointer to next character to be transmitted
    movwf   FSR0L           ; point FSR at the character
    rlf     INDF0,F         ; get the first bit to transmit

    banksel SERIAL_OUT_P

    bcf     SERIAL_OUT_P,SERIAL_OUT ; set data line low first (brief low if bit is to be a one will
                                    ; be ignored by receiver)
    btfsc   STATUS,C
    bsf     SERIAL_OUT_P,SERIAL_OUT ; set high if bit was a 1

    banksel LCDFlags

endOfByteCheck:

    decfsz  LCDBitCount,F       ; count number of bits to transmit
    goto    endISR

    bsf     LCDFlags,stopBit    ; signal to transmit stop bit on next interrupt

    incf    LCDBufferPtr,F      ; point to next character in buffer

    decfsz  LCDBufferCnt,F      ; buffer empty?
    goto    endISR

    bsf     LCDFlags,endBuffer  ; signal to stop transmitting after next stop bit

    goto    endISR

; end of handleTimer0Interrupt
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; flipSign
;
; Flips the sign (positive to negative or vice verse) of the word in scratch1:scratch0
;
; On entry:
;
; var scratch1:scratch0 = word to be flipped
;
; Returns flipped word in scratch1:scratch0
;
; Uses W, scratch0, scratch1
;

flipSign:

    comf    scratch0,F
    comf    scratch1,F
    incf    scratch0,F				; see note "incf vs decf rollover"
    btfsc   STATUS,Z		
    incf    scratch1,F
    return

; end of flipSign
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; scanButtons
;
; Scans the button inputs, returning their state in buttonState and providing debouncing delay.
;
; There are two entry points, scanButtons and scanButtonsQ:
;
;  Entry at scanButtons first performs a long delay which debounces the switch if it was pressed
;  recently and gives the user time to release the button before it triggers again.
;
;  Entry at scanButtonsQ performs a shorter delay for use by functions which already have some
;  built in delay.
;
; On entry: no values required
;
; On return button states in buttonState, previous state of buttons in buttonPrev.
;
; buttonState & buttonPrev bit assignments:
;
; bit 3: 0 = Reset/Zero/Enter button pressed
; bit 4: 0 = Jog Up button pressed
; bit 5: 0 = Jog Down button pressed
; bit 6: 0 = Mode button active
;
; Uses W, buttonState, scratch0, scratch1, scratch2, scratch3
;

scanButtons:

; delay here in case button was previously pushed to provide debounce
; delay before checking the button so the response will be instant
; works the same before or after for debounce purposes

; a button input has changed, so delay to debounce

    movlw   0x0
    movwf   scratch1
    movlw   0xff
    call    bigDelayA       ; delay a longer time - give user chance to release button

    goto    skipSB1
    
scanButtonsQ:

    ; short delay

    movlw   0x0
    movwf   scratch1
    movlw   0x1
    call    bigDelayA       ; delay a short time - calling function is expected to delay some also

skipSB1:

    movf    buttonState,W  ; store the previous state of the buttons
    movwf   buttonPrev

    clrw
    movwf   buttonState    ; start with all states 0

;test each port input and set button_state to match

    btfsc   SELECT_SW_P,SELECT_SW
    bsf     buttonState,SELECT_SW_STATE

    btfsc   JOG_UP_SW_P,JOG_UP_SW
    bsf     buttonState,JOG_UP_SW_STATE

    btfsc   JOG_DWN_SW_P,JOG_DWN_SW
    bsf     buttonState,JOG_DWN_SW_STATE

    btfsc   MODE_SW_P,MODE_SW
    bsf     buttonState,MODE_SW_STATE

    return

; end of scanButtons
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; zeroQuad
;
; Zeroes the 4 byte variable addressed by value in W.
;
; On entry:
;
; W contains address of first byte of the variable.
;
; Uses FSR, W
;

zeroQuad:

    movwf   FSR0L           ; point FSR to first byte
    clrw                    ; W = 0

    movwf  INDF0            ; clear each byte
    incf   FSR0L,F
    movwf  INDF0            ; clear each byte
    incf   FSR0L,F
    movwf  INDF0            ; clear each byte
    incf   FSR0L,F
    movwf  INDF0            ; clear each byte

    return

; end of zeroQuad
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; doExtModeMenu
;
; Displays and handles the Standard / Extended depth mode menu.
;
; If doExtModeMenu is called, the menu is displayed with option 1 highlighted.
; If doExtModeMenuA is called, the cursorPos and menuOption should be preloaded by the calling
;  function to set the default highlighted option.
;
; Menu display:
;
; "OPT AutoNotcher Rx.x" 
;
; 0x1, 0xC0
; "CHOOSE CONFIGURATION"
;
; 0x1, 0x94
; "1 - EDM Notch Cutter"
;
; 0x1, 0xd4
; "2 - EDM Extend Reach"
;
; 0x1, 0x94
; Carriage Return
;
; It then processes user input.
;
; Uses W, scratch0, scratch1, scratch2, scratch3, scratch4, scratch5, scratch6 (wip - needs updating)
;
; NOTE: LCD addressing is screwy - second line first column is 0xC0, third line is 0x94,
;       fourth line is 0xd4.
;

doExtModeMenu:

	; call here to default to option 1

    movlw   0x94
    movwf   cursorPos       ; option 1 highlighted
    movlw   0x1
    movwf   menuOption      ; option 1 currently selected

doExtModeMenuA:				; call here if default option has already been set by caller

;print the strings of the menu

    movlw   .0              ; "OPT AutoNotcher Rx.x"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed

    movlw   0xc0
    call    writeControl    ; position at line 2 column 1
    movlw   .1              ; "CHOOSE CONFIGURATION"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed
    
    movlw   0x94
    call    writeControl    ; position at line 3 column 1
    movlw   .2              ; "1 - EDM Notch Cutter"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed    

    movlw   0xd4
    call    writeControl    ; position at line 4 column 1
    movlw   .3              ; "2 - EDM Extend Reach"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed

; position the cursor on the default selection
    
    movf    cursorPos,W		; load the cursor position to highlight the current choice
    call    writeControl    ; write line 3 column 1
	call	turnOnBlink
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

; scan for button inputs, highlight selected option, will return when Reset/Enter/Zero pressed

LoopDEMM1:

	bcf		menuOption,7		; clear the menu page change flags
	bcf		menuOption,6

    movlw   .02                 ; maximum menu option number
    call    handleMenuInputs

; parse the selected option ---------------------------------------------------

    movf    menuOption,W       
    movwf   scratch0

    decfsz  scratch0,F
    goto    skipDEMM6

    ; handle option 1 - non-extended reach cutting head installed

    bcf     flags,EXTENDED_MODE ; set flag to 0

    movlw   STANDARD_RATIO1
    movwf   ratio1           
    movlw   STANDARD_RATIO0
    movwf   ratio0           ; sets the number of motor steps per 0.001"
    
    ; standard head mode values

    movlw   .32
    movwf   setupDelay
    movlw   .100
    movwf   normDelay

    goto    exitDEMM7
   
skipDEMM6:

    decfsz  scratch0,F
    goto    skipDEMM7

    ; handle option 2 - extended reach cutting head installed

    bsf     flags,EXTENDED_MODE ; set flag to 1

    movlw   EXTENDED_RATIO1
    movwf   ratio1          ; sets the number of motor steps per 0.001"
    movlw   EXTENDED_RATIO0
    movwf   ratio0          ; sets the number of motor steps per 0.001"

    ; extended head values

    movlw   .4
    movwf   setupDelay
    movlw   .12
    movwf   normDelay
    
    goto    exitDEMM7

skipDEMM7:
	
	; this part reached if a menu page change flag bit is set in menuOption
	; since there is only one page for this menu, ignore
	goto	LoopDEMM1

exitDEMM7:

	; store the negative of ratio1:ratio0 in ratio_neg1:ratio_neg0
	; the negative number is used to catch match when pre-scaler goes negative

    movf    ratio1,W
    movwf   ratio_neg1
    movf    ratio0,W
    movwf   ratio_neg0

    comf    ratio_neg0,F
    comf    ratio_neg1,F
    incf    ratio_neg0,F		; see note "incf vs decf rollover"
    btfsc   STATUS,Z
    incf    ratio_neg1,F
    
    return    

; end of doExtModeMenu
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; doMainMenu
;
; Displays and handles the main menu page 1.  If the user moves cursor down while on bottom option
; page 2 is displayed.
;
; If doMainMenu is called, the menu is displayed with option 1 highlighted.
; If doMainMenuA is called, the cursorPos and menuOption should be preloaded by the calling
;  function to set the default highlighted option.
;
; Screen displayed:
;
; "OPT EDM Notch Cutter"
;
; 0x1, 0xc2
; "1 - Set Cut Depth" or "1 - Depth = "
;
; 0x1, 0x96
; "2 - Cut Notch"
;
; 0x1, 0xd6
; "3 - Jog Electrode"
;
; 0x1, 0xc2
; Carriage Return (to place cursor on first option)
;
; It then processes user input.
;
; Uses W, scratch0, scratch1, scratch2, scratch3, scratch4, scratch5, scratch6, scratch7, scratch8
;
; NOTE: LCD addressing is screwy - second line first column is 0xC0, third line is 0x94,
;       fourth line is 0xd4.
;

doMainMenu:

	; call here to default to option 1

    movlw   0xc0
    movwf   cursorPos       ; option 1 highlighted
    movlw   0x1
    movwf   menuOption      ; option 1 currently selected

doMainMenuA:				; call here if default option has already been set by caller

;print the strings of the menu

    call    clearScreen     ; clear the LCD screen (next print will flush this to LCD)
    
    btfsc  flags,EXTENDED_MODE  ; check for extended mode
    goto    extendedModeDMM

; display the menu header for standard mode

    movlw   .4              ; "OPT EDM Notch Cutter"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed
    goto    skipDMM    

extendedModeDMM:

; display the menu header for extended mode

    movlw   .13             ; "OPT EDM Extend Reach"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed

skipDMM:

; if depth not already set (=0), display this string

    movlw   0xc0
    call    writeControl    ; position at line 2 column 1

    movlw   depth3
    call    isZero          ; is depth variable zero?
    btfss   STATUS,Z
    goto    skipString5     ; if not zero, jump to display the depth

    movlw   .5              ; "1 - Set Cut Depth"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed
    
    goto    skipString6;

; if depth already set (!=0), display the depth value

skipString5:

    movlw   .6              ; "1 - Depth = "
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed
    
    movlw   depth3
    call    displayBCDVar   ; display the depth to cut
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

skipString6:

    movlw   0x94
    call    writeControl    ; position at line 3 column 1

    movlw   .7              ; "2 - Cut Notch"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed    

    movlw   0xd4
    call    writeControl    ; position at line 4 column 1

    movlw   .8              ; "3 - Jog Electrode"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed    

;position the cursor on the default selection

    movf    cursorPos,W		; load the cursor position to highlight the current choice
    call    writeControl    ; position at line 2 column 1
	call	turnOnBlink
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

loopDMM1:

	bcf		menuOption,7		; clear the menu page change flags
	bcf		menuOption,6

; scan for button inputs, highlight selected option, will return when Reset/Enter/Zero pressed

    movlw   .03                 ; maximum menu option number
    call    handleMenuInputs

; parse the selected option ---------------------------------------------------

    movf    menuOption,W       
    movwf   scratch0

    decfsz  scratch0,F
    goto    skipDMM1

    ; handle option 1 - set cut depth
    
    call    setDepth        ; allow user to adjust the depth value

    goto    doMainMenu      ; repeat main menu

    return 
   
skipDMM1:

    decfsz  scratch0,F
    goto    skipDMM2

    ; handle option 2 - Cut Notch

	; flag is ignored because this was a pain - better to be able to zero after a cut - can start a new
	; cut this way without powering down
    bsf     flags,CUT_STARTED ; set flag that cut started so unit cannot be zeroed

    call    cutNotch		; start the auto notch cut function

    goto    doMainMenu      ; repeat main menu
    
skipDMM2:

    decfsz  scratch0,F
    goto   	skipDMM3

    ; handle option 3 - Jog Electrode

    call    jogMode         ; allow user to manually position the height of the cutting blade

    goto    doMainMenu      ; repeat main menu

skipDMM3:

	btfsc	menuOption,7
	goto	loopDMM1			; no previous menu page, ignore

	btfsc	menuOption,6
	goto    doMainMenuPage2     ; display next menu page

	; this part should never be reached unless there is a programming error
	; just redo the menu in this case

	goto	doMainMenu
    
; end of doMainMenu
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; doMainMenuPage2
;
; Displays and handles the main menu page 2.  If the user moves cursor up while on top option
; page 1 is displayed.
;
; If doMainMenuPage2 is called, the menu is displayed with option 1 highlighted.
; If doMainMenuPage2A is called, the cursorPos and menuOption should be preloaded by the calling
;  function to set the default highlighted option.
;
; Screen displayed:
;
; 0x1, 0xc2
; "4 - Cycle Test"
;
; 0x1, 0x96
; "5 - Motor Dir Normal" or "5 - Motor Dir Reverse"
;
; 0x1, 0xc2
; Carriage Return (to place cursor on first option)
;
; It then processes user input.
;
; Uses W, scratch0, scratch1, scratch2, scratch3, scratch4, scratch5, scratch6, scratch7, scratch8
;
; NOTE: LCD addressing is screwy - second line first column is 0xC0, third line is 0x94,
;       fourth line is 0xd4.
;

doMainMenuPage2:

	; call here to default to option 1

    movlw   0x80
    movwf   cursorPos       ; option 1 highlighted
    movlw   0x1
    movwf   menuOption      ; option 1 currently selected

doMainMenuPage2A:			; call here if first option has already been set by caller

;print the strings of the menu

    call    clearScreen     ; clear the LCD screen (next print will flush this to LCD)

; display the first option

    movlw   .21             ; "4 - " prefix so can re-use "Cycle Test" string
    call    printString     ; print the string
	call    waitLCD         ; wait until buffer printed (can't use printString again before this)
    movlw   .19             ; "Cycle Test"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed

; display the second option

    movlw   0xc0
    call    writeControl    ; position at line 2 column 1

    movlw   .20             ; "5 - Motor Dir " or "5 - Motor Dir "
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed

    btfsc  	flags,MOTOR_DIR_MODE    ; check for reverse motor direction
	goto	revDirDMMP2

    movlw   .22             ; add "Normal" suffix to motor dir line
    call    printString     ; print the string
	call    waitLCD         ; wait until buffer printed (can't use printString again before this)
	goto	placeCursorDMMP2

revDirDMMP2:

    movlw   .23             ; add "Reverse" suffix to motor dir line
    call    printString     ; print the string
	call    waitLCD         ; wait until buffer printed (can't use printString again before this)

placeCursorDMMP2:

;position the cursor on the default selection

    movf    cursorPos,W		; load the cursor position to highlight the current choice
    call    writeControl    ; position at line 1 column 1
	call	turnOnBlink
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

loopDMMP21:

	bcf		menuOption,7		; clear the menu page change flags
	bcf		menuOption,6

; scan for button inputs, highlight selected option, will return when Reset/Enter/Zero pressed

    movlw   .02                 ; maximum menu option number
    call    handleMenuInputs

; parse the selected option ---------------------------------------------------

    movf    menuOption,W       
    movwf   scratch0

    decfsz  scratch0,F
    goto    skipDMMP21

    ; handle option 4 - Cycle Test (menuOption value is 1)
    
    call    cycleTest       ; enter the "Cycle Test" function

    goto    doMainMenuPage2 ; refresh menu
 
skipDMMP21:

    decfsz  scratch0,F
    goto    skipDMMP24

    ; handle option 5 - Motor Direction Change (menuOption value is 2)

    btfss	flags,MOTOR_DIR_MODE    ; motor direction is reverse?
	goto	skipDMMP23

	bcf		flags,MOTOR_DIR_MODE    ; set motor direction to normal
	call    saveFlagsToEEprom       ; save the new setting to EEprom
    goto    doMainMenuPage2         ; refresh menu

skipDMMP23:

	bsf		flags,MOTOR_DIR_MODE	; set motor direction to reverse
    call    saveFlagsToEEprom       ; save the new setting to EEprom
    goto    doMainMenuPage2         ; refresh menu
    
skipDMMP24:

	btfss	menuOption,7
	goto	skipDMMP25

	;go back to previous menu with last option defaulted	
    movlw   0xd4
    movwf   cursorPos       ; last option highlighted
    movlw   0x3
    movwf   menuOption      ; last option currently selected
	goto	doMainMenuA		; display previous menu page

skipDMMP25:

	btfsc	menuOption,6
	goto    loopDMMP21			; no next menu page, ignore

	; this part should never be reached unless there is a programming error
	; just redo the menu in this case

	goto	doMainMenuPage2
    
; end of doMainMenuPage2
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; cutNotch
;
; Automatically cuts the notch.  Moves the cutting blade down until the over-current limit input
; triggers then moves head back up until limit condition is removed.
;
; NOTE: The system monitors the voltage of the cutting supply, NOT the current.  When the voltage
; is high, the current must be low so the blade is lowered.  When the voltage is low, the current
; must be high so the blade is raised.  This makes some of the logic appear to be backwards if
; it is erroneously assumed that the current is being monitored instead of the voltage.
;
; The user can adjust the agressiveness of the advance using the "Jog Up" and "Jog Down" buttons.
; This affects the time between sparks to trigger an advance.
;
; NOTE: If the cutting power supply is turned off, the head will be raised due to the fact that
;       the voltage appears to be low.
;
; On entry:
;
; position contains current height of the cutting blade
; depth contains the desired depth of the cut
;
; Uses W, FSR, PCLATH, TMR0, OPTION_REG, preScaler,
; 	scratch0, scratch1, scratch2, scratch3, scratch4, scratch5, scratch6, scratch7, scratch8
;
; For notch cutting mode switch out the smart code on retract.
;
; For wall reduction cutting mode, switch in the smart code on retract.
; A good value for the overCurrentTimer retract is 1fff (hard coded).
;

cutNotch:

    movlw   0x0
    movwf   scratch1
    movlw   0xff
    call    bigDelayA       ; delay - give user chance to release button
    
    call    clearScreen     ; clear the LCD screen

    movlw   .14             ; "Turn on Cut Voltage"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed    

	call	setupCutNotchAndCycleTest	; finish the screen setup

    bcf     flags,AT_DEPTH  ; clear the depth reached flag
    bsf     flags,UPDATE_DISPLAY ; force display update first time through

    movlw   .0              ; zero the switch debounce timer
    movwf   debounce0
    movwf   debounce1      

    movlw   ' '				; these variables store the direction symbol (an asterisk)
    movwf   scratch7		; for display to show which direction the head is going
    movlw   ' '				; clear them so garbage won't be displayed first time through
    movwf   scratch8

    bsf     POWER_ON_P,POWER_ON    ; turn on the cutting voltage

cutLoop:

    bsf     POWER_ON_P,POWER_ON    ; turn on the cutting voltage repeatedly
                    ; (this is a hack fix because the pin seems to get set low by electrical noise)
                    ; (may not be true for properly grounded PCBs with high voltage caps in place)

    btfsc   flags,UPDATE_DISPLAY
    call    displayPosLUCL  ; update the display if data has been modified

    btfsc   flags,AT_DEPTH  ; displayPosLUCL sets flags:AT_DEPTH if depth reached and time to exit
    goto    exitCN

    btfss   SELECT_SW_P,SELECT_SW
    goto    exitCN          ; exit the notch cutting mode if the reset button pressed


    ; if the delay timer has not reached zero, don't respond to button press

    movf    debounce0,W
    iorwf   debounce1,W
    btfss   STATUS,Z
    goto    checkHiLimit    ; debounce timer not zeroed, don't check buttons

checkUPDWNButtons:

    btfss   JOG_UP_SW_P,JOG_UP_SW
    call    adjustSpeedUp   ; increment the speed (sparkLevel) value

    btfss   JOG_DWN_SW_P,JOG_DWN_SW
    call    adjustSpeedDown ; decrement the speed (sparkLevel) value

checkHiLimit:

    call    sparkTimer
    btfss   STATUS,Z
    goto    checkLoLimit

moveDownLUCL:

; voltage too high (current too low) - move cutting blade down

    movlw   position3
    call    incBCDVar       ; going down increments the position

    movlw   ' '
    movwf   scratch7
    movlw   '*'
    movwf   scratch8        ; display asterisk by "Down" label

    bsf     flags,UPDATE_DISPLAY ; force display update to show change

    call    pulseMotorDownWithDelay  	; move motor one step
                                 		; no delay before stepping because enough time wasted above

checkLoLimit:
 
    btfsc   flags,WALL_MODE         ; no retract smart code for notch, use it for Wall (see header notes)
    goto    wallModeCN

notchModeCN:                        ; next two lines for notch mode
      
    btfsc   LO_LIMIT_P,LO_LIMIT     ; is voltage too low? (current too high)
    goto    cutLoop
    goto    moveUpLUCL              ; time to retract

wallModeCN:                         ; next two lines for wall reduction mode
   
    call    overCurrentTimer
    btfss   STATUS,Z
    goto    cutLoop

moveUpLUCL:

; voltage too low (current too high) - move cutting blade up

    call    pulseMotorUpWithDelay  	; move motor one step - delay to allow motor to move
    
    movlw   position3
    call    decBCDVar				; going up decrements the position

    movlw   '*'
    movwf   scratch7                ; display asterisk by "Up" label
    movlw   ' '
    movwf   scratch8

    bsf     flags,UPDATE_DISPLAY ; force display update to show change when retract is done
								 ; the retract loop never calls the display update code

	bsf		flags,UPDATE_DIR_SYM	; flag that the direction marker needs to changed to "Up"
									; this can't be done immediately because the LDC print
									; service might be busy, so set a flag to trigger the
									; update during the retract loop when the LCD is ready

; enter a fast loop without much overhead to retract quickly until over-current is gone

quickRetractCN:

    btfss   SELECT_SW_P,SELECT_SW
    goto    exitCN          ; exit the notch cutting mode if the reset button pressed

    call    pulseMotorWithDelay    	; move motor one step - delay to allow motor to move
    
    movlw   position3
    call    decBCDVar				; going up decrements the position

	; Because the retract locks into a loop until the over current is cleared, the display
	; update code is never called to show the asterisk by the "Up" label which is confusing.
	; This section waits until the LCD print service is not busy and then does a one time
	; update of the direction symbol.

	btfss	flags, UPDATE_DIR_SYM
	goto    skipDirSymUpdateCN
    banksel LCDFlags
    btfsc   LCDFlags,LCDBusy
	goto    skipDirSymUpdateCN

    banksel flags
	bcf		flags,UPDATE_DIR_SYM	; only update the display one time while looping
 
    movlw   0xc0
    call    writeControl    ; position at line 2 column 1
    movf    scratch7,W
    call    writeChar       ; write asterisk or space by "Up" label
    movlw   0x94
    call    writeControl    ; position at line 3 column 1
    movf    scratch8,W
    call    writeChar       ; write asterisk or space by "Down" label
    call    flushLCD        ; force buffer to print, but don't wait because jogging is time
                            ; critical

skipDirSymUpdateCN:

    banksel LO_LIMIT_P

    btfss   LO_LIMIT_P,LO_LIMIT     ; check again, loop quickly until current is within
    goto    quickRetractCN          ; limits to avoid glow plugging
    
    goto    cutLoop

displayPosLUCL:             		; updates the display if it is time to do so

    banksel LCDFlags

    btfss   LCDFlags,LCDBusy
    goto    displayCN
    goto    checkPositionCN 		; don't display if print buffer not ready      

displayCN:

    banksel flags

    bcf     flags,UPDATE_DISPLAY 	; clear flag so no update until data changed again

    ; display asterisk at "Up" or "Down" label depending on blade direction or erase if no movement

    movlw   0xc0
    call    writeControl    ; position at line 2 column 1
    movf    scratch7,W
    call    writeChar       ; write asterisk or space by "Up" label
    movlw   0x94
    call    writeControl    ; position at line 3 column 1
    movf    scratch8,W
    call    writeChar       ; write asterisk or space by "Down" label

    call    displaySpeed    ; display the current advance speed value (sparkTimer level)

    movlw   0xdf
    call    writeControl    ; position in desired location
    call    displayPos      ; display the location of the head relative to the zero point
    call    flushLCD        ; force buffer to print, but don't wait because jogging is time
                            ; critical
    
    ; compare position with desired cut depth, exit when reached

checkPositionCN:

    banksel depth3

    movlw   depth3
    call    isPosGtYQ

    bcf     flags,AT_DEPTH
    btfsc   STATUS,C
    bsf     flags,AT_DEPTH  ; if isPosGTYP returned C=1, depth reached so set flag

    return

exitCN:

    bcf     POWER_ON_P,POWER_ON    ; turn off the cutting voltage

    call    waitLCD         ; wait until buffer printed

    btfsc   flags,DATA_MODIFIED     ; if data has been modified, save to eeprom
    call    saveSparkLevelsToEEprom

    bcf     flags,DATA_MODIFIED

    return

; end of cutNotch
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupCutNotchAndCycleTest
;
; Prepares the screen for the cutNotch and cycleTest functions.  These share nearly identical
; screens.
;

setupCutNotchAndCycleTest:

    movlw   0xc1
    call    writeControl    ; position at line 2 column 2

    movlw   .15
    call    printString     ; "Up   Speed>"
    call    waitLCD         ; wait until buffer printed

    call    displaySpeed    ; display the current advance speed value (sparkTimer level)
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

    movlw   0x95
    call    writeControl    ; position at line 3 column 2

    movlw   .16
    call    printString     ; "Down  Stop>"
    call    waitLCD         ; wait until buffer printed    

    movlw   depth3
    call    displayBCDVar   ; display the depth to cut

    movlw   0x22
    call    writeChar       ; write '"' for inch mark

    movlw   0xe5
    call    writeControl    ; position at line 4 column 18
    movlw   0x22
    call    writeChar       ; write '"' for inch mark

    movlw   0xdf
    call    writeControl    ; position at line 4 column 12
    call    displayPos      ; display the location of the head relative to the zero point

    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

	return

; end of setupCutNotchAndCycleTest
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; cycleTest
;
; Performs a test to verify the proper operation of the system.  The head is lowered until the
; high current limit is inactive - this occurs when the blade nearly touches the pipe and
; current begins to flow.  The head is then retracted back to the original position.  This cycle
; is repeated until the user exits the test.  With each touchdown, only a tiny cut is made so the
; depth should only change very slowly after many touchdowns.
;
; For the best test, the operator should use the jog mode to raise the head as high as possible
; so that the maximum range is used in each cycle.
;
; NOTE: The system monitors the voltage of the cutting supply, NOT the current.  When the voltage
; is high, the current must be low so the blade is lowered.  When the voltage is low, the current
; must be high so the blade is raised.  This makes some of the logic appear to be backwards if
; it is erroneously assumed that the current is being monitored instead of the voltage.
;
; Although the target depth to cut and the cutting speed labels are displayed, they are not
; used by this function.
;
; NOTE: If the cutting power supply is turned off, the head will be raised due to the fact that
;       the voltage appears to be low.
;
; On entry:
;
; Uses W, FSR, PCLATH, TMR0, OPTION_REG, preScaler,
; 	scratch0, scratch1, scratch2, scratch3, scratch4, scratch5, scratch6, scratch7, scratch8
;

cycleTest:
    
    call    clearScreen     ; clear the LCD screen

    movlw   .19             ; "Cycle Test"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed    

	call	setupCutNotchAndCycleTest	; finish the screen setup

    bsf     flags,UPDATE_DISPLAY ; force display update first time through

    movlw   ' '				; these variables store the direction symbol (an asterisk)
    movwf   scratch7		; for display to show which direction the head is going
    movlw   ' '				; clear them so garbage won't be displayed first time through
    movwf   scratch8

    bsf     POWER_ON_P,POWER_ON    ; turn on the cutting voltage

    movlw   0x3
    movwf   scratch1
    movlw   0xe8
    call    bigDelayA       ; delay to give power supply a chance to come up

restartCycleCT:

;debug mks - replace with scratch variables
    movlw   .0				; clear the cycle distance counter to track distance lowered
    movwf   debug0
	movwf   debug1
;debug mks

cycleLoopCT:

    bsf     POWER_ON_P,POWER_ON    ; turn on the cutting voltage repeatedly
                            ; (this is a hack fix because the pin seems to get set low by electrical noise)
							; (may not be true for properly grounded PCBs with high voltage caps in place)

    btfsc   flags,UPDATE_DISPLAY
    call    displayPosLUCL  ; update the display if data has been modified

    btfss   SELECT_SW_P,SELECT_SW
    goto    exitCT          ; exit the notch cutting mode if the reset button pressed

checkHiLimitCT:

    btfsc   HI_LIMIT_P,HI_LIMIT     ; is voltage too high? (current too low, not touching yet)
    goto    upCycleCT             	; voltage drop - blade touching - begin up cycle
									; unlike the cutNotch function, the blade never sits still

moveDownCT:

; voltage too high (current too low, not touching yet) - move cutting blade down

	;count how many ticks the blade travels down before touchdown - on the up cycle this
	;count will be used to stop the blade in approximately the original start position
	;each time - as a cut is made, the up and down positions will move down slowly
	incf	debug0,F			; increment low byte ~ see note "incf vs decf rollover"
    btfsc   STATUS,Z
	incf	debug1,F			; increment high byte

    movlw   position3
    call    incBCDVar       ; going down increments the position

    movlw   ' '
    movwf   scratch7
    movlw   '*'
    movwf   scratch8        ; display asterisk by "Down" label

    bsf     flags,UPDATE_DISPLAY ; force display update to show change

    call    pulseMotorDownWithDelay	; move motor one step
                                 	; no delay before stepping because enough time wasted above

	goto	cycleLoopCT

upCycleCT:

; voltage too low (current too high - blade touching) - move cutting blade up

    movlw   '*'
    movwf   scratch7                ; display asterisk by "Up" label
    movlw   ' '
    movwf   scratch8

    bsf     flags,UPDATE_DISPLAY ; force display update to show change when retract is done
								 ; the retract loop never calls the display update code

	bsf		flags,UPDATE_DIR_SYM	; flag that the direction marker needs to changed to "Up"
									; this can't be done immediately because the LDC print
									; service might be busy, so set a flag to trigger the
									; update during the retract loop when the LCD is ready

; enter a fast loop without much overhead to retract quickly until head returns to starting position

quickRetractCT:

    btfss   SELECT_SW_P,SELECT_SW
    goto    exitCT          ; exit the notch cutting mode if the reset button pressed

    call    pulseMotorUpWithDelay  	; move motor up one step - delay to allow motor to move
    
    movlw   position3
    call    decBCDVar				; going up decrements the position

	; Because the retract locks into a loop until the starting position is reached, the display
	; update code is never called to show the asterisk by the "Up" label which is confusing.
	; This section waits until the LCD print service is not busy and then does a one time
	; update of the direction symbol.

	btfss	flags, UPDATE_DIR_SYM
	goto    skipDirSymUpdateCT

    banksel LCDFlags

    btfsc   LCDFlags,LCDBusy
	goto    skipDirSymUpdateCT

    banksel flags
	bcf		flags,UPDATE_DIR_SYM	; only update the display one time while looping
 
    movlw   0xc0
    call    writeControl    ; position at line 2 column 1
    movf    scratch7,W
    call    writeChar       ; write asterisk or space by "Up" label
    movlw   0x94
    call    writeControl    ; position at line 3 column 1
    movf    scratch8,W
    call    writeChar       ; write asterisk or space by "Down" label
    call    flushLCD        ; force buffer to print, but don't wait because jogging is time
                            ; critical

skipDirSymUpdateCT:

    banksel debug0

	; user counter to return blade to original start position

    movlw   .1	            ; decrement LSByte
    subwf   debug0,F		; see note "incf vs decf rollover"
    btfss   STATUS,C		; did LSByte roll under (0->255)?
	decf	debug1,F		; decrement MSByte after LSByte roll under
	movf	debug0,W		; check MSB:LSB for zero
	iorwf	debug1,W
	btfsc	STATUS,Z
	goto    restartCycleCT	; restart loop when counter reaches zero

    goto    quickRetractCT
	
exitCT:

    bcf     POWER_ON_P,POWER_ON    ; turn off the cutting voltage

    call    waitLCD         ; wait until buffer printed

    return

; end of cycleTest
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; sparkTimer
;
; Tracks time between sparking of the blade.  Uses high limit comparator input - if this signal
; is 0, then voltage and current are good - if 1 then the blade needs to move down.  If the blade
; moves down every time this signal goes to 1, it advances too quickly - it should only advance
; when no spark has occurred for a specified time.
;
; The timer counts down and is reset any time the voltage/current are good.  If the timer makes
; it to zero without being reset, then an advance is required.
;
; Returns Z = 0 if not time to advance, Z = 1 if time to advance (lower blade).
;
; A smaller value makes the advance more aggressive.
;

sparkTimer:

    btfsc   HI_LIMIT_P,HI_LIMIT     ; is voltage too high? (current too low)
    goto    noAdvanceST             ; voltage is good, don't advance

; check the advance timer

    bcf     STATUS,Z        ; preset flag - decfsz does not affect Z

    decfsz  sparkTimer0,F   ; count down
    return                  ; returns Z = 0, no advance due to not timed out
    decfsz  sparkTimer1,F
    return                  ; returns Z = 0, no advance due to not timed out

    movlw   0x01            ; NOTE - don't use less than 0x01 (0 = 0xff)
    movwf   sparkTimer0
    movf    sparkLevel,W    ; NOTE - don't use less than 0x01 (0 = 0xff)
    movwf   sparkTimer1
   
    bsf     STATUS,Z        ; return Z = 1, blade advance required

    return

noAdvanceST:

    ; voltage is good, so reset timer

    movlw   0x01            ; NOTE - don't use less than 0x01 (0 = 0xff)
    movwf   sparkTimer0
    movf    sparkLevel,W    ; NOTE - don't use less than 0x01 (0 = 0xff)
    movwf   sparkTimer1

    bcf     STATUS,Z        ; return Z = 0, no blade advance

    return

; end of sparkTimer
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; overCurrentTimer
;
; Tracks time between over-current spikes.  Uses low limit comparator input - if this signal
; is 0, then voltage and current are good - if 1 then the blade needs to up .  If the blade
; moves up every time this signal goes to 1, it rises too quickly - it should only retract
; when an overvoltage has occurred for a specified time.
;
; The timer counts down and is reset any time the voltage/current are good.  If the timer makes
; it to zero without being reset, then a retract is required.
;
; Returns Z = 0 if not time to retract, Z = 1 if time to retract (raise blade).
;
; A smaller value makes the retract more responsive - will retract more often.
;
; Further explanation:
; The system has to be in overcurrent for the entire duration of the timer to force a retract.  This
; prevents retracts on short spikes of overcurrent.
;

overCurrentTimer:

    btfsc   LO_LIMIT_P,LO_LIMIT     ; is voltage too low? (current too high)
    goto    noRetractOCT            ; voltage is good, don't retract

; check the retract timer

    bcf     STATUS,Z        ; preset flag - decfsz does not affect Z

    decfsz  overCurrentTimer0,F	; count down
    return             		    ; returns Z = 0, no retract due to not timed out
    decfsz  overCurrentTimer1,F
    return                  	; returns Z = 0, no retract due to not timed out

    movlw   0xff            ; NOTE - don't use less than 0x01 (0 = 0xff)
    movwf   overCurrentTimer0
    movlw   0x1f            ; NOTE - don't use less than 0x01 (0 = 0xff)
                            ; ver 7.6b = 0
    movwf   overCurrentTimer1
   
    bsf     STATUS,Z        ; return Z = 1, blade retract required

    return

noRetractOCT:

    ; voltage is good, so reset timer

    movlw   0xff            ; NOTE - don't use less than 0x01 (0 = 0xff)
    movwf   overCurrentTimer0
    movlw   0x1f            ; NOTE - don't use less than 0x01 (0 = 0xff)
                            ; ver 7.6b = 0
    movwf   overCurrentTimer1

    bcf     STATUS,Z        ; return Z = 0, no blade advance

    return

; end of overCurrentTimer
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; adjustSpeedUp and adjustSpeedDown
;
; Call adjustSpeedUp if jog up button toggled, adjustSpeedDown if jog down button toggled.
;
; If jog up then speed is increased by one, rolling from 9 to 1 if appropriate.
; If jog down then speed is decreased by one, rolling from 1 to 9 if appropriate.
;
; Uses W, speedValue, sparkLevel, sparkLevelNotch, sparkLevelWall
;
; speedValue contains 1-9 to represent sparkLevel 0x01-0x81
;
; In actual use, the sparkLevel is used as the upper byte of the actual value: sparkLevel:0x01
;

adjustSpeedUp:

; jog up button press    

    incf    speedValue,F    ; increment the value
    
    movlw   .10
    subwf   speedValue,W    ; check if 10 reached
    btfss   STATUS,Z
    goto    updateAS        ; display the digit

    movlw   .1
    movwf   speedValue      ; roll around to 1 after 9

    goto    updateAS        ; display the digit

adjustSpeedDown:

; jog down button press

    decf    speedValue,F    ; decrement the value
    
    movlw   .0
    subwf   speedValue,W    ; check if less than 1
    btfss   STATUS,Z
    goto    updateAS        ; display the digit

    movlw   .9
    movwf   speedValue      ; roll around to 9 after 1

updateAS:

; update and save to eeprom the new value but do not display the new value - will be handled
; by the cutNotch function

    btfsc   flags,WALL_MODE ; Notch or Wall mode?
    goto    wallModeAS

    movlw   sparkLevelNotch ; transfer value to Notch variable
    movwf   FSR0L

    goto    processValueAS

wallModeAS:

    movlw   sparkLevelWall  ; transfer value to Notch variable
    movwf   FSR0L

processValueAS:

    ; convert speedValue from 1-9 to 0x01-0x81 and store in sparkLevel and Notch or Wall
    ; variable (pointed by FSR)

    movlw   0x01
    subwf   speedValue,W
    movwf   sparkLevel
    rlf     sparkLevel,F    ; move low nibble to high
    rlf     sparkLevel,F
    rlf     sparkLevel,F
    rlf     sparkLevel,F
    movf    sparkLevel,W    ; get the rotated value
    andlw   0xf0            ; mask off lower nibble
    addlw   0x01            ; set lower nibble
    movwf   sparkLevel      ; store value in sparkLevel
    movwf   INDF0           ; store value in appropriate variable
 
    bsf     flags,DATA_MODIFIED ; set flag so values will be saved

    bsf     flags,UPDATE_DISPLAY ; set flag so display will be updated

    movlw   0x45            ; start the switch debounce timer at 0x1245
    movwf   debounce0
    movlw   0x12
    movwf   debounce1

    return

; end of adjustSpeedUp and adjustSpeedDown
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; pulseMotorDownWithDelay
;
; Moves the motor down one click, delaying as necessary for proper motor operation.
;
; Whether the direction bit is set or cleared depends upon the state of the Normal/Reverse
; direction flag.  This option can be set by the user to adjust for systems with different wiring.
;

pulseMotorDownWithDelay:

    bsf     MOTOR_DIR_P,MOTOR_DIR       ; motor down for normal direction option
	btfsc	flags,MOTOR_DIR_MODE        ; is motor direction option reverse?
    bcf     MOTOR_DIR_P,MOTOR_DIR       ; motor up for reverse direction option

	goto	pulseMotorWithDelay

; end of pulseMotorDownWithDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; pulseMotorUpWithDelay
;
; Moves the motor up one click, delaying as necessary for proper motor operation.
;
; Whether the direction bit is set or cleared depends upon the state of the Normal/Reverse
; direction flag.  This option can be set by the user to adjust for systems with different wiring.
;

pulseMotorUpWithDelay:

    bcf     MOTOR_DIR_P,MOTOR_DIR       ; motor up for normal direction option
	btfsc	flags,MOTOR_DIR_MODE        ; is motor direction option reverse?
    bsf     MOTOR_DIR_P,MOTOR_DIR       ; motor up for reverse direction option
	goto	pulseMotorWithDelay

; end of pulseMotorUpWithDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; pulseMotorWithDelay
;
; Delays and then pulses the motor.  The motor will not respond if there is not enough delay
; between pulses.
;
; For no delay in cases where enough time is wasted between pulses, call pulseMotorNoDelay.
;
; On entry:
;
; Desired motor direction bit set, ie:
;       bcf     MOTOR,DIR_SEL
;   or
;       bsf     MOTOR,DIR_SEL
;
; Uses W, FSR, PCLATH, scratch0, scratch1, scratch2, scratch3
;
; NOTE: The delay value of .15 worked for both standard and extended heads.  The cut rate
; for the extended head was about 50% too slow compared to desired rate of 0.001 per minute.
; The burn was very consistent and customer wanted to try paralleling the power supplies
; to increase burn rate rather than experiment with program changes.  Orginally, this delay
; value was loaded from normDelay.  If it is decided that each head needs a different value,
; a new variable needs to be created (ex: cutDelay) and loaded specifically for each head type
; because normDelay and setupDelay are already used to control the speeds in jog mode.  The values
; for jog mode may not work for cut mode because of the difference in the code exec time.
;

pulseMotorWithDelay:

    movlw   0x0
    movwf   scratch1
    ;movf    normDelay,W
    movlw   .15             ; see notes in header regarding this value (use .15 for normal head)
    call    bigDelayA

    bcf     MOTOR_STEP_P,MOTOR_STEP
    nop
    nop
    bsf     MOTOR_STEP_P,MOTOR_STEP ; pulse motor controller step line to advance motor one step

    return

pulseMotorNoDelay:

    bcf     MOTOR_STEP_P,MOTOR_STEP
    nop
    nop
    bsf     MOTOR_STEP_P,MOTOR_STEP ; pulse motor controller step line to advance motor one step

    return

; end of pulseMotorAndDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setDepth
;
; This function allows the user to set the depth of the cut.
; 
; Uses W, FSR, PCLATH, TMR0, OPTION_REG, cursorPos, depth, buttonState, buttonPrev,
; 	scratch0, scratch1, scratch2, scratch3, scratch4, scratch5, scratch6, scratch7
; 

setDepth:

; set up the display

    call    clearScreen     ; clear the LCD screen

    movlw   .9              ; "Set Cut Depth"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed

    movlw   0xc4
    call    writeControl    ; position at line 2 column 4
    movlw   .10             ; "0.000 inches"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed

    movlw   0x94
    call    writeControl    ; position at line 3 column 1

    movlw   .17             ; print "Notch Mode" if in notch mode
    btfsc   flags,WALL_MODE ; which mode?
    movlw   .18             ; print "Wall Mode" if in wall mode
   
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed

    movlw   0xc4
    call    writeControl    ; position back at line 2 column 4

    movlw   depth3
    call    displayBCDVar   ; display the variable over the "0.000 inches" string so it can be edited

    movlw   0xc4
    call    writeControl
	call	turnOnBlink
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

; handle editing

    movlw   depth3          
    movwf   scratch7        ; first character being edited
    
    movlw   0xc4
    movwf   cursorPos       ; cursor position

loopSD:

    call    adjustBCDDigit  ; handle editing of the digit
        
    incf    scratch7,f      ; point to the next digit

    movlw   depth0
    addlw   .1
    subwf   scratch7,W
    btfsc   STATUS,Z
    goto    endSD           ; exit if past the last digit in the variable

    movlw   depth2
    subwf   scratch7,W
    btfsc   STATUS,Z
    incf    cursorPos,F     ; if on second digit, move cursor twice to skip decimal point

    incf    cursorPos,F     ; move the cursor to the next digit
    
    movf    cursorPos,W
    call    writeControl
    call    turnOnBlink
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

    goto    loopSD

endSD:

    call    saveDepthValueToEEprom  ; save the value stored for depth in the EEProm

    goto    setCutMode

; end of setDepth
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setCutMode
;
; This function allows the user to select the Notch or Wall Reduction mode.  The appropriate
; string is displayed on line 3 (Notch or Wall Mode) and the first character is highlighted. If
; the up or down switch is pressed (or toggled), the mode will flip.  Pressing the Reset/Enter
; button will save the mode and exit.
; 
; wip mks - update Uses W, FSR, PCLATH, TMR0, OPTION_REG, cursorPos, depth, buttonState, buttonPrev,
; 	scratch0, scratch1, scratch2, scratch3, scratch4, scratch5, scratch6, scratch7
; 

setCutMode:

    movlw   0x94
    call    writeControl    ; position at line 3 column 1

    movlw   .17             ; print "Notch Mode" if in notch mode
    btfsc   flags,WALL_MODE ; which mode?
    movlw   .18             ; print "Wall Mode" if in wall mode
   
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed

    movlw   0x94
    call    writeControl
	call	turnOnBlink
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

loopSCM:

    call    scanButtons     ; watch for user input

    btfsc   buttonState,JOG_UP_SW_STATE
    goto    skip_upSCM      ; skip if Up switch not pressed

; jog up button press    

    movlw   b'00001000'     ; flip the Notch/Wall cut mode flag
    xorwf	flags,F

    goto    setCutMode      ; update the display

skip_upSCM:

    btfsc   buttonState,JOG_DWN_SW_STATE
    goto    skip_dwnSCM     ; skip if Down switch not pressed

; jog down button press

    movlw   b'00001000'     ; flip the Notch/Wall cut mode flag
    xorwf	flags,F
    
    goto    setCutMode      ; update the display

skip_dwnSCM:

    btfsc   buttonState,SELECT_SW_STATE
    goto    loopSCM             ; loop without updating display if no button pressed

; set sparkLevel to value for Notch or Wall mode depending on current mode

    movf    sparkLevelNotch,W   ; use Notch mode value if in Wall Reduction mode
    btfsc   flags,WALL_MODE
    movf    sparkLevelWall,W    ; use Wall mode value if in Wall Reduction mode
    movwf   sparkLevel          ; save the selected value

    call    saveFlagsToEEprom   ; save the flags variable to eeprom so that the mode will be
                                ; recalled on power up

    return                  ; return when Reset/Enter/Zero button pressed

; end of setCutMode
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; adjustBCDDigit
;
; This function allows the user to adjust the value of a BCD digit.
;
; On entry
;
; scratch7 = memory address of the digit
; cursorPos = screen location of the digit
;
; Uses W, FSR, buttonState, buttonPrev, cursorPos
;     scratch0, scratch1, scratch2, scratch3, scratch7
;

adjustBCDDigit:

loopABD:

    call    scanButtons     ; watch for user input

    btfsc   buttonState,JOG_UP_SW_STATE
    goto    skip_upABD      ; skip if Up switch not pressed

; jog up button press    

    movf    scratch7,W
    movwf   FSR0L
    incf    INDF0,F         ; increment the digit
    
    movlw   .10
    subwf   INDF0,W         ; check if 10 reached
    btfss   STATUS,Z
    goto    updateABD       ; display the digit

    movlw   .0
    movwf   INDF0           ; roll around to 0 after 9

    goto    updateABD       ; display the digit

skip_upABD:

    btfsc   buttonState,JOG_DWN_SW_STATE
    goto    skip_dwnABD    ; skip if Down switch not pressed

; jog down button press

    movf    scratch7,W
    movwf   FSR0L
    decf    INDF0,F         ; decrement the digit
    
    movlw   0xff            
    subwf   INDF0,W         ; check if less than 0
    btfss   STATUS,Z
    goto    updateABD       ; display the digit

    movlw   .9
    movwf   INDF0           ; roll around to 9 after 0

    goto    updateABD       ; display the digit

skip_dwnABD:

    btfsc   buttonState,SELECT_SW_STATE
    goto    loopABD        ; loop if Reset/Select switch not pressed

; reset/enter/zero button press - digit finished, so exit

    return                  ; return when Reset/Enter/Zero button pressed

updateABD:

; update the character on the screen

    movf    cursorPos,W
    call    writeControl    ; prepare to overwrite the digit
    
    movf    scratch7,W
    movwf   FSR0L
    movf    INDF0,W
    addlw   0x30            ; convert to ASCII
    call    writeChar       ; write the digit

    movf    cursorPos,W
    call    writeControl    ; 
	call	turnOnBlink

    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done
    
    goto    loopABD         ; continue editing
    
; end of adjustBCDDigit
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; jogMode
;
; This function allows the user to manually adjust the height of the cutting blade using
; the Up/Down buttons.  If the Mode switch is in "Normal" position, the head moves slowly, if in
; "Setup" position, the head moves more quickly.  In "Normal" mode, the Reset/Enter/Zero button
; exits the Jog Mode, in "Setup" mode, the Reset/Enter/Zero button zeroes the displayed position
; of the height.
; 
; Uses W, FSR, PCLATH, TMR0, OPTION_REG, cursorPos, flags, position buttonState, buttonPrev, preScaler
; 	scratch0, scratch1, scratch2, scratch3, scratch4, scratch5, scratch6
;

jogMode:

; set up the display

    call    clearScreen     ; clear the LCD screen

    movlw   0x86
    call    writeControl    ; position at line 1 column 6
    movlw   .11             ; "Jog Mode"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed

    movlw   0xc4
    call    writeControl    ; position at line 2 column 4
    movlw   .12             ; "Zero or Exit"
    call    printString     ; print the string
    call    waitLCD         ; wait until buffer printed

    movlw   0xe5
    call    writeControl    ; position at line ? column 6
    movlw   0x22
    call    writeChar       ; write '"' for inch mark

    movlw   0xdf
    call    writeControl    ; position in desired location
    call    displayPos      ; display the location of the head relative to the zero point
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

    movlw   0x0
    movwf   scratch1
    movlw   0xff
    call    bigDelayA       ; delay - give user chance to release button

    bsf     POWER_ON_P,POWER_ON    ; turn on the cutting voltage
    
loopJM:

    call    scanButtonsQ    ; monitor user input - Q entry point for short delay so motor runs
                            ; faster

    btfsc   buttonState,JOG_UP_SW_STATE
    goto    chk_dwnJM       ; skip if Up switch not pressed

; jog up button press    

    bcf     MOTOR_DIR_P,MOTOR_DIR   ; motor up for normal direction option
	btfsc	flags,MOTOR_DIR_MODE    ; is motor direction option reverse?
    bsf     MOTOR_DIR_P,MOTOR_DIR   ; motor up for reverse direction option

    nop
    nop
    nop

    bcf     MOTOR_STEP_P,MOTOR_STEP
    nop
    nop
    bsf     MOTOR_STEP_P,MOTOR_STEP ; pulse motor controller step line to advance motor one step

    movlw   position3
    call    decBCDVar       ; going up decrements the position

    goto    updateJM        ; display the new location

chk_dwnJM:

    btfsc   buttonState,JOG_DWN_SW_STATE
    goto    not_dwnJM      ; skip if Down switch not pressed

; jog down button press

    bsf     MOTOR_DIR_P,MOTOR_DIR   ; motor down for normal direction option
	btfsc	flags,MOTOR_DIR_MODE    ; is motor direction option reverse?
    bcf     MOTOR_DIR_P,MOTOR_DIR   ; motor up for reverse direction option

    nop
    nop
    nop

    bcf     MOTOR_STEP_P,MOTOR_STEP
    nop
    nop
    bsf     MOTOR_STEP_P,MOTOR_STEP ; pulse motor controller step line to advance motor one step

    movlw   position3
    call    incBCDVar       ; going down increments the position

    goto    updateJM        ; display the new location

not_dwnJM:

    btfsc   buttonState,SELECT_SW_STATE
    goto    loopJM          ; loop if Reset/Select switch not pressed

; reset/enter/zero button press

    btfsc   MODE_SW_P,MODE_SW   ; in Setup mode?
    goto    exitJM              ;exit Jog Mode if not when Select button pressed

; removed because this was a pain - better to be able to zero after a cut - can start a new
; cut this way without powering down
;    btfsc   flags,CUT_STARTED   ; if the cut has been started, do not allow operator to zero
;    goto    exitJM              ;  zero button will exit jog mode instead

; set the current height as zero

    movlw   position3
    call    zeroQuad        ; clear the position variable
    clrf    positionSign    ; set sign positive

    call    waitLCD         ; wait until print buffer is empty    
    movlw   0xdf
    call    writeControl    ; position in desired location
    call    displayPos      ; display the position
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

    goto    loopJM          ; stay in jog mode after zeroing

updateJM:

    movlw   0x0             ; delay between each motor step or motor won't respond
    movwf   scratch1
    movf    setupDelay,W
    call    bigDelayA

    btfsc   MODE_SW_P,MODE_SW   ; in Setup mode?
    goto    noExtraDelayJM      ; extra pause if not in setup speed mode to slow down the head

    movlw   0x0             ; delay extra in normal speed mode
    movwf   scratch1
    movf    normDelay,W
    call    bigDelayA

noExtraDelayJM:

    banksel LCDFlags

    btfss   LCDFlags,LCDBusy
    goto    displayJM

    banksel flags

    goto    loopJM          ; don't display if print buffer not ready      

displayJM:

    banksel flags

    movlw   0xdf
    call    writeControl    ; position in desired location
    call    displayPos      ; display the location of the head relative to the zero point
    call    flushLCD        ; force buffer to print, but don't wait because jogging is time
                            ; critical

    goto    loopJM          ; stay in jog mode


exitJM:

    bcf     POWER_ON_P,POWER_ON    ;  turn off the cutting voltage

    call    waitLCD         ; wait until buffer printed

    return

; end of jogMode
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; displayPos
;
; Displays the current height of the head relative to the zero position.  The variables are in
; unpacked BCD format, the smaller is always subtracted from the bigger and the sign applied
; afterwards.  The resulting difference is displayed on the LCD.
;
; On entry:
;
; Cursor should be positioned at desired print location of the LCD
;
; Uses W, FSR, TMR0, OPTION_REG, scratch0, scratch1, scratch2, scratch3, scratch4
;

displayPos:

        movf    positionSign,F  ; sign of position variable
        btfss   STATUS,Z
        goto    negativeDP      ; jump if position is negative

; position is positive

        movlw   0x20
        call    writeChar   ; display a space instead of negative sign

        movlw   position3
        goto    displayBCDVar

negativeDP:

; position is positive

        movlw   0x2d        ; ASCII '-'
        call    writeChar   ; display a negative sign

        movlw   position3
        goto    displayBCDVar

; end of displayPos
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; clearScreen
;
; Clears the LCD screen.
;
; Uses W, TMR0, OPTION_REG, scratch0, scratch1, scratch2, scratch3
;

clearScreen:

; reset LCD modes to known and clear the screen

    call    turnOffBlink		; turn off character blinking

    movlw   CLEAR_SCREEN_CMD	; send Clear Display control code to the LCD
    call    writeControl    
    
    movlw   0x80				; position cursor at line 1 column 1
    call    writeControl    	

    return                  	; exit menu on error

; end of clearScreen
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; handleMenuInputs
;
; Handles menu inputs, moving the cursor to highlight the selected option. Returns when
; Reset/Enter/Zero button pressed.
;
; On entry:
;
; menuOption contains the number of the currently selected option.
; W should contain the maximum menu option number.
;
; Uses W, TMR0, OPTION_REG, buttonState, buttonPrev, cursorPos, menuOption,
;   scratch0, scratch1, scratch2, scratch3, scratch4
;

handleMenuInputs:

    movwf   scratch4        ; store the maximum menu item number   

loopHMMI:

	; check to see if bit 7 or 6 of menuOption has been set
	; this flags that the menu page needs to be changed

	btfsc	menuOption,7
	return
	btfsc	menuOption,6
	return

    call    scanButtons     ; watch for user input

    btfsc   buttonState,JOG_UP_SW_STATE
    goto    skip_upHMMI     ; skip if Up switch not pressed

    call    selectHigherOption  ; adjusts menu_option to reflect new selection
    
    goto    loopHMMI

skip_upHMMI:

    btfsc   buttonState,JOG_DWN_SW_STATE
    goto    skip_dwnHMMI    ; skip if Down switch not pressed

    movf    scratch4,W          ; maximum number of options
    call    selectLowerOption   ; adjusts menu_option to reflect new selection

    goto    loopHMMI

skip_dwnHMMI:

    btfsc   buttonState,SELECT_SW_STATE
    goto    loopHMMI        ; loop if Reset/Select switch not pressed

    return                  ; return when Reset/Enter/Zero button pressed

; end of handleMenuInputs
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; selectHigherOption
;
; Decrements menu_option if it is not already 1, returns selected option in menuOption.
;
; If the user attempts to move above option 1, bit 7 of menuOption is set upon return.
; This flag bit can be used to trigger a menu page change if there are more than one.
; The lower 6 bits are able to handle 63 menu options - more than enough.
;
; On entry:
;
; menuOption contains the number of the currently selected option.
;
; On return, menuOption contains the newly selected option.
;
; Uses W, TMR0, OPTION_REG, menuOption, scratch0, scratch1, scratch2, scratch3
;

selectHigherOption:

    decfsz  menuOption,F    ; move to option listed higher on the screen (lower number)
    goto    moveCursorSHO

; reached end of options

    movlw   .1
    movwf   menuOption      ; don't allow option less than 1

	bsf     menuOption,7	; set bit 7 to show that user attempted to move curser
						    ; above first option - can be used to change menu pages

    return

; move cursor
; Note that if the first option is not on line one or the last option not on the last line,
; this section won't be reached because the code above prevents moving beyond option 1 or
; the max option - thus constraining the cursor to the proper lines.
; Note that the numbering for the lines is screwy - 0x80, 0xc0, 0x94, 0xd4

moveCursorSHO:

    movlw   0xd4
    subwf   cursorPos,W     ; is cursor at 0xd4?
    btfss   STATUS,Z    
    goto    line2SHO

    movlw   0x94            ; move cursor up one line
    movwf   cursorPos
    call    writeControl    ; write the cursor to the LCD
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

    return

line2SHO:

    movlw   0x94
    subwf   cursorPos,W     ; is cursor at 0x94?
    btfss   STATUS,Z    
    goto    line3SHO

    movlw   0xc0            ; move cursor up one line
    movwf   cursorPos
    call    writeControl    ; write the cursor to the LCD
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

	return

line3SHO:

    movlw   0xc0
    subwf   cursorPos,W     ; is cursor at 0xc0?
    btfss   STATUS,Z    
    goto    line4SHO

    movlw   0x80            ; move cursor up one line
    movwf   cursorPos
    call    writeControl    ; write the cursor to the LCD
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

	return

line4SHO:                   ; don't move cursor if at the top

    return

; end of selectHigherOption
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; selectLowerOption
;
; Increments menu_option if it is not already at maximum specifed in W, returns selected option in
; menuOption.
;
; If the user attempts to move below the max option, bit 6 of menuOption is set upon return.
; This flag bit can be used to trigger a menu page change if there are more than one.
; The lower 6 bits are able to handle 63 menu options - more than enough.
;
; On entry:
;
; menu_option contains the number of the currently selected option
; W contains the maximum allowable option number.
;
; On return, menuOption contains the newly selected option.
;
; Uses W, TMR0, OPTION_REG, menuOption, scratch0, scratch1, scratch2, scratch3
;

selectLowerOption:

    movwf   scratch0        ; store the maximum

    addlw   .1              ; needs to be max option + 1 for the equal 

    incf    menuOption,F    ; move to option listed lower on the screen (higher number)
    
    subwf   menuOption,W
    btfss   STATUS,Z        ; jump if menuOption = W
    goto    moveCursorSLO

    movf    scratch0,W		; already at max option, so exit
    movwf   menuOption     	; set menuOption to the maximum option allowed

	bsf     menuOption,6	; set bit 6 to show that user attempted to move curser
						    ; above first option - can be used to change menu pages

    return

; move cursor
; Note that if the first option is not on line one or the last option not on the last line,
; this section won't be reached because the code above prevents moving beyond option 1 or
; the max option - thus constraining the cursor to the proper lines.
; Note that the numbering for the lines is screwy - 0x80, 0xc0, 0x94, 0xd4

moveCursorSLO:

    movlw   0x80
    subwf   cursorPos,W     ; is cursor at 0x80?
    btfss   STATUS,Z    
    goto    line2SLO

    movlw   0xc0            ; move cursor down one line
    movwf   cursorPos
    call    writeControl    ; write the cursor to the LCD
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

    return

line2SLO:

    movlw   0xc0
    subwf   cursorPos,W     ; is cursor at 0xc0?
    btfss   STATUS,Z    
    goto    line3SLO

    movlw   0x94            ; move cursor down one line
    movwf   cursorPos
    call    writeControl    ; write the cursor to the LCD
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

	return

line3SLO:

    movlw   0x94
    subwf   cursorPos,W     ; is cursor at 0x94?
    btfss   STATUS,Z    
    goto    line4SLO

    movlw   0xd4            ; move cursor down one line
    movwf   cursorPos
    call    writeControl    ; write the cursor to the LCD
    call    flushAndWaitLCD ; force the LCD buffer to print and wait until done

	return

line4SLO:                   ; don't move cursor if at the bottom

    return

; end of selectLowerOption
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; displayBCDVar
;
; Displays the 4 digit BCD variable addressed by value in W.  A decimal point is placed between
; the first and second digits
;
; On entry:
;
; W contains address of BCD variable.
;
; Uses W, FSR, TMR0, OPTION_REG, scratch0, scratch1, scratch2, scratch3, scratch4, scratch5
;

displayBCDVar:

    movwf   scratch5        ; store pointer

    movwf   FSR0L           ; point FSR at first digit
    
    movf    INDF0,W
    addlw   0x30            ; convert BCD digit to ASCII
    call    writeChar       ; write the first digit

    movlw   0x2e            
    call    writeChar       ; write decimal point

    movlw   .3              
    movwf   scratch4        ; three bytes after the decimal point

loopDBV1:

    incf    scratch5,F      ; move to next digit    
    movf    scratch5,W
    movwf   FSR0L           ; point FSR to next digit

    movf    INDF0,W
    addlw   0x30            ; convert BCD digit to ASCII
    call    writeChar       ; write the first digit

    decfsz  scratch4,F      ; stop when count is zero
    goto    loopDBV1

    return

; end of displayBCDVar
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; displaySpeed
;
; Displays the current speed value (stored as sparkTimer).  The value in sparkTimer ranges from
; 0x0101 to 0x8101.  The value is converted to ASCII 1 - 9 and displayed.
;
; NOTE: The data is placed in the print buffer but is not submitted to be printed.  After using
; this function, call flushLCD or printString to flush the buffer.
;
; On entry:
;
; no presets required
;
; On exit:
;
; speedValue contains 1-9 to represent sparkLevel 0x01-0x81
;
; In actual use, the sparkLevel is used as the upper byte of the actual value: sparkLevel:0x01
;
; Uses W, TMR0, OPTION_REG, scratch0, scratch1, scratch2, scratch3, sparkValue
;

displaySpeed:

    movlw   0xcd
    call    writeControl    ; position at line 2 column 14

    movf    sparkLevel,W    ; get the current speed/sparkLevel value
    movwf   speedValue      ; store in variable so we can manipulate it
    rrf     speedValue,F    ; move high nibble to low
    rrf     speedValue,F
    rrf     speedValue,F
    rrf     speedValue,F
    movf    speedValue,W    ; get the rotated value
    andlw   0x0f            ; mask off upper nibble
    addlw   0x01            ; shift up one
    movwf   speedValue      ; store speed value for use by other functions

    addlw   0x30            ; convert BCD digit to ASCII
    call    writeChar       ; write the first digit

    return

; end of displaySpeed
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; isZero
;
; Checks the four byte variable addressed by the value in W to see if it is zero.
;
; On entry:
;
; W contains address of variable.
;
; On return Z bit is set if value is zero, cleared otherwise.
;
; Uses W, FSR scratch0
;

isZero:

    movwf   FSR0L           ; point FSR at first digit
    
    movlw   .4              
    movwf   scratch0        ; four bytes

loopIZ1:

    movf    INDF0,W         ; check each byte
    btfss   STATUS,Z        ; if any byte not zero return
    return

    incf    FSR0L,F         ; point to next digit

    decfsz  scratch0,F      ; stop when count is zero
    goto    loopIZ1

    bsf     STATUS,Z        ; set the Z flag - variable was zero

    return

; end of isZero
;--------------------------------------------------------------------------------------------------
   
;--------------------------------------------------------------------------------------------------
; SetBank0ClrWDT        
;
; Set Bank 0, Clear high byte of FSR pointers, Clear WatchDog timer
; 

SetBank0ClrWDT:

    clrf   FSR0H            ;high byte of indirect addressing pointers -> 0
    clrf   FSR1H

    banksel flags

    clrwdt                  ;keep watchdog from triggering

    return

;end of SetBank0ClrWDT
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; bigDelay
;
; Delays while periodically clearing WDT to prevent watch dog timer trigger.
;
; On entry at bigDelay, W holds LSB of delay value, MSB will be 0.
; On entry at bigDelayA, scratch1:W holds delay value.
;
; Notes on code for decrementing to 0:
;  subtract 1 from LSB by adding 0xff (two's comp for -1)
;  C bit will be set until LSB goes from 0 to 255; 0 is only value added to 0xff that won't carry
;  When C bit not set, subtract 1 from MSB until it reaches 0
;
; Uses W, scratch0, scratch1, scratch2, scratch3
;

bigDelay:
    clrf    scratch1
bigDelayA:
    movwf   scratch0        ; store W

LoopBD1:

	; call inner delay for each count of outer delay

    movlw   0x1
    movwf   scratch3
    movlw   0x6a            ; scratch3:W = delay value
    call    smallDelayA

    movlw   0xff            ; decrement LSByte by adding -1
    addwf   scratch0,F
    btfss   STATUS,C		; did LSByte roll under (0->255)?
	decf	scratch1,F		; decrement MSByte after LSByte roll under
	movf	scratch0,W		; check MSB:LSB for zero
	iorwf	scratch1,W
	btfsc	STATUS,Z
	goto    SetBank0ClrWDT  ; counter = 0, reset stuff and return

    goto    LoopBD1         ; loop until outer counter is zero

; end of bigDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; smallDelay
;
; Delays while periodically clearing WDT to prevent watch dog timer trigger.
;
; On entry at smallDelay, W holds LSB of delay value, MSB will be 0.
; On entry at smallDelayA, scratch3:W holds delay value.
;
; Uses scratch2, scratch3
;

smallDelay:
    clrf    scratch3
smallDelayA:
    movwf   scratch2        ; store W

LoopSD1:

    clrwdt                  ; keep watch dog timer from triggering

    movlw   0xff            ; decrement LSByte by adding -1
    addwf   scratch2,F
    btfss   STATUS,C		; did LSByte roll under (0->255)?
	decf	scratch3,F		; decrement MSByte after LSByte roll under
	movf	scratch2,W		; check MSB:LSB for zero
	iorwf	scratch3,W
	btfsc	STATUS,Z
	return

    goto    LoopSD1         ; loop until outer counter is zero

; end of smallDelay
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; isXgtY    Compare X word with Y word
;
; Entry point isXgtY checks if X > Y
; Entry point isXltY checks if X < Y
;
; If condition is true,  Z = 0 and W = 0xff
; If condition is false, Z = 1 and W = 0x00
;
; On entry:
;
; X word : scratch1:scratch0
; Y word : scratch3:w
;
; Uses W, scratch0, scratch1, scratch2, scratch3, scratch4
;

isXgtY:

    movwf   scratch2        ; scratch2 = W
    movlw   0x1             ; W = 1 (use this later to mask for less than flag)
    goto    L12

isXltY:
    movwf   scratch2        ; scratch2 = w
    movlw   0x4             ; W = 4 (use this later to mask for greater than flag)
    goto    L12

L12:
    movwf   scratch4        ; store flag mask
    movf    scratch3,W
    subwf   scratch1,W      ; W = var21 - scratch0 (compare upper bytes of word)
    btfss   STATUS,Z        ; upper bytes are same, compare lower bytes
    goto    L13
    movf    scratch2,W
    subwf   scratch0,W

; determine if result means x > y, x < y, or x == y -- REMEMBER: Borrow flag is inverse:
;                  0 = borrow, 1 = no borrow

L13:
    movlw   0x4             ; W = 4  preload flag for less than (Borrow (B=0) & not Z flags)
    btfsc   STATUS,C
    movlw   0x1             ; W = 1  replace with greater than (No Borrow (B=1))
    btfsc   STATUS,Z
    movlw   0x2             ; W = 2  replace with equal flag because of Z being set
    
    andwf   scratch4,W      ; compare flag with the mask selected by the entry point
    btfss   STATUS,Z
    movlw   0xff            ; W = 0xff if condition true, 0 if not

    return

; end of isXgtY
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; isPosGtYQ
;
; Compares head position value with YQ, a four byte value.  The position value can be negative but
; the YQ value must always be positive.
;
; On entry:
;
; W = address of YQ, value to compare position with. 
;
; Returns C = 1 if Pos >= YQ.
;
; Uses W, FSR
;

isPosGtYQ:

    movwf   FSR0L           ; point FSR to YQ

    movf    INDF0,W         ; compare most sig digits
    subwf   position3,W
    btfss   STATUS,C        ; if no borrow, position3 is larger or equal
    return
    
    incf    FSR0L,F         ; compare next digit
    movf    INDF0,W
    subwf   position2,W
    btfss   STATUS,C        ; if no borrow, position2 is larger or equal
    return

    incf    FSR0L,F         ; compare next digit
    movf    INDF0,W
    subwf   position1,W
    btfss   STATUS,C        ; if no borrow, position1 is larger or equal
    return
        
    incf    FSR0L,F         ; compare next digit
    movf    INDF0,W
    subwf   position0,W
    btfss   STATUS,C        ; if no borrow, position1 is larger or equal
    return

    bcf     STATUS,C        ; preload for less than
    movf    positionSign,W  ; load the sign byte (affects Z flag but not C)
    btfss   STATUS,Z        
    return                  ; if Z flag set, sign is negative, return with C bit cleared
                            ; (position < YQ)

    bsf     STATUS,C        ; all digits of position are >= all digits of YQ and position is
                            ; positive: return with C = 1

    return
    
; end of isPosGtYQ
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; incBCDVar
;
; Increments the signed unpacked BCD variable with format: d3, d2, d1, d0, sign.
;
; The value is prescaled with the variable preScaler - this counter will be incremented each time
; the function is called and the BCD variable only modified when the counter reaches the value.
; The stored in the variable ratio.  preScaler will then wrap around to 0.
;
; On entry:
;
; W = address of value to be incremented.
;
; Uses W, FSR, preScaler, scratch0
;

incBCDVar:

    movwf   scratch0        ; store the variable address

    incf    preScaler0,F    ; count up the preScaler ~ see note "incf vs decf rollover"
    btfsc   STATUS,Z    
    incf    preScaler1,F
    
    movf    ratio1,W        ; byte 1
    subwf   preScaler1,W
    btfss   STATUS,Z         
    return
    movf    ratio0,W        ; byte 0
    subwf   preScaler0,W
    btfss   STATUS,Z         
    return                  ; if preScaler not maxed, don't inc BCD 

    clrf    preScaler1
    clrf    preScaler0      ; restart the counter

    movf    scratch0,W       ; reload the variable address

    addlw   .4              
    movwf   FSR0L           ; point to the sign
    
    movf    INDF0,F
    btfss   STATUS,Z        ; check pos/neg
    goto    negativeIBV

; value is positive or 0 (sign is always + for zero), so add one to it to increment

    movf    scratch0,W      ; retrieve variable address
    call    incBCDAbs       ; add one

    return

negativeIBV:

; value is negative so subtract one to increment it

    movf    scratch0,W      ; retrieve variable address
    call    decBCDAbs       ; decrement one

    movf    scratch0,W      ; retrieve variable address
    call    isQuadZero      ; check for zero
    
    btfss   STATUS,Z        
    return                  ; value is not zero

    movf    scratch0,W      ; retrieve variable address
    addlw   .4              
    movwf   FSR0L           ; point to the sign
    movlw   0x0
    movwf   INDF0           ; set sign positive

    return

; end of incBCDVar
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; decBCDVar
;
; Decrements the signed unpacked BCD variable with format: d3, d2, d1, d0, sign.
;
; The value is prescaled with the variable preScaler - this counter will be decremented each time
; the function is called and the BCD variable only modified when the counter reaches zero.  The
; preScaler will be reloaded from the variable ratio.
;
; On entry:
;
; W = address of value to be decremented.
;
; Uses W, FSR, preScaler, scratch0
;

decBCDVar:

    movwf   scratch0        ; store the variable address

    movlw  .1
    subwf   preScaler0,F    ; count down the preScaler ~ see note "incf vs decf rollover"
    btfss   STATUS,C
    decf    preScaler1,F

    movf    ratio_neg1,W    ; byte 1
    subwf   preScaler1,W
    btfss   STATUS,Z         
    return
    movf    ratio_neg0,W	; byte 0
    subwf   preScaler0,W
    btfss   STATUS,Z         
    return                  ; if preScaler not maxed, don't dec BCD 

    clrf    preScaler1
    clrf    preScaler0      ; restart the counter

    movf    scratch0,W       ; reload the variable address

    call    isQuadZero      ; check if zero
    btfsc   STATUS,Z
    goto    negativeDBV     ; if it is zero, add one to decrement

    movf    scratch0,W  
    addlw   .4              
    movwf   FSR0L           ; point to the sign
    
    movf    INDF0,F
    btfss   STATUS,Z        ; check pos/neg
    goto    negativeDBV
    
; value is positive, so subtract one from it to decrement

    movf    scratch0,W      ; retrieve variable address
    call    decBCDAbs       ; decrement one

    return

negativeDBV:

; value is negative or zero so add one to it to decrement
; since value is negative or zero, decrementing will always result in negative

    movf    scratch0,W      ; retrieve variable address
    addlw   .4              
    movwf   FSR0L           ; point to the sign
    movlw   0x01
    movwf   INDF0           ; set sign negative

    movf    scratch0,W      ; retrieve variable address
    call    incBCDAbs       ; add one

    return

; end of decBCDVar
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; incBCDAbs
;
; Adds one to BCD variable ignoring its sign.
;
; On entry:
;
; W = address of value to be incremented.
;
; Uses W, FSR
;

incBCDAbs:

    addlw   .3              ; point to digit 0
    movwf   FSR0L           ; point FSR to variable

    movlw   .1
    addwf   INDF0,F         ; add one to digit 0
    movlw   .10
    subwf   INDF0,W          ; compare with 9
    btfss   STATUS,C       
    return                  ; return if borrow (C = 1), digit < 10

    clrf    INDF0           ; wraps to 0 after 9
    
    decf    FSR0L,F          ; digit 1
    movlw   .1
    addwf   INDF0,F         ; add one to digit 1
    movlw   .10
    subwf   INDF0,W         ; compare with 9
    btfss   STATUS,C       
    return                  ; return if borrow (C = 1), digit < 10
   
    clrf    INDF0           ; wraps to 0 after 9

    decf    FSR0L,F         ; digit 2
    movlw   .1
    addwf   INDF0,F         ; add one to digit 2
    movlw   .10
    subwf   INDF0,W         ; compare with 9
    btfss   STATUS,C       
    return                  ; return if borrow (C = 1), digit < 10

    clrf    INDF0           ; wraps to 0 after 9

    decf    FSR0L,F         ; digit 3
    movlw   .1
    addwf   INDF0,F         ; add one to digit 3
    movlw   .10
    subwf   INDF0,W         ; compare with 9
    btfss   STATUS,C       
    return                  ; return if borrow (C = 1), digit < 10

    clrf    INDF0           ; wraps to 0 after 9
    
    ; if carry from most significant digit, allow to roll over 
    ; (should never happen, hardware can't travel that distance)

    return
    
; end of incBCDAbs
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; decBCDAbs
;
; Subtracts one from BCD variable ignoring its sign.
;
; On entry:
;
; W = address of value to be incremented.
;
; Uses W, FSR
;

decBCDAbs:

    addlw   .3              ; point to digit 0
    movwf   FSR0L           ; point FSR to variable

    movlw   .1
    subwf   INDF0,F         ; subtract one from digit 0
    btfsc   STATUS,C        ; borrow? (C will = 0)
    return

    movlw   .9
    movwf   INDF0           ; wraps to 9 after 0
    
    decf    FSR0L,F         ; digit 1
    movlw   .1
    subwf   INDF0,F         ; subtract one from digit 1
    btfsc   STATUS,C        ; borrow? (C will = 0)
    return

    movlw   .9
    movwf   INDF0           ; wraps to 9 after 0
    
    decf    FSR0L,F         ; digit 2
    movlw   .1
    subwf   INDF0,F         ; subtract one from digit 2
    btfsc   STATUS,C        ; borrow? (C will = 0)
    return

    movlw   .9
    movwf   INDF0           ; wraps to 9 after 0

    decf    FSR0L,F         ; digit 3
    movlw   .1
    subwf   INDF0,F         ; subtract one from digit 3
        
    btfsc   STATUS,C        ; borrow? (C will = 0)
    return

    movlw   .9
    movwf   INDF0           ; wraps to 9 after 0

    ; if borrow from most significant digit, allow to roll over 
    ; (should never happen, hardware can't travel that distance)

    return
    
; end of decBCDAbs
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; isQuadZero
;
; Checks if the four byte variable pointed to by W is zero.
;
; On entry:
;
; W = address of variable to be tested
;
; On return, Z = 1 if variable is zero.
;
; Uses W, FSR
;

isQuadZero:

    movwf   FSR0L           ; point to variable

    movf    INDF0,F
    btfss   STATUS,Z        ; check for zero
    return

    incf    FSR0L,F         ; next digit
    movf    INDF0,F
    btfss   STATUS,Z        ; check for zero
    return

    incf    FSR0L,F         ; next digit
    movf    INDF0,F
    btfss   STATUS,Z        ; check for zero
    return

    incf    FSR0L,F         ; next digit
    movf    INDF0,F

    return

; end of isQuadZero
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; printString
;
; Prints the string specified by W.  The string is placed after any data already in the print
; buffer, all data in the buffer is then begun to be transmitted to the LCD.
;
; Does not wait for the data to be printed - call waitLCD after calling this function to wait
; until string is printed.
;
; On entry:
;
; W contains index of desired character - first character is W = 0
;
; Uses W, PCLATH, TMR0, OPTION_REG,
;   scratch0, scratch1, scratch2, scratch3, scratch4, scratch5, scratch6
;
; After placing the string characters in the LCD print buffer, the buffer is flushed to force
; transmission of the buffer.  Any characters placed in the buffer before the string will also
; be printed, so place control codes in the buffer first and then call this function to print
; everything.
;

printString:

    movwf   scratch6        ; store string index

    ; call to get first character just so we can get the length of the string

    movwf   scratch0        ; store string index
    movlw   #.0             ; number of character to retrieve (first = 0)   
    bsf     PCLATH,3
    call    getStringChar   ; retrieve character from string
    bcf     PCLATH,3

    movf    scratch1,W
    movwf   scratch4        ; scratch4 = string length
    clrf    scratch5        ; zero character index

loopPS:
    
    movf    scratch6,W
    movwf   scratch0        ; scratch0 = string index
    movf    scratch5,W      ; W = character index
  
    bsf     PCLATH,3  
    call    getStringChar   ; get next character
    bcf     PCLATH,3

    call    writeChar       ; write the character to the LCD

    incf    scratch5,F      ; move to next character
    movf    scratch4,W      ; get the string length
    subwf   scratch5,W      ; check if index at end of string

    btfss   STATUS,Z
    goto    loopPS          ; loop until length of string reached

endPS:

    goto    flushLCD        ; force printing of the buffer    

; end of printString
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; flushLCD
;
; Forces the data in the LCD print buffer to start transmission to the LCD.
;
;

flushLCD:

; prepare the interrupt routine for printing

    banksel LCDBuffer0

    movlw   LCDBuffer0
    movwf   LCDBufferPtr
    bsf     LCDFlags,startBit
    bsf     LCDFlags,LCDBusy    ; set this bit last to make sure everything set up before interrupt

    banksel flags

    return

; end of flushLCD
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitLCD
;
; Waits until the LCD buffer has been transmitted to the LCD and is ready for more data.
;

waitLCD:

    banksel LCDFlags

loopWBL1:                   ; loop until interrupt routine finished writing character

    btfsc   LCDFlags,LCDBusy
    goto    loopWBL1

    banksel flags

    return

; end of waitLCD
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; flushAndWaitLCD
;
; Forces the data in the LCD print buffer to start transmission to the LCD then waits until the LCD
; buffer has been transmitted to the LCD and is ready for more data.
;

flushAndWaitLCD:

    call    flushLCD
    call    waitLCD    

    return

; end of flushAndWaitLCD
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; turnOffBlink
;
; Turns off blink function in the LCD. The cursor is also turned off.
;
; The command stored in the LCD print buffer after any data already in the buffer.
;
; Uses W, TMR0, OPTION_REG, scratch0, scratch1, scratch2, scratch3
;
; NOTE: The data is placed in the print buffer but is not submitted to be printed.  After using
; this function, call flushLCD or printString to flush the buffer.
;

turnOffBlink:

    movlw   DISPLAY_ONOFF_CMD_FLAG | DISPLAY_ON_FLAG ; display on, cursor off, blink off

    call    writeControl

    return

; end of turnOffBlink
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; turnOnBlink
;
; Turns on blink function in the LCD - the current at the address in the address register blinks.
; The cursor is also turned off.
;
; The command is stored in the LCD print buffer after any data already in the buffer.
;
; Uses W, TMR0, OPTION_REG, scratch0, scratch1, scratch2, scratch3
;
; NOTE: The data is placed in the print buffer but is not submitted to be printed.  After using
; this function, call flushLCD or printString to flush the buffer.
;

turnOnBlink:

    movlw   DISPLAY_ONOFF_CMD_FLAG | DISPLAY_ON_FLAG | BLINK_ON_FLAG ; display on, cursor off, blink on

    call    writeControl

    return

; end of turnOnBlink
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeChar
;
; Writes an ASCII character to the LCD : writes 0x0 to the LCD, then writes byte in W to LCD.
; This is stored in the LCD print buffer after any data already in the buffer.
;
; On entry:
;
; W contains byte to write
;
; Uses W, TMR0, OPTION_REG, scratch0, scratch1, scratch2, scratch3
;
; NOTE: The data is placed in the print buffer but is not submitted to be printed.  After using
; this function, call flushLCD or printString to flush the buffer.
;

writeChar:

    movwf    scratch0       ; store character

    clrf    scratch1        ; scratch1 = 0

    call    writeWordLCD    ; write 0 followed by scratch0 to LCD
    
    return

; end of writeChar
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeControl
;
; Writes a control code to the LCD PIC : sends 0x1 to the LCD PIC, then sends byte in W.
;
; These values are stored in the LCD print buffer after any data already in the buffer and
; transmitted by another function.
;
; On entry:
;
; W contains byte to write
;
; Uses W, TMR0, OPTION_REG, scratch0, scratch1, scratch2, scratch3
;
; NOTE: The data is placed in the print buffer but is not submitted to be printed.  After using
; this function, call flushLCD or printString to flush the buffer.
;

writeControl:

    movwf    scratch0       ; store character

    movlw   0x1
    movwf   scratch1        ; scratch1 = 1
    
    call    writeWordLCD    ; write 0 followed by scratch0 to LCD

    return

; end of writeControl
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeWordLCD
; 
; This subroutine writes the word in scratch1:0 to the LCD screen.
;
; On entry:
; 
; scratch1 contains first byte to write
; scratch0 contains second byte to write
;
; Uses W, TMR0, OPTION_REG, scratch0, scratch1, scratch2, scratch3
;
; NOTE: The data is placed in the print buffer but is not submitted to be printed.  After using
; this function, call flushLCD or printString to flush the buffer.
;

writeWordLCD:
    
    movf    scratch1,W      ; get first byte to write
    call    writeByteLCD

    movf    scratch0,W      ; get second byte to write
    call    writeByteLCD    ;(0x68b)
    
    return

; end of writeWordLCD
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeByteLCD
;
; This subroutine writes the byte in W to the LCD screen.
;
; On entry:
; 
; W contains byte to write
;
; Uses W, TMR0, OPTION_REG, scratch2, scratch3, LCDscratch
;
; NOTE: The data is placed in the print buffer but is not submitted to be printed.  After using
; this function, call flushLCD or printString to flush the buffer.
;

writeByteLCD:

    banksel LCDScratch0

    movwf   LCDScratch0     ; store character

    movf    LCDBufferPtr,W
    movwf   FSR0L           ; point to next buffer position

    movf    LCDScratch0,W   ; retrieve character

    movwf   INDF0           ; store character in buffer

    incf    LCDBufferCnt,F  ; count characters placed in the buffer
    incf    LCDBufferPtr,F  ; point to next buffer position

    banksel flags

    return    

; end of writeByteLCD
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; readFromEEprom
; 
; Read block of bytes from EEprom.
;
; Address in EEprom from which to read first byte should be in eepromAddressH/L.
; Number of bytes to read should be in eepromCount.
; Indirect register FSR0 should point to first byte in RAM to be written into.
; The block of bytes will be copied from EEprom to RAM.
; 

readFromEEprom:

loopRFE1:

    call    readByteFromEEprom1ViaI2C

    banksel scratch0            ; store byte read to the buffer
    movwf   scratch0
    movwi   FSR0++

    banksel eepromAddressL
	incf	eepromAddressL,F    ; move to next address in EEprom
    btfsc   STATUS,Z
	incf	eepromAddressH,F	; increment high byte on low byte rollover

    banksel eepromCount
	decfsz	eepromCount,F       ; count down number of bytes transferred
	goto	loopRFE1            ; not zero yet - transfer more bytes

	return

; end of readFromEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeToEeprom
;
; Writes block of bytes to EEprom.
;
; Address in EEprom to store first byte should be in eepromAddressH/L.
; Number of bytes to write should be in eepromCount.
; Indirect register FSR0 should point to first byte in RAM to be written.
; The block of bytes will be copied from RAM to EEprom.
;

writeToEEprom:

loopWTE1:	

    moviw   FSR0++              ; store byte to be written in scratch0
    banksel scratch0
    movwf   scratch0

    call    writeByteToEEprom1ViaI2C

    banksel eepromAddressL
	incf	eepromAddressL,F    ; move to next address in EEprom
    btfsc   STATUS,Z
	incf	eepromAddressH,F	; increment high byte on low byte rollover

    call    waitForEEprom1WriteCycleFinished

    banksel eepromCount
	decfsz	eepromCount,F       ; count down number of bytes transferred
	goto	loopWTE1            ; not zero yet - trasfer more bytes

	return

; end of writeToEEprom
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setDigitalPotInChip1
;
; Sets the pot specified by scratch0 in digital pot chip 1 to the value in scratch1.
;

setDigitalPotInChip1:

    call    clearSSP1IF             ; make sure flag is cleared before starting

    call    generateI2CStart

    movlw   DIGITAL_POT1_WRITE_ID   ; send proper ID to write to digital pot chip 1
    call    sendI2CByte             ; send byte in W register on I2C bus after SP1IF goes high

    banksel scratch0                ; send the address of the pot in the chip to access
    movf    scratch0,W
    call    sendI2CByte

    banksel scratch1                ; send the pot value
    movf    scratch1,W
    call    sendI2CByte

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon transmission completion

    call    generateI2CStop

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon stop condition finished

    return

; end of setDigitalPotInChip1
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; writeByteToEEprom1ViaI2C
;
; Writes a byte to EEprom1 via the I2C bus.
;
; The byte to be written should be in scratch0.
; The EEprom target address wordk should be in eepromAddressL/H
;

writeByteToEEprom1ViaI2C:

    call    clearSSP1IF             ; make sure flag is cleared before starting

    call    generateI2CStart

    movlw   EEPROM1_WRITE_ID        ; send proper ID to write to EEprom 1
    call    sendI2CByte             ; send byte in W register on I2C bus after SSP1IF goes high

    banksel eepromAddressH          ; send the address high byte after SSP1IF goes high
    movf    eepromAddressH,W
    call    sendI2CByte

    banksel eepromAddressL          ; send the address low byte after SSP1IF goes high
    movf    eepromAddressL,W
    call    sendI2CByte

    banksel scratch0                ; send the byte to be written after SSP1IF goes high
    movf    scratch0,W
    call    sendI2CByte

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon transmission completion

    call    generateI2CStop

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon stop condition finished

    return

; end of writeByteToEEprom1ViaI2C
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; readByteFromEEprom1ViaI2C
;
; Reads a byte from EEprom1 via the I2C bus.
;
; The byte read will be returned in scratch0.
; The EEprom source address wordk should be in eepromAddressL/H
;
; To read a byte from the EEprom, the source address is first set by using a write command. A
; restart condition is then generated and the byte read using a read command.
; After the read, a NACK is sent to the slave followed by a stop condition.
;

readByteFromEEprom1ViaI2C:

    call    clearSSP1IF             ; make sure flag is cleared before starting

    call    generateI2CStart

    movlw   EEPROM1_WRITE_ID        ; send proper ID to write to EEprom 1 (used to set address)
    call    sendI2CByte             ; send byte in W register on I2C bus after SP1IF goes high

    banksel eepromAddressH          ; send the address high byte
    movf    eepromAddressH,W
    call    sendI2CByte

    banksel eepromAddressL          ; send the address low byte
    movf    eepromAddressL,W
    call    sendI2CByte

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon transmission completion

    call    generateI2CRestart

    movlw   EEPROM1_READ_ID        ; send proper ID to write to EEprom 1
    call    sendI2CByte            ; send byte in W register via I2C bus after SP1IF goes high

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon transmission completion

    banksel SSPCON2
    bsf     SSPCON2,RCEN

    call    waitForSSP1IFHighThenClearIt;    wait for high flag upon reception completion

    banksel SSP1BUF                 ; store the received byte in scratch0
    movf    SSP1BUF,W
    banksel scratch0
    movwf   scratch0

    banksel SSPCON2                 ; send NACK to terminate read
    bsf     SSPCON2,ACKDT           ; send high bit (NACK)
    bsf     SSPCON2,ACKEN           ; enable NACK transmission

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon NACK transmission complete

    banksel SSPCON2
    bcf     SSPCON2,ACKDT           ; reset to send ACKs

    call    generateI2CStop

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon stop condition finished

    return

; end of readByteFromEEprom1ViaI2C
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; waitForEEprom1WriteCycleFinished
;
; Waits for EEprom1 to finish its write cycle.
;
; The EEprom will not respond to a write command with an ACK during a write cycle, so a write
; command is repeatedly sent until the EEprom responds with an ACK. A stop condition is then
; generated to abort the write command.
;

waitForEEprom1WriteCycleFinished:

wfewcf1:

    call    clearSSP1IF             ; make sure flag is cleared before starting

    call    generateI2CStart

    movlw   EEPROM1_WRITE_ID        ; send proper ID to write to EEprom 1
    call    sendI2CByte             ; send byte in W register on I2C bus after SP1IF goes high

    call    waitForSSP1IFHighThenClearIt; wait for high flag upon transmission completion

    ; abort the write operation regardless of success
    ; it was only used to check for ACK from slave

    call    generateI2CStop

    call    waitForSSP1IFHighThenClearIt    ; wait for high flag upon stop condition finished

    banksel SSPCON2
    btfsc   SSPCON2,ACKSTAT         ; check for low ACK from slave, repeat loop if NACK
    goto    wfewcf1

    return

; end of waitForEEprom1WriteCycleFinished
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; generateI2CStart
;
; Generates a start condition on the I2C bus.
;

generateI2CStart:

    banksel SSPCON2
    bsf     SSPCON2,SEN

    return

; end of generateI2CStart
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; generateI2CRestart
;
; Generates a restart condition on the I2C bus.
;

generateI2CRestart:

    banksel SSPCON2
    bsf     SSPCON2,RSEN

    return

; end of generateI2CRestart
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; generateI2CStop
;
; Generates a stop condition on the I2C bus.
;

generateI2CStop:

    banksel SSPCON2
    bsf     SSPCON2,PEN

    return

; end of generateI2CStop
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
    return;           ; simulated by the IDE
    endif

    banksel PIR1

wfsh1:
    btfss   PIR1, SSP1IF
    goto    wfsh1

    return

; end of waitForSSP1IFHigh
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
; sendI2CByte
;
; Waits until the SSP1IF bit in register PIR1 goes high and then transmits the byte in the W
; register on the I2C bus.
;

sendI2CByte:

    ; wait for SSP1IF to go high

    call    waitForSSP1IFHigh

    ; put byte in transmit buffer

    banksel SSPBUF
    movwf   SSPBUF

    ; clear interrupt flag

    call    clearSSP1IF

    return

; end of sendI2CByte
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; setupI2CMaster7BitTransmitMode
;
; Sets the MASTER SYNCHRONOUS SERIAL PORT (MSSP) MODULE to the I2C Master Transmission mode using
; the 7 bit address mode.
;
; NOTE: RB4 and RB6 must have been configured elswhere as inputs for this mode.
;

setupI2CMaster7BitTransmitMode:


    movlw   0x27			; set baud rate at 100kHz for oscillator frequency of 16 Mhz
    banksel SSPADD
    movwf   SSPADD

    banksel SSPCON1
    bcf	SSPCON1,SSP1M0		; SSPM = b1000 ~ I2C Master mode, clock = FOSC / (4 * (SSPADD+1))(4)
    bcf	SSPCON1,SSP1M1
    bcf	SSPCON1,SSP1M2
    bsf	SSPCON1,SSP1M3

    bsf	SSPCON1,SSPEN		;enables the MSSP module

    return

; end setupI2CMaster7BitTransmitMode
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; applyASCIINumDigitLimits
;
; Limits an ASCII value in scratch0 between ASCII '0' and '9'.
;
; Compares the value in scratch0 with ASCII for '0' and '9'. If the value is less than that for
; the '0' character, it is replaced with that value. If greater than the '9' value, it is replaced
; with that value.
;

applyASCIINumDigitLimits:

    banksel scratch0

    movlw   0x30                        ; lower limit is ASCII for '0'
    movwf   scratch1

    movlw   0x39                        ; upper limit is ASCII for '9'
    movwf   scratch2

    goto    applyLimitsToByteValue

; end applyASCIINumDigitLimits
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; applyBCDDigitLimits
;
; Limits an unpacked BCD value in scratch0 between 0 and 9.
;
; Compares the value in scratch0 with 0 and 9. If the value is less than that for
; the 0, it is replaced with that value. If greater than 9, it is replaced
; with that value.
;

applyBCDDigitLimits:

    banksel scratch0

    movlw   .0                           ; lower limit for unpacked BCD
    movwf   scratch1

    movlw   .9                           ; upper limit for unpacked BCD
    movwf   scratch2

    goto    applyLimitsToByteValue

; end applyBCDDigitLimits
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; applyLimitsToByteValue
;
; Clips a value between a lower and upper limit.
;
; Compares the value in scratch0 with the lower limit in scratch1 and the upper limit in scratch2.
;
; If scratch0 < scratch1, scratch0 = scratch1
; If scratch0 > scratch2, scratch0 = scratch2
;

applyLimitsToByteValue:

    banksel scratch0

    movf    scratch0,W          ; check if scratch0 greater than scratch2
    subwf   scratch2,W
    btfsc   STATUS,C
    goto    notHigher

    movf    scratch2,W          ; was greater than so replace scratch0 with scratch1
    movwf   scratch0

notHigher:

    movf    scratch1,W          ; check if scratch0 less than scratch1
    subwf   scratch0,W
    btfsc   STATUS,C
    goto    notLower

    movf    scratch1,W          ; was less than so replace scratch0 with scratch1
    movwf   scratch0

notLower:

    return

; end applyLimitsToByteValue
;--------------------------------------------------------------------------------------------------

;--------------------------------------------------------------------------------------------------
; getStringChar
;
; All the strings are stored here - the function returns one character from one string.
;
; On entry:
;
; Data Bank 0 should already be selected.
; scratch0 contains index of desired string - first string is scratch0 = 0
; W contains index of desired character - first character is W = 0
;
; On return W contains the character.
; scratch1 contains the length of the string
; Data Bank 0 will still be selected.
;
; CAUTION: No string can cross over the PCL boundary (256 bytes) - when adding a new string, use
; org to reposition it if it will cross the boundary.  PCLATH MUST be adjusted for each string
; to point to its bank location in memory:
;
;    movlw   0x0c            ; make sure this points to the page the string is on!!!!
;    movwf   PCLATH
;
; CAUTION: If a string is inserted, all following strings must be checked to make sure they
; are not shifted to span a PCL boundary (every 256 bytes) - use ORG to prevent this.  Every
; value loaded into PCLATH must be checked. Use View/Disassembly to see the address positions.
;
; Uses W, PCLATH, scratch0, scratch1, scratch2
;

    org     0xc00           ; start on this boundary

getStringChar:

    incf    scratch0,F      ; increment to index properly
    movwf   scratch2        ; store character selector

string0:    ; "OPT AutoNotcher Rx.x"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string1         ; skip to next string if count not 0

    movlw   #.20      
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0c            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector

    addwf   PCL,F
    retlw   'O'
    retlw   'P'
    retlw   'T'
    retlw   ' '
    retlw   'A'
    retlw   'u'
    retlw   't'
    retlw   'o'
    retlw   'N'
    retlw   'o'
    retlw   't'
    retlw   'c'
    retlw   'h'
    retlw   'e'
    retlw   'r'
    retlw   ' '
    retlw   '7'
    retlw   '.'
    retlw   '7'
    retlw   'f'

string1:    ; "CHOOSE CONFIGURATION"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string2         ; skip to next string if count not 0

    movlw   #.20      
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0c            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   'C'
    retlw   'H'
    retlw   'O'
    retlw   'O'
    retlw   'S'
    retlw   'E'
    retlw   ' '
    retlw   'C'
    retlw   'O'
    retlw   'N'
    retlw   'F'
    retlw   'I'
    retlw   'G'
    retlw   'U'
    retlw   'R'
    retlw   'A'
    retlw   'T'
    retlw   'I'
    retlw   'O'
    retlw   'N'

string2:    ; "1 - EDM Notch Cutter"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string3         ; skip to next string if count not 0

    movlw   #.20      
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0c            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   '1'
    retlw   ' '
    retlw   '-'
    retlw   ' '
    retlw   'E'
    retlw   'D'
    retlw   'M'
    retlw   ' '
    retlw   'N'
    retlw   'o'
    retlw   't'
    retlw   'c'
    retlw   'h'
    retlw   'C'
    retlw   'u'
    retlw   't'
    retlw   't'
    retlw   'e'
    retlw   'r'

string3:    ; "2 - EDM Extend Reach"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string4         ; skip to next string if count not 0

    movlw   #.20      
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0c            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   '2'
    retlw   ' '
    retlw   '-'
    retlw   ' '
    retlw   'E'
    retlw   'D'
    retlw   'M'
    retlw   ' '
    retlw   'E'
    retlw   'x'
    retlw   't'
    retlw   'e'
    retlw   'n'
    retlw   'd'
    retlw   ' '
    retlw   'R'
    retlw   'e'
    retlw   'a'
    retlw   'c'
    retlw   'h'

string4:    ; "OPT EDM Notch Cutter"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string5         ; skip to next string if count not 0

    movlw   #.20      
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0c            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   'O'
    retlw   'P'
    retlw   'T'
    retlw   ' '
    retlw   'E'
    retlw   'D'
    retlw   'M'
    retlw   ' '
    retlw   'N'
    retlw   'o'
    retlw   't'
    retlw   'c'
    retlw   'h'
    retlw   'C'
    retlw   'u'
    retlw   't'
    retlw   't'
    retlw   'e'
    retlw   'r'

string5:    ; "1 - Set Cut Depth"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string6         ; skip to next string if count not 0

    movlw   #.17      
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0c            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   '1'
    retlw   ' '
    retlw   '-'
    retlw   ' '
    retlw   'S'
    retlw   'e'
    retlw   't'
    retlw   ' '
    retlw   'C'
    retlw   'u'
    retlw   't'
    retlw   ' '
    retlw   'D'
    retlw   'e'
    retlw   'p'
    retlw   't'
    retlw   'h'

string6:    ; "1 - Depth = "

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string7         ; skip to next string if count not 0

    movlw   #.12      
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0c            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   '1'
    retlw   ' '
    retlw   '-'
    retlw   ' '
    retlw   'D'
    retlw   'e'
    retlw   'p'
    retlw   't'
    retlw   'h'
    retlw   ' '
    retlw   '='
    retlw   ' '

string7:    ; "2 - Cut Notch"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string8         ; skip to next string if count not 0

    movlw   #.13      
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0c            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   '2'
    retlw   ' '
    retlw   '-'
    retlw   ' '
    retlw   'C'
    retlw   'u'
    retlw   't'
    retlw   ' '
    retlw   'N'
    retlw   'o'
    retlw   't'
    retlw   'c'
    retlw   'h'

string8:    ; "3 - Jog Electrode"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string9         ; skip to next string if count not 0

    movlw   #.17      
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0c            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   '3'
    retlw   ' '
    retlw   '-'
    retlw   ' '
    retlw   'J'
    retlw   'o'
    retlw   'g'
    retlw   ' '
    retlw   'E'
    retlw   'l'
    retlw   'e'
    retlw   'c'
    retlw   't'
    retlw   'r'
    retlw   'o'
    retlw   'd'
    retlw   'e'

string9:    ;"   Set Cut Depth"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string10        ; skip to next string if count not 0

    movlw   #.16
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0c            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   ' '
    retlw   ' '
    retlw   ' '
    retlw   'S'
    retlw   'e'
    retlw   't'
    retlw   ' '
    retlw   'C'
    retlw   'u'
    retlw   't'
    retlw   ' '
    retlw   'D'
    retlw   'e'
    retlw   'p'
    retlw   't'
    retlw   'h'

string10:    ;"0.000 inches"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string11        ; skip to next string if count not 0

    movlw   #.12
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0d            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   '0'
    retlw   '.'
    retlw   '0'
    retlw   '0'
    retlw   '0'
    retlw   ' '
    retlw   'i'
    retlw   'n'
    retlw   'c'
    retlw   'h'
    retlw   'e'
    retlw   's'

string11:    ;"Jog Mode"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string12        ; skip to next string if count not 0

    movlw   #.8
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0d            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   'J'
    retlw   'o'
    retlw   'g'
    retlw   ' '
    retlw   'M'
    retlw   'o'
    retlw   'd'
    retlw   'e'

string12:    ;"Zero or Exit"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string13        ; skip to next string if count not 0

    movlw   #.12
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0d            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   'Z'
    retlw   'e'
    retlw   'r'
    retlw   'o'
    retlw   ' '
    retlw   'o'
    retlw   'r'
    retlw   ' '
    retlw   'E'
    retlw   'x'
    retlw   'i'
    retlw   't'

string13:    ;"OPT EDM Extend Reach"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string14        ; skip to next string if count not 0

    movlw   #.20
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0d            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   'O'
    retlw   'P'
    retlw   'T'
    retlw   ' '
    retlw   'E'
    retlw   'D'
    retlw   'M'
    retlw   ' '
    retlw   'E'
    retlw   'x'
    retlw   't'
    retlw   'e'
    retlw   'n'
    retlw   'd'
    retlw   ' '
    retlw   'R'
    retlw   'e'
    retlw   'a'
    retlw   'c'
    retlw   'h'

string14:    ;"Turn on Cut Voltage"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string15        ; skip to next string if count not 0

    movlw   #.19
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0d            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   'T'
    retlw   'u'
    retlw   'r'
    retlw   'n'
    retlw   ' '
    retlw   'o'
    retlw   'n'
    retlw   ' '
    retlw   'C'
    retlw   'u'
    retlw   't'
    retlw   ' '
    retlw   'V'
    retlw   'o'
    retlw   'l'
    retlw   't'
    retlw   'a'
    retlw   'g'
    retlw   'e'

string15:    ; "Up   Speed>"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string16        ; skip to next string if count not 0

    movlw   #.11
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0d            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   'U'
    retlw   'p'
    retlw   ' '
    retlw   ' '
    retlw   ' '
    retlw   'S'
    retlw   'p'
    retlw   'e'
    retlw   'e'
    retlw   'd'
    retlw   '>'
   
string16:   ; "Down  Stop>"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string17        ; skip to next string if count not 0

    movlw   #.11
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0d            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   'D'
    retlw   'o'
    retlw   'w'   
    retlw   'n'
    retlw   ' '
    retlw   ' '
    retlw   'S'
    retlw   't'
    retlw   'o'
    retlw   'p'
    retlw   '>'

string17:   ; "Notch Mode"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string18        ; skip to next string if count not 0

    movlw   #.10
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0d            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   'N'
    retlw   'o'
    retlw   't'   
    retlw   'c'
    retlw   'h'
    retlw   ' '
    retlw   'M'
    retlw   'o'
    retlw   'd'
    retlw   'e'

string18:   ; "Wall Mode"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string19        ; skip to next string if count not 0

    movlw   #.9
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0d            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   'W'
    retlw   'a'
    retlw   'l'   
    retlw   'l'
    retlw   ' '
    retlw   'M'
    retlw   'o'
    retlw   'd'
    retlw   'e'

string19:   ; "Cycle Test"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string20        ; skip to next string if count not 0

    movlw   #.10
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0d            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   'C'
    retlw   'y'
    retlw   'c'   
    retlw   'l'
    retlw   'e'
    retlw   ' '
    retlw   'T'
    retlw   'e'
    retlw   's'
	retlw   't'

string20:   ; "5 - Motor Dir "

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string21        ; skip to next string if count not 0

    movlw   #.14
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0d            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   '5'
    retlw   ' '
    retlw   '-'
    retlw   ' '
    retlw   'M'
    retlw   'o'
    retlw   't'   
    retlw   'o'
    retlw   'r'
    retlw   ' '
    retlw   'D'
    retlw   'i'
    retlw   'r'
	retlw   ' '

string21:   ; "4 - "

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string22        ; skip to next string if count not 0

    movlw   #.4
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0d            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   '4'
    retlw   ' '
    retlw   '-'
    retlw   ' '

string22:   ; "Normal"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string23        ; skip to next string if count not 0

    movlw   #.6
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0d            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   'N'
    retlw   'o'
    retlw   'r'
    retlw   'm'
    retlw   'a'
    retlw   'l'

    org     0xe00           ; skip ahead so string doesn't cross 256 byte boundary

string23:   ; "Reverse"

    decfsz  scratch0,F      ; count down until desired string reached
    goto    string24        ; skip to next string if count not 0

    movlw   #.3
    movwf   scratch1        ; scratch1 = length of string

    movlw   0x0e            ; point to this program memory page
    movwf   PCLATH
    movf    scratch2,W      ; restore character selector
    
    addwf   PCL,F
    retlw   'R'
    retlw   'e'
    retlw   'v'

string24:
    return                  ; no string here yet

; end of getStringChar
;--------------------------------------------------------------------------------------------------

    END
