;  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PIC16F722_INVERTER_SQ2SPWM [Rev. 1.0] ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
;
;  This code is under MIT License
;  Copyright (c) 2022 Sayantan Sinha
;

#include <PIC16F722.inc>
#include <xc.inc>
                                  ; PIC16F722 Configuration Bit Settings
CONFIG FOSC = HS                  ; HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN
CONFIG WDTE = OFF                 ; WDT disabled and can be enabled by SWDTEN bit of the WDTCON register
CONFIG PWRTE = ON                 ; PWRT enabled
CONFIG MCLRE = OFF                ; RE3/MCLR pin function is digital input, MCLR internally tied to VDD
CONFIG CP = OFF                   ; Program memory code protection is disabled
CONFIG BOREN = ON                 ; Brown-out Reset (BOR) enabled
CONFIG BORV = 19                  ; Brown-out Voltage = 1.9 V
CONFIG PLLEN = ON                 ; INTOSC Frequency is 16 MHz
CONFIG VCAPEN = 0                 ; VCAP functionality for internal LDO is enabled on RA0

#define HS1     PORTB, 7          ; PORT read/write macro
#define LS1     PORTB, 6
#define HS2     PORTC, 1
#define LS2     PORTC, 0
#define I_L     PORTA, 1
#define V_B     PORTA, 2
#define MFB     PORTA, 3
#define FAN     PORTA, 4
#define VMNS    PORTA, 5
#define PWM     PORTC, 2
#define SRCLK   PORTC, 3
#define RCLK    PORTC, 4
#define SER     PORTC, 5
#define NTC     PORTC, 6
#define RNU     PORTC, 7
#define OLD     PORTB, 0
#define RLY_OUT PORTB, 1
#define RLY_IN  PORTB, 2
#define I_CHG   PORTB, 3
#define RPOF    PORTB, 4
#define SW1     PORTB, 5
#define MNPS    PORTE, 3          ; Main Supply Pos Half Sense (MCLR pin, always i/p)

#define P_HS1     TRISB, 7        ; TRIS bit macro
#define P_LS1     TRISB, 6
#define P_HS2     TRISC, 1
#define P_LS2     TRISC, 0
#define P_I_L     TRISA, 1
#define P_V_B     TRISA, 2
#define P_MFB     TRISA, 3
#define P_FAN     TRISA, 4
#define P_VMNS    TRISA, 5
#define P_PWM     TRISC, 2
#define P_SRCLK   TRISC, 3
#define P_RCLK    TRISC, 4
#define P_SER     TRISC, 5
#define P_NTC     TRISC, 6
#define P_RNU     TRISC, 7
#define P_OLD     TRISB, 0
#define P_RLY_OUT TRISB, 1
#define P_RLY_IN  TRISB, 2
#define P_I_CHG   TRISB, 3
#define P_RPOF    TRISB, 4
#define P_SW1     TRISB, 5

#define ANS_I_L     ANSELA, 1     ; ANSEL macro
#define ANS_V_B     ANSELA, 2
#define ANS_MFB     ANSELA, 3
#define ANS_VMNS    ANSELA, 5
#define ANS_OLD     ANSELB, 0
#define ANS_RLY_OUT ANSELB, 1
#define ANS_RLY_IN  ANSELB, 2
#define ANS_I_CHG   ANSELB, 3
#define ANS_RPOF    ANSELB, 4
#define ANS_SW1     ANSELB, 5
                
#define CH_IL   0b00000111       ; AN1
#define CH_VB   0b00001011       ; AN2
#define CH_VMNS 0b00010011       ; AN4
#define CH_ICHG 0b00100111       ; AN9

#define ICHG_MAX  122            ; Max charging current (5 A)
#define ICHG_TH   120            ; Charging current threshold (4.5 A)
#define VB_MAX    181            ; Max batt voltage (14.2 V)
#define VB_TH     176            ; Batt voltage threshold (13.8 V)
#define VB_LOW    134            ; Low batt voltage (10.5 V)
#define IL_MAX    100            ; Max load current
#define MNS_TH    6              ; Threshold value for mnsVoltage
#define MNS_MAX   215            ; Max value for mnsVoltage
#define MNS_MIN   40             ; Min value for mnsVoltage
#define T0CHG_MAX 250            ; Min duty cycle
#define T0CHG_MIN 195            ; Max duty cycle
				
#define PSCY  invStatus, 0       ; PSCY = 1  : +Ve half cycle, PSCY = 0 : -Ve half cycle
#define VBR   invStatus, 1       ; VBR = 1   : battery voltage has been read
#define VMNSR invStatus, 2       ; VMNSR = 1 : mains voltage has been read
#define ILR   invStatus, 3       ; ILR = 1   : load current has been read
#define ICHGR invStatus, 4       ; ICHGR = 1 : charging current has been read
#define INVZC invStatus, 5       ; INVZC = 1 : inverter zero-crossing
#define MNSZC invStatus, 6       ; MNSZC = 1 : mains zero-crossing
#define INVON invStatus, 7       ; INVON = 1 : inverter on
#define ZC_STATUS_T 0b00011011   ; @zc toggle: ICHGR, ILR, VBR, PSCY

#define RLYION  invStatus2, 0    ; RLYION = 1  : Relay In (Main supply relay) is ON
#define RLYOON  invStatus2, 1    ; RLYOON = 1  : Relay Out is ON
#define LB      invStatus2, 2    ; LB = 1      : Low batt voltage
#define OL      invStatus2, 3    ; OL = 1      : Over load
#define MNSON   invStatus2, 4    ; MNSON = 1   : Main supply ON
#define CHGON   invStatus2, 5    ; CHGON = 1   : Charging ON
#define ECO     invStatus2, 6    ; ECO = 1     : Eco mode ON
#define SW1ON   invStatus2, 7    ; SW1ON = 1   : SW1 (Inverter On switch) is at ON position

#define QRST    quickflag, 0
#define QSET    quickflag, 1
#define LEDUPDT quickflag, 2     ; LEDUPDT = 1 : led satus has been updated
#define MNPH    quickflag, 3     ; MNPH = 1    : main supply at pos half
#define DCRI    quickflag, 4     ; DCRI = 1    : decrease i value
#define PHMCH   quickflag, 5     ; PHMCH = 1   : phase of mns and inv are matched
#define CHGER   quickflag, 6     ; CHGER = 1   : charging error

#define BUZ      ledReg, 1       ; Buzzer
#define LED_RUPS ledReg, 2       ; Indicator LED for RUPS
#define LED_ECO  ledReg, 3       ; Indicator LED for ECO mode
#define LED_OL   ledReg, 4       ; Indicator LED for Over Load
#define LED_CHG  ledReg, 5       ; Indicator LED for Charging
#define LED_MAIN ledReg, 6       ; Indicator LED for Mains On
#define LED_LB   ledReg, 7       ; Indicator LED for Low Battery
#define MSK_BUZ  0xFD            ; Mask BUZ flag leaving the LED flags intact
#define MSK_LED  0x02            ; Mask all LED flags leaving the BUZ flag intact

#define BUZCLONE   blinkLed, 0   ; Holds the status for BUZ flag during BEEP On
#define BEEP       blinkLed, 1   ; Beep the Buzzer
#define BLINK_RUPS blinkLed, 2   ; Blink LED for RUPS
#define BLINK_ECO  blinkLed, 3   ; Blink LED for ECO mode
#define BLINK_OL   blinkLed, 4   ; Blink LED for Over Load
#define BLINK_CHG  blinkLed, 5   ; Blink LED for Charging
#define BLINK_MAIN blinkLed, 6   ; Blink LED for Mains On
#define BLINK_LB   blinkLed, 7   ; Blink LED for Low Battery

table        equ 0x20            ; Lookup table entry starting address (# total entry = 50; 20h to 51h)
temp_ledReg  equ 0x53
blinkLed     equ 0x54            ; Flags for blinking LEDs
invStatus2   equ 0x55            ; DB7 = SW1ON, U, U, U, MNSON, INVMODE, RLY2, DB0 = RLY1
chgDuty      equ 0x56            ; Timer0 value for setting chg duty cycle
i            equ 0x5A            ; Index for lookup table
invStatus    equ 0x5B            ; DB7 = INVON, MNSZC, INVZC, ICHGR, ILR, VMNSR, VBR, DB0 = PSCY
batVoltage   equ 0x5C
mnsVoltage   equ 0x5D
loadCurrent  equ 0x5E
chgCurrent   equ 0x5F
beepCounter  equ 0x60
quickflag    equ 0x61
errCount     equ 0x62
tick500us    equ 0x63
tick100ms    equ 0x64
srCounter    equ 0x65
ledReg       equ 0x66            ; Holds the states for the LEDs (DB7 = LED_LB, LED_MAIN, LED_CHG, LED_OL, LED_ECO, RUPS, BUZ, DB0 = U)
timePrev     equ 0x67            ; Holds the time snapped previously
delayCounter equ 0x68            ; Counter for x(100 ms) delay (e.g. delayCounter = 5 : 500 ms delay)
timeMnsOn    equ 0x69            ; Holds the time for mains voltage arrival
tick500usPre equ 0x6A
mnsPeak      equ 0x6B
timeFanOn    equ 0x6C

temp_w       equ 0x70            ; Shadow registers
temp_status  equ 0x71
temp_fsr     equ 0x72 
temp_pclath  equ 0x73

PSECT resetVect, class = CODE, delta = 2    
resetVect:
    PAGESEL main
    goto main

PSECT CODE, delta = 2
main:
    org 0x00
    goto start_main
     
    org 0x04                     ; Default ISR_vector
    banksel 0                    ; Register bank 0
    movwf temp_w                 ; Store w in shadow reg
    swapf STATUS, w              ; store STATUS_REG in shadow reg
    movwf temp_status            ; (swap is used to avoid change in status flags)
                                 ; Goto user defined interrupt service routine
    btfss INVON                  ; INVON = 1 ?
    goto locate_isr_chg          ; No : goto ISR for chg
locate_isr_inv:                  ; ISR locations for inverter mode
    btfss TMR1IF                 ; Yes: timer1 IF = 1 ?
    goto ISR_t0                  ; No : goto ISR for timer0 overflow
    goto ISR_t1                  ; Yes: goto ISR for timer1 compare match
locate_isr_chg:                  ; ISR locations for charging mode
    btfss TMR1IF                 ; Timer1 IF = 1 ?
    goto ISR_t0_CHG              ; No : goto ISR for timer0 overflow
    goto ISR_t1_CHG              ; Yes: goto ISR for timer1 compare match

start_main:                      ; Program counter comes to this location after reset
    banksel i
        
    ;;;;;;;;;;;;;;;;;;;;; LOOKUP TABLE ;;;;;;;;;;;;;;;;;;;;;
    movlw table
    incf FSR, f
    movlw 255
    movwf INDF
    incf FSR, f
    movlw 251
    movwf INDF
    incf FSR, f
    movlw 248
    movwf INDF
    incf FSR, f
    movlw 244
    movwf INDF
    incf FSR, f
    movlw 241
    movwf INDF
    incf FSR, f
    movlw 237
    movwf INDF
    incf FSR, f
    movlw 234
    movwf INDF
    incf FSR, f
    movlw 230
    movwf INDF
    incf FSR, f
    movlw 227
    movwf INDF
    incf FSR, f
    movlw 223
    movwf INDF
    incf FSR, f
    movlw 220
    movwf INDF
    incf FSR, f
    movlw 216
    movwf INDF
    incf FSR, f
    movlw 213
    movwf INDF
    incf FSR, f
    movlw 210
    movwf INDF
    incf FSR, f
    movlw 206
    movwf INDF
    incf FSR, f
    movlw 203
    movwf INDF
    incf FSR, f
    movlw 200
    movwf INDF
    incf FSR, f
    movlw 197
    movwf INDF
    incf FSR, f
    movlw 194
    movwf INDF
    incf FSR, f
    movlw 191
    movwf INDF
    incf FSR, f
    movlw 188
    movwf INDF
    incf FSR, f
    movlw 185
    movwf INDF
    incf FSR, f
    movlw 182
    movwf INDF
    incf FSR, f
    movlw 180
    movwf INDF
    incf FSR, f
    movlw 177
    movwf INDF
    incf FSR, f
    movlw 175
    movwf INDF
    incf FSR, f
    movlw 172
    movwf INDF
    incf FSR, f
    movlw 170
    movwf INDF
    incf FSR, f
    movlw 167
    movwf INDF
    incf FSR, f
    movlw 165
    movwf INDF
    incf FSR, f
    movlw 163
    movwf INDF
    incf FSR, f
    movlw 161
    movwf INDF
    incf FSR, f
    movlw 159
    movwf INDF
    incf FSR, f
    movlw 157
    movwf INDF
    incf FSR, f
    movlw 156
    movwf INDF
    incf FSR, f
    movlw 154
    movwf INDF
    incf FSR, f
    movlw 153
    movwf INDF
    incf FSR, f
    movlw 151
    movwf INDF
    incf FSR, f
    movlw 150
    movwf INDF
    incf FSR, f
    movlw 149
    movwf INDF
    incf FSR, f
    movlw 148
    movwf INDF
    incf FSR, f
    movlw 147
    movwf INDF
    incf FSR, f
    movlw 146
    movwf INDF
    incf FSR, f
    movlw 145
    movwf INDF
    incf FSR, f
    movlw 144
    movwf INDF
    incf FSR, f
    movlw 144
    movwf INDF
    incf FSR, f
    movlw 144
    movwf INDF
    incf FSR, f
    movlw 143
    movwf INDF
    incf FSR, f
    movlw 143
    movwf INDF
    incf FSR, f
    movlw 143
    movwf INDF
    ;;;;;;;;;;;;;;;;;;;;;; END OF TABLE ;;;;;;;;;;;;;;;;;;;;;;
    
    ; Clear vars
    clrf blinkLed     
    clrf invStatus2   
    clrf chgDuty      
    clrf i            
    clrf invStatus    
    clrf batVoltage   
    clrf mnsVoltage   
    clrf loadCurrent  
    clrf chgCurrent   
    clrf beepCounter  
    clrf quickflag    
    clrf errCount     
    clrf tick500us    
    clrf tick100ms    
    clrf srCounter    
    clrf ledReg       
    clrf timePrev     
    clrf delayCounter 
    clrf timeMnsOn    
    clrf tick500usPre 
    clrf mnsPeak    
    clrf timeFanOn
   
    bsf VBR
    bsf VMNSR
    bsf ILR
    bsf ICHGR
    bsf PSCY                     ; To start the inverter with positive half
    bsf INVON

init_IO_Pins:
    banksel PORTA                ; Clear all  ports
    clrf PORTA
    clrf PORTB
    clrf PORTC
    
    banksel TRISA                ; TRIS reg
    bcf P_HS1                    ; Output pins
    bcf P_LS1
    bcf P_HS2
    bcf P_LS2
    bcf P_FAN
    bcf P_PWM
    bcf P_SRCLK
    bcf P_RCLK
    bcf P_SER
    bcf P_RLY_OUT
    bcf P_RLY_IN
    
    bsf P_I_L                    ; Input pins
    bsf P_V_B
    bsf P_MFB
    bsf P_VMNS
    bsf P_NTC
    bsf P_RNU
    bsf P_OLD
    bsf P_I_CHG
    bsf P_RPOF
    bsf P_SW1

    banksel ANSELA               ; Set pins as analog/digital
    clrf ANSELA
    clrf ANSELB
    bsf ANS_I_L                  ; Analog pins
    bsf ANS_V_B
    bsf ANS_VMNS
    bsf ANS_I_CHG
	
initADC:                         ; Initialize the ADC module
    banksel ADCON1
    movlw 0b00100000             ; ADC clk = f_osc / 32, Vref = VDD
    movwf ADCON1                 ; (see ds_PIC16F722 p. 100)
    banksel ADCON0
    bsf ADON                     ; Turn ON ADC module
    
init_Timer0:                     ; Initialize timer0 for inv mode without enabling the INT
    banksel OPTION_REG           ; Register bank 1
    bcf T0CS                     ; T0CS = 0 : Clk source = internal
    bcf PSA                      ; PSA = 0  : Prescaler is assigned to timer0
    bcf PS2                      ; PS2 = 0  : Prescaler = 1:2
    bcf PS1                      ; PS1 = 0
    bcf PS0                      ; PS0 = 0
    banksel 0                    ; Register bank 0
    
init_PWM:                        ; Setup PWM @ 20 kHz, 30% duty cycle
    movlw 0b01001100             ; for generating -5 V for Op Amp Vss supply
    movwf T2CON                  ; Postscaler = 1:10, Timer2 On, Prescaler = 1:1
    banksel PR2
    movlw 249
    movwf PR2                    ; PWM freq = 20 kHz
    banksel 0
    movlw 75
    movwf CCPR1L                 ; PWM duty cycle = 30%
    movlw 0x0F                   ; CCP1 LSB bits DB<1:0> = 00, PWM mode
    movwf CCP1CON
    
test_HW:                         ; Test hardware e.g. LEDs, FAN, BUZZER etc.
    movlw 0xFF                   ; Turns on all LEDs including buzzer
    movwf ledReg
    call LED_UPDATE
    bsf FAN                      ; Turn on fan
    movlw 8
    movwf delayCounter           ; Load timer for delay 0.8 s (8 * 100 ms)
    call DELAY_100MSX
    clrf ledReg                  ; Turn off all LEDs and buzzer
    call LED_UPDATE
    bcf FAN                      ; Turn off fan
    
init_system:
    bcf INVON
    bsf MNSON
    goto loopForever
mns_off:
    btfss SW1                    ; No : SW1 = 1 ?
    goto mns_on                  ; No : goto sw_off
    call INV_ON                  ; Yes: turn on inverter
    btfsc INVON                  ; Inv on success ?
    goto loopForever             ; Yes: goto main loop
    call READ_MNS                ; Read mian supply voltage
    goto led_update_cont         ; Test mains voltage
mns_on:
    btfsc INVON
    call INV_OFF                 ; If inverter is on then call INV_OFF
    goto loopForever
    
loopForever:
    banksel 0                    ; Bank 0
    btfss INVON                  ; Inverter on = 1 ?
    goto mns_mode                ; No : goto mns mode
;################################## INV MODE ###################################
inv_mode:                        ; Yes: inverter mode
    btfss SW1
    goto mns_on
    
    movf i, w
    btfss ZERO                   ; i = 0 ?
    goto clr_INVZC               ; NO : goto clear INVZC
    btfsc INVZC                  ; Yes: INVZC = 0 ?
    goto read_bat_volt           ; Yes: goto read_vb
    bsf INVZC                    ; No : set INVZC = 1
    bcf LEDUPDT                  ; Clear led updated (LEDUPDT) flag
    goto read_bat_volt
clr_INVZC:
    bcf INVZC
    movf i, w
    sublw 45                     ; 45 - i
    btfss ZERO                   ; i = 45 ?
    goto read_bat_volt           ; No : goto read bat voltage
    btfss PSCY                   ; Yes: is inv in pos half ?
    goto match_neg_half          ; No :
    btfss MNPH                   ; Yes: is mns in pos half ?
    goto phase_not_mch
    bsf PHMCH                    ; Yes: set PHMCH(phase matched)
    goto read_bat_volt
match_neg_half:
    btfsc MNPH
    goto phase_not_mch
    bsf PHMCH
    goto read_bat_volt
phase_not_mch:
    bcf PHMCH
read_bat_volt:
    call READ_VB                 ; Yes: read batt voltage
read_load_cur:
    call READ_IL                 ; Yes: Read load current
read_mns_voltage:
    call READ_MNS                ; Read the main suply
switch_over:
    btfss MNSZC
    goto test_err
    btfss MNSON
    goto skip_switch_over
    btfss PHMCH
    goto skip_switch_over
    movf i, w
    sublw 5                      ; 10 - i
    btfss CARRY
    goto skip_switch_over
    goto mns_on
skip_switch_over:
    bcf MNSZC
test_err:
    movf errCount, w
    sublw 200                    ; 200 - errCount
    btfss CARRY                  ; errCount > 200 ?
    goto mns_on                  ; Yes: turn off inverter
    btfsc LB                     ; LB = 1 ?
    goto led_update_cont         ; Yes: goto led update & continue
    btfsc OL                     ; OL = 1 ?
    goto led_update_cont         ; Yes: goto led update & continue
    clrf errCount                ; If LB = 0 & OL = 0 then errCount = 0
    goto led_update_cont
;################################## MNS MODE ###################################
mns_mode:
    call READ_MNS                ; Read mian supply voltage
    btfsc MNSON
    goto skip_sw_test
    btfsc SW1
    goto mns_off                 ; If mains failure & sw1 ON then goto mans_off
    btfsc CHGON
    call CHG_OFF
    bcf LED_MAIN
    goto skip_chg_on
    
skip_sw_test:    
    btfss CHGON                  ; CHGON = 1?
    call CHG_ON                  ; No : call CHG_ON
    bsf LED_MAIN
    
    btfss MNSZC
    goto skip_chg_on
    btfss MNPH
    goto standby_pos_half
    bcf PSCY
    goto clear_mns_zc
standby_pos_half:
    incf i
    movf i, w
    sublw 250                    ; 250 - i
    btfsc CARRY
    goto skip_chg_on
    bsf PSCY
clear_mns_zc:
    bcf MNSZC
    clrf i
skip_chg_on:
    movf tick500usPre, w         ; Test 10 ms passed ?
    subwf tick500us, w           ; tick500us - tick500usPre
    btfsc CARRY
    goto skip_ovf_adj
    movf tick500usPre, w         ; 
    sublw 199                    ; 199 - tick500usPre
    addwf tick500us, w           ; (199 - tick500usPre) + tick500us
skip_ovf_adj:
    sublw 39                     ; 39 - (tick500us ~ tick500usPre)
    btfsc CARRY                  ; (tick500us = tick500usPre) > 18 ?
    goto led_update_cont         ; No : Don't increase or decrease duty cycle now
    movf tick500us, w            ; Yes: Recor current tick500us value
    movwf tick500usPre
    bcf LEDUPDT
    
    btfss CHGON
    goto led_update_cont
    call READ_VB                 ; No : read batt voltage
    call READ_ICHG               ; Read charging current
    
    movf chgCurrent, w
    sublw ICHG_MAX               ; ICHG_MAX - chgCurrent
    btfss CARRY                  ; chgCurrent > ICHG_MAX ?
    goto dec_duty                ; Yes: goto dec_duty
    
    movf batVoltage, w
    sublw VB_MAX                 ; VB_MAX - batVoltage
    btfss CARRY                  ; batVoltage > VB_MAX?
    goto dec_duty                ; Yes: goto dec_duty
    
    movf chgCurrent, w           ; No : test chgCurrent > ICHG_TH?
    sublw ICHG_TH                ; ICHG_TH - chgCurrent
    btfss CARRY                  ; chgCurrent > ICHG_TH?
    goto rec_time_now            ; Yes: don't increase duty cycle
    movf batVoltage, w           ; No : test batVoltage > V_TH?
    sublw VB_TH                  ; VB_TH - batVoltage
    btfss CARRY                  ; batVoltage > V_TH?
    goto rec_time_now            ; Yes: don't increase duty cycle
inc_duty:
    movf chgDuty, w              ; No : increase duty cycle
    sublw T0CHG_MIN              ; T0CHG_MIN - chgDuty
    btfsc CARRY                  ; chgDuty > T0CHG_MIN?
    goto skip_duty_inc
    decf chgDuty, f              ; Yes: chgDuty--
    goto rec_time_now            ; Goto check for over load
skip_duty_inc:
    movf chgCurrent, w
    sublw 35                     ; 35 - chgCurrent
    btfsc CARRY
    goto test_chg_ok             ; chgCurrent <= 35
    goto rec_time_now            ; Check for over load
dec_duty:
    movlw T0CHG_MAX
    subwf chgDuty, w             ; chgDuty - T0CHG_MAX
    btfsc CARRY                  ; chgDuty < T0CHG_MAX?
    goto test_chg_ok
    incf chgDuty, f              ; Yes: chgDuty++
    goto rec_time_now            ; Check for over load
test_chg_ok:
    movf timeMnsOn, w            ; Test if 3 s elapsed
    subwf tick100ms, w           ; tick100ms - timeMnsOn
    btfsc CARRY
    goto skip_ovf_adj2
    movf timeMnsOn, w            ; 
    sublw 255                    ; 255 - timeMnsOn
    addwf tick100ms, w           ; (255 - timeMnsOn) + tick100ms
skip_ovf_adj2:
    sublw 29                     ; 29 - (tick100ms ~ timeMnsOn)
    btfsc CARRY                  ; (tick100ms ~ timeMnsOn) > 29 ?
    goto test_ol                 ; No : check for over load
    bsf CHGER                    ; Yes: Set charging err flag
    call CHG_OFF                 ; Turn off charging
    goto led_update_cont
rec_time_now:
    movf tick100ms, w
    movwf timeMnsOn
test_ol:
    call READ_IL
    movf errCount, w
    sublw 29                     ; 29 - errCount
    btfsc CARRY
    goto skip_err_sd
    bsf CHGER
    call CHG_OFF
    clrf errCount
    goto test_chg_led
skip_err_sd:
    btfss OL                     ; Over load flag = 1 ?
    clrf errCount                ; No : clear error counter
test_chg_led:
    movf chgCurrent, w
    sublw 45                     ; 45 - chgCurrent (~1 A)
    btfsc CARRY
    goto is_vb_float
    movf chgCurrent, w
    sublw 52                     ; 52 - chgCurrent (~1.5 A)
    btfss CARRY
    bsf LED_CHG
    goto led_update_cont
is_vb_float:
    movf batVoltage, w
    sublw VB_TH                  ; VB_TH - batVoltage
    btfss CARRY
    bcf LED_CHG
led_update_cont:
    btfsc MNPS                   ; MNPS (mains pos half sense) = 0 : mains @+Ve half
    goto mns_neg_half
    btfsc MNPH
    goto skip_mns_test
    bsf MNPH
    goto mns_test
mns_neg_half:
    btfss MNPH
    goto skip_mns_test
    bcf MNPH
mns_test:
    bsf MNSZC                    ; Set flag for mns ZC
    movf mnsPeak, w
    sublw MNS_MIN                ; MNS_MIN - mnsPeak
    btfsc CARRY
    goto mns_failure             ; If mnsVoltage <= MNS_MIN then goto mains failure
    movf mnsPeak, w
    sublw MNS_MAX                ; MNS_MAX - mnsPeak
    btfss CARRY
    goto mns_failure             ; If mnsVoltage > MNS_MAX then goto mains failure
    btfsc MNSON                  ; MNSON = 1 ?
    goto skip_rec_time           ; Yes: skip recording time for mains arrival
    movf tick100ms, w            ; If MNSON flag was not set then 
    movwf timeMnsOn              ; record time when main supply arrives
skip_rec_time:
    bsf MNSON
    clrf mnsPeak
    goto skip_mns_test
mns_failure:
    bcf MNSON                    ; Clear the MNSON flag
    clrf mnsPeak
skip_mns_test:
    btfsc LEDUPDT
    goto continue                ; If LEDUPDT flag = 1 then goto continue
    movlw 8                      ; else send ledReg byte to 74HC595 (visit: https://www.ti.com/product/SN54HC595)
    movwf srCounter              ; Load shift counter = 8
    movf ledReg, w               ; Copy data from ledReg
    movwf temp_ledReg            ; to a temporary register
    bcf SER                      ; Clear serial data pin
    bcf RCLK                     ; Clear the storage register clk
    bcf SRCLK                    ; Clear shift register clk
loop1:
    rlf temp_ledReg, f           ; Take D7 of ledReg into carry
    btfsc CARRY                  ;
    bsf SER                      ; If D7 = 1, set serial data pin
    bsf SRCLK                    ; Set shift register clk
    nop                          ; Give delay to latch data-bit
    bcf SRCLK                    ; Clear shift register clk
    bcf SER                      ; Clear serial data pin
    decfsz srCounter             ; Decrement shift counter
    goto loop1                   ; If shift counter != 0 then goto loop1
    bsf RCLK                     ; Set the storage register clk
    nop                          ; Give delay to latch data-byte
    bcf RCLK                     ; Clear the storage register clk
    bsf LEDUPDT                  ; Set the LEDUPDT flag
       
continue:
    btfsc NTC
    goto fan_on
    movf timeFanOn, w
    subwf tick100ms, w           ; tick100ms - timeFanOn
    sublw 29                     ; 29 - (tick100ms - timeFanOn)
    btfss CARRY
    bcf FAN
    goto skip_fan_on
fan_on:
    bsf FAN
    movf tick100ms, w
    movwf timeFanOn
skip_fan_on:
    btfss TMR2IF                 ; Test timer2 IF
    goto loopForever
    bcf TMR2IF                   ; If timer2 IF is set then clear IF
    incf tick500us, f            ; Increse 500us tick counter
    movlw 199
    subwf tick500us, w           ; tick500us - 199
    btfss CARRY
    goto loopForever
    clrf tick500us               ; If tick500us >= 199 : clear tick500us,
    incf tick100ms, f            ; increse 100 ms tick counter
        
    btfss BEEP                   ; BEEP = 1 ?
    goto skip_buz_off            ; No : don't turn off buzzer
    btfsc BUZ                    ; 'BUZ' = 1 ?
    bsf BUZCLONE                 ; Yes : Set 'BUZCLONE'
    btfss BUZ                    ; Yes: BUZ = 1 ?
    goto skip_buz_off            ; No : don't turn off buzzer
    bcf BUZ                      ; Yes: turn off buzzer
    decfsz beepCounter           ; --beepCounter > 0 ?
    goto skip_buz_off            ; Yes: don't clear BEEP flag
    bcf BEEP                     ; No: clear BEEP flag
    bcf BUZCLONE                 ; clear BUZCLONE
skip_buz_off:
    movf timePrev, w             ; Test if 900 ms has been passed:
    subwf tick100ms, w           ; tick100ms - timePrev = dt_100ms
    sublw 9                      ; 9 - dt_100ms
    btfsc CARRY                  ; tick100ms - timePrev > 9 ?
    goto loopForever             ; No : continue to the inf loop
    btfss BEEP
    goto skip_retrive_buz
    btfsc BUZCLONE               ; Yes: BUZCLONE = 1 ?
    bsf BUZ                      ; Yes: set the BUZ flag
skip_retrive_buz:
    movf blinkLed, w             ; Toggle LED status bits according to
    xorwf ledReg, f              ; the corresponding blink flags
    
    btfss BUZ                    ; Yes: BUZ = 1 ?
    bcf BUZCLONE                 ; No : Clear BUZCLONE
    
skip_beep:
    movf tick100ms, w
    movwf timePrev               ; Snap the current 100ms timer into timePrev
    goto loopForever
    
;############################## FUNC DEFINATIONS ###############################
    
ISR_t1:                          ; ISR for timer1 overflow
    bcf HS1                      ; HS1 = 0
    bcf HS2                      ; HS2 = 0
    bcf TMR1IF                   ; Reset timer1 IF
    
    movf i, w
    btfsc ZERO                   ; i = 0 ?
    bcf DCRI                     ; Yes: DCRI (decrease i) = 0
    movlw 48                     ; Test i < 48 ?
    subwf i, w                   ; i - 48
    btfsc CARRY                  ; i < 48 ?
    bsf DCRI                     ; No : DCRI (decrease i) = 1
    btfss DCRI                   ; DCRI = 1 ?
    goto inc_index               ; No : goto inc_index
    decf i, f                    ; Yes: i--
    goto $+2
inc_index:
    incf i, f                    ; i++
    btfss ZERO                   ; i = 0 ?
    goto skip_zc                 ; No : goto skip toggling
    movlw ZC_STATUS_T            ; Load w with status flags to be toggled
    xorwf invStatus, f           ; @zc ICHGR = 0, ILR = 0, VMNSR = 0, VBR = 0, PSCY = T
skip_zc:
    movf i, w
    sublw 4                      ; 4 - i
    btfsc CARRY                  ; i > 4 ?
    goto setQRST                 ; No : goto setQRST
    bcf QRST                     ; Yes: clear QRST
    goto testQSET                ; Goto test QSET
setQRST:
    bsf QRST                     ; If i <= 4 then QRST = 1
    goto set_ls_high
testQSET:
    movf i, w                    ; Copy i to w
    sublw 34                     ; 34 - i
    btfsc CARRY                  ; i > 34 ?
    goto clearQSET               ; No : clear QSET flag  
setQSET: 
    bsf QSET                     ; If i > 34 then QSET = 1
    goto $+2                     ; Skip the next instruction
clearQSET:
    bcf QSET                     ; If i <= 34 then clear QSET
    
set_ls_high:    
    bsf LS1                      ; LS1 = 1
    bsf LS2                      ; LS2 = 1
    btfsc TMR0IF                 ; Have a quick check if timer0 IF is set
    goto ISR_t0                  ; If timer0 IF is already set, goto t0 INT
    btfss QSET                   ; Test if QSET (quick set) = 1
    goto retFromISR              ; If i >= 63, return from ISR
       
wait_t0INT:                      ; If i < 63, goto wait until timer0 IF is set
    btfsc TMR0IF
    goto ISR_t0
    goto wait_t0INT
		
    goto retFromISR              ; For safty :D
                                 ; ISR_t2 ends here
	
ISR_t0:                          ; ISR for timer0 overflow
    btfsc INVZC                  ; Test if inverter zc = 0
    goto skipHS                  ; If zc = 1, don't set HS = 1
    btfss PSCY                   ; Test if PSCY = 1
    goto negCycle                ; If PSCY = 0, goto -Ve half cycle
posCycle:                        ; If PSCY = 1 (+Ve half cycle)
    bcf LS1                      ; LS1 = 0
    bcf TMR0IF                   ; Reset timer0 IF
    movf  i, w                   ; Copy variable i to w
    addwf FSR, f                 ; FSR = table + i
    movf INDF , w                ; Get data from lookup table array to w
    bsf HS1                      ; HS1 = 1
    goto loadDuty
negCycle:                        ; If PSCY = 0 (-Ve half cycle)
    bcf LS2                      ; LS2 = 0
    bcf TMR0IF                   ; Reset timer0 IF
    movf  i, w                   ; Copy variable i to w
    addwf FSR, f                 ; FSR = table + i
    movf INDF , w                ; Get data from lookup table array to w
    bsf HS2                      ; HS2 = 1
    goto loadDuty                ;
skipHS:
    bcf TMR0IF                   ; Reset timer0 IF
    movf  i, w                   ; Copy variable i to w
    addwf FSR, f                 ; FSR = table + i
    movf INDF , w                ; Get data from lookup table array to w
loadDuty:
    movwf TMR1L                  ; Copy value from w to TMR1L
    clrf TMR1H
    bcf TMR1IF                   ; Clear timer1 IF to avoid false INT
    comf TMR1H                   ; Timer1 high-order byte = FFh
    btfss QRST                   ; Test if QRST (quick reset) = 1
    goto retFromISR              ; If QRST = 0 then return from ISR
wait_t1INT:                      ; Wait until timer1 IF is set
    btfsc TMR1IF
    goto ISR_t1
    goto wait_t1INT
    goto retFromISR              ; For safty :D
    ; ISR_t0 ends here
        
ISR_t0_CHG:
    bcf LS1
    bcf LS2
    bcf TMR0IF
    goto retFromISR
    
ISR_t1_CHG:
    bcf HS1
    bcf HS2
    bsf LS1
    bsf LS2
    bcf TMR1IF
    movlw 110
    movwf TMR1L                  ; TMR1L = 110 for 15.8 kHz frequency
    comf TMR1H, f                ; Timer1 lower byte = FFh
reloadTimer0:
    movf chgDuty, w
    movwf TMR0
    bcf TMR0IF
retFromISR:                      ; Common routine for returning from ISR
    movlw table                  ; 
    movwf FSR                    ; Reset to FSR = table
    swapf temp_status, w         ; Retrive STATUS_REG value
    movwf STATUS                 ; Move w to f will not affect the STATUS_REG
    swapf temp_w, f              ; Retrive w content
    swapf temp_w, w              ; (Swap instruction is used so that any flag in STATUS is not modified)
    retfie
    
READ_MNS:                        ; Read main supply voltage
    movlw 0b11000011             ; Clear the analog channel select bits and GO flag
    andwf ADCON0, f
    movlw CH_VMNS                ; Load channel address for VMNS and set GO flag
    iorwf ADCON0, f
    btfsc GO                     ; Is conversion done?
    goto $-1                     ; No, check GO flag again
    movf ADRES, w                ; Read ADC result
    movwf mnsVoltage             ; Store result
    subwf mnsPeak, w             ; mnsPeak - mnsVoltage
    btfsc CARRY                  ; mnsVoltage > mnsPeak ?
    goto skip_peak_updt          ; No : continue without updating peak voltage
    movf mnsVoltage, w           ; Yes: Update mains peak voltage
    movwf mnsPeak
    
skip_peak_updt:
    movf mnsVoltage, w
    sublw MNS_TH                 ; MNS_TH - mnsVoltage
    btfsc CARRY                  ; mnsVoltage > MNS_TH ?
    bcf MNSON                    ; No : clear MNSON
    movf mnsPeak, w
    sublw MNS_MAX                ; MNS_MAX - mnsPeak
    btfss CARRY                  ; mnsPeak > MNS_MAX ?
    bcf MNSON                    ; Yes: clear MNSON
    return
    
INV_ON:                          ; Turn ON the inverter
    bcf CHGON                    ; Clear charging on flag
    btfsc LB                     ; Is low batt flag on ?
    goto inv_on_fail             ; Yes: goto inv_on_fail
    
    bcf RLY_IN                   ; No : Turn off main supply input relay
    bcf RLYION                   ; Clear flag for input relay on
    bcf RLY_OUT                  ; Turn off output relay (relay2)
    bcf RLYOON                   ; Clear flag for output realy on
    
    bcf GIE                      ; Disable global INT
    bcf LS1                      ; Clear LS1 & LS2 in case charging was on
    bcf LS2
    
    movlw 0x21          
    movwf T1CON                  ; Timer1 Prescaler = 1:4, Timer1 Enable
    
    movlw table                  ; w = beginning address of the lookup table
    movwf FSR                    ; FSR = table_beginning
    clrf i                       ; i = 0
    bcf DCRI                     ; DCRI (decrease i) = 0

    bsf INVON                    ; Set flag for inverter on
    clrf errCount
    clrf ledReg                  ; Turn off all LEDs
    bsf LED_ECO                  ; Turn on ECO LED
    movlw 5                      ; Beep 5 times
    movwf beepCounter            ; 
    bsf BEEP                     ; Set BEEP flag to start beeping
    
    bsf PEIE                     ; Enable Peripheral INT
    bsf GIE                      ; Enable Global INT
    
    clrf TMR1H
    clrf TMR1L
    bcf TMR1IF                   ; TMR1IF = 0
    banksel PIE1                 ; Register bank 1
    bsf TMR1IE                   ; Enable Timer1 INT
    banksel 0                    ; Register bank 0
    
    clrf TMR0                    ; TMR0 = 0
    bcf TMR0IF                   ; TMR0IF = 0
    bsf TMR0IE                   ; Enable Timer0 INT
    
    movlw 0xFF
    movwf TMR1H                  ; Timer1 higher order byte = FFh
    movlw 157         
    movwf TMR1L                  ; TMR1 = 157
    
    return                       ; Return (Inverter ON was successful)    
inv_on_fail:
    bcf INVON
    retlw 0                     ; Return with w = 0 (inverter ON was a failure)
    
INV_OFF:                        ; Turn OFF inverter
    banksel INTCON
    bcf GIE                     ; Disable Global INT
    bcf HS1                     ; Turn off all transistors
    bcf HS2                     ; in the H-bridge
    bcf LS1                     ;
    bcf LS2                     ;
    bcf PEIE                    ; Disable Peripheral INT
    
    bsf RLY_IN                  ; Turn on input relay
    bsf RLYION
    bcf RLY_OUT                 ; Turn off output relay
    bcf RLYOON
    
    bcf TMR0IE                  ; Disable Timer0 INT
    banksel PIE1                ; Register bank 1
    bcf TMR1IE                  ; Disable Timer1 INT
    banksel 0                   ; Register bank 0
    bcf INVON                   ; Clear flag for inverter on
    bcf CHGER                   ; Clear charging error flag
    btfss MNSON                 ; Main supply on ?
    goto no_normal_sd           ; No : goto no-normal-shut-down
    movf tick100ms, w           ; Record time 
    movwf timeMnsOn             ; when inv turns on
    bsf LED_MAIN                ; Yes: turn on MAIN LED
    bcf LED_OL                  ; Turn off OL LED
    bcf LED_LB                  ; Turn off low batt LED
    bcf BLINK_CHG
    bcf LED_CHG
    bcf BUZ                     ; Turn off buzzer
    return                      ; Return
no_normal_sd:
    btfss SW1
    goto skip_no_normal_wait
    btfsc LB                    ; Low batt voltage ?
    bsf LED_LB                  ; Yes: turn on LB LED
    btfsc OL                    ; Over load ?
    bsf LED_OL                  ; Yes: turn on OL LED
    bsf BUZ                     ; Turn on buzzer
    call LED_UPDATE             ; 
    movlw 70                    ;
    movwf delayCounter          ; Load timer for delay 7 s (70 * 100 ms)
    call DELAY_100MSX           ; Wait 7 s
skip_no_normal_wait:
    bcf BUZ                     ; Turn off buzzer
    bcf BEEP                    ; Turn off beep if enabled
    btfss SW1
    clrf ledReg
    call LED_UPDATE
    return
    
CHG_ON:
    btfsc CHGER                 ; Chg err flag = 1 ?
    return                      ; Don't turn on charging
    movf timeMnsOn, w
    subwf tick100ms, w          ; tick100ms - timeMnsOn
    btfsc CARRY
    goto skip_ovf1              ; If tick100ms >= timeMnsOn then goto skip_ovf1
    movf timeMnsOn              ; If tick100ms < timeMnsOn then adjust for overflow
    sublw 255
    addwf tick100ms, w
skip_ovf1:
    sublw 20                    ; 20 - w : Test if 2 s elapsed after mains arrival
    btfsc CARRY
    return                      ; If 2 s is not passed yet then return
turn_on_chg:
;    bsf RLY_OUT                 ; Turn on output relay
;    bsf RLYOON
    bsf LED_CHG                 ; Turn on
    call LED_UPDATE             ; the CHG LED
    movlw 240                   ; Load duty cycle with 240
    movwf chgDuty               ; chgDuty = 110 : 100% duty cycle, chgDuty = 255 : 5% duty cycle
    movwf TMR0                  ; TMR0 = 240
    movlw 0xFF
    movwf TMR1H                 ; Timer1 higher order byte = FFh
    movlw 110
    movwf TMR1L                 ; TMR1L = 110 for ~15.8 kHz frequency
    movlw 0x11          
    movwf T1CON                 ; Timer1 Prescaler = 1:2, Timer1 Enable
    bcf TMR0IF                  ; TMR0IF = 0
    bcf TMR1IF                  ; TMR1IF = 0
    bsf GIE                     ; Enable Global INT
    bsf PEIE                    ; Enable Peripheral INT
    bsf TMR0IE                  ; Enable Timer0 INT
    banksel PIE1                ; Register bank 1
    bsf TMR1IE                  ; Enable Timer1 INT
    banksel 0                   ; Register bank 0
    bsf CHGON                   ; Set charging on flag
    bsf LED_MAIN
    bsf LED_CHG                 ; Set flag for charging LED
    bcf BLINK_CHG               ; Stop blinking chg LED
    return                      ; Return (Charging ON was successful)
    
 CHG_OFF:
    banksel INTCON
    bcf GIE                     ; Disable Global INT
    bcf HS1                     ; Turn off all transistors
    bcf HS2                     ; in the H-bridge
    bcf LS1                     ;
    bcf LS2                     ;
    bcf PEIE                    ; Disable Peripheral INT
    bcf TMR0IE                  ; Disable Timer0 INT
    banksel PIE1                ; Register bank 1
    bcf TMR1IE                  ; Disable Timer1 INT
    banksel 0                   ; Register bank 0
    bcf CHGON                   ; Clear charging on flag
    bcf LED_CHG                 ; Clear flag for charging on LED
    bcf RLY_OUT                 ; Turn off output relay
    bcf RLYOON
    btfss MNSON
    bcf LED_CHG
    btfss CHGER                 ; Chg err flag = 1 ?
    return
    bsf BLINK_CHG               ; Yes: blink chg led
    return
    
READ_VB:                        ; Read batt voltage
    movlw 0b11000011            ; Mask for clearing the CHS bits
    andwf ADCON0, f             ; Clear the analog channel select bits
    movlw CH_VB                 ; Load the ADC channel address 
    iorwf ADCON0, f             ; & set GO falg
    btfsc GO                    ; Is conversion done?
    goto $-1                    ; No, check GO flag again
    movf ADRES, w               ; Read ADC result
    movwf batVoltage            ; Store result
    sublw VB_LOW                ; VB_LOW - batVoltage
    btfss CARRY                 ; batVoltage > VB_LOW ?
    goto batt_volt_ok           ; Yes: goto batt voltage ok
    bsf LB                      ; No : set the LB (low batt) flag
    incf errCount, f            ; errCount++
    goto $+2
batt_volt_ok:
    bcf LB
    bsf VBR                     ; Set VBR to indicate VB has been read
    return
    
READ_IL:                        ; Read load current
    btfss INVON                 ; Inv ON ?
    goto load_cur_ok            ; No : test over load detection pin
    movlw 0b11000011            ; Mask for clearing the CHS bits
    andwf ADCON0, f             ; Clear the CHS bits
    movlw CH_IL                 ; Load the ADC channel address & set GO falg
    iorwf ADCON0, f             ;
    btfsc GO                    ; Is conversion done?
    goto $-1                    ; No, check GO flag again
    movf ADRES, w               ; Read ADC result
    movwf loadCurrent           ; Store result
    sublw IL_MAX                ; IL_MAX - loadCurrent
    btfsc CARRY                 ; loadCurrent < IL_MAX ?
    goto load_cur_ok            ; Yes: goto load current ok
    bsf OL                      ; No : set over load (OL) flag
    goto $+2
load_cur_ok:
    bcf OL
    btfsc OLD                   ; OLD (over load detection) = HIGH ?
    bsf OL                      ; Yes: set OL flag
    btfsc OL                    ; OL = 1 ?
    incf errCount, f            ; Yes: errCount++
    bsf ILR                     ; Set ILR to indicate IL has been read
    return
    
READ_ICHG:                      ; Read charging current
    movlw 0b11000011            ; Mask for clearing the CHS bits
    andwf ADCON0, f             ; Clear the analog channel select bits
    movlw CH_ICHG               ; Load the ADC channel address 
    iorwf ADCON0, f             ; & set GO falg
    btfsc GO                    ; Is conversion done?
    goto $-1                    ; No, check GO flag again
    movf ADRES, w               ; Read ADC result
    movwf chgCurrent            ; Store result
    bsf ICHGR                   ; Set ICHGR to indicate ICHG has been read
    return
    
LED_UPDATE:                     ; Send ledReg byte to 74HC595 (visit: https://www.ti.com/product/SN54HC595)
    movlw 8                     ; 
    movwf srCounter             ; Load shift counter = 8
    movf ledReg, w              ; Copy data from ledReg
    movwf temp_ledReg           ; to a temporary register
    bcf SER                     ; Clear serial data pin
    bcf RCLK                    ; Clear the storage register clk
    bcf SRCLK                   ; Clear shift register clk
loop2:
    rlf temp_ledReg, f          ; Take D7 of ledReg into carry
    btfsc CARRY                 ;
    bsf SER                     ; If D7 = 1, set serial data pin
    bsf SRCLK                   ; Set shift register clk
    nop                         ; Give delay to latch data-bit
    bcf SRCLK                   ; Clear shift register clk
    bcf SER                     ; Clear serial data pin
    decfsz srCounter            ; Decrement shift counter
    goto loop2                  ; If shift counter != 0 then goto loop2
    bsf RCLK                    ; Set the storage register clk
    nop                         ; Give delay to latch data-byte
    bcf RCLK                    ; Clear the storage register clk
    bsf LEDUPDT                 ; Set the LEDUPDT flag
    return
    
DELAY_100MSX:
    clrf tick500us
wait500us:
    btfss TMR2IF                ; Test timer2 IF
    goto $-1
    bcf TMR2IF                  ; If timer2 IF is set then clear IF
    incf tick500us, f           ; Increse 500us tick counter
    movlw 199
    subwf tick500us, w          ; tick500us - 199
    btfss CARRY                 ; tick500us > 199 ?
    goto wait500us              ; No : goto wait500us
    clrf tick500us              ; Yes: clear tick500us,
    decfsz delayCounter, f      ; delayCounter--; delayCounter > 0 ?
    goto wait500us              ; Yes: goto wait500us
    return                      ; No : return from delay
    
    goto main                   ; main ends here
    END resetVect