/*
 * File:   main.c
 * Author: Chloe Fulbrook, Ethan Lee, Devon Calvin
 *
 * Created FOR ENCM 511
 * PLEASE ADD DATE CREATED HERE: 2025-10-02
 * 
 * FAILURE TO UPDATE THIS HEADER WITH YOUR GROUP MEMBER NAMES
 * MAY RESULT IN PENALTIES
 */

// FSEC
#pragma config BWRP = OFF    //Boot Segment Write-Protect bit->Boot Segment may be written
#pragma config BSS = DISABLED    //Boot Segment Code-Protect Level bits->No Protection (other than BWRP)
#pragma config BSEN = OFF    //Boot Segment Control bit->No Boot Segment
#pragma config GWRP = OFF    //General Segment Write-Protect bit->General Segment may be written
#pragma config GSS = DISABLED    //General Segment Code-Protect Level bits->No Protection (other than GWRP)
#pragma config CWRP = OFF    //Configuration Segment Write-Protect bit->Configuration Segment may be written
#pragma config CSS = DISABLED    //Configuration Segment Code-Protect Level bits->No Protection (other than CWRP)
#pragma config AIVTDIS = OFF    //Alternate Interrupt Vector Table bit->Disabled AIVT

// FBSLIM
#pragma config BSLIM = 8191    //Boot Segment Flash Page Address Limit bits->8191

// FOSCSEL
#pragma config FNOSC = FRC    //Oscillator Source Selection->Internal Fast RC (FRC)
#pragma config PLLMODE = PLL96DIV2    //PLL Mode Selection->96 MHz PLL. Oscillator input is divided by 2 (8 MHz input)
#pragma config IESO = OFF    //Two-speed Oscillator Start-up Enable bit->Start up with user-selected oscillator source

// FOSC
#pragma config POSCMD = NONE    //Primary Oscillator Mode Select bits->Primary Oscillator disabled
#pragma config OSCIOFCN = ON    //OSC2 Pin Function bit->OSC2 is general purpose digital I/O pin
#pragma config SOSCSEL = OFF    //SOSC Power Selection Configuration bits->Digital (SCLKI) mode
#pragma config PLLSS = PLL_FRC    //PLL Secondary Selection Configuration bit->PLL is fed by the on-chip Fast RC (FRC) oscillator
#pragma config IOL1WAY = ON    //Peripheral pin select configuration bit->Allow only one reconfiguration
#pragma config FCKSM = CSECMD    //Clock Switching Mode bits->Clock switching is enabled,Fail-safe Clock Monitor is disabled

// FWDT
#pragma config WDTPS = PS32768    //Watchdog Timer Postscaler bits->1:32768
#pragma config FWPSA = PR128    //Watchdog Timer Prescaler bit->1:128
#pragma config FWDTEN = ON_SWDTEN    //Watchdog Timer Enable bits->WDT Enabled/Disabled (controlled using SWDTEN bit)
#pragma config WINDIS = OFF    //Watchdog Timer Window Enable bit->Watchdog Timer in Non-Window mode
#pragma config WDTWIN = WIN25    //Watchdog Timer Window Select bits->WDT Window is 25% of WDT period
#pragma config WDTCMX = WDTCLK    //WDT MUX Source Select bits->WDT clock source is determined by the WDTCLK Configuration bits
#pragma config WDTCLK = LPRC    //WDT Clock Source Select bits->WDT uses LPRC

// FPOR
#pragma config BOREN = ON    //Brown Out Enable bit->Brown Out Enable Bit
#pragma config LPCFG = OFF    //Low power regulator control->No Retention Sleep
#pragma config DNVPEN = ENABLE    //Downside Voltage Protection Enable bit->Downside protection enabled using ZPBOR when BOR is inactive

// FICD
#pragma config ICS = PGD1    //ICD Communication Channel Select bits->Communicate on PGEC1 and PGED1
#pragma config JTAGEN = OFF    //JTAG Enable bit->JTAG is disabled

// FDEVOPT1
#pragma config ALTCMPI = DISABLE    //Alternate Comparator Input Enable bit->C1INC, C2INC, and C3INC are on their standard pin locations
#pragma config TMPRPIN = OFF    //Tamper Pin Enable bit->TMPRN pin function is disabled
#pragma config SOSCHP = ON    //SOSC High Power Enable bit (valid only when SOSCSEL = 1->Enable SOSC high power mode (default)
#pragma config ALTI2C1 = ALTI2CEN    //Alternate I2C pin Location->SDA1 and SCL1 on RB9 and RB8

#include "xc.h"

#define LED0    LATBbits.LATB5
#define LED1    LATBbits.LATB6
#define LED2    LATBbits.LATB7
#define PB1     PORTBbits.RB8
#define PB0     PORTAbits.RA4
#define PB2     PORTBbits.RB9   

#define DEBOUNCE 60

uint16_t PB0_event;         // should add more of these for the states
uint16_t PB1_event;
uint16_t PB2_event;

uint16_t toggle = 0;

uint16_t PB2_times[6] = {62500, 31250, 15625, 7812, 3906, 1953};
                        // 4s,   2s,   1s,    0.5s, 0.25s, 0.125s
uint16_t PB2_times_index = 0;

volatile uint32_t ms_ticks = 0;

/* === NEW: tiny latch to detect PB2 release (press->release = one step) === */
static volatile unsigned char pb2_down = 0;

void IOinit();
void timerInit();
void delay_ms(uint16_t ms);

/**
 * You might find it useful to add your own #defines to improve readability here
 */

int main(void) {
    ANSELA = 0x0000; /* keep this line as it sets I/O pins that can also be analog to be digital */
    ANSELB = 0x0000; /* keep this line as it sets I/O pins that can also be analog to be digital */
    
    // newClk(500);

    IOinit();
    timerInit();
    T1CONbits.TON = 0;
    T2CONbits.TON = 0;
    T3CONbits.TON = 0;
    
    typedef enum{
        STATE_OFF, 
        STATE_PB0,
        STATE_PB1, 
        STATE_PB0_PB1        
    } state_t;
    
    state_t state = STATE_OFF;
    
    /* Let's clear some flags */
    PB0_event = 0;
    PB1_event = 0;
    PB2_event = 0;

    while(1) {     
        switch(state){
            case STATE_OFF:
                while(state == STATE_OFF){
                    LED0 = 0;
                    LED1 = 0; 
                    LED2 = 0;
                    
                    Idle();
                    delay_ms(DEBOUNCE);
                    
                    if(PB0_event == 1 && PB1_event == 1){
                        LED0 = 1;
                        PR2 = 7812;    
                        TMR2 = 0;
                        T2CONbits.TON = 1;
                        state = STATE_PB0_PB1;
                        break;
                    }
                    else if(PB0_event == 1){
                        LED0 = 1;
                        PR2 = 3906; 
                        TMR2 = 0;
                        T2CONbits.TON = 1;
                        state = STATE_PB0;
                        break;
                    }
                    else if(PB1_event == 1){
                        LED1 = 1;
                        T2CONbits.TON = 0;
                        T3CONbits.TON = 1;
                        state = STATE_PB1;
                        break;
                    }
                }
                break;
                
            case STATE_PB0:
                while(state == STATE_PB0){
                    Idle();
                    delay_ms(DEBOUNCE);
                    
                    if(PB0_event == 0){
                        LED0 = 0;
                        T2CONbits.TON = 0;
                        state = STATE_OFF;
                        break;
                    }
                    else if(PB1_event == 1){
                        PR2 = 7812; 
                        TMR2 = 0;
                        state = STATE_PB0_PB1;
                        break;
                    }
                }
                break;
                
            case STATE_PB1:
                while(state == STATE_PB1){
                    Idle();
                    delay_ms(DEBOUNCE);
                    
                    if(PB1_event == 0){
                        T3CONbits.TON = 0;
                        state = STATE_OFF;
                        break;
                    }
                    else if(PB0_event == 1){
                        LED1 = 0;
                        T3CONbits.TON = 0;
                        PR2 = 7812; 
                        TMR2 = 0;
                        T2CONbits.TON = 1;
                        state = STATE_PB0_PB1;
                        break;
                    }
                }
                break;
                
            case STATE_PB0_PB1:
                while(state == STATE_PB0_PB1){
                    Idle();
                    delay_ms(DEBOUNCE);
                    
                    if(PB0_event == 0 && PB1_event == 0){
                        T3CONbits.TON = 0;
                        T2CONbits.TON = 0;
                        state = STATE_OFF;
                        break;
                    }
                    else if(PB1_event == 0){
                        LED1 = 0;
                        T3CONbits.TON = 0;
                        PR2 = 3906;   
                        TMR2 = 0;
                        T2CONbits.TON = 1;
                        state = STATE_PB0;
                        break;
                    }
                    else if(PB0_event == 0){
                        LED0 = 0;
                        T2CONbits.TON = 0;
                        T3CONbits.TON = 1;
                        state = STATE_PB1;
                        break;
                    }
                }
                break;
                
            default:
                break;
        } // end switch statement
    } // end while(1)
    
    return 0;
}

/* Interrupt for delay function on Timer 1 */
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void){
    IFS0bits.T1IF = 0;
    T1CONbits.TON = 0 ;
}

/* Timer 2 interrupt subroutine */
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void){
    IFS0bits.T2IF = 0;
    LED0 ^= 1;          // toggle LED given timer interrupt
}

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void){
    IFS0bits.T3IF = 0;
    LED1 ^= 1;
}

void __attribute__ ((interrupt, no_auto_psv)) _IOCInterrupt(void) {
    // PORTA
    if(IOCSTATbits.IOCPAF){
        if(!PORTAbits.RA4){     // RA4 went low (PB0)
            PB0_event = 1;
        } else {
            PB0_event = 0;
        }
        IOCSTATbits.IOCPAF = 0;
    }
    
    if(IOCSTATbits.IOCPBF){
        // PB1 (RB8)
        if(!PORTBbits.RB8){
            PB1_event = 1;
        } else {
            PB1_event = 0;
        }
        
        // PB2 (RB9) — count ONE step on release (press->release)
        if(!PORTBbits.RB9){          // pressed (active-low)
            pb2_down = 1;
        } else {                     // released (line high)
            if(pb2_down){
                pb2_down = 0;        // consume the press
                PB2_event = 1;       // keep if you want to observe in main (not required)
                PB2_times_index++;
                if(PB2_times_index == 6){
                    PB2_times_index = 0;
                }
                PR3 = PB2_times[PB2_times_index];
                TMR3 = 0;
            }
        }

        IOCSTATbits.IOCPBF = 0;
    }
    
    IFS1bits.IOCIF = 0;     // clear the global IOC flag
}

void IOinit(){
    // set pins for LEDs
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB7 = 0;
    
    LED0 = 0;
    LED1 = 0;
    LED2 = 0;
    
    // B8 is button1 (the top one)
    TRISBbits.TRISB8 = 1;       // RB8 input
    IOCPUBbits.CNPUB8 = 1;      // pullup RB8
    
    PADCONbits.IOCON = 1;       // 
    IOCNBbits.IOCNB8 = 1;       // negative edge
    IOCPBbits.IOCPB8 = 1;       // positive edge
    IOCSTATbits.IOCPBF = 0;     // clear flags
    
    IFS1bits.IOCIF = 0;
    IPC4bits.IOCIP = 3;
    IEC1bits.IOCIE = 1;
    
    // for button 0 (middle)
    TRISAbits.TRISA4 = 1;       // RA4 is input
    IOCPUAbits.CNPUA4 = 1;      // pullup on RA4
    IOCNAbits.IOCNA4 = 1;       // negative edge
    IOCPAbits.IOCPA4 = 1;       // positive edge
    IOCSTATbits.IOCPAF = 0;     // clear port status flags
    
    // for button 2 (RB9) (bottom)
    TRISBbits.TRISB9 = 1;       // RB9 input
    IOCPUBbits.CNPUB9 = 1;      // pullup RB9
    IOCNBbits.IOCNB9 = 1;       // negative edge (press)
    IOCPBbits.IOCPB9 = 1;       // **NEW: rising edge** (release) so we can count on release
    IOCSTATbits.IOCPBF = 0;     // clear flags
}

void timerInit(){
    //T1CON config bits
    T1CONbits.TCKPS = 3;
    T1CONbits.TCS = 0; // use internal clock
    T1CONbits.TSIDL = 0; //operate in idle mode
    IPC0bits.T1IP = 2; //7 is highest and 1 is lowest pri.
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1; //enable timer interrupt
    
    PR1 = 65500;
    TMR1 = 0;
    T1CONbits.TON = 1;
    
    T2CONbits.T32 = 0; // operate timer 2/3 as 16 bit timer
    
    //T2CON config
    T2CONbits.TCKPS = 3;        // 1:256 prescaler
    T2CONbits.TCS = 0;          // internal clock
    T2CONbits.TSIDL = 0;        // continue operation in idle mode
    IPC1bits.T2IP = 2;          // priority
    IFS0bits.T2IF = 0;          // clear flag
    IEC0bits.T2IE = 1;          // interrupt enable
    PR2 = 3906;                 // 0.25s count
    TMR2 = 0;
    T2CONbits.TON = 1;          // timer on
    
    //T3CON config
    T3CONbits.TCKPS = 3; // 1:256
    T3CONbits.TCS = 0;   // internal clock
    T3CONbits.TSIDL = 0; // operate in idle mode
    IPC2bits.T3IP = 2;   // priority
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;   // enable timer interrupt
    TMR3 = 0;
    T3CONbits.TON = 1;
}

void delay_ms(uint16_t ms){
    uint16_t saved_IEC0 = IEC0;
    IEC0 = 0;
    
    // Delay function
    PR1 = (ms / 0.064);
    TMR1 = 0;
    
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    
    T1CONbits.TON = 1;
    Idle();
    
    IEC0 = saved_IEC0;
    IEC0bits.T1IE = 0;     
}
