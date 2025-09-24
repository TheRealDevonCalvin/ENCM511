/*
 * File:   main.c
 * Author: ENTER GROUP MEMBER NAME(S) HERE
 *
 * Created FOR ENCM 511
 * PLEASE ADD DATE CREATED HERE: 2025-XX-XX
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

#define PB0     PORTAbits.RA4
#define PB1     PORTBbits.RB8


void LED0_constant_on(void);
void LED0_blink(uint32_t length_inst);
void LED0_button(void);


int main(void) {
    
    TRISBbits.TRISB5 = 0;       // set rb5 as output (LED0)
    TRISBbits.TRISB6 = 0;       // set rb6 as output (LED1)
    TRISBbits.TRISB7 = 0;       // set rb7 as output (LED2)
        
    TRISAbits.TRISA4 = 1;       // set ra4 as input (PB0)
    TRISBbits.TRISB8 = 1;       // set rb8 as input (PB1)
    
    IOCPUAbits.CNPUA4 = 1;      // pullup on ra4 for button
    IOCPUBbits.CNPUB8 = 1;      // pullup on rb8 for button 
    
    
    
    while(1){
        
        if(!PB0 && !PB1){       // if both buttons are pressed
            LED2 = 1;
        }
        
        else if(!PB0 && PB1){   // else if button 0 is pressed and button 1 is not pressed
            /*
             * 8MHz clock cycle, program counter increments at 4MHz
             * so program counter increments once every 250ns ???
             * to get 0.25s, need to complete 1_000_000 instructions
             * because 0.25s / (250ns) = 1_000_000
             * from the disassembly view, the for loop has approximately 20 instructions
             * so 1000000 / 20 = 50000, so the for loops should count approx 50k times for 0.25s
             */
            
            uint32_t on025, off025;
            // LED0 on for count duration
            for(on025 = 0; on025 <= 50000; on025++){
                LED0 = 1;
                if(PB0){    // if button 0 is released, turn off LED0 and break the loop
                    LED0 = 0;
                    break;
                }
            }
            
            // LED0 off for count duration
            for(off025 = 0; off025 <= 50000; off025++){
                LED0 = 0;
                if(PB0){    // if button 0 is released, break the loop
                    break;
                }
            }
        } // end else if(!PB0 && PB1)
        
        else if(!PB1 && PB0){   // else if button 1 is pressed and button 0 is not pressed
            /*
             * 8MHz clock cycle, program counter increments at 4MHz (250ns)
             * to get 2s, need to complete 8million instructions
             * because 2s / (250ns) = 8_000_000
             * from disassembly, roughly 20 instructions in the for loop
             * so 8000000 / 20 = 400000, so for loops should count until 400k times for 2s
             */
            
            uint32_t on2, off2;
            // LED1 on for count duration
            for(on2 = 0; on2 <= 400000; on2++){
                LED1 = 1;
                if(PB1){    // if button 1 is released, turn off LED1 and break the loop
                    LED1 = 0;
                    break;
                }
            }
            
            // LED0 off for count duration
            for(off2 = 0; off2 <= 400000; off2++){
                LED1 = 0;
                if(PB1){    // if button 1 is released, break the loop
                    break;
                }
            }
        } // end else if(!PB1 && PB0)
        
        
        else {      // otherwise all LEDs off
            LED0 = 0;
            LED1 = 0;
            LED2 = 0;
        } // end else
        
        
    } // end while(1)
    
    return 0;
} // end main(void)



void LED0_constant_on(void){
    LED0 = 1;
}

void LED0_blink(uint32_t length_inst){
    while(1){
        for(uint32_t i = 0; i < length_inst; i++){
            LED0 = 1;
        }
        for(uint32_t j = 0; j < length_inst; j++){
            LED0 = 0;
        }
    }
}

void LED0_button(void){
    if(!PB1){
        LED0 = 1;
    }
    if(PB1){
        LED0 = 0;
    }
}