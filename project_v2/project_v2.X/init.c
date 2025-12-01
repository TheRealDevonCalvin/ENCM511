#include "xc.h"
#include "init.h"


void IOinit(void){
    /*
     * This function initializes the IO pins used for the application. 
     * It also configures the IOC interrupts for the pushbuttons
     */
    // configure LED pins as outputs
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB7 = 0;
    
    // default the LEDs off
    LATBbits.LATB5 = 0;
    LATBbits.LATB6 = 0;
    LATBbits.LATB7 = 0;

    // change all pins that can be analog to be digital for the IOs
    // we set the ADC pin to analog elsewhere
    ANSELA = 0x0000; 
    ANSELB = 0x0000; 

    // set the button IOs as inputs
    TRISAbits.TRISA4 = 1;   // PB1 (RA4)
    TRISBbits.TRISB8 = 1;   // PB2 (RB8)
    TRISBbits.TRISB9 = 1;   // PB3 (RB9)

    // pull-ups on each button pin so the active low works
    IOCPUAbits.CNPUA4 = 1;
    IOCPUBbits.CNPUB8 = 1;
    IOCPUBbits.CNPUB9 = 1;
    
    // clear flag bits on each pin
    IOCFAbits.IOCFA4 = 0;   // RA4
    IOCFBbits.IOCFB8 = 0;   // RB8
    IOCFBbits.IOCFB9 = 0;   // RB9

    // clear port flag bits
    IOCSTATbits.IOCPAF = 0;
    IOCSTATbits.IOCPBF = 0;

    // clear global IOC flag
    IFS1bits.IOCIF = 0;

    // read ports once to latch
    // -----------------------------------------------------------------
    // this is what was causing the odd pb3 behaviour on startup. a microchip reference manual
    //    https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/ReferenceManuals/70005186a.pdf
    // says that sometimes odd behaviour will occur if the port is not read before IOC is enabled
    // so this is to help fix that and cause latching for the IOC change comparison
    // -----------------------------------------------------------------
    volatile uint16_t dummyA = PORTA;
    volatile uint16_t dummyB = PORTB;

    // configure edges for the interrupts
    IOCNAbits.IOCNA4 = 1;   // negative edge RA4
    IOCPAbits.IOCPA4 = 1;   // positive edge RA4

    IOCNBbits.IOCNB8 = 1;   // negative edge RB8
    IOCPBbits.IOCPB8 = 1;   // positive edge RB8

    IOCNBbits.IOCNB9 = 1;   // negative edge RB9
    IOCPBbits.IOCPB9 = 1;   // positive edge RB9

    // enable IOC
    PADCONbits.IOCON = 1;   // enable IOC 
    IPC4bits.IOCIP = 3;     // set priority
    IEC1bits.IOCIE = 1;     // enable global IOC interrupt

    // clear flags again for safety
    IOCFAbits.IOCFA4 = 0;
    IOCFBbits.IOCFB8 = 0;
    IOCFBbits.IOCFB9 = 0;
    IFS1bits.IOCIF = 0;
}

void timerInit(){
    /*
     * This function initializes the hardware timers used in the application.
     * We use timers 2 and 3, and freeRTOS co-opts timer 1 for its use 
     */
 
    T2CONbits.T32 = 0;          // operate timer 2/3 as 16 bit timers
    
    // T2CON config
    // T2 is used for the LED2 state machine
    T2CONbits.TCKPS = 3;        // 1:256 prescaler
    T2CONbits.TCS = 0;          // clock source: internal clock
    T2CONbits.TSIDL = 0;        // continue operation in idle mode
    IPC1bits.T2IP = 5;          // priority set
    IFS0bits.T2IF = 0;          // clear flag
    IEC0bits.T2IE = 1;          // enable timer interrupt

    // T3CON config
    // T3 is used to trigger the ADC to convert
    T3CONbits.TCKPS = 3;        // set prescaler to 1:256
    T3CONbits.TCS = 0;          // use internal clock
    T3CONbits.TSIDL = 0;        // operate in idle mode
    IPC2bits.T3IP = 2;          // set priority
    IFS0bits.T3IF = 0;          // clear flag
    IEC0bits.T3IE = 0;          // disable timer interrupt (for adc)
                                // ^ done because don't need an ISR, adc converts automatically
}


