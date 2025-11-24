#include "xc.h"
#include "init.h"

// this file has not changed from previous lab 3

void IOinit(){
    /*
     * This function initializes the IOs used in the lab
     * This includes the LEDs and buttons. 
     */
    
    // set pins for LEDs as outputs
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB7 = 0;
    
    // set LEDs to off
    LED0 = 0;
    LED1 = 0;
    LED2 = 0;
    
    
    // button inits! (includes the IOC setups)

    // these are global IOC interrupt controllers
    PADCONbits.IOCON = 1;       // enables interrupt on change
    IFS1bits.IOCIF = 0;         // clear IOC interrupt flag
    IPC4bits.IOCIP = 3;         // set IOC priority to 3
    IEC1bits.IOCIE = 1;         // enable the interrupt
    
    
    // RB8 is button1 (the top one)
    TRISBbits.TRISB8 = 1;       // set RB8 as input
    IOCPUBbits.CNPUB8 = 1;      // pullup RB8 (so button to ground works)
    
    IOCNBbits.IOCNB8 = 1;       // enable interrupt on negative edge
    IOCPBbits.IOCPB8 = 1;       // enable interrupt positive edge
    IOCSTATbits.IOCPBF = 0;     // clear flags in IOC status register for port B
    IOCFBbits.IOCFB8 = 0;         // clear RB8 flag
    
    
    // RA4 is button0 (middle)
    TRISAbits.TRISA4 = 1;       // set RA4 as input
    IOCPUAbits.CNPUA4 = 1;      // pullup on RA4 (again, so button to ground works)
    
    IOCNAbits.IOCNA4 = 1;       // enable interrupt negative edge transition 
    IOCPAbits.IOCPA4 = 1;       // enable interrupt positive edge
    IOCSTATbits.IOCPAF = 0;     // clear port A status flags for IOC
    
    
    // RB9 is button 2 (bottom)
    TRISBbits.TRISB9 = 1;       // set RB9 as input
    IOCPUBbits.CNPUB9 = 1;      // pullup RB9
    
    IOCNBbits.IOCNB9 = 1;       // interrupt on negative edge
    IOCPBbits.IOCPB9 = 1;
    IOCSTATbits.IOCPBF = 0;     // clear flags (again?)
    IOCFBbits.IOCFB9 = 0;         // clear RB9 flag
}

void timerInit(){
    /*
     * This function initializes all of the timers used in the lab
     * We use timers 1, 2, and 3. 
     */
    
    // commented out cause co-opted by the freertos
//    // T1CON config bits
//    // T1 is used for the delay function 
//    T1CONbits.TCKPS = 3;        // prescaler 1:256
//    T1CONbits.TCS = 0;          // use internal clock as source
//    T1CONbits.TSIDL = 0;        // keep running in idle mode
//    IPC0bits.T1IP = 2;          // set timer interrupt priority 
//    IFS0bits.T1IF = 0;          // clear flag
//    IEC0bits.T1IE = 1;          // enable timer interrupt
     
    
    T2CONbits.T32 = 0;          // operate timer 2/3 as 16 bit timer
    
    // T2CON config
    // T2 is used for the debouncing
//    T2CONbits.TCKPS = 3;        // 1:256 prescaler
    T2CONbits.TCKPS = 3;        // 1:1 prescaler
    T2CONbits.TCS = 0;          // clock source: internal clock
    T2CONbits.TSIDL = 0;        // continue operation in idle mode
    IPC1bits.T2IP = 5;          // priority set
    IFS0bits.T2IF = 0;          // clear flag
    IEC0bits.T2IE = 1;          // disable timer interrupt

    
    // T3CON config
    // T3 is used to control the blinking of the LED
    T3CONbits.TCKPS = 3;        // set prescaler to 1:256
    T3CONbits.TCS = 0;          // use internal clock
    T3CONbits.TSIDL = 0;        // operate in idle mode
    IPC2bits.T3IP = 2;          // set priority
    IFS0bits.T3IF = 0;          // clear flag
//    IEC0bits.T3IE = 1;          //enable timer interrupt
    IEC0bits.T3IE = 0;          // disable timer interrupt (for adc)
}


//void delay_ms(uint16_t ms){
//    /*
//     * This delay function is only valid for times below ~4190ms
//     * This is due to the PR1 register being defined as a uint16_t,
//     * meaning it has a valid range up to 2^16 - 1 (65535) or it overflows
//     * 
//     */
//    
//    // save the current values of the IEC register and clear it
//    // helps ensure the delay function doesn't get interrupted
//    uint16_t saved_IEC0 = IEC0;
//    IEC0 = 0;
//    
//    // calculate the ticks for the timer counter
//   
//    if(ms > 4190) {
//        // if the PR1 count is assigned a bigger number than it can, assign it to its max
//        PR1 = 65535;
//    }
//    else PR1 = (ms / 0.064);
//    TMR1 = 0;               // initialize the counter start to 0
//    
//    IFS0bits.T1IF = 0;      // clear the flag
//    IEC0bits.T1IE = 1;      // enable the interrupt
//    
//    T1CONbits.TON = 1;      // turn on the timer
//    Idle();                 // put CPU into idle to create the delay
//    
//    IEC0 = saved_IEC0;      // restore the previous IEC register (return to normal operation)
//    IEC0bits.T1IE = 0;      // disable the interrupt for timer 1 while delay is not used
//        
//}
