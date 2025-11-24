/* 
 * File:   init.h
 * Author: fulbr
 *
 * Created on October 14, 2025, 4:39 PM
 */

#ifndef INIT_H
#define	INIT_H


#define LED0    LATBbits.LATB5
#define LED1    LATBbits.LATB6
#define LED2    LATBbits.LATB7

#define PB1     PORTAbits.RA4
#define PB2     PORTBbits.RB8
#define PB3     PORTBbits.RB9   

#ifdef	__cplusplus
extern "C" {
#endif

void IOinit();
void timerInit();
void delay_ms(uint16_t ms);

#ifdef	__cplusplus
}
#endif

#endif	/* INIT_H */



