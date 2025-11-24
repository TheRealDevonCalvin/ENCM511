/* 
 * File:   adc.h
 * Author: fulbr
 *
 * Created on October 31, 2025, 6:33 PM
 */



#ifndef ADC_H
#define	ADC_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "xc.h"
    
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
    
    #define LED4 LATBbits.LATB15
    
extern QueueHandle_t adcQueue;
    
void ADC1_init(void);
uint16_t do_ADC(void);
uint16_t do_ADC_avg(uint8_t samples);

#ifdef	__cplusplus
}
#endif

#endif	/* ADC_H */

