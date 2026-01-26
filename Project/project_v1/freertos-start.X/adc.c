#include "adc.h"
#include "xc.h"


void ADC1_init(void){

    AD1CON2bits.PVCFG = 0;      // positve voltage reference to AVDD
    AD1CON2bits.NVCFG0 = 0;     // negative voltage reference to AVSS
    
    AD1CHSbits.CH0NA = 0;       // neg ref to Vss for channel a
    AD1CHSbits.CH0SA = 5;       // pos ref to AN5 (pin7) for channel a
    
//    AD1CON3bits.ADCS = 0xFF;    // set to 256*Tcy
//    AD1CON3bits.ADCS = 15;      // ?????
    AD1CON3bits.ADCS = 1;
    
    // DMAEN, ASEN are 0
    
//    AD1CON2bits.SMPI = 3;
    AD1CON2bits.SMPI = 0;
    
    AD1CON3bits.ADRC = 0;       // use system clock
    
//    AD1CON1bits.SSRC = 7;       // set to auto-convert
    AD1CON1bits.SSRC = 2;       // set to use tmr3 trigger
    
//    AD1CON3bits.SAMC = 0b11111; // set to auto sample 31 Tad
    AD1CON3bits.SAMC = 5;       // 5TAD sample time
    AD1CON1bits.FORM = 0;       // absolute decimal result
    AD1CON1bits.MODE12 = 0;     // 10 bit mode
    
    IEC0bits.AD1IE = 1;
    IFS0bits.AD1IF = 0;
    
    AD1CON1bits.ASAM = 1;       // auto sample (or can manually set)
        
}

uint16_t do_ADC(void){
    uint16_t result = 0;    // variable for result
    while(!(AD1CON1bits.DONE)){}    // waiting for the conversion to be done
    result = ADC1BUF0;      // get the result
    return result;          // return the result
}


uint16_t do_ADC_avg(uint8_t samples){
    uint32_t sum = 0;   // track the sum. this needs to be uint32 cause the sum may overflow
    for(uint8_t i = 0; i < samples; i++){
        // loop through the number of samples to track
        sum += do_ADC();    // add to the sum
    }
    AD1CON1bits.ADON = 0;   // turn off the ADC when done sampling
    return (uint16_t)(sum / samples);   // divide to get the average value, and cast back to a uint16
}


//void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void){
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    uint32_t sum = 0;
//    LED4 ^= 1;
//    
//    sum += ADC1BUF0;
//    sum += ADC1BUF1;
//    sum += ADC1BUF2;
//    sum += ADC1BUF3;
//    
//    uint16_t avg = (uint16_t)(sum >> 2);    // div with right shift cause faster than floating point
//    
//
////    uint32_t temp = (uint32_t)avg * (uint32_t)PWM_PER;
//    uint32_t temp = (uint32_t)avg * (uint32_t)400;
//    temp = temp / 1023U;
////    if(temp > PWM_PER) temp = PWM_PER;
//    if(temp > 400) temp = 400;
//    if(led2_mode == LED_MODE_ON || led2_mode == LED_MODE_BLINK){
//        led2_variable_duty = (uint16_t)temp;
//    }
//
//    
//    //xQueueSendFromISR(adcQueue, &avg, &xHigherPriorityTaskWoken);
//    
//    IFS0bits.AD1IF = 0;
//    
//    if(xHigherPriorityTaskWoken) {
//        portYIELD();  // triggers context switch
//    }
//}