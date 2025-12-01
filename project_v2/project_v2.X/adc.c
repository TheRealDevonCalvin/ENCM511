#include "adc.h"

volatile uint16_t raw_adc_result;
volatile uint16_t adc_result;
volatile uint16_t led2_intensity;

void ADC1_init(void){

    AD1CON2bits.PVCFG = 0;      // positve voltage reference to AVDD
    AD1CON2bits.NVCFG0 = 0;     // negative voltage reference to AVSS
    
    AD1CHSbits.CH0NA = 0;       // neg ref to Vss for channel a
    AD1CHSbits.CH0SA = 5;       // pos ref to AN5 (pin7) for channel a

    AD1CON3bits.ADCS = 1;
    
    // DMAEN, ASEN are 0

    AD1CON2bits.SMPI = 0;       // interrupt after one sample
    
    AD1CON3bits.ADRC = 0;       // use system clock
    
    AD1CON1bits.SSRC = 2;       // set to use tmr3 trigger
    
    AD1CON3bits.SAMC = 5;       // 5TAD sample time
    AD1CON1bits.FORM = 0;       // absolute decimal result
    AD1CON1bits.MODE12 = 0;     // 10 bit mode
    
    IEC0bits.AD1IE = 1;         // enable interrupts
    IFS0bits.AD1IF = 0;         // clear flag
    
    AD1CON1bits.ASAM = 1;       // auto sample (or can manually set)
        
}

void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void){
    /*
     * The adc interrupt handles the potentiometer-set variable brightness of LED2. 
     * For this application, we are not polling the done bit, rather relying on the ISR. 
     * The adc is set according to the running of T3, so it runs periodically to allow for responsiveness
     */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    raw_adc_result = ADC1BUF0;      // store the result in the adc buffer as the raw result
                                    // necessary because raw result is used to display the adc reading on the screen
    
    adc_result = raw_adc_result;    // set the adc result as the raw to help with clipping

    // here we're clipping the adc result to avoid overflowing or breaking the pwm
    if(adc_result >= 1023) adc_result = 1022;
    if(adc_result <= 0) adc_result = 1;
        
    // transform the adc result into essentially a duty cycle, clipping if necessary
    uint32_t temp = ((uint32_t)adc_result * PWM_PER) / 1023U;
    if(temp > PWM_PER) temp = PWM_PER;
    if(temp <= 1) temp = 1;

    // we protect the variable duty with a semaphore to avoid races when both the adc may be modifying the variable 
    // and the led2 state machine may be reading it
    xSemaphoreTakeFromISR(led2_sem, &xHigherPriorityTaskWoken);
    led2_variable_duty = (uint16_t)temp;
    led2_intensity = led2_variable_duty * 100 / 400;
    xSemaphoreGiveFromISR(led2_sem, &xHigherPriorityTaskWoken);
    
    IFS0bits.AD1IF = 0; // clear the adc interrupt flag
    
    if(xHigherPriorityTaskWoken) {
        portYIELD();  // triggers context switch if a higher priority task is woken
    }
}

/*------------------------------------------------------------------------------------------*/
// these functions not currently used in this application
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
/*------------------------------------------------------------------------------------------*/