#include "led_module.h"

// variables for the pwm implementation
volatile uint16_t PWM_TON = 400;
volatile uint16_t PWM_PER = 400;
volatile uint8_t pwm_on = 1;

volatile uint8_t decrease = 1;    

// timer and semaphore handles for the LEDs
TimerHandle_t led_blink_timer;
TimerHandle_t led_alternate_timer;

SemaphoreHandle_t led2_sem;

// variables to help with the LED2 state machine
volatile led_mode_t led2_mode = LED_MODE_OFF;
volatile uint8_t led2_blink_phase = 0;
volatile uint8_t led1_blink_en = 0;

volatile uint16_t led2_variable_duty = 1;     
volatile uint8_t led2_output_en = 0;


void v_blink_callback_count(TimerHandle_t xTimer){
    // this timer callback is responsible for creating the 1s blink rates of LEDs 1 and 2
    led2_blink_phase ^= 1;  // toggles blinking phase of LED2. this is used in the LED2 state machine
    if(led1_blink_en){
        // if LED1 is enabled to blink, toggles the pin at 1s intervals
        LED1 ^= 1;
    }
}

void v_blink_callback_alternating(TimerHandle_t xTimer){
    // this timer callback is responsible for LEDs 0 and 1 blinking rapidly in an alternating fashion
    // each LED state is toggled, but are alternating because upon starting the timer, they are in opposite states
    LED0 ^= 1;
    LED1 ^= 1;
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void){
    /*
     * the T2 interrupt handles the small state machine on LED2. This LED must be able to pulse, blink, 
     * and be steady on/off, as well as maintain a variable intensity as given by the ADC potentiometer.
     * Because of the multiple modes, it makes sense to implement the LED as a state machine.
     * 
     * We use a hardware timer for this to ensure responsiveness to the PWM changes. 
     */
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    TMR2 = 0;               // reset the timer start count
    IFS0bits.T2IF = 0;      // clear the interrupt flag
    uint16_t duty;          // an intermediate/internal variable to hold the current duty cycle
    
    switch(led2_mode){
        // switch on the mode of the LED2 state machine
        case LED_MODE_OFF:
            // if the led should be off, disable output and set the duty to 0
            led2_output_en = 0;
            duty = 0;
            break;
        case LED_MODE_ON:
            // if the led should be constantly on, enable the output and adjust the variable duty according to the ADC reading
            led2_output_en = 1;
            xSemaphoreTakeFromISR(led2_sem, &xHigherPriorityTaskWoken); // semaphore protected bc duty gets modified in the adc isr also
            duty = led2_variable_duty;
            xSemaphoreGiveFromISR(led2_sem, &xHigherPriorityTaskWoken);
            break;
        case LED_MODE_BLINK:
            // if the led should be blinking, set the variably duty according to the adc
            xSemaphoreTakeFromISR(led2_sem, &xHigherPriorityTaskWoken);
            duty = led2_variable_duty;
            xSemaphoreGiveFromISR(led2_sem, &xHigherPriorityTaskWoken);
            led2_output_en = led2_blink_phase;  // and enable the output based on the phase of the blinking        
            break;
        case LED_MODE_PULSE:
            // if the led should be pulsing (as in the wait state of the fsm)
            led2_output_en = 1; // enable output
            if(decrease){
                // if the decreasing flag is on, we decrease the pwm on time by pulse_step_size to decrease duty
                PWM_TON -= PULSE_STEP_SIZE;
                if(PWM_TON <= PULSE_STEP_SIZE){
                    // now if the next step will wrap the variable, change the decreasing flag to increasing
                    decrease = 0;
                }
            }
            if(!decrease){
                // if the decreasing flag is off, increase the pwm on time by pulse_step_size to increase duty
                PWM_TON += PULSE_STEP_SIZE;
                if(PWM_TON >= PWM_PER - PULSE_STEP_SIZE){
                    // if the next step will go past the period of the pwm, swap to be increasing
                    decrease = 1;
                }
            }
            duty = PWM_TON; // assign the on time of the pwm to the duty for later use
            break;        
    }// end switch(led2_mode)

    if(led2_output_en){
        // if the output is enabled, create the pwm based on the earlier assigned duty
        if(pwm_on){
            // if the pwm_on is high, do one phase of the pwm
            LED2 = 0;               // turn led2 off
            PR2 = PWM_PER - duty;   // set the timer count to the period-duty
            pwm_on = 0;             // switch the flag
        }
        else{
            LED2 = 1;               // turn led2 on
            PR2 = duty;             // set timer count to the duty cycle 
            pwm_on = 1;             // switch the flag
        }
    }
    else{
        // if the led2 output is not enabled, force LED2 off
        LED2 = 0;
    }
    
    if(xHigherPriorityTaskWoken) {
        portYIELD();  // triggers context switch if a higher priority task is woken
    }
}