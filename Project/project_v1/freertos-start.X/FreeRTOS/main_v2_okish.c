/*
 * File:   main.c
 * Author: ENCM 511
 *
 * Created on Oct 22, 2025
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


#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"
#include "semphr.h"
#include <xc.h>


#include "timers.h"
#include "init.h"
#define DEBOUNCE_DELAY pdMS_TO_TICKS(50)
#define NUM_BUTTONS 3

#define LED4 LATBbits.LATB15
#define LED5 LATBbits.LATB14

#define CLICK_DUR pdMS_TO_TICKS(500)
#define LONG_PRESS_DUR pdMS_TO_TICKS(750) // can adjust as needed

#define COMBO_DUR pdMS_TO_TICKS(200)



#define MAX(a, b) ((a) > (b) ? (a) : (b))       // watch this macro yall
#define MIN(a, b) (((a) < (b)) ? (a) : (b))



#define TASK_STACK_SIZE 200
#define TASK_PRIORITY 5

SemaphoreHandle_t uart_sem;

void vApplicationIdleHook( void )
{
}
/*-----------------------------------------------------------*/

uint8_t broked = 0;
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

    broked = 1;
	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
    Disp2String(pcTaskName);
	for( ;; );
}

static int counter = 0;

void vTask1(void *pvParameters)
{
    for(;;)
    {
        xSemaphoreTake(uart_sem, portMAX_DELAY);
        Disp2String("hello from Task 1\n\r");
        xSemaphoreGive(uart_sem);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vTask2(void *pvParameters)
{
    for(;;)
    {

        xSemaphoreTake(uart_sem, portMAX_DELAY);
        Disp2String("hello from Task 2\n\r");
        xSemaphoreGive(uart_sem);
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void vTask3(void *pvParameters)
{
    for(;;)
    {

        _LATB5 ^= 1;
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void prvHardwareSetup(void)
{
    TRISBbits.TRISB5 = 0;
    InitUART2();
    IOinit();
    timerInit();
    
}


volatile uint16_t PWM_TON = 300;
volatile uint16_t PWM_TOFF = 1;       // these may need to be vars not macros to be editable
                                        // note that we can NOT set these to be 0 (janks the timer)
volatile uint16_t PWM_PER = 300;
volatile uint8_t pwm_on = 1;

volatile uint8_t dec = 1;
volatile uint8_t pulse = 1;     // control if the LED should be pulsing in the T3ISR

typedef enum{
    STATE_WAIT,
    STATE_INPUT,
    STATE_COUNT, 
    STATE_END
} state_t;
volatile state_t curr_state = STATE_WAIT;


QueueHandle_t ButtonQueueHandle;



TimerHandle_t DebounceTimers[NUM_BUTTONS];
//TimerHandle_t DebounceTimerHandle;


volatile uint8_t b_before_debounce[NUM_BUTTONS] = {1,1,1};
volatile uint8_t b_after_debounce[NUM_BUTTONS];
volatile uint8_t b_last_valid[NUM_BUTTONS] = {1, 1, 1};
volatile TickType_t b_press_times[NUM_BUTTONS];
volatile TickType_t b_release_times[NUM_BUTTONS];

volatile uint8_t b_copy[NUM_BUTTONS];

volatile uint8_t debounce_active[NUM_BUTTONS] = {0,0,0};
volatile uint8_t debounce_done[NUM_BUTTONS] = {0,0,0};




void v_timer_callback(TimerHandle_t xTimer){
    
    //uint8_t button_id = *(uint8_t*) pvTimerGetTimerID(xTimer);
    uint8_t button_id = (uint8_t)(uintptr_t)pvTimerGetTimerID(xTimer);
   
    
//    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
//        uint8_t b_curr = 0;
//        switch(i){
//            case 0: b_curr = PORTAbits.RA4; break;
//            case 1: b_curr = PORTBbits.RB8; break;
//            case 2: b_curr = PORTBbits.RB9; break;
//        }
//
//        if(b_curr == b_before_debounce[i]){
//            b_after_debounce[i] = b_curr;
//
//            if(!b_curr) {
//                b_press_times[i] = xTaskGetTickCount();
//            } else {
//                b_release_times[i] = xTaskGetTickCount();
//            }
//        }
//    }
//    uint8_t b_curr = 0;
//    switch(button_id){
//        case 0: b_curr = PORTAbits.RA4; break;
//        case 1: b_curr = PORTBbits.RB8; break;
//        case 2: b_curr = PORTBbits.RB9; break;
//            
//    }
//    if(b_curr == b_before_debounce[button_id]){
//        b_after_debounce[button_id] = b_curr;
//        
//        if(b_after_debounce[button_id]){
//            b_release_times[button_id] = xTaskGetTickCount();
//        }
//        else if(!b_after_debounce[button_id]){
//            b_press_times[button_id] = xTaskGetTickCount();
//        }
//    }
    uint8_t raw1 = PORTAbits.RA4;
    uint8_t raw2 = PORTBbits.RB8;
    uint8_t raw3 = PORTBbits.RB9;
    
    
    debounce_done[button_id] = 1;
    uint8_t all_active_done = 1;
    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
        if(debounce_active[i] && !debounce_done[i]){
            all_active_done = 0;
            break;
        }
    }
    
    
    
    if(all_active_done){
    if(raw1 == b_before_debounce[0]){
        b_after_debounce[0] = b_before_debounce[0];
        if(b_after_debounce[0]){ 
            b_release_times[0] = xTaskGetTickCount();
        }
        else if(!b_after_debounce[0]){
            b_press_times[0] = xTaskGetTickCount();
        }
    }
//    else b_after_debounce[0] = 1;
    if(raw2 == b_before_debounce[1]){
        b_after_debounce[1] = b_before_debounce[1];
        if(b_after_debounce[1]){ 
            b_release_times[1] = xTaskGetTickCount();
        }
        else if(!b_after_debounce[1]){
            b_press_times[1] = xTaskGetTickCount();
        }
    }
//    else b_after_debounce[1] = 1;
    if(raw3 == b_before_debounce[2]){
        b_after_debounce[2] = b_before_debounce[2];
        if(b_after_debounce[2]){ 
            b_release_times[2] = xTaskGetTickCount();
        }
        else if(!b_after_debounce[2]){
            b_press_times[2] = xTaskGetTickCount();
        }
    }
//    else b_after_debounce[2] = 1;
    
    
//    uint8_t b_copy[NUM_BUTTONS];
    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
        b_copy[i] = b_after_debounce[i];
        debounce_done[i] = 0;
        debounce_active[i] = 0;
    }
    xQueueSend(ButtonQueueHandle, &b_copy, 0);
    }
    
    // _____________________________________________________________________________________________
//    TickType_t pb1_dur = b_release_times[0] - b_press_times[0];
//    TickType_t pb2_dur = b_release_times[1] - b_press_times[1];
//    TickType_t pb3_dur = b_release_times[2] - b_press_times[2];
//    
//    // ok chat ive decided, in an effort to not kill myself, that we are not doing presses 
//    // we are only doing clicks
//    // holding will occur on the release after the hold (after like a duration but whatever)
//    // i hate it here
//    
//    
////    if(!b_after_debounce[0] && !b_after_debounce[1] && !b_after_debounce[2]
////       && b_last_valid[0] && b_last_valid[1] && b_last_valid[2]){
////        Disp2String("all low\n\r");
////    }
//    if(b_after_debounce[0] && b_after_debounce[1] && b_after_debounce[2]
//       && !b_last_valid[0] && !b_last_valid[1] && !b_last_valid[2]){
//        Disp2String("all low\n\r");
//    }
////    else if(!b_after_debounce[0] && !b_after_debounce[1] 
////            && b_last_valid[0] && b_last_valid[1]){
////        Disp2String("PB1PB2 press\n\r");
////    }
//    else if(b_after_debounce[0] && b_after_debounce[1] 
//            && !b_last_valid[0] && !b_last_valid[1]){
//        Disp2String("PB1PB2 click\n\r");
//        if(pb1_dur >= LONG_PRESS_DUR && pb2_dur >= LONG_PRESS_DUR){
//            Disp2String("Long pb1pb2\n\r");
//        }
//    }
////    else if(!b_after_debounce[0] && !b_after_debounce[2] 
////            && b_last_valid[0] && b_last_valid[2]){
////        Disp2String("PB1PB3 press\n\r");
////    }
//    else if(b_after_debounce[0] && b_after_debounce[2] 
//            && !b_last_valid[0] && !b_last_valid[2]){
//        Disp2String("PB1PB3 click\n\r");
//    }
////    else if(!b_after_debounce[1] && !b_after_debounce[2] 
////            && b_last_valid[1] && b_last_valid[2]){
////        Disp2String("PB2PB3 press\n\r");
////    }
//    else if(b_after_debounce[1] && b_after_debounce[2] 
//            && !b_last_valid[1] && !b_last_valid[2]){
//        Disp2String("PB2PB3 click\n\r");
//    }
//   
//    
//    
//    
//    else if(b_after_debounce[0] && b_after_debounce[1] && b_after_debounce[2]
//            && !b_last_valid[0] && b_last_valid[1] && b_last_valid[2]){
//        Disp2String("PB1 CLICK\n\r");
//        if(pb1_dur >= LONG_PRESS_DUR){
//            Disp2String("long pb1\n\r");
//        }
//    }
//    else if(b_after_debounce[0] && b_after_debounce[1] && b_after_debounce[2]
//            && b_last_valid[0] && !b_last_valid[1] && b_last_valid[2]){
//        Disp2String("PB2 CLICK\n\r");
//    }
//    else if(b_after_debounce[0] && b_after_debounce[1] && b_after_debounce[2]
//            && b_last_valid[0] && b_last_valid[1] && !b_last_valid[2]){
//        Disp2String("PB3 CLICK\n\r");
//    }
//    
//
//    
//    
//    
//    
//    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
//        b_last_valid[i] = b_after_debounce[i];
//    }
    // ______________________________________________________________________________________-
    

}




void v_button_task(void *pvParameters){
    Disp2String("hello in button task\n\r");
 
    uint8_t b_aft[NUM_BUTTONS];
    
    // must debounce still
    // ^^ i think this is decent now?
    
    for(;;){
        //if(xQueueReceive(ButtonQueueHandle, &b_event, portMAX_DELAY) == pdTRUE){
        if(xQueueReceive(ButtonQueueHandle, &b_aft, portMAX_DELAY) == pdTRUE){

            TickType_t pb1_dur = b_release_times[0] - b_press_times[0];
            TickType_t pb2_dur = b_release_times[1] - b_press_times[1];
            TickType_t pb3_dur = b_release_times[2] - b_press_times[2];

            // ok chat ive decided, in an effort to not kill myself, that we are not doing presses 
            // we are only doing clicks
            // holding will occur on the release after the hold (after like a duration but whatever)
            // i hate it here


        //    if(!b_after_debounce[0] && !b_after_debounce[1] && !b_after_debounce[2]
        //       && b_last_valid[0] && b_last_valid[1] && b_last_valid[2]){
        //        Disp2String("all low\n\r");
        //    }
            if(b_aft[0] && b_aft[1] && b_aft[2]
               && !b_last_valid[0] && !b_last_valid[1] && !b_last_valid[2]){
                Disp2String("all low\n\r");
            }
        //    else if(!b_after_debounce[0] && !b_after_debounce[1] 
        //            && b_last_valid[0] && b_last_valid[1]){
        //        Disp2String("PB1PB2 press\n\r");
        //    }
            else if(b_aft[0] && b_aft[1] 
                    && !b_last_valid[0] && !b_last_valid[1]
                    && (b_aft[2] == b_last_valid[2]) // new
                    ){
                Disp2String("PB1PB2 click\n\r");
                if(pb1_dur >= LONG_PRESS_DUR && pb2_dur >= LONG_PRESS_DUR){
                    Disp2String("Long pb1pb2\n\r");
                }
            }
        //    else if(!b_after_debounce[0] && !b_after_debounce[2] 
        //            && b_last_valid[0] && b_last_valid[2]){
        //        Disp2String("PB1PB3 press\n\r");
        //    }
            else if(b_aft[0] && b_aft[2] 
                    && !b_last_valid[0] && !b_last_valid[2]
                    && (b_aft[1] == b_last_valid[1])    // new
                    ){
                Disp2String("PB1PB3 click\n\r");
            }
        //    else if(!b_after_debounce[1] && !b_after_debounce[2] 
        //            && b_last_valid[1] && b_last_valid[2]){
        //        Disp2String("PB2PB3 press\n\r");
        //    }
            else if(b_aft[1] && b_aft[2] 
                    && !b_last_valid[1] && !b_last_valid[2]
                    && (b_aft[0] == b_last_valid[0])    // new
                    ){
                Disp2String("PB2PB3 click\n\r");
                if(pb2_dur >= LONG_PRESS_DUR && pb3_dur >= LONG_PRESS_DUR){
                    Disp2String("Long pb2pb3\n\r");
                }
            }




            else if(b_aft[0] /*&& b_aft[1] && b_aft[2]*/
                    && !b_last_valid[0] /*&& b_last_valid[1] && b_last_valid[2]*/
                    && (b_aft[1] == b_last_valid[1]) && (b_aft[2] == b_last_valid[2])
                    ){
                Disp2String("PB1 CLICK\n\r");
                if(pb1_dur >= LONG_PRESS_DUR){
                    Disp2String("long pb1\n\r");
                }
            }
            else if(/*b_aft[0] &&*/ b_aft[1] /*&& b_aft[2]*/
                    && /*b_last_valid[0] &&*/ !b_last_valid[1] /*&& b_last_valid[2]*/
                    && (b_aft[0] == b_last_valid[0]) && (b_aft[2] == b_last_valid[2])
                    ){
                Disp2String("PB2 CLICK\n\r");
            }
            else if(/*b_aft[0] && b_aft[1] &&*/ b_aft[2]
                    && /*b_last_valid[0] && b_last_valid[1] &&*/ !b_last_valid[2]
                    && (b_aft[0] == b_last_valid[0]) && (b_aft[1] == b_last_valid[1])
                    ){
                Disp2String("PB3 CLICK\n\r");
                if(pb3_dur >= LONG_PRESS_DUR){
                    Disp2String("long pb3\n\r");
                }
            }






            for(uint8_t i = 0; i < NUM_BUTTONS; i++){
                b_last_valid[i] = b_aft[i];
            }
            
        }
 
        
    }
    
}

void v_main_state_task(void *pvParameters){

    Disp2String("hello in main\n\r");
    for(;;){
        // state logic?
        
        switch(curr_state){
            case STATE_WAIT:
                // handle waiting behaviour
                // this is LED pulsing
                // click of PB1 to do the transition
                //Disp2String("in wait state\n\r");
                pulse = 1;
//                T3CONbits.TON = 1;
                
                break;
                
            case STATE_INPUT:
                // get user input for the mins/secs 
                // 
                // long press of PB2/PB3 resets input time
                // click of PB2/PB3 proceeds state to begin counting down
                pulse = 0;
                LED4 = 1;
                //Disp2String("STATE INPUT!!");
                
                break;
                
            case STATE_COUNT:
                // led1 blinks
                // update time remaining display as we count
                // led2 blinks with variable display given the potentiometer (ADC) input
                // 
                // click PB3 pause countdown and blinking
                    // re-click, restarts countdown and blinking
                // long press PB3 abort timer, reset to wait state
                
                // if 'i' entered on UART, toggle display for just count, or count, ADC, intensity
                // if 'b' entered on UART, toggle LED2 blinking or solid
                
                // exit upon timer elapse
                
                break;
                
            case STATE_END:
                // display end message (countdown done)
                // LED0/LED1 rapidly blink alternating
                // LED2 solidly on (still pot for brightness)
                
                // after 5s elapsed, go back to wait
                
                break;
            
        }   // end switch(curr_state)
    }
}




int main(void) {
    
    prvHardwareSetup();
    
    ANSELA = 0x0000; /* keep this line as it sets I/O pins that can also be analog to be digital */
    ANSELB = 0x0000; /* keep this line as it sets I/O pins that can also be analog to be digital */
    
    //ANSELB |= 0b1000; // set RB3 (pin 7) to be analog input 
    
    ANSELBbits.ANSB9 = 0;
    TRISBbits.TRISB9 = 1;
    TRISBbits.TRISB15 = 0;
    TRISBbits.TRISB14 = 0;
    LED4 = 0;
    LED5 = 0;

//	xTaskCreate( vTask1, "Task1", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
//    xTaskCreate( vTask2, "Task2", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
//    xTaskCreate( vTask3, "Task3", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
//    uart_sem = xSemaphoreCreateMutex();
    
    ButtonQueueHandle = xQueueCreate(10, sizeof(uint8_t) * NUM_BUTTONS);
    
    
    
    xTaskCreate(v_button_task, "Button Task", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);
    xTaskCreate(v_main_state_task, "Main State", configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL);
    
    
    
    static uint8_t timer_ids[NUM_BUTTONS];
    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
        timer_ids[i] = i;
        //DebounceTimers[i] = xTimerCreate("Debounce Timer", DEBOUNCE_DELAY, pdFALSE, (void *)&timer_ids[i], v_timer_callback);
        DebounceTimers[i] = xTimerCreate("Debounce Timer", DEBOUNCE_DELAY, pdFALSE, (void *)(uintptr_t)i, v_timer_callback);
    }
    //DebounceTimerHandle = xTimerCreate("Debounce Timer", DEBOUNCE_DELAY, pdFALSE, (void *) 0, v_timer_callback);

    
//    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
//        Disp2Dec(i);
//        Disp2String("\n\r");
//        if(DebounceTimers[i] == NULL){
//            Disp2String("Timer Create Failed for button ");
//            Disp2Dec(i);
//            Disp2String("\n\r");
//        }
//    }
   
    //xTimerStart(DebounceTimers[2], 0); // what the hell bro why did this fix it

    

    
    TMR3 = 0;
    PR3 = PWM_TON;
    T3CONbits.TON = 1;

    vTaskStartScheduler();
    
    for(;;);
}



void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void){
    /*
     * used to control the wait period of the adc so we only get a new sample (only enter state sample)
     * every 0.25s. this helps with stability
     */
    //LED0 ^= 1;
    TMR3 = 0;               // reset the timer start count
    IFS0bits.T3IF = 0;      // clear the interrupt flag
    if(pulse){
        uint8_t step_size = 3;
        if(pwm_on){
            LED2 = 0;
            PWM_TOFF = (PWM_PER - PWM_TON == 0) ? 1 : PWM_PER - PWM_TON;
            PR3 = PWM_TOFF;
            pwm_on = 0;
        }
        else if(!pwm_on){
            LED2 = 1;
            if(dec){
                PWM_TON -= step_size;
                if(PWM_TON <= step_size){
                    dec = 0;
                }
            }
            if(!dec){
                PWM_TON += step_size;
                if(PWM_TON >= PWM_PER - step_size){
                    dec = 1;
                }
            }
            PR3 = PWM_TON;
            pwm_on = 1;
        }
    }
    
}



/* 
 * ok team its looking like we're gonna do some nonsense up in here, hold on to your hats
 * we have three buttons (pb1, pb2, pb3), and they all need to debounce (unfortunately, debounce is the death of me)
 * we have the enum type (button_event_t) that tracks press and release
 * we prolly also wanna track pending events, and mark the buttons as pending in the isr (and if press/release)
 * and start a debounce timer (i think lets use a software timer???) and we clear the IOC flags
 * when the timer triggers, we enter a callback (i think?) and check if there are events pending
 * if yes, we send them to the queue and clear the pending flags
 * this sends the queue to the button task, where we handle the presses however
 * should be the timer itself that actually performs the debounce??? just the wait to stabilize?
 * 
 * ok so like tracking pending events was not the play i dont think. it was a lot of work, and i dont think helpful
 * 
 * 
 * lowkey that did not work in the end. we broke it too many times
 * we have reverted back to the lab3 style button logic
 * 
 * */

void __attribute__ ((interrupt, no_auto_psv)) _IOCInterrupt(void){
//    static button_event_t pending_events[NUM_BUTTONS];
//    static uint8_t event_pending[NUM_BUTTONS] = {0, 0, 0};
   
    //debounce_prog[2] = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //button_event_t b_event;
//    LED0 ^=1;
    
//    LED0 ^= 1;


   
    // consider if should use timer start or timer reset
    // PB1
    if(IOCFAbits.IOCFA4){
        IOCFAbits.IOCFA4 = 0;   // clear flag on RA4 pin
        //taskENTER_CRITICAL();
        b_before_debounce[0] = PORTAbits.RA4;
        
        
        if(b_before_debounce[0] != b_last_valid[0]){
            debounce_active[0] = 1;
            debounce_done[0] = 0;
            xTimerResetFromISR(DebounceTimers[0], &xHigherPriorityTaskWoken);
        }
        //taskEXIT_CRITICAL();
        //xTimerStartFromISR(DebounceTimerHandle, &xHigherPriorityTaskWoken);
        
        
    }
    
    // for PB2
    if(IOCFBbits.IOCFB8){
        IOCFBbits.IOCFB8 = 0;   // clear flag on RB8 pin
        
        //taskENTER_CRITICAL();
        b_before_debounce[1] = PORTBbits.RB8;
        
        if(b_before_debounce[1] != b_last_valid[1]){
            debounce_active[1] = 1;
            debounce_done[1] = 0;
            
            xTimerResetFromISR(DebounceTimers[1], &xHigherPriorityTaskWoken);
        }
        //taskEXIT_CRITICAL();
        //xTimerStartFromISR(DebounceTimerHandle, &xHigherPriorityTaskWoken);
        
        
    }
    
    // for PB3
    if(IOCFBbits.IOCFB9){  
        IOCFBbits.IOCFB9 = 0;   // clear flag on RB9 pin
        
        //taskENTER_CRITICAL();
        b_before_debounce[2] = PORTBbits.RB9;

        if(b_before_debounce[2] != b_last_valid[2]){
            debounce_active[2] = 1;
            debounce_done[2] = 0;
            
            xTimerResetFromISR(DebounceTimers[2], &xHigherPriorityTaskWoken);
        }
        //taskEXIT_CRITICAL();
        //xTimerStartFromISR(DebounceTimerHandle, &xHigherPriorityTaskWoken);
        
        
    }
   
    
    //xTimerStartFromISR(DebounceTimerHandle, &xHigherPriorityTaskWoken);     // should be safe?
    IFS1bits.IOCIF = 0;  // clear global IOC flags 
    if(xHigherPriorityTaskWoken) {
        portYIELD();  // triggers context switch
    }
    
//    xQueueSendFromISR(ButtonQueueHandle, &b_event, &xHigherPriorityTaskWoken);
     
    
}

