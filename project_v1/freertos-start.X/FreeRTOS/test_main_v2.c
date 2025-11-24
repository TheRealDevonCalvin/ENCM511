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
#define DEBOUNCE_DELAY pdMS_TO_TICKS(20)
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

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
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

typedef enum{
    PB1_PRESS,
    PB1_RELEASE,
    PB2_PRESS,
    PB2_RELEASE,
    PB3_PRESS,
    PB3_RELEASE
} button_event_t;
QueueHandle_t ButtonQueueHandle;

TickType_t PB1_press_time;
TickType_t PB1_release_time;
TickType_t PB2_press_time;
TickType_t PB2_release_time;
TickType_t PB3_press_time;
TickType_t PB3_release_time;

TimerHandle_t DebounceTimers[NUM_BUTTONS];

volatile button_event_t pending_events[NUM_BUTTONS];
volatile uint8_t event_pending[NUM_BUTTONS] = {0, 0, 0};
volatile uint8_t debounce_prog[NUM_BUTTONS] = {0, 0, 0};
volatile uint8_t b_last_valid[NUM_BUTTONS] = {1, 1, 1};

void v_timer_callback(TimerHandle_t xTimer){
    
    uint8_t button_id = *(uint8_t*) pvTimerGetTimerID(xTimer);
    if(button_id == 2){
        LED5 ^= 1;
    }
//    if(event_pending[button_id]){
//        xQueueSend(ButtonQueueHandle, &pending_events[button_id], 0);
//        event_pending[button_id] = 0;
//    }
//    
//    debounce_prog[button_id] = 0;
    

    uint8_t b_curr_state;
    switch(button_id){
        case 0: b_curr_state = PORTAbits.RA4; break;
        case 1: b_curr_state = PORTBbits.RB8; break;
        case 2: b_curr_state = PORTBbits.RB9; break;
    }
    
    if(b_curr_state == pending_events[button_id] && b_curr_state != b_last_valid[button_id]){
        button_event_t b_event;

        switch(button_id){
            case 0: b_event = b_curr_state ? PB1_RELEASE : PB1_PRESS; break;
            case 1: b_event = b_curr_state ? PB2_RELEASE : PB2_PRESS; break;
            case 2: b_event = b_curr_state ? PB3_RELEASE : PB3_PRESS; break;
        }
        //debounce_prog[button_id] = 0;
        xQueueSend(ButtonQueueHandle, &b_event, 0);
        b_last_valid[button_id] = b_curr_state;
    }
    debounce_prog[button_id] = 0;
}


typedef struct{
    TickType_t press_time;
    TickType_t release_time;
    uint8_t is_pressed;
} button_state_t;
button_state_t PB1_s, PB2_s, PB3_s;


uint8_t PB1PB2_reported = 0;
uint8_t PB1PB2_long_reported = 0;
uint8_t PB1PB3_reported = 0;
uint8_t PB1PB3_long_reported = 0;
uint8_t PB2PB3_reported = 0;
uint8_t PB2PB3_long_reported = 0;
uint8_t PB1PB2PB3_reported = 0;


void check_combos(void){
    LED5 = 1;
    TickType_t now = xTaskGetTickCount();
    
    
    // all three
    // note that this DOES NOT fully work right now!!!
    TickType_t overlap_start_3 = MAX(PB1_s.press_time, MAX(PB2_s.press_time, PB3_s.press_time));
    TickType_t overlap_end_3 = MIN(
        PB1_s.is_pressed ? now : PB1_s.release_time,
        MIN(PB2_s.is_pressed ? now : PB2_s.release_time,
            PB3_s.is_pressed ? now : PB3_s.release_time)
    );

    if(overlap_end_3 > overlap_start_3 && (overlap_end_3 - overlap_start_3) >= COMBO_DUR){
        if(!PB1PB2PB3_reported){
            Disp2String("PB1PB2PB3 combo\n\r");
            PB1PB2PB3_reported = 1;
        }
        // pass??
        PB1PB2_reported = PB1PB3_reported = PB2PB3_reported = 0;
        return;
    } else {
        PB1PB2PB3_reported = 0;
    }
    
    // pb1 pb2 combo
    TickType_t overlap_start = MAX(PB1_s.press_time, PB2_s.press_time);
    TickType_t overlap_end = MIN(PB1_s.is_pressed ? now : PB1_s.release_time, 
                                     PB2_s.is_pressed ? now : PB2_s.release_time);
    if(overlap_end > overlap_start){
        TickType_t overlap_dur = overlap_end - overlap_start;
        if(overlap_dur >= COMBO_DUR && overlap_dur < LONG_PRESS_DUR){
            
            if(!PB1PB2_reported){
                Disp2String("PB1PB2 combo\n\r");
                PB1PB2_reported = 1;
            }
        }
        else if(overlap_dur >= LONG_PRESS_DUR){
            if(!PB1PB2_long_reported){
                Disp2String("PB1PB2 LONG press\n\r");
                PB1PB2_long_reported = 1;
            }
        }
    }
    else{
        PB1PB2_reported = 0;
        PB1PB2_long_reported = 0;
    }
    
    // pb1 pb3 combo
    overlap_start = MAX(PB1_s.press_time, PB3_s.press_time);
    overlap_end = MIN(PB1_s.is_pressed ? now : PB1_s.release_time, 
                                     PB3_s.is_pressed ? now : PB3_s.release_time);
    if(overlap_end > overlap_start){
        TickType_t overlap_dur = overlap_end - overlap_start;
        if(overlap_dur >= COMBO_DUR && overlap_dur < LONG_PRESS_DUR){
            
            if(!PB1PB3_reported){
                Disp2String("PB1PB3 combo\n\r");
                PB1PB3_reported = 1;
            }
        }
        else if(overlap_dur >= LONG_PRESS_DUR){
            if(!PB1PB3_long_reported){
                Disp2String("PB1PB3 LONG press\n\r");
                PB1PB3_long_reported = 1;
            }
        }
    }
    else{
        PB1PB3_reported = 0;
        PB1PB3_long_reported = 0;
    }
    
    // pb2 pb3 combo
    overlap_start = MAX(PB2_s.press_time, PB3_s.press_time);
    overlap_end = MIN(PB2_s.is_pressed ? now : PB2_s.release_time, 
                                     PB3_s.is_pressed ? now : PB3_s.release_time);
    if(overlap_end > overlap_start){
        TickType_t overlap_dur = overlap_end - overlap_start;
        if(overlap_dur >= COMBO_DUR && overlap_dur < LONG_PRESS_DUR){
            
            if(!PB2PB3_reported){
                Disp2String("PB2PB3 combo\n\r");
                PB2PB3_reported = 1;
            }
        }
        else if(overlap_dur >= LONG_PRESS_DUR){
            if(!PB2PB3_long_reported){
                Disp2String("PB2PB3 LONG press\n\r");
                PB2PB3_long_reported = 1;
            }
        }
    }
    else{
        PB2PB3_reported = 0;
        PB2PB3_long_reported = 0;
    }

}

void v_button_task(void *pvParameters){
    Disp2String("hello in button task\n\r");
    
    button_event_t b_event;
    
    // must debounce still
    // ^^ i think this is decent now?
    
    for(;;){
        if(xQueueReceive(ButtonQueueHandle, &b_event, portMAX_DELAY) == pdTRUE){
            
            
            if(b_event == PB1_PRESS){
                Disp2String("button1 press\n\r");
                //TickType_t PB1_press_time;
                //PB1_press_time = xTaskGetTickCount();
                PB1_s.press_time = xTaskGetTickCount();
                PB1_s.is_pressed = 1;
                
            }
            else if(b_event == PB1_RELEASE){
                Disp2String("button1 RELEASE\n\r");
                //PB1_release_time = xTaskGetTickCount();
                PB1_s.release_time = xTaskGetTickCount();
                PB1_s.is_pressed = 0;
                
                TickType_t PB1_dur = PB1_s.release_time - PB1_s.press_time;
                if(PB1_dur < LONG_PRESS_DUR){
                    // click
                    if(curr_state == STATE_WAIT){
                        pulse = 0;
                        LED2 = 0;
                        curr_state = STATE_INPUT;
                        
                    }
                }
                else if(PB1_dur >= LONG_PRESS_DUR){
                    //long press?
                    Disp2String("b1 long press\n\r");
                }
                
                
            }
            else if(b_event == PB2_PRESS){
                Disp2String("button2 press\n\r");
                //PB2_press_time = xTaskGetTickCount();
                PB2_s.press_time = xTaskGetTickCount();
                PB2_s.is_pressed = 1;
                
            }
            else if(b_event == PB2_RELEASE){
                Disp2String("button2 RELEASE\n\r");
                //PB2_release_time = xTaskGetTickCount();
                PB2_s.release_time = xTaskGetTickCount();
                PB2_s.is_pressed = 0;
                
                TickType_t PB2_dur = PB2_s.release_time - PB2_s.press_time;
                if(PB2_dur < LONG_PRESS_DUR){
                    // click
                    continue;
                }
                else if(PB2_dur >= LONG_PRESS_DUR){
                    //long press?
                    Disp2String("b2 long press\n\r");
                }
            }
            
            else if(b_event == PB3_PRESS){
                Disp2String("button3 press\n\r");
                //PB3_press_time = xTaskGetTickCount();
                PB3_s.press_time = xTaskGetTickCount();
                PB3_s.is_pressed = 1;
                
            }
            else if(b_event == PB3_RELEASE){
                Disp2String("button3 RELEASE\n\r");
                //PB3_release_time = xTaskGetTickCount();
                PB3_s.release_time = xTaskGetTickCount();
                PB3_s.is_pressed = 0;
                
                TickType_t PB3_dur = PB3_s.release_time - PB3_s.press_time;
                if(PB3_dur < LONG_PRESS_DUR){
                    // click
                    continue;
                    
                }
                else if(PB3_dur >= LONG_PRESS_DUR){
                    //long press?
                    Disp2String("b3 long press\n\r");
                }
            }
        
        check_combos();
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
    
    ButtonQueueHandle = xQueueCreate(10, sizeof(button_event_t));
    
    
    
    xTaskCreate(v_button_task, "Button Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(v_main_state_task, "Main State", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    
    
    
    static uint8_t timer_ids[NUM_BUTTONS];
    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
        timer_ids[i] = i;
        DebounceTimers[i] = xTimerCreate("Debounce Timer", DEBOUNCE_DELAY, pdFALSE, (void *)&timer_ids[i], v_timer_callback);
    }
    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
        Disp2Dec(i);
        Disp2String("\n\r");
        if(DebounceTimers[i] == NULL){
            Disp2String("Timer Create Failed for button ");
            Disp2Dec(i);
            Disp2String("\n\r");
        }
    }
   
    //xTimerStart(DebounceTimers[2], 0); // what the hell bro why did this fix it

    // i think the following fixes it just as well
    // i think the problem was with the debouncing flag on pb3???? it was setting for some reason?
    debounce_prog[0] = 0;
    debounce_prog[1] = 0;
    debounce_prog[2] = 0;
    
    
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
 * */

void __attribute__ ((interrupt, no_auto_psv)) _IOCInterrupt(void){
//    static button_event_t pending_events[NUM_BUTTONS];
//    static uint8_t event_pending[NUM_BUTTONS] = {0, 0, 0};
   
    //debounce_prog[2] = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //button_event_t b_event;
//    LED0 ^=1;
    
    LED0 ^= 1;

    if(IOCFBbits.IOCFB9) {
        //LED2 ^= 1;   // RB9 flag detected
    } else {
        //LED2 = 0;   // RB9 flag NOT set
    }

    if(debounce_prog[2]) {
        LED1 ^= 1;   // debounce active
    } else {
        //LED1 = 0;   // debounce idle
    }
    /*
    if(IOCFAbits.IOCFA4){
        pending_events[0] = PORTAbits.RA4;
        if(!debounce_prog[0]){
            debounce_prog[0] = 1;
            xTimerStartFromISR(DebounceTimers[0], &xHigherPriorityTaskWoken);
        }
        IOCFAbits.IOCFA4 = 0;   // clear flag on RA4 pin
    }
    
    if(IOCFBbits.IOCFB8){
        pending_events[1] = PORTBbits.RB8;
        if(!debounce_prog[1]){
            debounce_prog[1] = 1;
            xTimerStartFromISR(DebounceTimers[1], &xHigherPriorityTaskWoken);
        }
        IOCFBbits.IOCFB8 = 0;   // clear flag on RB8 pin
    }
    
    if(IOCFBbits.IOCFB9){
        pending_events[2] = PORTBbits.RB9;
        if(!debounce_prog[2]){
            debounce_prog[2] = 1;
            xTimerStartFromISR(DebounceTimers[2], &xHigherPriorityTaskWoken);
        }
        IOCFBbits.IOCFB9 = 0;   // clear flag on RB9 pin
    }
    */
    
    
//    TickType_t now = xTaskGetTickCountFromISR();    // should be safe
   
    
    if(IOCFAbits.IOCFA4 && !debounce_prog[0]){       // for PB1

//        b_event = (PORTAbits.RA4) ? PB1_RELEASE : PB1_PRESS;
//        xQueueSendFromISR(ButtonQueueHandle, &b_event, &xHigherPriorityTaskWoken);
        //pending_events[0] = (PORTAbits.RA4) ? PB1_RELEASE : PB1_PRESS;
        pending_events[0] = PORTAbits.RA4;
        event_pending[0] = 1;
        debounce_prog[0] = 1;
        xTimerStartFromISR(DebounceTimers[0], &xHigherPriorityTaskWoken);
        
        IOCFAbits.IOCFA4 = 0;   // clear flag on RA4 pin
    }
    
    if(IOCFBbits.IOCFB8 && !debounce_prog[1]){       // for PB2
//        b_event = (PORTBbits.RB8) ? PB2_RELEASE : PB2_PRESS;
//        xQueueSendFromISR(ButtonQueueHandle, &b_event, &xHigherPriorityTaskWoken);
        //LED2 ^= 1;
        //pending_events[1] = (PORTBbits.RB8) ? PB2_RELEASE : PB2_PRESS;
        pending_events[1] = PORTBbits.RB8;
        event_pending[1] = 1;
        debounce_prog[1] = 1;
        xTimerStartFromISR(DebounceTimers[1], &xHigherPriorityTaskWoken);
        
        IOCFBbits.IOCFB8 = 0;   // clear flag on RB8 pin
    }
    
    if(IOCFBbits.IOCFB9 && !debounce_prog[2]){       // for PB3
//        b_event = (PORTBbits.RB9) ? PB3_RELEASE : PB3_PRESS;
//        xQueueSendFromISR(ButtonQueueHandle, &b_event, &xHigherPriorityTaskWoken);
        //LED1 ^= 1;
        
        //pending_events[2] = (PORTBbits.RB9) ? PB3_RELEASE : PB3_PRESS;
        pending_events[2] = PORTBbits.RB9;
        event_pending[2] = 1;
        debounce_prog[2] = 1;

        BaseType_t ret = xTimerStartFromISR(DebounceTimers[2], &xHigherPriorityTaskWoken);
        if(ret != pdPASS){
            LED4 = 1;
        }
        IOCFBbits.IOCFB9 = 0;   // clear flag on RB9 pin
    }
   
    
    //xTimerStartFromISR(DebounceTimerHandle, &xHigherPriorityTaskWoken);     // should be safe?
    IFS1bits.IOCIF = 0;  // clear global IOC flags 
    if(xHigherPriorityTaskWoken) {
        portYIELD();  // triggers context switch
    }
    
//    xQueueSendFromISR(ButtonQueueHandle, &b_event, &xHigherPriorityTaskWoken);
     
    
}

