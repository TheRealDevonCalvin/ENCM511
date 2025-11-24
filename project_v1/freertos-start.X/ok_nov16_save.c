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
#define NUM_ENTRY 4

#define LED4 LATBbits.LATB15
#define LED5 LATBbits.LATB14

#define CLICK_DUR pdMS_TO_TICKS(500)
#define LONG_PRESS_DUR pdMS_TO_TICKS(750) // can adjust as needed

#define COMBO_DUR pdMS_TO_TICKS(200)




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

/*---------------LED--------------------*/
volatile uint16_t PWM_TON = 400;
volatile uint16_t PWM_TOFF = 1;       // these may need to be vars not macros to be editable
                                        // note that we can NOT set these to be 0 (janks the timer)
volatile uint16_t PWM_PER = 400;
volatile uint8_t pwm_on = 1;

volatile uint8_t dec = 1;
volatile uint8_t pulse = 1;     // control if the LED should be pulsing in the T3ISR

#define BLINK_RATE pdMS_TO_TICKS(1000)    // 1s
#define PULSE_STEP_SIZE 3

TimerHandle_t led_blink_timer;


typedef enum{
    LED_MODE_OFF,
    LED_MODE_ON,
    LED_MODE_PULSE,
    LED_MODE_BLINK
} led_mode_t;

volatile led_mode_t led2_mode = LED_MODE_OFF;
volatile uint8_t led2_blink_phase = 0;
volatile uint8_t led1_blink_en = 0;

volatile uint8_t led2_variable_duty = 1000;      // later to be modified by adc???
volatile uint8_t led2_output_en = 0;

/*--------------- END LED--------------------*/


typedef enum{
    STATE_WAIT,
    STATE_INPUT,
    STATE_COUNT, 
    STATE_END
} state_t;
volatile state_t curr_state = STATE_WAIT;


QueueHandle_t ButtonQueueHandle;
QueueHandle_t ProcessedButtonQueue;

QueueHandle_t UartRxQueue;

TimerHandle_t DebounceTimers[NUM_BUTTONS];

typedef struct{
    uint8_t id;
    uint8_t level;
    TickType_t ts;
} button_event_t;

volatile uint8_t b_before_debounce[NUM_BUTTONS];      // value sampled in ISR
volatile uint8_t b_after_debounce[NUM_BUTTONS];       // value after debounce
volatile uint8_t b_last_valid[NUM_BUTTONS] = {1,1,1}; // last handled (stable) state (1=released)
volatile TickType_t b_press_times[NUM_BUTTONS];      // last press time
volatile TickType_t b_release_times[NUM_BUTTONS];    // last release time
volatile uint8_t b_handled[NUM_BUTTONS] = {1,1,1};    // 0 = this press/release not yet handled by v_button_task
volatile uint8_t debounce_active[NUM_BUTTONS] = {0,0,0}; // timers started
volatile uint8_t debounce_done[NUM_BUTTONS] = {0,0,0};



volatile char time_entry[NUM_ENTRY] = {0,0,0,0};
volatile uint8_t time_ready = 0;
volatile uint8_t time_entry_index = 0;


typedef enum{
    EVT_CLICK, 
    EVT_LONG, 
    EVT_COMBO, 
    EVT_LONG_COMBO
} event_type_t;

typedef struct{
    TickType_t ts;
    uint8_t buttons_mask;
    event_type_t type;
} processed_event_t;

void v_timer_callback(TimerHandle_t xTimer){
    
    uint8_t button_id = (uint8_t)(uintptr_t)pvTimerGetTimerID(xTimer);
    
    uint8_t raw = 1;
    switch(button_id){
        case 0: raw = PORTAbits.RA4; break;
        case 1: raw = PORTBbits.RB8; break;
        case 2: raw = PORTBbits.RB9; break;
        default: return;
    }
    
    b_after_debounce[button_id] = raw;
    TickType_t now = xTaskGetTickCount();
    if(raw == 0){
        b_press_times[button_id] = now;
        b_handled[button_id] = 0;
    }
    else{
        b_release_times[button_id] = now;
    }

    button_event_t evt;
    evt.id = button_id;
    evt.level = raw;
    evt.ts = now;

    xQueueSend(ButtonQueueHandle, &evt, 0);

    debounce_active[button_id] = 0;
    debounce_done[button_id] = 1;
}




void v_button_task(void *pvParameters){
    Disp2String("hello in button task\n\r");
 
    button_event_t evt;
    
    uint8_t state[NUM_BUTTONS];
    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
        state[i] = b_last_valid[i];
    }
    
    for(;;){
        if(xQueueReceive(ButtonQueueHandle, &evt, portMAX_DELAY) == pdTRUE){
            uint8_t id = evt.id;
            uint8_t level = evt.level;
            TickType_t ts = evt.ts;
            
            state[id] = level;
            b_last_valid[id] = level;
            
            Disp2String("DBG: button ");
            Disp2Dec(id + 1);
            Disp2String(level ? " released\n\r" : " pressed\n\r");
            
            if(level == 1){
                b_release_times[id] = ts;
            }
            else{
                b_press_times[id] = ts;
                b_handled[id] = 0;
            }
            
            uint8_t candidates_mask = 0;
            uint8_t cand_count = 0;
            TickType_t min_press = (TickType_t)0xffffffff;
            TickType_t max_press = 0;
            
            for(uint8_t i = 0; i < NUM_BUTTONS; i++){
                if(b_handled[i] == 0 && state[i] == 1){
                    candidates_mask |= (1 << i);
                    cand_count++;
                    if(b_press_times[i] < min_press){
                        min_press = b_press_times[i];
                    }
                    if(b_press_times[i] > max_press){
                        max_press = b_press_times[i];
                    }
                }
            }
            
            if(cand_count > 0){
                TickType_t press_spread = max_press - min_press;
                
                if(cand_count >= 2 && press_spread <= COMBO_DUR){
                    uint8_t all_long = 1;
                    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
                        if(candidates_mask & (1 << i)){
                            TickType_t dur = b_release_times[i] - b_press_times[i];
                            if(dur < LONG_PRESS_DUR){
                                all_long = 0;
                                break;
                            }
                        }
                    }
                    
                    // __________________________________________-
                    processed_event_t pevt;
                    pevt.ts = ts;
                    pevt.buttons_mask = candidates_mask;
                    pevt.type = all_long ? EVT_LONG_COMBO : EVT_COMBO;
                    
                    xQueueSend(ProcessedButtonQueue, &pevt, 0);
                    // __________________________________________-
                    
                    if(all_long){
//                        Disp2String("LONG combo: ");
                    }
                    else{
//                        Disp2String("Combo: ");
                    }
                    
                    uint8_t first = 1;
                    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
                        if(candidates_mask & (1<<i)){
//                            if(!first) Disp2String("+");
//                            Disp2Dec(i + 1);
                            first = 0;
                        }
                    }
//                    Disp2String("\n\r");
                    
                    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
                        if(candidates_mask & (1<<i)){
                            b_handled[i] = 1;
                        }
                    } 
                }
                else if(cand_count == 1){
                    uint8_t only = 0;
                    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
                        if(candidates_mask & (1<<i)){
                            only = i;
                            break;
                        }
                    }                    
                    
                    uint8_t potential_combo = 0;
                    for(uint8_t j = 0; j < NUM_BUTTONS; j++){
                        if(j == only) continue;
                        if(b_handled[j] == 0){
                            if(state[j] == 0){
                                TickType_t dt = (b_press_times[only] > b_press_times[j]) ? (b_press_times[only] - b_press_times[j]) : (b_press_times[j] - b_press_times[only]);
                                if(dt <= COMBO_DUR){
                                    potential_combo = 1;
                                    break;
                                }
                            }
                        }
                    }
                    
                    if(potential_combo){                       
                    }
                    else{
                  
                        TickType_t dur = b_release_times[only] - b_press_times[only];
                        // __________________________________________-
                        processed_event_t pevt;
                        pevt.ts = ts;
                        pevt.buttons_mask = (1 << only);
                    
                        if(dur >= LONG_PRESS_DUR){
//                            Disp2String("LONG PB");
//                            Disp2Dec(only + 1);
//                            Disp2String("\n\r");
                            
                            pevt.type = EVT_LONG;
                        }
                        else{
//                            Disp2String("PB");
//                            Disp2Dec(only+1);
//                            Disp2String(" click\n\r");
                            
                            pevt.type = EVT_CLICK;
                        }
                        
                        xQueueSend(ProcessedButtonQueue, &pevt, 0);
                        b_handled[only] = 1;
                        
                        // __________________________________________-
                    }
                }
                
                else{
                    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
                        if(candidates_mask & (1<<i)){
                            TickType_t dur = b_release_times[i] - b_press_times[i];
                            if(dur >= LONG_PRESS_DUR){
//                                Disp2String("LONG PB");
//                                Disp2Dec(i + 1);
//                                Disp2String("\n\r");
                            }
                            else{
//                                Disp2String("PB");
//                                Disp2Dec(i+1);
//                                Disp2String(" click\n\r");
                            }
                            b_handled[i] = 1;
                        }
                    }
                }
            }// end if(cand_count > 0)            
        }// end if(xQueueReceive())
    }// end main for(;;) loop
} // end button task

void v_blink_callback_count(TimerHandle_t xTimer){
    
    led2_blink_phase ^= 1;  // toggles blinking phase
                            // this is fine to run always, as we pick and choose when it is used

    if(led1_blink_en){
        LED1 ^= 1;
    }
}

void v_main_state_task(void *pvParameters){

    Disp2String("hello in main\n\r");
    
    processed_event_t pevt;
    for(;;){
        // state logic?
        while(xQueueReceive(ProcessedButtonQueue, &pevt, 0) == pdTRUE){
            switch(pevt.type){
                case EVT_CLICK:
                    Disp2String("main see click");
                    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
                        if(pevt.buttons_mask & (1 << i)){
                            Disp2Dec(i + 1);
                            Disp2String("\n\r");
                        }
                    }
                    // deal with transition out of STATE_WAIT
                    if(curr_state == STATE_WAIT && (pevt.buttons_mask & (1 << 0))){
                        curr_state = STATE_INPUT;
                        time_ready = 0;
                        Disp2String("Enter a Time: mm:ss");
                        Disp2String("\033[15G");        // escape sequence to place the cursor at the tens place
                                                        // (the 15th character in the display)
                    }                    
                    break;
                case EVT_LONG:
                    Disp2String("main see LONG");
                    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
                        if(pevt.buttons_mask & (1 << i)){
                            Disp2Dec(i + 1);
                            Disp2String("\n\r");
                        }
                    }
                    break;
                case EVT_COMBO:
                    Disp2String("main see combo");
                    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
                        if(pevt.buttons_mask & (1 << i)){
                            Disp2Dec(i + 1);
                            Disp2String("+");
                        }
                    }
                    Disp2String("\n\r");
                    
                    // deal with pb2/pb3 clicks out of STATE_INPUT
                    if(curr_state == STATE_INPUT && (pevt.buttons_mask & (11 << 1))){
                        if(time_ready){
                            curr_state = STATE_COUNT;
                            Disp2String("yay count!\n\r");
                            
                            // check these. aiming to use them to sync the blinking
                            LED1 = 1;
                            LED2 = 1;
                            led1_blink_en = 1;
                            
                            led2_mode = LED_MODE_BLINK;     //moved here cause overwriting in fsm
                        }
                        else if(!time_ready){
                            Disp2String("oops, time not ready!\n\r");
                            curr_state = STATE_INPUT;
                            
                            time_ready = 0;
                            time_entry_index = 0;
                            Disp2String("Enter a Time: mm:ss");
                            Disp2String("\033[15G");        // escape sequence to place the cursor at the tens place
                                                            // (the 15th character in the display)
                            for(uint8_t j = 0; j < NUM_ENTRY; j++){
                                time_entry[j] = 0;
                            }
                            xQueueReset(UartRxQueue);       // clears the queue! so no carryover of jankiness
                        }
                    }
                    
                    break;
                case EVT_LONG_COMBO:
                    Disp2String("main see LONG combo");
                    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
                        if(pevt.buttons_mask & (1 << i)){
                            Disp2Dec(i + 1);
                            Disp2String("+");
                        }
                    }
                    Disp2String("\n\r");
                    
                    // deal w long PB2/PB3 to reset input in STATE_INPUT
                    if(curr_state == STATE_INPUT && (pevt.buttons_mask & (11 << 1))){
                        curr_state = STATE_INPUT; 
                        time_ready = 0;
                        time_entry_index = 0;
                        Disp2String("Enter a Time: mm:ss");
                        Disp2String("\033[15G");        // escape sequence to place the cursor at the tens place
                                                        // (the 15th character in the display)
                        for(uint8_t j = 0; j < NUM_ENTRY; j++){
                            time_entry[j] = 0;
                        }
                        xQueueReset(UartRxQueue);
                    }
                    
                    break;
                    
            }   // end switch(pevt.type)
        }// end while(xQueueReceive)
        
        
        switch(curr_state){
            case STATE_WAIT:
                // handle waiting behaviour
                // this is LED pulsing
                // click of PB1 to do the transition
                //Disp2String("in wait state\n\r");
                
                pulse = 1;      // set LED2 to pulse according to the timer3
                led2_mode = LED_MODE_PULSE;
                
                break;
                
            case STATE_INPUT:
                // get user input for the mins/secs 
                // 
                // long press of PB2/PB3 resets input time
                // click of PB2/PB3 proceeds state to begin counting down
                
                pulse = 0;      // turn off the pulsing of LED2
                LED2 = 0;       // and ensure LED2 itself gets turned off
                led2_mode = LED_MODE_OFF;
                
                
                LED5 = 1;       // THIS IS A DEBUG!!! KILL LATER!!!!             
                
                uint8_t recv_byte;
                if(!time_ready){
                    if(xQueueReceive(UartRxQueue, &recv_byte, 10) == pdTRUE){
                        // ^ queue is used to avoid blocking
                        char c = (char)recv_byte;

                        if(c >= '0' && c <= '9'){
                            time_entry[time_entry_index] = c;
                            XmitUART2(c, 1);

                            if(time_entry_index == 1){
                                Disp2String("\033[1C"); // skip to help with alignment
                            }
                            time_entry_index++;

                            if(time_entry_index >= NUM_ENTRY){
                                time_entry_index = 0;
                                //signal ready to go???
                                time_ready = 1;
                            }
                        }
                        else{}
                    }
                }
                
                
                
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
                
                
                
                //led1_blink_en = 1;
                //led2_mode = LED_MODE_BLINK;
                
                /*-----------current working on---------------*/
                if(xQueueReceive(UartRxQueue, &recv_byte, 10) == pdTRUE){
                    
                    char c = (char)recv_byte;
                    if(c == 'b'){
                        Disp2String("char b was entered\n\r");
                        if(led2_mode == LED_MODE_ON) led2_mode = LED_MODE_BLINK;
                        else if(led2_mode == LED_MODE_BLINK) led2_mode = LED_MODE_ON;
                    }
                    if(c == 'i'){
                        Disp2String("char i was entered\n\r");
                        // handle display changes
                    }
                    xQueueReset(UartRxQueue);
                }
                    
                    
                /*-----------END current working on ---------------*/
                
                
                break;
                
            case STATE_END:
                // display end message (countdown done)
                // LED0/LED1 rapidly blink alternating
                // LED2 solidly on (still pot for brightness)
                
                // after 5s elapsed, go back to wait
                
                break;
            
        }   // end switch(curr_state)
    }// end for(;;)
}// end main_state_task()




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

    // help w init bounce maybe?
    b_after_debounce[0] = PORTAbits.RA4;
    b_after_debounce[1] = PORTBbits.RB8;
    b_after_debounce[2] = PORTBbits.RB9;
    
    
//	xTaskCreate( vTask1, "Task1", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
//    xTaskCreate( vTask2, "Task2", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
//    xTaskCreate( vTask3, "Task3", configMINIMAL_STACK_SIZE, NULL, 3, NULL );
//    uart_sem = xSemaphoreCreateMutex();
    
    
    ButtonQueueHandle = xQueueCreate(10, sizeof(button_event_t));
    
    ProcessedButtonQueue = xQueueCreate(10, sizeof(processed_event_t));
    
    UartRxQueue = xQueueCreate(32, sizeof(uint8_t));    // uint8_t == char so all is well
    
    
    xTaskCreate(v_button_task, "Button Task", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);
    xTaskCreate(v_main_state_task, "Main State", configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL);
    
    
    
    static uint8_t timer_ids[NUM_BUTTONS];
    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
        timer_ids[i] = i;
        DebounceTimers[i] = xTimerCreate("Debounce Timer", DEBOUNCE_DELAY, pdFALSE, (void *)(uintptr_t)i, v_timer_callback);
    }
    
    
    led_blink_timer = xTimerCreate("Blink Timer", BLINK_RATE, pdTRUE, (void *)0, v_blink_callback_count);
    xTimerStart(led_blink_timer, 0);
    
    Disp2String("Press PB1 to begin!\n\r");
    

    LED1 = 0;
    TMR3 = 0;
    PR3 = PWM_TON;
    T3CONbits.TON = 1;

    vTaskStartScheduler();
    
    for(;;);
}



void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void){
    /*
     * pulsing?????
     */
    //LED0 ^= 1;
    TMR3 = 0;               // reset the timer start count
    IFS0bits.T3IF = 0;      // clear the interrupt flag
    uint16_t duty;
    
    switch(led2_mode){
        case LED_MODE_OFF:
            led2_output_en = 0;
            duty = 0;
            break;
        case LED_MODE_ON:
            led2_output_en = 1;
            duty = led2_variable_duty;
            break;
        case LED_MODE_BLINK:
            duty = led2_variable_duty;
            led2_output_en = led2_blink_phase;
            
//            if(led2_blink_phase){
//                duty = led2_variable_duty;
//            }
//            else{
//                duty = 0;
//            }
            
            break;
        case LED_MODE_PULSE:
            led2_output_en = 1;
            if(dec){
                PWM_TON -= PULSE_STEP_SIZE;
                if(PWM_TON <= PULSE_STEP_SIZE){
                    dec = 0;
                }
            }
            if(!dec){
                PWM_TON += PULSE_STEP_SIZE;
                if(PWM_TON >= PWM_PER - PULSE_STEP_SIZE){
                    dec = 1;
                }
            }
            duty = PWM_TON;
            break;        
    }// end switch(led2_mode)
    
//    if(led2_mode == LED_MODE_BLINK && !led2_blink_phase){
//        duty = 0;
//    }
    
//    static uint8_t pwm_on = 0;
    if(led2_output_en){
        if(pwm_on){
            LED2 = 0;
            PR3 = PWM_PER - duty;
            pwm_on = 0;
        }
        else{
            LED2 = 1;
            PR3 = duty;
            pwm_on = 1;
        }
    }
    else{
        LED2 = 0;
    }
    

    
}


void __attribute__ ((interrupt, no_auto_psv)) _IOCInterrupt(void){

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    LED4 ^= 1;              // debugging LED


    // PB1
    if(IOCFAbits.IOCFA4){
        IOCFAbits.IOCFA4 = 0;   // clear flag on RA4 pin

        b_before_debounce[0] = PORTAbits.RA4;
        
        if(b_before_debounce[0] != b_last_valid[0]){
            debounce_done[0] = 0;
            if(debounce_active[0]){
            }
            else{
                xTimerStartFromISR(DebounceTimers[0], &xHigherPriorityTaskWoken);
                debounce_active[0] = 1;
            }
        }       
    }
    
    // for PB2
    if(IOCFBbits.IOCFB8){
        IOCFBbits.IOCFB8 = 0;   // clear flag on RB8 pin
        
        b_before_debounce[1] = PORTBbits.RB8;
        if(b_before_debounce[1] != b_last_valid[1]){
            debounce_done[1] = 0;
            if(debounce_active[1]){
            }
            else{
                xTimerStartFromISR(DebounceTimers[1], &xHigherPriorityTaskWoken);
                debounce_active[1] = 1;
            }
        }
    }
    
    // for PB3
    if(IOCFBbits.IOCFB9){  
        IOCFBbits.IOCFB9 = 0;   // clear flag on RB9 pin
        
        b_before_debounce[2] = PORTBbits.RB9;
        if(b_before_debounce[2] != b_last_valid[2]){
            debounce_done[2] = 0;

            if(debounce_active[2]){
            }
            else{
                xTimerStartFromISR(DebounceTimers[2], &xHigherPriorityTaskWoken);
                debounce_active[2] = 1;
            }
        }
    }
   
    
    IFS1bits.IOCIF = 0;  // clear global IOC flags 
    if(xHigherPriorityTaskWoken) {
        portYIELD();  // triggers context switch
    }
       
}
