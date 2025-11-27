/*
 * File:   main.c
 * Author: Chloe Fulbrook, Ethan Lee, Devon Calvin
 *
 * Created FOR ENCM 511
 * PLEASE ADD DATE CREATED HERE: 2025-11-08
 * 
 * FAILURE TO UPDATE THIS HEADER WITH YOUR GROUP MEMBER NAMES
 * MAY RESULT IN PENALTIES
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

#include "adc.h"

#include "timers.h"
#include "init.h"

#define NUM_ENTRY 4

//#define LED4 LATBbits.LATB15
#define LED5 LATBbits.LATB14

#define TASK_STACK_SIZE 200
#define TASK_PRIORITY 5

SemaphoreHandle_t uart_sem;
SemaphoreHandle_t led2_sem;

SemaphoreHandle_t button_sem;


void vApplicationIdleHook( void )
{
}
/*-----------------------------------------------------------*/

uint8_t broked = 0;     // DEBUG VARIABLE
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

    broked = 1; // DEBUG
	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
    Disp2String(pcTaskName);
	for( ;; );
}




void prvHardwareSetup(void){
    // this function calls the initializer functions required for the application
    TRISBbits.TRISB5 = 0;
    InitUART2();
    IOinit();
    timerInit();
    ADC1_init();  
}

/*--------------- LED vars --------------------*/

volatile uint16_t PWM_TON = 400;
volatile uint16_t PWM_TOFF = 1;       // these may need to be vars not macros to be editable
                                        // note that we can NOT set these to be 0 (janks the timer)
volatile uint16_t PWM_PER = 400;
volatile uint8_t pwm_on = 1;

volatile uint8_t dec = 1;
volatile uint8_t pulse = 1;     // control if the LED should be pulsing in the T2ISR

#define BLINK_RATE pdMS_TO_TICKS(1000)    // 1s
#define PULSE_STEP_SIZE 3

#define ALT_BLINK_RATE pdMS_TO_TICKS(200)   //200ms??

TimerHandle_t led_blink_timer;
TimerHandle_t led_alternate_timer;


typedef enum{
    LED_MODE_OFF,
    LED_MODE_ON,
    LED_MODE_PULSE,
    LED_MODE_BLINK
} led_mode_t;

volatile led_mode_t led2_mode = LED_MODE_OFF;
volatile uint8_t led2_blink_phase = 0;
volatile uint8_t led1_blink_en = 0;

volatile uint16_t led2_variable_duty = 1;      // later to be modified by adc???
volatile uint8_t led2_output_en = 0;

/*--------------- END LED vars --------------------*/


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

QueueHandle_t DisplayQueue;

QueueHandle_t adcQueue;
QueueHandle_t DebounceQueue;

TaskHandle_t main_state_task_handler;



/*--------------- button vars --------------------*/

#define DEBOUNCE_DELAY pdMS_TO_TICKS(50)
#define NUM_BUTTONS 3

#define CLICK_DUR pdMS_TO_TICKS(500)
#define LONG_PRESS_DUR pdMS_TO_TICKS(750) // can adjust as needed

#define COMBO_DUR pdMS_TO_TICKS(200)


TimerHandle_t DebounceTimers[NUM_BUTTONS];

typedef struct{
    uint8_t mask;
    uint8_t active;
    TickType_t ts;
} button_event_t;

button_event_t active_press = {0,0,0};

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


volatile uint8_t b_before_debounce[NUM_BUTTONS];      // value sampled in ISR
volatile uint8_t b_after_debounce[NUM_BUTTONS];       // value after debounce
volatile uint8_t b_last_valid[NUM_BUTTONS] = {1,1,1}; // last handled (stable) state (1=released)
volatile TickType_t b_press_times[NUM_BUTTONS];      // last press time
volatile TickType_t b_release_times[NUM_BUTTONS];    // last release time
volatile uint8_t b_handled[NUM_BUTTONS] = {1,1,1};    // 0 = this press/release not yet handled by v_button_task
volatile uint8_t debounce_active[NUM_BUTTONS] = {0,0,0}; // timers started
volatile uint8_t debounce_done[NUM_BUTTONS] = {0,0,0};


#define PB1_MASK (1 << 0)
#define PB3_MASK (1 << 2) 
#define PB2_PB3_MASK ((1 << 1) | (1 << 2))

/*--------------- END button vars --------------------*/


/*-------------- COUNTDOWN vars -----------------*/

volatile char time_entry[NUM_ENTRY] = {0,0,0,0};
volatile uint8_t time_ready = 0;
volatile uint8_t time_entry_index = 0;

volatile uint16_t time_seconds = 0;

volatile uint8_t countdown_running = 0;
volatile char time_string[NUM_ENTRY + 2];


/*-------------- END COUNTDOWN vars -----------------*/


/*-------------- end timeout vars -----------------*/
TimerHandle_t end_timer;
#define END_TIMEOUT pdMS_TO_TICKS(5000)

typedef enum{
    NOTIFY_COUNTDOWN, 
    NOTIFY_END_TIMEOUT
} main_notify_t;

/*-------------- END end timeout vars -----------------*/


/*-------------- ADC vars -----------------*/

// currently, a big decision to be made is whether the ADC should be sampled periodically or as an on-change kind of thing
// additionally, should the ADC do polling, or is it better to use the ADC interrupt
    // doing the interrupt would require investigation into how it works
    // i guess the consideration comes to CPU time/usage and if we think we can afford to busy-wait poll the ADC flag.
    // BUT, will the adc isr run continuously and cause problems there? or is it disable-able so it can run periodic??
    // another potential issue, cant call functions from that ADC ISR, but I assume this isnt a huge deal bc just read flag and store result?
    //

    // interrupt has buffer fill stuff going on (SMPI register)
        // SMPI sets a number of samples to obtain before interrupting. 
        // stores them in a series of ADC1BUFx registers, that could hypothetically be averaged????
        // eliminates the need for the do_adc_avg things, and shouldnt have a busy-wait????
        // this could then probably run in the background??? 
        // BUT the isr would need to be coordinated so it doesnt hurt any other operation
        // this could be tricky, bc priority, UART use, etc. 
        // AND the user input can happen at any time, so what happens then???? if they twist at a bad time??
        
        // is it worth enabling/disabling the interrupts as needed? or can we pause(?) it somehow?

// even then, does ADC go in background and run main, or should it have a task? 
    // given a task, more modular, easier separability. 
        // also potentially easier delays with vTaskDelay(until?)()
    // in main, potential to jank up main???

// then, another thing is the repeated updating
    // should the ADC only update the screen as the countdown updates the screen, or should it update on change
    // problems with both imo. 
    // if update only on countdown change, we'd have problems with the responsiveness of the screen, meaning that the
    // brightness may change, but the screen won't update that until the next countdown
    // but if update consistently, will we have collision or races?? or just jitter in the screen???


// ALSO may need to consider timing for the ADC
    // ADC takes some amount of time to get through the SAR, so need to make sure that this does not interfere with 
    // with other things happening in the system, and that it has enough time to appropriately sample
    // and how does this play into the periodic vs aperiodic tasks thoughts, polling vs ISR, etc???

// ^^ on the above:
    // we are NOT polling the done flag, just pulling straight from the interrupt!!
    // so far (as of Nov 23 when i wrote this), things seem to be working decently with this. 


#define tmr5ms pdMS_TO_TICKS(5)

volatile uint16_t raw_adc_result;
volatile uint16_t adc_result;

volatile uint16_t led2_intensity;   // ideally, will be used to discover the relative(%maybe?) intensity of LED2 based on adc
                                    // i suppose this is just basically the duty cycle lol

/*-------------- END ADC vars -----------------*/


/*-------------- display vars -----------------*/

typedef enum{
    DISP_CLEAR,
    DISP_TIME, 
    DISP_FULL,
    DISP_TEXT,
} display_cmd_t;

typedef struct{
    display_cmd_t cmd;
    
//    uint16_t val1;
//    uint16_t val2;
    
    char time[10];
    uint16_t adc;
    uint16_t intensity;
    
//    char text[32];
    
} display_msg_t;

// vv currently used (above not rn)
typedef enum{
    DISP_MODE_TIME,
    DISP_MODE_FULL,
} disp_mode_t;

volatile disp_mode_t display_mode = DISP_MODE_TIME;


/*-------------- END display vars -----------------*/


//function prototype see if fix
void v_main_state_task(void *pvParameters);

//volatile uint8_t last_stable_state = 0b111;
void v_button_task(void *pvParameters){
    vTaskDelay(100); // let hardware stabilize
    uint8_t pb1 = PB1;
    uint8_t pb2 = PB2;
    uint8_t pb3 = PB3;
    uint8_t last_stable_state = (~((pb3<<2)|(pb2<<1)|pb1)) & 0b111;  // all released (assuming active-low)
//    last_stable_state = 0b111;
    
   xSemaphoreTake(uart_sem, portMAX_DELAY);
    Disp2String("Initial button state: ");
    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
        if(last_stable_state & (1<<i)) Disp2String("R "); // released
        else Disp2String("P "); // pressed
    }
    Disp2String("\n\r");
    xSemaphoreGive(uart_sem);
    for(;;){
        // Wait for ISR to signal an edge occurred
        if (xSemaphoreTake(button_sem, portMAX_DELAY) == pdTRUE){

            
            // Delay for debounce window
            vTaskDelay(DEBOUNCE_DELAY);
           
            // Read debounced states
            pb1 = PB1;
            pb2 = PB2;
            pb3 = PB3;
            uint8_t curr_button = (pb3 << 2) | (pb2 << 1) | (pb1 << 0);
            uint8_t pressed_mask = (~curr_button) & 0b111;
           
            // Only act if state actually changed after debounce
            if(pressed_mask != last_stable_state){
                last_stable_state = pressed_mask;

                if(pressed_mask != 0){
                    active_press.mask = pressed_mask;
                    active_press.ts = xTaskGetTickCount();
                    active_press.active = 1;
                }
                else if(active_press.active){
                    // button released
                    TickType_t dur = xTaskGetTickCount() - active_press.ts;
                    
                    processed_event_t pevt;
                    pevt.buttons_mask = active_press.mask;
                    pevt.ts = xTaskGetTickCount();
                    
                    uint8_t num_pressed = 0;
                    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
                        if(active_press.mask & (1<<i)){
                            num_pressed++;
                        }
                    }
                    if(num_pressed == 1){
                        if(dur >= LONG_PRESS_DUR){
                            pevt.type = EVT_LONG;
                            Disp2String("LONG PB: ");
                        }
                        else{
                            pevt.type = EVT_CLICK;
                            Disp2String("Click PB: ");
                        }
                    }
                    else{
                        if(dur >= LONG_PRESS_DUR){
                            pevt.type = EVT_LONG_COMBO;
                            Disp2String("Long Combo: ");
                        }
                        else{
                            pevt.type = EVT_COMBO;
                            Disp2String("Combo: ");
                        }
                    }
                    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
                        if(active_press.mask & (1<<i)){
                            Disp2Dec(i+1);
                            Disp2String(" ");
                        }
                    }
                    Disp2String("\n\r");
                    
                    xQueueSend(ProcessedButtonQueue, &pevt, 0);
                    active_press.active = 0;
                }
            }
        }
    }
}
    

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

void v_end_callback(TimerHandle_t xTimer){
    // this timer callback runs once the 5 seconds of timeout have elapsed after the completion of the countdown
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // the callback notifies the main state task that the timer has elapsed, and the main state deals with the transition
    xTaskNotifyFromISR(main_state_task_handler, (1<<NOTIFY_END_TIMEOUT), eSetBits, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken){
        portYIELD();
    }
}


void int_to_str_dec(uint16_t dec, char *str){
    // a function that converts a decimal number to a string. this allows it to be used by the UART
    int i = 0;
    if(dec == 0){
        // if the number is just zero, put that in the string and move on
        str[i] = '0';
        i++;    // dont forget to increment the index variable!
    }
    else{
        while(dec > 0){
            // while the number is more than 0, we modulo off the digits and put them in the string
            str[i++] = (dec % 10) + '0';
            dec /= 10;
        }
    }
    
    int start = 0;
    int end = i - 1;
    while(start < end){
        // this loop just reverses the string so it's the right order
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end --;
    }
    str[i] = '\0';  // null-terminate the string 
    return; 
}

void v_countdown_task(void *pvParameters){
    // this task is responsible for the actual 
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    display_msg_t msg;
    
    for(;;){
        if(countdown_running && time_seconds > 0){
            // if the countdown is running, and there is time left on it,
            // we wait one second (using delayuntil to have better timing)
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
            
            
            // --------------------------------------------------------------------------
            // this needs to be checked with the timing of the countdown
            // trying to use it to stop the prints after the pause/abort message
            if(!countdown_running){
                // resync timer anchor????
                xLastWakeTime = xTaskGetTickCount();
                continue;
            }
            // --------------------------------------------------------------------------
            
            
            time_seconds--; // decrement the time remaining
            
            // calculate the minutes and seconds for display and package into a time string
            uint16_t t = time_seconds;
            uint8_t mm = t / 60;
            uint8_t ss = t % 60;
            
            time_string[0] = (mm / 10) + '0';
            time_string[1] = (mm % 10) + '0';
            time_string[2] = ':';
            time_string[3] = (ss / 10) + '0';
            time_string[4] = (ss % 10) + '0';
            time_string[5] = '\0';
            
            
            uint8_t i; 
            for(i = 0; time_string[i] != '\0'; i++){
                msg.time[i] = time_string[i];
            }
            msg.time[i] = '\0';
//            msg.time = time_string;
            msg.adc = raw_adc_result;
            msg.intensity = led2_intensity;
            
            // ------------------------------------------------------------------------------------
            // this should really go in a display function
            // i do not like how this is here. also like modularity ig
            // also eventually do one-line display or overwriting so its less spammy on the uart
            // ------------------------------------------------------------------------------------
            if(display_mode == DISP_MODE_TIME){
//                xSemaphoreTake(uart_sem, portMAX_DELAY);
//                Disp2String(time_string);
//                Disp2String("\n\r");
//                xSemaphoreGive(uart_sem);
                msg.cmd = DISP_TIME;
            }
            else if(display_mode == DISP_MODE_FULL){
//                xSemaphoreTake(uart_sem, portMAX_DELAY);
//                Disp2String(time_string);
//                XmitUART2(' ', 1);
//                Disp2String("ADC reading: ");
//                char adc_str[10];
//                int_to_str_dec(raw_adc_result, adc_str);
//                Disp2String(adc_str);
//                Disp2String(" LED2 Intensity: ");
//                char led_intensity_str[10];
//                led2_intensity = led2_variable_duty * 100 / 400;
//                int_to_str_dec(led2_intensity, led_intensity_str);
//                Disp2String(led_intensity_str);
//                Disp2String("%\n\r");
//                xSemaphoreGive(uart_sem);
                msg.cmd = DISP_FULL;
            }
            
//            uint8_t i; 
//            for(i = 0; time_string[i] != '\0'; i++){
//                msg.text[i] = time_string[i];
//            }
//            msg.text[i] = '\0';
//            msg.cmd = DISP_TIME;
            xQueueSend(DisplayQueue, &msg, 0);
                
            if(time_seconds == 0){
                // once the timer has run out, notify the main task, and turn off the countdown
                countdown_running = 0;
                xTaskNotify(main_state_task_handler, (1<<NOTIFY_COUNTDOWN), eSetBits);
            }
        }
        else{
            // if the countdown isn't running, we don't want to get stuck waiting for something to happen in this
            // high priority task, so a short else-block runs to avoid starvation
            vTaskDelay(pdMS_TO_TICKS(10));
            xLastWakeTime = xTaskGetTickCount();
        }
    }
}

void v_display_task(void *pvParametes){
    // gonna use this to control the display im thinking
    
    display_msg_t msg;
    
    for(;;){
        if(xQueueReceive(DisplayQueue, &msg, portMAX_DELAY) == pdTRUE){ // should we use blocking portMAX_DELAY???
            
            xSemaphoreTake(uart_sem, portMAX_DELAY);    // again, portMAX????
            
            switch(msg.cmd){
                case DISP_CLEAR:
                    
                    break;
                case DISP_TIME:
                    Disp2String(msg.time);
                    Disp2String("\n\r");
                    break;
                case DISP_FULL:
                    Disp2String(msg.time);
                    XmitUART2(' ', 1);
                    Disp2String("ADC reading: ");
                    char adc_str[10];
                    int_to_str_dec(msg.adc, adc_str);
                    Disp2String(adc_str);
                    Disp2String(" LED2 Intensity: ");
                    char led_intensity_str[10];
//                    led2_intensity = led2_variable_duty * 100 / 400;
                    int_to_str_dec(msg.intensity, led_intensity_str);
                    Disp2String(led_intensity_str);
                    Disp2String("%\n\r");
                    
                    break;
                case DISP_TEXT:
                    
                    break;
                    
                default:
                    Disp2String("oops\n\r");    // debug again, remove
                    break;
                
            }// end switch
            
            xSemaphoreGive(uart_sem);
        }// end if queue receive
        
        
    }// end for(;;)
}


void v_main_state_task(void *pvParameters){
    // this is the main task controlling the finite state machine running the application
//    Disp2String("hello in main\n\r");
    
    processed_event_t pevt;
    uint32_t ulNotifiedValue = 0;
    for(;;){
        uint8_t got_button = xQueueReceive(ProcessedButtonQueue, &pevt, 0) == pdTRUE;
        uint8_t got_notify = xTaskNotifyWait(0x00, 0xFFFFFFFFUL, &ulNotifiedValue, 0) == pdTRUE;
        
        switch(curr_state){
            case STATE_WAIT:
                // handle waiting behaviour
                // this is LED pulsing
                // click of PB1 to do the transition
                
                pulse = 1;      // set LED2 to pulse according to the timer2
                led2_mode = LED_MODE_PULSE;
                
                if(got_button){
                    // deal with transition out of STATE_WAIT
                    if(pevt.type == EVT_CLICK && (pevt.buttons_mask == PB1_MASK)){
                        // transition occurs when PB1 is clicked in STATE_WAIT
                        // essentially, if the button mask is the PB1 mask and we're in wait, because we're in the click 
                        // part of the button type, this is a pb1 click
                        curr_state = STATE_INPUT;   // transition state
                        time_ready = 0;             // set the time ready flag to 0 (user hasn't inputted time yet)
                        xSemaphoreTake(uart_sem, portMAX_DELAY);    // take semaphore to protect shared UART
                        // display the time entry message
                        Disp2String("Enter a Time: mm:ss");
                        Disp2String("\033[15G");        // escape sequence to place the cursor at the tens place
                                                        // (the 15th character in the display)
                        xSemaphoreGive(uart_sem);   // return the semaphore so others can use UART
                    }
                }// end if(got_button)
                break;
                
            case STATE_INPUT:
                // get user input for the mins/secs 
                // 
                // long press of PB2/PB3 resets input time
                // click of PB2/PB3 proceeds state to begin counting down
                
                pulse = 0;      // turn off the pulsing of LED2
                LED2 = 0;       // and ensure LED2 itself gets turned off
                led2_mode = LED_MODE_OFF;
                
                
                if(got_button){
                    // deal with pb2/pb3 clicks out of STATE_INPUT
                    if(pevt.type == EVT_COMBO && (pevt.buttons_mask == PB2_PB3_MASK)){
                        // if in state input, and the combo event was on PB2 and PB3, 
                        // we want to start the countdown (if the time is ready to go)
                        if(time_ready){
                            // if time is ready (user has input valid time)
                            curr_state = STATE_COUNT;       // transition state to the count state
                            
                            // DEBUG ---------------------------------------------------------
                            Disp2String("yay count!\n\r");
                            //----------------------------------------------------------------
                            
                            countdown_running = 1;  // set the countdown running flag on
                            
                            //----------------------------------------------------------------
                            // check these. aiming to use them to sync the blinking
                            //----------------------------------------------------------------
                            LED1 = 1;               // force LED1 and LED2 to be in the same phase??
                            LED2 = 1;
                            led1_blink_en = 1;      // enable the blinking of LED1
                            led2_blink_phase = 1;   // force the blink phase of LED2 to on
                            
                            led2_mode = LED_MODE_BLINK; // set the mode of the led2 state machine to blink
                        }
                        else if(!time_ready){
                            // if the time is not ready (user clicked before entering all digits or something)
                            curr_state = STATE_INPUT;   // loop back to the input state
                            
                            time_ready = 0;         // set time ready flag to 0
                            time_entry_index = 0;   // reset the time entry index for input collection
                            xSemaphoreTake(uart_sem, portMAX_DELAY);    // take semaphore print message, return semaphore
                            Disp2String("oops, time not ready!\n\r");
                            Disp2String("Enter a Time: mm:ss");
                            Disp2String("\033[15G");        // escape sequence to place the cursor at the tens place
                                                            // (the 15th character in the display)
                            xSemaphoreGive(uart_sem);
                            for(uint8_t j = 0; j < NUM_ENTRY; j++){
                                // loop over the time entry buffer and reset all of the values back to zero
                                time_entry[j] = 0;
                            }
                            xQueueReset(UartRxQueue);       // clears the UART receive queue so no carryover of prev. chars
                        }
                    }
                    // deal w long PB2/PB3 to reset input in STATE_INPUT
                    if(pevt.type == EVT_LONG_COMBO && (pevt.buttons_mask == PB2_PB3_MASK)){
                        // if in input state and the long combo was on PB2 and PB3, we want to reset the input
                        curr_state = STATE_INPUT;   // loop back to state input
                        time_ready = 0;             // set the ready flag to 0, as time is not ready
                        time_entry_index = 0;       // reset the time entry index
                        xSemaphoreTake(uart_sem, portMAX_DELAY);    // take semaphore, print, and return semaphore
                        Disp2String("Enter a Time: mm:ss");
                        Disp2String("\033[15G");        // escape sequence to place the cursor at the tens place
                                                        // (the 15th character in the display)
                        xSemaphoreGive(uart_sem);
                        for(uint8_t j = 0; j < NUM_ENTRY; j++){
                            // loop over the time entry buffer to reset it back to zero
                            time_entry[j] = 0;
                        }
                        xQueueReset(UartRxQueue);   // clear the uart receive queue 
                    }
                }// end if(got_button)
                
                uint8_t recv_byte;
                if(!time_ready){
                    if(xQueueReceive(UartRxQueue, &recv_byte, 10) == pdTRUE){
                        // ^ queue is used to avoid blocking
                        char c = (char)recv_byte;
                        
                        if(c >= '0' && c <= '9'){
                            time_entry[time_entry_index] = c;
                            xSemaphoreTake(uart_sem, portMAX_DELAY);    // check that this is the right spot for this??
                            XmitUART2(c, 1);

                            if(time_entry_index == 1){
                                Disp2String("\033[1C"); // skip to help with alignment
                            }
                            xSemaphoreGive(uart_sem);
                            time_entry_index++;

                            if(time_entry_index >= NUM_ENTRY){
                                time_entry_index = 0;
                                //signal ready to go???
                                time_ready = 1;
                            }
                        }
                        
                        if(time_ready){
                            time_seconds = (time_entry[0] - '0') * 600 
                                         + (time_entry[1] - '0') * 60
                                         + (time_entry[2] - '0') * 10
                                         + (time_entry[3] - '0');                            
                        }
                    }
                }                
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

                
                if(got_button){
                    // deal w the pause/restart of the count
                    if(pevt.type == EVT_CLICK && (pevt.buttons_mask == PB3_MASK)){
                        // if we're in the count state and the processed button mask was the PB3 mask
                        // bc we're in the click part of the button type, it was a pb3 click
                        countdown_running ^= 1; // so toggle whether the countdown is running
                        
                        if(countdown_running == 0){
                            // if the countdown is no longer running, 
                            xSemaphoreTake(uart_sem, portMAX_DELAY);    // take the UART semaphore
                            Disp2String("Countdown Paused\n\r");        // print that the countdown has been paused
                            xSemaphoreGive(uart_sem);                   // return the semaphore!
                            if(xTimerIsTimerActive(led_blink_timer) == pdTRUE){
                                // if the blinking timer is running, we want to stop it, as on countdown pause the 
                                // blinking lights should also pause
                                xTimerStop(led_blink_timer, 0); // so stop timer
                            }
                        }
                        else if(countdown_running == 1){
                            // otherwise if the countdown is now running, 
                            xSemaphoreTake(uart_sem, portMAX_DELAY);    // take the UART semaphore
                            Disp2String("Countdown Resumed\n\r");       // print that the countdown has resumed
                            xSemaphoreGive(uart_sem);                   // give the semaphore back
                            if(xTimerIsTimerActive(led_blink_timer) == pdFALSE){
                                // if the blinking timer is NOT running, we need to start it up again
                                xTimerStart(led_blink_timer, 0);    // so call it to start
                            }
                        }
                    }
                    // to abort the countdown
                    if(pevt.type == EVT_LONG && (pevt.buttons_mask == PB3_MASK)){
                        // if we are in the count state and the long press was on PB3, we want to abort countdown
                        led1_blink_en = 0;          // disable the LED1 blinking
                        led2_mode = LED_MODE_ON;    // turn LED2 to its constant on state
                        LED0 = 1;                   // set LED0 and LED1 to opposite states, to allow for alternating blinking
                        LED1 = 0;
                        xTimerStart(led_alternate_timer, 0);    // start the alternating timer
                        if(xTimerIsTimerActive(end_timer) == pdFALSE){
                            // if the end timer is not currently running, start it 
                            // this is the 5s timeout before returning to the waiting state
                            xTimerStart(end_timer, 0);
                        }
                        
                        //----------------------------------------------------------------
                        // does time_seconds need a semaphore????
                        //----------------------------------------------------------------
                        
                        time_seconds = 0;       // force the countdown amount back to 0 to reset
                        countdown_running = 0;  // stop the counter running

                        xSemaphoreTake(uart_sem, portMAX_DELAY);    // take semaphore, print that countdown aborted, and return sem
                        Disp2String("Countdown Aborted\n\r");
                        xSemaphoreGive(uart_sem);

                        curr_state = STATE_END;     // transition the current state to STATE_END
                    }                    
                }// end if(got_button)
                
                if(got_notify && (ulNotifiedValue & (1<<NOTIFY_COUNTDOWN))){
                    // if the current state is the countdown state,
                    led1_blink_en = 0;          // enable the led1 to blink 
                    led2_mode = LED_MODE_ON;    // force led2 to constant on
                    LED0 = 1;                   // force LEDs 0 and 1 to opposite states so the alternating blink works
                    LED1 = 0;
                    xTimerStart(led_alternate_timer, 0);    // and start the alternating blink timer

                    if(xTimerIsTimerActive(end_timer) == pdFALSE){
                        // if the end timeout timer is not currently running, we start it
                        // this provides the 5s timeout after the countdown completes
                        xTimerStart(end_timer, 0);
                    }

                    xSemaphoreTake(uart_sem, portMAX_DELAY);    // take the semaphore, print completion message, give back sem
                    Disp2String("Countdown Complete\n\r");
                    xSemaphoreGive(uart_sem);

                    curr_state = STATE_END; // transition the state to the end state
                }
                
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
                        
                        if(display_mode == DISP_MODE_TIME) display_mode = DISP_MODE_FULL;
                        else if(display_mode == DISP_MODE_FULL) display_mode = DISP_MODE_TIME;
                    }              
                    xQueueReset(UartRxQueue);       // idk if this is exactly how this should be??
                }                
                break;
                
            case STATE_END:
                // display end message (countdown done)
                // LED0/LED1 rapidly blink alternating
                // LED2 solidly on (still pot for brightness)
                
                // after 5s elapsed, go back to wait
                                
                if(got_notify && (ulNotifiedValue & (1 << NOTIFY_END_TIMEOUT))){
                    // if the notified value was the timeout end notification (after the 5s timeout after countdown)
     
                    // if current state is the end state
                    LED0 = 0;       // turn LEDs 1 and 0 off
                    LED1 = 0;
                    if(xTimerIsTimerActive(led_alternate_timer) == pdTRUE){
                        // if the alternating timer is currently on, we turn it off
                        xTimerStop(led_alternate_timer, portMAX_DELAY);
                    }
                    
                    xSemaphoreTake(uart_sem, portMAX_DELAY);    // take the semaphore, print the wait state message, and return sem
                    Disp2String("Press PB1 to begin!\n\r");
                    xSemaphoreGive(uart_sem);
                    curr_state = STATE_WAIT;    // transition back to the waiting state                   
                }                
                break;
        }// end switch
    }// end for(;;)
}// end main_state_task()


int main(void) {
    
    prvHardwareSetup();
    
    ANSELA = 0x0000; /* keep this line as it sets I/O pins that can also be analog to be digital */
    ANSELB = 0x0000; /* keep this line as it sets I/O pins that can also be analog to be digital */
    
    ANSELB |= 0b1000; // set RB3 (pin 7) to be analog input for ADC potentiometer
    
    // DEBUGS again, take them out!!!
    // ----------------------------------------------------------------
    TRISBbits.TRISB9 = 1;
    TRISBbits.TRISB15 = 0;
    TRISBbits.TRISB14 = 0;
    LED4 = 0;
    LED5 = 0;
    // ----------------------------------------------------------------

    // help w init bounce maybe?
    b_after_debounce[0] = PORTAbits.RA4;
    b_after_debounce[1] = PORTBbits.RB8;
    b_after_debounce[2] = PORTBbits.RB9;
    
    
    IFS1bits.IOCIF = 0;
    IOCFAbits.IOCFA4 = 0;
    IOCFBbits.IOCFB8 = 0;
    IOCFBbits.IOCFB9 = 0;
    IEC1bits.IOCIE = 1; // if not already set, enable IOC interrupt
  
    uart_sem = xSemaphoreCreateMutex();
    led2_sem = xSemaphoreCreateMutex();
    
    button_sem = xSemaphoreCreateBinary();
    
    xSemaphoreGive(button_sem);
    
    ButtonQueueHandle = xQueueCreate(10, sizeof(button_event_t));
    ProcessedButtonQueue = xQueueCreate(10, sizeof(processed_event_t));
    UartRxQueue = xQueueCreate(32, sizeof(uint8_t));    // uint8_t == char so all is well
    DisplayQueue = xQueueCreate(10, sizeof(display_msg_t));

    
    DebounceQueue = xQueueCreate(10, sizeof(uint8_t));
    //xTaskCreate(v_debounce_task, "Debounce Task", configMINIMAL_STACK_SIZE * 2, NULL, 5, NULL);
    
    // CONSIDER if we swap prio of button and countdown, do we fix that overflow stuff?
    xTaskCreate(v_button_task, "Button Task", configMINIMAL_STACK_SIZE * 2, NULL, 3, NULL);
    xTaskCreate(v_main_state_task, "Main State", configMINIMAL_STACK_SIZE * 4, NULL, 2, &main_state_task_handler);
    
    xTaskCreate(v_countdown_task, "Countdown Task", configMINIMAL_STACK_SIZE * 2, NULL, 4, NULL);   // i think this should be highest pri??
    
    xTaskCreate(v_display_task, "Display Task", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
    
    
    static uint8_t timer_ids[NUM_BUTTONS];
//    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
//        timer_ids[i] = i;
//        DebounceTimers[i] = xTimerCreate("Debounce Timer", DEBOUNCE_DELAY, pdFALSE, (void *)(uintptr_t)i, v_timer_callback);
//    }
    
    led_blink_timer = xTimerCreate("Blink Timer", BLINK_RATE, pdTRUE, (void *)0, v_blink_callback_count);
    xTimerStart(led_blink_timer, 0);
    
    led_alternate_timer = xTimerCreate("Alternating Blink Timer", ALT_BLINK_RATE, pdTRUE, (void*)0, v_blink_callback_alternating);
    
    
    end_timer = xTimerCreate("End Timer", END_TIMEOUT, pdFALSE, (void*)0, v_end_callback);
    
    xSemaphoreTake(uart_sem, portMAX_DELAY);
    Disp2String("Press PB1 to begin!\n\r");
    xSemaphoreGive(uart_sem);
    
    // some of this is DEBUG
    LED1 = 0;
    LED0 = 1;
    LED5 = 1;
    LED4 = 0;
    
    
    TMR2 = 0;
    PR2 = PWM_TON;
    T2CONbits.TON = 1;
    
    
    TMR3 = 0;
    PR3 = 200;      //?????? lowkey this is just some number that seems to be fine for now. 
                    // adjust to find a sweet spot
    T3CONbits.TON = 1;
    AD1CON1bits.ADON = 1;

    vTaskStartScheduler();
    
    for(;;);
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
            if(dec){
                // if the decreasing flag is on, we decrease the pwm on time by pulse_step_size to decrease duty
                PWM_TON -= PULSE_STEP_SIZE;
                if(PWM_TON <= PULSE_STEP_SIZE){
                    // now if the next step will wrap the variable, change the decreasing flag to increasing
                    dec = 0;
                }
            }
            if(!dec){
                // if the decreasing flag is off, increase the pwm on time by pulse_step_size to increase duty
                PWM_TON += PULSE_STEP_SIZE;
                if(PWM_TON >= PWM_PER - PULSE_STEP_SIZE){
                    // if the next step will go past the period of the pwm, swap to be increasing
                    dec = 1;
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


void __attribute__ ((interrupt, no_auto_psv)) _IOCInterrupt(void){

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(IOCFAbits.IOCFA4 || IOCFBbits.IOCFB8 || IOCFBbits.IOCFB9)
    xSemaphoreGiveFromISR(button_sem, &xHigherPriorityTaskWoken);
//    IOCFAbits.IOCFA4 = 0;
//    IOCFBbits.IOCFB8 = 0;
//    IOCFBbits.IOCFB9 = 0;
//    // PB1 RA4
//    if(IOCFAbits.IOCFA4){
//        IOCFAbits.IOCFA4 = 0;
//        uint8_t id = 0;
//        xQueueSendFromISR(DebounceQueue, &id, &xHigherPriorityTaskWoken);
//    }
//
//    // PB2 RB8
//    if(IOCFBbits.IOCFB8){
//        IOCFBbits.IOCFB8 = 0;
//        uint8_t id = 1;
//        xQueueSendFromISR(DebounceQueue, &id, &xHigherPriorityTaskWoken);
//    }
    
//
//    // PB3 RB9
//    if(IOCFBbits.IOCFB9){
//        IOCFBbits.IOCFB9 = 0;
//        uint8_t id = 2;
//        xQueueSendFromISR(DebounceQueue, &id, &xHigherPriorityTaskWoken);
//    }
   
    
    IFS1bits.IOCIF = 0;  // clear global IOC flags 
    if(xHigherPriorityTaskWoken) {
        portYIELD();  // triggers context switch
    }
       
}



void __attribute__((interrupt, no_auto_psv)) _ADC1Interrupt(void){
    /*
     * The adc interrupt handles the potentiometer-set variable brightness of LED2. 
     * For this application, we are not polling the done bit, rather relying on the ISR. 
     * The adc is set according to the running of T3, so it runs periodically to allow for responsiveness
     */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // --------------------------------------------------------------------------
    LED4 ^= 1;      // DEBUG remove later
    // --------------------------------------------------------------------------
    
    raw_adc_result = ADC1BUF0;      // store the result in the adc buffer as the raw result
                                    // necessary as raw result is used to display the adc reading on the screen
    
    adc_result = raw_adc_result;    // set the adc result as the raw to help with clipping
    
    // --------------------------------------------------------------------------
    // not entirely sure what happens, but if these aren't clipped it janks and the LED gets stuck at an extreme
    // (though idk if its this clipping or the one with temp that actually fixes it?)
    // there's some sort of overflow problem basically im pretty sure
    // --------------------------------------------------------------------------
    
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