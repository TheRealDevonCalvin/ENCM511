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
#pragma config WINDIS = OFF    //Watchdog Timer Window Enable bit->Watchdog Timer in Non-Window mode;
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
#include "timers.h"
#include "semphr.h"
#include <xc.h>

#include "adc.h"
#include "led_module.h"
#include "button_module.h"
#include "countdown_module.h"
#include "uart.h"
#include "init.h"

// an enum type for the state machine
typedef enum{
    STATE_WAIT,
    STATE_INPUT,
    STATE_COUNT, 
    STATE_END
} state_t;
volatile state_t curr_state = STATE_WAIT;   // default the state to wait

TaskHandle_t main_state_task_handler;

void vApplicationIdleHook( void )
{
    Idle();
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
    Disp2String(pcTaskName);
	for( ;; );
}

void prvHardwareSetup(void){
    /* 
     * This function calls the initializer functions required for the application
     */
    InitUART2();
    IOinit();
    timerInit();
    ADC1_init();  
}

void v_main_state_task(void *pvParameters){
    /*
     * This task is the main task controlling the overall state machine. 
     * All state transitions occur here to avoid spaghetti logic, and this makes 
     * potential future state transitions easier to implement. 
     */
    
    // variables for the queue receive and notify receives, and for the uart accepted byte
    processed_event_t pevt;
    uint32_t ulNotifiedValue;
    uint8_t recv_byte;
    for(;;){
        // if there's something to receive in the queue, or a task notified main, set the respective got flag
        uint8_t got_button = xQueueReceive(processed_button_queue, &pevt, 0) == pdTRUE;
        uint8_t got_notify = xTaskNotifyWait(0x00, 0xFFFFFFFFUL, &ulNotifiedValue, 0) == pdTRUE;
        uint8_t got_something = (xQueueReceive(processed_button_queue, &pevt, portMAX_DELAY) == pdTRUE) || (xTaskNotifyWait(0x00, 0xFFFFFFFFUL, &ulNotifiedValue, portMAX_DELAY) == pdTRUE)
                                || (xQueueReceive(uart_rx_queue, &recv_byte, portMAX_DELAY) == pdTRUE);
        
        switch(curr_state){
            // switch over curr_state to run the state machine
            case STATE_WAIT:
                /*
                 * STATE_WAIT handles the waiting behaviour of the application. 
                 * Here, LED2 pulses smoothly from dim to bright and back. 
                 * 
                 * Transition out of this state is caused by a click of PB1
                 */
                
                pulse = 1;      // set LED2 to pulse according to the timer2
                led2_mode = LED_MODE_PULSE;
                
                if(got_something){
                    // deal with transition out of STATE_WAIT
                    if(pevt.type == EVT_CLICK && (pevt.buttons_mask == PB1_MASK)){
                        // transition occurs when PB1 is clicked in STATE_WAIT
                        
                        curr_state = STATE_INPUT;   // transition state
                        time_ready = 0;             // set the time ready flag to 0 (user hasn't inputted time yet)
                        
                        // display the time entry message
                        xSemaphoreTake(uart_sem, portMAX_DELAY);    // take semaphore to protect shared UART
                        Disp2String("\033[2J\033[HEnter a Time: mm:ss");
                        Disp2String("\033[15G");        // escape sequence to place the cursor at the tens place
                                                        // (the 15th character in the display)
                        xSemaphoreGive(uart_sem);   // return the semaphore so others can use UART
                    }
                }// end if(got_button)
                break;
                
            case STATE_INPUT:
                /*
                 * STATE_INPUT handles the state when the app is waiting on the user to enter the time.
                 * A long combo click of PB2/PB3 resets the input time, looping the state back
                 * 
                 * Transition out of this state is cause by a combo click of PB2/PB3 and begins the countdown
                 */
                
                pulse = 0;      // turn off the pulsing of LED2
                LED2 = 0;       // and ensure LED2 itself gets turned off
                led2_mode = LED_MODE_OFF;
                
                if(got_something){
                    // deal with pb2/pb3 clicks out of STATE_INPUT
                    if(pevt.type == EVT_COMBO && (pevt.buttons_mask == PB2_PB3_MASK)){
                        // if in state input, and the combo event was on PB2 and PB3, 
                        // we want to start the countdown (if the time is ready to go)
                        if(time_ready){
                            // if time is ready (user has input valid time)
                            curr_state = STATE_COUNT;       // transition state to the count state
                            
                            // display start message on the screen (using semaphore protection)
                            xSemaphoreTake(uart_sem, portMAX_DELAY);
                            Disp2String("\033[2J\033[HCountdown Started");
                            xSemaphoreGive(uart_sem);
                            
                            countdown_running = 1;  // set the countdown running flag on
                            
                            // cause blinking to begin
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
                            
                            // take semaphore, print message, return semaphore
                            xSemaphoreTake(uart_sem, portMAX_DELAY);    
                            Disp2String("\033[2J\033[Hoops, time not ready!\r\n");
                            Disp2String("Enter a Time: mm:ss");
                            Disp2String("\033[15G");        // escape sequence to place the cursor at the tens place
                                                            // (the 15th character in the display)
                            xSemaphoreGive(uart_sem);
                            
                            for(uint8_t j = 0; j < NUM_ENTRY; j++){
                                // loop over the time entry buffer and reset all of the values back to zero
                                time_entry[j] = 0;
                            }
                            xQueueReset(uart_rx_queue);       // clears the UART receive queue so no carryover of prev. chars
                        }
                    }
                    // deal w long PB2/PB3 to reset input in STATE_INPUT
                    if(pevt.type == EVT_LONG_COMBO && (pevt.buttons_mask == PB2_PB3_MASK)){
                        // if in input state and the long combo was on PB2 and PB3, we want to reset the input
                        curr_state = STATE_INPUT;   // loop back to state input
                        time_ready = 0;             // set the ready flag to 0, as time is not ready
                        time_entry_index = 0;       // reset the time entry index
                        
                        // take semaphore, print message, return semaphore
                        xSemaphoreTake(uart_sem, portMAX_DELAY);    
                        Disp2String("\033[2J\033[HEnter a Time: mm:ss");
                        Disp2String("\033[15G");        // escape sequence to place the cursor at the tens place
                                                        // (the 15th character in the display)
                        xSemaphoreGive(uart_sem);
                        
                        for(uint8_t j = 0; j < NUM_ENTRY; j++){
                            // loop over the time entry buffer to reset it back to zero
                            time_entry[j] = 0;
                        }
                        xQueueReset(uart_rx_queue);   // clear the uart receive queue 
                    }
                }// end if(got_button)
                
                if(!time_ready){
                    // if the time ready flag is not set, we are still accepting chars
                    if(xQueueReceive(uart_rx_queue, &recv_byte, portMAX_DELAY) == pdTRUE){
                        // accept from the queue, sent from the uart rx isr
                        char c = (char)recv_byte;   // save the received byte
                        
                        if(c >= '0' && c <= '9'){
                            // if the char is out of the 0-9 digit range, we ignore it. Only want to accept digits
                            time_entry[time_entry_index] = c;           // place the char at the current index in time_entry
                            
                            // update the display as chars are entered, semaphore for uart protection
                            xSemaphoreTake(uart_sem, portMAX_DELAY); 
                            XmitUART2(c, 1);
                            if(time_entry_index == 1){
                                Disp2String("\033[1C"); // skip to help with alignment
                            }
                            xSemaphoreGive(uart_sem);
                            time_entry_index++;     // increment the time entry index

                            if(time_entry_index >= NUM_ENTRY){
                                // if all places in the entry buffer are filled, set the index to 0 and signal time ready
                                time_entry_index = 0;
                                time_ready = 1;
                            }
                        }
                        
                        if(time_ready){
                            // if time becomes ready, we convert the mm:ss entry format to seconds
                            // this helps make the countdown easier to work with
                            xSemaphoreTake(time_sem, portMAX_DELAY);
                            time_seconds = (time_entry[0] - '0') * 600 
                                         + (time_entry[1] - '0') * 60
                                         + (time_entry[2] - '0') * 10
                                         + (time_entry[3] - '0');   
                            // if greater than 99:59 has been entered, cap it to 99:59
                            // this is because of how the display works, it will not display properly above that
                            if(time_seconds > 5999) time_seconds = 5999;
                            xSemaphoreGive(time_sem);
                        }
                    }
                }                
                break;
                
            case STATE_COUNT:
                /*
                 * STATE_COUNT is responsible for the countdown and associated LED behaviour, as well as
                 * accepting uart input characters, and potentiometer changes.
                 * Clicking PB3 pauses and restarts the timer (and blinking).
                 * Inputting 'b' over uart changes LED2 from blinking to solid.
                 * Inputting 'i' over uart changes display from just time to time + adc + intensity.
                 * Changing the potentiometer changes brightness of LED2.
                 * 
                 * Transition out of the state occurs either on timer elapse, or on a long click 
                 * of PB3 that aborts the timer. 
                 */
                
                if(got_something){
                    // deal w the pause/restart of the count
                    if(pevt.type == EVT_CLICK && (pevt.buttons_mask == PB3_MASK)){
                        // if event was a click, and mask was PB3, a PB3 click has occurred
                        countdown_running ^= 1; // so toggle whether the countdown is running
                        
                        if(countdown_running == 0){
                            // if the countdown is no longer running, 
                            
                            // take semaphore, print pause message, and return semaphore 
                            xSemaphoreTake(uart_sem, portMAX_DELAY);    
                            Disp2String("\033[2J\033[HCountdown Paused\r\n");
                            xSemaphoreGive(uart_sem);                   
                            if(xTimerIsTimerActive(led_blink_timer) == pdTRUE){
                                // if the blinking timer is running, we want to stop it, as on countdown pause the 
                                // blinking lights should also pause
                                xTimerStop(led_blink_timer, 0); // so stop timer
                            }
                        }
                        else if(countdown_running == 1){
                            // otherwise if the countdown is now running, 
                            
                            // take semaphore, print resume message, return semaphore
                            xSemaphoreTake(uart_sem, portMAX_DELAY);   
                            Disp2String("\033[2J\033[HCountdown Resumed\r\n");    
                            xSemaphoreGive(uart_sem);             
                            if(xTimerIsTimerActive(led_blink_timer) == pdFALSE){
                                // if the blinking timer is NOT running, we need to start it up again
                                xTimerStart(led_blink_timer, 0);    // so call it to start
                            }
                        }
                    }
                    // to abort the countdown
                    if(pevt.type == EVT_LONG && (pevt.buttons_mask == PB3_MASK)){
                        // if long press was queued on PB3, we want to abort countdown
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
                        
                        xSemaphoreTake(time_sem, portMAX_DELAY);
                        time_seconds = 0;       // force the countdown amount back to 0 to reset
                        xSemaphoreGive(time_sem);
                        
                        countdown_running = 0;  // stop the counter running

                        // take semaphore, print that countdown aborted, and return sem
                        xSemaphoreTake(uart_sem, portMAX_DELAY);    
                        Disp2String("\033[2J\033[HCountdown Aborted\r\n");
                        xSemaphoreGive(uart_sem);

                        curr_state = STATE_END;     // transition the current state to STATE_END
                    }                    
                }// end if(got_button)
                
                if(got_something && (ulNotifiedValue & (1<<NOTIFY_COUNTDOWN))){
                    // if the current state is the countdown state and main is notified that the countdown has ended
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

                    // take the semaphore, print completion message, give back sem
                    xSemaphoreTake(uart_sem, portMAX_DELAY);    
                    Disp2String("\033[2J\033[HCountdown Complete\r\n");
                    xSemaphoreGive(uart_sem);

                    curr_state = STATE_END; // transition the state to the end state
                }
                
                if(xQueueReceive(uart_rx_queue, &recv_byte, 10) == pdTRUE){
                    // if something was received in the uart queue
                    char c = (char)recv_byte;   // store it
                    if(c == 'b'){
                        // if the char was 'b', we change the mode of LED2 between blinking and solid on
                        // both states are still safe for ADC potentiometer-adjusted intensity
                        if(led2_mode == LED_MODE_ON) led2_mode = LED_MODE_BLINK;
                        else if(led2_mode == LED_MODE_BLINK) led2_mode = LED_MODE_ON;
                    }
                    if(c == 'i'){
                        // if the char was 'i', change the mode of the display between displaying just
                        // time remaining, and time+ADC reading + LED2 intensity
                        if(display_mode == DISP_MODE_TIME) display_mode = DISP_MODE_FULL;
                        else if(display_mode == DISP_MODE_FULL) display_mode = DISP_MODE_TIME;
                    }              
                    // note that if any other character is entered, it is ignored
                    xQueueReset(uart_rx_queue);       // reset the queue
                }                
                break;
                
            case STATE_END:
                /*
                 * STATE_END is responsible for the LED behaviour and timeout after the countdown has finished.
                 * LEDs 0 and 1 blink rapidly in an alternating fashion. 
                 * LED2 remains solidly on, still with variable brightness.
                 * 
                 * Transition out of this state occurs when the 5s timeout has elapsed, and we return to wait. 
                 */
                                
                if(got_something && (ulNotifiedValue & (1 << NOTIFY_END_TIMEOUT))){
                    // if the notified value was the timeout end notification (after the 5s timeout after countdown ends)
     
                    LED0 = 0;       // turn LEDs 1 and 0 off
                    LED1 = 0;
                    if(xTimerIsTimerActive(led_alternate_timer) == pdTRUE){
                        // if the alternating timer is currently on, we turn it off
                        xTimerStop(led_alternate_timer, portMAX_DELAY);
                    }
                    // take the semaphore, print the wait state message, and return sem
                    xSemaphoreTake(uart_sem, portMAX_DELAY);    
                    Disp2String("\033[2J\033[HPress PB1 to begin!\r\n");
                    xSemaphoreGive(uart_sem);
                    
                    curr_state = STATE_WAIT;    // transition back to the waiting state                   
                }                
                break;
        }// end switch
    }// end for(;;)
}// end main_state_task()


int main(void) {
    /*
     * The main function is responsible for the initialization and setup of the tasks, semaphores, 
     * queues, etc. that are used in the application
     */
    prvHardwareSetup();     // calls the initialization functions
    
    ANSELB |= 0b1000; // set RB3 (pin 7) to be analog input for ADC potentiometer
        
    // create the necessary tasks for the application
    xTaskCreate(v_button_task, "Button Task", configMINIMAL_STACK_SIZE * 2, NULL, 3, NULL);
    xTaskCreate(v_main_state_task, "Main State", configMINIMAL_STACK_SIZE * 4, NULL, 2, &main_state_task_handler);
    xTaskCreate(v_countdown_task, "Countdown Task", configMINIMAL_STACK_SIZE * 2, NULL, 4, NULL);   
    xTaskCreate(v_display_task, "Display Task", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
    
    // create the timer for the led blinking. blinks at BLINK_RATE (1s), and auto resets
    led_blink_timer = xTimerCreate("Blink Timer", BLINK_RATE, pdTRUE, (void *)0, v_blink_callback_count);
    xTimerStart(led_blink_timer, 0);    // start the timer
    
    // create the timer for the alternating blinking of PB1/2. blinks at ALT_BLLINK_RATE (200ms), and auto resets
    led_alternate_timer = xTimerCreate("Alternating Blink Timer", ALT_BLINK_RATE, pdTRUE, (void*)0, v_blink_callback_alternating);
    
    // create the timer for the end timeout. runs for END_TIMEOUT (5s), and does not auto reset
    end_timer = xTimerCreate("End Timer", END_TIMEOUT, pdFALSE, (void*)0, v_end_callback);
    
    // create semaphores to protect shared resources
    uart_sem = xSemaphoreCreateMutex();
    led2_sem = xSemaphoreCreateMutex();
    button_sem = xSemaphoreCreateBinary();
    time_sem = xSemaphoreCreateMutex();
    
    // create queues for inter-task communication
    processed_button_queue = xQueueCreate(10, sizeof(processed_event_t));
    uart_rx_queue = xQueueCreate(32, sizeof(uint8_t));    // uint8_t == char so all is well
    display_queue = xQueueCreate(10, sizeof(display_msg_t));
    
    // display startup message
    xSemaphoreTake(uart_sem, portMAX_DELAY);
    Disp2String("\033[2J\033[HPress PB1 to begin!\r\n");
    xSemaphoreGive(uart_sem);
    
    // configure timer2 start and stop counts, and enable, to begin pulsing LED2
    TMR2 = 0;
    PR2 = PWM_TON;
    T2CONbits.TON = 1;
    
    // configure timer3 start and stop counts, and enable it and the adc for variable intensity
    TMR3 = 0;
    PR3 = 200;      
    T3CONbits.TON = 1;
    AD1CON1bits.ADON = 1;   
    
    // start scheduler
    vTaskStartScheduler();
    
    for(;;);
} // end main()

