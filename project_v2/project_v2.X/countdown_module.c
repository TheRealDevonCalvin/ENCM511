#include "countdown_module.h"


volatile char time_entry[NUM_ENTRY] = {0,0,0,0};
volatile uint8_t time_ready = 0;
volatile uint8_t time_entry_index = 0;

volatile uint16_t time_seconds = 0;

volatile uint8_t countdown_running = 0;
volatile char time_string[NUM_ENTRY + 2];

TimerHandle_t end_timer;
SemaphoreHandle_t uart_sem;
SemaphoreHandle_t time_sem;

volatile disp_mode_t display_mode = DISP_MODE_TIME;

QueueHandle_t display_queue;



void v_end_callback(TimerHandle_t xTimer){
    // this timer callback runs once the 5 seconds of timeout have elapsed after the completion of the countdown
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // the callback notifies the main state task that the timer has elapsed, and the main state deals with the transition
    xTaskNotifyFromISR(main_state_task_handler, (1<<NOTIFY_END_TIMEOUT), eSetBits, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken){
        portYIELD();
    }
}

void v_countdown_task(void *pvParameters){
    /*
     * This task is responsible for the actual countdown timer of the application. 
     */
    TickType_t xLastWakeTime = xTaskGetTickCount(); // collect last wake time for use in vTaskDelayUntil
    
    display_msg_t msg;  // to be sent to the display queue
    
    for(;;){
        if(countdown_running && time_seconds > 0 && (xSemaphoreTake(time_sem, portMAX_DELAY) == pdTRUE)){
            // if the countdown is running, and there is time left on it,
            // we wait one second (using delayuntil to have better timing)
            xSemaphoreGive(time_sem);
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
            
            xSemaphoreTake(time_sem, portMAX_DELAY);
            time_seconds--; // decrement the time remaining
            
            // calculate the minutes and seconds for display and package into a time string
            uint16_t t = time_seconds;
            uint8_t mm = t / 60;
            uint8_t ss = t % 60;
            xSemaphoreGive(time_sem);
            
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
            // add the adc result and led2 intensity to the struct sent over the display queue
            msg.adc = raw_adc_result;
            msg.intensity = led2_intensity;
            
            // depending on the display mode set by the 'i' char in the state machine, pick how to display in queue
            if(display_mode == DISP_MODE_TIME){
                msg.cmd = DISP_TIME;
            }
            else if(display_mode == DISP_MODE_FULL){
                msg.cmd = DISP_FULL;
            }
            
            xQueueSend(display_queue, &msg, 0); // send the compiled message over the queue
                
            if(time_seconds == 0 && (xSemaphoreTake(time_sem, portMAX_DELAY) == pdTRUE)){
                // once the timer has run out, notify the main task, and turn off the countdown
                xSemaphoreGive(time_sem);
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

void v_display_task(void *pvParameters){
    /*
     * This task is used to update the display while the countdown is in progress.
     */
    
    display_msg_t msg;
    
    for(;;){
        if(xQueueReceive(display_queue, &msg, portMAX_DELAY) == pdTRUE){ 
            // if there's a message in the queue
            
            xSemaphoreTake(uart_sem, portMAX_DELAY);    // take uart semaphore
            
            switch(msg.cmd){
                // switch on the command of the message
                case DISP_TIME:
                    // if only supposed to display the time
                    Disp2String("\033[2J\033[H");   // clear display
                    Disp2String(msg.time);          // display the time
                    Disp2String("\r\n");
                    break;
                case DISP_FULL:
                    // if supposed to display time + ADC + intensity
                    Disp2String("\033[2J\033[H");       // clear display
                    Disp2String(msg.time);              // display time
                    XmitUART2(' ', 1);
                    Disp2String("ADC reading: ");       // display adc reading
                    char adc_str[10];
                    int_to_str_dec(msg.adc, adc_str);
                    Disp2String(adc_str);
                    Disp2String(" LED2 Intensity: ");   // display led intensity
                    char led_intensity_str[10];
                    int_to_str_dec(msg.intensity, led_intensity_str);
                    Disp2String(led_intensity_str);
                    Disp2String("%\r\n");
                    break;
                default:
                    // if something went awry, display an error
                    Disp2String("oops. Issue in display task\r\n");   
                    break;
            }// end switch
            xSemaphoreGive(uart_sem);   // return semaphore
        }// end if queue receive
    }// end for(;;)
}
