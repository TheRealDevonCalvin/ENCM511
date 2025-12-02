#include "button_module.h"

button_event_t active_press = {0,0,0};
QueueHandle_t processed_button_queue;
SemaphoreHandle_t button_sem;

void v_button_task(void *pvParameters){
    /*
     * This task is responsible for the both the debouncing and the combo and long press detection on the buttons.
     */
    
    // read initial states, and use a mask to store the last stable state (note last stable is 1 if pressed)
    uint8_t pb1 = PORTAbits.RA4;
    uint8_t pb2 = PORTBbits.RB8;
    uint8_t pb3 = PORTBbits.RB9;
    uint8_t last_stable_state = (~((pb3 << 2) | (pb2 << 1) | (pb1 << 0))) & 0b111; 

    for(;;){
        // wait for the button semaphore from the IOC ISR to signal that an edge has occurred
        if(xSemaphoreTake(button_sem, portMAX_DELAY) == pdTRUE){
            vTaskDelay(DEBOUNCE_DELAY); // delay for debounce window
           
            // read debounced states of each button
            pb1 = PORTAbits.RA4;
            pb2 = PORTBbits.RB8;
            pb3 = PORTBbits.RB9;
            
            uint8_t curr_button = (pb3 << 2) | (pb2 << 1) | (pb1 << 0); // mask of button's current logical states
            uint8_t pressed_mask = (~curr_button) & 0b111;  // change that mask to show 1 if pressed
           
            if(pressed_mask != last_stable_state){
                // if the buttons' states have changed since the last event, act. otherwise, don't do anything
                last_stable_state = pressed_mask;   // last stable is updated to the current pressed mask

                if(pressed_mask != 0){
                    // if at least one button is pressed, an event (active press) is occurring
                    // our button detections only occur on the released edge
                    active_press.mask = pressed_mask;       // update the mask in the struct type to the pressed mask
                    active_press.ts = xTaskGetTickCount();  // get the timestamp of when press occurred
                    active_press.active = 1;                // flag that there is an active press
                }
                else if(active_press.active){
                    // button released, so a click or long click has been completed
                    TickType_t dur = xTaskGetTickCount() - active_press.ts; // store duration for long detection
                    
                    processed_event_t pevt;                 // create a processed event type
                    pevt.buttons_mask = active_press.mask;  // update the processed mask to match the active mask
                    pevt.ts = xTaskGetTickCount();          // get timestamp
                    
                    uint8_t num_pressed = 0;
                    for(uint8_t i = 0; i < NUM_BUTTONS; i++){
                        // loop to check how many buttons are pressed/involved in the combo
                        if(active_press.mask & (1<<i)){
                            num_pressed++;
                        }
                    }
                    if(num_pressed == 1){
                        // if only one button involved
                        if(dur >= LONG_PRESS_DUR){
                            // check if it was a long click. if yes, update the processed type
                            pevt.type = EVT_LONG;
                        }
                        else{
                            // if not long, update processed type to simple click
                            pevt.type = EVT_CLICK;
                        }
                    }
                    else{
                        // otherwise, if multiple buttons involved
                        if(dur >= LONG_PRESS_DUR){
                            // if it was a long click, update processed type to long combo
                            pevt.type = EVT_LONG_COMBO;
                        }
                        else{
                            // if not long, update processed type to simple combo
                            pevt.type = EVT_COMBO;
                        }
                    }
                    
                    // send the processed event to the processed queue for use in main
                    xQueueSend(processed_button_queue, &pevt, 0);
                    active_press.active = 0;    // clear the active flag in the active press
                }
            } // end if(pressed != last stable)
        } // end if(semaphore take)
    } // end for(;;)
} // end button task


void __attribute__ ((interrupt, no_auto_psv)) _IOCInterrupt(void){
    /*
     * The IOC interrupt handles detection of button events for all of the three pushbuttons.
     * For this application, the debounce strategy is to give a semaphore in the ISR, which is taken from 
     * a button task that waits for a debouncing period, and then events are handled from there. 
     */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if(IOCFAbits.IOCFA4){
        // if the edge was on pin RA4 (PB1), clear flag and give button semaphore
        IOCFAbits.IOCFA4 = 0;
        xSemaphoreGiveFromISR(button_sem, &xHigherPriorityTaskWoken);
    }
    if(IOCFBbits.IOCFB8){
        // same as above but if edge was on pin RB8 (PB2)
        IOCFBbits.IOCFB8 = 0;
        xSemaphoreGiveFromISR(button_sem, &xHigherPriorityTaskWoken);
    }
    
    if(IOCFBbits.IOCFB9){
        // same as above but if edge was on pin RB9 (PB3)
        IOCFBbits.IOCFB9 = 0;
        xSemaphoreGiveFromISR(button_sem, &xHigherPriorityTaskWoken);
    }
    
    IFS1bits.IOCIF = 0;  // clear global IOC flags 
    if(xHigherPriorityTaskWoken) {
        portYIELD();  // triggers context switch
    }
}