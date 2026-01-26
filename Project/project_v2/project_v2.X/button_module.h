/* 
 * File:   button_module.h
 * Author: fulbr
 *
 * Created on November 30, 2025, 12:46 PM
 */

#ifndef BUTTON_MODULE_H
#define	BUTTON_MODULE_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    
#include "xc.h"
    
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

    
#define DEBOUNCE_DELAY pdMS_TO_TICKS(50)
#define NUM_BUTTONS 3

#define LONG_PRESS_DUR pdMS_TO_TICKS(750) // can adjust as needed
    
#define PB1     PORTAbits.RA4
#define PB2     PORTBbits.RB8
#define PB3     PORTBbits.RB9   
    
#define PB1_MASK (1 << 0)
#define PB3_MASK (1 << 2) 
#define PB2_PB3_MASK ((1 << 1) | (1 << 2))

    
typedef struct{
    uint8_t mask;
    uint8_t active;
    TickType_t ts;
} button_event_t;

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

extern SemaphoreHandle_t button_sem;
extern QueueHandle_t processed_button_queue;

void v_button_task(void *pvParameters);

#ifdef	__cplusplus
}
#endif

#endif	/* BUTTON_MODULE_H */

