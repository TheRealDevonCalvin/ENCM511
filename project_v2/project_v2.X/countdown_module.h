/* 
 * File:   countdown_module.h
 * Author: fulbr
 *
 * Created on November 30, 2025, 1:52 PM
 */

#ifndef COUNTDOWN_MODULE_H
#define	COUNTDOWN_MODULE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "xc.h"
    
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "adc.h"
#include "uart.h"
    
#define NUM_ENTRY 4
#define END_TIMEOUT pdMS_TO_TICKS(5000)
    

extern volatile char time_entry[NUM_ENTRY];
extern volatile uint8_t time_ready;
extern volatile uint8_t time_entry_index;

extern volatile uint16_t time_seconds;

extern volatile uint8_t countdown_running;
extern volatile char time_string[NUM_ENTRY + 2];

extern TimerHandle_t end_timer;
extern SemaphoreHandle_t time_sem;
extern SemaphoreHandle_t uart_sem;

typedef enum{
    NOTIFY_COUNTDOWN, 
    NOTIFY_END_TIMEOUT
} main_notify_t;

extern TaskHandle_t main_state_task_handler;


/*-------------- display vars -----------------*/
typedef enum{
    DISP_TIME, 
    DISP_FULL
} display_cmd_t;

typedef struct{
    display_cmd_t cmd;
    char time[10];
    uint16_t adc;
    uint16_t intensity;
    
} display_msg_t;

typedef enum{
    DISP_MODE_TIME,
    DISP_MODE_FULL,
} disp_mode_t;

extern volatile disp_mode_t display_mode;
extern QueueHandle_t display_queue;
/*-------------- END display vars -----------------*/


void v_countdown_task(void *pvParameters);
void v_display_task(void *pvParameters);
void v_end_callback(TimerHandle_t xTimer);


#ifdef	__cplusplus
}
#endif

#endif	/* COUNTDOWN_MODULE_H */

