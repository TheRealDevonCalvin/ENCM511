/* 
 * File:   led_module.h
 * Author: fulbr
 *
 * Created on November 30, 2025, 11:50 AM
 */

#ifndef LED_MODULE_H
#define	LED_MODULE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
    
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
    
#define LED0    LATBbits.LATB5
#define LED1    LATBbits.LATB6
#define LED2    LATBbits.LATB7
    
    
#define BLINK_RATE pdMS_TO_TICKS(1000)    // 1s
#define PULSE_STEP_SIZE 3

#define ALT_BLINK_RATE pdMS_TO_TICKS(200)   // 200ms alternating blink rate
    
extern volatile uint16_t PWM_TON;
extern volatile uint16_t PWM_PER;
extern volatile uint8_t pulse;

extern TimerHandle_t led_blink_timer;
extern TimerHandle_t led_alternate_timer;
    
extern SemaphoreHandle_t led2_sem;

typedef enum{
    LED_MODE_OFF,
    LED_MODE_ON,
    LED_MODE_PULSE,
    LED_MODE_BLINK
} led_mode_t;

extern volatile led_mode_t led2_mode;
extern volatile uint8_t led2_blink_phase;
extern volatile uint8_t led1_blink_en;

extern volatile uint16_t led2_variable_duty;     
extern volatile uint8_t led2_output_en;


void v_blink_callback_count(TimerHandle_t xTimer);
void v_blink_callback_alternating(TimerHandle_t xTimer);

#ifdef	__cplusplus
}
#endif

#endif	/* LED_MODULE_H */

