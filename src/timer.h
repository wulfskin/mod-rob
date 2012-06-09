/*
 * timer.h
 *
 * Created: 07.06.2012 11:39:49
 *  Author: Hans-Peter
 */ 

// Supported: Output Capture Unit etc.
// Not supported: Input Capture Unit, PWM modes, toggle of OCnA/B/C pins
// TODO: Check functionality of Timer4 and Timer5

#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>

#define TIMER_ERROR_SUCCESS					0x00
#define TIMER_ERROR_INVALID_OPERATION		0x01
#define TIMER_ERROR_INVALID_COUNTER			0x02

typedef enum {
	TPS_DISABLED = 0x00,
	TPS_NONE = 0x11,
	TPS_DIV_8 = 0x22,
	TPS_DIV_32 = 0x30,
	TPS_DIV_64 = 0x43,
	TPS_DIV_128 = 0x50,
	TPS_DIV_256 = 0x64,
	TPS_DIV_1028 = 0x75
	// external clock not supported yet
} timer_prescaler;

typedef enum {
	TOM_NORMAL = 0x00,
	TOM_CLEAR_ON_COMPARE = 0x42
	// more modes are not supported at the moment
} timer_operation_mode;

typedef enum {
	TIT_OVERFLOW = 0, // TOIEn
	TIT_OUTPUT_COMPARE_MATCH_A = 1, // OCIEnA
	TIT_OUTPUT_COMPARE_MATCH_B = 2, // OCIEnB
	TIT_OUTPUT_COMPARE_MATCH_C = 3 // OCIEnC
	//titInputCapture = 5 // ICIEn
} timer_interrupt_types;

typedef enum {
	TVT_COUNTER_VALUE,
	TVT_OUTPUT_COMPARE_A,
	TVT_OUTPUT_COMPARE_B,
	TVT_OUTPUT_COMPARE_C
} timer_value_type;

int timer_set_prescaler(uint8_t timer, timer_prescaler prescaler);

int timer_disable(uint8_t timer);

int timer_set_mode(uint8_t timer, timer_operation_mode timer_mode);

int timer_get_act_value(uint8_t timer, timer_value_type type, uint16_t * act_value);

int timer_set_value(uint8_t timer, timer_value_type type, uint16_t new_value);

int timer_set(uint8_t timer, uint16_t new_value);

int timer_get(uint8_t timer, uint16_t * act_value);

int timer_reset(uint8_t timer);

int timer_init(uint8_t timer, timer_operation_mode mode, timer_prescaler prescaler, uint16_t preset);

#ifndef TIMER_ENABLE_SIMPLE_INTERRUPTS

typedef void (*timer_callback)(void);

int timer_set_interrupt(uint8_t timer, timer_interrupt_types interrupt_type, timer_callback callback);

#endif

#endif /* __TIMER_H */