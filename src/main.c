/*
 * main.c
 *
 * Created: 05.06.2012 20:47:04
 *  Author: Hans-Peter
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <serial.h>
#include "io.h"
#include "timer.h"

void timer1_overflow()
{
	; // Overflow will never occur in mode TOM_CLEAR_ON_COMPARE
}

void timer1_compa()
{
	LED_TOGGLE(LED_ALL);
	if (led_get(LED_AUX))
		printf("AUX is ON.\n");
	else
		printf("AUX is off.\n");
}

void button_up()
{
	if (BTN_UP_PRESSED)
		printf("BUTTON UP PRESSED.\n");
	else
		printf("BUTTON UP RELEASED.\n");
}

void button_down()
{
	if (BTN_DOWN_PRESSED)
	printf("BUTTON DOWN PRESSED.\n");
	else
	printf("BUTTON DOWN RELEASED.\n");
}

void button_left()
{
	if (BTN_LEFT_PRESSED)
	printf("BUTTON LEFT PRESSED.\n");
	else
	printf("BUTTON LEFT RELEASED.\n");
}

void button_right()
{
	if (BTN_RIGHT_PRESSED)
	printf("BUTTON RIGHT PRESSED.\n");
	else
	printf("BUTTON RIGHT RELEASED.\n");
}

void button_start()
{
	if (BTN_START_PRESSED)
	printf("BUTTON START PRESSED.\n");
	else
	printf("BUTTON START RELEASED.\n");
}

void mic_active()
{
	if (MIC_ACTIVE)
	printf("MIC GETS ACTIVE.\n");
	else
	printf("MIC GETS INACTIVE.\n");
}

int main(void)
{
	io_init();
	serial_initialize(57600);
	// Choose timer
	uint8_t timer = 4;
	// Enable interrupts
	sei();
	timer_set_interrupt(timer, TIT_OVERFLOW, &timer1_overflow);
	
	timer_set_interrupt(timer, TIT_OUTPUT_COMPARE_MATCH_A, &timer1_compa);
	timer_set_value(timer, TVT_OUTPUT_COMPARE_A, 15624);
	
	if (timer_init(timer, TOM_CLEAR_ON_COMPARE, TPS_DIV_1028, 0) == TIMER_ERROR_SUCCESS)
	{
		printf("Timer works!\n");
	}
	io_set_interrupt(BTN_UP, &button_up);
	io_set_interrupt(BTN_DOWN, &button_down);
	io_set_interrupt(BTN_LEFT, &button_left);
	io_set_interrupt(BTN_RIGHT, &button_right);
	io_set_interrupt(BTN_START, &button_start);
	io_set_interrupt(MIC_SIGNAL, &mic_active);
	uint8_t t = 0;
    while(1)
    {
        // BUZZ
		BUZZ;
		_delay_ms(100);
    }
	return 0;
}