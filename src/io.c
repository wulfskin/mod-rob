/*! \file io.c
    \brief Interface to the Robotis CM-510 controller I/O peripheries (declaration part, see io.h for an interface description).

 */

#include "io.h"

#include <stddef.h>

/// \private Button interrupt mask definition.
#define IO_INTERRUPT_MASK	0xF3

/// \private Number of available interrupts.
#define IO_INTERRUPT_AMOUNT	8

/// \private Storage for interrupt call back functions.
static volatile io_callback io_interrupt_callback[IO_INTERRUPT_AMOUNT] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

inline void buzzer_init()
{
	// Define buzzer port as output
	DDRB |= BUZZER;
}

inline void led_init()
{
	// Define LED ports as outputs
	DDRC |= LED_ALL;
	// Switch LEDs off
	LED_OFF(LED_ALL);
}

inline uint8_t led_get(uint8_t led)
{
	// Perform one cycle to update port states.
	// This is necessary if the function was called directly after a set command.
	asm("nop");
	// Return inverted result (active-low)
	return ((PINC & led) != led);
}		

inline void mic_init()
{
	// Define microphone port as input
	DDRD &= ~MIC_SIGNAL;
	// Active pull-up resistor (????????????????????????????????)
	PORTD |= MIC_SIGNAL;
}

inline void btn_init()
{
	// Define button ports D and E as inputs
	DDRE &= ~(BTN_UP | BTN_DOWN | BTN_LEFT | BTN_RIGHT);
	DDRD &= ~(BTN_START);
	// Active pull-up resistor
	PORTE |= (BTN_UP | BTN_DOWN | BTN_LEFT | BTN_RIGHT);
	PORTD |= (BTN_START);
	// Set external interrupt control bits for BTN_START and MIC_SIGNAL
	EICRA |= (1 << ISC10) | (1 << ISC00);
	EICRA &= ~((1 << ISC11) | (1 << ISC01));
	// Set external interrupt control bits for BTN_UP/DOWN/LEFT_RIGHT
	EICRB |= (1 << ISC70) | (1 << ISC60) | (1 << ISC50) | (1 << ISC40);
	EICRB &= ~((1 << ISC71) | (1 << ISC61) | (1 << ISC51) | (1 << ISC41));
}

int io_set_interrupt(uint8_t io_port, io_callback callback)
{
	int res = 0;
	uint8_t mask = io_port & IO_INTERRUPT_MASK;
	if (mask)
	{
		if (callback != NULL)
			EIMSK |= mask;
		else
			EIMSK &= ~mask;
	}
	for (uint8_t i = 0; i < IO_INTERRUPT_AMOUNT; i++)
	{
		if (mask & (1 << i))
		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				io_interrupt_callback[i] = callback;
			}				
		}			
	}	
	return res;
}

void io_init()
{
	buzzer_init();
	led_init();
	mic_init();
	btn_init();
}

/// \private Interrupt service routine 0.
ISR(INT0_vect)
{
	io_callback callback = io_interrupt_callback[0];
	if (callback != NULL)
		callback();
}

/// \private Interrupt service routine 1.
ISR(INT1_vect)
{
	io_callback callback = io_interrupt_callback[1];
	if (callback != NULL)
		callback();
}

/// \private Interrupt service routine 4.
ISR(INT4_vect)
{
	io_callback callback = io_interrupt_callback[4];
	if (callback != NULL)
		callback();
}

/// \private Interrupt service routine 5.
ISR(INT5_vect)
{
	io_callback callback = io_interrupt_callback[5];
	if (callback != NULL)
		callback();
}

/// \private Interrupt service routine 6.
ISR(INT6_vect)
{
	io_callback callback = io_interrupt_callback[6];
	if (callback != NULL)
		callback();
}

/// \private Interrupt service routine 7.
ISR(INT7_vect)
{
	io_callback callback = io_interrupt_callback[7];
	if (callback != NULL)
		callback();
}