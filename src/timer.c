/*! \file timer.c
    \brief Timer/Counter abstraction layer for ATmega2561 microcontroller (declaration part, see timer.h for an interface description).
 */

#include "timer.h"

#include <stddef.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

/// \private Macro to mask the normal prescaler.
#define TIMER_MASK_PRESCALER_NORMAL(PRESCALER)		((PRESCALER) & 0x0F)
/// \private Macro to mask the fine prescaler for timer2.
#define TIMER_MASK_PRESCALER_FINE(PRESCALER)		(((PRESCALER) & 0xF0) >> 4)

/// \private Timer prescaler bitmask.
#define TIMER_PRESCALER_MASK		0x07
/// \private Macro to set timer prescaler register.
#define TIMER_SET_PRESCALER(REGISTER, PRESCALER)		((REGISTER) = ((REGISTER) & ~TIMER_PRESCALER_MASK) + (PRESCALER))

/// \private Timer mode mask for timer 0.
#define TIMER_MASK_MODE_0(MODE)		((MODE) & 0x0F)
/// \private Timer mode mask for timer 1 and similiar.
#define TIMER_MASK_MODE_1(MODE)		(((MODE) & 0xF0) >> 4)
/// \private Timer mode mask for timer 2.
#define TIMER_MASK_MODE_2(MODE)		TIMER_MASK_MODE_0(MODE)

/// \private Timer mode mask for part A.
#define TIMER_MODE_MASK_A				0x03
/// \private Timer mode mask for part B.
#define TIMER_MODE_MASK_B				0x0C

/// \private Timer mode mask for timer0 control register A.
#define TIMER_MODE_REGISTER_MASK_0A		0x03
/// \private Timer mode mask for timer0 control register B.
#define TIMER_MODE_REGISTER_MASK_0B		0x08

/// \private Timer mode mask for timer1 control register A similar.
#define TIMER_MODE_REGISTER_MASK_1A		0x03
/// \private Timer mode mask for timer1 control register A and similar.
#define TIMER_MODE_REGISTER_MASK_1B		0x18


#ifndef TIMER_ENABLE_SIMPLE_INTERRUPTS
/// \private Interrupt mask for timer0.
#define TIMER_INTERRUPT_MASK_0		(0x07)
/// \private Interrupt mask for timer1 and similar.
#define TIMER_INTERRUPT_MASK_1		(0x2F)
/// \private Interrupt mask for timer2.
#define TIMER_INTERRUPT_MASK_2		TIMER_INTERRUPT_MASK_0

/// \private Amount of counters supported by hardware.
#define TIMER_AMOUNT 6
/// \private Amount of interrupts supported by each timer (5) and software (4).
#define TIMER_INTERRUPT_TYPES 4

/// \private Global variable to store the timer interrupt callback functions.
static volatile timer_callback timer_interrupt_callback[TIMER_AMOUNT][TIMER_INTERRUPT_TYPES] = { {NULL, NULL, NULL, NULL},
																								 {NULL, NULL, NULL, NULL},
																								 {NULL, NULL, NULL, NULL},
																								 {NULL, NULL, NULL, NULL},
																								 {NULL, NULL, NULL, NULL},
																								 {NULL, NULL, NULL, NULL}, 
																							   };
#endif

int timer_set_prescaler(const uint8_t timer, const timer_prescaler prescaler)
{
	uint8_t scaler = 0;
	// Calculate prescaler bit-combination depending on available prescalers
	switch (timer)
	{
		case 0:
		case 1:
		case 3:
		case 4:
		case 5:
			scaler = TIMER_MASK_PRESCALER_NORMAL(prescaler);
		break;
			
		case 2:
			scaler = TIMER_MASK_PRESCALER_FINE(prescaler);
		break;
		
		default:
			return TIMER_ERROR_INVALID_TIMER;
	}
	// Check if prescaler combination is valid
	if (scaler || prescaler == TPS_DISABLED)
	{
		// Set new prescaler value
		switch (timer)
		{
			case 0:
				TIMER_SET_PRESCALER(TCCR0B, scaler);
			break;
			
			case 1:
				TIMER_SET_PRESCALER(TCCR1B, scaler);
			break;
			
			case 2:
				TIMER_SET_PRESCALER(TCCR2B, scaler);
			break;
			
			case 3:
				TIMER_SET_PRESCALER(TCCR3B, scaler);
			break;
			
			case 4:
				TIMER_SET_PRESCALER(TCCR4B, scaler);
			break;
			
			case 5:
				TIMER_SET_PRESCALER(TCCR5B, scaler);
			break;
		}
		return TIMER_ERROR_SUCCESS;
	}
	else
		return TIMER_ERROR_INVALID_OPERATION;
}

int timer_disable(const uint8_t timer)
{
	// Disabling the prescaler clock is disabling the whole timer
	return timer_set_prescaler(timer, TPS_DISABLED);
}

/// \private Internal function to set mode register on timer0.
void timer0_set_mode(const uint8_t mode, volatile uint8_t * control_register1, volatile uint8_t * control_register2)
{
	*control_register1 = (*control_register1 & ~TIMER_MODE_REGISTER_MASK_0A) + (mode & TIMER_MODE_MASK_A);
	*control_register2 = (*control_register2 & ~TIMER_MODE_REGISTER_MASK_0B) + ((mode & TIMER_MODE_MASK_B) << 1);	
}

/// \private Internal function to set mode register on timer1.
void timer1_set_mode(const uint8_t mode, volatile uint8_t * control_register1, volatile uint8_t * control_register2)
{
	*control_register1 = (*control_register1 & ~TIMER_MODE_REGISTER_MASK_1A) + (mode & TIMER_MODE_MASK_A);
	*control_register2 = (*control_register2 & ~TIMER_MODE_REGISTER_MASK_1B) + ((mode & TIMER_MODE_MASK_B) << 1);
}	

/// \private Internal function to set mode register on timer2.
void timer2_set_mode(const uint8_t mode, volatile uint8_t * control_register1, volatile uint8_t * control_register2)
{
	timer0_set_mode(mode, control_register1, control_register2);
}

int timer_set_mode(const uint8_t timer, const timer_operation_mode timer_mode)
{
	uint8_t mode = 0;
	switch (timer)
	{
		case 0:
			mode = TIMER_MASK_MODE_0(timer_mode);
		break;
			
		case 1:
		case 3:
		case 4:
		case 5:
			mode = TIMER_MASK_MODE_1(timer_mode);
		break;
			
		case 2:
			mode = TIMER_MASK_MODE_2(timer_mode);
		break;
			
		default:
			return TIMER_ERROR_INVALID_TIMER;
	}
	if (mode != 0 || timer_mode == TOM_NORMAL)
	{
		switch (timer)
		{
			case 0:
				//TIMER_SET_MODE_0(TCCR0A, TCCR0B, mode);
				timer0_set_mode(mode, &TCCR0A, &TCCR0B);
			break;
			
			case 1:
				timer1_set_mode(mode, &TCCR1A, &TCCR1B);
			break;
				
			case 2:
				timer2_set_mode(mode, &TCCR2A, &TCCR2B);
			break;
			
			case 3:
				timer1_set_mode(mode, &TCCR3A, &TCCR3B);
			break;
				
			case 4:
				timer1_set_mode(mode, &TCCR4A, &TCCR4B);
			break;
			
			case 5:
				timer1_set_mode(mode, &TCCR5A, &TCCR5B);
			break;
		}
		// Return success
		return TIMER_ERROR_SUCCESS;
	}
	else
	return TIMER_ERROR_INVALID_OPERATION;		
}

int timer_get_act_value(const uint8_t timer, const timer_value_type type, uint16_t * act_value)
{
	// Check  for valid pointer
	if (act_value == NULL)
		return TIMER_ERROR_INVALID_OPERATION;
		
	// Do operation depending on timer
	switch (timer)
	{
		case 0:
			switch (type)
			{
				case TVT_COUNTER_VALUE:
					*act_value = TCNT0;
					return TIMER_ERROR_SUCCESS;
					
				case TVT_OUTPUT_COMPARE_A:
					*act_value = OCR0A;
					return TIMER_ERROR_SUCCESS;
					
				case TVT_OUTPUT_COMPARE_B:
					*act_value = OCR0B;
					return TIMER_ERROR_SUCCESS;
					
				default:
					return TIMER_ERROR_INVALID_OPERATION;
			}
			
		case 1:
			switch (type)
			{
				case TVT_COUNTER_VALUE:
					*act_value = TCNT1;
					return TIMER_ERROR_SUCCESS;
					
				case TVT_OUTPUT_COMPARE_A:
					*act_value = OCR1A;
					return TIMER_ERROR_SUCCESS;
					
				case TVT_OUTPUT_COMPARE_B:
					*act_value = OCR1B;
					return TIMER_ERROR_SUCCESS;
				
				case TVT_OUTPUT_COMPARE_C:
					*act_value = OCR1C;
					return TIMER_ERROR_SUCCESS;
					
				default:
					return TIMER_ERROR_INVALID_OPERATION;
			}					
		
		case 2:
			switch (type)
			{
				case TVT_COUNTER_VALUE:
					*act_value = TCNT2;
					return TIMER_ERROR_SUCCESS;
					
				case TVT_OUTPUT_COMPARE_A:
					*act_value = OCR2A;
					return TIMER_ERROR_SUCCESS;
					
				case TVT_OUTPUT_COMPARE_B:
					*act_value = OCR2B;
					return TIMER_ERROR_SUCCESS;
					
				default:
					return TIMER_ERROR_INVALID_OPERATION;
			}					
			
		case 3:
			switch (type)
			{
				case TVT_COUNTER_VALUE:
					*act_value = TCNT3;
					return TIMER_ERROR_SUCCESS;
				
				case TVT_OUTPUT_COMPARE_A:
					*act_value = OCR3A;
					return TIMER_ERROR_SUCCESS;
					
				case TVT_OUTPUT_COMPARE_B:
					*act_value = OCR3B;
					return TIMER_ERROR_SUCCESS;

				case TVT_OUTPUT_COMPARE_C:
					*act_value = OCR3C;
					return TIMER_ERROR_SUCCESS;
				
				default:
					return TIMER_ERROR_INVALID_OPERATION;
			}
		
		case 4:
			switch (type)
			{
				case TVT_COUNTER_VALUE:
					*act_value = TCNT4;
					return TIMER_ERROR_SUCCESS;
				
				case TVT_OUTPUT_COMPARE_A:
					*act_value = OCR4A;
					return TIMER_ERROR_SUCCESS;
				
				case TVT_OUTPUT_COMPARE_B:
					*act_value = OCR4B;
					return TIMER_ERROR_SUCCESS;

				case TVT_OUTPUT_COMPARE_C:
					*act_value = OCR4C;
					return TIMER_ERROR_SUCCESS;
				
				default:
					return TIMER_ERROR_INVALID_OPERATION;
			}
		
		default:
			return TIMER_ERROR_INVALID_TIMER;
	}	
}

int timer_set_value(const uint8_t timer, const timer_value_type type, const uint16_t new_value)
{
	// Do operation depending on timer
	switch (timer)
	{
		case 0:
			switch (type)
			{
				case TVT_COUNTER_VALUE:
					TCNT0 = (uint8_t) new_value;
					return TIMER_ERROR_SUCCESS;
			
				case TVT_OUTPUT_COMPARE_A:
					OCR0A = (uint8_t) new_value;
					return TIMER_ERROR_SUCCESS;
			
				case TVT_OUTPUT_COMPARE_B:
					OCR0B = (uint8_t) new_value;
					return TIMER_ERROR_SUCCESS;
			
				default:
					return TIMER_ERROR_INVALID_OPERATION;
			}
		
		case 1:
			switch (type)
			{
				case TVT_COUNTER_VALUE:
					TCNT1 = new_value;
					return TIMER_ERROR_SUCCESS;
				
				case TVT_OUTPUT_COMPARE_A:
					OCR1A = new_value;
					return TIMER_ERROR_SUCCESS;
				
				case TVT_OUTPUT_COMPARE_B:
					OCR1B = new_value;
					return TIMER_ERROR_SUCCESS;
				
				case TVT_OUTPUT_COMPARE_C:
					OCR1C = new_value;
					return TIMER_ERROR_SUCCESS;
				
				default:
					return TIMER_ERROR_INVALID_OPERATION;
			}
		
		case 2:
			switch (type)
			{
				case TVT_COUNTER_VALUE:
					TCNT2 = (uint8_t) new_value;
					return TIMER_ERROR_SUCCESS;
			
				case TVT_OUTPUT_COMPARE_A:
					OCR2A = (uint8_t) new_value;
					return TIMER_ERROR_SUCCESS;
			
				case TVT_OUTPUT_COMPARE_B:
					OCR2B = (uint8_t) new_value;
					return TIMER_ERROR_SUCCESS;
			
				default:
					return TIMER_ERROR_INVALID_OPERATION;
			}
		
		case 3:
			switch (type)
			{
				case TVT_COUNTER_VALUE:
					TCNT3 = new_value;
					return TIMER_ERROR_SUCCESS;
			
				case TVT_OUTPUT_COMPARE_A:
					OCR3A = new_value;
					return TIMER_ERROR_SUCCESS;
			
				case TVT_OUTPUT_COMPARE_B:
					OCR3B = new_value;
					return TIMER_ERROR_SUCCESS;

				case TVT_OUTPUT_COMPARE_C:
					OCR3C = new_value;
					return TIMER_ERROR_SUCCESS;
			
				default:
					return TIMER_ERROR_INVALID_OPERATION;
			}
		
		case 4:
			switch (type)
			{
				case TVT_COUNTER_VALUE:
					TCNT4 = new_value;
					return TIMER_ERROR_SUCCESS;
			
				case TVT_OUTPUT_COMPARE_A:
					OCR4A = new_value;
					return TIMER_ERROR_SUCCESS;
			
				case TVT_OUTPUT_COMPARE_B:
					OCR4B = new_value;
					return TIMER_ERROR_SUCCESS;

				case TVT_OUTPUT_COMPARE_C:
					OCR4C = new_value;
					return TIMER_ERROR_SUCCESS;
			
				default:
					return TIMER_ERROR_INVALID_OPERATION;
			}
		
		default:
			return TIMER_ERROR_INVALID_TIMER;
	}
}

int timer_set(const uint8_t timer, const uint16_t new_value)
{
	return timer_set_value(timer, TVT_COUNTER_VALUE, new_value);	
}

int timer_get(const uint8_t timer, uint16_t * act_value)
{
	return timer_get_act_value(timer, TVT_COUNTER_VALUE, act_value);
}

int timer_reset(const uint8_t timer)
{
	return timer_set(timer, 0);	
}

int timer_init(const uint8_t timer, const timer_operation_mode mode, const timer_prescaler prescaler, const uint16_t preset)
{
	// Initialize result
	int res = TIMER_ERROR_SUCCESS;
	// Disable timer first
	res = timer_disable(timer);
	if (res == TIMER_ERROR_SUCCESS)
	{
		// Set timer operation mode
		res = timer_set_mode(timer, mode);
		if (res == TIMER_ERROR_SUCCESS)
		{
			// Set preset value
			res = timer_set(timer, preset);
			if (res == TIMER_ERROR_SUCCESS)
			{
				// Set prescaler value and enable timer
				res = timer_set_prescaler(timer, prescaler);	
			}
		}
	}
	return res;
}

#ifndef TIMER_ENABLE_SIMPLE_INTERRUPTS
int timer_set_interrupt(const uint8_t timer, const timer_interrupt_types interrupt_type, const timer_callback callback)
{
	uint8_t interrupt = 0;
	switch (timer)
	{
		case 0:
		case 2:
		case 3:
		case 4:
		case 5:
			interrupt = (1 << interrupt_type) & TIMER_INTERRUPT_MASK_0;
		break;
		
		case 1:
			interrupt = (1 << interrupt_type) & TIMER_INTERRUPT_MASK_1;
		break;
		
		default:
			return TIMER_ERROR_INVALID_TIMER;
	}
	if (interrupt)
	{
		switch (timer)
		{
			case 0:
				if (callback != NULL)
					TIMSK0 |= interrupt;
				else
				TIMSK0 &= ~interrupt;
			break;
			
			case 1:
				if (callback != NULL)
					TIMSK1 |= interrupt;
				else
					TIMSK1 &= ~interrupt;
			break;
			
			case 2:
				if (callback != NULL)
					TIMSK2 |= interrupt;
				else
					TIMSK2 &= ~interrupt;
			break;
			
			case 3:
				if (callback != NULL)
					TIMSK3 |= interrupt;
				else
					TIMSK3 &= ~interrupt;
			break;
			
			case 4:
				if (callback != NULL)
					TIMSK4 |= interrupt;
				else
					TIMSK4 &= ~interrupt;
			break;
			
			case 5:
				if (callback != NULL)
					TIMSK5 |= interrupt;
				else
					TIMSK5 &= ~ interrupt;
			break;
		}
		// Assign callback function for execution in case of interrupt
		// Important: This code must be atomic to avoid race conditions with the calling ISR.
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			// Assign new callback function
			timer_interrupt_callback[timer][interrupt_type] = callback;
		}
		// Operation completed successfully
		return TIMER_ERROR_SUCCESS;
	}
	else
		return TIMER_ERROR_INVALID_OPERATION;
}

/// \private Timer0 overflow interrupt service routine.
ISR (TIMER0_OVF_vect)
{
	timer_callback callback = timer_interrupt_callback[0][TIT_OVERFLOW];
	if (callback != NULL)
		callback();		
}

/// \private Timer0 output compare A interrupt service routine.
ISR (TIMER0_COMPA_vect)
{
	timer_callback callback = timer_interrupt_callback[0][TIT_OUTPUT_COMPARE_MATCH_A];
	if (callback != NULL)
		callback();
}

/// \private Timer0 output compare B interrupt service routine.
ISR (TIMER0_COMPB_vect)
{
	timer_callback callback = timer_interrupt_callback[0][TIT_OUTPUT_COMPARE_MATCH_A];
	if (callback != NULL)
		callback();
}

/// \private Timer1 overflow interrupt service routine.
ISR (TIMER1_OVF_vect)
{
	timer_callback callback = timer_interrupt_callback[1][TIT_OVERFLOW];
	if (callback != NULL)
		callback();
}

/// \private Timer1 output compare A interrupt service routine.
ISR (TIMER1_COMPA_vect)
{
	timer_callback callback = timer_interrupt_callback[1][TIT_OUTPUT_COMPARE_MATCH_A];
	if (callback != NULL)
		callback();
}

/// \private Timer1 output compare B interrupt service routine.
ISR (TIMER1_COMPB_vect)
{
	timer_callback callback = timer_interrupt_callback[1][TIT_OUTPUT_COMPARE_MATCH_B];
	if (callback != NULL)
		callback();
}

/// \private Timer1 output compare C interrupt service routine.
ISR (TIMER1_COMPC_vect)
{
	timer_callback callback = timer_interrupt_callback[1][TIT_OUTPUT_COMPARE_MATCH_C];
	if (callback != NULL)
		callback();
}

/// \private Timer2 overflow interrupt service routine.
ISR (TIMER2_OVF_vect)
{
	timer_callback callback = timer_interrupt_callback[2][TIT_OVERFLOW];
	if (callback != NULL)
		callback();
}

/// \private Timer2 output compare A interrupt service routine.
ISR (TIMER2_COMPA_vect)
{
	timer_callback callback = timer_interrupt_callback[2][TIT_OUTPUT_COMPARE_MATCH_A];
	if (callback != NULL)
		callback();
}

/// \private Timer2 output compare B interrupt service routine.
ISR (TIMER2_COMPB_vect)
{
	timer_callback callback = timer_interrupt_callback[2][TIT_OUTPUT_COMPARE_MATCH_A];
	if (callback != NULL)
		callback();
}

/// \private Timer3 overflow interrupt service routine.
ISR (TIMER3_OVF_vect)
{
	timer_callback callback = timer_interrupt_callback[3][TIT_OVERFLOW];
	if (callback != NULL)
		callback();
}

/// \private Timer3 output compare A interrupt service routine.
ISR (TIMER3_COMPA_vect)
{
	timer_callback callback = timer_interrupt_callback[3][TIT_OUTPUT_COMPARE_MATCH_A];
	if (callback != NULL)
		callback();
}

/// \private Timer3 output compare B interrupt service routine.
ISR (TIMER3_COMPB_vect)
{
	timer_callback callback = timer_interrupt_callback[3][TIT_OUTPUT_COMPARE_MATCH_B];
	if (callback != NULL)
		callback();
}

/// \private Timer3 output compare C interrupt service routine.
ISR (TIMER3_COMPC_vect)
{
	timer_callback callback = timer_interrupt_callback[3][TIT_OUTPUT_COMPARE_MATCH_C];
	if (callback != NULL)
		callback();
}

/// \private Timer4 overflow interrupt service routine.
ISR (TIMER4_OVF_vect)
{
	timer_callback callback = timer_interrupt_callback[4][TIT_OVERFLOW];
	if (callback != NULL)
		callback();
}

/// \private Timer4 output compare A interrupt service routine.
ISR (TIMER4_COMPA_vect)
{
	timer_callback callback = timer_interrupt_callback[4][TIT_OUTPUT_COMPARE_MATCH_A];
	if (callback != NULL)
		callback();
}

/// \private Timer4 output compare B interrupt service routine.
ISR (TIMER4_COMPB_vect)
{
	timer_callback callback = timer_interrupt_callback[4][TIT_OUTPUT_COMPARE_MATCH_B];
	if (callback != NULL)
		callback();
}

/// \private Timer4 output compare C interrupt service routine.
ISR (TIMER4_COMPC_vect)
{
	timer_callback callback = timer_interrupt_callback[4][TIT_OUTPUT_COMPARE_MATCH_C];
	if (callback != NULL)
		callback();
}

/// \private Timer5 overflow interrupt service routine.
ISR (TIMER5_OVF_vect)
{
	timer_callback callback = timer_interrupt_callback[5][TIT_OVERFLOW];
	if (callback != NULL)
		callback();
}

/// \private Timer5 output compare A interrupt service routine.
ISR (TIMER5_COMPA_vect)
{
	timer_callback callback = timer_interrupt_callback[5][TIT_OUTPUT_COMPARE_MATCH_A];
	if (callback != NULL)
		callback();
}

/// \private Timer5 output compare B interrupt service routine.
ISR (TIMER5_COMPB_vect)
{
	timer_callback callback = timer_interrupt_callback[5][TIT_OUTPUT_COMPARE_MATCH_B];
	if (callback != NULL)
		callback();
}

/// \private Timer5 output compare C interrupt service routine.
ISR (TIMER5_COMPC_vect)
{
	timer_callback callback = timer_interrupt_callback[5][TIT_OUTPUT_COMPARE_MATCH_C];
	if (callback != NULL)
		callback();
}
#endif /* TIMER_ENABLE_SIMPLE_INTERRUPTS */