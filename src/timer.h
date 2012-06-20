/*! \file timer.h
    \brief Timer/Counter abstraction layer for ATmega2561 micro controller.
	\author Hans-Peter Wolf
	\copyright GNU Public License V3
	\date 2012

	\file timer.h
	\details This file provides easy-to-use access to the timers/counters of the ATmega2561 micro controller. For convenience in the
	following only the term _timer(s)_ is used for both of them.
	
	The ATmega2561 micro controller provides 6 timers with different functions. For the sake of simplicity
	only a (necessary) subpart of the functions which are available hardware wise are supported by this interface.
	
	The micro controller basically provides two different kind of timers: Two general purpose 8bit timers 0 and 2
	with slightly different features and four high-precision 16bit timers 1, 3, 4 and 5 with a wide range of features.
	
	The following table provides a short overview of the software-wise supported and
	<span style="text-decoration:line-through;">non-supported</span> features of each timer:
	<a name="table">
	<table>
		<tr>
			<td>&nbsp;</td>
			<td><b>Timer0</b></td>
			<td><b>Timer1</b></td>
			<td><b>Timer2</b></td>
			<td><b>Timer3</b></td>
			<td><b>Timer4</b></td>
			<td><b>Timer5</b></td>
		</td>
		<tr>
			<td><b>Resolution:</b></td>
			<td>8 bit</td>
			<td>16 bit</td>
			<td>8 bit</td>
			<td colspan="3" align="center">16 bit</td>
		</tr>
		<tr>
			<td><b>Modes:</b></td>
			<td colspan="6" align="center">
				Normal mode (infinitely up-counting with overflow at maximum value)<br>
				Clear Timer On Compare Match (CTC) mode (infinitely up-counting and clearance when output compare value A is reached)<br>
				<span style="text-decoration:line-through;">Fast PWM mode</span><br>
				<span style="text-decoration:line-through;">Phase Correct PWM mode</span>
			</td>
		</tr>
		<tr valign="top">
			<td><b>Interrupts:</b></td>
			<td>Overflow<br>Output Compare Match A<br>Output Compare Match B</td>
			<td>
				Overflow<br>Output Compare Match A<br>Output Compare Match B<br>Output Compare Match C<br>
				<span style="text-decoration:line-through;">Input Capture Interrupt</span>
			</td>
			<td>
				Overflow<br>Output Compare Match A<br>Output Compare Match B
			</td>
			<td colspan="3" align="center">
				Overflow<br>Output Compare Match A<br>Output Compare Match B<br>Output Compare Match C<br>
				<span style="text-decoration:line-through;">Input Capture Interrupt</span>
			</td>
<!--
			<td>
				Overflow<br>Output Compare Match A<br>Output Compare Match B<br>Output Compare Match C<br>
				<span style="text-decoration:line-through;">Input Capture Interrupt</span>
			</td>
			<td>
				Overflow<br>Output Compare Match A<br>Output Compare Match B<br>Output Compare Match C<br>
				<span style="text-decoration:line-through;">Input Capture Interrupt</span>
			</td>
-->
		</tr>
		<tr valign="top">
			<td><b>Prescaler:</b></td>
			<td>
				Clock frequency<br>Clock frequency / 8<br>Clock frequency / 64<br>Clock frequency / 256<br>
				Clock frequency / 1024<br><span style="text-decoration:line-through;">External source</span>
			</td>
			<td>
				Clock frequency<br>Clock frequency / 8<br>Clock frequency / 64<br>Clock frequency / 256<br>
				Clock frequency / 1024<br><span style="text-decoration:line-through;">External source</span>
			</td>
			<td>
				Clock frequency<br>Clock frequency / 8<br>Clock frequency / 32<br>Clock frequency / 64<br>
				Clock frequency / 128<br>Clock frequency / 256<br>Clock frequency / 1024
			</td>
			<td colspan="3" align="center">
				Clock frequency<br>Clock frequency / 8<br>Clock frequency / 64<br>Clock frequency / 256<br>
				Clock frequency / 1024<br><span style="text-decoration:line-through;">External source</span>
			</td>
		</tr>
		<tr>
			<td>Other:</td>
			<td>-</td>
			<td><span style="text-decoration:line-through;">External Event Counter<br>Input Capture Noise Canceler</span></td>
			<td><span style="text-decoration:line-through;">External 32kHz Clock</span>
			<td colspan="3" align="center">
				<span style="text-decoration:line-through;">External Event Counter<br>Input Capture Noise Canceler</span>
			</td>
		</tr>
	</table>
	</a>
	
	\par Example:
	The following example provides a short overview of the capabilities of this interface. It sets up timer0
	in Clear Timer on Compare Match (CTC) mode (#TOM_CLEAR_ON_COMPARE) with a prescaler of
	_clock frequency / 64_ (#TPS_DIV_64). In order to achieve an interrupt at a final frequency of 1kHz the
	compare match register A is set accordingly and an interrupt on compare match A (#TIT_OUTPUT_COMPARE_MATCH_A) is assigned.
\code
void timer0_compA()
{
	; // Do something every ms
};

int timer0_initialize()
{
	// Return value
	int res = 0;
	// Timer number
	uint8_t timer = 0;
	// Compare match A value
	uint8_t compare_value = F_CPU / 64 / 1000 - 1;
	// Set compare match A value
	if (timer_set_value(timer, TVT_OUTPUT_COMPARE_A, compare_value) == TIMER_ERROR_SUCCESS)
	{
		// Set interrupt on compare match A
		if (timer_set_interrupt(timer, TIT_OUTPUT_COMPARE_MATCH_A, &timer0_compA) == TIMER_ERROR_SUCCESS)
		{
			// Initialize and start timer
			if (timer_init(timer, TOM_CLEAR_ON_COMPARE, TPS_DIV_64, 0) == TIMER_ERROR_SUCCESS)
			{
				res = 1;
			}
		}
	}
	return res;
}		
\endcode
 */

// TODO: Check functionality of Timer4 and Timer5

#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>

/// Return value in case of success.
#define TIMER_ERROR_SUCCESS					0x00
/// Return value in case of an invalid operation.
#define TIMER_ERROR_INVALID_OPERATION		0x01
/// Return value in case of an invalid counter.
#define TIMER_ERROR_INVALID_TIMER			0x02

/** Definition of the timer prescaler.
	Definition of the timer prescaler assigned by #timer_set_prescaler.
 */
typedef enum {
	/// Disable timer
	TPS_DISABLED = 0x00,
	/// Set prescaler to clock frequency
	TPS_NONE = 0x11,
	/// Set prescaler to clock frequency / 8
	TPS_DIV_8 = 0x22,
	/// Set prescaler to clock frequency / 32
	TPS_DIV_32 = 0x30,
	/// Set prescaler to clock frequency / 64
	TPS_DIV_64 = 0x43,
	/// Set prescaler to clock frequency / 128
	TPS_DIV_128 = 0x50,
	/// Set prescaler to clock frequency / 256
	TPS_DIV_256 = 0x64,
	/// Set prescaler to clock frequency / 1024
	TPS_DIV_1024 = 0x75
	// Future work: External clock not supported yet
} timer_prescaler;

/** Definition of the timer operation mode.
	Definition of the timer operation mode assigned by #timer_set_mode.
 */
typedef enum {
	/// Set operation mode to normal. In this mode the timer is constantly up-counting until it overflows
	/// and starts from the bottom.
	TOM_NORMAL = 0x00,
	/// Set operation mode to Clear Timer On Compare Match (CTC). In this mode the timer is constantly up-counting
	/// until it reaches the value in the Compare Match Register A and starts from the bottom.
	TOM_CLEAR_ON_COMPARE = 0x42
	// Future work: More modes are not supported at the moment
} timer_operation_mode;

/** Definition of the timer interrupt types.
	Definition of the timer interrupt type assigned by #timer_set_interrupt.
 */
typedef enum {
	/// Set interrupt on overflow
	TIT_OVERFLOW = 0, // TOIEn
	/// Set interrupt on compare match with register A
	TIT_OUTPUT_COMPARE_MATCH_A = 1, // OCIEnA
	/// Set interrupt on compare match with register B
	TIT_OUTPUT_COMPARE_MATCH_B = 2, // OCIEnB
	/// Set interrupt on compare match with register C
	TIT_OUTPUT_COMPARE_MATCH_C = 3 // OCIEnC
	// Future work: titInputCapture = 5 // ICIEn
} timer_interrupt_types;

/** Definition of the timer value types.
	Definition of the timer value that can be set using #timer_set_value.
 */
typedef enum {
	/// Set counter value
	TVT_COUNTER_VALUE,
	/// Set value in compare match register A
	TVT_OUTPUT_COMPARE_A,
	/// Set value in compare match register B
	TVT_OUTPUT_COMPARE_B,
	/// Set value in compare match register C
	TVT_OUTPUT_COMPARE_C
	// Future work: Set value in input match register
} timer_value_type;

/** Function to set the prescaler of a timer.
	This function sets the prescaler of a timer. Normally it is not necessary to call this function
	directly as the prescaler is already set when initializing the timer by calling #timer_init.
	\param[in]	timer		Timer to be used.
	\param[in]	prescaler	Timer prescaler to be set.
	\note Use the function #timer_disable to disable a timer.
	\note Refer to the <a href="#table">feature overview table</a> to see which prescalers
	are supported for each timer.
	\returns The function returns #TIMER_ERROR_SUCCESS in case of success. The function fails with #TIMER_ERROR_INVALID_TIMER if
	the specified timer is invalid or with #TIMER_ERROR_INVALID_OPERATION if the specified prescaler is not supported by the timer.
 */
int timer_set_prescaler(const uint8_t timer, const timer_prescaler prescaler);

/** Function to disable a timer.
	\param[in]	timer		Timer to be used.
	\returns The function returns #TIMER_ERROR_SUCCESS in case of success. The function fails with #TIMER_ERROR_INVALID_TIMER if
	the specified timer is invalid.
 */
int timer_disable(const uint8_t timer);

/** Function to set the timer operation mode
	This function sets the operation mode of a timer. Normally it is not necessary to call this function
	directly as the operation mode is already set when initializing the timer by calling #timer_init.
	\param[in]	timer		Timer to be used.
	\param[in]	timer_mode	Timer operation mode to be set.
	\note Refer to the <a href="#table">feature overview table</a> to see which operation modes
	are supported for each timer.
	\returns The function returns #TIMER_ERROR_SUCCESS in case of success. The function fails with #TIMER_ERROR_INVALID_TIMER if
	the specified timer is invalid or with #TIMER_ERROR_INVALID_OPERATION if the specified operation mode is not supported by the timer.
 */
int timer_set_mode(const uint8_t timer, const timer_operation_mode timer_mode);

/** Function to get a timer specific value.
	This function reads a timer specific value.
	\param[in]	timer		Timer to be used.
	\param[in]	type		Timer value type to be set.
	\param[in]	act_value	Pointer to a timer value to be read.
	\note Not all timers support 16 bit values. Refer to the <a href="#table">feature overview table</a> to see which specific
	timer values and resolutions are supported for each timer.
	\returns The function returns #TIMER_ERROR_SUCCESS in case of success and stores the requested value in \p act_value.
	The function fails with #TIMER_ERROR_INVALID_TIMER if
	the specified timer is invalid or with #TIMER_ERROR_INVALID_OPERATION if the specified value is not available on the timer.
 */
int timer_get_act_value(const uint8_t timer, const timer_value_type type, uint16_t * act_value);

/** Function to set a timer specific value.
	This function reads a timer specific value.
	\param[in]	timer		Timer to be used.
	\param[in]	type		Timer value type to be set.
	\param[in]	new_value	Timer value to be set.
	\note Not all timers support 16 bit values. Refer to the <a href="#table">feature overview table</a> to see which specific
	timer values and resolutions are supported for each timer.
	\returns The function returns #TIMER_ERROR_SUCCESS in case of success. The function fails with #TIMER_ERROR_INVALID_TIMER if
	the specified timer is invalid or with #TIMER_ERROR_INVALID_OPERATION if the specified value is not available on the timer.
 */
int timer_set_value(const uint8_t timer, const timer_value_type type, const uint16_t new_value);

/** Function to set a timer value.
	\param[in]	timer		Timer to be used.
	\param[in]	new_value	Timer value to be set.
	\note Not all timers support 16 bit values. Refer to the <a href="#table">feature overview table</a> to see which resolutions
	are supported for each timer.
	\returns The function returns #TIMER_ERROR_SUCCESS in case of success. The function fails with #TIMER_ERROR_INVALID_TIMER if
	the specified timer is invalid or with #TIMER_ERROR_INVALID_OPERATION if the specified operation mode is not supported by the timer.
 */
int timer_set(const uint8_t timer, const uint16_t new_value);

/** Function to get a timer value.
	\param[in]	timer		Timer to be used.
	\param[out]	act_value	Pointer to the timer value to be read.
	\note Not all timers support 16 bit values. Refer to the <a href="#table">feature overview table</a> to see which resolutions
	are supported for each timer.
	\returns The function returns #TIMER_ERROR_SUCCESS in case of success and stores the requested value in \p act_value.
	The function fails with #TIMER_ERROR_INVALID_TIMER if
	the specified timer is invalid or with #TIMER_ERROR_INVALID_OPERATION if the specified operation mode is not supported by the timer.
 */
int timer_get(const uint8_t timer, uint16_t * act_value);

/** Function to reset a timer to zero.
	\param[in]	timer		Timer to be used.
	\returns The function returns #TIMER_ERROR_SUCCESS in case of success. The function fails with #TIMER_ERROR_INVALID_TIMER if
	the specified timer is invalid.
 */
int timer_reset(const uint8_t timer);

/** Function to initialize a timer with a certain operation mode and prescaler.
	This function initializes a timer with a certain operation mode, prescaler and preset and starts counting.
	Use this function rather than calling #timer_set_mode, #timer_set_prescaler and #timer_set separately.
	\param[in]	timer		Timer to be used.
	\param[in]	mode		Timer operation mode to be set.
	\param[in]	prescaler	Timer prescaler to be set.
	\param[in]	preset		Timer preset value to be set.
	\note Refer to the <a href="#table">feature overview table</a> to see which settings
	are supported for each timer.
	\returns The function returns #TIMER_ERROR_SUCCESS in case of success. The function fails with #TIMER_ERROR_INVALID_TIMER if
	the specified timer is invalid or with #TIMER_ERROR_INVALID_OPERATION if any setting is not supported by the timer.
 */
int timer_init(const uint8_t timer, const timer_operation_mode mode, const timer_prescaler prescaler, const uint16_t preset);

#ifndef TIMER_ENABLE_SIMPLE_INTERRUPTS

/// Timer callback function definition.
typedef void (*timer_callback)(void);

/** Function to assign a timer interrupt.
	This function defines a callback function to be called in case a certain interrupt occurs.
	\param[in]	timer			Timer to be used.
	\param[in]	interrupt_type	Timer interrupt type to assign callback for.
	\param[in]	callback		Callback function to be called in case the specified interrupts occurs.
	Assign this parameter to _NULL_ to disable an interrupt.
	\note General interrupts must be enabled in order to make the interrupts work. To enable
	them call the function _sei()_ before.
	\note The function is not available if the compiler definition _TIMER_ENABLE_SIMPLE_INTERRUPTS_ is set.
	Then the interrupts must be defined manually by using the _ISR()_ macros.
	\note Refer to the <a href="#table">feature overview table</a> to see which interrupts
	are supported for each timer.
	\returns The function returns #TIMER_ERROR_SUCCESS in case of success. The function fails with #TIMER_ERROR_INVALID_TIMER if
	the specified timer is invalid or with #TIMER_ERROR_INVALID_OPERATION if the specified interrupt type is not supported by the timer.
 */
int timer_set_interrupt(const uint8_t timer, const timer_interrupt_types interrupt_type, const timer_callback callback);

#endif

#endif /* __TIMER_H */