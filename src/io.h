/*! \file io.h
    \brief Interface to the Robotis CM-510 controller I/O peripheries.
	\author Hans-Peter Wolf
	\copyright GNU Public License V3
	\date 2012

	\file io.h
    \details This file provides easy-to-use access
	to the I/O (LEDs, buttons, the microphone and the buzzer) of the CM-510
	controller from Robotis. In order to use this interface it has to be 
	initialized once by calling #io_init(). Then the following functions and 
	macros can be used.
	
	In order to get started the following examples might be helpful:
	
	\par 1. Switching an LED on:
\code
// Initialize LEDs once
led_init(); 
// Switch the auxiliary (AUX) LED on
LED_ON(LED_AUX);
// Other LEDs are: LED_POWER, LED_TXD, LED_RXD, LED_AUX, LED_MANAGE, LED_PROGRAM, LED_PLAY
LED_ON(LED_TXD | LED_RXD); // Several LEDs can be put on by using bitwise-or.
// Switch all LEDs off
LED_OFF(LED_ALL);
\endcode

	\details \par 2. Checking if a button is pressed:
\code
// Initialize the buttons once
btn_init();
// Check if button start is pressed
if (BTN_START_PRESSED) ;
// Other buttons are: BTN_UP_PRESSED, BTN_DOWN_PRESSED, BTN_LEFT_PRESSED, BTN_RIGHT_PRESSED
\endcode

	\details \par 3. Check if the microphone detects a signal:
\code
// Initialize the microphone once
mic_init();
// Check for a signal
if (MIC_ACTIVE) ;
\endcode

	\details A more sophisticated way of accessing the microphone and the buttons is provided
	by a callback (interrupt) interface. Please refer to #io_set_interrupt() for more information.

	\details \par 4. Generate a buzz:
\code
// initialize the buzzer
buzzer_init();
// Generate a buzz
BUZZ;
\endcode
 */

#ifndef	__IO_H
#define __IO_H

#include <avr/io.h>
#include <util/atomic.h>

#ifdef __cplusplus
extern "C" {
#endif

// *********************************************************************************
// PORTB
//    Toggle buzzer pin to buzz

// Symbolic address of buzzer port.
#define BUZZER			0x20

/// Generate a buzz.
#define BUZZ			(PORTB ^= (BUZZER))

/// Function to initialize the buzzer port.
void buzzer_init();

// *********************************************************************************
// PORTC - LEDs
//    Set pin output low to turn LED on.
//    Can control multiple LEDs with bit-wise OR
//        e.g. LED_ON(LED_POWER | LED_AUX);

/// Symbolic address the power LED.
#define LED_POWER		0x01

/// \deprecated Symbolic address of the power LED (alias).
#define LED_BAT			LED_POWER

/// Symbolic address of the transfer data (TxD) LED.
#define LED_TXD			0x02

/// Symbolic address of the receive data (RxD) LED.
#define LED_RXD			0x04

/// \deprecated Symbolic address of the transfer data (TxD) LED (alias).
#define LED_TxD			LED_TXD

/// \deprecated Symbolic address of the receive data (RxD) LED (alias).
#define LED_RxD			LED_RXD

/// Symbolic address of the auxiliary (AUX) LED.
#define LED_AUX			0x08

/// Symbolic address of the manage mode (MANAGE) LED.
#define LED_MANAGE		0x10

/// Symbolic address of the program mode (PROGRAM) LED.
#define LED_PROGRAM		0x20

/// Symbolic address of the play mode (PLAY) LED.
#define LED_PLAY		0x40

/// Symbolic address of all LEDs.
#define LED_ALL			(LED_POWER | LED_TXD | LED_RXD | LED_AUX | LED_MANAGE | LED_PROGRAM | LED_PLAY)

/** Macro to switch LEDs on.
 * \code
// Switch the power LED on
LED_ON(LED_POWER);
	 
// Switch both the receive and the transfer data LEDs on
LED_ON(LED_TXD | LED_RXD);
 * \endcode
 */
#define LED_ON(x)		(PORTC &= ~(x))

/** Macro to switch LEDs off.
 * \code
 // Switch all LEDs off
 LED_OFF(LED_ALL);
 * \endcode
 */
#define LED_OFF(x)		(PORTC |= (x))

/// Macro to toggle LEDs.
#define LED_TOGGLE(x)	(PORTC ^= (x))

/** Function to initialize controller for LED usage.
 *  The function defines PIN0 - PIN6 of port C of the micro controller as outputs
 *  in order to use them to control the connected LEDs and switches them off (=high).
 */
void led_init();

/** Function to check whether LEDs are set.
 * The function checks whether LEDs are set. All LEDs (#LED_POWER, #LED_MANAGE, #LED_PROGRAM,
 * #LED_PLAY, #LED_TXD, #LED_RXD and #LED_AUX) and a combination of them including
 * #LED_ALL are supported.
 * \param[in]	led		LEDs to check.
 * \returns Returns non-zero, in case _all_ specified LEDs are switched on.
 */
uint8_t led_get(uint8_t led);

// *********************************************************************************
// PORTD - BUTTONS AND MICROPHONE
//    Input high when button pressed or microphone input sufficiently loud.

/// Symbolic address of the start button (START).
#define BTN_START		0x01

/// Macro to check whether the start button is pressed.
#define BTN_START_PRESSED	((PIND & BTN_START) == 0)

/// Symbolic address of the microphone signal.
#define MIC_SIGNAL		0x02

/// Macro to check whether the microphone is active.
#define MIC_ACTIVE		((PIND & MIC_SIGNAL) == 0)

/** Function to initialize controller for button usage.
 * The function defines PIN4 - PIN7 of port E and PIN0 of port D of the micro controller 
 * as inputs in order to use them to check the connected buttons. It also activates the
 * the pull-up resistor.
 */
void mic_init();

// *********************************************************************************
// PORTE - 
//    Input high when button pressed

/// Symbolic address of the up button (U).
#define BTN_UP			0x10
/// Symbolic address of the down button (D).
#define BTN_DOWN		0x20
/// Symbolic address of the left button (L).
#define BTN_LEFT		0x40
/// Symbolic address of the right button (R).
#define BTN_RIGHT		0x80

/// Macro to check whether the up button is pressed.
#define BTN_UP_PRESSED		((PINE & BTN_UP) == 0)

/// Macro to check whether the down button is pressed.
#define BTN_DOWN_PRESSED	((PINE & BTN_DOWN) == 0)

/// Macro to check whether the left button is pressed.
#define BTN_LEFT_PRESSED	((PINE & BTN_LEFT) == 0)

/// Macro to check whether the right button is pressed.
#define BTN_RIGHT_PRESSED	((PINE & BTN_RIGHT) == 0)

/** Function to initialize controller for button usage.
 * The function defines PIN4 - PIN7 of port E and PIN0 of port D of the micro controller 
 * as inputs in order to use them to check the connected buttons. It also activates the
 * the pull-up resistor.
 */
void btn_init();

/// I/O callback function definition.
typedef void (*io_callback)(void);

/** Function to define interrupts for the microphone and buttons.
 * This function can be used to assign callback functions to be called
 * in case the state of the microphone or the buttons changes. In case more than one port
 * is defined, the callback function for all of them will be the same.
 * \par Example:
\code
#include <avr/interrupt.h>

void btn_start_callback()
{
	if (BTN_START_PRESSED)
		printf("Button start was pressed.\n");
	else
		printf("Button start was released.\n");
}	

// Initialize I/O
io_init();
// Initialize general interrupts
sei();
// Assign callback function for BTN_START
if (io_set_interrupt(BTN_START, &btn_start_callback))
	printf("Interrupt enabled.\n");
\endcode
 * \param[in]	io_port		Port(s) for which the callback function should be assigned.
 * Valid ports are #MIC_SIGNAL, #BTN_START, #BTN_UP, #BTN_DOWN, #BTN_LEFT and #BTN_RIGHT.
 * \param[in]	callback	Callback function to be called in case the state changes.
 * Assign this parameter to _NULL_ to disable interrupts.
 * \returns The return value is non-zero in case the operation has been successful. That is
 * if the provided port(s) was/were valid.
 * \note General interrupts must be enabled in order to make the interrupts work. To enable
 * them call the function _sei()_ before. 
 */
int io_set_interrupt(uint8_t io_port, io_callback callback);

/** Function to initialize the complete I/O peripheries.
 * This helper function initializes the complete I/O peripheries
 * by initializing the buzzer, LEDs, microphone and buttons internally.
 * Alternatively, one can initialize each component separately.
 */ 
void io_init();

#ifdef __cplusplus
}
#endif

#endif