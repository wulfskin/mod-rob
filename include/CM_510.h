// CM-510/700 ease of use C/C++ preprocessor definitions
// Pulled mostly from Robotis example code and cleaned/formatted

// NOTES:
// - The CM-700 is nearly identical to the CM-510, except the CM-700 has
//    no Buzzer, Microphone, or D-PAD on board.
// - ADC Ports on the CM-700 do not have a keyed plastic enclosure, so be 
//    sure the connector is properly oriented.  The numbering convention
//    of the IR sensor is opposite that of the CM-510 and CM-700 (grey wire
//    is pin 1 on IR sensor page, but on the CM-510/CM-700 it is pin 5 which
//    is not connected).
// - There are three lines of code in the ZigBee_RC100 example code for both
//    the CM-510 and CM-700 that perplex me a bit.  Given that none of these
//    pins are listed on the Robotis support site for the CM-700 as being 
//    connected to any signal of note nor are they set as outputs, I am unsure
//    that they are actually functional.  A more indepth look is at the end
//    of this file.
//        PORTD &= ~0x80;	//PORT_LINK_PLUGIN = 0;   // no pull up
//        PORTD &= ~0x20;	//PORT_ENABLE_RXD_LINK_PC = 0;
//        PORTD |= 0x40;	//PORT_ENABLE_RXD_LINK_ZIGBEE = 1;

/*! \file CM_510.h
    \brief Symbolic definitions of the Robotis CM-510 controller peripherie.
    \copyright GNU Public License.
	\author Unknown
    
    This file contains useful symbolic definitions in order to simplify the access
	to LEDs, buttons, microphone and analog-digital	converters (ADC) of the CM-510
	controller from Robotis.
*/

#ifndef	_CM_510_DEFINES_H_ 
#define _CM_510_DEFINES_H_

#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

// ADC
//     When using IR sensors, must wait ~12 ms between enabling IR PORT
//      and starting conversion
//     Output of ADC retrieved with 'ADC'  (e.g. 'int value=ADC;')
// ADC Enable, Clock 1/64div.
#define	ADC_ENABLE		ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1)
// Define which ADC pin to use (also sets reference voltage to AREF pin (5V?))
#define USE_ADC_PORT_1	ADMUX = 1
#define USE_ADC_PORT_2	ADMUX = 2
#define USE_ADC_PORT_3	ADMUX = 3
#define USE_ADC_PORT_4	ADMUX = 4
#define USE_ADC_PORT_5	ADMUX = 5
#define USE_ADC_PORT_6	ADMUX = 6
// Clear ADC conversion flag, then begin new conversion
#define ADC_BEGIN		ADCSRA |= (1 << ADIF); ADCSRA |= (1 << ADSC)
// True while conversion incomplete (e.g. 'while (ADC_INCOMPLETE);' )
#define ADC_INCOMPLETE	!(ADCSRA & (1 << ADIF))

// PORTA
//    Set pin output low to set external header high (inverted via transistor)
//    Can control multiple ports with bit-wise OR
//        e.g. EXT_PORT_ON(EXT_PORT_1 | EXT_PORT_2);
#define EXT_PORTS_ENABLE	DDRA|=(0xFC)
#define EXT_PORTS_DISABLE	DDRA&=~(0xFC)
#define EXT_PORT_1		0x80
#define EXT_PORT_2		0x40
#define EXT_PORT_3		0x20
#define EXT_PORT_4		0x10
#define EXT_PORT_5		0x08
#define EXT_PORT_6		0x04
#define EXT_PORT_ON(x)	PORTA&=~(x)
#define EXT_PORT_OFF(x)	PORTA|=(x)

// PORTB
//    Set pin output low to turn on (inverted via transistor?)
#define BUZZER			0x20
#define BUZZER_ON		PORTB&=~(BUZZER)
#define BUZZER_OFF		PORTB|=(BUZZER)

// PORTC
//    Set pin output low to turn LED on (inverted via transistor?)
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

/// Symbolic address of all LEDs
#define LED_ALL			(LED_POWER | LED_TXD | LED_RXD | LED_AUX | LED_MANAGE | LED_PROGRAM | LED_PLAY)

/** Macro to switch LEDs on.
 * \code
// Switch the power LED on
LED_ON(LED_POWER);
	 
// Switch both the receive and the transfer data LEDs on
LED_ON(LED_TXD | LED_RXD);
 * \endcode
 */
#define LED_ON(x)		PORTC &= ~(x)

/** Macro to switch LEDs off.
 * \code
 // Switch all LEDs off
 LED_OFF(LED_ALL);
 * \endcode
 */
#define LED_OFF(x)		PORTC |= (x)

/** Macro to initialize controller for LED usage.
 * The macro defines PIN0 - PIN6 of port C of the microcontroller as outputs
 * in order to use them to control the connected LEDs and switches them off (=high).
 */
#define LED_Init		DDRC = LED_ALL; LED_OFF(LED_ALL)

// PORTD
//    Input high when button pressed or mic input sufficiently loud
/// Symbolic address of the start button (START)
#define BTN_START		0x01

#define BTN_START_PRESSED	(PIND&BTN_START)

#define MIC_SIGNAL		0x02
#define MIC_ACTIVE		(PIND&MIC_SIGNAL)

// PORTE
//    Input high when button pressed
/// Symbolic address of the up button (U)
#define BTN_UP			0x10
/// Symbolic address of the down button (D)
#define BTN_DOWN		0x20
/// Symbolic address of the left button (L)
#define BTN_LEFT		0x40
/// Symbolic address of the right button (R)
#define BTN_RIGHT		0x80

#define BTN_UP_PRESSED		(PINE&(BTN_UP))
#define BTN_DOWN_PRESSED	(PINE&(BTN_DOWN))
#define BTN_LEFT_PRESSED	(PINE&(BTN_LEFT))
#define BTN_RIGHT_PRESSED	(PINE&(BTN_RIGHT))

/** Macro to initialize controller for button usage.
 * The macro defines PIN4 - PIN7 of port E and PIN0 of port D of the microcontroller 
 * as inputs in order to use them to check the connected buttons.
 */
 #define BTN_Init		DDRE &= ~(BTN_UP | BTN_DOWN | BTN_LEFT | BTN_RIGHT); \
						PORTE |= (BTN_UP | BTN_DOWN | BTN_LEFT | BTN_RGIHT); \
						DDRD &= ~(BTN_START); \
						PORTD |= (BTN_START)

// ZIGBEE
// The three steps that perplex me (PORTD is by default in input mode and none
//  of the example programs change that)
//    1) Turn off pull-up resistor of PORTD PIN7 (UNLISTED ON SCHEMATIC)
//        Suspect this is for the CM-510 with its audio connector (with 
//        a built-in switch) for RS-232 serial link
//    2) Turn off pull-up resistor of PORTD PIN5 (UNLISTED ON SCHEMATIC)
//        Suspect this is for disabling the RS-232 level converter, but if 
//        it really were, shouldn't this pin be an output instead of an input?
//    3) Turn on pull-up resistor of PORTD PIN6 (UNLISTED ON SCHEMATIC)
//        Suspect this is related to the RS-232 level converter and/or a line
//        buffer, but if it really were, shouldn't this pin be an output
//        instead of an input?  Also, why are we not using the pin labeled
//        'ZIGBEE_RESET' on the CM-700 header schematic?  I am guessing 
//        that 'ZIGBEE_RESET' (PORTD PIN4 = 0x10) is connected to Q1 through
//        R8 as labeled on the silkscreen and can turn on/off 3.3V power to
//        the ZIG-110 module.
#define ZIG110_ENABLE	PORTD &= ~0x80; PORTD &= ~0x20; PORTD |= 0x40

// UNTESTED, but going by the very limited CM-700 schematic on the Robotis
//  support site...
// ZIG-100 power/reset control is PORTD PIN4 and should be an output
//#define ZIG110_RESET	DDRC|=0x10; PORTD|=0x10;_delay_ms(10);PORTD&=~0x10
//#define ZIG110_DOWN	DDRC|=0x10; PORTD|=0x10
//#define ZIG110_UP		DDRC|=0x10; PORTD&=~0x10

#ifdef __cplusplus
}
#endif

#endif
