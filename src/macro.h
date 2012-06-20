/*! \file macro.h
    \brief Macros for accessing single bits within a register 
    */

#ifndef __MACRO_H
#define __MACRO_H

/*!Gets the value of the bit number BIT within the register REGISTER.
*
* Example:
*\code
*	if( GET(PORTB,0) == 1 ){
*		//...do something if the PORTB0 pin is set 
*	}
*\endcode
*/
#define GET(REGISTER,BIT) ((REGISTER) & (1<<(BIT)))

/*!Sets the bit number BIT within the register REGISTER.
*
* Example:
*\code
*	SET(ADMUX,3) //sets de bit 3 in the ADMUX register
*\endcode
*/

#define SET(REGISTER,BIT) ((REGISTER) |= (1<<(BIT)))

/*!Clears the bit number BIT within the register REGISTER.
*
* Example:
*\code
*	CLEAR(ADMUX,3) //clears de bit 3 in the ADMUX register
*\endcode
*/
#define CLEAR(REGISTER,BIT) (REGISTER &= ~(1<<(BIT)))

/*!Toggles the bit number BIT within the register REGISTER.
*
* Example:
*\code
*	TOGGLE(PORTB,3) //Toggles de bit 3 in the PORTB register
*
*\endcode
*/
#define TOGGLE(REGISTER,BIT) ((REGISTER) ^= (1<<(BIT)))

#endif /* __MACRO_H */