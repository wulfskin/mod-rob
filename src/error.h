/*! \file error.h
    \brief Led sequence that can be used to show errors at runtime
	\author Walter Gambelunghe
	\copyright GNU Public License V3
	\date 2012
	
	\file error.h
	
	\details This file defines a function to show a led sequence that can be used to indicate an error occouring at runtime.
	The sequence consists in lighting all the leds, one by one, and then turning them off, again one by one.
	There are two versions of this function, to show the sequence only once, or play it forever in a loop.
	
	Examples:
	\par 1. Shows the sequence once
	\code
		if(current_position > desired_max_position)
			error();
	\endcode
	\par 2. Blocking error
	\code
		if(current_index > NUMBER_OF_ELEMENTS)
			error_blocking();
	\endcode
*/

#ifndef __ERROR_H
#define __ERROR_H

/** Show led sequence once. Should be used for undesirable behaviors that do not compromise the correct execution 
 */
void error(void);

/** Block execution and show led sequence forever. Should be used for errors that can lead to unpredictible behaviors e.g. exceeding the index bounds of an array
 */
void error_blocking(void);


#endif /*__ERROR_H */