/*! \file error.c
    \brief Led sequence that can be used to show errors at runtime (declaration part, see error.h for an interface description).
	\author Walter Gambelunghe
	\copyright GNU Public License V3
	\date 2012
*/

#include <avr/io.h>
#include <util/delay.h>
#include "macro.h"
void error( void )
{
	char current,cycle;
	for (current=0,cycle=0;cycle<14;current=(current+1)%7,cycle++){
		TOGGLE(PORTC,current);   	//toggle led)
		_delay_ms(100); 			//short delay
	}
}

void error_blocking(void){
	while(1){					//loop forever
	error();
	}
}
