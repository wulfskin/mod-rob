#include <avr/io.h>
#include <util/delay.h>
#include "macro.h"
void error( void )
{
	char current,cycle;
	for (current=0,cycle=0;cycle<14;current=(current+1)%7,cycle++){
		TOGGLE(PORTC,current);   //Set PC0 to low (turn on led)
		_delay_ms(100); //wait 1000 ms = 1 sec
	}
}

void error_blocking(void){
	while(1){
	error();
	}
}
