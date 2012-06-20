/*! \file sensor.c
    \brief Interface to control the sensors in the Robotis Bioloid Kit (declaration part, see sensor.h for an interface description).

 */

#include <avr/io.h>
#include <util/delay.h>
#include "error.h"
#include "macro.h"


void sensor_init(uint8_t port,uint8_t type){
	//check port is [1:6]
	if(port<1 || port>6){
		error_blocking();				//error, trying to initialize wrog port
	}
	//Set PORTS FOR IR
	if (type){
	SET(DDRA,(8-port)); 				//set port for IR as output
	SET(PORTA,(8-port)); 				//turn off IR
	}
	//Set ADC
	ADCSRA= _BV(ADEN) | _BV(ADIF)  | _BV(ADPS2) |_BV(ADPS1) |_BV(ADPS0) ;  //Enable AD,Clean IF,ADPS2:0 = 111-> 1/128div
	
}

uint8_t sensor_read(uint8_t port,uint8_t type) {
	//check port is [1:6]
	if(port<1 || port>6){
		error_blocking();				//error, trying to read wrog port
	}
	if(type){
		CLEAR(PORTA,(8-port));		//Turn on IR
		_delay_us(12);				// Short Delay for rising sensor signal
	}
	ADMUX = port;					//Select ADC channel	
	SET(ADCSRA,ADIF);				//Clear ADC Interrupt Flag
	SET(ADCSRA,ADSC);				//Start Conversion
	while( !(ADCSRA & (1 << ADIF)) ); 	//wait for conversion to finish
	//sensor_read(sensor);
	if(type) {
		SET(PORTA,(8-port));			//Turn off IR led  
	}
	// Scale measurement value
	uint16_t adc = ADC;
	if (adc > 511)
		adc = 511;
	return (uint8_t) (adc >> 1);		//remove less significant bit 
}






