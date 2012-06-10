#include <avr/io.h>
#include <util/delay.h>
#include "error.h"
#include "macro.h"


void sensor_init(char sensor,char type){
	//check sensor is [1:6]
	if(sensor<1 || sensor>6){
		error_blocking();	
	}
	//Set PORTS FOR IR
	if (type){
	SET(DDRA,(8-sensor)); //set port for IR as output
	SET(PORTA,(8-sensor)); //turn off IR
	}
	//Set ADC
	ADCSRA= _BV(ADEN) | _BV(ADIF)  | _BV(ADPS2) |_BV(ADPS1) |_BV(ADPS0) ;  //Enable AD,Clean IF,ADPS2:0 = 111-> 1/128div
	
}

uint8_t sensor_read(char sensor,char type) {
	//check sensor is [1:6]
	if(sensor<1 || sensor>6){
		error_blocking();
	}
	if(type){
		CLEAR(PORTA,(8-sensor));		//Turn on IR
		_delay_us(12);					// Short Delay for rising sensor signal
	}
	ADMUX = sensor;					//Select ADC channel	
	SET(ADCSRA,ADIF);				//Clear ADC Interrupt Flag
	SET(ADCSRA,ADSC);				//Start Conversion
	while( !(ADCSRA & (1 << ADIF)) ); //wait for conversion to finish
	//sensor_read(sensor);
	if(type) {
		SET(PORTA,(8-sensor));			//Turn off IR led  
	}
	// Scale measurement value
	uint16_t adc = ADC;
	if (adc > 511)
		adc = 511;
	return (uint8_t) (adc >> 1);
}

/*
int sensor_read(int sensor) {
	//check sensor is [1:6]
	if(sensor<1 || sensor>6){
		error_blocking();
	}
	SET(ADCSRA,ADIF);				//Clear ADC Interrupt Flag
	SET(ADCSRA,ADSC);				//Start Conversion
	while( !(ADCSRA & (1 << ADIF)) ); //wait for conversion to finish
		
	return ADC;
}

*/




