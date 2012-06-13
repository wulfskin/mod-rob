#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "../macro.h"
#include "../sensor.h"
#include "../motor.h"
#include <dynamixel.h>
#include "../serial.h"
#include "../error.h"
#include "../serialzigbee.h"
#include "../io.h"

#define NUM_ACTUATOR 3

int16_t turn=0;

void something(void){
	LED_TOGGLE(LED_MANAGE);
	unsigned char data;
	serial_read(&data, 1);
	//printf("%c\n", data);
	
	if(data=='a'){
		turn = turn>=120 ? 120 : turn+20;
		printf("turn=%d\n", turn);
	}		
	else if(data=='d'){
			turn = turn<= -120 ? -120 : turn-20;
			printf("turn=%d\n", turn);
	}
	else if(data=='w'){
		turn=0;
		printf("turn=%d\n", turn);
	}					
}

		//uint8_t ids[3]={4,7,8};
		uint8_t ids[2]={7,8};
		//uint16_t positions[3];//={512+40,512+70,512-70};
		uint16_t positions[2];//={512+40,512+70,512-70};

	int main() {
		int CommStatus;
				
		dxl_initialize(0,1);
		serial_initialize(57600);
		serial_set_rx_callback(&something);
		serial_set_zigbee();
		sei();
		io_init();
		int i=0,j=0,readd=0,moving=0;
		printf("test\n");
		dxl_write_word(4,GOAL_POSITION_L,512);
		dxl_write_word(7,GOAL_POSITION_L,512);
		dxl_write_word(8,GOAL_POSITION_L,512);
		while(!(BTN_DOWN_PRESSED));
		for(i=0;i<500;i++){
			
			//positions[0]=512+30;
			//positions[1]=512+70;
			//positions[2]=512+turn-70;
			//motor_sync_move(3,ids,positions);
			
			//positions[0]=512+30;
			
			dxl_write_word(4,GOAL_POSITION_L,527+30);
			_delay_ms(60);
			positions[0]=512+90;
			positions[1]=512+turn-90;
			motor_sync_move(2,ids,positions);
			
				
			//_delay_ms(1);
/*			do{
				moving=0;
				for(j=0;j<3;j++){
					_delay_ms(1);
					readd=dxl_read_byte(ids[j],MOVING);
					CommStatus=dxl_get_result();					
					if(CommStatus!=COMM_RXSUCCESS)
					//printf("motor %d moving status:%d\n",j,readd);
						printf("error %d on motor %d  \n\n",CommStatus,j);
					
					moving|=readd;
				}
			}while(moving);*/								
			_delay_ms(200);
			//positions[0]=512-30;
			//positions[1]=512-70;
			//positions[2]=512+turn+70;
			//motor_sync_move(3,ids,positions);
			
			dxl_write_word(4,GOAL_POSITION_L,527-30);
			_delay_ms(60);
			positions[0]=512-90;
			positions[1]=512+turn+90;
			motor_sync_move(2,ids,positions);
			
			_delay_ms(200);
/*			do{
				moving=0;
				for(j=0;j<3;j++){
					readd=dxl_read_byte(ids[j],MOVING);
					//printf("motor %d moving status:%d\n",j,readd);
					_delay_ms(1);
					moving|=readd;
				}
			}while(moving); */
			
		}//closes for loop
			
		
		while(1) {
						
		}
		return 0;
}