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

#define CENTER 512
#define TWIST_AMPLITUDE 20
#define TWIST_OFFSET 0

#define LOOP 100000

#define STEP 250

int16_t turn=0,diff=0;

void something(void){
	LED_TOGGLE(LED_MANAGE);
	unsigned char data;
	serial_read(&data, 1);
	//printf("%c\n", data);
	
	if(data=='a'){
		turn = turn>=180 ? 180 : turn+20;
		printf("turn=%d\n", turn);
	}		
	else if(data=='d'){
			turn = turn<= -180 ? -180 : turn-20;
			printf("turn=%d\n", turn);
	}
	else if(data=='w'){
		turn=0;
		printf("turn=%d\n", turn);
	}					
}

void wait_for_position(int number, int setoffset) {
			//positions[0]=512+30;
			dxl_write_word(number,GOAL_POSITION_L,setoffset);
			uint32_t count=0;
			do
			{
				diff=(setoffset)-dxl_read_word(number,PRESENT_POSITION_L);
				if (diff < 0)
				diff = -diff;
				_delay_us(1000);
				count+=1;
				if ((count&0x007F) == 0x0000)
				 printf("Motor %d now has counted to %u\n", number, count);
			} while ( diff>5 && count<LOOP );	
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
		printf("start\n");
		dxl_write_word(4,GOAL_POSITION_L,512);
		dxl_write_word(7,GOAL_POSITION_L,512);
		//dxl_write_word(8,GOAL_POSITION_L,512);
		
		
		while(!(BTN_DOWN_PRESSED));
		//for(i=0;i<500;i++){
		while(1) {	
			//positions[0]=512+30;
			//positions[1]=512+70;
			//positions[2]=512+turn-70;
			//motor_sync_move(3,ids,positions);
			/*
						//positions[0]=512+30;
						dxl_write_word(4,GOAL_POSITION_L,CENTER+TWIST_OFFSET+TWIST_AMPLITUDE);
						uint32_t count=0;
						do
						{
							diff=(CENTER+TWIST_OFFSET+TWIST_AMPLITUDE)-dxl_read_word(4,PRESENT_POSITION_L);
							if (diff < 0)
							diff = -diff;
							_delay_us(10);
							count+=1;
						} while ( diff>5 && count<LOOP );

			*/
			wait_for_position(4,CENTER+TWIST_OFFSET+TWIST_AMPLITUDE);
			printf("end loop 1,\n");
			//} while (dxl_read_byte(4, MOVING));
			
			/*dxl_write_word(7,GOAL_POSITION_L,CENTER+turn+STEP);
			count=0;
			do
			{
				diff=(CENTER+turn+STEP)-dxl_read_word(7,PRESENT_POSITION_L);
				if (diff < 0)
				diff = -diff;
				_delay_us(10);
				count+=1;
			} while ( diff>5 && count<LOOP );*/
			wait_for_position(7, CENTER+turn+STEP);	
			printf("end loop 2,\n");
			
			//positions[0]=CENTER-turn+STEP;
			//positions[1]=CENTER;//+turn-STEP;
			//motor_sync_move(2,ids,positions);
			
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
			//_delay_ms(200);
		
			//positions[0]=512-30;
			//positions[1]=512-70;
			//positions[2]=512+turn+70;
			//motor_sync_move(3,ids,positions);
			
			/*dxl_write_word(4,GOAL_POSITION_L,CENTER+TWIST_OFFSET-TWIST_AMPLITUDE);
			count=0;
			do{
				diff=(CENTER+TWIST_OFFSET-TWIST_AMPLITUDE)-dxl_read_word(4,PRESENT_POSITION_L);
				if (diff < 0)
				diff = -diff;
				_delay_us(10);
				count+=1;
			} while ( diff>5 && count<LOOP );*/
			wait_for_position(4, CENTER+TWIST_OFFSET-TWIST_AMPLITUDE);
			printf("end loop 3,\n");
			//} while (dxl_read_byte(4, MOVING));
					
			//_delay_ms(60);
			//positions[0]=CENTER-turn-STEP;
			//positions[1]=CENTER;//+turn+STEP;
			//motor_sync_move(2,ids,positions);
			/*dxl_write_word(7,GOAL_POSITION_L,CENTER+turn-STEP);
			//_delay_ms(200);
			count=0;
			do{
				diff=(CENTER+turn-STEP)-dxl_read_word(7,PRESENT_POSITION_L);
				if (diff < 0)
				diff = -diff;
				_delay_us(10);
				count+=1;
			} while ( diff>5 && count<LOOP );*/
			wait_for_position(7, CENTER+turn-STEP);
			printf("end loop 4,\n");
/*			do{
				moving=0;
				for(j=0;j<3;j++){
					readd=dxl_read_byte(ids[j],MOVING);
					//printf("motor %d moving status:%d\n",j,readd);
					_delay_ms(1);
					moving|=readd;
				}
			}while(moving); */

			PrintErrorCode();
			printf("Cycle,\n");
			
		//}//closes for loop
			
		
		
						
		}// close infinite loop
		return 0;
}