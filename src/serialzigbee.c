#include "serialzigbee.h"
#include <avr/io.h>

void serial_set_zigbee() {
	
  //Output on the 3 pins
  DDRD &= ~(LINK_PLUGIN | ENABLE_RXD_LINK_PC | ENABLE_RXD_LINK_ZIGBEE);
	
  PORTD &= ~(LINK_PLUGIN | ENABLE_RXD_LINK_PC); //Deactivate LINK_PLUGIN and ENABLE_RXD_LINK_PC   // no pull up
  PORTD |= ENABLE_RXD_LINK_ZIGBEE; //Activate ENABLE_RXD_LINK_ZIGBEE

}

void serial_set_wire() {
  DDRD &= ~(LINK_PLUGIN | ENABLE_RXD_LINK_PC | ENABLE_RXD_LINK_ZIGBEE);
  
  //Dunno yet, with next line
  //PORTD &= ~0x80; //PORT_LINK_PLUGIN = 0;   // no pull up
  
  PORTD &= ~(ENABLE_RXD_LINK_ZIGBEE); //Deactivate ENABLE_RXD_LINK_ZIGBEE
  PORTD |= ENABLE_RXD_LINK_PC; //Activate ENABLE_RXD_LINK_PC
  
}

	