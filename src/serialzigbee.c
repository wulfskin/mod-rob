/*! \file serialzigbee.c
    \brief Zigbee serial communication enabler and disabler (declaration part, see serialzigbee.h for an interface description).
 */

#include <avr/io.h>
#include "serialzigbee.h"

void serial_set_zigbee() {
	
  //Output on the 3 pins -MAY NOT BE NESSESARY
  //DDRD &= ~(LINK_PLUGIN | ENABLE_RXD_LINK_PC | ENABLE_RXD_LINK_ZIGBEE);
	
  PORTD &= ~(LINK_PLUGIN | ENABLE_RXD_LINK_PC); //Deactivate LINK_PLUGIN and ENABLE_RXD_LINK_PC   // no pull up
  PORTD |= ENABLE_RXD_LINK_ZIGBEE; //Activate ENABLE_RXD_LINK_ZIGBEE

}

void serial_set_wire() {
  //-MAY NOT BE NESSESARY
  //DDRD &= ~(LINK_PLUGIN | ENABLE_RXD_LINK_PC | ENABLE_RXD_LINK_ZIGBEE);
  
  PORTD &= ~(ENABLE_RXD_LINK_ZIGBEE); //Deactivate ENABLE_RXD_LINK_ZIGBEE
  PORTD |= LINK_PLUGIN | ENABLE_RXD_LINK_PC; //Activate ENABLE_RXD_LINK_PC
  
}

	