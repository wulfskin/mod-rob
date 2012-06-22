/*! \file serialzigbee.h
    \brief Zigbee serial communication enabler and disabler
	\author Dennis Hellner
	\copyright GNU Public License V3
	\date 2012
	
	\file serialzigbee.h
	\details The serialzigbee is a add on for the typical serial connection, which is working for the Robotis controller CM-510.
	According to the Robotis E-manual the serial communication receive pin can be set to receive ether from serial wire or Zigbee.
	The port LINK_PLUGIN and ENABLE_RXD_LINK_PC on the microcontroller should be disabled (set to zero) and the	ENABLE_RXD_LINK_ZIGBEE should be activated (set to one).
	
	The serial sending will still be send both trough wire and Zigbee.
 */

#ifndef _SERIALZIGBEE_HEADER
#define _SERIALZIGBEE_HEADER

#ifdef __cplusplus
extern "C" {
#endif

/// Declearing of pin position map in port D for link_plugin pin
#define LINK_PLUGIN 0x80
/// Declearing of pin position map in port D for enable rxd link pc pin
#define ENABLE_RXD_LINK_PC 0x20
/// Declearing of pin position map in port D for enable rxd link zigbee pin
#define ENABLE_RXD_LINK_ZIGBEE 0x40

/**
  Activates recieving throug Zigbee connection
  Sets the link plugin and enable rxd link pc to zero for disabling recieving data from wire
  Activates the enable rxd link zigbee for enabling recieving data from Zigbee
  */
void serial_set_zigbee();

/**
  Activates recieving throug wire connection
  Deactivates the enable rxd link zigbee for disabling recieving data from Zigbee
  Sets the link plugin and enable rxd link pc to one for enabling recieving data from wire
  */
void serial_set_wire();


#ifdef __cplusplus
}
#endif

#endif
