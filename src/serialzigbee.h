/*! \file serialzigbee.c
    \brief Zigbee serial communication enabler and disabler
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
