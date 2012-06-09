#ifndef _SERIALZIGBEE_HEADER
#define _SERIALZIGBEE_HEADER

#ifdef __cplusplus
extern "C" {
#endif

#define LINK_PLUGIN 0x80
#define ENABLE_RXD_LINK_PC 0x20
#define ENABLE_RXD_LINK_ZIGBEE 0x40


void serial_set_zigbee();
void serial_set_wire();


#ifdef __cplusplus
}
#endif

#endif
