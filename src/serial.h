/*! \file serial.h
    \brief Robotis serial connection interface with receive callback extension.
	\author Robotis
	\author Hans-Peter Wolf (only receive callback function extension)
	\copyright Unknown
	\date 2012
	
	\file serial.h
	\details This file contains the serial connection interface provided by Robotis for the CM-510 controller.
	The source code including documentation can be found in their <a href="http://support.robitis.com/en/">e-manual</a>. Additionally,
	we have added the possibility to register a callback function which is called whenever a char is received over
	the serial interface. This function #serial_set_rx_callback is described here.
 */
#ifndef _SERIAL_HEADER
#define _SERIAL_HEADER

#ifdef __cplusplus
extern "C" {
#endif

/// Receive callback function definition.
typedef void (*serial_rx_callback)(void);

/// \cond Ignore this part from documentation.
void serial_initialize(long ubrr);
void serial_write( unsigned char *pData, int numbyte );
unsigned char serial_read( unsigned char *pData, int numbyte );
int serial_get_qstate(void);
/// \endcond

/** Function to register a data receive callback function.
	This function registers a callback function to be called in case one character is received over the
	serial connection. The received character can be read by calling _serial_read()_.
	See the <a href="http://support.robitis.com/en/">Robotis e-Manual</a> for more help on that function.
	\param[in]	callback			Callback function to be called.
	Assign this parameter to _NULL_ to disable the callback function.
	\note General interrupts must be enabled in order to make the interrupts work. To enable
	them call the function _sei()_ before. 
 */
void serial_set_rx_callback(const serial_rx_callback callback);

#ifdef __cplusplus
}
#endif

#endif
