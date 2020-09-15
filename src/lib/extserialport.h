/*
 * A software development kit for Sensapex Micromanipulator
 *
 * Copyright (c) 2012 Sensapex. All rights reserved
 *
 * This file is part of Sensapex Micromanipulator SDK
 *
 * The Sensapex micromanipulator SDK is free software: you can redistribute
 * it and/or modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * The Sensapex Micromanipulator SDK is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with the Sensapex micromanipulator SDK. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

#ifndef EXTSERIALPORT_H
#define EXTSERIALPORT_H

#if defined(_WIN32) || defined(_WIN64) || defined(WIN32) || defined(WIN64)
# include <windows.h>
# define _WINDOWS
# define bool BOOL
# define EXTSERIALPORTSHARED_EXPORT
// dll export not needed when linked into upper level library
//# if defined(EXTSERIALPORT_LIBRARY)
//#  define EXTSERIALPORTSHARED_EXPORT __declspec(dllimport)
//# else
//#  define EXTSERIALPORTSHARED_EXPORT __declspec(dllexport)
//# endif
# define EXTSERIALPORT_DEF_PORT "com1"
# define EXTSERIALPORT_FD_T HANDLE
# define EXTSERIALPORT_INVALID_HANLDLE_V INVALID_HANDLE_VALUE
#else
# include <stdbool.h>
# ifdef __MACH__ // Defined for OS X
#  define EXTSERIALPORT_DEF_PORT "/dev/cu.usbserial-FTU6ZL5S"
# else
#  define EXTSERIALPORT_DEF_PORT "/dev/ttyUSB0"
# endif
# define EXTSERIALPORTSHARED_EXPORT
# define EXTSERIALPORT_FD_T int
# define EXTSERIALPORT_INVALID_HANLDLE_V (-1)
#endif

#define EXTSERIALPORT_DEF_BAUDRATE 115200
#define EXTSERIALPORT_DEF_TIMEOUT  500

#ifdef __cplusplus
extern "C" {
#endif

typedef struct extserial_port_state_s
{
	EXTSERIALPORT_FD_T fd;
	unsigned msg_received, msg_sent, errors;
	unsigned long data_received, data_sent;
	int last_error;

} extserial_port_state_t, *extserial_port_handle_t;

// NULL return value means an error for open
EXTSERIALPORTSHARED_EXPORT extserial_port_handle_t extserial_port_open(const char *port);

// Negative values means error, reason from hndl->last_error
EXTSERIALPORTSHARED_EXPORT int  extserial_port_set_timeout(const extserial_port_handle_t hndl,
										   const unsigned value);
EXTSERIALPORTSHARED_EXPORT int  extserial_port_set_baudrate(const extserial_port_handle_t hndl,
											const unsigned value);
EXTSERIALPORTSHARED_EXPORT int  extserial_port_set_hw_flowcontrol(const extserial_port_handle_t hndl,
												 bool enable);

EXTSERIALPORTSHARED_EXPORT long extserial_port_data_available(const extserial_port_handle_t hndl);

EXTSERIALPORTSHARED_EXPORT long extserial_port_read(const extserial_port_handle_t hndl,
										 unsigned char *data, const long maxSize);
EXTSERIALPORTSHARED_EXPORT long extserial_port_write(const extserial_port_handle_t hndl,
										 const unsigned char *data, const long maxSize);

// Close the port (if open) and free the state structure allocated in open
EXTSERIALPORTSHARED_EXPORT void extserial_port_close(extserial_port_handle_t hndl);

#ifdef __cplusplus
}
#endif

#endif // EXTSERIALPORT_H
