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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

#include "extserialport.h"

#ifndef _WINDOWS
#include <termios.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#endif

#undef  CBAUD
#ifndef _WINDOWS

typedef struct termios termios_t;

// Data received during the init may be corrupted

static int enable_port(extserial_port_handle_t hndl)
{
	termios_t comm;
	if(tcgetattr(hndl->fd, &comm) < 0)
		return -1;
	comm.c_cflag  |= CREAD;
	return tcsetattr(hndl->fd, TCSAFLUSH, &comm);
}
#else

// At least initially no-op function in windows
static int enable_port(extserial_port_handle_t hndl)
{
	return hndl->fd == INVALID_HANDLE_VALUE?-1:0;
}

static int get_config(extserial_port_handle_t hndl, LPCOMMCONFIG config)
{
	unsigned long confSize = sizeof(COMMCONFIG);
	memset(config, 0, confSize);
	config->dwSize = confSize;
	if(!GetCommConfig(hndl->fd, config, &confSize))
	{
		hndl->last_error = GetLastError();
		return -1;
	}
	return 0;
}

static int set_config(extserial_port_handle_t hndl, LPCOMMCONFIG config)
{
	if(!SetCommConfig(hndl->fd, config, sizeof(COMMCONFIG)))
	{
		hndl->last_error = GetLastError();
		return -1;
	}
	return 0;
}
#endif

static int init_port(extserial_port_handle_t hndl)
{	
#ifndef _WINDOWS
	termios_t comm;
	if(tcgetattr(hndl->fd, &comm) < 0)
		return -1;
	// Ignore modem control lines
	comm.c_cflag    |=  CLOCAL;
	// Disble read and reset vairous other static config bits
	// These lines are copied from QExtSerialPort
	comm.c_cflag    &= (~(CREAD|CLOCAL));
	comm.c_lflag    &= (~(ICANON|ECHO|ECHOE|ECHOK|ECHONL|ISIG));
	comm.c_iflag    &= (~(INPCK|IGNPAR|IGNBRK|PARMRK|ISTRIP|ICRNL|IXANY));
	comm.c_oflag    &= (~OPOST);
	comm.c_cc[VMIN]  = 0;
	comm.c_cc[VINTR] = _POSIX_VDISABLE;
	comm.c_cc[VQUIT] = _POSIX_VDISABLE;
	comm.c_cc[VSTART]= _POSIX_VDISABLE;
	comm.c_cc[VSTOP] = _POSIX_VDISABLE;
	comm.c_cc[VSUSP] = _POSIX_VDISABLE;
	// The interface is simplified by supporting only the needed port config of ...
	// 8 databists
	comm.c_cflag  &=  (~CSIZE);
	comm.c_cflag  |=  CS8;
	// No parity
	comm.c_cflag  &=  (~PARENB);
	// 1 stop bit
	comm.c_cflag  &=  (~CSTOPB);
	return tcsetattr(hndl->fd, TCSAFLUSH, &comm);
#else
	COMMCONFIG commConfig;

	if(get_config(hndl, &commConfig) < 0)
		return -1;

	commConfig.dcb.fBinary = TRUE;
	commConfig.dcb.fAbortOnError = FALSE;
	commConfig.dcb.fNull = FALSE;
	// as for posix part, 8N1
	commConfig.dcb.fParity = FALSE;
	commConfig.dcb.Parity  = NOPARITY;
	commConfig.dcb.StopBits = ONESTOPBIT;
	commConfig.dcb.ByteSize = 8;
	commConfig.dcb.fOutxDsrFlow = FALSE;
	commConfig.dcb.fInX = FALSE;
	commConfig.dcb.fOutX = FALSE;
	commConfig.dcb.fOutxCtsFlow = FALSE;
	commConfig.dcb.fRtsControl  = RTS_CONTROL_DISABLE;
	return set_config(hndl, &commConfig);
#endif
}

static void port_close(extserial_port_handle_t hndl)
{
	if(hndl->fd != EXTSERIALPORT_INVALID_HANLDLE_V)
	{
#ifndef _WINDOWS
		close(hndl->fd);
#else
		FlushFileBuffers(hndl->fd);
		CloseHandle(hndl->fd);
#endif
	}
	free(hndl);
}

EXTSERIALPORTSHARED_EXPORT
void extserial_port_close(extserial_port_handle_t hndl)
{
	return port_close(hndl);
}

EXTSERIALPORTSHARED_EXPORT
long extserial_port_write(const extserial_port_handle_t hndl,
			   const unsigned char *data, const long maxSize)
{
	int ret, err = 0;
	if(hndl->fd < 0)
		return -2;

//	fwrite(data,maxSize,1,stdout);

#ifndef _WINDOWS
	if((ret = write(hndl->fd, data, maxSize)) < 0)
		err = errno;
#else
	DWORD bytesWritten = -1;
	if(WriteFile(hndl->fd, data, maxSize, &bytesWritten, NULL))
		ret = (int) bytesWritten;
	else
	{
		ret = -1;
		err = GetLastError();
	}
#endif
	if(ret < 0)
	{
		hndl->errors++;
		hndl->last_error = err;
	}
	else
	{
		hndl->msg_sent++;
		hndl->data_sent += ret;
	}
	return ret;
}

EXTSERIALPORTSHARED_EXPORT
long extserial_port_read(const extserial_port_handle_t hndl,
			  unsigned char *data, const long maxSize)
{
	int ret, err = 0;
	if(hndl->fd < 0)
		return -2;	
#ifndef _WINDOWS
	if((ret = read(hndl->fd, (void*)data, maxSize)) < 0)
		err = errno;
#else
	DWORD bytesRead = -1;
	if(ReadFile(hndl->fd, (void*)data, maxSize, &bytesRead, NULL))
		ret = (int) bytesRead;
	else
	{
		ret = -1;
		err = GetLastError();
	}
#endif
	if(ret < 0)
	{
		hndl->errors++;
		hndl->last_error = err;
	}
	else
	{
//		puts(data);
		hndl->msg_received++;
		hndl->data_received += ret;
	}
	return ret;
}

EXTSERIALPORTSHARED_EXPORT
long extserial_port_data_available(const extserial_port_handle_t hndl)
{
	int ret = -1;

	if(hndl->fd != EXTSERIALPORT_INVALID_HANLDLE_V)
	{
#ifndef _WINDOWS
		if(ioctl(hndl->fd, FIONREAD, &ret) < 0)
		{
			ret = -1;
			hndl->last_error = errno;
		}
#else
		COMSTAT comStat;
		DWORD errorMask = 0;
		if(!ClearCommError(hndl->fd, &errorMask, &comStat) || errorMask)
		{
			hndl->last_error = GetLastError();
			if(errorMask)
				ret = -errorMask;
		}
		else
			ret = comStat.cbInQue;
#endif
	}
	return ret;
}

EXTSERIALPORTSHARED_EXPORT
int extserial_port_set_hw_flowcontrol(extserial_port_handle_t hndl, bool enable)
{
	int ret = 0;
#ifndef _WINDOWS
	termios_t comm;
	tcgetattr(hndl->fd, &comm);
	if(enable)
	{
		comm.c_cflag|=CRTSCTS;
		comm.c_iflag&=(~(IXON|IXOFF|IXANY));
	}
	else // No flow control
	{
		comm.c_cflag&=(~CRTSCTS);
		comm.c_iflag&=(~(IXON|IXOFF|IXANY));
	}
	if((ret = tcsetattr(hndl->fd, TCSAFLUSH, &comm)) < 0)
		hndl->last_error = errno;
#else
	COMMCONFIG commConfig;
	if((ret = get_config(hndl, &commConfig)) < 0)
		return ret;

	commConfig.dcb.fOutxDsrFlow = FALSE;
	commConfig.dcb.fInX = FALSE;
	commConfig.dcb.fOutX = FALSE;
	if(enable)
	{
		commConfig.dcb.fOutxCtsFlow = TRUE;
		commConfig.dcb.fRtsControl  = RTS_CONTROL_HANDSHAKE;
	}
	else
	{
		commConfig.dcb.fOutxCtsFlow = FALSE;
		commConfig.dcb.fRtsControl  = RTS_CONTROL_DISABLE;
	}
	ret = set_config(hndl, &commConfig);
#endif
	return ret;
}

static int set_timeout(extserial_port_handle_t hndl, unsigned value)
{
	int ret = 0;
#ifndef _WINDOWS
	termios_t comm;
	tcgetattr(hndl->fd, &comm);
	comm.c_cc[VTIME] = value/100;
	if((ret = tcsetattr(hndl->fd, TCSAFLUSH, &comm)) < 0)
		hndl->last_error = errno;
#else
	COMMTIMEOUTS commTimeOuts;
	memset(&commTimeOuts, 0, sizeof(commTimeOuts));
	commTimeOuts.ReadTotalTimeoutConstant = value;
	commTimeOuts.WriteTotalTimeoutConstant = value;
	if(!SetCommTimeouts(hndl->fd, &commTimeOuts))
		ret = -1;
#endif
	return ret;
}

EXTSERIALPORTSHARED_EXPORT
int extserial_port_set_timeout(extserial_port_handle_t hndl, unsigned value)
{
	return set_timeout(hndl, value);
}

// No all possible values included here for linux
static int set_baudrate(const extserial_port_handle_t hndl, const unsigned value)
{
#ifndef _WINDOWS
	int baud;
	termios_t comm;
	switch(value)
	{
		case 300:
			baud = B300;
			break;
		case 600:
			baud = B600;
			break;
		case 1200:
			baud = B1200;
			break;
		case 2400:
			baud = B2400;
			break;
		case 4800:
			baud = B4800;
			break;
		case 9600:
			baud = B9600;
			break;
		case 19200:
			baud = B19200;
			break;
		case 38400:
			baud = B38400;
			break;
		case 57600:
			baud = B57600;
			break;
		case 115200:
			baud = B115200;
			break;
		case 230400:
			baud = B230400;
			break;
#ifndef WITHOUT_HI_SERIAL_PORT_SPEEDS
		case 1000000:
			baud = B1000000;
			break;
		case 4000000:
			baud = B4000000;
			break;
#endif
		default:
			baud = -1;
			break;
	}

	if(baud > 0)
	{
		tcgetattr(hndl->fd, &comm);
#ifdef CBAUD
		comm.c_cflag&=(~CBAUD);
		comm.c_cflag|=baud;
#else
		cfsetispeed(&comm, baud);
		cfsetospeed(&comm, baud);
#endif
		if(tcsetattr(hndl->fd, TCSAFLUSH, &comm) < 0)
		{
			hndl->last_error = errno;
			return -2;
		}
	}
	return baud;
#else
	COMMCONFIG commConfig;

	if(value < 300)
		return -1;
	if(get_config(hndl, &commConfig) < 0)
		return -2;
	commConfig.dcb.BaudRate = (DWORD) value;
	if(set_config(hndl, &commConfig) < 0)
		return -3;
	return value;
#endif
}

EXTSERIALPORTSHARED_EXPORT
int extserial_port_set_baudrate(const extserial_port_handle_t hndl, const unsigned value)
{
	return set_baudrate(hndl, value);
}

EXTSERIALPORTSHARED_EXPORT
extserial_port_handle_t extserial_port_open(const char *port)
{
	extserial_port_state_t *state;
	state = malloc(sizeof(extserial_port_state_t));
	memset(state, 0, sizeof(extserial_port_state_t));

#ifndef _WINDOWS
    if((state->fd = open(port?port:EXTSERIALPORT_DEF_PORT, O_RDWR)) < 0)
	{
		free(state);
		return NULL;
	}
#else
	if((state->fd = CreateFileA(port, GENERIC_READ|GENERIC_WRITE, 0, 0, OPEN_EXISTING,
					0 , 0)) == EXTSERIALPORT_INVALID_HANLDLE_V)
	{            // FILE_FLAG_OVERLAPPED, 0))
		free(state);
		return NULL;
	}
#endif
	if(init_port(state) < 0 ||
		set_baudrate(state, EXTSERIALPORT_DEF_BAUDRATE) < 0 ||
		set_timeout(state, EXTSERIALPORT_DEF_TIMEOUT) < 0   ||
		enable_port(state) < 0)
	{
		port_close(state);
		return NULL;
	}
	return state;
}
