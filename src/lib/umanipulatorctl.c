/*
 * A software development kit for Sensapex Micromanipulator
 *
 * Copyright (c) 2012-2015, Sensapex Oy
 * All rights reserved.
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

#include "umanipulatorctl.h"
#include "extserialport.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/timeb.h>

#include "common.h"
#include "crc.h"

#define VERSION_STR "v0.984"
#define COPYRIGHT "Copyright (c) Sensapex 2012-2015. All rights reserved"
#define umanipulatorctl_state_size sizeof(umanipulatorctl_state_t)

const char rcsid[] = "$Id: Micro manipulator interface "VERSION_STR" "__DATE__" "COPYRIGHT" Exp $";

const char *umanipulatorctl_get_version() { return VERSION_STR; }

#ifdef _WINDOWS
HRESULT __stdcall DllRegisterServer(void) { return S_OK; }
HRESULT __stdcall DllUnregisterServer(void) { return S_OK; }
#endif

// Function pointer prototype for get_position ext.
typedef int (*get_pos_ext)(umanipulatorctl_state_t *, const unsigned char, const unsigned, unsigned int *);

// Local functions to used with reading actuator positions
static int do_umanipulatorctl_x_position_ext(umanipulatorctl_state_t * hndl,
                                          const unsigned char dev,
                                          const unsigned time_limit,
                                          unsigned int *elapsedptr);

static int do_umanipulatorctl_y_position_ext(umanipulatorctl_state_t * hndl,
                                          const unsigned char dev,
                                          const unsigned time_limit,
                                          unsigned int *elapsedptr);

static int do_umanipulatorctl_z_position_ext(umanipulatorctl_state_t * hndl,
                                          const unsigned char dev,
                                          const unsigned time_limit,
                                          unsigned int *elapsedptr);

// Variables holding the pointer to currently used position getters
static get_pos_ext get_pos_x_func_ptr = &do_umanipulatorctl_x_position_ext;
static get_pos_ext get_pos_y_func_ptr = &do_umanipulatorctl_y_position_ext;
static get_pos_ext get_pos_z_func_ptr = &do_umanipulatorctl_z_position_ext;


static int baseX2long(const unsigned char *dataBuffer, unsigned char offset);
static unsigned char get_dev_index(const unsigned char dev);
// This flag indicates the used axis naming
static int g_useMCUaxis = 1;

void umanipulatorctl_setAxisNaming(umanipulatorctl_state_t *hndl, const int mcuAxisNaming)
{
    // Set the global status flag
    g_useMCUaxis = mcuAxisNaming;

    if (g_useMCUaxis)
    {
        // MCU axis naming selected
        get_pos_x_func_ptr = &do_umanipulatorctl_x_position_ext;
        get_pos_y_func_ptr = &do_umanipulatorctl_y_position_ext;
        get_pos_z_func_ptr = &do_umanipulatorctl_z_position_ext;
    }
    else
    {
        // CU axis naming selected
        get_pos_x_func_ptr = &do_umanipulatorctl_z_position_ext;
        get_pos_y_func_ptr = &do_umanipulatorctl_x_position_ext;
        get_pos_z_func_ptr = &do_umanipulatorctl_y_position_ext;
    }

    // reset local position caches
    if (hndl)
    {
        int i = 0;
        for (i = 0; i < UMANIPULATORCTL_MAX_MANIPULATORS; i++)
        {
            umanipulatorctl_positions_t *positions = &hndl->last_positions[i];
            umanipulatorctl_positions_t *targetpositions = &hndl->target_positions[i];

            positions->x = 0;
            positions->x_last_updated = 0;
            positions->y = 0;
            positions->y_last_updated = 0;
            positions->z = 0;
            positions->z_last_updated = 0;

            targetpositions->x = 0;
            targetpositions->x_last_updated = 0;
            targetpositions->y = 0;
            targetpositions->y_last_updated = 0;
            targetpositions->z = 0;
            targetpositions->z_last_updated = 0;
        }
    }
}

static unsigned long int get_timestamp()
{
    struct timeb ts;
    ftime(&ts);
    return (unsigned long int)(ts.time*1000L+ts.millitm);
}

static unsigned long int get_elapsed(const unsigned long ts)
{
    return get_timestamp() - ts;
}

int  umanipulatorctl_last_os_error(const umanipulatorctl_state_t *hndl)
{
    if(!hndl || !hndl->port)
        return -1;
    return hndl->port->last_error;
}

static const char *get_errorstr(const int error_code, char *buf, size_t buf_size)
{
#ifndef _WINDOWS
    if(strerror_r(error_code, buf, buf_size) < 0)
        snprintf(buf, buf_size, "Unknown error %d", error_code);
#else
    LPCWSTR wcBuffer = NULL;
//  LPCWSTR wcBuffer = NULL;
    if(!FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER |
                     FORMAT_MESSAGE_FROM_SYSTEM |FORMAT_MESSAGE_IGNORE_INSERTS,
                     NULL, error_code, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                     (LPWSTR)&wcBuffer, 0, NULL)  || !buf)
        snprintf(buf, buf_size, "Unknown error %d", error_code);
    else
        WideCharToMultiByte(CP_UTF8, 0, wcBuffer, -1, buf, buf_size, NULL, NULL);
    if(wcBuffer)
        LocalFree((LPWSTR)wcBuffer);
#endif
    return buf;
}

static int is_valid_mode(const int mode)
{
    int retval = 1;

    if (mode < UMANIPULATORCTL_MODE_1 || mode > UMANIPULATORCTL_MODE_SNAIL)
        retval = 0;

    return retval;
}

static int is_valid_storage(const int storage)
{
    int retval = 1;

    if (storage < 0 || storage > 3)
        retval = 0;

    return retval;
}

const char *umanipulatorctl_last_os_errorstr(umanipulatorctl_state_t *hndl)
{
    if(!hndl)
        return umanipulatorctl_errorstr(UMANIPULATORCTL_NOT_OPEN);
    return get_errorstr(hndl->port->last_error, hndl->errorstr_buffer,
                        sizeof(hndl->errorstr_buffer));
}

umanipulatorctl_error_t umanipulatorctl_last_error(const umanipulatorctl_state_t *hndl)
{
    if(!hndl)
        return UMANIPULATORCTL_NOT_OPEN;
    return hndl->last_error;
}

const char *umanipulatorctl_last_errorstr(umanipulatorctl_state_t *hndl)
{
    static char open_errorstr[80];

    // Special handling for NULL handle
    if(!hndl)
    {
    #ifndef _WINDOWS
        int error_code = errno;
    #else
        int error_code = GetLastError();
    #endif
        if(!error_code)
            umanipulatorctl_errorstr(UMANIPULATORCTL_NOT_OPEN);
        return get_errorstr(error_code, open_errorstr, sizeof(open_errorstr));
    }

    if(hndl->last_error == UMANIPULATORCTL_OS_ERROR)
        return umanipulatorctl_last_os_errorstr(hndl);
    return umanipulatorctl_errorstr(hndl->last_error);
}

const char *umanipulatorctl_errorstr(const int ret_code)
{
    const char *errorstr;
    if(ret_code >= 0)
        return "No error";
    switch(ret_code)
    {
        case UMANIPULATORCTL_OS_ERROR:
            errorstr = "Operation system error";
            break;
        case UMANIPULATORCTL_NOT_OPEN:
            errorstr = "Not opened";
            break;
        case UMANIPULATORCTL_TIMEOUT:
            errorstr = "Timeout";
            break;
        case UMANIPULATORCTL_INVALID_ARG:
            errorstr = "Invalid argument";
            break;
        case UMANIPULATORCTL_INVALID_DEV:
            errorstr = "Invalid device id";
            break;
        case UMANIPULATORCTL_INVALID_RESP:
            errorstr = "Invalid response";
            break;
        case UMANIPULATORCTL_INVALID_CRC:
            errorstr = "CRC does not match";
            break;
        default:
            errorstr = "Unknown error";
            break;
    }
    return errorstr;
}

static int is_invalid_dev(const unsigned char dev)
{
    if(dev > 0x30 && dev <= 0x40)
        return 0;
    return UMANIPULATORCTL_INVALID_DEV;
}

umanipulatorctl_state_t * umanipulatorctl_open(const char *port,
                                               const unsigned int timeout)
{
    umanipulatorctl_state_t *hndl;
    if(!port)
        return NULL;
    if(!(hndl = malloc(umanipulatorctl_state_size)))
        return NULL;
    memset(hndl, 0, umanipulatorctl_state_size);
    hndl->refresh_time_limit = UMANIPULATORCTL_DEF_REFRESH_TIME;
#ifdef _WINDOWS
    // Syntax "\\.\COM12" needed for ports above COM9
    char portName[10];
    if(*port != '\\' && strlen(port) < 6)
    {
        strcpy(portName,"\\\\.\\");
        strcat(portName, port);
        port = NULL;
    }
    if(!(hndl->port = extserial_port_open(port?port:portName)))
#else
    if(!(hndl->port = extserial_port_open(port)))
#endif
    {
        free(hndl);
        return NULL;
    }
    if(timeout > 0)
        extserial_port_set_timeout(hndl->port, timeout);
    return hndl;
}

void umanipulatorctl_close(umanipulatorctl_state_t *hndl)
{
    if(!hndl)
        return;
    if(hndl->port)
        extserial_port_close(hndl->port);
    free(hndl);
}

static int set_last_error(umanipulatorctl_state_t *hndl, int code)
{
    if(hndl)
        hndl->last_error = code;
    return code;
}

int umanipulatorctl_set_timeout(umanipulatorctl_state_t *hndl, const unsigned int value)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(value > 10000)
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_ARG);
    return extserial_port_set_timeout(hndl->port, value);
}


int umanipulatorctl_select_dev(umanipulatorctl_state_t *hndl,
                               const unsigned char dev)
{
    int ret;
    int mcu_major, mcu_minor, hw_id;
    umanipulatorctl_mcu_info_t *info;

    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    // Ping the device
    if((ret = umanipulatorctl_ping(hndl, dev)) < 0)
        return ret;
    // read the mcu version
    if ((ret = umanipulatorctl_read_version(hndl, &mcu_major, &mcu_minor)) < 0)
        return ret;
    // Read the hw id
    hw_id = umanipulatorctl_read(hndl, dev, HW_ID_REG);
    if ((hw_id & 0xffff) == 0xffff)
        hw_id = 0;

    info = &hndl->mcu_info[get_dev_index(dev)];

    info->mcu_ver_major = mcu_major;
    info->mcu_ver_minor = mcu_minor;
    info->mcu_hw_id = hw_id;

    // Store datas in handle
    hndl->last_device_sent = dev;

    return 0;
}

int umanipulatorctl_set_refresh_time_limit(umanipulatorctl_state_t * hndl,
                                        const unsigned value)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(value > 60000)
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_ARG);
    hndl->refresh_time_limit = value;
    return 0;
}

int  umanipulatorctl_is_busy(umanipulatorctl_state_t *hndl)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(hndl->last_device_sent))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    return umanipulatorctl_is_busy_ext(hndl, hndl->last_device_sent);
}

int  umanipulatorctl_is_busy_status(umanipulatorctl_status_t status)
{
    if(status < 0)
        return -1;
    // Any bit in the lower byte of the status value means busy (after MCU 1.172)
    if(status&0xff)
        return 1;
    return 0;
}

int  umanipulatorctl_is_busy_ext(umanipulatorctl_state_t *hndl, const unsigned char dev)
{
    int status = umanipulatorctl_status_ext(hndl, dev);
    return umanipulatorctl_is_busy_status(status);
}

umanipulatorctl_status_t  umanipulatorctl_status(umanipulatorctl_state_t *hndl)
{
    if(!hndl) {
        set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
        return UMANIPULATORCTL_STATUS_READ_ERROR;
    }
    if(is_invalid_dev(hndl->last_device_sent)) {
        set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
        return UMANIPULATORCTL_STATUS_READ_ERROR;
    }

    int status = umanipulatorctl_status_ext(hndl, hndl->last_device_sent);

    if (status <= UMANIPULATORCTL_STATUS_READ_ERROR)
        return UMANIPULATORCTL_STATUS_READ_ERROR;

    return (umanipulatorctl_status_t)status;
}

int  umanipulatorctl_status_ext(umanipulatorctl_state_t *hndl, const unsigned char dev)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    return umanipulatorctl_read(hndl,dev, STATUS_REG);
}

int umanipulatorctl_x_position(umanipulatorctl_state_t * hndl)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(hndl->last_device_sent))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    return umanipulatorctl_x_position_ext(hndl, hndl->last_device_sent,
                                          hndl->refresh_time_limit, NULL);
}

int umanipulatorctl_y_position(umanipulatorctl_state_t * hndl)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(hndl->last_device_sent))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    return umanipulatorctl_y_position_ext(hndl, hndl->last_device_sent,
                                          hndl->refresh_time_limit, NULL);
}

int umanipulatorctl_z_position(umanipulatorctl_state_t * hndl)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(hndl->last_device_sent))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    return umanipulatorctl_z_position_ext(hndl, hndl->last_device_sent,
                                          hndl->refresh_time_limit, NULL);
}

int umanipulatorctl_store_mem_current_position(umanipulatorctl_state_t * hndl)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(hndl->last_device_sent))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    return umanipulatorctl_store_mem_current_position_ext(hndl, hndl->last_device_sent, 0);
}


int umanipulatorctl_store_mem_current_position_ext(umanipulatorctl_state_t * hndl,
                                                   const unsigned char dev,
                                                   const unsigned char storage_id)
{
    unsigned char cmd = MEM_3;
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    if(!is_valid_storage(storage_id))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_ARG);

    switch(storage_id)
    {
        case 1:
            cmd = MEM_1;
            break;
        case 2:
            cmd = MEM_2;
            break;
        case 0:
        default:
            break;
    }
    return umanipulatorctl_cmd(hndl, dev, cmd);
}

int umanipulatorctl_goto_mem_position(umanipulatorctl_state_t * hndl)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(hndl->last_device_sent))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    return umanipulatorctl_goto_mem_position_ext(hndl,hndl->last_device_sent, 0);
}

int umanipulatorctl_goto_mem_position_ext(umanipulatorctl_state_t * hndl,
                                          const unsigned char dev,
                                          const unsigned char storage_id)
{
    unsigned char cmd = RET_3;
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    if(!is_valid_storage(storage_id))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_ARG);

    switch(storage_id)
    {
        case 1:
            cmd = RET_1;
            break;
        case 2:
            cmd = RET_2;
            break;
        default:
            break;
    }
    return umanipulatorctl_cmd(hndl, dev, cmd);
}

int umanipulatorctl_store_mem_position(umanipulatorctl_state_t * hndl,
                                       const int x, const int y, const int z)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(hndl->last_device_sent))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    return umanipulatorctl_store_mem_position_ext(hndl,hndl->last_device_sent,x,y,z,0);
}

static unsigned char get_dev_index(const unsigned char dev)
{
    return dev - 0x31;
}

static int is_invalid_pos(int pos)
{
    if(pos < 0 || pos > UMANIPULATORCTL_MAX_POSITION)
        return UMANIPULATORCTL_INVALID_ARG;
    return 0;
}

int umanipulatorctl_store_mem_position_ext(umanipulatorctl_state_t * hndl,
                                           const unsigned char dev,
                                           const int _x, const int _y, const int _z,
                                           const unsigned char storage_id)
{
    umanipulatorctl_positions_t *positions;
    int ret;
    unsigned char addr;
    int mcu_x, mcu_y, mcu_z;

    if (g_useMCUaxis)
    {
        mcu_x = _x;
        mcu_y = _y;
        mcu_z = _z;
    }
    else
    {
        mcu_x = _y;
        mcu_y = _z;
        mcu_z = _x;
    }

    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    if(!is_valid_storage(storage_id))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_ARG);

    positions = &hndl->target_positions[get_dev_index(dev)];
    if(!is_invalid_pos(mcu_x))
    {
        switch(storage_id)
        {
            case 1:
                addr = X_AXIS_POS_MEM_1;
                break;
            case 2:
                addr = X_AXIS_POS_MEM_2;
                break;
            default:
                addr = X_AXIS_POS_MEM_3;
                break;
        }
        if((ret = umanipulatorctl_write(hndl, dev, addr, mcu_x)) < 0)
            return ret;
        positions->x = mcu_x;
        positions->x_last_updated = get_timestamp();
    }

    if(!is_invalid_pos(mcu_y))
    {
        switch(storage_id)
        {
            case 1:
                addr = Y_AXIS_POS_MEM_1;
                break;
            case 2:
                addr = Y_AXIS_POS_MEM_2;
                break;
            default:
                addr = Y_AXIS_POS_MEM_3;
                break;
        }
        if((ret = umanipulatorctl_write(hndl, dev, addr, mcu_y)) < 0)
            return ret;
        positions->y = mcu_y;
        positions->y_last_updated = get_timestamp();
    }

    if(!is_invalid_pos(mcu_z))
    {
        switch(storage_id)
        {
            case 1:
                addr = Z_AXIS_POS_MEM_1;
                break;
            case 2:
                addr = Z_AXIS_POS_MEM_2;
                break;
            default:
                addr = Z_AXIS_POS_MEM_3;
                break;
        }
        if((ret = umanipulatorctl_write(hndl, dev, addr, mcu_z)) < 0)
            return ret;
        positions->z = mcu_z;
        positions->z_last_updated = get_timestamp();
    }

    return 0;
}

int umanipulatorctl_get_position(umanipulatorctl_state_t * hndl,
                                  int *x, int *y, int *z)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);;
    if(is_invalid_dev(hndl->last_device_sent))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    return umanipulatorctl_get_position_ext(hndl, hndl->last_device_sent,
                                            hndl->refresh_time_limit,
                                            x, y, z);
}

int umanipulatorctl_get_position_ext(umanipulatorctl_state_t * hndl,
                                         const unsigned char dev,
                                         const unsigned int  time_limit,
                                         int *x, int *y, int *z)
{
    int val_x = -1, val_y = -1, val_z = -1;
    int ret;
    int axismask_pre = 0;
    int axismask_post = 0;

    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);

    if (x)
        axismask_pre |= 1;
    if (y)
        axismask_pre |= 2;
    if (z)
        axismask_pre |= 4;

    // TO BE CHECKED: How to diffrentiate negative coordinates and errors?
    if(x && (val_x = umanipulatorctl_x_position_ext(hndl,dev,time_limit,NULL)) >= 0)
        axismask_post |= 1;
    if(y && (val_y = umanipulatorctl_y_position_ext(hndl,dev,time_limit,NULL)) >= 0)
        axismask_post |= 2;
    if(z && (val_z = umanipulatorctl_z_position_ext(hndl,dev,time_limit,NULL)) >= 0)
        axismask_post |= 4;

    if (axismask_pre == axismask_post)
    {
        if (x)
            *x = val_x;
        if (y)
            *y = val_y;
        if (z)
            *z = val_z;

        ret = axismask_post;

    }
    else
        ret = -(axismask_pre ^= axismask_post);

    return ret;
}

int umanipulatorctl_x_position_ext(umanipulatorctl_state_t * hndl,
                                    const unsigned char dev,
                                    const unsigned time_limit,
                                    unsigned int *elapsedptr)
{
    return get_pos_x_func_ptr(hndl, dev, time_limit, elapsedptr);
}

int do_umanipulatorctl_x_position_ext(umanipulatorctl_state_t * hndl,
                                    const unsigned char dev,
                                    const unsigned time_limit,
                                    unsigned int *elapsedptr)
{
    umanipulatorctl_positions_t *positions;
    int ret;
    unsigned long start;
    unsigned elapsed;

    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    positions = &hndl->last_positions[get_dev_index(dev)];
    elapsed = get_elapsed(positions->x_last_updated);
    if((elapsed < time_limit || !time_limit) && time_limit != UMANIPULATORCTL_TIMELIMIT_DISABLED)
        ret = positions->x;
    else
    {
        start = get_timestamp();
        hndl->last_device_position_read_received = dev;
        ret = umanipulatorctl_read_position(hndl,dev, ACTUATOR_X_ID);
        elapsed = get_elapsed(start);
    }
    if(elapsedptr)
        *elapsedptr = elapsed;
    return ret;
}

int umanipulatorctl_y_position_ext(umanipulatorctl_state_t * hndl,
                                    const unsigned char dev,
                                    const unsigned time_limit,
                                    unsigned int *elapsedptr)
{
    return get_pos_y_func_ptr(hndl, dev, time_limit, elapsedptr);
}

int do_umanipulatorctl_y_position_ext(umanipulatorctl_state_t * hndl,
                                    const unsigned char dev,
                                    const unsigned time_limit,
                                    unsigned int *elapsedptr)
{
    umanipulatorctl_positions_t *positions;
    int ret;
    unsigned long start;
    unsigned elapsed;

    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);;
    if(is_invalid_dev(dev))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    positions = &hndl->last_positions[get_dev_index(dev)];
    elapsed = get_elapsed(positions->y_last_updated);
    if((elapsed < time_limit || !time_limit) && time_limit != UMANIPULATORCTL_TIMELIMIT_DISABLED)
        ret = positions->y;
    else
    {
        start = get_timestamp();
        hndl->last_device_position_read_received = dev;
        ret = umanipulatorctl_read_position(hndl, dev, ACTUATOR_Y_ID);
        elapsed = get_elapsed(start);
//      positions->y_last_updated = start;
    }
    if(elapsedptr)
        *elapsedptr = elapsed;
    return ret;
}

int umanipulatorctl_z_position_ext(umanipulatorctl_state_t * hndl,
                                    const unsigned char dev,
                                    const unsigned time_limit,
                                    unsigned int *elapsedptr)
{
    return get_pos_z_func_ptr(hndl, dev, time_limit, elapsedptr);
}

int do_umanipulatorctl_z_position_ext(umanipulatorctl_state_t * hndl,
                                    const unsigned char dev,
                                    const unsigned time_limit,
                                    unsigned int *elapsedptr)
{
    umanipulatorctl_positions_t *positions;
    int ret;
    unsigned long start;
    unsigned elapsed;

    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    positions = &hndl->last_positions[get_dev_index(dev)];
    elapsed = get_elapsed(positions->z_last_updated);
    if((elapsed < time_limit || !time_limit) && time_limit != UMANIPULATORCTL_TIMELIMIT_DISABLED)
        ret = positions->z;
    else
    {
        start = get_timestamp();
        hndl->last_device_position_read_received = dev;
        ret = umanipulatorctl_read_position(hndl, dev, ACTUATOR_Z_ID);
        elapsed = get_elapsed(start);
//      positions->z_last_updated = start;
    }
    if(elapsedptr)
        *elapsedptr = elapsed;
    return ret;
}

int umanipulatorctl_get_mem_position(umanipulatorctl_state_t * hndl,
                                     int *x, int *y, int *z)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(hndl->last_device_sent))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    return umanipulatorctl_get_mem_position_ext(hndl, hndl->last_device_sent, x, y, z, 0);
}

int umanipulatorctl_stop(umanipulatorctl_state_t * hndl)
{
    return umanipulatorctl_stop_ext(hndl, hndl->last_device_sent);
}

int umanipulatorctl_stop_ext(umanipulatorctl_state_t * hndl,
                             const unsigned char dev)
{
    return umanipulatorctl_cmd(hndl, dev, STOP_DRIVE);
}

int umanipulatorctl_set_control_unit_pcmode(umanipulatorctl_state_t * hndl,
                                            const int enablePCMode)
{
    unsigned char command;
    int retVal;
    unsigned char current_device = hndl->last_device_sent;

    if (enablePCMode)
        command = PC_MODE_RESERVE;
    else
        command = PC_MODE_RELEASE;

    retVal = umanipulatorctl_cmd(hndl, MULTICAST_DEV_ID, command);

    hndl->last_device_sent = current_device;

    if (retVal >= 0)
    {
        retVal = umanipulatorctl_status(hndl);

        if (retVal >= 0)
        {
            // Check that the mode has been set successfully
            int pcmode_enabled = (retVal & STATUS_PC_MODE);

            if ((enablePCMode && !pcmode_enabled) ||
                (!enablePCMode && pcmode_enabled))
            {
                retVal = -1;
            }
        }
    }

    return retVal;
}

static int read_mem_position(umanipulatorctl_state_t * hndl,
                             const unsigned char dev,
                             int *x, int *y, int *z,
                             const unsigned char storage_id)
{
    int ret;

    switch(storage_id)
    {
        case 1:
            if((ret = umanipulatorctl_read(hndl,dev,X_AXIS_POS_MEM_1)) < 0)
                return ret;
            *x = (short)ret;
            if((ret = umanipulatorctl_read(hndl,dev,Y_AXIS_POS_MEM_1)) < 0)
                return ret;
            *y = (short)ret;
            if((ret = umanipulatorctl_read(hndl,dev,Z_AXIS_POS_MEM_1)) < 0)
                return ret;
            *z = (short)ret;
            break;
        case 2:
            if((ret = umanipulatorctl_read(hndl,dev,X_AXIS_POS_MEM_2)) < 0)
                return ret;
            *x = (short)ret;
            if((ret = umanipulatorctl_read(hndl,dev,Y_AXIS_POS_MEM_2)) < 0)
                return ret;
            *y = (short)ret;
            if((ret = umanipulatorctl_read(hndl,dev,Z_AXIS_POS_MEM_2)) < 0)
                return ret;
            *z = (short)ret;
            break;
        default:
            if((ret = umanipulatorctl_read(hndl,dev,X_AXIS_POS_MEM_3)) < 0)
                return ret;
            *x = (short)ret;
            if((ret = umanipulatorctl_read(hndl,dev,Y_AXIS_POS_MEM_3)) < 0)
                return ret;
            *y = (short)ret;
            if((ret = umanipulatorctl_read(hndl,dev,Z_AXIS_POS_MEM_3)) < 0)
                return ret;
            *z = (short)ret;
            break;
    }
    return storage_id;
}


int umanipulatorctl_get_mem_position_ext(umanipulatorctl_state_t * hndl,
                                         const unsigned char dev,
                                         int *_x, int *_y, int *_z,
                                         const unsigned char storage_id)
{
    umanipulatorctl_positions_t *positions;
    int cnt = 0;
    int *mcu_x, *mcu_y, *mcu_z;
    if (g_useMCUaxis)
    {
        mcu_x = _x;
        mcu_y = _y;
        mcu_z = _z;
    }
    else
    {
        mcu_x = _y;
        mcu_y = _z;
        mcu_z = _x;
    }

    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    if(!is_valid_storage(storage_id))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_ARG);

    if(storage_id)
        return read_mem_position(hndl, dev, mcu_x, mcu_y, mcu_z, storage_id);
    // Default function is to return the target position from local copy
    positions = &hndl->target_positions[get_dev_index(dev)];
    if(mcu_x && positions->x_last_updated)
    {
        *mcu_x = positions->x;
        cnt++;
    }
    if(mcu_y && positions->y_last_updated)
    {
        *mcu_y = positions->y;
        cnt++;
    }
    if(mcu_z && positions->z_last_updated)
    {
        *mcu_z = positions->z;
        cnt++;
    }
    return cnt;
}

static void msg_frame_clear(msg_frame_t *msg)
{
    memset(msg,0x30,sizeof(msg_frame_t));
}

static void msg_frame_init(msg_frame_t *msg)
{
    msg_frame_clear(msg);
    msg->start = STX;
    msg->end = ETX;
    msg->device = '1';
}

static int msg_send(umanipulatorctl_state_t *hndl, msg_frame_t *msg)
{
    int ret;
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);;
    add_frame_crc(msg);
    ret = extserial_port_write(hndl->port, (const unsigned char*)msg, msg_frame_size);
    if(ret > 0)
        hndl->last_device_sent = msg->device;
    if(ret < 0)
        return set_last_error(hndl, UMANIPULATORCTL_OS_ERROR);
    return ret;
}

static int get_address(const msg_frame_t *msg, int *address)
{
    char buffer[3];
    if(!msg || !address)
        return UMANIPULATORCTL_INVALID_ARG;
    sprintf(buffer, "%c%c", msg->address[0], msg->address[1]);
    if(sscanf(buffer, "%x", address) == 1)
        return 1;
    return UMANIPULATORCTL_INVALID_RESP;
}

static int get_data(const msg_frame_t *msg, int *data)
{
    char buffer[5];
    if(!msg || !data)
        return UMANIPULATORCTL_INVALID_ARG;
    if(msg->function == GET_ACTUATOR_POS_RESP)
    {
        *data = baseX2long(msg->data, 32);
        return 1;
    }
    else
    {
        sprintf(buffer, "%c%c%c%c", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
        if(sscanf(buffer, "%x", data) == 1)
            return 1;
    }
    return UMANIPULATORCTL_INVALID_RESP;
}

static int update_positions(umanipulatorctl_state_t *hndl, msg_frame_t *msg)
{
    umanipulatorctl_positions_t *positions;
    int data, addr;
    unsigned char dev;

    if(!hndl)
        return UMANIPULATORCTL_NOT_OPEN;
    if(get_address(msg, &addr) < 0)
        return UMANIPULATORCTL_INVALID_ARG;
    if(msg->function == READ_DATA_REG && (addr == X_POSITION_REG ||
             addr == Y_POSITION_REG || addr == Z_POSITION_REG))
    {
        hndl->last_device_position_read_received = msg->device;
        return 0;
    }
    if(msg->device != DEF_CU_ID)
        return UMANIPULATORCTL_INVALID_DEV;
    if(msg->function != WRITE_DATA_REG &&
       msg->function != GET_ACTUATOR_POS_RESP)
        return UMANIPULATORCTL_INVALID_RESP;

    dev = hndl->last_device_position_read_received;
    if(is_invalid_dev(dev))
        return UMANIPULATORCTL_INVALID_DEV;
    positions = &hndl->last_positions[get_dev_index(dev)];
    if(get_data(msg, &data) < 0)
        return UMANIPULATORCTL_INVALID_RESP;

    switch(addr)
    {
        case X_POSITION_REG:
            positions->x = data;
            positions->x_last_updated = get_timestamp();
            break;
        case Y_POSITION_REG:
            positions->y = data;
            positions->y_last_updated = get_timestamp();
            break;
        case Z_POSITION_REG:
            positions->z = data;
            positions->z_last_updated = get_timestamp();
            break;
        default:
            return UMANIPULATORCTL_INVALID_RESP;
    }
    return 1;
}

static int msg_recv(umanipulatorctl_state_t *hndl, msg_frame_t *msg)
{
    int ret;
    unsigned char *p = (unsigned char *)msg;

    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(!msg)
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_ARG);

    msg_frame_clear(msg);

    // Seek for STX byte
    while((ret = extserial_port_read(hndl->port, p, 1)) == 1)
    {
        if(*p == STX)
            break;
    }
    // zero means timeout, negative an error
    if(ret < 1)
    {
        if(!ret)
            return set_last_error(hndl, UMANIPULATORCTL_TIMEOUT);
        return set_last_error(hndl, UMANIPULATORCTL_OS_ERROR);
    }

    for(p++; (ret = extserial_port_read(hndl->port, p, 1)) == 1; p++)
    {
        if(*p == STX)
            return set_last_error(hndl, UMANIPULATORCTL_INVALID_RESP);
        if(*p == ETX && p != &msg->end)
            return set_last_error(hndl, UMANIPULATORCTL_INVALID_RESP);
        if(*p != ETX && p == &msg->end)
            return set_last_error(hndl, UMANIPULATORCTL_INVALID_RESP);
        if(*p == ETX && p == &msg->end)
            break;
    }
    if(ret < 1)
    {
        if(!ret)
            return set_last_error(hndl, UMANIPULATORCTL_TIMEOUT);
        return set_last_error(hndl, UMANIPULATORCTL_OS_ERROR);
    }
    if(check_frame_crc(msg) < 0)
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_CRC);

    update_positions(hndl, msg);
    hndl->last_device_received = msg->device;

    // Frame received correctly
    return 1;
}

int umanipulatorctl_recv(umanipulatorctl_state_t * hndl)
{
    int ret, cnt = 0;
    msg_frame_t resp;

    while(cnt < 10 && (ret = msg_recv(hndl, &resp)) > 0)
        cnt++;
    if(cnt > 0)
        return cnt;
    return ret;
}

static int check_resp(const msg_frame_t *msg, const msg_frame_t *resp)
{
    if(msg->function == GET_ACTUATOR_POS_REQ)
    {
        if(resp->function != GET_ACTUATOR_POS_RESP)
            return -2;
    }
    else if(msg->function != READ_DATA_REG)
    {
        if(msg->device != resp->device && msg->device != MULTICAST_DEV_ID)
            return -1;
        if(msg->function != resp->function)
            return -2;
    }
    else if(resp->function != WRITE_DATA_REG)
        return -3;
    if(memcmp(msg->address, resp->address, 2))
        return -4;
    return 1;
}

int umanipulatorctl_ping(umanipulatorctl_state_t * hndl, const unsigned char dev)
{
    int ret;
    msg_frame_t msg;
    msg_frame_t resp;
    msg_frame_init(&msg);
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    msg.device = dev;
    msg.function = READ_STATUS_REG;

    if((ret = msg_send(hndl, &msg)) < 0)
        return ret;
    while((ret = msg_recv(hndl, &resp)) > 0)
    {
        if(check_resp(&msg, &resp) > 0)
            return 1;
    }
    return ret;
}

int umanipulatorctl_cmd(umanipulatorctl_state_t * hndl, const unsigned char dev,
                        const unsigned char cmd)
{
    int ret, data;
    char buffer[5];
    msg_frame_t msg;
    msg_frame_t resp;
    msg_frame_init(&msg);
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    msg.device = dev;
    msg.function = WRITE_COMMAND_REG;
    sprintf(buffer,"%04x",(int)cmd);
    memcpy(&msg.data, buffer, 4);
    if((ret = msg_send(hndl, &msg)) < 0)
        return ret;
    while((ret = msg_recv(hndl, &resp)) > 0)
    {
        if(check_resp(&msg, &resp) > 0 &&
             get_data(&msg, &data) > 0 && data == cmd)
        {
            return 1;
        }
    }
    return ret;
}


int umanipulatorctl_read(umanipulatorctl_state_t * hndl, const unsigned char dev,
                        const unsigned char addr)
{
    int ret, data;
    char buffer[3];
    msg_frame_t msg;
    msg_frame_t resp;

    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return -2;

    msg_frame_init(&msg);
    msg.device = dev;
    msg.function = READ_DATA_REG;
    sprintf(buffer,"%02x",addr);
    msg.address[0] = buffer[0];
    msg.address[1] = buffer[1];

    if((ret = msg_send(hndl, &msg)) < 0)
        return ret;
    while((ret = msg_recv(hndl, &resp)) > 0)
    {
        if(check_resp(&msg, &resp) > 0)
        {
            if(get_data(&resp, &data) < 0)
                return set_last_error(hndl, UMANIPULATORCTL_INVALID_RESP);
            return data;
        }
    }
    return ret-10;
}

int umanipulatorctl_read_position(umanipulatorctl_state_t * hndl, const unsigned char dev,
                        const unsigned char addr)
{
    int ret, data;
    char buffer[3];
    msg_frame_t msg;
    msg_frame_t resp;
    umanipulatorctl_mcu_info_t *info;

    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return -2;

    msg_frame_init(&msg);
    msg.device = dev;

    info = &hndl->mcu_info[get_dev_index(dev)];

    if (info->mcu_ver_major > 3 || (info->mcu_ver_major == 3 && info->mcu_ver_minor >= 957))
        msg.function = GET_ACTUATOR_POS_REQ;
    else
        msg.function = READ_DATA_REG;

    sprintf(buffer,"%02x",addr);
    msg.address[0] = buffer[0];
    msg.address[1] = buffer[1];

    if((ret = msg_send(hndl, &msg)) < 0)
        return ret;
    while((ret = msg_recv(hndl, &resp)) > 0)
    {
        if(check_resp(&msg, &resp) > 0)
        {
            if(get_data(&resp, &data) < 0)
                return -4;

            if (msg.function == GET_ACTUATOR_POS_REQ)
            {
                if (!(info->mcu_hw_id & MCU_HWID_NM_POS))
                    data = data / 1000; // Convert nm scale data to um scale - just for backwards compability reasons.
            }
            else
                data = (short int) data;

            return data;
        }
    }
    return ret-10;
}

static int baseX2long(const unsigned char *dataBuffer, unsigned char offset)
{
    unsigned int retval = 0;
    int byte;
    char workbuf[9+1];
    char *ptrworkbuf = workbuf;

    memset(workbuf, '\0', sizeof(workbuf));

    for (byte = 0; byte < 4; byte++)
    {
        unsigned char byteVal = *dataBuffer;
        dataBuffer++;
        if (byte == 0)
        {
            if (byteVal & 0b10000000)
            {
                // Negative number. Handle it
                // ... Step 1: add a sign in destaintion buffer
                *ptrworkbuf = '-';
                ptrworkbuf++;
                // ... Step 2: remove the sign bit from source data
                byteVal &= 0b01111111;
            }
        }
        byteVal -= offset;
        sprintf(ptrworkbuf, "%02d", byteVal);
        ptrworkbuf += 2;
    }

    retval = atoi(workbuf);
    return retval;
}

int umanipulatorctl_write(umanipulatorctl_state_t * hndl,
                           const unsigned char dev,
                           const unsigned char addr,
                           const unsigned short val)
{
    int data, ret;
    char buffer[5];
    msg_frame_t msg;
    msg_frame_t resp;
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return -2;

    msg_frame_init(&msg);
    msg.device = dev;
    msg.function = WRITE_DATA_REG;

    sprintf(buffer,"%02x",(int)addr);
    msg.address[0] = buffer[0];
    msg.address[1] = buffer[1];
    sprintf(buffer,"%04x",(int)val);
    memcpy(&msg.data, buffer, 4);
    if(msg_send(hndl, &msg) < 0)
        return -3;
    while((ret = msg_recv(hndl, &resp)) > 0)
    {
        if(check_resp(&msg, &resp) > 0)
        {
            if(get_data(&resp, &data) < 0)
                return -4;
            return data;
        }
    }
    return ret-10;
}

int umanipulatorctl_write_speed(umanipulatorctl_state_t * hndl,
                           const unsigned char dev,
                           const unsigned char addr,
                           const unsigned short val)
{
    char buffer[5];
    msg_frame_t msg;
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return -2;

    msg_frame_init(&msg);
    msg.device = dev;
    msg.function = WRITE_SPEED_REG;

    sprintf(buffer,"%02x",(int)addr);
    msg.address[0] = buffer[0];
    msg.address[1] = buffer[1];
    sprintf(buffer,"%04x",(int)val);
    memcpy(&msg.data, buffer, 4);
    if(msg_send(hndl, &msg) < 0)
        return -3;
    return 1;
}

int umanipulatorctl_read_version(umanipulatorctl_state_t * hndl,
                                 int *major, int *minor)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);;
    if(is_invalid_dev(hndl->last_device_sent))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    return umanipulatorctl_read_version_ext(hndl,hndl->last_device_sent,
                                            major, minor);
}

int umanipulatorctl_read_version_ext(umanipulatorctl_state_t * hndl,
                                 const unsigned char dev,
                                 int *major, int *minor)
{
    int ret = 0;
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return -2;
    if(major)
    {
        if((ret = umanipulatorctl_read(hndl, dev, VERSION_MAJOR_REG)) < 0)
            return ret;
        *major = ret;
    }
    if(minor)
    {
        if((ret = umanipulatorctl_read(hndl, dev, VERSION_MINOR_REG)) < 0)
            return ret;
        *minor = ret;
    }
    return ret;
}

umanipulatorctl_mode_t umanipulatorctl_get_mode(umanipulatorctl_state_t * hndl)
{
    if(!hndl)
    {
        set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
        return UMANIPULATORCTL_MODE_READ_ERROR;
    }

    if(is_invalid_dev(hndl->last_device_sent))
    {
        set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
        return UMANIPULATORCTL_MODE_READ_ERROR;
    }

    return umanipulatorctl_get_mode_ext(hndl,hndl->last_device_sent);
}

umanipulatorctl_mode_t umanipulatorctl_get_mode_ext(umanipulatorctl_state_t * hndl,
                                                 const unsigned char dev)
{
    int ret, ver = 0;
    if(!hndl)
    {
        set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
        return UMANIPULATORCTL_MODE_READ_ERROR;
    }
    if(is_invalid_dev(dev))
        return UMANIPULATORCTL_MODE_READ_ERROR;
    if((ret = umanipulatorctl_status_ext(hndl, dev)) < 0)
        return UMANIPULATORCTL_MODE_READ_ERROR;
    if(ret&STATUS_PEN_MODE)
        return UMANIPULATORCTL_MODE_PEN;
    if(ret&STATUS_SNAIL_MODE)
        return UMANIPULATORCTL_MODE_SNAIL;
    if((ret = umanipulatorctl_read_version(hndl, &ver, NULL)) < 0)
        return ret;
    if((ret = umanipulatorctl_read(hndl, dev, INCREMENT_REG)) < 0)
        return UMANIPULATORCTL_MODE_READ_ERROR;
    if(ver < 3)
    {
        switch(ret)
        {
            case V2_SPEED1_INCREMENT: return UMANIPULATORCTL_MODE_1;
            case V2_SPEED2_INCREMENT: return UMANIPULATORCTL_MODE_2;
            case V2_SPEED3_INCREMENT: return UMANIPULATORCTL_MODE_3;
            case V2_SPEED4_INCREMENT: return UMANIPULATORCTL_MODE_4;
            case V2_SPEED5_INCREMENT: return UMANIPULATORCTL_MODE_5;
        }
    }
    else // v 3 or 4
    {
        switch(ret)
        {
            case V3_SPEED1_INCREMENT:   return UMANIPULATORCTL_MODE_1;
            case V4_SPEED1_INCREMENT:   return UMANIPULATORCTL_MODE_1;
            case V3_4_SPEED2_INCREMENT: return UMANIPULATORCTL_MODE_2;
            case V3_4_SPEED3_INCREMENT: return UMANIPULATORCTL_MODE_3;
            case V3_4_SPEED4_INCREMENT: return UMANIPULATORCTL_MODE_4;
            case V3_4_SPEED5_INCREMENT: return UMANIPULATORCTL_MODE_5;
        }
    }
    return UMANIPULATORCTL_MODE_UNKNOWN;
}

int umanipulatorctl_set_mode(umanipulatorctl_state_t * hndl,
                             const umanipulatorctl_mode_t mode)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);;
    if(is_invalid_dev(hndl->last_device_sent))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    return umanipulatorctl_set_mode_ext(hndl, hndl->last_device_sent, mode, 0);
}

int umanipulatorctl_set_mode_ext(umanipulatorctl_state_t * hndl,
                                 const unsigned char dev,
                                 const umanipulatorctl_mode_t mode,
                                 const int extended_pen_step)
{
    int ret, ver = 0, cmd = NORMAL_DRIVE_MODE, step = 2;
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);
    if(is_invalid_dev(dev))
        return -2;
    if(!is_valid_mode(mode))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_ARG);
    if((ret = umanipulatorctl_read_version(hndl, &ver, NULL)) < 0)
        return ret;
    switch(mode)
    {
        case UMANIPULATORCTL_MODE_PEN:
            cmd = PENETRATION_DRIVE_MODE;
            // Coarse ratio from nm to step count in pen mode
            if(extended_pen_step < 1 || extended_pen_step > 10000)
                step = 2;
            else if((step = extended_pen_step/200) < 1)
                step = 1;
            break;
        case UMANIPULATORCTL_MODE_SNAIL:
            cmd = SNAIL_DRIVE_MODE;
            if(ver < 3)
                step = V2_SNAIL_INCREMENT;
            else
                step = V3_SNAIL_INCREMENT;
            break;
        case UMANIPULATORCTL_MODE_5:
            if(ver < 3)
                step = V2_SPEED5_INCREMENT;
            else
                step = V3_SPEED5_INCREMENT;
            break;
        case UMANIPULATORCTL_MODE_4:
            if(ver < 3)
                step = V2_SPEED4_INCREMENT;
            else
                step = V3_SPEED4_INCREMENT;
            break;
        case UMANIPULATORCTL_MODE_3:
            if(ver < 3)
                step = V2_SPEED3_INCREMENT;
            else
                step = V3_SPEED3_INCREMENT;
            break;
        case UMANIPULATORCTL_MODE_2:
            if(ver < 3)
                step = V2_SPEED2_INCREMENT;
            else
                step = V3_SPEED2_INCREMENT;
            break;
        case UMANIPULATORCTL_MODE_1:
        default:
            if(ver < 3)
                step = V2_SPEED1_INCREMENT;
            else
                step = V3_SPEED1_INCREMENT;
            break;
    }
    if((ret = umanipulatorctl_write(hndl, dev, INCREMENT_REG, step)) < 0)
        return ret;
    return umanipulatorctl_cmd(hndl, dev, cmd);
}

int umanipulatorctl_set_step(umanipulatorctl_state_t * hndl, const int value)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);;
    if(is_invalid_dev(hndl->last_device_sent))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    return umanipulatorctl_set_step_ext(hndl, hndl->last_device_sent, value);
}

// Value is scaled to provide memory drive speed value*0.1 um/ms.
int umanipulatorctl_set_step_ext(umanipulatorctl_state_t * hndl,
                                 const unsigned char dev,
                                 const int value)
{
    int reg_value;
    // Old manipulator firmwares had 10nm target step precision,
    // This is preserved for backward compatilibyt for values below 100.
    // Values above are interpreted as 1nm target step size.
    if(value > 0 && value < 500)
        reg_value = 100 + value;
    else
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_ARG);
    return umanipulatorctl_write(hndl, dev, STEP_LENGHT, reg_value);
}

int umanipulatorctl_get_step(umanipulatorctl_state_t * hndl)
{
    if(!hndl)
        return set_last_error(hndl, UMANIPULATORCTL_NOT_OPEN);;
    if(is_invalid_dev(hndl->last_device_sent))
        return set_last_error(hndl, UMANIPULATORCTL_INVALID_DEV);
    return umanipulatorctl_get_step_ext(hndl, hndl->last_device_sent);
}

int umanipulatorctl_get_step_ext(umanipulatorctl_state_t * hndl,
                                 const unsigned char dev)
{
    int ret;
    if((ret = umanipulatorctl_read(hndl, dev, STEP_LENGHT )) < 0)
        return ret;
    if(ret > 100)
        return ret-100;
    return ret*10;
}
