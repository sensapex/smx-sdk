/**
 * A software development kit for Sensapex Micromanipulator, simplified interface
 * for the LabView integration
 *
 * Copyright (c) 2012-2015 Sensapex. All rights reserved
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

#include "umlabvif.h"

#include "umanipulatorctl.h"
#include "common.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/timeb.h>

#define VERSION "0.5.5.1"
#define VERSION_NUM 0.551

#define DEF_TIMEOUT 200
#define LOG_FILE "umlabvif.log"

// Labview and matlab gots easily confused if using struct pointers,
// thus this wrapper with simplified interface stores the handle pointer internally

// Typedefs
typedef int (*get_position_nm)(const int);
typedef int (*step)(const int);

typedef struct umlabvif_state_s
{
    int pos_average;
    int start_ts, start_pos, end_pos, elapsed;
    int org_mem_drive_step_size;
    int org_speed_mode;
    int calib_step_size;
    int tw8_pos_sensor;
    float velocity, velocity_sum;
    int velocity_n;
    FILE *logfp;
} umlabvif_state_t;

static umanipulatorctl_state_t *_hndl = NULL;

static umlabvif_state_t _state =
{ 5, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0, NULL };

void umlabvif_set_log_to_stderr()
{   _state.logfp = stderr; }

void umlabvif_set_log_to_stdout()
{   _state.logfp = stdout; }

void umlabvif_set_log_to_file(const char *filename)
{   _state.logfp = fopen(filename, "w"); }

void umlabvif_enable_tw8_pos_sensor(const int enable)
{
    if (enable)
        _state.tw8_pos_sensor = 1;
    else
        _state.tw8_pos_sensor = 0;
}

static void logger(const char *format, ...)
{
    va_list args;
    FILE *fp;
    if(_state.logfp)
        fp = _state.logfp;
    else if((fp = fopen(LOG_FILE,"at")) == NULL)
        return;
    va_start(args, format);
    vfprintf(fp, format, args);
    va_end(args);
    if(!_state.logfp)
        fclose(fp);
}

static int umlabvif_is_invalid_pos_nm(int pos)
{
  if(pos < 0 || pos > UMANIPULATORCTL_MAX_POSITION*1000)
      return UMANIPULATORCTL_INVALID_ARG;
  return 0;
}

static int do_setmode(const int mode)
{
    int retry;
    int retval;

    for (retry = 0; retry < 3; retry++) {
        retval = umlabvif_set_mode(mode);
        if (retval < 0)
            continue;
        break;
    }
    return retval;
}

static int do_getmode()
{
    int retry;
    int retval;

    for (retry = 0; retry < 3; retry++) {
        retval = umlabvif_get_mode();
        if (retval < 0)
            continue;
        break;
    }
    return retval;
}

int umlabvif_open(const char *port)
{
    int ret = -1;
    if(_hndl)
    {
        logger("umlabvif_open: was already opened\n");
        umanipulatorctl_close(_hndl);
    }
    if(!port)
        logger("umlabvif_open: NULL argument\n");
    else
    {
        logger("umlabvif_open: version %s (umanipulatorctl %s)\n",
               VERSION, umanipulatorctl_get_version());
        logger("umlabvif_open: opening %s\n", port);
        if((_hndl = umanipulatorctl_open(port, DEF_TIMEOUT)) == NULL)
        {
            logger("umlabvif_open: failed %s (%d)\n",
                   umanipulatorctl_last_errorstr(_hndl),
                   umanipulatorctl_last_os_error(_hndl));
        }
        else
            ret = 0;
    }
    return ret;
}

int umlabvif_close()
{
    if(!_hndl)
    {
        logger("umlabvif_close: not opened\n");
        return -1;
    }
    umanipulatorctl_close(_hndl);
    _hndl = NULL;
    return 0;
}

int umlabvif_select_dev(const int dev)
{
    int i, ret;
    logger("umlabvif_select_dev: selecting %d\n",dev);
    // In MAC OS X there is often garbage in the input buffer
    // Thus these few retries for the first message
    for(i = 0; i < 3; i++)
    {
        if((ret = umanipulatorctl_select_dev(_hndl, 0x30+dev)) >= 0)
            break;
    }
    if(ret < 0)
    {
        logger("umlabvif_select_dev: failed %s (%d)\n",
           umanipulatorctl_last_errorstr(_hndl),
           umanipulatorctl_last_error(_hndl));
        return ret;
    }
    umanipulatorctl_set_refresh_time_limit(_hndl, 1);
    return ret;
}

int umlabvif_x_position()
{
    int ret = umanipulatorctl_x_position(_hndl);
    if(ret < 0)
        logger("umlabvif_x_position: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    else
        logger("umlabvif_x_position: %d\n", ret);
    if(ret > 0x8000)
        ret = 0;
    return ret;
}

int umlabvif_x_position_nm(const int average_count)
{
    int avg, ret = 0, n = 0, fails = 0, nm_pos = 0;
    if(average_count < 1)
        avg = 1;
    else
        avg = average_count;

    while(n < avg)
    {
        ret = umanipulatorctl_x_position_ext(_hndl, _hndl->last_device_sent,UMANIPULATORCTL_TIMELIMIT_DISABLED, NULL);
        if(ret < 0)
        {
            if(++fails > 3)
                break; // 3 failing read -> out of loop

            continue; //skip negative value
        }
        fails  = 0;
        nm_pos += (int)((short)(ret))*1000;
        n++;
    }
    if(ret < 0 || !n)
        return -1;
    return nm_pos/n;
}

int umlabvif_y_position()
{
    int ret = umanipulatorctl_y_position(_hndl);
    if(ret < 0)
        logger("umlabvif_y_position: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    else
        logger("umlabvif_y_position: %d\n", ret);
    if(ret > 0x8000)
        ret = 0;
    return ret;
}

int umlabvif_y_position_nm(const int average_count)
{
    int avg, ret = 0, n = 0, fails = 0, nm_pos = 0;
    if(average_count < 1)
        avg = 1;
    else
        avg = average_count;

    while(n < avg)
    {
        ret = umanipulatorctl_y_position_ext(_hndl, _hndl->last_device_sent, UMANIPULATORCTL_TIMELIMIT_DISABLED, NULL);
        if(ret < 0)
        {
            if(++fails > 3)
                break; // 3 failing read -> out of loop

            continue; //skip negative value
        }
        fails  = 0;
        nm_pos += (int)((short)(ret))*1000;
        n++;
    }
    if(ret < 0 || !n)
        return -1;
    return nm_pos/n;
}

int umlabvif_z_position()
{
    int ret = umanipulatorctl_z_position(_hndl);
    if(ret < 0)
        logger("umlabvif_z_position: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    else
        logger("umlabvif_z_position: %d\n", ret);
    if(ret > 0x8000)
        ret = 0;
    return ret;
}

int umlabvif_z_position_nm(const int average_count)
{
    int avg, ret = 0, n = 0, fails = 0, nm_pos = 0;
    if(average_count < 1)
        avg = 1;
    else
        avg = average_count;

    while(n < avg)
    {
        ret = umanipulatorctl_z_position_ext(_hndl, _hndl->last_device_sent, UMANIPULATORCTL_TIMELIMIT_DISABLED, NULL);
        if(ret < 0)
        {
            if(++fails > 3)
                break; // 3 failing read -> out of loop

            continue; //skip negative value
        }
        fails  = 0;
        if(_state.tw8_pos_sensor)
            nm_pos += ret;  // ret is already in nm scale and 32 bit integer
        else
            nm_pos += (int)((short)(ret))*1000; // ret is in um scale -> convert to nm scale
        n++;
    }
    if(ret < 0 || !n)
        return -1;
    return nm_pos/n;
}

int umlabvif_goto_position(const int x, const int y, const int z)
{
    int ret;
    logger("umlabvif_goto_position: %d %d %d\n", x, y, z);

    if((ret = umanipulatorctl_store_mem_position(_hndl, x, y, z)))
        logger("umlabvif_goto_position: store mem failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    if(ret >= 0 && (ret = umanipulatorctl_goto_mem_position(_hndl)) < 0)
        logger("umlabvif_goto_position: goto mem failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    return ret;
}

int umlabvif_set_mode(const int speed)
{
    int ret;
    logger("umlabvif_set_mode: %d\n", speed);
    if((ret = umanipulatorctl_set_mode(_hndl,(umanipulatorctl_mode_t)speed)) < 0)
        logger("umlabvif_set_mode: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    return ret;
}

int umlabvif_get_mode()
{
    int ret;
    if((ret = umanipulatorctl_get_mode(_hndl)) < 0)
        logger("umlabvif_get_mode: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    else
        logger("umlabvif_get_mode: %d\n", ret);
    return ret;
}

int umlabvif_set_step(const int value)
{
    int ret;
    logger("umlabvif_set_step: %d\n", value);
    if((ret = umanipulatorctl_set_step(_hndl, value)) < 0)
        logger("umlabvif_set_step: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    return ret;
}

int umlabvif_get_step()
{
    int ret;
    if((ret = umanipulatorctl_get_step(_hndl)) < 0)
        logger("umlabvif_get_step: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    else
        logger("umlabvif_get_step: %d\n", ret);
    return ret;
}

int umlabvif_is_busy()
{   return umanipulatorctl_is_busy(_hndl);  }

static int write_speed(const unsigned char axel, const unsigned short speed)
{
    int ret;
    if((ret = umanipulatorctl_write_speed(_hndl, _hndl->last_device_sent, axel, speed)) < 0)
        logger("umlabvif_x|y|z_take_step: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    return ret;
}

#define DEF_SPEED_REG_VALUE_FORWARD  3
#define DEF_SPEED_REG_VALUE_BACKWARD (-3)

// Higher speed register value causing slower movement in step mode may be given explicitly,
// values 0 and 1 means full speed forward and backward, respectively
int umlabvif_x_take_step(const int reverse_direction)
{
    unsigned short speed;
    if(reverse_direction < DEF_SPEED_REG_VALUE_FORWARD &&
       reverse_direction > DEF_SPEED_REG_VALUE_BACKWARD)
    {
        speed = reverse_direction?DEF_SPEED_REG_VALUE_BACKWARD:DEF_SPEED_REG_VALUE_FORWARD;
        logger("umlabvif_x_take_step: %s\n", reverse_direction?"backward":"forward");
    }
    else
    {
        speed = reverse_direction;
        logger("umlabvif_x_take_step: %s (%d)\n", speed > 0x7fff?"backward":"forward", abs((int)speed));
    }
    return write_speed(X_SPEED_REG, reverse_direction>0?DEF_SPEED_REG_VALUE_BACKWARD:DEF_SPEED_REG_VALUE_FORWARD);
}

int umlabvif_y_take_step(const int reverse_direction)
{
    unsigned short speed;
    if(reverse_direction < DEF_SPEED_REG_VALUE_FORWARD &&
       reverse_direction > DEF_SPEED_REG_VALUE_BACKWARD)
    {
        speed = reverse_direction?DEF_SPEED_REG_VALUE_BACKWARD:DEF_SPEED_REG_VALUE_FORWARD;
        logger("umlabvif_y_take_step: %s\n", reverse_direction?"backward":"forward");
    }
    else
    {
        speed = reverse_direction;
        logger("umlabvif_y_take_step: %s (%d)\n", speed > 0x7fff?"backward":"forward", abs((int)speed));
    }
    return write_speed(Y_SPEED_REG, speed);
}

int umlabvif_z_take_step(const int reverse_direction)
{
    unsigned short speed;
    if(reverse_direction < DEF_SPEED_REG_VALUE_FORWARD &&
       reverse_direction > DEF_SPEED_REG_VALUE_BACKWARD)
    {
        speed = reverse_direction?DEF_SPEED_REG_VALUE_BACKWARD:DEF_SPEED_REG_VALUE_FORWARD;
        logger("umlabvif_z_take_step: %s\n", reverse_direction?"backward":"forward");
    }
    else
    {
        speed = reverse_direction;
        logger("umlabvif_z_take_step: %s\n", speed > 0x7fff?"backward":"forward");
    }
    return write_speed(Z_SPEED_REG, speed);
}

#define MEM_SPEED_REG 38
#define MEM_SLOW_LIMIT_REG 39
#define MEM_DEF_SLOW_LIMIT 256
#define MEM_EXT_SLOW_LIMIT 2048
#define MEM_EXR_SLOW_LIMIT 2049

int umlabvif_set_mem_speed_mode(const int mode)
{
    int ret;
    int mem_speed = 3;
    int mem_slow_limit = 0;

    if(mode == 0)
        logger("umlabvif_set_mem_speed_mode: off\n");
    else if(mode == 1)
    {
        mem_speed = 0x500;
        mem_slow_limit = MEM_DEF_SLOW_LIMIT;
        logger("umlabvif_set_mem_speed_mode: normal\n");
    }
    else if(mode == 2)
    {
        mem_speed = 0x900;
        mem_slow_limit = MEM_EXT_SLOW_LIMIT;
        logger("umlabvif_set_mem_speed_mode: extended\n");
    }
    else if(mode == 3)
    {
        mem_speed = 0x5555;
        mem_slow_limit = MEM_EXR_SLOW_LIMIT;
        logger("umlabvif_set_mem_speed_mode: extreme\n");
    }
    else
    {
        logger("umlabvif_set_mem_speed_mode: invalid mode %d\n", mode);
        return -1;
    }
    if((ret = umanipulatorctl_write(_hndl,_hndl->last_device_sent,
                                    MEM_SPEED_REG, mem_speed)) < 0)
    {
        logger("umlabvif_set_mem_speed_mode: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    }
    else if((ret = umanipulatorctl_write(_hndl,_hndl->last_device_sent,
                                         MEM_SLOW_LIMIT_REG, mem_slow_limit)) < 0)
    {
        logger("umlabvif_set_mem_speed_mode: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    }
    return ret;
}

int umlabvif_get_mem_speed_mode()
{
    int ret;
    if((ret = umanipulatorctl_read(_hndl, _hndl->last_device_sent,
                                   MEM_SLOW_LIMIT_REG)) < 0)
    {
        logger("umlabvif_get_mem_speed_mode: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    }
    else
    {
        if(ret == MEM_DEF_SLOW_LIMIT)
            ret = 1;
        if(ret == MEM_EXT_SLOW_LIMIT)
            ret = 2;
        if(ret == MEM_EXR_SLOW_LIMIT)
            ret = 3;
        logger("umlabvif_get_mem_speed_mode: %d\n");
    }
    return ret;
}

#define STATUS_ZERO_UNKNOWN_MASK 0x7000

int umlabvif_zero_pos_unknown()
{
    int ret;
    if((ret = umanipulatorctl_read(_hndl, _hndl->last_device_sent,
                                   STATUS_REG)) < 0)
    {
        logger("umlabvif_get_mem_speed_mode: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    }
    ret = (ret&STATUS_ZERO_UNKNOWN_MASK)?1:0;
    logger("umlabvif_zero_pos_unknown: %d\n", ret);
    return ret;
}

#define ZERO_POINT_CMD 1
#define VOLTAGE_CALIBRATE_CMD 7

int umlabvif_init_zero_pos()
{
    int ret;
    logger("umlabvif_init_zero_pos");
    if((ret = umanipulatorctl_cmd(_hndl, _hndl->last_device_sent,
                                        ZERO_POINT_CMD)) < 0)
    {
        logger("umlabvif_init_zero_pos: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    }
    return ret;
}

int umlabvif_speed_calibration()
{
    int ret;
    logger("umlabvif_speed_calibration");
    if((ret = umanipulatorctl_cmd(_hndl, _hndl->last_device_sent, VOLTAGE_CALIBRATE_CMD)) < 0)
    {
        logger("umlabvif_speed_calibration: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    }
    return ret;
}

int umlabvif_cmd(const unsigned char cmd)
{
    int ret;
    logger("umlabvif_cmd");

    if((ret = umanipulatorctl_cmd(_hndl, _hndl->last_device_sent, cmd)) < 0)
    {
        logger("umlabvif_cmd: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    }
    return ret;
}

#define VIRTUAL_Z_ENABLE                37
#define VIRTUAL_Z_DISABLE               38

int umlabvif_enable_virtual_x(const unsigned char enable)
{
    int ret;
    logger("umlabvif_enable_virtual_x");
    char cmd = VIRTUAL_Z_ENABLE;

    if (!enable)
        cmd = VIRTUAL_Z_DISABLE;

    if((ret = umanipulatorctl_cmd(_hndl, _hndl->last_device_sent, cmd)) < 0)
    {
        logger("umlabvif_enable_virtual_x: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    }
    return ret;

}

int umlabvif_write(const unsigned char addr, const short value)
{
    int ret;
    logger("umlabvif_write");

    if((ret = umanipulatorctl_write(_hndl, _hndl->last_device_sent, addr, value) < 0))
    {
        logger("umlabvif_write: failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    }
    return ret;
}

// Stimulate functions - supporting initially only Z axis

static unsigned long int get_timestamp()
{
  struct timeb ts;
  ftime(&ts);
  return (unsigned long int)(ts.time*1000L+ts.millitm);
}

static void clear_velocity()
{
    _state.velocity_sum = 0.0;
    _state.velocity_n = 0;
}

static float average_velocity()
{
    if(_state.velocity_n)
        return _state.velocity_sum/_state.velocity_n;
    return 0.0;
}

#define SHADOW_REG_NOT_SUPPORTED 0xfbad

static int read_tw8_velocity()
{
    int ret;
    float velocity;
    if((ret = umanipulatorctl_read(_hndl, _hndl->last_device_sent, TW8_VELOCITY_SHADOW_REG)) < 0)
    {
        logger("read_tw8_velocity: failed %s (%d)\n", umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
        return ret;
    }
    if(ret == SHADOW_REG_NOT_SUPPORTED)
    {
        logger("read_tw8_velocity: manipulator firmware too old, velocity not available\n");
        return 0;
    }
    // 20um cycle 24MHz clock, see page 9 of the TW8 programming manual
    // unit is KHz*um = K*um/s = mm/s i.e. um/ms and resolution 0.228 um/ms
    velocity = (float)((short)ret)*24.0*20.0/-2097.152;
    _state.velocity_sum += velocity;
    _state.velocity_n++;
//  logger("read_tw8_velocity: %3.1f um/ms\n", velocity);
    return _state.velocity_n;
}

static int read_velocity()
{
    int ret;
    float velocity;
    if((ret = umanipulatorctl_read(_hndl, _hndl->last_device_sent, MEM_DRIVE_VELOCITY_SHADOW_REG)) < 0)
    {
        logger("read_velocity: failed %s (%d)\n", umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
        return ret;
    }
    if(ret == SHADOW_REG_NOT_SUPPORTED)
    {
        logger("read_velocity: manipulator firmware too old, velocity not available\n");
        return 0;
    }
    // Velocity scale is nm/ms in the MCU code
    if((velocity = (float)((short)ret)/1000.0) != 0.0)
    {
        _state.velocity_sum += velocity;
        _state.velocity_n++;
    }
    logger("read_velocity: %3.1f um/ms\n", velocity);
    return _state.velocity_n;
}

static int wait_while_busy()
{
    int ret, fails = 0;
    unsigned long start_ts;
    start_ts = get_timestamp();
    while((ret = umlabvif_is_busy()) != 0)
    {
        if(ret < 0)
        {
            if(fails++ > 200)
                break;
        }
        else
            fails = 0;
        if(_state.tw8_pos_sensor)
            read_tw8_velocity();
        else
            read_velocity();
        msleep(10+(rand()%30));
    }
    if(ret < 0)
        return ret;
    return (int)(get_timestamp() - start_ts)+1;
}

static int round_position(const int pos_nm, int *nm_fraction)
{
    if(pos_nm < 0)
        return pos_nm;
    if(_state.tw8_pos_sensor && nm_fraction)
    {
        *nm_fraction = pos_nm%1000;
        return pos_nm/1000;
    }
    else
        return (int)(pos_nm+0.5)/1000;
}

static int move_to_position(const int x_nm, const int y_nm, const int z_nm)
{
    int ret, x_um, y_um, z_um, z_nm_fraction = 0;
    x_um = round_position(x_nm, NULL);
    y_um = round_position(y_nm, NULL);
    z_um = round_position(z_nm, &z_nm_fraction);
    if(_state.tw8_pos_sensor)
    {
        // In the single axis manipulator with high accuracy position sensor,  nanometer
        // fraction of the Z_AXIS_POS_MEM 3 is stored into the Y axis register
        umanipulatorctl_write(_hndl, _hndl->last_device_sent, Y_AXIS_POS_MEM_3, z_nm_fraction);
    }
    // This will store the um positions and send memory drive command
    if((ret = umlabvif_goto_position(x_um, y_um, z_um)) < 0)
        return ret;
    return wait_while_busy();
}

// This experimental functionality does not provide good enough performance
#ifdef OPENLOOP_MOVE

static int take_step(const int speed)
{
    int ret, pos1_nm, pos2_nm, step_size;
    if((ret  = umlabvif_z_position_nm(_state.pos_average)) < 0)
        return ret;
    pos1_nm = ret;
    clear_velocity();
    // Open loop step size calibration.
    if((ret = umlabvif_z_take_step(speed)) < 0)
        return ret;
    if((ret = wait_while_busy()) < 0)
        return ret;
    if((ret = umlabvif_z_position_nm(_state.pos_average)) < 0)
        return ret;
    pos2_nm = ret;
    if(speed > 0x7fff)
        step_size = pos1_nm-pos2_nm;
    else
        step_size = pos2_nm-pos1_nm;
    logger("umlabvif_stimulate_init: Pos: %3.2fum, %3.2fum step, MCU reported average speed %3.2f um/ms\n",
               (float)pos2_nm/1000.0, (float)step_size/1000.0, average_velocity());
    return 0;
}

static int read_dac_value()
{
    int ret;
    if((ret = umanipulatorctl_read(_hndl, _hndl->last_device_sent, Z_DAC_VALUE_SHADOW_REG)) < 0)
        logger("umlabvif_stimulate_init: Write failed failed %s (%d)\n", umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    else
        logger("umlabvif_stimulate_init: DAC %d\n", ret);
    return ret;
}

#endif

int umlabvif_stimulate_init(const char *port, const int dev,
                            const int speed_100nm_per_ms)
{
    int ret;
    // Open port
    if((ret = umlabvif_open(port)) < 0)
        return ret;
    // Select device
    if((ret = umlabvif_select_dev(dev)) < 0)
        return ret;
    // Store current position to prevent X and Y axis movement
    if((ret = umanipulatorctl_store_mem_current_position(_hndl)) < 0)
    {
        logger("umlabvif_stimulate_init: store_mem_current_position failed %s (%d)\n",
               umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_os_error(_hndl));
        return ret;
    }
    // Do not slow near target position (i.e. attempt to use constant velocity)

    //if((ret = umlabvif_set_mem_speed_mode(0)) < 0)
    //    return ret;

    // Make sure that normal mode is active i.e. not PEN or Snail
    if((ret = umlabvif_get_mode()) < 0)
        return ret;
    if(ret == UMANIPULATORCTL_MODE_PEN || ret == UMANIPULATORCTL_MODE_SNAIL)
    {
        logger("umlabvif_stimulate_init: Disabling %s mode\n", ret == UMANIPULATORCTL_MODE_PEN?"PEN":"Snail");
        if((ret = umlabvif_set_mode(UMANIPULATORCTL_MODE_1)) < 0)
            return ret;
    }
    // Store mem drive step size and the increment register defining the speed mode
    _state.org_mem_drive_step_size = umlabvif_get_step();
    _state.org_speed_mode = umanipulatorctl_read(_hndl, _hndl->last_device_sent, INCREMENT_REG);

    // Set speed used for the memory drive mode
    if((ret = umlabvif_set_step(speed_100nm_per_ms*5)) < 0)
        return ret;
#ifdef OPENLOOP_MOVE
    if(pulse_calib_burst_count)
    {
        if((ret = umanipulatorctl_write(_hndl, _hndl->last_device_sent, INCREMENT_REG, initial_pulse_count)) < 0)
        {
            logger("umlabvif_stimulate_init: Write failed failed %s (%d)\n", umanipulatorctl_last_errorstr(_hndl),
                   umanipulatorctl_last_error(_hndl));
            return ret;
        }
        // Steps backward
        for(i = 0; i < pulse_calib_burst_count; i++)
        {
            if((ret == take_step(0xffff-write_speed_value)) < 0)
                return ret;
            read_dac_value();
        }
        // Forward
        for(i = 0; i < pulse_calib_burst_count; i++)
        {
            if((ret == take_step(write_speed_value)) < 0)
                return ret;
            read_dac_value();
        }
        // Drive back to the start position stored to the manipulators memory above,
        // position -1 is invalid and thus not overwritten over the stored value
        if((ret = move_to_position(-1, -1, -1)) < 0)
            return ret;
    }
#endif // OPENLOOP_MOVE
    // Read accurate start position
    if((ret = umlabvif_z_position_nm(_state.pos_average)) < 0)
        return ret;
    _state.start_pos = ret;
    logger("umlabvif_stimulate_init: Start position %3.2fum\n", (float)_state.start_pos/1000.0);
    return 0;
}

void umlabvif_stimulate_set_pos_average(const int count)
{   _state.pos_average = count; }

static int get_elapsed()
{
    int elapsed = umanipulatorctl_read(_hndl, _hndl->last_device_sent, MEM_DRIVE_TIME_SHADOW_REG);
    if(elapsed == SHADOW_REG_NOT_SUPPORTED)
    {
        logger("umlabvif_stimulate_z: manipulator firmware too old, timer value not available\n");
        elapsed = 0;
    }
    return elapsed;
}

static float calc_velocity(const int pos_now, const int pos_prev, const int elapsed)
{
    if(elapsed)
        return (float)(pos_now - pos_prev) / ((float)elapsed*1000.0);
    return 0.0;
}

static void log_speed(const int pos_now, const int pos_prev, const int elapsed)
{
    logger("umlabvif_stimulate_z: %3.2fum in %d ms, calculated %3.2f um/ms, manipulator reported %3.2f um/ms (n %d)\n",
           ((float)(pos_now-pos_prev))/1000.0, elapsed,
           calc_velocity(pos_now, pos_prev, elapsed),
           average_velocity(), _state.velocity_n);
}

int umlabvif_stimulate_z(const int depth_10nm, const int delay)
{
    int ret;

    clear_velocity();
    _state.start_ts = get_timestamp();

    if((ret = move_to_position(-1, -1, _state.start_pos + depth_10nm*10)) < 0)
        return ret;
    if((ret = umlabvif_z_position_nm(_state.pos_average)) < 0)
        return ret;
    _state.end_pos = ret;
    _state.elapsed  = get_elapsed();
    if(_state.velocity_n < 2)
        _state.velocity = calc_velocity(_state.end_pos, _state.start_pos, _state.elapsed);
    else
        _state.velocity = average_velocity();
    log_speed(_state.end_pos, _state.start_pos, _state.elapsed);
    if(delay)
    {
        logger("umlabvif_stimulate_z: delay %d ms\n",delay);
        msleep(delay);
    }
    clear_velocity();
    if((ret = move_to_position(-1, -1, _state.start_pos)) < 0)
        return ret;
    if((ret = umlabvif_z_position_nm(_state.pos_average)) < 0)
        return ret;
    log_speed(ret, _state.end_pos, get_elapsed());
    return 0;
}

void umlabvif_stimulate_close()
{
    if(_hndl)
    {
        if(_state.org_mem_drive_step_size > 0)
            umlabvif_set_step(_state.org_mem_drive_step_size);
        if(_state.org_speed_mode > 0)
            umanipulatorctl_write(_hndl, _hndl->last_device_sent, INCREMENT_REG,_state.org_speed_mode);
        umlabvif_close();
    }
}

int umlabvif_get_start_ts()
{
    return _state.start_ts;
}

int umlabvif_get_elapsed()
{
    return _state.elapsed;
}

float umlabvif_get_start_pos()
{
    return (float)_state.start_pos/1000.0;
}

float umlabvif_get_end_pos()
{
    return (float)_state.end_pos/1000.0;
}

float umlabvif_get_velocity()
{
    return _state.velocity;
}

float umlabvif_get_version_num()
{
    return VERSION_NUM;
}

const char *umlabvif_get_version()
{
    return VERSION;
}

#define PROT_HWID_REG       93

int umlabvif_get_hw_id()
{
    int ret = 0;
    if((ret = umanipulatorctl_read(_hndl, _hndl->last_device_sent, PROT_HWID_REG)) < 0)
        logger("umlabvif_get_hw_id: Read failed %s (%d)\n", umanipulatorctl_last_errorstr(_hndl),
               umanipulatorctl_last_error(_hndl));
    else
        logger("umlabvif_get_hw_id: HW_ID %d\n", ret);
    return ret;
}

#define GOTO_POS_NM_TARGET_ACCURACY     200
#define GOTO_POS_NM_AVEG_ITERARTIONS    2
#define GOTO_POS_NM_STEP_MAX_CNT        40
#define GOTO_POS_NM_TRIPLE              3

int umlabvif_goto_position_nm(const int x_nm, const int y_nm, const int z_nm)
{
    int err = move_to_position(x_nm, y_nm, z_nm);
    if (err >= 0 && !_state.tw8_pos_sensor) {

        // Do the manual fine tunings
        int i, axis;
        int delta_cpy;
        // Save the current mode
        int cur_mode = do_getmode();

        if (cur_mode >= 0) {

            // Set mode 1
            if ((err = do_setmode(UMANIPULATORCTL_MODE_1)) < 0)
                return err;

            for (axis = 0; axis < GOTO_POS_NM_TRIPLE; axis++) {
                int target_pos_nm;
                get_position_nm get_pos_nm_func_ptr;
                step take_step_func_ptr;

                // Set axis specific moderators
                switch (axis)
                {
                    case 0:
                        target_pos_nm = x_nm;
                        get_pos_nm_func_ptr = &umlabvif_x_position_nm;
                        take_step_func_ptr = &umlabvif_x_take_step;
                        break;
                    case 1:
                        target_pos_nm = y_nm;
                        get_pos_nm_func_ptr = &umlabvif_y_position_nm;
                        take_step_func_ptr = &umlabvif_y_take_step;
                        break;
                    case 2:
                        target_pos_nm = z_nm;
                        get_pos_nm_func_ptr = &umlabvif_z_position_nm;
                        take_step_func_ptr = &umlabvif_z_take_step;
                        break;
                }

                // Fine tune the position if needed.
                if (!umlabvif_is_invalid_pos_nm (target_pos_nm)) {
                    for (i = 0; i < GOTO_POS_NM_STEP_MAX_CNT; i++) {
                        // Get the current position
                        int cur_nm = get_pos_nm_func_ptr(GOTO_POS_NM_AVEG_ITERARTIONS);
                        if (cur_nm < 0){
                            logger("Get position, negative value\n");
                            return -1; // try only once
                        }

                        int delta = target_pos_nm - cur_nm;
                        delta_cpy = delta;
                        logger("umlabvif_goto_position_nm: delta_nm = %d\n", delta);

                        if (abs(delta) > GOTO_POS_NM_TARGET_ACCURACY) {
                            // Take a step closer to target.
                            take_step_func_ptr(delta > 0 ? 0 : 1);
                            wait_while_busy();
                            continue;
                        }
                        break;
                    }

                    if (abs(delta_cpy) > (GOTO_POS_NM_TARGET_ACCURACY )) {
                        logger("Fine tune not in target, axis:%d:\n",axis);
                        return -1;
                    }

                }
                else {
                    logger("umlabvif is invalid position.\n");
                    return -1;
                }
            }
        }
        // Restore the original mode
        do_setmode(cur_mode);
    }
    return err;
}
