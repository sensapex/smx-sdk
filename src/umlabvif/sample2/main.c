/**
 * A software development kit for Sensapex Micromanipulator, simplified interface
 * for the LabView integration, sample main demonstrating combined usage of fast
 * memory position drive with high accurate aproach to the target
 *
 * Copyright (c) 2013 Sensapex. All rights reserved
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define VERSION_STR "v0.105"
#define COPYRIGHT   "(c) 2013 Sensapex"
#define UNDEF       (-1)
#define MSG_RETRY   3

#ifndef PORT
#ifdef WIN32
#include <windows.h>
#define PORT "com1"
#define sleep(t) Sleep((t)*1000)
#define msleep(t) Sleep((t))
#else
#define PORT "/dev/ttyUSB0"
#include <unistd.h>
#include <sys/timeb.h>
#define msleep(t) usleep((t)*1000)
#endif
#endif

const char rcsid[] = "$Id: Micro manipulator labview sample 2 "VERSION_STR" "__DATE__" "COPYRIGHT" Exp $";

typedef struct params_s
{
    int verbose, y, Y, dev, loop, delay, mem_drive_step_len, mem_drive_margin;
    int busy_delay, mem_drive_speed_slow_down_mode, pulse_speed_mode;
    int msg_delay, pos_read_average_count, pulse_speed_value;
    float target_pos_accuracy;
    char *port;
} params_t;

static void usage(char **argv)
{
    fprintf(stderr,"usage: %s [options]\n",argv[0]);
    fprintf(stderr,"Generic options\n");
    fprintf(stderr,"-p\tport\te.g. /dev/ttyUSB1 or com2 (def: %s)\n", PORT);
    fprintf(stderr,"-d\tdev\t1,2,3... (def: 1)\n");
    fprintf(stderr,"-v\t\tverbose\n");
    fprintf(stderr,"Position change\n");
    fprintf(stderr,"-z\tum\trelative target position (def -10)\n");
    fprintf(stderr,"-Z\tum\tabsolutely target position\n");
    fprintf(stderr,"Loop\n");
    fprintf(stderr,"-n\tcount\toscillate between current and target position (def 100)\n");
    fprintf(stderr,"-t\tmsecs\tdelay at turn-around (def 1000ms)\n");
    fprintf(stderr,"\nAdvanced options\n");
    fprintf(stderr,"Speed tuning\n");
    fprintf(stderr,"-s\tmode\tspeed mode during stepping: 1, 2 or 7 = Snail (def: 1)\n");
    fprintf(stderr,"-S\tvalue\tPulse splitting control during stepping (def: 3)\n");
    fprintf(stderr,"-m\t0-3\tmemory drive slow down mode, 0 = off, 3 = extreme (def: 3)\n");
    fprintf(stderr,"-M\tnm\tmemory drive target step length (def 20)\n");
    fprintf(stderr,"-D\tms\tdelay between messages (def: 5)\n");
    fprintf(stderr,"-W\tms\tdelay between messages in busy wait (def: 20)\n");
    fprintf(stderr,"Accuracy tuning\n");
    fprintf(stderr,"-A\tum\tmemory position drive margin (def: 0.5um)\n");
    fprintf(stderr,"-a\tum\ttarget position accuracy (def: 0.5um)\n");
    fprintf(stderr,"-c\tcount\tposition read averaging count (def: 4)\n");
    fprintf(stderr,"%s %s\n", argv[0], VERSION_STR);
  exit(1);
}

// Exits via usage() if an error occurs
static void parse_args(int argc, char *argv[], params_t *params)
  {
  int i, v;
  float f;
  memset(params,0,sizeof(params_t));
  params->port = PORT;
  params->dev = 1;
  params->Y = UNDEF;
  params->y = -10;
  params->mem_drive_speed_slow_down_mode = 3;
  params->mem_drive_step_len = 20;
  params->mem_drive_margin   = 0.5;
  params->pulse_speed_value  = 3;
  params->pulse_speed_mode   = 1;
  params->pos_read_average_count = 4;
  params->target_pos_accuracy = 0.5;
  params->msg_delay = 5;
  params->busy_delay = 20;
  params->delay = 1000;
  params->loop = 100;

  for(i = 1; i < argc; i++)
    {
    if(argv[i][0] == '-')
      {
      switch(argv[i][1])
        {
        case 'h': usage(argv);
        case 'v':
          params->verbose++;
          break;
        case 'z':
          if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && v > -20000 &&  v <= 20000)
              params->y = v;
          else
              usage(argv);
          break;
        case 'Z':
          if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && v >= 0 && v <= 20000)
              params->Y = v;
          else
              usage(argv);
          break;
        case 'n':
          if(i < argc-1 && (sscanf(argv[++i],"0x%x",&v) == 1 ||
                            sscanf(argv[i],"%d",&v) == 1) && v > 0)
              params->loop = v;
          else
              usage(argv);
          break;
        case 't':
          if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && v >= 0)
              params->delay = v;
          else
              usage(argv);
          break;
        case 'd':
          if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && v >= 0 && v <= 16)
              params->dev = v;
          else
              usage(argv);
          break;
        case 's':
          if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && (v == 1 || v == 2 || v == 7))
              params->pulse_speed_mode = v;
          else
              usage(argv);
          break;
        case 'S':
          if(i < argc-1 &&
                  (sscanf(argv[++i],"0x%x",&v) == 1 || sscanf(argv[i],"%d",&v) == 1) &&
                  v >= 3 && v <= 0x5555)
              params->pulse_speed_value = v;
          else
              usage(argv);
          break;
        case 'M':
          if(sscanf(argv[++i],"%d",&v) == 1 && v > 0)
              params->mem_drive_step_len = v;
          else
              usage(argv);
          break;
        case 'm':
          if(sscanf(argv[++i],"%d",&v) == 1 && v >= 0 && v <= 3)
              params->mem_drive_speed_slow_down_mode = v;
          else
              usage(argv);
          break;
        case 'D':
          if(sscanf(argv[++i],"%d",&v) == 1 && v > 0 && v < 1000)
              params->msg_delay  = v;
          else
              usage(argv);
          break;
        case 'W':
          if(sscanf(argv[++i],"%d",&v) == 1 && v > 0 && v < 2000)
              params->busy_delay  = v;
          else
              usage(argv);
          break;
        case 'A':
          if(sscanf(argv[++i],"%f",&f) == 1 && f >= 0.0 && f <= 1000)
              params->mem_drive_margin = f;
          else
              usage(argv);
          break;
        case 'a':
          if(sscanf(argv[++i],"%f",&f) == 1 && f >= 0.0 && f <= 10)
              params->target_pos_accuracy = f;
          else
              usage(argv);
          break;
        case 'c':
          if(sscanf(argv[++i],"%d",&v) == 1 && v >= 0 && v <= 100)
              params->pos_read_average_count = v;
          else
              usage(argv);
          break;
        case 'p':
          if(i < argc-1 && argv[i+1][0] != '-')
              params->port = argv[++i];
          else
              usage(argv);
          break;
        default:
          usage(argv);
          break;
        }
      }
    else
      usage(argv);
    }
}

static unsigned long get_ms_time()
{
#ifdef WIN32
    return timeGetTime();
#else
    struct timeb ts;
    ftime(&ts);
    return (unsigned long)(ts.time*1000L+ts.millitm);
#endif
}

static float elapsed(unsigned long start_time)
{
    return (float)(get_ms_time() - start_time)/1000.0;
}

static int avg_position(const char axis,  params_t *params, float *avg_position)
{
    int i, j, pos;
    *avg_position = 0;
    // Average over N position readings
    for(i = 0; i < params->pos_read_average_count; i++)
    {
        // Retry single position reading 3 times
        for(j = 0; j < MSG_RETRY; j++)
        {
            switch(axis)
            {
                case 'x': pos = umlabvif_x_position(); break;
                case 'y': pos = umlabvif_y_position(); break;
                case 'z': pos = umlabvif_z_position(); break;
                default: pos = -1;
            }
            if(pos >= 0)
                break;
            msleep(params->msg_delay);
        }
        if(pos < 0)
            return pos;
        *avg_position += (float)((short)pos);
        msleep(params->msg_delay);
    }
    *avg_position /= (float)params->pos_read_average_count;
    return 0;
}

static int avg_y_position_with_statistics(params_t *params, float *current_pos,
                                          const float goto_pos, unsigned long start_time)
{
    int ret;
    float distance;
    if((ret = avg_position('y', params, current_pos)) < 0)
    {
        fprintf(stderr,"position read failed\n");
        return ret;
    }
    if(params->verbose > 1)
    {
        distance = fabs(*current_pos - goto_pos);
        printf("%4.3f\t%4.3f\t%4.3f\n", elapsed(start_time),*current_pos, distance);
    }
    return ret;
}

static int wait_busy(params_t *params)
{
    int ret, j;
    do
    {
        for(j = 0; j < MSG_RETRY; j++)
        {
            if((ret = umlabvif_is_busy()) >= 0)
                break;
        }
        msleep(params->busy_delay);
    } while(ret > 0);
    return ret;
}

static int accurate_goto_position(params_t *params, const float goto_pos)
{
    int j, x, z , ret, mem_drive_pos, speed;
    float current_pos;
    unsigned long int start_time = get_ms_time();

    if((ret = avg_position('x', params, &current_pos)) < 0)
        return ret;
    x = floor(current_pos+0.5);
    if((ret = avg_position('z', params, &current_pos)) < 0)
        return ret;
    z = floor(current_pos+0.5);
    if((ret = avg_y_position_with_statistics(params, &current_pos, goto_pos, start_time)) < 0)
        return ret;

    if(fabs(current_pos - goto_pos) > params->mem_drive_margin)
    {
        if(current_pos > goto_pos)
            mem_drive_pos = goto_pos + params->mem_drive_margin;
        else
            mem_drive_pos = goto_pos - params->mem_drive_margin;
        if(params->verbose)
            fprintf(stderr,"%4.3fs driving to position %d using memory position drive mode\n",
                    elapsed(start_time), mem_drive_pos);
        for(j = 0; j < MSG_RETRY; j++)
        {
            if((ret = umlabvif_goto_position(x, mem_drive_pos, z)) >= 0)
                break;
        }
        if(ret < 0)
            return ret;
        if((ret = wait_busy(params)) < 0)
            return ret;
        if((ret = avg_y_position_with_statistics(params, &current_pos, goto_pos, start_time)) < 0)
            return ret;
        if(params->verbose)
            fprintf(stderr,"%4.3fs memory position drive completed, using step mode for final %1.3fum\n",
                    elapsed(start_time), fabs(goto_pos-current_pos));
    }
    while(fabs(current_pos - goto_pos) > params->target_pos_accuracy)
    {
        if(current_pos < goto_pos)
            speed = params->pulse_speed_value;
        else
            speed = -params->pulse_speed_value;
        if((ret = umlabvif_y_take_step(speed)) < 0)
            break;
        if((ret = wait_busy(params)) < 0)
            return ret;
        msleep(params->msg_delay);
        if((ret = avg_y_position_with_statistics(params, &current_pos, goto_pos, start_time)) < 0)
            break;
    }
    if(ret >= 0 && params->verbose)
        fprintf(stderr,"%4.3fs stepping completed, position accuracy %1.3fum\n",
                elapsed(start_time), fabs(goto_pos-current_pos));

    return ret;
}

int main(int argc, char *argv[])
{
    int home_pos, target_pos, goto_pos, current_pos, loop = 0;
    float avg_pos;
    params_t params;
    parse_args(argc, argv, &params);

    if(umlabvif_open(params.port) < 0)
	{
		fprintf(stderr,"%s: open failed\n", argv[0]);
		exit(2);
	}
    if(umlabvif_select_dev(params.dev) < 0 && umlabvif_select_dev(params.dev) < 0)
	{
		fprintf(stderr,"%s: select_dev failed\n", argv[0]);
        umlabvif_close();
		exit(2);
    }
    if(avg_position('y', &params, &avg_pos) < 0)
	{
        fprintf(stderr, "%s: initial avg position read failed\n", argv[0]);
		umlabvif_close();
		exit(3);
    }
    home_pos = (int) floor(avg_pos+0.5);
    if(params.verbose)
        fprintf(stderr, "%s: home position %d (%4.3f)\n", argv[0], home_pos, avg_pos);
    if(params.Y != UNDEF)
        target_pos = params.Y;
    else if(params.y)
        target_pos = home_pos + params.y;
    else
    {
        fprintf(stderr, "%s: Target position not defined\n", argv[0]);
        umlabvif_close();
        exit(3);
    }
    if(params.mem_drive_speed_slow_down_mode != UNDEF &&
            umlabvif_set_mem_speed_mode(params.mem_drive_speed_slow_down_mode) < 0)
    {
        fprintf(stderr,"%s: set mem speed near target mode failed\n", argv[0]);
        umlabvif_close();
        exit(2);
    }
    if(params.mem_drive_step_len != UNDEF &&
            umlabvif_set_step(params.mem_drive_step_len) < 0)
    {
        fprintf(stderr,"%s: set step failed\n", argv[0]);
        umlabvif_close();
        exit(2);
    }
    if(params.pulse_speed_mode > 0 && umlabvif_set_mode(params.pulse_speed_mode) < 0)
    {
        fprintf(stderr,"%s: set mode failed\n", argv[0]);
        umlabvif_close();
        exit(2);
    }

    if(params.verbose && umlabvif_zero_pos_unknown())
        fprintf(stderr,"zero pos is unknown or uncertain\n");

    do
    {
        loop++;
        if(loop&1)
        {
            current_pos = home_pos;
            goto_pos = target_pos;
        }
        else
        {
            current_pos = target_pos;
            goto_pos = home_pos;
        }
        if(params.verbose)
            fprintf(stderr,"driving from %d to %d (%d/%d)\n",
                    current_pos, goto_pos, loop, params.loop);
        if(accurate_goto_position(&params, goto_pos) < 0)
            break;
        if(loop < params.loop-1)
        {
            if(params.verbose)
                fprintf(stderr, "wait %dms\n", params.delay);
            msleep(params.delay);
        }
    } while(loop < params.loop);

	umlabvif_close();
	exit(0);
	return 0;
}
