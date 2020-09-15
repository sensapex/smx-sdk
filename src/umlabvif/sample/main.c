/**
 * A software development kit for Sensapex Micromanipulator, simplified interface
 * for the LabView integration, sample main
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

#include "umlabvif.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main(int argc, char *argv[])
{
	int i, x,y,z;
	if(argc < 2)
	{
		fprintf(stderr, "usage: %s port [x y z]\n",argv[0]);
		exit(1);
	}
	if(umlabvif_open(argv[1]) < 0)
	{
		fprintf(stderr,"%s: open failed\n", argv[0]);
		exit(2);
	}
	if(umlabvif_select_dev(1) < 0)
	{
		fprintf(stderr,"%s: select_dev failed\n", argv[0]);
		exit(2);
	}
	if(umlabvif_set_mem_speed_mode(2) < 0)
	{
		fprintf(stderr,"%s: set mem speed mode failed\n", argv[0]);
		exit(2);
	}
	if(umlabvif_set_mode(1) < 0)
	{
		fprintf(stderr,"%s: set mode failed\n", argv[0]);
		exit(2);
	}
	if(umlabvif_set_step(30) < 0)
	{
		fprintf(stderr,"%s: set step failed\n", argv[0]);
		exit(2);
	}
	if(umlabvif_zero_pos_unknown())
	{
		fprintf(stderr,"%s: zero pos unknown, initializing\n", argv[0]);
		if(umlabvif_init_zero_pos() < 0)
		{
			fprintf(stderr,"%s: init zero pos failed\n", argv[0]);
			exit(2);
		}
	}
#ifdef TEST_SPEED_CALIBRATION_COMMAND
 else
	{
		fprintf(stderr,"%s: speed calibration\n", argv[0]);
		if(umlabvif_speed_calibration() < 0)
		{
			fprintf(stderr,"%s: speed calib failed\n", argv[0]);
			exit(2);
		}
	}
#endif

	if((x = umlabvif_x_position()) < 0)
	{
		fprintf(stderr, "%s: x_position failed\n", argv[0]);
		umlabvif_close();
		exit(3);
	}
	if((y = umlabvif_y_position()) < 0)
	{
		fprintf(stderr, "%s: y_position failed\n", argv[0]);
		umlabvif_close();
		exit(3);
	}
	if((z = umlabvif_z_position()) < 0)
	{
		fprintf(stderr, "%s: z_position failed\n", argv[0]);
		umlabvif_close();
		exit(3);
	}
	printf("%s: current position %d %d %d\n", argv[0],x,y,z);

	printf("%s: current mode %d and step %d\n", argv[0],
		   umlabvif_get_mode(), umlabvif_get_step());

	if(argc < 5)
	{
		if(argc == 3 && sscanf(argv[2],"%d", &i) == 1)
		{
			printf("%s: Stepping Z %d step%s forward\n", argv[0],i,i>1?"s":"");
			for( ; i > 0; i--)
			{
				if(umlabvif_z_take_step(0) < 0)
				{
					fprintf(stderr, "%s: take step failed\n", argv[0]);
					break;
				}
			}
		}
		umlabvif_close();
		exit(0);
	}
	if(sscanf(argv[2],"%d",&x) != 1 || x < 0 || x > 20500)
	{
		fprintf(stderr, "%s: invalid x %s\n", argv[0], argv[2]);
		umlabvif_close();
		exit(4);
	}
	if(sscanf(argv[3],"%d",&y) != 1 || y < 0 || y > 20500)
	{
		fprintf(stderr, "%s: invalid y %s\n", argv[0], argv[3]);
		umlabvif_close();
		exit(4);
	}
	if(sscanf(argv[4],"%d",&z) != 1 || z < 0 || z > 20500)
	{
		fprintf(stderr, "%s: invalid z %s\n", argv[0], argv[4]);
		umlabvif_close();
		exit(4);
	}
	if(umlabvif_goto_position(x, y, z) < 0)
	{
		fprintf(stderr, "%s: goto failed\n", argv[0]);
		umlabvif_close();
		exit(5);
	}
	umlabvif_close();
	exit(0);
	return 0;
}

