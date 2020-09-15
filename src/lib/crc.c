/*
 * A software development kit for Sensapex Micromanipulator
 *
 * Copyright (c) 2012, Sensapex Oy
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

#include "crc.h"
#include <stdio.h>
#include <string.h>

int calc_crc(unsigned char *msg, unsigned int len)
{
	unsigned char lask;
	unsigned int i, crc = 0xFFFF;

	for (i = 0; i < len; i++)
	{
		// XXX which first ?
		crc = ((0xFF00 & crc) | (msg[i] & 0x00FF) ^ (crc & 0x00FF));
		for(lask = 8; lask !=0; lask--)
		{
			if((crc & 0x0001) == 0)
				crc >>= 1;
			else
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
		}
	}
	return crc;
}

void add_frame_crc(msg_frame_t *msg)
{ 	
	char buffer[5];
	sprintf(buffer, "%04X",(int) calc_crc(&msg->device, msg_frame_data_size));
	memcpy(msg->crc, buffer, 4);
 }

int check_frame_crc(msg_frame_t *msg)
{
	char buffer[5];
	int crc1, crc2 = calc_crc(&msg->device, msg_frame_data_size);
	memcpy(buffer, msg->crc, 4);
	buffer[4] = '\0';
	if(sscanf(buffer,"%x", &crc1) != 1)
		return -2;
	return crc1 == crc2?1:-1;
}
