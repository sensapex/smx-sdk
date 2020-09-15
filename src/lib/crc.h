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

#ifndef CRC_H
#define CRC_H

#include "common.h"

extern void add_frame_crc(msg_frame_t *frame);
extern int check_frame_crc(msg_frame_t *frame);

#endif

