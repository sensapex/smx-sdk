/*
 * A software development kit for Sensapex Micromanipulator
 *
 * Copyright (c) 2012-2014, Sensapex Oy
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

#ifndef _COMMON_H_
#define _COMMON_H_

// Device Ids
#define DEF_DEV_ID              0x31
#define DEF_CU_ID               0x0F
#define MULTICAST_DEV_ID        0x40

// Actuator IDs
#define ACTUATOR_X_ID           2
#define ACTUATOR_Y_ID           3
#define ACTUATOR_Z_ID           4

#define STX                     0x02
#define ETX                     0x03

typedef struct msg_frame_s
{
    unsigned char start;      // 0
    unsigned char device;     // 1
    unsigned char function;   // 2
    unsigned char address[2]; // 3 4
    unsigned char data[4];    // 5 6 7 8
    unsigned char crc[4];     // 9 10 11 12
    unsigned char end;        // 13
} msg_frame_t;

#define msg_frame_size sizeof(msg_frame_t)
#define msg_frame_data_size (msg_frame_size-6)

// Functions
#define WRITE_COMMAND_REG       0x30    // '0'
#define READ_STATUS_REG         0x31    // '1'
#define WRITE_SPEED_REG         0x32    // '2'
#define WRITE_DATA_REG          0x33    // '3'
#define READ_DATA_REG           0x34    // '4'
#define GET_ACTUATOR_POS_REQ    0x36    // '6'
#define GET_ACTUATOR_POS_RESP   0x37    // '7'

// Commands
#define GO_ZERO_POSITION        1
#define CALIBRATE               2
#define PC_MODE_RESERVE         4
#define PC_MODE_RELEASE         5
#define VOLTAGE_CALIBRATE       7
#define MEM_1                   10
#define RET_1                   11
#define MEM_2                   12
#define RET_2                   13
#define MEM_3                   14
#define RET_3                   15
#define	NORMAL_DRIVE_MODE       18
#define	SNAIL_DRIVE_MODE        19
#define PENETRATION_DRIVE_MODE  20
#define CALIBRATE_X_AXIS        21
#define CALIBRATE_Y_AXIS        22
#define CALIBRATE_Z_AXIS        23
#define GOING_TO_SLEEP          24
#define MEM_3_INCR_X            31
#define MEM_3_INCR_Y            32
#define MEM_3_INCR_Z            33
#define MEM_3_DECR_X            34
#define MEM_3_DECR_Y            35
#define MEM_3_DECR_Z            36
#define STOP_DRIVE              67

// Some drive speed related defines
#define V3_4_SPEED5_INCREMENT   500
#define V3_4_SPEED4_INCREMENT   400
#define V3_4_SPEED3_INCREMENT   300
#define V3_4_SPEED2_INCREMENT   64
#define V4_SPEED1_INCREMENT     16
#define V3_SPEED1_INCREMENT     8
#define V3_4_SNAIL_INCREMENT    2

#define V3_SPEED5_INCREMENT     500
#define V3_SPEED4_INCREMENT     400
#define V3_SPEED3_INCREMENT     300
#define V3_SPEED2_INCREMENT     64
#define V3_SPEED1_INCREMENT     8
#define V3_SNAIL_INCREMENT      2

#define V2_SPEED5_INCREMENT     128
#define V2_SPEED4_INCREMENT     64
#define V2_SPEED3_INCREMENT     16
#define V2_SPEED2_INCREMENT     8
#define V2_SPEED1_INCREMENT     2
#define V2_SNAIL_INCREMENT      1

// Register addresses
#define STATUS_REG              0x01
#define X_SPEED_REG             0x02
#define Y_SPEED_REG             0x03
#define Z_SPEED_REG             0x04
#define INCREMENT_REG           0x05
#define X_POSITION_REG          0x08
#define Y_POSITION_REG          0x09
#define Z_POSITION_REG          0x0A // 10
#define Z_NM_POSITION_REG       0x0E // 14
#define MEM_3_STEP              0x0F // 15
#define X_AXIS_POS_MEM_1        0x10 // 16
#define Y_AXIS_POS_MEM_1        0x11 // 17
#define Z_AXIS_POS_MEM_1        0x12 // 18
#define X_AXIS_POS_MEM_2        0x13 // 19
#define Y_AXIS_POS_MEM_2        0x14 // 20
#define Z_AXIS_POS_MEM_2        0x15 // 21
#define X_AXIS_POS_MEM_3        0x16 // 22, these mem pos registers
#define Y_AXIS_POS_MEM_3        0x17 // 23  for PC usage and
#define Z_AXIS_POS_MEM_3        0x18 // 24, thus not stored permanently
#define STEP_LENGHT             0x1F // 31
#define DEVICE_ID               0x2B // 43
#define VERSION_MAJOR_REG       0x2E // 46
#define VERSION_MINOR_REG       0x2F // 47
#define HW_ID_REG               0x5D // 93


// Some of the read only shadow registers
#define TW8_VELOCITY_SHADOW_REG         129
#define Z_DAC_VALUE_SHADOW_REG          130
#define MEM_DRIVE_TIME_SHADOW_REG       240
#define MEM_DRIVE_VELOCITY_SHADOW_REG   241

// Some mode bits in status reg
#define STATUS_SNAIL_MODE       0x0100
#define STATUS_PEN_MODE         0x0200
#define STATUS_VIRTUAL_Z        0x0400
#define STATUS_PC_MODE          0x8000

// Hardware capability bits in HWID_REG
#define MCU_HWID_NM_POS         0x01 // MCU's HW support bits in the HWID, nm position values i.e. TW8

#endif

