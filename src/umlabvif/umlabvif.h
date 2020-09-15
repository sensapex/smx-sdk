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

#ifndef UMLABVIF_H
#define UMLABVIF_H

#if defined(_WIN32) || defined(_WIN64) || defined(WIN32) || defined(WIN64)
# include <windows.h>
# define sleep(t) Sleep((t)*1000)
# define msleep(t) Sleep((t))
# ifndef _WINDOWS
#  define _WINDOWS
# endif
# if defined(UMLABVIF_LIBRARY)
#  define UMLABVIFSHARED_EXPORT __declspec(dllexport)
# else
#  define UMLABVIFSHARED_EXPORT __declspec(dllimport)
# endif
#else
# include <unistd.h>
# define msleep(t) usleep((t)*1000)
# define UMLABVIFSHARED_EXPORT
#endif

/**
 * The precompiler comdition below is utilized by C++ compilers and is
 * ignored by pure C ones
 */

#ifdef __cplusplus
extern "C" {
#endif

extern UMLABVIFSHARED_EXPORT int umlabvif_open(const char *port);
extern UMLABVIFSHARED_EXPORT int umlabvif_close();

extern UMLABVIFSHARED_EXPORT int umlabvif_select_dev(const int dev);

extern UMLABVIFSHARED_EXPORT int umlabvif_x_position();
extern UMLABVIFSHARED_EXPORT int umlabvif_y_position();
extern UMLABVIFSHARED_EXPORT int umlabvif_z_position();

extern UMLABVIFSHARED_EXPORT int umlabvif_x_position_nm(const int average_count);
extern UMLABVIFSHARED_EXPORT int umlabvif_y_position_nm(const int average_count);
extern UMLABVIFSHARED_EXPORT int umlabvif_z_position_nm(const int average_count);

extern UMLABVIFSHARED_EXPORT int umlabvif_goto_position(const int x,
                                                        const int y,
                                                        const int z);

/**
 * Move a manipulator to position. Target coordinates are in nm scale.
 */
extern UMLABVIFSHARED_EXPORT int umlabvif_goto_position_nm(const int x_nm,
                                                           const int y_nm,
                                                           const int z_nm);

/**
 * Speeds, pen and snail modes are casted to following enums in
 * the embedded lower layer library
 *
 * UMANIPULATORCTL_MODE_1       = 1,
 * UMANIPULATORCTL_MODE_2       = 2,
 * UMANIPULATORCTL_MODE_3       = 3,
 * UMANIPULATORCTL_MODE_4       = 4,
 * UMANIPULATORCTL_MODE_5       = 5,
 * UMANIPULATORCTL_MODE_PEN     = 6,
 * UMANIPULATORCTL_MODE_SNAIL   = 7
 */

extern UMLABVIFSHARED_EXPORT int umlabvif_set_mode(const int speed);
extern UMLABVIFSHARED_EXPORT int umlabvif_get_mode();

/**
  * Target step size in nm
  */
extern UMLABVIFSHARED_EXPORT int umlabvif_set_step(const int value);
extern UMLABVIFSHARED_EXPORT int umlabvif_get_step();

/**
  * returns 1 if moving under manual control or
  * driving into a memory position
  */

extern UMLABVIFSHARED_EXPORT int umlabvif_is_busy();

/**
  * Take a step, 0 for forward, 1 for backward
  *
  * Step length is controlled by mode and the target step_length setting
  *
  */

extern UMLABVIFSHARED_EXPORT int umlabvif_x_take_step(const int backward);
extern UMLABVIFSHARED_EXPORT int umlabvif_y_take_step(const int backward);
extern UMLABVIFSHARED_EXPORT int umlabvif_z_take_step(const int backward);

/**
 * Position drive speed, particularly near target can be affected in
 * the similar way as in the Control Unit,Speeds, pen and snail modes are casted to following enums in
 * the embedded lower layer library
 *
 * Slow down near target:
 * Off      = 0,
 * Normal   = 1,
 * Extended = 2
 */

extern UMLABVIFSHARED_EXPORT int umlabvif_set_mem_speed_mode(const int mode);
extern UMLABVIFSHARED_EXPORT int umlabvif_get_mem_speed_mode();

/**
 * Check if zero pos is known
 */

extern UMLABVIFSHARED_EXPORT int umlabvif_zero_pos_unknown();

/**
 * If it was, the user may prefer initializing the zero positions
 *
 */

extern UMLABVIFSHARED_EXPORT int umlabvif_init_zero_pos();

/**
 * Perform speed calibrations
 */

extern UMLABVIFSHARED_EXPORT int umlabvif_speed_calibration();

/**
 * Enable/disable virtual X-axel
 */

extern UMLABVIFSHARED_EXPORT int umlabvif_enable_virtual_x(const unsigned char enable);

/**
 * Send a command to manipulator
 */

extern UMLABVIFSHARED_EXPORT int umlabvif_cmd(const unsigned char cmd);

/**
 * Write data in a manipulator register
 */

extern UMLABVIFSHARED_EXPORT int umlabvif_write(const unsigned char addr, const short value);

/**
 * An experimental stimulus functionality implemented inside the library
 */

extern UMLABVIFSHARED_EXPORT int  umlabvif_stimulate_init(const char *port, const int dev, const int speed_100nm_per_ms);
extern UMLABVIFSHARED_EXPORT void umlabvif_stimulate_set_pos_average(const int count);
extern UMLABVIFSHARED_EXPORT int  umlabvif_stimulate_z(const int depth_10nm, const int delay);
extern UMLABVIFSHARED_EXPORT void umlabvif_stimulate_close();

/**
 * Getters for some values saved during umlabvif_stimulate_z function call
 */

extern UMLABVIFSHARED_EXPORT int   umlabvif_get_start_ts();
extern UMLABVIFSHARED_EXPORT int   umlabvif_get_elapsed();
extern UMLABVIFSHARED_EXPORT float umlabvif_get_start_pos();
extern UMLABVIFSHARED_EXPORT float umlabvif_get_end_pos();
extern UMLABVIFSHARED_EXPORT float umlabvif_get_velocity();

// Log file handling extensions
extern UMLABVIFSHARED_EXPORT void umlabvif_set_log_to_stdout();
extern UMLABVIFSHARED_EXPORT void umlabvif_set_log_to_stderr();
extern UMLABVIFSHARED_EXPORT void umlabvif_set_log_to_file(const char *filename);

/**
 * Getters for SDK version
 */
extern UMLABVIFSHARED_EXPORT const char *umlabvif_get_version();
extern UMLABVIFSHARED_EXPORT float umlabvif_get_version_num();

/**
 * Getter for manipulator hardware id
 */
extern UMLABVIFSHARED_EXPORT int umlabvif_get_hw_id();

/**
 * Enable an optical position sensor.
 * Note! This is supported by certain manipulator models only.
 */
extern UMLABVIFSHARED_EXPORT void umlabvif_enable_tw8_pos_sensor(const int enable);

#ifdef __cplusplus
}
#endif

#endif // UMLABVIF_H
