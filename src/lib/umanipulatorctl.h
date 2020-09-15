/**
 * @file    umanipulatorctl.h
 * @author  Veli-Matti Kananen  (veli-matti.kananen@sensapex.com)
 * @date    30 Jan 2015
 * @brief   This file contains a public API for Sensapex Micromanipulator SDK
 * @copyright   Copyright (c) 2012-2015 Sensapex. All rights reserved
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
 */

#ifndef UMANIPULATORCTL_H
#define UMANIPULATORCTL_H

/**
  \def UMANIPULATORCTLSHARED_EXPORT
  Internal compiler flag
*/

#if defined(_WIN32) || defined(_WIN64) || defined(WIN32) || defined(WIN64)
# ifndef _WINDOWS
#  define _WINDOWS
# endif
// This is needed e.g. by the labview wrapper layer embedding this library
# ifdef  UMANIPULATORCTLSHARED_DO_NOT_EXPORT
#  define UMANIPULATORCTLSHARED_EXPORT
# else
#  if defined(UMANIPULATORCTL_LIBRARY)
#   define UMANIPULATORCTLSHARED_EXPORT __declspec(dllexport)
#  else
#   define UMANIPULATORCTLSHARED_EXPORT __declspec(dllimport)
#  endif
# endif
#else
# define UMANIPULATORCTLSHARED_EXPORT
#endif

#ifdef _WINDOWS
# include <windows.h>
UMANIPULATORCTLSHARED_EXPORT HRESULT __stdcall DllRegisterServer(void);
UMANIPULATORCTLSHARED_EXPORT HRESULT __stdcall DllUnregisterServer(void);
#endif

/*
 * The precompiler condition below is utilized by C++ compilers and is
 * ignored by pure C ones
 */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Error enums
 */
typedef enum umanipulatorctl_error_e
{
    UMANIPULATORCTL_NO_ERROR  = 0,      /**< No error */
    UMANIPULATORCTL_OS_ERROR  = -1,     /**< Operating System level error */
    UMANIPULATORCTL_NOT_OPEN  = -2,     /**< The target device is not connected */
    UMANIPULATORCTL_TIMEOUT   = -3,     /**< Timeout occured */
    UMANIPULATORCTL_INVALID_ARG  = -4,  /**< Illegal command argument */
    UMANIPULATORCTL_INVALID_DEV  = -5,  /**< Illegal Device Id */
    UMANIPULATORCTL_INVALID_RESP = -6,  /**< Illegal response received */
    UMANIPULATORCTL_INVALID_CRC  = -7,  /**< Input mesasge CRC checksum failure */
} umanipulatorctl_error_t;

/**
 * @brief Manipulator status enums
 *
 * These cause busy state
 */
typedef enum umanipulatorctl_status_e
{
    UMANIPULATORCTL_STATUS_READ_ERROR = -1,     /**< Failure at status reading */
    UMANIPULATORCTL_STATUS_OK         =  0,     /**< No error */
    UMANIPULATORCTL_STATUS_X_MOVING   = 0x01,   /**< X-actuator is moving */
    UMANIPULATORCTL_STATUS_Y_MOVING   = 0x02,   /**< Y-actuator is moving */
    UMANIPULATORCTL_STATUS_Z_MOVING   = 0x04,   /**< Z-actuator is moving */
    UMANIPULATORCTL_STATUS_X_BUSY     = 0x10,   /**< X-actuator is busy */
    UMANIPULATORCTL_STATUS_Y_BUSY     = 0x20,   /**< Y-actuator is busy */
    UMANIPULATORCTL_STATUS_Z_BUSY     = 0x40,   /**< Z-actuator is busy */
    UMANIPULATORCTL_STATUS_JAMMED     = 0x80    /**< A manipulator is stucked */
} umanipulatorctl_status_t;

/**
 * @brief Manipulator operating modes
 */
typedef enum umanipulatorctl_mode_e
{
    UMANIPULATORCTL_MODE_READ_ERROR = -1,   /**< Mode reading failure */
    UMANIPULATORCTL_MODE_UNKNOWN = 0,       /**< Undefined mode */
    UMANIPULATORCTL_MODE_1       = 1,       /**< Speed mode 1 */
    UMANIPULATORCTL_MODE_2       = 2,       /**< Speed mode 2 */
    UMANIPULATORCTL_MODE_3       = 3,       /**< Speed mode 3 */
    UMANIPULATORCTL_MODE_4       = 4,       /**< Speed mode 4 */
    UMANIPULATORCTL_MODE_5       = 5,       /**< Speed mode 5 */
    UMANIPULATORCTL_MODE_PEN     = 6,       /**< Penetration mode */
    UMANIPULATORCTL_MODE_SNAIL   = 7        /**< Snail mode */
} umanipulatorctl_mode_t;

/*
 * Some defaults
 */
#define UMANIPULATORCTL_MAX_MANIPULATORS 15         /**< Max count of concurrent manipulators */
#define UMANIPULATORCTL_DEF_REFRESH_TIME 2500       /**< The default timeout (ms) of serial port */
#define UMANIPULATORCTL_MAX_POSITION     20400      /**< The upper absolute position limit for actuators */

#define UMANIPULATORCTL_TIMELIMIT_DISABLED 99999    /**< Skip the internal position cache.
                                                         Use this definition as a parameter to read an actuator position
                                                         directly from a manipulator */

/**
 * @brief Positions used in #umanipulatorctl_state_t
 */
typedef struct umanipulatorctl_positions_s
{
    unsigned long x;                /**< X-actuator position */
    unsigned long x_last_updated;   /**< Timestamp (in milliseconds) when X-actuator position has been updated */
    unsigned long y;                /**< Y-actuator position */
    unsigned long y_last_updated;   /**< Timestamp (in milliseconds) when Y-actuator position has been updated */
    unsigned long z;                /**< Z-actuator position */
    unsigned long z_last_updated;   /**< Timestamp (in milliseconds) when Z-actuator position has been updated */
} umanipulatorctl_positions_t;

/**
 * @brief MCU device specific data in #umanipulatorctl_state_t
 */
typedef struct umanipulatorctl_mcu_info_s
{
    int mcu_ver_major;                                  /**< Major number of mcu firmware version */
    int mcu_ver_minor;                                  /**< Minor number of mcu firmware version */
    int mcu_hw_id;                                      /**< MCU hardware model ID */
} umanipulatorctl_mcu_info_t;

/**
 * @brief The state struct, pointer to this is the session handle in the C API
 */
typedef struct umanipulatorctl_state_s
{
    struct extserial_port_state_s *port;                /**< Pointer to serial port details */
    unsigned char last_device_sent;                     /**< Device ID of selected and/or communicated target device */
    unsigned char last_device_received;                 /**< ID of device that has sent the latest message */
    unsigned char last_device_position_read_received;   /**< The ID of device that has responsed to a position update request */
    unsigned long last_received_time;                   /**< Timestemp of the latest incoming message */
    unsigned long refresh_time_limit;                   /**< Refresh timelimit for the position cache */
    umanipulatorctl_positions_t last_positions[UMANIPULATORCTL_MAX_MANIPULATORS];   /**< Actuator position cache */
    umanipulatorctl_positions_t target_positions[UMANIPULATORCTL_MAX_MANIPULATORS]; /**< Target position cache */
    umanipulatorctl_mcu_info_t mcu_info[UMANIPULATORCTL_MAX_MANIPULATORS]; /**< Manipulator details */
    int last_error;                                     /**< Error code of the latest error */
    char errorstr_buffer[80];                           /**< The work buffer of the latest error string handler */
} umanipulatorctl_state_t;

/**
 * @brief Open a local com port
 *
 * @param   port    Serial port ID
 * @param   timeout Serial port timeout value
 * @return  Pointer to created session handle. NULL if an error occured
 */
UMANIPULATORCTLSHARED_EXPORT umanipulatorctl_state_t *umanipulatorctl_open(const char *port,
                                                                           const unsigned int timeout);
/**
 * @brief Close the port
 *
 * (if open) and free the state structure allocated in open
 *
 * @param   hndl    Pointer to session handle
 * @return  None
 */
UMANIPULATORCTLSHARED_EXPORT void umanipulatorctl_close(umanipulatorctl_state_t *hndl);

/*
 * For all other C functions returning int, a negative values means error,
 * got the possible error number or description using some of these,
 * often the last one is enough
 */

/**
 * @brief This function can be used to get the actual operating system level error code
 * when umanipulatorctl_last_error returns UMANIPULATORCTL_OS_ERROR.
 *
 * @param   hndl    Pointer to session handle
 * @return  Error code
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_last_os_error(const umanipulatorctl_state_t *hndl);

/**
 * @brief Get the latest error
 *
 * @param   hndl    Pointer to session handle
 * @return  `umanipulatorctl_error_t` code
 */
UMANIPULATORCTLSHARED_EXPORT umanipulatorctl_error_t umanipulatorctl_last_error(const umanipulatorctl_state_t *hndl);

/**
 * @brief Translate an error code to human readable format
 *
 * @param   ret_code    Error code to be traslated
 * @return  Pointer to an error string
 */
UMANIPULATORCTLSHARED_EXPORT const char *umanipulatorctl_errorstr(int ret_code);

/**
 * @brief Get the latest error in human readable format
 *
 * @param   hndl    Pointer to session handle
 * @return  Pointer to an error string
 */
UMANIPULATORCTLSHARED_EXPORT const char *umanipulatorctl_last_errorstr(umanipulatorctl_state_t *hndl);

/**
 * @brief SDK library version
 *
 * @return  Pointer to version string
 */
UMANIPULATORCTLSHARED_EXPORT const char *umanipulatorctl_get_version();

/**
 * @brief Get the manipulator firmware version
 *
 * @param       hndl    Pointer to session handle
 * @param[out]  major   Pointer to an allocated buffer for firmware major number
 * @param[out]  minor   Pointer to an allocated buffer for firmware minor number
 * @return  Negative value if an error occured. Zero or positive value otherwise
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_read_version(umanipulatorctl_state_t * hndl,
                                                              int *major, int *minor);

/*
 * The simplified interface stores both the device and the refresh time into the state
 * sturcture and use those for all requests
 */

/**
 * @brief Select a manipulator
 *
 * @param       hndl    Pointer to session handle
 * @param       dev     Device ID of manipulator
 * @return  Negative value if an error occured. Zero or positive value otherwise
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_select_dev(umanipulatorctl_state_t *hndl,
                                                             const unsigned char dev);

/**
 * @brief Set refresh timelimit for the session position cache
 *
 * @param       hndl    Pointer to session handle
 * @param       value   New refresh timelimit for position cache (in milliseconds).
 * @return  Negative value if an error occured. Zero or positive value otherwise
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_set_refresh_time_limit(umanipulatorctl_state_t *hndl,
                                                             const unsigned int value);

/**
 * @brief Change the serial port timeout
 *
 * Initial value set when port opened
 *
 * @param   hndl    Pointer to session handle
 * @param   value   New serial port timeout (in milliseconds)
 * @return  Negative value if an error occured. Zero or positive value otherwise
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_set_timeout(umanipulatorctl_state_t *hndl,
                                                             const unsigned int value);

/**
 * @brief Read the session status
 *
 * @param   hndl    Pointer to session handle
 * @return  Session status. See `umanipulatorctl_status_t` for bit definitions
 */
UMANIPULATORCTLSHARED_EXPORT umanipulatorctl_status_t umanipulatorctl_status(umanipulatorctl_state_t *hndl);

/*
 * Status is a bit map and not all bits mean the manipulator being busy.
 * Detect busy state using these functions
 */
/**
 * @brief Check if the manipulator is busy
 *
 * @param   hndl    Pointer to session handle
 * @return  Positive value if the target manipulator is busy.
 *          Zero if the target manipulator is not busy. Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_is_busy(umanipulatorctl_state_t *hndl);

/**
 * @brief Check a busy status
 *
 * @param   status   `umanipulatorctl_status_t` value to be checked
 * @return  Positive value if 'status' is a busy status.
 *          Zero if 'status' is not a busy status. Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_is_busy_status(umanipulatorctl_status_t status);


/**
 * @brief Get manipulator mode
 *
 * e.g. Pen, Snail or Speed 1-5
 *
 * @param   hndl    Pointer to session handle
 * @return  Current mode
 */
UMANIPULATORCTLSHARED_EXPORT umanipulatorctl_mode_t umanipulatorctl_get_mode(umanipulatorctl_state_t * hndl);

/**
 * @brief Set manipulator mode
 *
 * Note that should not be used unless CU is in PC mode due
 * CU and manipulator states will not be syncronized.
 *
 * @param   hndl    Pointer to session handle
 * @param   mode    New mode
 * @return  Positive value if the operation was successful.
 *          Zero or negative value otherwise
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_set_mode(umanipulatorctl_state_t * hndl,
                                                          const umanipulatorctl_mode_t mode);

/**
 * @brief Get target step size in nm
 *
 * (not used in pen mode and scaled up at higher speeds)
 *
 * @param   hndl    Pointer to session handle
 * @return  Step length. Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_get_step(umanipulatorctl_state_t * hndl);

/**
 * @brief Set target step size in nm
 *
 * @param   hndl    Pointer to session handle
 * @param   value   New step length (in nm)
 * @return  Positive value if the operation was successful. Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_set_step(umanipulatorctl_state_t * hndl,
                                                          const int value);

/**
 * @brief Obtain the position of x-actuator
 *
 * Uses the cache value if found and refreshed inside the time limit
 *
 * @param   hndl    Pointer to session handle
 * @return  The x-actuator position
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_x_position(umanipulatorctl_state_t *hndl);

/**
 * @brief Obtain the position of y-actuator
 *
 * Uses the cache value if found and refreshed inside the time limit
 *
 * @param   hndl    Pointer to session handle
 * @return  The y-actuator position
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_y_position(umanipulatorctl_state_t *hndl);

/**
 * @brief Obtain the position of z-actuator.
 *
 * Uses the cache value if found and refreshed inside the time limit
 *
 * @param   hndl    Pointer to session handle
 * @return  The z-actuator position
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_z_position(umanipulatorctl_state_t *hndl);

/**
 * @brief Obtain the position of actuators using pointers.
 *
 * @param   hndl    Pointer to session handle
 * @param   x       Pointer to an allocated buffer for x-actuator position. (may be NULL)
 * @param   y       Pointer to an allocated buffer for y-actuator position. (may be NULL)
 * @param   z       Pointer to an allocated buffer for z-actuator position. (may be NULL)
 * @return  The number of storead values.
 *          Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_get_position(umanipulatorctl_state_t *hndl,
                                                           int *x, int *y, int *z);

/**
 * @brief Store the current position into memory location.
 *
 * @param   hndl    Session handle
 * @return  Positive value if the operation was successful.
 *          Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_store_mem_current_position(umanipulatorctl_state_t *hndl);

/**
 * @brief Store the given position into memory location.
 *
 * @param   hndl    Pointer to session handle
 * @param   x       x-coordinate
 * @param   y       y-coordinate
 * @param   z       z-coordinate
 * @return  Positive value if the operation was successful.
 *          Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_store_mem_position(umanipulatorctl_state_t *hndl,
                                                                    const int x, const int y, const int z);
/**
 * @brief Goto to the stored position
 *
 * @param   hndl    Pointer to session handle
 * @return  Positive value if the operation was successful. Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_goto_mem_position(umanipulatorctl_state_t *hndl);

/**
 * @brief Retrieve the stored position.
 *
 * This is a local copy stored by last #umanipulatorctl_store_mem_position,
 * not read from manipulator and thus not what was stored by
 * #umanipulatorctl_store_mem_current_position
 *
 * @param       hndl    Pointer to session handle
 * @param[out]  x       Pointer to an allocated buffer for x-actuator position. (may be NULL)
 * @param[out]  y       Pointer to an allocated buffer for y-actuator position. (may be NULL)
 * @param[out]  z       Pointer to an allocated buffer for z-actuator position. (may be NULL)
 * @return      Positive value if the operation was successful.
 *              Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_get_mem_position(umanipulatorctl_state_t *hndl,
                                                           int *x, int *y, int *z);
/**
 * @brief  Stop moving actuators.
 *
 * @param       hndl    Pointer to session handle
 * @return      Positive value if the operation was successful. Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_stop(umanipulatorctl_state_t * hndl);

/**
 * @brief  Set the control unit mode (PC mode or normal mode).
 *
 * @param       hndl            Pointer to session handle
 * @param       enablePCMode    Not equal to zero enables PC Mode. Zero (0) disables PC Mode.
 * @return      Positive value if the operation was successful.
 *              Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_set_control_unit_pcmode(
        umanipulatorctl_state_t * hndl, const int enablePCMode);

/*
 * Lower layer API carrying the device id and extended arguments.
 * These functions are used internally by the above API functions.
 * They may be used also if the application needs to control multiple manipulators
 * at the same time
 */
/**
 * @brief Ping manipulator
 *
 * @param   hndl    Pointer to session handle
 * @param   dev     Device ID
 * @return  Positive value if the operation was successful.
 *          Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_ping(umanipulatorctl_state_t *hndl,
                                                           const unsigned char dev);
/**
 * @brief Lower layer API to check if a manipulator is busy.
 *
 * Note! This API is should not be used directly.
 *
 * @param   hndl    Pointer to session handle
 * @param   dev     Device ID
 * @return  Positive value if the operation was successful.
 *          Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_is_busy_ext(umanipulatorctl_state_t *hndl,
                                                            const unsigned char dev);
/**
 * @brief Lower layer API to check a manipulator status.
 *
 * Note! This API is should not be used directly.
 *
 * @param   hndl    Pointer to session handle
 * @param   dev     Device ID
 * @return  Positive value if the operation was successful.
 *          Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_status_ext(umanipulatorctl_state_t *hndl,
                                                            const unsigned char dev);

/**
 * @brief Lower layer API to check a manipulator firmware version.
 *
 * Note! This API is should not be used directly.
 *
 * @param   hndl    Pointer to session handle
 * @param   dev     Device ID
 * @param[out]  major   Pointer to an allocated buffer for firmware major number
 * @param[out]  minor   Pointer to an allocated buffer for firmware minor number
 * @return  Positive value if the operation was successful.
 *          Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_read_version_ext(umanipulatorctl_state_t * hndl,
                                                              const unsigned char dev,
                                                              int *major, int *minor);

/**
 * @brief Lower layer API to get current mode of manipulator.
 *
 * Note! This API is should not be used directly.
 *
 * @param   hndl    Pointer to session handle
 * @param   dev     Device ID
 * @return  Positive value if the operation was successful.
 *          Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT umanipulatorctl_mode_t umanipulatorctl_get_mode_ext(umanipulatorctl_state_t * hndl,
                                                              const unsigned char dev);

/**
 * @brief An advanced API to store current position.
 *
 * Note! Frequently writing to storage_id:s 1 and 2 under PC SW control should be avoided due
 * they are stored in non-volatile memory with limited erase cycle count.
 *
 * @param   hndl        Pointer to session handle
 * @param   dev         Device ID
 * @param   storage_id  The destination memory location.
 *                      (0 = default, 1 = home, 2 = target, 3 = get values from the manipulator)
 * @return  Positive value if the operation was successful.
 *          Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_store_mem_current_position_ext(umanipulatorctl_state_t *hndl,
                                                                                const unsigned char dev,
                                                                                const unsigned char storage_id);

/**
 * @brief An advanced API to store the given position.
 *
 * Note! Frequently writing to storage_id:s 1 and 2 under PC SW control should be avoided due
 * they are stored in non-volatile memory with limited erase cycle count.
 *
 * @param   hndl        Pointer to session handle
 * @param   dev         Device ID
 * @param   x           x-coordinate
 * @param   y           y-coordinate
 * @param   z           z-coordinate
 * @param   storage_id  The destination memory location.
 *                      (0 = default, 1 = home, 2 = target, 3 = get values from the manipulator)
 * @return  Positive value if the operation was successful.
 *          Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_store_mem_position_ext(umanipulatorctl_state_t *hndl,
                                                                        const unsigned char dev,
                                                                        const int x, const int y, const int z,
                                                                        const unsigned char storage_id);

/**
 * @brief An advanced API to move actuators to stored position.
 *
 * @param   hndl        Pointer to session handle
 * @param   dev         Device ID
 * @param   storage_id  The destination memory location.
 *                      (0 = default, 1 = home, 2 = target, 3 = get values from the manipulator)
 * @return  Positive value if the operation was successful.
 *          Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_goto_mem_position_ext(umanipulatorctl_state_t *hndl,
                                                                       const unsigned char dev,
                                                                       const unsigned char storage_id);

/**
 * @brief An advanced API to stop manipulator actuators.
 *
 * @param   hndl        Pointer to session handle
 * @param   dev         Device ID
 * @return  Positive value if the operation was successful.
 *          Zero or negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_stop_ext(umanipulatorctl_state_t * hndl,
                                                          const unsigned char dev);

/**
 * @brief An advanced API to read a stored position.
 *
 * @param       hndl        Pointer to session handle
 * @param       dev         Device ID
 * @param[out]  x           Pointer to an allocated buffer for x-actuator position.
 * @param[out]  y           Pointer to an allocated buffer for y-actuator position.
 * @param[out]  z           Pointer to an allocated buffer for z-actuator position.
 * @param       storage_id  The source memory location.
 *                      (0 = default, 1 = home, 2 = target, 3 = get values from the manipulator)
 * @return      Positive value if the operation was successful.
 *              Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_get_mem_position_ext(umanipulatorctl_state_t *hndl,
                                                                      const unsigned char dev,
                                                                      int *x, int *y, int *z,
                                                                      const unsigned char storage_id);

/**
 * @brief Lower layer API to get a step length.
 * Note! This API is should not be used directly.
 *
 * @param   hndl    Pointer to session handle
 * @param   dev     Device ID
 * @return  Current step lenght (in nm).
 *          Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_get_step_ext(umanipulatorctl_state_t * hndl,
                                                              const unsigned char dev);

/**
 * @brief Lower layer API to set a step length.
 * Note! This API is should not be used directly.
 *
 * @param   hndl    Pointer to session handle
 * @param   dev     Device ID
 * @param   value   New step lenght (in nm)
 * @return  Positive value if the operation was successful.
 *          Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_set_step_ext(umanipulatorctl_state_t * hndl,
                                                              const unsigned char dev,
                                                              const int value);

/**
 * @brief Update the position cache
 *
 * This function can be called to read the serial port and thus update the positions
 * into the cache from the responses to the Control Unit in normal i.e. not in PC mode
 *
 * @param   hndl    Pointer to session handle
 * @return  Positive value indicates the count of received messages.
 *          Zero if no message received. Negative value indicates an error.
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_recv(umanipulatorctl_state_t *hndl);

/**
 * @brief An advanced API allowing to control the position value timings.
 *
 * A zero time_limit reads cached positions without sending any messages
 * to the manipulator.
 *
 * Note! Intented to be used when the Control Unit is not in PC mode.
 *
 * @param       hndl        Pointer to session handle
 * @param       dev         Device ID
 * @param       time_limit  Timelimit of cache values. If 0 then cached positions are used always.
 *                          If
 * @param[out]  x           Pointer to an allocated buffer for x-actuator position
 * @param[out]  y           Pointer to an allocated buffer for y-actuator position
 * @param[out]  z           Pointer to an allocated buffer for z-actuator position
 * @return      Positive value if the operation was successful.
 *              Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_get_position_ext(umanipulatorctl_state_t *hndl,
                                                           const unsigned char dev,
                                                           const unsigned int time_limit,
                                                           int *x, int *y, int *z);

/**
 * @brief An advanced API allowing to control the x-actuator position value timings.
 *
 * A zero time_limit reads cached positions without sending any messages
 * to the manipulator.
 *
 * Note! Intented to be used when the Control Unit is not in PC mode.
 *
 * If elapsed is not NULL, the referred variable will be populated milli seconds
 * since the position was updated
 *
 * @param       hndl        Pointer to session handle
 * @param       dev         Device ID
 * @param       time_limit  Timelimit of cache values. If `time_limit` is 0 then
 * cached positions are used always. If `time_limit` is #UMANIPULATORCTL_TIMELIMIT_DISABLED
 * then positions are read from manipulator always.
 * @param[out]  elapsed     The used timestamp for elapsed comparisation.
 * @return      The absolute position of x-actuator.
 *              Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_x_position_ext(umanipulatorctl_state_t *hndl,
                                                             const unsigned char dev,
                                                             const unsigned int time_limit,
                                                             unsigned int *elapsed);

/**
 * @brief An advanced API allowing to control the y-actuator position value timings.
 *
 * A zero time_limit reads cached positions without sending any messages
 * to the manipulator.
 *
 * Note! Intented to be used when the Control Unit is not in PC mode.
 *
 * If elapsed is not NULL, the referred variable will be populated milli seconds
 * since the position was updated
 *
 * @param       hndl        Pointer to session handle
 * @param       dev         Device ID
 * @param       time_limit  Timelimit of cache values. If 0 then cached positions are used always.
 * @param[out]  elapsed     The used timestamp for elapsed comparisation.
 * @return      The absolute position of y-actuator.
 *              Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_y_position_ext(umanipulatorctl_state_t *hndl,
                                                             const unsigned char dev,
                                                             const unsigned time_limit,
                                                             unsigned int *elapsed);

/**
 * @brief An advanced API allowing to control the z-actuator position value timings.
 *
 * A zero time_limit reads cached positions without sending any messages
 * to the manipulator.
 *
 * Note! Intented to be used when the Control Unit is not in PC mode.
 *
 * If elapsed is not NULL, the referred variable will be populated milli seconds
 * since the position was updated
 *
 * @param       hndl        Pointer to session handle
 * @param       dev         Device ID
 * @param       time_limit  Timelimit of cache values. If 0 then cached positions are used always.
 * @param[out]  elapsed     The used timestamp for elapsed comparisation.
 * @return      The absolute position of z-actuator.
 *              Negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_z_position_ext(umanipulatorctl_state_t *hndl,
                                                             const unsigned char dev,
                                                             const unsigned int time_limit,
                                                             unsigned int *elapsed);

/**
 * @brief An advanced API to set mode with device id and extended penetration step length (in nm)
 *
 * Note! This API should not be used unless CU is in PC mode due
 * CU and manipulator states will not be syncronized.
 *
 * @param   hndl        Pointer to session handle
 * @param   dev         Device ID
 * @param   mode        New mode
 * @param   extended_pen_step   Coarse ratio from nm to step count in pen mode.
 *                              This parameter has impact pnly when `mode` is UMANIPULATORCTL_MODE_PEN
 * @return  Positive value indicates the count of received messages.
 *          Zero if no message received. Negative value indicates an error.
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_set_mode_ext(umanipulatorctl_state_t * hndl,
                                                              const unsigned char dev,
                                                              const umanipulatorctl_mode_t mode,
                                                              const int extended_pen_step);

/**
 * @brief Send a command to manipulator.
 *
 * Note! This API is mainly for Sensapex internal development purpose and
 * should not be used unless you really know what you are doing.
 *
 * WARNING: Abusing this function may void device warranty
 *
 * @param   hndl    Pointer to session handle
 * @param   dev     Device ID
 * @param   cmd     Command ID
 * @return  Positive value if the operation was successful.
 *          Zero or negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_cmd(umanipulatorctl_state_t *hndl,
                                                           const unsigned char dev,
                                                           const unsigned char cmd);

/**
 * @brief Read a register content from Manipulator.
 *
 * Note! This API is mainly for Sensapex internal development purpose and
 * should not be used unless you really know what you are doing.
 *
 * WARNING: Abusing this function may void device warranty
 *
 * @param   hndl    Pointer to session handle
 * @param   dev     Device ID
 * @param   addr    Register address
 * @return  Positive value if the operation was successful.
 *          Zero or negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_read(umanipulatorctl_state_t *hndl,
                                                           const unsigned char dev,
                                                           const unsigned char addr);

/**
 * @brief Write a value in Manipulator register
 *
 * Note! This API is mainly for Sensapex internal development purpose and
 * should not be used unless you really know what you are doing.
 *
 * WARNING: Abusing this function may void device warranty
 *
 * @param   hndl    Pointer to session handle
 * @param   dev     Device ID
 * @param   addr    Register address
 * @param   val     Data to be written
 * @return  Positive value if the operation was successful.
 *          Zero or negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_write(umanipulatorctl_state_t *hndl,
                                                           const unsigned char dev,
                                                           const unsigned char addr,
                                                           const unsigned short val);

/**
 * @brief Reads an actuator position from Manipulator.
 *
 * @param   hndl    Pointer to session handle
 * @param   dev     Device ID
 * @param   addr    actuator ID
 * @return  Positive value if the operation was successful.
 *          Zero or negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_read_position(umanipulatorctl_state_t *hndl,
                                                           const unsigned char dev,
                                                           const unsigned char addr);

/**
 * @brief Write a value in Manipulator speed register
 *
 * Note! This API is mainly for Sensapex internal development purpose and
 * should not be used unless you really know what you are doing.
 *
 * WARNING: Abusing this function may void device warranty
 *
 * @param   hndl    Pointer to session handle
 * @param   dev     Device ID
 * @param   addr    Target register address
 * @param   val     Data to be written in register
 * @return  Positive value if the operation was successful.
 *          Zero or negative value indicates an error
 */
UMANIPULATORCTLSHARED_EXPORT int umanipulatorctl_write_speed(umanipulatorctl_state_t *hndl,
                                                           const unsigned char dev,
                                                           const unsigned char addr,
                                                           const unsigned short val);

/**
* @brief Set the axis naming
*
* This function maps SDK axis on Moter Control Unit (MCU) way or Control Unit (CU) way.

* @param   hndl             Pointer to session handle
* @param   mcuAxisNaming    Indicatas wheater MCU or CU axis naming is used.
*                           Zero enables CU type axis naming.
*                           Other values enables MCU type axis naming.
*/
UMANIPULATORCTLSHARED_EXPORT void umanipulatorctl_setAxisNaming(umanipulatorctl_state_t *hndl,
                                                                const int mcuAxisNaming);

/*
 * End of the C-API
 */

#ifdef __cplusplus
} // end of extern "C"

#define UMANIPULATORCTL_USE_LAST_DEV  0     /**< Use the same device ID than last time */

/**
 * @brief The UManipulatorCtl class.
 *
 * The class is a simple inline C++ wrapper class for a public Sensapex µManipulator SDK.
 * The class is implemented without any dependency to Qt or std class libraries.
 */
class UManipulatorCtl
{
public:
    /**
     * @brief Constructor
     */
    UManipulatorCtl() {  _handle = NULL; }
    /**
     * @brief Destructor
     */
    virtual ~UManipulatorCtl() { if(_handle) umanipulatorctl_close(_handle); }

    /**
    * @brief Set the axis naming
    *
    * This function maps SDK axis on Moter Control Unit (MCU) way or Control Unit (CU) way.
    *
    * @param   useMCUaxis    Indicatas wheater MCU or CU axis naming is used.
    *                        Zero enables CU type axis naming.
    *                        Other values enables MCU type axis naming.
    */
    void confAxis(const int useMCUaxis) {
        umanipulatorctl_setAxisNaming(_handle, useMCUaxis);
    }

    /**
     * @brief Open a local com port to communicate with a manipulator
     * @param port      Pointer to COM port desription string
     * @param timeout   COM port timeout value (in milliseconds)
     * @return `true` if operation was successful, `false` otherwise
     */
    bool open(const char *port, const unsigned int timeout)
    {	return (_handle = umanipulatorctl_open(port, timeout)) != NULL; }

    /**
     * @brief Check wheather a local com port is open for manipulator communication
     * @return `true` if this instance of `UManipulatorCtl` holds an open
     * serial port for manipulator. `false` otherwise.
     */
    bool isOpen()
    { 	return _handle != NULL; }

    /**
     * @brief Close the local com port (if open) and free the state structure allocated in open
     */
    void close()
    {	umanipulatorctl_close(_handle); _handle = NULL; }

    /**
     * @brief SDK library version
     * @return Pointer to version string
     */
    static const char *version() { return umanipulatorctl_get_version(); }

    /**
     * @brief Select a manipulator
     * @param dev   Device ID
     * @return `true` if operation was successful, `false` otherwise
     */
    bool select(const int dev)
    {	return  umanipulatorctl_select_dev(_handle, getDev(dev)) >= 0; }

    /**
     * @brief Check if a manipulator is available for communication
     * @param dev   Device ID
     * @return `true` if operation was successful, `false` otherwise
     */
    bool ping(const int dev)
    {	return  umanipulatorctl_ping(_handle, getDev(dev)) >= 0; }

    /**
     * @brief Get the status of manipulator
     * @return Manipulator status. See `umanipulatorctl_status_t` for bit definitions
     */
    umanipulatorctl_status_t status()
    { 	return	umanipulatorctl_status(_handle); }

    /**
     * @brief Check if the manipulator is busy.
     * @return Greater than zero if the manipulator is busy. Zero if manipulator is not busy.
     *         Less than zero indicates an error.
     * See `umanipulatorctl_status_t` for bit definitions. Returns `false` otherwise.
     */
    int busy()
    { 	return	umanipulatorctl_is_busy(_handle); }

    /**
     * @brief Check if a status is an error status
     * @param status    Value to be checked
     * @return `true` is `status` is an error status.
     * See `umanipulatorctl_status_t` for bit definitions. Returns `false` otherwise.
     */
    static bool errorStatus(umanipulatorctl_status_t status)
    {	return	(int)status < 0; }

    /**
     * @brief Check if a status is a busy status
     * @param status    Value to be checked
     * @return `true` is `status` is a busy status.
     * See `umanipulatorctl_status_t` for bit definitions. Returns `false` otherwise.
     */
    static bool busyStatus(umanipulatorctl_status_t status)
    {	return	umanipulatorctl_is_busy_status(status) > 0; }

    /**
     * @brief Execute a manipulator command
     * @param cmd   Command ID
     * @param dev   Device ID
     * @return `true` if operation was successful, `false` otherwise
     */
    bool cmd(const unsigned char cmd, const int dev = UMANIPULATORCTL_USE_LAST_DEV)
    {	return	umanipulatorctl_cmd(_handle, getDev(dev), cmd) >= 0; }

    /**
     * @brief Read data from manipulator register
     * @param addr  The address of target register
     * @param dev   Device ID
     * @return `true` if operation was successful, `false` otherwise
     */
    int read(const unsigned char addr, const int dev = UMANIPULATORCTL_USE_LAST_DEV)
    {	return  umanipulatorctl_read(_handle, getDev(dev), addr); }

    /**
     * @brief Write data in manipulator register
     * @param addr  The address of target register
     * @param value Data to be written
     * @param dev   Device ID
     * @return `true` if operation was successful, `false` otherwise
     */
    bool write(const unsigned char addr, const short value,
               const int dev = UMANIPULATORCTL_USE_LAST_DEV)
    {	return  umanipulatorctl_write(_handle, getDev(dev), addr, value) >= 0; }

    /**
     * @brief Write a value in manipulator speed register
     * @param addr  The address of destination speed register
     * @param value Value to be written
     * @param dev   Device ID
     * @return `true` if operation was successful, `false` otherwise
     */
    bool writeSpeed(const unsigned char addr, const short value,
               const int dev = UMANIPULATORCTL_USE_LAST_DEV)
    {	return  umanipulatorctl_write_speed(_handle, getDev(dev), addr, value) >= 0; }

    /**
     * @brief Read data messages from serial port and update the position cache
     * @return `true` if operation was successful and some messages were received,
     * `false` otherwise
     */
    bool update()
    {	return _handle && umanipulatorctl_recv(_handle) > 0; }

    /**
     * @brief Obtain the position of actuators.
     * @param x     Pointer to an allocated buffer for x-actuator position (may be NULL)
     * @param y     Pointer to an allocated buffer for y-actuator position (may be NULL)
     * @param z     Pointer to an allocated buffer for z-actuator position (may be NULL)
     * @param dev   Device ID
     * @param       timeLimit  Timelimit of cache values. If `timeLimit` is 0 then
     * cached positions are used always. If `timeLimit` is #UMANIPULATORCTL_TIMELIMIT_DISABLED
     * then positions are read from manipulator always.
     * @return `true` if operation was successful, `false` otherwise
     */
    bool getPositions(int *x, int *y, int *z,
                      const int dev = UMANIPULATORCTL_USE_LAST_DEV,
                      const unsigned int timeLimit = UMANIPULATORCTL_DEF_REFRESH_TIME)
    {
        bool retval = true;
        if(umanipulatorctl_get_position_ext(_handle, getDev(dev), timeLimit, x, y, z) < 0)
            retval = false;

        return retval;

    }

    /**
     * @brief Store the current position
     * @param dev       Device ID
     * @param storageId The destination memory location
     *        (0 = default, 1 = home, 2 = target, 3 = get values from the manipulator)
     * @return `true` if operation was successful, `false` otherwise
     */
    bool storeMem(const int dev = UMANIPULATORCTL_USE_LAST_DEV, const int storageId = 0)
    {
        return umanipulatorctl_store_mem_current_position_ext(_handle, getDev(dev),
                                                              storageId) >= 0;
    }

    /**
     * @brief Store the given position
     * @param   x           x-coordinate
     * @param   y           y-coordinate
     * @param   z           z-coordinate
     * @param   dev         Device ID
     * @param   storageId   The destination memory location
     *                      (0 = default, 1 = home, 2 = target, 3 = get values from the manipulator)
     * @return `true` if operation was successful, `false` otherwise
     */
    bool storeMem(const int x, const int y, const int z,
                  const int dev = UMANIPULATORCTL_USE_LAST_DEV, const int storageId = 0)
    {
        return umanipulatorctl_store_mem_position_ext(_handle, getDev(dev), x, y, z,
                                                      storageId) >= 0;
    }

    /**
     * @brief Get the stored position data
     * @param[out]  x       Pointer to an allocated buffer for x-actuator position
     * @param[out]  y       Pointer to an allocated buffer for y-actuator position
     * @param[out]  z       Pointer to an allocated buffer for z-actuator position
     * @param       dev     Device ID
     * @param       storageId   The destination memory location
     *              (0 = default, 1 = home, 2 = target, 3 = get values from the manipulator)
     * @return `true` if operation was successful, `false` otherwise
     */
    bool getMem(int *x, int *y, int *z, const int dev = UMANIPULATORCTL_USE_LAST_DEV,
                const int storageId = 0)
    {
        return umanipulatorctl_get_mem_position_ext(_handle, getDev(dev), x, y, z,
                                                    storageId) >= 0;
    }

    /**
     * @brief Move actuators to stored position
     * @param dev           Device ID
     * @param storageId     The destination memory location.
     *        (0 = default, 1 = home, 2 = target, 3 = get values from the manipulator)
     * @return `true` if operation was successful, `false` otherwise
     */
    bool gotoMem(const int dev = UMANIPULATORCTL_USE_LAST_DEV, const int storageId = 0)
    {
        return umanipulatorctl_goto_mem_position_ext(_handle, getDev(dev), storageId) >= 0;
    }

    /**
     * @brief Move actuators to given position
     * @param x     Destination position for x-actuator
     * @param y     Destination position for y-actuator
     * @param z     Destination position for z-actuator
     * @param dev   Device ID
     * @return `true` if operation was successful, `false` otherwise
     */
    bool gotoMem(const int x, const int y, const int z,
                 const int dev = UMANIPULATORCTL_USE_LAST_DEV)
    {
        bool retval = false;
        if (storeMem(x, y, z, dev))
            retval = gotoMem(dev);
        return retval;
    }

    /**
     * @brief Stop all moving actuators
     * @param dev   Device ID
     * @return `true` if operation was successful, `false` otherwise
     */
    bool stop(const int dev = UMANIPULATORCTL_USE_LAST_DEV)
    {   return umanipulatorctl_stop_ext(_handle, getDev(dev)) >= 0; }

    /**
     * @brief Set / unset the control unit in PC Mode
     * @param enable    Enable / release PC Mode. Not equal to zero changes
     *                  the control unit in PC Mode and zero (0) releases
     *                  the control unit from PC Mode.
     * @return `true` if operation was successful, `false` otherwise
     */
    bool set_control_unit_pcmode(const int enable)
    {   return umanipulatorctl_set_control_unit_pcmode(_handle, enable) >= 0; }

    /**
     * @brief Get the latest error code from manipulator
     * @return Error code
     */
    int lastError()
    {	return umanipulatorctl_last_error(_handle); }

    /**
     * @brief Get the latest error description from manipulator
     * @return Pointer to error description
     */
    const char *lastErrorText()
    { 	return umanipulatorctl_last_errorstr(_handle); }

    /**
     * @brief Get the manipulator firmware version
     * @param[out]  major   Pointer to an allocated buffer for firmware major number
     * @param[out]  minor   Pointer to an allocated buffer for firmware minor number
     * @param       dev     Device ID
     * @return `true` if operation was successful, `false` otherwise
     */
    bool readVersion(int *major, int *minor,
                    const int dev = UMANIPULATORCTL_USE_LAST_DEV)
    {
        return umanipulatorctl_read_version_ext(_handle, getDev(dev),
                                                major, minor) > 0;
    }

    /**
     * @brief Get the current mode of manipulator
     * @param   dev     Device ID
     * @return `true` if operation was successful, `false` otherwise
     */
    umanipulatorctl_mode_t getMode(const int dev = UMANIPULATORCTL_USE_LAST_DEV)
    {	return umanipulatorctl_get_mode_ext(_handle, getDev(dev)); }

    /**
     * @brief Set manipulator mode
     * @param mode  New mode
     * @param extendedPenModeStep   Coarse ratio from nm to step count in pen mode.
     *        This parameter has impact only when `mode` is UMANIPULATORCTL_MODE_PEN
     * @param dev   Device ID
     * @return `true` if operation was successful, `false` otherwise
     */
    bool setMode(const umanipulatorctl_mode_t mode,
                                   const int extendedPenModeStep = 0,
                                   const int dev = UMANIPULATORCTL_USE_LAST_DEV)
    {
        return umanipulatorctl_set_mode_ext(_handle, getDev(dev),
                                            mode, extendedPenModeStep) >= 0;
    }

    /**
     * @brief Get the current step length
     * @param dev   Device ID
     * @return `true` if operation was successful, `false` otherwise
     */
    int getStep(const int dev = UMANIPULATORCTL_USE_LAST_DEV)
    {	return umanipulatorctl_get_step_ext(_handle, getDev(dev)); }

    /**
     * @brief Set step length
     * @param value     New step lenght (in nm)
     * @param dev       Device ID
     * @return `true` if operation was successful, `false` otherwise
     */
    bool setStep(const int value, const int dev = UMANIPULATORCTL_USE_LAST_DEV)
    {	return umanipulatorctl_set_step_ext(_handle, getDev(dev),value) >= 0; }

private:
    /**
     * @brief Resolves device ID
     *
     * This privete method resolves the actual device ID used in communication.
     * @param   dev   Device ID in pure integer format
     * @return Device ID converted to ascii-format. Example: `dev' == 1 (0x01) -> '1' (0x31)
     */
    unsigned char getDev(const int dev)
    {
        if(dev == UMANIPULATORCTL_USE_LAST_DEV && _handle)
            return _handle->last_device_sent;
        return dev + 0x30;
    }
    /**
     * @brief Session handle
     */
    umanipulatorctl_state_t *_handle;
};

#endif /* c++ */

#endif /* UMANIPULATORCTL_H */

