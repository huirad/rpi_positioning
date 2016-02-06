/**************************************************************************
 * @brief Access library for I2C
 *
 * @details Encapsulate I2C access to a LSM9DS1 sensor on a Linux machine
 * The LSM9DS1 from ST Microelectronics is a 6DOF inertial sensor
 * @see http://www.st.com/web/en/catalog/sense_power/FM89/SC1448/PF259998
 * Derived from mpu6050.cpp by cloning and then adapting.
 * Duplication of several helper functions is purpose to obtain
 * self contained code with minimum dependencies.
 *
 * @author Helmut Schmidt <https://github.com/huirad>
 * @copyright Copyright (C) 2016, Helmut Schmidt
 *
 * @license MPL-2.0 <http://spdx.org/licenses/MPL-2.0>
 *
 **************************************************************************/


/** ===================================================================
 * 1.) INCLUDES
 */

 //provided interface
#include "i2ccomm.h"

//linux i2c access
#include <linux/i2c-dev.h> //RPi: located in /usr/include/linux/i2c-dev.h - all functions inline

//standard c library functions
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <pthread.h>


/** ===================================================================
 * 3.) PRIVATE VARIABLES AND FUNCTIONS
 * Functions starting with i2c_ encapsulate the I2C bus access
 * See
 *   https://www.kernel.org/doc/Documentation/i2c/dev-interface
 *   https://www.kernel.org/doc/Documentation/i2c/smbus-protocol
 * Functions starting with conv_ convert the raw data to common measurement units
 */

/** Global file descriptor - must be initialized by calling i2c_lsm9ds1_init()
 */
static int _i2c_fd = -1;
/** Global device address - must be initialized by calling i2c_lsm9ds1_init()
 */
static uint8_t _i2c_addr = 0;


/** Write a 8 bit unsigned integer to a register
 */
static bool i2c_write_uint8(uint8_t reg, uint8_t data)
{
    bool result = false;
    __s32 i2c_result;

    if (_i2c_fd < 0)
    {
        /* Invalid file descriptor */
    }
    else
    {
        i2c_result = i2c_smbus_write_byte_data(_i2c_fd, reg, data);
        if (i2c_result < 0)
        {
        /* ERROR HANDLING: i2c transaction failed */
        }
        else
        {
        result = true;
        }
    }
    return result;
}

/** Read a 8 bit unsigned integer from a register
 */
static bool i2c_read_uint8(uint8_t reg, uint8_t* data)
{
    bool result = false;
    __s32 i2c_result;

    if (_i2c_fd < 0)
    {
        /* Invalid file descriptor */
    }
    else
    {
        /* Using SMBus commands */
        i2c_result = i2c_smbus_read_byte_data(_i2c_fd, reg);
        if (i2c_result < 0)
        {
            /* ERROR HANDLING: i2c transaction failed */
        }
        else
        {
            *data = (uint8_t) i2c_result;
            //printf("Register 0x%02X: %08X = %d\n", reg, i2c_result, *data);
            result = true;
        }
    }
    return result;
}

/** Read a block of 8 bit unsigned integers from two consecutive registers
 *  Variant using 1 single ioctl() call instead of 1 read() followed by 1 write()
 *  See https://www.kernel.org/doc/Documentation/i2c/dev-interface
 *  on ioctl(file, I2C_RDWR, struct i2c_rdwr_ioctl_data *msgset).
 *  See also
 *    [i2c_rdwr_ioctl_data] (http://lxr.free-electrons.com/source/include/uapi/linux/i2c-dev.h#L64)
 *    [i2c_msg] (http://lxr.free-electrons.com/source/include/uapi/linux/i2c.h#L68)
 * Seems to be marginally faster than i2c_read_block_1(): Ca 1% when reading 8 bytes
 */
static bool i2c_read_block(uint8_t reg, uint8_t* data, uint8_t size)
{
    bool result = false;
    struct i2c_rdwr_ioctl_data i2c_data;
    struct i2c_msg msg[2];
    int i2c_result;

    if (_i2c_fd < 0)
    {
        /* Invalid file descriptor */
    }
    else
    {
        i2c_data.msgs = msg;
        i2c_data.nmsgs = 2;     // two i2c_msg

        i2c_data.msgs[0].addr = _i2c_addr;
        i2c_data.msgs[0].flags = 0;         // write
        i2c_data.msgs[0].len = 1;           // only one byte
        i2c_data.msgs[0].buf = (char*)&reg; // typecast to char*: see i2c-dev.h

        i2c_data.msgs[1].addr = _i2c_addr;
        i2c_data.msgs[1].flags = I2C_M_RD;  // read command
        i2c_data.msgs[1].len = size;
        i2c_data.msgs[1].buf = (char*)data; // typecast to char*: see i2c-dev.h

        i2c_result = ioctl(_i2c_fd, I2C_RDWR, &i2c_data);

        if (i2c_result < 0)
        {
            /* ERROR HANDLING: i2c transaction failed */
        }
        else
        {
            result = true;
        }
    }
    return result;
}


static bool i2c_lsm9ds1_init(const char* i2c_device, uint8_t i2c_addr)
{
    bool result = true;
    _i2c_fd = open(i2c_device, O_RDWR);
    if (_i2c_fd < 0)
    {
        /* ERROR HANDLING; you can check errno to see what went wrong */
        result = false;
    }
    else
    {
        if (ioctl(_i2c_fd, I2C_SLAVE, i2c_addr) < 0)
        {
            /* ERROR HANDLING; you can check errno to see what went wrong */
            result = false;
        }
        else
        {
            _i2c_addr = i2c_addr;
        }
    }
    return result;
}

static bool i2c_lsm9ds1_deinit()
{
    bool result = false;
    if (_i2c_fd < 0)
    {
        /* Invalid file descriptor */
    }
    else
    {
        close(_i2c_fd);
        _i2c_fd = -1;
        _i2c_addr = 0;
        result = true;
    }
    return result;
}

