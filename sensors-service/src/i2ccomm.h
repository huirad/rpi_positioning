/**************************************************************************
 * @brief Access library for I2C
 *
 * @details Encapsulate Linux I2C access via the user space device driver
 * @see https://www.kernel.org/doc/Documentation/i2c/dev-interface
 * @see https://www.kernel.org/doc/Documentation/i2c/smbus-protocol
 *
 * @author Helmut Schmidt <https://github.com/huirad>
 * @copyright Copyright (C) 2016, Helmut Schmidt
 *
 * @license MPL-2.0 <http://spdx.org/licenses/MPL-2.0>
 *
 **************************************************************************/

#ifndef INCLUDE_I2CCOMM
#define INCLUDE_I2CCOMM

#include <stdint.h>

class i2ccomm {

private:
    int _i2c_fd;
    uint8_t _i2c_addr;

public:
    i2ccomm(): _i2c_fd (-1), _i2c_addr(0) {};

    bool init(const char* i2c_device, uint8_t i2c_addr);
    bool deinit();

    bool write_uint8(uint8_t reg, uint8_t data);
    bool read_uint8(uint8_t reg, uint8_t* data);
    bool read_block(uint8_t reg, uint8_t* data, uint8_t size);
};

#endif //I2CCOMM