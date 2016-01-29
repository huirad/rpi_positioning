/**************************************************************************
 * @brief Access library for LSM9DS1 inertial sensor 
 *
 * @details Encapsulate I2C access to a LSM9DS1 sensor on a Linux machine
 * The LSM9DS1 from ST Microelectronics is a 6DOF inertial sensor
 * @see http://www.st.com/web/en/catalog/sense_power/FM89/SC1448/PF259998 
 *
 *
 * @author Helmut Schmidt <https://github.com/huirad>
 * @copyright Copyright (C) 2016, Helmut Schmidt
 * 
 * @license MPL-2.0 <http://spdx.org/licenses/MPL-2.0>
 *
 **************************************************************************/

#ifndef INCLUDE_LSM9DS1
#define INCLUDE_LSM9DS1

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
 
 
/** Part 0: I2C device names and device addresses
 *
 */

/** Typical I2C device names on Linux machines
  * For early Raspberry Pi machines, its typically "/dev/i2c-0"
  * For most later (starting already Q3/2012) Raspberry Pi machines, its typically "/dev/i2c-1"
  */
#define LSM9DS1_I2C_DEV_0 "/dev/i2c-0"
#define LSM9DS1_I2C_DEV_1 "/dev/i2c-1"
#define LSM9DS1_I2C_DEV_2 "/dev/i2c-2"
#define LSM9DS1_I2C_DEV_3 "/dev/i2c-3"
#define LSM9DS1_I2C_DEV_DEFAULT LSM9DS1_I2C_DEV_1

/** Possible I2C device addresses of the LSM9DS1
  * 0x68 is the default address
  */
#define LSM9DS1_ADDR_1 0x68
#define LSM9DS1_ADDR_2 0x69
 
/** Digital low pass filter settings
  * See the LSM9DS1 data sheet for details
  * http://www.st.com/web/en/resource/technical/document/datasheet/DM00103319.pdf
  */ 
enum ELSM9DS1LowPassFilterBandwidth
{
    LSM9DS1_DLPF_256HZ = 0x00,    //Gyro: 256Hz,  Accel: 260Hz
    LSM9DS1_DLPF_188HZ = 0x01,    //Gyro: 188Hz,  Accel: 184Hz
    LSM9DS1_DLPF_98HZ  = 0x02,    //Gyro:  98Hz,  Accel:  94Hz
    LSM9DS1_DLPF_42HZ  = 0x03,    //Gyro:  42Hz,  Accel:  44Hz
    LSM9DS1_DLPF_20HZ  = 0x04,    //Gyro:  20Hz,  Accel:  21Hz
    LSM9DS1_DLPF_10HZ  = 0x05,    //Gyro:  10Hz,  Accel:  10Hz
    LSM9DS1_DLPF_5HZ   = 0x06     //Gyro:   5Hz,  Accel:   5Hz
};

 
/** Part 1: Functions to access the LSM9DS1
 *
 */
  
/**
 * Container for quantities which can be expressed as 3 dimensional vector.
 * The axis system is to be assumed cartesian and right-handed
 * Measurement units depend on type of data:
 *   For accelerometer measurements, the unit is g
 *   For gyro measurements, the unit is degrees per seconds (deg/s)
 */
typedef struct 
{
  float x;        /**< Vector component of the quantity along the x-axis */
  float y;        /**< Vector component of the quantity along the y-axis */
  float z;        /**< Vector component of the quantity along the Z-axis */
} TLSM9DS1Vector3D;
  
/**
 * Initialize the LSM9DS1 access.
 * Must be called before using any of the other functions.
 * @param i2c_device the name of the i2c device on which the LSM9DS1 is attached
 * @param i2c_addr the I2C address of the LSM9DS1
 * @param bandwidth bandwidth of the digital low pass filter
 * @return true on success.
 */
bool lsm9ds1_init(const char* i2c_device = LSM9DS1_I2C_DEV_DEFAULT, uint8_t i2c_addr=LSM9DS1_ADDR_1, ELSM9DS1LowPassFilterBandwidth bandwidth=LSM9DS1_DLPF_256HZ); 

/**
 * Release the LSM9DS1 access.
 * @return true on success.
 */
bool lsm9ds1_deinit(); 

/**
 * Read the current acceleration, angular rate and temperature from the LSM9DS1
 * Any pointer may be NULL to indicate that the corresponding data is not requested
 * @param acceleration returns the acceleration vector. Measurement unit is g [1g = 9.80665 m/(s*s)]
 * @param gyro_angular_rate returns the angular rate vector as measured by gyroscope . Measurement unit is deg/s [degrees per seconds]
 * @param temperature returns the temperature in degrees celsius
 * @param timestamp returns a system timestamp in ms (milliseconds) derived from clock_gettime(CLOCK_MONOTONIC);
 * @return true on success.
 * @note: Never call this function while the callback mechanism is running!
 */
bool lsm9ds1_read_accel_gyro(TLSM9DS1Vector3D* acceleration, TLSM9DS1Vector3D* gyro_angular_rate, float* temperature, uint64_t* timestamp);

/** Part 2: Provide data via callback mechanism
 *
 */

/**
 * Callback type for LSM9DS1 data.
 * Use this type of callback if you want to register for LSM9DS1 data.
 * This callback may return buffered data (num_elements >1) depending on configuration
 * If the arrays contain buffered data (num_elements >1), the elements will be ordered with rising timestamps
 * Data in different arrays with the same index belong together
 * @param acceleration pointer to an array of TLSM9DS1Vector3D with size num_elements containing acceleration data
 * @param gyro_angular_rate pointer to an array of TLSM9DS1Vector3D with size num_elements containing angular rate data
 * @param temperature pointer to an array of float with size num_elements containing temperature data
 * @param timestamp pointer to an array of uint64_t with size num_elements containing timestamps
 * @param num_elements: allowed range: >=1. If num_elements >1, buffered data are provided. 
 */
typedef void (*LSM9DS1Callback)(const TLSM9DS1Vector3D acceleration[], const TLSM9DS1Vector3D gyro_angular_rate[], const float temperature[], const uint64_t timestamp[], const uint16_t num_elements);

/**
 * Register LSM9DS1 callback.
 * This is the recommended method for continuously accessing the LSM9DS1 data
 * The callback will be invoked when new LSM9DS1 data is available and the reader thread is running.
 * To start the reader thred, call mpu6050_start_reader_thread().
 * @param callback The callback which should be registered.
 * @return True if callback has been registered successfully.
 */
bool lsm9ds1_register_callback(LSM9DS1Callback callback);

/**
 * Deregister LSM9DS1 callback.
 * After calling this method no new LSM9DS1 data will be delivered to the client.
 * @param callback The callback which should be deregistered.
 * @return True if callback has been deregistered successfully.
 */
bool lsm9ds1_deregister_callback(LSM9DS1Callback callback);

/**
 * Start the LSM9DS1 reader thread.
 * This thread will call the callback function registered by mpu6050_register_callback()
 * The thread may be started before of after registering the callback function
 * @param sample_interval Interval in ms (milliseconds) at which LSM9DS1 data shall be read
 * @param num_samples Number of samples to read for one call of the callback function
 * @param average If true, the only the average (mean value) of the num_samples will be returned
 * @return True on success.
 * @note Be sure to select a meaningful combination of sample_interval and igital low pass filter bandwidth
 */
bool lsm9ds1_start_reader_thread(uint64_t sample_interval, uint16_t num_samples, bool average);

/**
 * Stop the LSM9DS1 reader thread.
 * @return True on success.
 */
bool lsm9ds1_stop_reader_thread();

/** Part 3: Utility functions and conversion factors
 *
 */

 /** Unit conversion factors
  *
  * LSM9DS1_UNIT_1_G: Standard gravity: 1g = 9.80665 m/(s*s)
  * @see http://en.wikipedia.org/wiki/Standard_gravity
  * @see http://www.bipm.org/utils/common/pdf/si_brochure_8_en.pdf#page=51
  * @see http://en.wikipedia.org/wiki/Gravitational_acceleration 
  *
  * LSM9DS1_UNIT_1_RAD_IN_DEG: Angle conversion from radian to degrees
  */
#ifndef M_PI
//No more available in C-99
#define LSM9DS1_PI (4.0*atan(1.0))
//#define M_PI 3.141592653589793238462643
#else
#define LSM9DS1_PI M_PI
#endif  
#define LSM9DS1_UNIT_1_G 9.80665
#define LSM9DS1_UNIT_1_RAD_IN_DEG (180.0/MPU6050_PI)

/**
 * Get system timestamp
 * @return returns a system timestamp in ms (milliseconds) derived from clock_gettime(CLOCK_MONOTONIC);
 */
uint64_t lsm9ds1_get_timestamp(); 


#ifdef __cplusplus
}
#endif

#endif //INCLUDE_LSM9DS1
