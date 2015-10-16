/**************************************************************************
* @licence app begin@
*
* SPDX-License-Identifier: MPL-2.0
*
* \ingroup SensorsService
* \author Helmut Schmidt <https://github.com/huirad>
*
* \copyright Copyright (C) 2015, Helmut Schmidt 
* 
* \license
* This Source Code Form is subject to the terms of the
* Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed with
* this file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* @licence end@
**************************************************************************/

//APIs provided
#include "sns-init.h"
#include "acceleration.h"
#include "gyroscope.h"

//standard headers
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>

//sns internals
#include "globals.h"
#include "log.h"
#include "mpu6050.h"

DLT_DECLARE_CONTEXT(gContext);


static void mpu6050_cb(const TMPU6050Vector3D acceleration[], const TMPU6050Vector3D gyro_angular_rate[], const float temperature[], const uint64_t timestamp[], const uint16_t num_elements)
{
    //simple implementation - assume num_elements always 1
    TAccelerationData accel = {0};
    TGyroscopeData gyro = {0};
    
    if (num_elements == 1)
    {
        accel.timestamp = timestamp[0];
        accel.x = acceleration[0].x*MPU6050_UNIT_1_G;
        accel.y = acceleration[0].y*MPU6050_UNIT_1_G;
        accel.z = acceleration[0].z*MPU6050_UNIT_1_G;
        accel.temperature = temperature[0];
        accel.validityBits = ACCELERATION_X_VALID | ACCELERATION_Y_VALID | 
                             ACCELERATION_Z_VALID | ACCELERATION_TEMPERATURE_VALID;

        gyro.timestamp = timestamp[0];
        gyro.yawRate = gyro_angular_rate[0].z;
        gyro.pitchRate = gyro_angular_rate[0].y;
        gyro.rollRate = gyro_angular_rate[0].x;
        gyro.temperature = temperature[0];
        gyro.validityBits = GYROSCOPE_YAWRATE_VALID | GYROSCOPE_PITCHRATE_VALID | 
                            GYROSCOPE_ROLLRATE_VALID | GYROSCOPE_TEMPERATURE_VALID;
                             
        updateAccelerationData(&accel, 1);
        updateGyroscopeData(&gyro, 1);
    }
}

bool snsInit()
{

    return true;
}

bool snsDestroy()
{
    return true;
}

void snsGetVersion(int *major, int *minor, int *micro)
{
    if(major)
    {
        *major = GENIVI_SNS_API_MAJOR;
    }

    if(minor)
    {
        *minor = GENIVI_SNS_API_MINOR;
    }

    if(micro)
    {
        *micro = GENIVI_SNS_API_MICRO;
    }
}

bool snsGyroscopeInit()
{
    bool is_ok = iGyroscopeInit();
    if (is_ok)
    {
        is_ok = mpu6050_init(MPU6050_I2C_DEV_1, MPU6050_ADDR_1, MPU6050_DLPF_42HZ); 
    }
    if (is_ok)
    {    
        is_ok = mpu6050_register_callback(&mpu6050_cb);
    }        
    if (is_ok)
    {
        is_ok = mpu6050_start_reader_thread(10, 10, true);
    }
    return is_ok;
}

bool snsGyroscopeDestroy()
{
    bool is_ok = mpu6050_stop_reader_thread();
    is_ok = is_ok && mpu6050_deregister_callback(&mpu6050_cb);
    is_ok = is_ok && mpu6050_deinit();
    is_ok = is_ok && iGyroscopeDestroy();
    return is_ok;    
}

bool snsAccelerationInit()
{
    //gyro is the master for mpu6050 - nothing further to initialize
    return iAccelerationInit();
}

bool snsAccelerationDestroy()
{
    //gyro is the master for mpu6050 - nothing further to deeinit
    return iAccelerationDestroy();
}

//+ some dummy implementations
bool snsVehicleSpeedInit()
{
    return iVehicleSpeedInit();
}

bool snsVehicleSpeedDestroy()
{
    return iVehicleSpeedDestroy();
}

bool snsWheeltickInit()
{
    return iWheeltickInit();
}

bool snsWheeltickDestroy()
{
    return iWheeltickDestroy();
}
