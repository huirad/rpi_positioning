/**************************************************************************
* @licence app begin@
*
* SPDX-License-Identifier: MPL-2.0
*
* \ingroup SensorsService
* \author Marco Residori <marco.residori@xse.de>
*
* \copyright Copyright (C) 2013, XS Embedded GmbH
* 
* \license
* This Source Code Form is subject to the terms of the
* Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed with
* this file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* @licence end@
**************************************************************************/

#include "globals.h"
#include "acceleration.h"
#include "sns-meta-data.h"

static pthread_mutex_t mutexCb  = PTHREAD_MUTEX_INITIALIZER;   //protects the callbacks
static pthread_mutex_t mutexData = PTHREAD_MUTEX_INITIALIZER;  //protects the data

static volatile AccelerationCallback cbAcceleration = 0;
static TAccelerationData gAccelerationData;
TAccelerationConfiguration gAccelerationConfiguration;

bool iAccelerationInit()
{
    cbAcceleration = 0;

    //example accelerometer configuration for a 3-axis accelerometer
    gAccelerationConfiguration.dist2RefPointX = 0;
    gAccelerationConfiguration.dist2RefPointY = 0;
    gAccelerationConfiguration.dist2RefPointZ = 0;    
    gAccelerationConfiguration.angleYaw = 0;
    gAccelerationConfiguration.anglePitch = 0;
    gAccelerationConfiguration.angleRoll = 0;
    gAccelerationConfiguration.sigmaX = 0;
    gAccelerationConfiguration.sigmaY = 0;
    gAccelerationConfiguration.sigmaZ = 0;
    gAccelerationConfiguration.typeBits = 
        ACCELERATION_X_PROVIDED |
        ACCELERATION_Y_PROVIDED |
        ACCELERATION_Z_PROVIDED;
    gAccelerationConfiguration.validityBits = 
      ACCELERATION_CONFIG_ANGLEYAW_VALID | 
      ACCELERATION_CONFIG_ANGLEPITCH_VALID |
      ACCELERATION_CONFIG_ANGLEROLL_VALID |
      ACCELERATION_CONFIG_TYPE_VALID;

    return true;
}

bool iAccelerationDestroy()
{
    pthread_mutex_lock(&mutexCb);
    cbAcceleration = 0;
    pthread_mutex_unlock(&mutexCb);

    return true;
}

bool snsAccelerationGetAccelerationData(TAccelerationData * accelerationData)
{
    if(!accelerationData)
    {
    	return false;
    }

    pthread_mutex_lock(&mutexData);
    *accelerationData = gAccelerationData;
    pthread_mutex_unlock(&mutexData);

    return true;
}

bool snsAccelerationRegisterCallback(AccelerationCallback callback)
{
    //printf("snsAccelerationRegisterCallback\n");
    if(cbAcceleration != 0) 
    {
        return false;
    }

    pthread_mutex_lock(&mutexCb);
    cbAcceleration = callback;
    pthread_mutex_unlock(&mutexCb);

    return true;
}

bool snsAccelerationDeregisterCallback(AccelerationCallback callback)
{
    //printf("snsAccelerationDeregisterCallback\n");
    if(cbAcceleration == callback && callback != 0)
    {
        pthread_mutex_lock(&mutexCb);
        cbAcceleration = 0;
        pthread_mutex_unlock(&mutexCb);

        return true;
    }

    return false;
}

bool snsAccelerationGetMetaData(TSensorMetaData *data)
{
    if(data != 0) 
    {
        return false;
    }
    
    pthread_mutex_lock(&mutexData);
    *data = gSensorsMetaData[3];
    pthread_mutex_unlock(&mutexData);

    return true;
}

bool snsAccelerationGetAccelerationConfiguration(TAccelerationConfiguration* config)
{
    *config = gAccelerationConfiguration;
    return true;
}

void updateAccelerationData(const TAccelerationData accelerationData[], uint16_t numElements)
{
    if (accelerationData != NULL && numElements > 0)
    {
        pthread_mutex_lock(&mutexData);
        gAccelerationData = accelerationData[numElements-1];
        pthread_mutex_unlock(&mutexData);
        pthread_mutex_lock(&mutexCb);
        if (cbAcceleration)
        {
            cbAcceleration(accelerationData, numElements);
        }
        pthread_mutex_unlock(&mutexCb);
    }
}

