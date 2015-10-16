/**************************************************************************
* Component Name: SensorsService
* Author: Marco Residori <marco.residori@xse.de>
*
* Copyright (C) 2013, XS Embedded GmbH
* 
* License:
* This Source Code Form is subject to the terms of the
* Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed with
* this file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
**************************************************************************/

#include "globals.h"
#include "vehicle-speed.h"
#include "sns-meta-data.h"

static pthread_mutex_t mutexCb  = PTHREAD_MUTEX_INITIALIZER;   //protects the callbacks
static pthread_mutex_t mutexData = PTHREAD_MUTEX_INITIALIZER;  //protects the data

static volatile VehicleSpeedCallback cbVehicleSpeed = 0;
static TVehicleSpeedData gVehicleSpeedData;

bool snsVehicleSpeedInit()
{
    cbVehicleSpeed = 0;

    return true;
}

bool snsVehicleSpeedDestroy()
{
    pthread_mutex_lock(&mutexCb);
    cbVehicleSpeed = 0;
    pthread_mutex_unlock(&mutexCb);

    return true;
}

bool snsVehicleSpeedGetVehicleSpeedData(TVehicleSpeedData* vehicleSpeed)
{
    if(!vehicleSpeed)
    {
    	return false;
    }

    pthread_mutex_lock(&mutexData);
    *vehicleSpeed = gVehicleSpeedData;
    pthread_mutex_unlock(&mutexData);

    return true;
}

bool snsVehicleSpeedRegisterCallback(VehicleSpeedCallback callback)
{
    //printf("snsVehicleSpeedRegisterCallback\n");
    if(cbVehicleSpeed != 0) 
    {
        return false;
    }

    pthread_mutex_lock(&mutexCb);
    cbVehicleSpeed = callback;
    pthread_mutex_unlock(&mutexCb);

    return true;
}

bool snsVehicleSpeedDeregisterCallback(VehicleSpeedCallback callback)
{
    //printf("snsGyroscopeDeregisterCallback\n");
    if(cbVehicleSpeed == callback && callback != 0)
    {
        pthread_mutex_lock(&mutexCb);
        cbVehicleSpeed = 0;
        pthread_mutex_unlock(&mutexCb);

        return true;
    }

    return false;
}

bool snsVehicleSpeedGetMetaData(TSensorMetaData *data)
{
    if(data != 0) 
    {
        return false;
    }
    
    pthread_mutex_lock(&mutexData);
    *data = gSensorsMetaData[2];
    pthread_mutex_unlock(&mutexData);

    return true;
}

void updateVehicleSpeedData(const TVehicleSpeedData vehicleSpeedData[], uint16_t numElements)
{
    if (vehicleSpeedData != NULL && numElements > 0)
    {
        pthread_mutex_lock(&mutexData);
        gVehicleSpeedData = vehicleSpeedData[numElements-1];
        pthread_mutex_unlock(&mutexData);
        pthread_mutex_lock(&mutexCb);
        if (cbVehicleSpeed)
        {
            cbVehicleSpeed(vehicleSpeedData, numElements);
        }
        pthread_mutex_unlock(&mutexCb);
    }
}
