/**************************************************************************
* @licence app begin@
*
* SPDX-License-Identifier: MPL-2.0
*
* \brief Utility functions to create and log GNSS specific log strings
*        
*
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

#include "snslog.h" 
#include "poslog.h"  

#include "gnss.h"

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>



#define LOG_STRING_SIZE 256

uint64_t snslog_get_timestamp()
{
    struct timespec time_value;
    if (clock_gettime(CLOCK_MONOTONIC, &time_value) != -1)
    {
        return (time_value.tv_sec*1000 + time_value.tv_nsec/1000000);
    }
    else
    {
        return 0xFFFFFFFFFFFFFFFF;
  }
}

void accelerationData_to_string(uint64_t timestamp, uint16_t countdown, const TAccelerationData* accelerationData, char *str, size_t size)
{
    if ((str) && (size > 0))
    {
        snprintf(
        str,
        size-1, //ensure that there is space for null-terminator
        "%lu,%hu$GVSNSACC,%lu,%7.4f,%7.4f,%7.4f,%5.1f,0X%08X",
        timestamp,
        countdown,
        accelerationData->timestamp,
        accelerationData->x,
        accelerationData->y,
        accelerationData->z,
        accelerationData->temperature,
        accelerationData->validityBits
        );
        str[size-1] = 0; //ensure that string is null-terminated
    }
}

void accelerationData_log(uint64_t timestamp, const TAccelerationData accelerationData[], uint16_t numElements)
{
    char logstring[LOG_STRING_SIZE] ;
    for (int i=0; i<numElements; i++)
    {
        TPoslogSeq seq = POSLOG_SEQ_CONT;
        if (i==0) seq|=POSLOG_SEQ_START;
        if (i==(numElements-1)) seq|=POSLOG_SEQ_STOP;
        accelerationData_to_string(timestamp, numElements-i-1, &accelerationData[i], logstring, LOG_STRING_SIZE);
        poslogAddString(logstring, seq);
    }
}

void gyroscopeData_to_string(uint64_t timestamp, uint16_t countdown, const TGyroscopeData* gyroData, char *str, size_t size)
{
    if ((str) && (size > 0))
    {
        snprintf(
        str,
        size-1, //ensure that there is space for null-terminator
        "%lu,%hu$GVSNSGYRO,%lu,%6.2f,%6.2f,%6.2f,%5.1f,0X%08X",
        timestamp,
        countdown,
        gyroData->timestamp,
        gyroData->yawRate,
        gyroData->pitchRate,
        gyroData->rollRate,
        gyroData->temperature,
        gyroData->validityBits
        );
        str[size-1] = 0; //ensure that string is null-terminated
    }
}

void gyroscopeData_log(uint64_t timestamp, const TGyroscopeData gyroData[], uint16_t numElements)
{
    char logstring[LOG_STRING_SIZE] ;
    for (int i=0; i<numElements; i++)
    {
        TPoslogSeq seq = POSLOG_SEQ_CONT;
        if (i==0) seq|=POSLOG_SEQ_START;
        if (i==(numElements-1)) seq|=POSLOG_SEQ_STOP;
        gyroscopeData_to_string(timestamp, numElements-i-1, &gyroData[i], logstring, LOG_STRING_SIZE);
        poslogAddString(logstring, seq);
    }
}
