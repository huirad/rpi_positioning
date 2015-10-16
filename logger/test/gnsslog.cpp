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

#include "gnsslog.h" 
#include "poslog.h"  

#include "gnss.h"

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>



#define LOG_STRING_SIZE 256

uint64_t gnsslog_get_timestamp()
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

void gnssPosition_to_string(uint64_t timestamp, uint16_t countdown, const TGNSSPosition* position, char *str, size_t size)
{
    if ((str) && (size > 0))
    {
        snprintf(
        str,
        size-1, //ensure that there is space for null-terminator
        "%lu,%hu$GVGNSPOS,%lu,%9.6f,%9.6f,%6.1f,%6.1f,%4.1f,%4.1f,%6.2f,%3.1f,%3.1f,%3.1f,%02hu,%02hu,%02hu,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%u,0X%08X,0X%08X,0X%08X,0X%08X",
        timestamp,
        countdown,
        position->timestamp,
        position->latitude,
        position->longitude,
        position->altitudeMSL,
        position->altitudeEll,
        position->hSpeed,
        position->vSpeed,
        position->heading,
        position->pdop,
        position->hdop,
        position->vdop,
        position->usedSatellites,
        position->trackedSatellites,
        position->visibleSatellites,
        position->sigmaHPosition,
        position->sigmaAltitude,
        position->sigmaHSpeed,
        position->sigmaVSpeed,
        position->sigmaHeading,
        position->fixStatus,
        position->fixTypeBits,
        position->activated_systems,
        position->used_systems,
        position->validityBits
        );
        str[size-1] = 0; //ensure that string is null-terminated
    }
}

void gnssTime_to_string(uint64_t timestamp, uint16_t countdown, const TGNSSTime* time, char *str, size_t size)
{
    if ((str) && (size > 0))
    {
        snprintf(
        str,
        size-1, //ensure that there is space for null-terminator
        "%lu,%hu$GVGNSTIM,%lu,%04u,%02u,%02u,%02u,%02u,%02u,%03u,0X%08X",
        timestamp,
        countdown,
        time->timestamp,
        time->year,
        time->month,
        time->day,
        time->hour,
        time->minute,
        time->second,
        time->ms,        
        time->validityBits
        );
        str[size-1] = 0; //ensure that string is null-terminated
    }
}

void gnssSatelliteDetail_to_string(uint64_t timestamp, uint16_t countdown, const TGNSSSatelliteDetail* satelliteDetails, char *str, size_t size)
{
    if ((str) && (size > 0))
    {
        snprintf(
        str,
        size-1, //ensure that there is space for null-terminator
        "%lu,%hu$GVGNSSAT,%lu,%u,%hu,%hu,%hu,%hu,0X%08X,%hu,0X%08X",
        timestamp,
        countdown,
        satelliteDetails->timestamp,
        satelliteDetails->system,
        satelliteDetails->satelliteId,
        satelliteDetails->azimuth,
        satelliteDetails->elevation,
        satelliteDetails->SNR,
        satelliteDetails->statusBits,
        satelliteDetails->posResidual,
        satelliteDetails->validityBits
        );
        str[size-1] = 0; //ensure that string is null-terminated
    }
}

void gnssPosition_log(uint64_t timestamp, const TGNSSPosition position[], uint16_t numElements)
{
    char logstring[LOG_STRING_SIZE] ;
    for (int i=0; i<numElements; i++)
    {
        TPoslogSeq seq = POSLOG_SEQ_CONT;
        if (i==0) seq|=POSLOG_SEQ_START;
        if (i==(numElements-1)) seq|=POSLOG_SEQ_STOP;
        gnssPosition_to_string(timestamp, numElements-i-1, &position[i], logstring, LOG_STRING_SIZE);
        poslogAddString(logstring, seq);
    }
}

void gnssTime_log(uint64_t timestamp, const TGNSSTime time[], uint16_t numElements)
{
    char logstring[LOG_STRING_SIZE] ;
    for (int i=0; i<numElements; i++)
    {
        TPoslogSeq seq = POSLOG_SEQ_CONT;
        if (i==0) seq|=POSLOG_SEQ_START;
        if (i==(numElements-1)) seq|=POSLOG_SEQ_STOP;
        gnssTime_to_string(timestamp, numElements-i-1, &time[i], logstring, LOG_STRING_SIZE);
        poslogAddString(logstring, seq);
    }    
}

void gnssSatelliteDetail_log(uint64_t timestamp, const TGNSSSatelliteDetail satelliteDetail[], uint16_t numElements)
{
    char logstring[LOG_STRING_SIZE] ;
    for (int i=0; i<numElements; i++)
    {
        TPoslogSeq seq = POSLOG_SEQ_CONT;
        if (i==0) seq|=POSLOG_SEQ_START;
        if (i==(numElements-1)) seq|=POSLOG_SEQ_STOP;
        gnssSatelliteDetail_to_string(timestamp, numElements-i-1, &satelliteDetail[i], logstring, LOG_STRING_SIZE);
        poslogAddString(logstring, seq);
    }  
}
