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

#ifndef INCLUDE_GENIVI_GNSS_LOG
#define INCLUDE_GENIVI_GNSS_LOG

#include "poslog.h"  
#include "gnss.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Provide a system timestamp in milliseconds.
 * @return system timestamp in milliseconds
 */
uint64_t gnsslog_get_timestamp();
    
/**
 * Convert a TGNSSPosition to its string representation
 * @param timestamp Timestamp when the GNSS position data have been received [ms]
 * @param countdown Number of following GNSS position data in same sequence
 * @param position GNSS position data to be converted to its string representation
 * @param str Pointer to string where to put the TGNSSPosition string representation
 * @param size Size of the string
 */
void gnssPosition_to_string(uint64_t timestamp, uint16_t countdown, const TGNSSPosition* position, char *str, size_t size);

/**
 * Convert a TGNSSTime to its string representation
 * @param timestamp Timestamp when the GNSS time data have been received [ms]
 * @param countdown Number of following GNSS time data in same sequence
 * @param time GNSS time data to be converted to its string representation
 * @param str Pointer to string where to put the TGNSSTime string representation
 * @param size Size of the string
 */
void gnssTime_to_string(uint64_t timestamp, uint16_t countdown, const TGNSSTime* time, char *str, size_t size);

/**
 * Convert a TGNSSSatelliteDetail to its string representation
 * @param timestamp Timestamp when the GNSS satellite data have been received [ms]
 * @param countdown Number of following GNSS satellite data in same sequence
 * @param satelliteDetails GNSS satellite data to be converted to its string representation
 * @param str Pointer to string where to put the TGNSSSatelliteDetail string representation
 * @param size Size of the string
 */
void gnssSatelliteDetail_to_string(uint64_t timestamp, uint16_t countdown, const TGNSSSatelliteDetail* satelliteDetails, char *str, size_t size);

/**
 * Log GNSS position data
 * @param timestamp Timestamp when the GNSS position data have been received [ms]
 * @param position pointer to an array of TGNSSPosition with size numElements 
 * @param numElements: number of elements of array position.  
 */
void gnssPosition_log(uint64_t timestamp, const TGNSSPosition position[], uint16_t numElements);

/**
 * Log GNSS time data
 * @param timestamp Timestamp when the GNSS time data have been received [ms]
 * @param time pointer to an array of TGNSSTime with size numElements 
 * @param numElements: number of elements of array time.  
 */
void gnssTime_log(uint64_t timestamp, const TGNSSTime time[], uint16_t numElements);

/**
 * Log GNSS satellite detail data
 * @param timestamp Timestamp when the GNSS satellite detail data have been received [ms]
 * @param satelliteDetail pointer to an array of TGNSSSatelliteDetail with size numElements 
 * @param numElements: number of elements of array satelliteDetail.  
 */
void gnssSatelliteDetail_log(uint64_t timestamp, const TGNSSSatelliteDetail satelliteDetail[], uint16_t numElements);

#ifdef __cplusplus
}
#endif

#endif