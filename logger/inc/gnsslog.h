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


#ifdef __cplusplus
extern "C" {
#endif

/**
 * Convert a TGNSSPosition structure to a log string.
 * @note: The log string will *not* contain a line break (\n) at the end.
 * @param timestamp Timestamp for the current time in ms to be added to the header of the log string.
 *   This timestamp shall be based on the same time source as the timestamp within the position parameter.
 * @param countdown Countdown value to be added to the header of the log string.
 * @param position TGNSSPosition structure to be converted to the log string
 * @param str Pointer to a string variable where the log string will be written to.
 * @param size Size of the string variable where the log string will be written to.
 */
void gnssPosition_to_string(uint64_t timestamp, uint16_t countdown, const TGNSSPosition* position, char *str, size_t size);

/**
 * Convert a TGNSSTime structure to a log string.
 * @note: The log string will *not* contain a line break (\n) at the end.
 * @param timestamp Timestamp for the current time in ms to be added to the header of the log string.
 *   This timestamp shall be based on the same time source as the timestamp within the time parameter.
 * @param countdown Countdown value to be added to the header of the log string.
 * @param position TGNSSTime structure to be converted to the log string
 * @param str Pointer to a string variable where the log string will be written to.
 * @param size Size of the string variable where the log string will be written to.
 */
void gnssTime_to_string(uint64_t timestamp, uint16_t countdown, const TGNSSTime* time, char *str, size_t size);

/**
 * Convert a TGNSSSatelliteDetail structure to a log string.
 * @note: The log string will *not* contain a line break (\n) at the end.
 * @param timestamp Timestamp for the current time in ms to be added to the header of the log string.
 *   This timestamp shall be based on the same time source as the timestamp within the satellite detail parameter.
 * @param countdown Countdown value to be added to the header of the log string.
 * @param position TGNSSSatelliteDetail structure to be converted to the log string
 * @param str Pointer to a string variable where the log string will be written to.
 * @param size Size of the string variable where the log string will be written to.
 */
void gnssSatelliteDetail_to_string(uint64_t timestamp, uint16_t countdown, const TGNSSSatelliteDetail* satelliteDetails, char *str, size_t size);


/**
 * Write GNSS position data to the position log.
 * 
 * @param position pointer to an array of TGNSSPosition with size numElements 
 * @param numElements number of TGNSSPosition elements in array position
 */
void gnssPosition_log(uint64_t timestamp, const TGNSSPosition position[], uint16_t numElements);

/**
 * Write GNSS time data to the position log.
 * 
 * @param position pointer to an array of TGNSSTime with size numElements 
 * @param numElements number of TGNSSTime elements in array position
 */
void gnssTime_log(uint64_t timestamp, const TGNSSTime time[], uint16_t numElements);

/**
 * Write GNSS satelliet detail data to the position log.
 * 
 * @param position pointer to an array of TGNSSSatelliteDetail with size numElements 
 * @param numElements number of TGNSSSatelliteDetail elements in array position
 */
void gnssSatelliteDetail_log(uint64_t timestamp, const TGNSSSatelliteDetail satelliteDetail[], uint16_t numElements);

#ifdef __cplusplus
}
#endif

#endif
