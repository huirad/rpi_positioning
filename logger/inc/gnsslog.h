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

int snprintf(char *str, size_t size, const char *format, ...);
  
int gnssTime_to_string(const TGNSSTime* time, char *str, size_t size);
int gnssSatelliteDetail_to_string(const TGNSSSatelliteDetail* satelliteDetails, char *str, size_t size);
int gnssPosition_to_string(const TGNSSPosition* position, char *str, size_t size);

void gnssTime_log(uint64_t timestamp, const TGNSSTime time[], uint16_t numElements);
void gnssSatelliteDetail_log(uint64_t timestamp, const TGNSSSatelliteDetail satelliteDetail[], uint16_t numElements);
void gnssPosition_log(uint64_t timestamp, const TGNSSPosition position[], uint16_t numElements);

CHECK!!! SUPPORTfor buffered data !!!!!!!!!!!!!
- when/where to add external t

#ifdef __cplusplus
}
#endif

#endif
