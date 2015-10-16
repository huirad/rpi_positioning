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

#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdbool.h>
#include <pthread.h>
#include <time.h>

#include "sns-init.h"
#include "wheel.h"
#include "acceleration.h"
#include "gyroscope.h"
#include "vehicle-speed.h"
#include "sns-meta-data.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const TSensorMetaData gSensorsMetaData[];

void updateAccelerationData(const TAccelerationData accelerationData[], uint16_t numElements);
void updateGyroscopeData(const TGyroscopeData gyroData[], uint16_t numElements);
void updateWheelticks(const TWheelticks ticks[], uint16_t numElements);
void updateVehicleSpeedData(const TVehicleSpeedData vehicleSpeedData[], uint16_t numElements);

#ifdef __cplusplus
}
#endif

#endif /* GLOBALS_H */
