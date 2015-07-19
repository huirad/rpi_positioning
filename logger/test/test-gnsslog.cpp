/**************************************************************************
* @licence app begin@
*
* SPDX-License-Identifier: MPL-2.0
*
* \brief Test program for GNSS logging
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

#include "poslog.h"
#include "gnsslog.h"
#if (DLT_ENABLED)
#include "dlt.h"
#endif
#include <syslog.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>

#include "gnss-init.h"
#include "gnss.h"


static void cbTime(const TGNSSTime time[], uint16_t numElements)
{
    gnssTime_log(gnsslog_get_timestamp(), time, numElements);
}

static void cbPosition(const TGNSSPosition position[], uint16_t numElements)
{
    gnssPosition_log(gnsslog_get_timestamp(), position, numElements);
}

int main()
{
    int major;
    int minor;
    int micro;
    char version_string[64];
    
#if (DLT_ENABLED)  
    DLT_REGISTER_APP("GLT","Test Application for GNSS Logging");
#endif
    poslogSetFD(STDOUT_FILENO);
    poslogInit();
    poslogSetActiveSinks(POSLOG_SINK_DLT|POSLOG_SINK_SYSLOG|POSLOG_SINK_FD|POSLOG_SINK_CB);
        
    gnssGetVersion(&major, &minor, &micro);
    sprintf(version_string, "0,0$GVGNSVER,%d,%d,%d", major, minor, micro);
    poslogAddString(version_string);

    gnssInit();
    gnssRegisterTimeCallback(&cbTime);    
    gnssRegisterPositionCallback(&cbPosition);    

    sleep(10);
    
    gnssDeregisterPositionCallback(&cbPosition);    
    gnssDeregisterTimeCallback(&cbTime);        
    gnssDestroy();
    poslogDestroy();
}