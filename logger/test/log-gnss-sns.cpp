/**************************************************************************
* @licence app begin@
*
* SPDX-License-Identifier: MPL-2.0
*
* \brief Test program for GNSS+SNS logging
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
#include "snslog.h"
#if (DLT_ENABLED)
#include "dlt.h"
#endif

#include <syslog.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include <inttypes.h>

#include "gnss-init.h"
#include "gnss.h"
#include "gnss-status.h"
#include "sns-init.h"



/**
 * Double buffer for log strings
 * Purpose: avoid blockingt of callback functions by I/O
 * Multiple writer threads are allowed but only on reader thread
 */
class DBuf {

#define DBUF_STRING_SIZE 256
#define DBUF_NUM_LINES 512

    struct SBuf {
    public:      
        char strings [DBUF_STRING_SIZE] [DBUF_NUM_LINES];
        uint16_t rnext;
        uint16_t wnext;
        SBuf(): rnext(0), wnext(0) {};
    };

    SBuf* wbuf;
    SBuf* rbuf;
    pthread_mutex_t wmutex;
    pthread_mutex_t rmutex;
public:
    
    DBuf()
    {
        pthread_mutex_init(&wmutex, NULL);
        pthread_mutex_init(&rmutex, NULL);
        wbuf = new(SBuf);
        rbuf = new(SBuf);
    }

    /**
     * Add a string to the buffer.
     * Return true on success, false on failure (e.g. buffer full).
     */
    bool write(const char* s)
    {
        bool retval = false;
        pthread_mutex_lock(&wmutex);
        if (s && wbuf && (wbuf->wnext < DBUF_NUM_LINES))
        {
            strncpy(wbuf->strings[wbuf->wnext], s, DBUF_STRING_SIZE-1);
            wbuf->strings[wbuf->wnext][DBUF_STRING_SIZE-1] = 0;
            wbuf->wnext++;
            retval = true;
        }
        pthread_mutex_unlock(&wmutex);
        return retval;
    }
    
    /**
     * Read next string from the buffer
     * Return NULL, if no more string available
     */
    const char* read()
    {
        const char* ret = NULL;
        pthread_mutex_lock(&rmutex);
        if (rbuf && (rbuf->rnext < rbuf->wnext) && (rbuf->rnext < DBUF_NUM_LINES) )
        {
            ret = rbuf->strings[rbuf->rnext];
            rbuf->rnext++;
        }
        pthread_mutex_unlock(&rmutex);
        return ret;
    }
    
    /**
     * Swap read and write buffers. 
     * Clears read buffer before to ensure that new write buffer is empty.
     * Shall only be called by reader thread when read buffer has been 
     * completely processed.
     */
    void swap()
    {
        SBuf* tmp;
        pthread_mutex_lock(&rmutex);
        rbuf->rnext = 0;
        rbuf->wnext = 0;
        pthread_mutex_lock(&wmutex);
        tmp = wbuf;
        wbuf = rbuf;
        rbuf = tmp;
        pthread_mutex_unlock(&wmutex);
        pthread_mutex_unlock(&rmutex);        
    }
    
};



#define GNSS_INIT_MAX_RETRIES 30

//global variable to control the main loop - will be set by signal handlers or status callback
static volatile bool sigint = false;
static volatile bool sigterm = false;
static volatile bool gnss_failure = false;
//global pointer to the double buffer
DBuf* dbuf = 0;
pthread_t g_logthread;

/**
 * Logger callback to add string to the double buffer.
 */
void dbufCb(const char* string)
{
    if (dbuf)
    {
        //TODO handle buffer full
        dbuf->write(string);
        printf("WBUF %s\n", string);
    }
}

/**
 * Background thread to write double buffer to a file.
 * 
 */
void* loop_log_writer(void*)
{
    //TODO Open file (and close at thread end)
    //TODO exit condition: on sigint or gnss_failure write rest of buffer
    printf("LOGWRITER\n");
    while (dbuf && !sigint && !sigterm)
    {
        const char* s = 0;
        //process read buffer - should alway be empty
        while (s = dbuf->read())
        {
            printf("BUF1 %s\n", s);
        }
        //swap and process previous write buffer
        dbuf->swap();
        while (s = dbuf->read())
        {
            printf("BUF2 %s\n", s);
        }
        printf("LOGWRITER SLEEP\n");
        sleep(5);
        printf("LOGWRITER WAKEUP\n");
    }
    printf("LOGWRITER END\n");
}

static void sigHandler (int sig, siginfo_t *siginfo, void *context)
{
    if (sig == SIGINT) 
    {
        sigint = true;
    }
    else 
    if (sig == SIGTERM) 
    {
        sigterm = true;
    }
}

static bool registerSigHandlers()
{
    bool is_success = true;
    
    struct sigaction action;
    memset (&action, '\0', sizeof(action));
    action.sa_sigaction = &sigHandler;
    action.sa_flags = SA_SIGINFO;

    if (sigaction(SIGINT, &action, NULL) < 0) 
    {
        is_success = false;
    }
    if (sigaction(SIGTERM, &action, NULL) < 0) 
    {
        is_success = false;
    }
    return is_success;
}



static void cbTime(const TGNSSTime time[], uint16_t numElements)
{
    gnssTimeLog(gnsslogGetTimestamp(), time, numElements);
}

static void cbPosition(const TGNSSPosition position[], uint16_t numElements)
{
    gnssPositionLog(gnsslogGetTimestamp(), position, numElements);
}

static void cbGNSSStatus(const TGNSSStatus *status)
{
    if (status && (status->validityBits & GNSS_STATUS_STATUS_VALID))
    {
        char status_string[64];
        sprintf(status_string, "#GNSS Status: %d", status->status);
        poslogAddString(status_string);
        if (status->status == GNSS_STATUS_FAILURE)
        {
            gnss_failure = true;
        }
    }
}

static void cbAccel(const TAccelerationData accelerationData[], uint16_t numElements)
{
    accelerationDataLog(snslogGetTimestamp(), accelerationData, numElements);
}

static void cbGyro(const TGyroscopeData gyroData[], uint16_t numElements)
{
    gyroscopeDataLog(snslogGetTimestamp(), gyroData, numElements);
}


int main (int argc, char *argv[])
{
    int major;
    int minor;
    int micro;
    char version_string[64];
    
    bool is_poslog_init_ok = false;
    bool is_sns_init_ok = false;    
    bool is_sns_gyro_init_ok = false;
    bool is_sns_accel_init_ok = false;
    bool is_gnss_init_ok = false;
    int gnss_init_tries = 0;
    
    registerSigHandlers();
    
#if (DLT_ENABLED)
    DLT_REGISTER_APP("GLT","GNSS/SNS Logger");
#endif
    poslogSetFD(STDOUT_FILENO);
    is_poslog_init_ok = poslogInit();
    
    if(argv[1])
    {
        dbuf = new(DBuf);
        poslogSetCB(dbufCb);
        pthread_create(&g_logthread, NULL, loop_log_writer, NULL);
    }
    
    if (is_poslog_init_ok)
    {
        //poslogSetActiveSinks(POSLOG_SINK_DLT|POSLOG_SINK_FD|POSLOG_SINK_CB);
        poslogSetActiveSinks(POSLOG_SINK_DLT|POSLOG_SINK_CB);

        gnssGetVersion(&major, &minor, &micro);
        sprintf(version_string, "0,0$GVGNSVER,%d,%d,%d", major, minor, micro);
        poslogAddString(version_string);
        snsGetVersion(&major, &minor, &micro);
        sprintf(version_string, "0,0$GVSNSVER,%d,%d,%d", major, minor, micro);
        poslogAddString(version_string);

        is_sns_init_ok = snsInit();
        if (is_sns_init_ok)
        {
            is_sns_gyro_init_ok = snsGyroscopeInit();
            if(is_sns_gyro_init_ok)
            {
                poslogAddString("#INF snsGyroscopeInit() success");
                snsGyroscopeRegisterCallback(&cbGyro);
            }
            is_sns_accel_init_ok = snsAccelerationInit();
            if (is_sns_accel_init_ok)
            {
                poslogAddString("#INF snsAccelerationInit() success");
                snsAccelerationRegisterCallback(&cbAccel);
            }
            if (!is_sns_gyro_init_ok && !is_sns_accel_init_ok)
            {
                is_sns_init_ok = false;
                snsDestroy();
            }
            else
            {
                poslogAddString("#INF snsInit() success");
            }
        }

        //GNSS device may be available a bit late after startup
        is_gnss_init_ok = gnssInit();
        while (!is_gnss_init_ok && (gnss_init_tries < GNSS_INIT_MAX_RETRIES) && !sigint && !sigterm)
        {
            sleep(1);
            is_gnss_init_ok = gnssInit();
            gnss_init_tries += 1;
        }
        if (is_gnss_init_ok)
        {
            poslogAddString("#INF gnssInit() success");
            gnssRegisterTimeCallback(&cbTime);
            gnssRegisterPositionCallback(&cbPosition);
            gnssRegisterStatusCallback(&cbGNSSStatus);
        }

        if (is_sns_init_ok || is_gnss_init_ok)
        {
            while(!sigint && !sigterm && !gnss_failure)
            {
                sleep(1);
            }
        }
        else
        {
            poslogAddString("#ERR snsInit() or gnssInit() failure - terminating");
        }
        
        //if not interrupted by SIGTERM then we have time to cleanup
        if (!sigterm) 
        {
            if (sigint)
            {
                poslogAddString("#SIGINT");
            }
            if (gnss_failure)
            {
                poslogAddString("#GNSS_STATUS_FAILURE");
            }
            if (is_sns_init_ok)
            {
                if (is_sns_accel_init_ok)
                {
                    snsAccelerationDeregisterCallback(&cbAccel);
                    snsAccelerationDestroy();
                }
                if (is_sns_gyro_init_ok)
                {
                    snsGyroscopeRegisterCallback(&cbGyro);
                    snsGyroscopeDestroy();
                }
                snsDestroy();
            }
            if (is_gnss_init_ok)
            {
                gnssDeregisterStatusCallback(&cbGNSSStatus);
                gnssDeregisterPositionCallback(&cbPosition);
                gnssDeregisterTimeCallback(&cbTime);
                gnssDestroy();
            }
        }
        poslogDestroy();
    }
    //TODO Wait until logger thread is terminated
#if (DLT_ENABLED)
    DLT_UNREGISTER_APP();
#endif
}
