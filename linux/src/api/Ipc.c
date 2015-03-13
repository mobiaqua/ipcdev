/*
 * Copyright (c) 2012-2015 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 *  @file       Ipc.c
 *
 *  @brief      Starts and stops user side Ipc
 *              All setup/destroy APIs on user side will be call from this
 *              module.
 */

/* standard headers */
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

/* package headers */
#include <ti/ipc/Std.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/NameServer.h>

/* User side headers */
#include <ladclient.h>

/* IPC startup/shutdown stuff: */
#include <ti/ipc/MultiProc.h>
#include <GateHWSpinlock.h>
#include <_GateMP.h>
#include <_MultiProc.h>
#include <_MessageQ.h>
#include <_NameServer.h>

/* module definition */
typedef struct {
    Int                         refCount;
    pthread_mutex_t             gate;
    Ipc_TransportFactoryFxns   *transportFactory;
} Ipc_Module;


/* =============================================================================
 *  Globals
 * =============================================================================
 */
static Ipc_Module Ipc_module = {
    .refCount           = 0,
    .gate               = PTHREAD_MUTEX_INITIALIZER,
    .transportFactory   = NULL
};

GateHWSpinlock_Config _GateHWSpinlock_cfgParams;
static LAD_ClientHandle ladHandle;


/** ============================================================================
 *  Functions
 *  ============================================================================
 */
static void cleanup(int arg);


/*
 *  ======== Ipc_start ========
 */
Int Ipc_start(Void)
{
    MessageQ_Config         msgqCfg;
    MultiProc_Config        mpCfg;
#if defined(GATEMP_SUPPORT)
    GateHWSpinlock_Config   gateHWCfg;
#endif
    Int         status;
    LAD_Status  ladStatus;

    /* function must be serialized */
    pthread_mutex_lock(&Ipc_module.gate);

    /* ensure only first thread performs startup procedure */
    if (++Ipc_module.refCount > 1) {
        status = Ipc_S_ALREADYSETUP;
        goto exit;
    }

    /* make sure transport factory has been configured */
    if (Ipc_module.transportFactory == NULL) {
        status = Ipc_E_INVALIDSTATE;
        goto exit;
    }

    /* Catch ctrl-C, and cleanup: */
    (void) signal(SIGINT, cleanup);

    if (getenv("IPC_DEBUG") != NULL) {
        /* turn on tracing */
        if (getenv("IPC_DEBUG")[0] == '1') {
            /* level 1 enables typical user API tracing */
            _MessageQ_verbose = TRUE;
            _MultiProc_verbose = TRUE;
            _NameServer_verbose = TRUE;
#if defined(GATEMP_SUPPORT)
            _GateMP_verbose = TRUE;

            _GateHWSpinlock_verbose = TRUE;
#endif
        }
        else if ((getenv("IPC_DEBUG")[0] == '2') ||
                (getenv("IPC_DEBUG")[0] == '3')) {
            /* levels 2 and 3 add socket and LAD client tracing */
            _MessageQ_verbose = TRUE;
            _MultiProc_verbose = TRUE;
            _NameServer_verbose = TRUE;

#if defined(GATEMP_SUPPORT)
            _GateMP_verbose = TRUE;

            _GateHWSpinlock_verbose = TRUE;
#endif

            /* internals - should be declared in respective _*.h files */
            extern Bool _SocketFxns_verbose;
            extern Bool _LAD_Client_verbose;

            _SocketFxns_verbose = TRUE;
            _LAD_Client_verbose = TRUE;
        }
    }

    ladStatus = LAD_connect(&ladHandle);
    if (ladStatus != LAD_SUCCESS) {
        printf("Ipc_start: LAD_connect() failed: %d\n", ladStatus);
        status = Ipc_E_FAIL;
        goto exit;
    }

    /*  Get MultiProc configuration from LAD and initialize local
     *  MultiProc config structure.
     */
    MultiProc_getConfig(&mpCfg);
    _MultiProc_initCfg(&mpCfg);

    status = NameServer_setup();

    if (status >= 0) {
        MessageQ_getConfig(&msgqCfg);
        MessageQ_setup(&msgqCfg);

        /* invoke the transport factory create method */
        status = Ipc_module.transportFactory->createFxn();

        if (status < 0) {
            goto exit;
        }
    }
    else {
        printf("Ipc_start: NameServer_setup() failed: %d\n", status);
        status = Ipc_E_FAIL;
    }

    /* Start GateMP only if device has support */
#if defined(GATEMP_SUPPORT)
    if (GateMP_isSetup()) {
        /*
         * Get HWSpinlock base address and size from LAD and
         * initialize the local config structure.
         */
        GateHWSpinlock_getConfig(&gateHWCfg);
        _GateHWSpinlock_cfgParams = gateHWCfg;

        status = GateHWSpinlock_start();
        if (status < 0) {
            printf("Ipc_start: GateHWSpinlock_start failed: %d\n",
                status);
            status = Ipc_E_FAIL;
            goto gatehwspinlockstart_fail;
        }
        else {
            status = GateMP_start();
            if (status < 0) {
                printf("Ipc_start: GateMP_start failed: %d\n",
                status);
                status = Ipc_E_FAIL;
                goto gatempstart_fail;
            }
        }
    }
#endif
    /* Success */
    goto exit;
#if defined(GATEMP_SUPPORT)
gatempstart_fail:
    GateHWSpinlock_stop();
gatehwspinlockstart_fail:
#if 0
    for (procId = procId - 1; (procId > 0) && (status >= 0); procId--) {
        MessageQ_detach(procId);
    }
#endif
#endif

exit:
    /* if error, must decrement reference count */
    if (status < 0) {
        Ipc_module.refCount--;
    }

    pthread_mutex_unlock(&Ipc_module.gate);

    return (status);
}

/*
 *  ======== Ipc_stop ========
 */
Int Ipc_stop(Void)
{
    Int32       status = Ipc_S_SUCCESS;
    LAD_Status  ladStatus;
    Int         i;
    UInt16      procId;
    UInt16      clusterSize;
    UInt16      clusterBase;

    /* function must be serialized */
    pthread_mutex_lock(&Ipc_module.gate);

    /* ensure only last thread performs stop procedure */
    if (--Ipc_module.refCount > 0) {
        goto exit;
    }

    /* invoke the transport factory delete method */
    Ipc_module.transportFactory->deleteFxn();

    /* needed to enumerate processors in cluster */
    clusterSize = MultiProc_getNumProcsInCluster();
    clusterBase = MultiProc_getBaseIdOfCluster();

    /* detach from all remote processors, assuming they are up */
    for (i = 0, procId = clusterBase; i < clusterSize; i++, procId++) {

        /*  no need to detach from myself */
        if (MultiProc_self() == procId) {
            continue;
        }
#if 0
        status = MessageQ_detach(procId);
        if (status < 0) {
            printf("Ipc_stop: MessageQ_detach(%d) failed: %d\n",
                procId, status);
            status = Ipc_E_FAIL;
            goto exit;
       }
#endif
    }

    status = MessageQ_destroy();
    if (status < 0) {
        printf("Ipc_stop: MessageQ_destroy() failed: %d\n", status);
        status = Ipc_E_FAIL;
        goto exit;
    }

    status = NameServer_destroy();
    if (status < 0) {
        printf("Ipc_stop: NameServer_destroy() failed: %d\n", status);
        status = Ipc_E_FAIL;
        goto exit;
    }

    ladStatus = LAD_disconnect(ladHandle);
    if (ladStatus != LAD_SUCCESS) {
        printf("LAD_disconnect() failed: %d\n", ladStatus);
        status = Ipc_E_FAIL;
        goto exit;
    }

exit:
    pthread_mutex_unlock(&Ipc_module.gate);

    return (status);
}

/*
 *  ======== Ipc_transportConfig ========
 */
Int Ipc_transportConfig(Ipc_TransportFactoryFxns *factory)
{
    Int status = Ipc_S_SUCCESS;

    pthread_mutex_lock(&Ipc_module.gate);

    /*  Only the first caller can actually set the transport factory.
     *  Subsequent callers (e.g. multi-threaded application) must be
     *  using the same factory. Otherwise, it is an error.
     */
    if (Ipc_module.transportFactory == NULL) {
        Ipc_module.transportFactory = factory;
    }
    else if (Ipc_module.transportFactory != factory) {
        status = Ipc_E_INVALIDARG;
        goto exit;
    }

exit:
    pthread_mutex_unlock(&Ipc_module.gate);

    return (status);
}

static void cleanup(int arg)
{
    printf("Ipc: Caught SIGINT, calling Ipc_stop...\n");
    Ipc_stop();
    exit(0);
}
