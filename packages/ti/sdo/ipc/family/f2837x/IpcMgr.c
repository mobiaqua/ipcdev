/*
 * Copyright (c) 2014, Texas Instruments Incorporated
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
/*
 *  ======== IpcMgr.c ========
 */

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Startup.h>

#include <ti/sysbios/hal/Hwi.h>
#include <ti/sdo/ipc/family/f2837x/NotifyDriverCirc.h>
#include <ti/sdo/ipc/family/f2837x/NameServerBlock.h>
#include <ti/sdo/ipc/family/f2837x/TransportCirc.h>
#include <ti/sdo/ipc/_MessageQ.h>
#include <ti/sdo/ipc/_Notify.h>
#include <ti/sdo/utils/_MultiProc.h>

#include "package/internal/IpcMgr.xdc.h"

#define IPCACK  (0x50000)
#define IPCSTS  (IPCACK + 0x2)
#define IPCSET  (IPCACK + 0x4)
#define IPCCLR  (IPCACK + 0x6)
#define IPCFLG  (IPCACK + 0x8)

#define CPU1_IPCFLAG  10
#define CPU2_IPCFLAG  11

/*
 *  ======== IpcMgr_Module_startup ========
 *  In this function CPU1 is used to enable shared memory.
 *  In addition, the owner of each block of memory initializes its blocks
 *  based on the static configuration parameters.
 *  IPC flags are used by CPU1 and CPU2 for synchronization.
 *
 *  CPU1 starts by creating its driver instances and then sets CPU2's
 *  IPC flag, allowing CPU2 proceed.  Then CPU1 waits for its
 *  IPC flag to be set by CPU2 before proceeding. Once CPU1's IPC flag is set
 *  by the CPU2, CPU1 clears its IPC flag and continues.
 *
 *  CPU2 starts by waiting for its IPC flag to be set by CPU1.
 *  Once its IPC flag is set by CPU1, CPU2 clears its IPC flag and
 *  proceeds to create its driver instances.  CPU2 then sets CPU1's
 *  IPC flag to let CPU1 proceed.
 *
 *  The shared memory usage looks like the following:
 *
 *      |--------------------|
 *      | Notify Driver      |
 *      |                    |
 *      |--------------------|
 *      | NameServer         |
 *      | Remote Driver      |
 *      |--------------------|
 *      | MessageQ Transport |
 *      |                    |
 *      |--------------------|
 */
Int IpcMgr_Module_startup(Int phase)
{
    Int status;
    SizeT memReq;
    Ptr writeAddr = (UInt32 *)IpcMgr_writeAddr;
    Ptr readAddr = (UInt32 *)IpcMgr_readAddr;
    UInt16 remoteProcId;
    NotifyDriverCirc_Params notifyDrvParams;
    TransportCirc_Params transportParams;
    volatile UInt32 *set = (volatile UInt32 *)IPCSET;
    volatile UInt32 *stat = (volatile UInt32 *)IPCSTS;
    volatile UInt32 *ack = (volatile UInt32 *)IPCACK;

    /*
     *  This code assumes that the device's CPU1 and CPU2 MultiProc Ids
     *  are next to each other (e.g. n and n + 1) and that the first
     *  one is even (e.g. n is even).
     */
    if (MultiProc_self() & 1) {
        /* I'm odd */
        remoteProcId = MultiProc_self() - 1;
    }
    else {
        /* I'm even */
        remoteProcId = MultiProc_self() + 1;
    }

    /* Wait for Hwi module to initialize first because of NotifyDriverCirc  */
    if (!Hwi_Module_startupDone()) {
        return Startup_NOTDONE;
    }

    if (!IpcMgr_cpu1) {
        /* Wait for CPU1 to set CPU2's IPC flag */
        while (!(*stat & (1 << CPU2_IPCFLAG))) {
        }

        /* Clear own IPC flag */
        *ack = 1 << CPU2_IPCFLAG;
    }

    /* Determine the amount of memory required for NotifyDriverCirc */
    NotifyDriverCirc_Params_init(&notifyDrvParams);
    notifyDrvParams.writeAddr = writeAddr;
    memReq = NotifyDriverCirc_sharedMemReq(&notifyDrvParams);

    /* Call NotifyCircSetup attach to remote processor */
    status = IpcMgr_notifyCircAttach(remoteProcId,
                 writeAddr, readAddr);

    Assert_isTrue(status >= 0, IpcMgr_A_internal);

    /* Update the read/write address */
    writeAddr = (Ptr)((UInt32)writeAddr + memReq);
    readAddr = (Ptr)((UInt32)readAddr + memReq);

    /* Determine the amount of memory required for NameServerBlock */
    memReq = NameServerBlock_sharedMemReq(NULL);

    /* Call NameServerBlock attach to remote processor */
    status = IpcMgr_nameServerAttach(remoteProcId, writeAddr, readAddr);

    Assert_isTrue(status >= 0, IpcMgr_A_internal);

    /* Update the read/write address */
    writeAddr = (Ptr)((UInt32)writeAddr + memReq);
    readAddr = (Ptr)((UInt32)readAddr + memReq);

    /* Determine the amount of memory required for TransportCirc */
    TransportCirc_Params_init(&transportParams);
    transportParams.writeAddr = writeAddr;
    memReq = TransportCirc_sharedMemReq(&transportParams);

    /* Call TransportCircSetup attach to remote processor */
    status = IpcMgr_transportCircAttach(remoteProcId,
                 writeAddr, readAddr);

    Assert_isTrue(status >= 0, IpcMgr_A_internal);

    if (IpcMgr_cpu1) {
        /* Set CPU2 IPC flag to tell CPU2 to proceed */
        *set = 1 << CPU2_IPCFLAG;

        /* Wait for CPU2 to set CPU1's IPC flag */
        while (!(*stat & (1 << CPU1_IPCFLAG))) {
        }

        /* Clear own IPC flag */
        *ack = 1 << CPU1_IPCFLAG;
    }
    else {
        /* Set CPU1's IPC flag to tell CPU1 to proceed */
        *set = 1 << CPU1_IPCFLAG;
    }
    return (Startup_DONE);
}

/*
 *  ======== IpcMgr_notifyCircAttach ========
 *  Initialize interrupt
 */
Int IpcMgr_notifyCircAttach(UInt16 remoteProcId, Ptr writeAddr, Ptr readAddr)
{
    NotifyDriverCirc_Params notifyDrvParams;
    NotifyDriverCirc_Handle notifyDrvHandle;
    ti_sdo_ipc_Notify_Handle notifyHandle;
    Error_Block eb;
    Int status = Notify_S_SUCCESS;

    /* Initialize the error block */
    Error_init(&eb);

    /* Setup the notify driver to the remote processor */
    NotifyDriverCirc_Params_init(&notifyDrvParams);

    /* Set the read/write address of the param */
    notifyDrvParams.readAddr = readAddr;
    notifyDrvParams.writeAddr = writeAddr;

    /* Create the notify driver instance */
    notifyDrvHandle = NotifyDriverCirc_create(&notifyDrvParams, &eb);
    if (notifyDrvHandle == NULL) {
        return (Notify_E_FAIL);
    }

    /* Create the notify instance */
    notifyHandle = ti_sdo_ipc_Notify_create(
                       NotifyDriverCirc_Handle_upCast(notifyDrvHandle),
                       remoteProcId, 0, NULL, &eb);

    if (notifyHandle == NULL) {
        NotifyDriverCirc_delete(&notifyDrvHandle);
        status = Notify_E_FAIL;
    }

    return (status);
}

/*
 *  ======== IpcMgr_nameServerAttach ========
 */
Int IpcMgr_nameServerAttach(UInt16 remoteProcId, Ptr writeAddr, Ptr readAddr)
{
    NameServerBlock_Params nsbParams;
    NameServerBlock_Handle handle;
    Int status = NameServerBlock_E_FAIL;
    Error_Block eb;

    Error_init(&eb);

    /* Init the param */
    NameServerBlock_Params_init(&nsbParams);

    /* Set the read/write addresses */
    nsbParams.readAddr  = readAddr;
    nsbParams.writeAddr = writeAddr;

    /* Create only if notify driver has been created to remote proc */
    if (Notify_intLineRegistered(remoteProcId, 0)) {
        handle = NameServerBlock_create(remoteProcId,
                                        &nsbParams,
                                        &eb);
        if (handle != NULL) {
            status = NameServerBlock_S_SUCCESS;
        }
    }

    return (status);
}

/*
 *  ======== IpcMgr_transportCircAttach ========
 */
Int IpcMgr_transportCircAttach(UInt16 remoteProcId, Ptr writeAddr,
    Ptr readAddr)
{
    TransportCirc_Handle handle;
    TransportCirc_Params params;
    Int status = MessageQ_E_FAIL;
    Error_Block eb;

    Error_init(&eb);

    /* Init the transport parameters */
    TransportCirc_Params_init(&params);
    params.readAddr = readAddr;
    params.writeAddr = writeAddr;

    /* Make sure notify driver has been created */
    if (Notify_intLineRegistered(remoteProcId, 0)) {
        handle = TransportCirc_create(remoteProcId, &params, &eb);

        if (handle != NULL) {
            status = MessageQ_S_SUCCESS;
        }
    }

    return (status);
}
