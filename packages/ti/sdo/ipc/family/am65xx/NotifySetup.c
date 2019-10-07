/*
 * Copyright (c) 2017-2019 Texas Instruments Incorporated - http://www.ti.com
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
 *  ======== NotifySetup.c ========
 */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Startup.h>

#include <ti/sdo/ipc/_Notify.h>
#include <ti/sdo/ipc/family/am65xx/NotifyDriverMbx.h>
#include <ti/sdo/ipc/notifyDrivers/NotifyDriverShm.h>
#include <ti/sdo/utils/_MultiProc.h>
#include <ti/sdo/ipc/family/am65xx/NotifySciClient.h>

#if defined(xdc_target__isaCompatible_v7R)

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/family/arm/v7r/keystone3/Hwi.h>
#include <ti/sysbios/family/arm/v7r/keystone3/Core.h>

#elif defined(xdc_target__isaCompatible_v8A)
#include <ti/sysbios/family/arm/gicv3/Hwi.h>
#else
#error Invalid target
#endif

#include "package/internal/NotifySetup.xdc.h"

/* register access methods */
#define REG16(A)        (*(volatile UInt16 *)((uintptr_t)A))
#define REG32(A)        (*(volatile UInt32 *)((uintptr_t)A))

/* ipc helper macros */
#define MAILBOX_REG_VAL(m) (0x1 << (2 * (m)))

#define VIRTID(procId) (NotifySetup_procIdTable[(procId)])

#define MBX_BASEADDR_IDX(idx) ((NotifySetup_mailboxTable[(idx)] >> 16) & 0xFFFF)

#define MBX_USER_IDX(idx) ((NotifySetup_mailboxTable[(idx)] >> 8) & 0xFF)

#define SUBMBX_IDX(idx) (NotifySetup_mailboxTable[(idx)] & 0xFF)

#define MAILBOX_ADDR(idx) \
        (NotifySetup_mailboxBaseAddr[MBX_BASEADDR_IDX(idx)])

#define MAILBOX_STATUS(idx) \
        (MAILBOX_ADDR((idx)) + 0xC0 + (0x4 * SUBMBX_IDX((idx))))

#define MAILBOX_IRQENABLE_SET(idx) \
        (MAILBOX_ADDR((idx)) + 0x108 + (0x10 * MBX_USER_IDX((idx))))

#define MBOX_IRQ_ENABLE(idx) \
    ((REG32(MAILBOX_IRQENABLE_SET((idx))) & \
    MAILBOX_REG_VAL(SUBMBX_IDX((idx)))) != 0)

#define MBOX_MSG_COUNT(idx) (REG32(MAILBOX_STATUS((idx))))

/*
 *************************************************************************
 *                      Module functions
 *************************************************************************
 */

/*
 *  ======== NotifySetup_Module_startup ========
 */
Int NotifySetup_Module_startup(Int phase)
{
    Int32 retVal;
    UInt16 rangeStart;
    UInt16 rangeNum;
#if defined(xdc_target__isaCompatible_v7R)
    UInt32 coreId;

    coreId = Core_getId();
#endif

    retVal = NotifySciClient_Init();
    if ( retVal < 0 ) {
        return Startup_NOTDONE;
    }

#if defined(xdc_target__isaCompatible_v7R)
    if (coreId == 0) {
        retVal = NotifySciClient_getIntNumRange(NotifySciClient_R5F_0_CORE_INDEX,
                                                NotifySciClient_SECONDARYHOST_SPECIFIC_HOST,
                                                &rangeStart, &rangeNum);
        if ((retVal < 0) || (rangeNum < 2))  {
            retVal = NotifySciClient_getIntNumRange(NotifySciClient_R5F_0_CORE_INDEX,
                                                    NotifySciClient_SECONDARYHOST_ALL,
                                                    &rangeStart, &rangeNum);
            if ((retVal < 0) || (rangeNum < 2))  {
                return Startup_NOTDONE;
            }
        }
        /* Src id : Host */
        NotifySetup_module->interruptTable[1] = rangeStart;
        /* Src id : R5F-1 */
        NotifySetup_module->interruptTable[2] = rangeStart+1;
    } else if (coreId == 1) { /* R5F-1 */
        retVal = NotifySciClient_getIntNumRange(NotifySciClient_R5F_1_CORE_INDEX,
                                                NotifySciClient_SECONDARYHOST_SPECIFIC_HOST,
                                                &rangeStart, &rangeNum);
        if ((retVal < 0) || (rangeNum < 2))  {
            retVal = NotifySciClient_getIntNumRange(NotifySciClient_R5F_1_CORE_INDEX,
                                                    NotifySciClient_SECONDARYHOST_ALL,
                                                    &rangeStart, &rangeNum);
            if ((retVal < 0) || (rangeNum < 4))  {
                return Startup_NOTDONE;
            }
            /* Src id : R5F-0 */
            NotifySetup_module->interruptTable[0] = rangeStart+2;
            /* Src id : Host */
            NotifySetup_module->interruptTable[1] = rangeStart+3;
        } else {
            /* Src id : R5F-0 */
           NotifySetup_module->interruptTable[0] = rangeStart;
            /* Src id : Host */
           NotifySetup_module->interruptTable[1] = rangeStart+1;
        }
    } else {
        return Startup_NOTDONE;
    }
#elif defined(xdc_target__isaCompatible_v8A)
        retVal = NotifySciClient_getIntNumRange(NotifySciClient_A53_0_CORE_INDEX,
                                                NotifySciClient_SECONDARYHOST_SPECIFIC_HOST,
                                                &rangeStart, &rangeNum);
        if ((retVal < 0) || (rangeNum < 2))  {
            retVal = NotifySciClient_getIntNumRange(NotifySciClient_A53_0_CORE_INDEX,
                                                    NotifySciClient_SECONDARYHOST_ALL,
                                                    &rangeStart, &rangeNum);
            if ((retVal < 0) || (rangeNum < 2))  {
                return Startup_NOTDONE;
            }
        }
        /* Src id : R5F-0 */
        NotifySetup_module->interruptTable[0] = rangeStart;
        /* Src id : R5F-1 */
        NotifySetup_module->interruptTable[2] = rangeStart+1;
#endif
    return (Startup_DONE);
}

/*
 *  ======== NotifySetup_interruptTable ========
 */
UInt16 NotifySetup_interruptTable(Int srcVirtId)
{
    return (NotifySetup_module->interruptTable[srcVirtId]);
}

/*
 *  ======== NotifySetup_attach ========
 *  Create driver instance specified at config time.
 *
 *  This functions is generated by the NotifySetup.xdt template.
 */

/*
 *  ======== NotifySetup_sharedMemReq ========
 *  Compute how much shared memory is required by the driver.
 *
 *  This functions is generated by the NotifySetup.xdt template.
 */

/*
 * ======== NotifySetup_numIntLines ========
 * Return number of available interrupt lines to the current processor.
 */
UInt16 NotifySetup_numIntLines(UInt16 remoteProcId)
{
    return (1);
}

/*
 *  ======== NotifySetup_driverType ========
 *  Find driver type for given connection.
 *
 *  Search the connection array for the given remote processor. If
 *  found, return the requested notify driver type.
 */
NotifySetup_Driver NotifySetup_driverType(UInt16 remoteProcId)
{
    Int i;
    NotifySetup_Driver driver = NotifySetup_Driver_SHAREDMEMORY;

    /* look for remote processor in connection array */
    for (i = 0; i < NotifySetup_module->connAry.length; i++) {
        if (remoteProcId == NotifySetup_module->connAry.elem[i].procId) {
            driver = NotifySetup_module->connAry.elem[i].driver;
            break;
        }
    }

    return (driver);
}

/*
 *  ======== NotifySetup_plugHwi ========
 */
Void NotifySetup_plugHwi(UInt16 remoteProcId, Int cpuIntrNum,
        NotifySetup_DriverIsr isr)
{
    Error_Block eb;
    UInt        key;
    Hwi_Params  hwiParams;
    UInt16      srcVirtId;
#if defined(xdc_target__isaCompatible_v7R) \
    || defined(xdc_target__isaCompatible_v8A)
    UInt16      idx;
    UInt        mbxIdx;
    int retVal;
#endif

#if defined(xdc_target__isaCompatible_v7R)
    UInt32      coreId;

    coreId = Core_getId();
#endif

    Error_init(&eb);

    /* disable interrupts */
    key = Hwi_disable();
#if defined(xdc_target__isaCompatible_v7R)
    /* connect mailbox interrupts at startup */

    if (coreId == 0) {
        /* R5F-0 */
        if (remoteProcId == MultiProc_getId("R5F-1") ) {
            /* Navss mailbox 2 User 0 */
            /* Release NAVSS & MCU Level Interrupt router */
            NotifySciClient_IrqRelease(NotifySciClient_R5F_0_CORE_INDEX,
                                   NotifySciClient_MAILBOX_CLUSTER2_SRC_ID_INDEX,
                                   NotifySciClient_MAILBOX_USER_0, cpuIntrNum);
            /* Navss mailbox 2 User 0 */
            /* Configure NAVSS & MCU Level Interrupt router */
            retVal = NotifySciClient_IrqSet(NotifySciClient_R5F_0_CORE_INDEX,
                                   NotifySciClient_MAILBOX_CLUSTER2_SRC_ID_INDEX,
                                   NotifySciClient_MAILBOX_USER_0, cpuIntrNum);
        } else { /* Host */
            /* Navss mailbox 0 User 1 */
            /* Release NAVSS & MCU Level Interrupt router */
            NotifySciClient_IrqRelease(NotifySciClient_R5F_0_CORE_INDEX,
                                   NotifySciClient_MAILBOX_CLUSTER0_SRC_ID_INDEX,
                                   NotifySciClient_MAILBOX_USER_1, cpuIntrNum);
            /* Navss mailbox 0 User 1 */
            /* Configure NAVSS & MCU Level Interrupt router */
            retVal = NotifySciClient_IrqSet(NotifySciClient_R5F_0_CORE_INDEX,
                                   NotifySciClient_MAILBOX_CLUSTER0_SRC_ID_INDEX,
                                   NotifySciClient_MAILBOX_USER_1, cpuIntrNum);
         }
    } else if (coreId == 1) { /* R5F-1 */
        if (remoteProcId == MultiProc_getId("R5F-0") ) {
            /* Navss mailbox 2 User 1 */
            /* Configure NAVSS & MCU Level Interrupt router */
            NotifySciClient_IrqRelease(NotifySciClient_R5F_1_CORE_INDEX,
                                   NotifySciClient_MAILBOX_CLUSTER2_SRC_ID_INDEX,
                                   NotifySciClient_MAILBOX_USER_1, cpuIntrNum);
            /* Configure NAVSS & MCU Level Interrupt router */
            retVal = NotifySciClient_IrqSet(NotifySciClient_R5F_1_CORE_INDEX,
                                   NotifySciClient_MAILBOX_CLUSTER2_SRC_ID_INDEX,
                                   NotifySciClient_MAILBOX_USER_1, cpuIntrNum);

        } else { /* Host */
            /* Navss mailbox 1 User 1 */
            /* Release NAVSS & MCU Level Interrupt router */
            NotifySciClient_IrqRelease(NotifySciClient_R5F_1_CORE_INDEX,
                                   NotifySciClient_MAILBOX_CLUSTER1_SRC_ID_INDEX,
                                   NotifySciClient_MAILBOX_USER_1, cpuIntrNum);
            /* Configure NAVSS & MCU Level Interrupt router */
            retVal = NotifySciClient_IrqSet(NotifySciClient_R5F_1_CORE_INDEX,
                                   NotifySciClient_MAILBOX_CLUSTER1_SRC_ID_INDEX,
                                   NotifySciClient_MAILBOX_USER_1, cpuIntrNum);
        }
    } else {
        /* Error */
        Assert_isTrue(FALSE, NotifySetup_A_internal);
    }

#elif defined(xdc_target__isaCompatible_v8A)
       if (remoteProcId == MultiProc_getId("R5F-0") ) {
           /* Navss mailbox 0 User 0 */
           /* Release NAVSS interrupt router */
           NotifySciClient_IrqRelease(NotifySciClient_A53_0_CORE_INDEX,
                                   NotifySciClient_MAILBOX_CLUSTER0_SRC_ID_INDEX,
                                   NotifySciClient_MAILBOX_USER_0, cpuIntrNum);
           /* Configure NAVSS interrupt router */
           retVal = NotifySciClient_IrqSet(NotifySciClient_A53_0_CORE_INDEX,
                                   NotifySciClient_MAILBOX_CLUSTER0_SRC_ID_INDEX,
                                   NotifySciClient_MAILBOX_USER_0, cpuIntrNum);

       } else {
           /* Navss mailbox 1 User 0 */
           /* Release NAVSS interrupt router */
           NotifySciClient_IrqRelease(NotifySciClient_A53_0_CORE_INDEX,
                                 NotifySciClient_MAILBOX_CLUSTER1_SRC_ID_INDEX,
                                 NotifySciClient_MAILBOX_USER_0, cpuIntrNum);
           /* Configure NAVSS interrupt router */
           retVal = NotifySciClient_IrqSet(NotifySciClient_A53_0_CORE_INDEX,
                                 NotifySciClient_MAILBOX_CLUSTER1_SRC_ID_INDEX,
                                 NotifySciClient_MAILBOX_USER_0, cpuIntrNum);
       }
#else
#error Invalid target
#endif
    Assert_isTrue((retVal == 0),
                  NotifySetup_A_error_resource_allocation);
    /* Additional check to handle case when Assert is disabled */
    if (retVal != 0) {
        return;
    }

    /* map remote processor id to virtual id */
    srcVirtId = VIRTID(remoteProcId);

    /* save driver ISR in dispatch table */
    NotifySetup_module->isrDispatchTable[srcVirtId] = isr;

#if defined(xdc_target__isaCompatible_v7R) \
    || defined(xdc_target__isaCompatible_v8A)

    /* compute table index for given source and destination */
    idx = (srcVirtId * NotifySetup_NUM_CORES) + VIRTID(MultiProc_self());

    /* compute mailbox index */
    mbxIdx = MBX_BASEADDR_IDX(idx);

    /* make sure the interrupt is plugged only once */
    NotifySetup_module->numPlugged[mbxIdx]++;

    if (NotifySetup_module->numPlugged[mbxIdx] == 1) {

        Hwi_Params_init(&hwiParams);
        hwiParams.maskSetting = Hwi_MaskingOption_LOWER;
        hwiParams.arg = cpuIntrNum;

        Hwi_create(cpuIntrNum, NotifySetup_dispatchIsr, &hwiParams, &eb);
        /* TODO: add error handling */

        Hwi_enableInterrupt(cpuIntrNum);
    }

#else
#error Invalid target
#endif

    /* restore interrupts */
    Hwi_restore(key);
}

/*
 *  ======== NotifySetup_unplugHwi ========
 */
Void NotifySetup_unplugHwi(UInt16 remoteProcId, Int cpuIntrNum)
{
    UInt        key;
    Hwi_Handle  hwi;
    UInt16      srcVirtId;
#if defined(xdc_target__isaCompatible_v7R) \
    || defined(xdc_target__isaCompatible_v8A)
    UInt16      idx;
    UInt        mbxIdx;
#endif
#if defined(xdc_target__isaCompatible_v7R)
    UInt32      coreId;

    coreId = Core_getId();
#endif

    /* disable global interrupts (TODO: should be a gated module) */
    key = Hwi_disable();

    /* map processor id to virtual id */
    srcVirtId = VIRTID(remoteProcId);

    /* remove driver isr from dispatch table */
    NotifySetup_module->isrDispatchTable[srcVirtId] = NULL;

#if defined(xdc_target__isaCompatible_v7R) \
    || defined(xdc_target__isaCompatible_v8A)

    /* decrement plug count */
    idx = (srcVirtId * NotifySetup_NUM_CORES) + VIRTID(MultiProc_self());
    mbxIdx = MBX_BASEADDR_IDX(idx);
    NotifySetup_module->numPlugged[mbxIdx]--;

    /* unplug interrupt if last user */
    if (NotifySetup_module->numPlugged[0] == 0) {
        hwi = Hwi_getHandle(cpuIntrNum);
        Hwi_delete(&hwi);
    }
#if defined(xdc_target__isaCompatible_v7R)
    /* connect mailbox interrupts at startup */

    if (coreId == 0) {
        /* R5F-0 */
        if (remoteProcId == MultiProc_getId("R5F-1") ) {
            /* Navss mailbox 2 User 0 */
            /* Release NAVSS & MCU Level Interrupt router */
            NotifySciClient_IrqRelease(NotifySciClient_R5F_0_CORE_INDEX,
                                   NotifySciClient_MAILBOX_CLUSTER2_SRC_ID_INDEX,
                                   NotifySciClient_MAILBOX_USER_0, cpuIntrNum);
        } else { /* Host */
            /* Navss mailbox 0 User 1 */
            /* Release NAVSS & MCU Level Interrupt router */
            NotifySciClient_IrqRelease(NotifySciClient_R5F_0_CORE_INDEX,
                                   NotifySciClient_MAILBOX_CLUSTER0_SRC_ID_INDEX,
                                   NotifySciClient_MAILBOX_USER_1, cpuIntrNum);
         }
    } else if (coreId == 1) { /* R5F-1 */
        if (remoteProcId == MultiProc_getId("R5F-0") ) {
            /* Navss mailbox 2 User 1 */
            /* Configure NAVSS & MCU Level Interrupt router */
            NotifySciClient_IrqRelease(NotifySciClient_R5F_1_CORE_INDEX,
                                   NotifySciClient_MAILBOX_CLUSTER2_SRC_ID_INDEX,
                                   NotifySciClient_MAILBOX_USER_1, cpuIntrNum);
        } else { /* Host */
            /* Navss mailbox 1 User 1 */
            /* Release NAVSS & MCU Level Interrupt router */
            NotifySciClient_IrqRelease(NotifySciClient_R5F_1_CORE_INDEX,
                                   NotifySciClient_MAILBOX_CLUSTER1_SRC_ID_INDEX,
                                   NotifySciClient_MAILBOX_USER_1, cpuIntrNum);
        }
    } else {
        /* Error */
        Assert_isTrue(FALSE, NotifySetup_A_internal);
    }

#elif defined(xdc_target__isaCompatible_v8A)
       if (remoteProcId == MultiProc_getId("R5F-0") ) {
           /* Navss mailbox 0 User 0 */
           /* Release NAVSS interrupt router */
           NotifySciClient_IrqRelease(NotifySciClient_A53_0_CORE_INDEX,
                                   NotifySciClient_MAILBOX_CLUSTER0_SRC_ID_INDEX,
                                   NotifySciClient_MAILBOX_USER_0, cpuIntrNum);
       } else {
           /* Navss mailbox 1 User 0 */
           /* Release NAVSS interrupt router */
           NotifySciClient_IrqRelease(NotifySciClient_A53_0_CORE_INDEX,
                                 NotifySciClient_MAILBOX_CLUSTER1_SRC_ID_INDEX,
                                 NotifySciClient_MAILBOX_USER_0, cpuIntrNum);
       }
#else
#error Invalid target
#endif
#endif /* defined(xdc_target__isaCompatible_v7R) \
    || defined(xdc_target__isaCompatible_v8A) */

    /* restore global interrupts */
    Hwi_restore(key);
}

/*
 *  ======== NotifySetup_Shm_attach ========
 */
Int NotifySetup_Shm_attach(UInt16 remoteProcId, Ptr sharedAddr)
{
    NotifyDriverShm_Params notifyShmParams;
    NotifyDriverShm_Handle shmDrvHandle;
    ti_sdo_ipc_Notify_Handle notifyHandle;
    Int status = Notify_S_SUCCESS;
    Error_Block eb;

    Error_init(&eb);

    NotifyDriverShm_Params_init(&notifyShmParams);
    notifyShmParams.sharedAddr = sharedAddr;
    notifyShmParams.remoteProcId  = remoteProcId;

    /* create the notify driver instance */
    shmDrvHandle = NotifyDriverShm_create(&notifyShmParams, &eb);

    if (shmDrvHandle == NULL) {
        return (Notify_E_FAIL);
    }

    /* create the front-end notify instance */
    notifyHandle = ti_sdo_ipc_Notify_create(
            NotifyDriverShm_Handle_upCast(shmDrvHandle), remoteProcId, 0,
            NULL, &eb);

    if (notifyHandle == NULL) {
        NotifyDriverShm_delete(&shmDrvHandle);
        status = Notify_E_FAIL;
    }

    return (status);
}

/*!
 *  ======== NotifySetup_Shm_sharedMemReq ========
 */
SizeT NotifySetup_Shm_sharedMemReq(UInt16 remoteProcId, Ptr sharedAddr)
{
    SizeT memReq;
    NotifyDriverShm_Params notifyShmParams;

    NotifyDriverShm_Params_init(&notifyShmParams);
    notifyShmParams.sharedAddr = sharedAddr;

    memReq = NotifyDriverShm_sharedMemReq(&notifyShmParams);

    return (memReq);
}

/*
 *  ======== NotifySetup_Mbx_attach ========
 */
Int NotifySetup_Mbx_attach(UInt16 remoteProcId, Ptr sharedAddr)
{
    Int status = Notify_S_SUCCESS;
    NotifyDriverMbx_Params params;
    NotifyDriverMbx_Handle driver;
    ti_sdo_ipc_Notify_Handle notify;
    UInt16 virtId;
    Error_Block eb;

    Error_init(&eb);

    NotifyDriverMbx_Params_init(&params);
    params.remoteProcId = remoteProcId;

    /* set the intVectorId if on the R5F */
    if ((MultiProc_self() == NotifySetup_r5f_0ProcId) ||
        (MultiProc_self() == NotifySetup_r5f_1ProcId)) {

        virtId = VIRTID(remoteProcId);
        params.intVectorId = NotifySetup_module->interruptTable[virtId];
    }

    /* set the intVectorId if on the HOST */
    if (MultiProc_self() == NotifySetup_hostProcId) {
        virtId = VIRTID(remoteProcId);
        params.intVectorId = NotifySetup_module->interruptTable[virtId];
    }

    /* create the notify driver instance */
    driver = NotifyDriverMbx_create(&params, &eb);

    if (driver == NULL) {
        return (Notify_E_FAIL);
    }

    /* create the front-end notify instance */
    notify = ti_sdo_ipc_Notify_create(NotifyDriverMbx_Handle_upCast(driver),
            remoteProcId, 0, NULL, &eb);

    if (notify == NULL) {
        NotifyDriverMbx_delete(&driver);
        status = Notify_E_FAIL;
    }

    return (status);
}

/*!
 *  ======== NotifySetup_Mbx_sharedMemReq ========
 */
SizeT NotifySetup_Mbx_sharedMemReq(UInt16 remoteProcId, Ptr sharedAddr)
{
    SizeT memReq = 0;

    return (memReq);
}

/*
 *************************************************************************
 *                       Internal functions
 *************************************************************************
 */

/*
 *  ======== NotifySetup_dispatchIsr ========
 *  Dispatch the current interrupt to the appropriate notify driver
 *
 *  The given interrupt may be shared by multiple notify drivers. This
 *  ISR inspects the mailbox which raised the interrupt and looks for
 *  all FIFOs which have data and raise the given interrupt. For each
 *  one, the interrupt is dispatched to the registered driver for that
 *  FIFO.
 *
 *  @param(arg) The eventId which raised the interrupt.
 */
Void NotifySetup_dispatchIsr(UArg arg)
{
    Int numProcessed;
    UInt16 idx;
    UInt16 srcVirtId;
    UInt16 dstVirtId = VIRTID(MultiProc_self());
    NotifySetup_DriverIsr driver;

    do {
        numProcessed = 0;

        for (srcVirtId = 0; srcVirtId < NotifySetup_NUM_CORES; srcVirtId++) {

            /* skip null drivers, processor not in system or self */
            driver = NotifySetup_module->isrDispatchTable[srcVirtId];

            if (driver == NULL) {
                continue;
            }

            /* check if processor would raise the given hardware eventId */
            if (arg == NotifySetup_module->interruptTable[srcVirtId]) {

                /* compute table index for given source and destination */
                idx = (srcVirtId * NotifySetup_NUM_CORES) + dstVirtId;

                /* check if submailbox has a message and irq is enabled */
                if ((MBOX_MSG_COUNT(idx) != 0) && MBOX_IRQ_ENABLE(idx)) {

                    /* invoke driver isr to deliver the event */
                    (*driver)(idx);

                    /* event has been delivered */
                    numProcessed++;
                }
            }
        }
    } while (numProcessed != 0);
}
