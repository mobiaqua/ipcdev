/*
 * Copyright (c) 2017-2018 Texas Instruments Incorporated - http://www.ti.com
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


#define EVENT_GROUP_SIZE 32

/* register access methods */
#define REG16(A)        (*(volatile UInt16 *)(A))
#define REG32(A)        (*(volatile UInt32 *)(A))

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

#define M2M_LVL_INT_RTR_BASE 0x00A10000
#define M2M_LV_INT_ICR0_OFFSET 0x4
#define NAVSS_INT_RTR_BASE 0x310E0000
#define NAVSS_INT_ICR0_OFFSET 0x4

/* Corresponds to VIM: 160 */
#define M2M_LVL_INT_RTR_OUTPUT_R5F0_0 0
/* Corresponds to VIM: 161 */
#define M2M_LVL_INT_RTR_OUTPUT_R5F0_1 1
/* Corresponds to VIM: 162 */
#define M2M_LVL_INT_RTR_OUTPUT_R5F1_0 2
/* Corresponds to VIM: 163 */
#define M2M_LVL_INT_RTR_OUTPUT_R5F1_1 3

#define NAVSS_INT_RTR_INPUT_MAILBOX0_USER0 436
#define NAVSS_INT_RTR_INPUT_MAILBOX0_USER1 437

#define NAVSS_INT_RTR_INPUT_MAILBOX1_USER0 432
#define NAVSS_INT_RTR_INPUT_MAILBOX1_USER1 433

#define NAVSS_INT_RTR_INPUT_MAILBOX2_USER0 428
#define NAVSS_INT_RTR_INPUT_MAILBOX2_USER1 429

/* Corresponds to GIC: 496 */
#define NAVSS_INT_RTR_OUTPUT_A53_0    112
/* Corresponds to GIC: 497 */
#define NAVSS_INT_RTR_OUTPUT_A53_1    113

#define NAVSS_INT_RTR_OUTPUT_R5F0_0 120
#define NAVSS_INT_RTR_OUTPUT_R5F1_0 121
#define NAVSS_INT_RTR_OUTPUT_R5F0_1 122
#define NAVSS_INT_RTR_OUTPUT_R5F1_1 123

/* The following INPUT is connected to NAVSS OUTPUT 120 */
#define M2M_LVL_INT_RTR_INPUT_A53_PEND_120 184
/* The following INPUT is connected to NAVSS OUTPUT 121 */
#define M2M_LVL_INT_RTR_INPUT_A53_PEND_121 185

/* The following INPUT is connected to NAVSS OUTPUT 122 */
#define M2M_LVL_INT_RTR_INPUT_A53_PEND_122 186
/* The following INPUT is connected to NAVSS OUTPUT 123 */
#define M2M_LVL_INT_RTR_INPUT_A53_PEND_123 187

static inline void connect_m2m_lvl_int_rtr(UInt32 input_evt, UInt32 output_line)
{
#ifdef INTERRUPT_ROUTING_THROUGH_DMSC
    /* TODO: Need to add code to configure routing through DMSC */
#else
    /* TODO: Eventually the interrupt routing below cannot be done
     * directly. Need to go through DMSC. Currently this is done
     * directly to help Pre-silicon testing
     */
    *((UInt32 *)(M2M_LVL_INT_RTR_BASE + M2M_LV_INT_ICR0_OFFSET) + output_line) = input_evt;
#endif
}

static inline void connect_navss_int_rtr(UInt32 input_evt, UInt32 output_line)
{
#ifdef INTERRUPT_ROUTING_THROUGH_DMSC
    /* TODO: Need to add code to configure routing through DMSC */
#else
    /* TODO: Eventually the interrupt routing below cannot be done
     * directly. Need to go through DMSC. Currently this is done
     * directly to help Pre-silicon testing
     */
    *((UInt32 *)(NAVSS_INT_RTR_BASE + NAVSS_INT_ICR0_OFFSET) + output_line) = input_evt;
#endif
}

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
#if defined(xdc_target__isaCompatible_v7R)
    /* connect mailbox interrupts at startup */
    if ((Core_getId() == 0)) {
        /* R5F-0 */
        /* Navss mailbox 0 User 1 */
        /* Configure NAVSS interrupt router */
        connect_navss_int_rtr(NAVSS_INT_RTR_INPUT_MAILBOX0_USER1,
            NAVSS_INT_RTR_OUTPUT_R5F0_0);
        /* Configure MCU level interrupt router */
        connect_m2m_lvl_int_rtr(M2M_LVL_INT_RTR_INPUT_A53_PEND_120,
            M2M_LVL_INT_RTR_OUTPUT_R5F0_0);

        /* plug mbx2 only if R5F-1 exists */
        if ((MultiProc_getId("R5F-1") != MultiProc_INVALIDID)) {
            /* Navss mailbox 2 User 0 */
            /* Configure NAVSS interrupt router */
            connect_navss_int_rtr(NAVSS_INT_RTR_INPUT_MAILBOX2_USER0,
                NAVSS_INT_RTR_OUTPUT_R5F0_1);
            /* Configure MCU level interrupt router */
            connect_m2m_lvl_int_rtr(M2M_LVL_INT_RTR_INPUT_A53_PEND_121,
                M2M_LVL_INT_RTR_OUTPUT_R5F0_1);
        }
    }
    else { /* R5F-1 */
        /* Navss mailbox 1 User 1 */
        /* Configure NAVSS interrupt router */
        connect_navss_int_rtr(NAVSS_INT_RTR_INPUT_MAILBOX1_USER1,
            NAVSS_INT_RTR_OUTPUT_R5F1_0);
        /* Configure MCU level interrupt router */
        connect_m2m_lvl_int_rtr(M2M_LVL_INT_RTR_INPUT_A53_PEND_122,
            M2M_LVL_INT_RTR_OUTPUT_R5F1_1);

        /* plug mbx2 only if R5F-0 exists */
        if ((MultiProc_getId("R5F-0") != MultiProc_INVALIDID)) {
            /* Navss mailbox 2 User 1 */
            /* Configure NAVSS interrupt router */
            connect_navss_int_rtr(NAVSS_INT_RTR_INPUT_MAILBOX2_USER1,
                NAVSS_INT_RTR_OUTPUT_R5F1_1);
            /* Configure MCU level interrupt router */
            connect_m2m_lvl_int_rtr(M2M_LVL_INT_RTR_INPUT_A53_PEND_123,
                M2M_LVL_INT_RTR_OUTPUT_R5F1_1);
        }
    }
    return (Startup_DONE);

#elif defined(xdc_target__isaCompatible_v8A)
    /* Navss mailbox 0 User 0 */
    /* Configure NAVSS interrupt router */
    connect_navss_int_rtr(NAVSS_INT_RTR_INPUT_MAILBOX0_USER0,
        NAVSS_INT_RTR_OUTPUT_A53_0);
    /* Navss mailbox 1 User 0 */
    /* Configure NAVSS interrupt router */
    connect_navss_int_rtr(NAVSS_INT_RTR_INPUT_MAILBOX1_USER0,
        NAVSS_INT_RTR_OUTPUT_A53_1);
    return (Startup_DONE);

#else
#error Invalid target
#endif
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
#endif

    Error_init(&eb);

    /* disable interrupts */
    key = Hwi_disable();

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

#else
#error Invalid target
#endif

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
