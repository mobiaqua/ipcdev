/*
 * Copyright (c) 2014 Texas Instruments Incorporated - http://www.ti.com
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
 *  ======== NotifyDriverMbx.c ========
 */

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Startup.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/shared/vayu/IntXbar.h>

#include <ti/sdo/ipc/_Ipc.h>
#include <ti/sdo/ipc/_Notify.h>
#include <ti/sdo/ipc/family/vayu/NotifySetup.h>
#include <ti/sdo/ipc/interfaces/INotifyDriver.h>
#include <ti/sdo/utils/_MultiProc.h>

#include "package/internal/NotifyDriverMbx.xdc.h"

/* Bit mask operations */
#define SET_BIT(num,pos)            ((num) |= (1u << (pos)))
#define CLEAR_BIT(num,pos)          ((num) &= ~(1u << (pos)))
#define TEST_BIT(num,pos)           ((num) & (1u << (pos)))

/* register access methods */
#define REG16(A)        (*(volatile UInt16 *)(A))
#define REG32(A)        (*(volatile UInt32 *)(A))

#define MAILBOX_FIFOLENGTH 4
#define PROCID(idx) (NotifyDriverMbx_procIdTable[idx])

#define MBX_BASEADDR_IDX(idx) \
        ((NotifyDriverMbx_mailboxTable[idx] >> 16) & 0xFFFF)

#define MAILBOX_ADDR(idx) \
        (NotifyDriverMbx_mailboxBaseAddr[MBX_BASEADDR_IDX(idx)])

#define MBX_TABLE_IDX(src, dst) \
        ((PROCID(src) * NotifyDriverMbx_NUM_CORES) + PROCID(dst))

#define SUBMBX_IDX(idx) (NotifyDriverMbx_mailboxTable[idx] & 0xFF)

#define MBX_USER_IDX(idx) ((NotifyDriverMbx_mailboxTable[idx] >> 8) & 0xFF)

#define MAILBOX_REG_VAL(m) (0x1 << (2 * m))

#define MAILBOX_MESSAGE(idx) \
        (MAILBOX_ADDR(idx) + 0x40 + (0x4 * SUBMBX_IDX(idx)))

#define MAILBOX_STATUS(idx) \
        (MAILBOX_ADDR(idx) + 0xC0 + (0x4 * SUBMBX_IDX(idx)))

#define MAILBOX_IRQSTATUS_CLR(idx) \
        (MAILBOX_ADDR(idx) + 0x104 + (0x10 * MBX_USER_IDX(idx)))

#define MAILBOX_IRQENABLE_SET(idx) \
        (MAILBOX_ADDR(idx) + 0x108 + (0x10 * MBX_USER_IDX(idx)))

#define MAILBOX_IRQENABLE_CLR(idx) \
        (MAILBOX_ADDR(idx) + 0x10C + (0x10 * MBX_USER_IDX(idx)))

#define MAILBOX_EOI_REG(idx) (MAILBOX_ADDR(idx) + 0x140)

#define EVENT_GROUP_SIZE 32

/* empty the mailbox for the given index, clear its interrupt */
#define MAILBOX_INIT(idx)                                                   \
    while (REG32(MAILBOX_STATUS(idx)) != 0) {                               \
        REG32(MAILBOX_MESSAGE(idx));                                        \
    }                                                                       \
    REG32(MAILBOX_IRQSTATUS_CLR(idx)) = MAILBOX_REG_VAL(SUBMBX_IDX(idx));

/*
 *************************************************************************
 *                       Module functions
 *************************************************************************
 */

/*
 *  ======== NotifyDriverMbx_Module_startup ========
 */
Int NotifyDriverMbx_Module_startup(Int phase)
{
#if defined(xdc_target__isaCompatible_64)
    extern cregister volatile UInt DNUM;

    if (IntXbar_Module_startupDone()) {
        /* connect mailbox interrupts at startup */
        if (DNUM == 0) {               /* DSP1 */
            IntXbar_connect(24, 284);  // eve1 mailbox 0 user 1
            IntXbar_connect(25, 293);  // eve2 mailbox 0 user 1
            IntXbar_connect(26, 249);  // system mailbox 5 user 0

            NotifyDriverMbx_module->interruptTable[6] = 57; // IPU1-0
            NotifyDriverMbx_module->interruptTable[9] = 57; // IPU1-1

            /* plug eve3 and eve4 mbxs only if eve3 and eve4 exists */
            if ((MultiProc_getId("EVE3") != MultiProc_INVALIDID) ||
                (MultiProc_getId("EVE4") != MultiProc_INVALIDID)) {
                IntXbar_connect(27, 302);  // eve3 mailbox 0 user 1
                IntXbar_connect(28, 311);  // eve4 mailbox 0 user 1
            }

            /* plug mbx7 only if DSP2 or IPU2 exists */
            if ((MultiProc_getId("DSP2") != MultiProc_INVALIDID) ||
                (MultiProc_getId("IPU2") != MultiProc_INVALIDID) ||
                (MultiProc_getId("IPU2-0") != MultiProc_INVALIDID)) {
                IntXbar_connect(29, 257);  // system mailbox 7 user 0
                NotifyDriverMbx_module->interruptTable[7] = 60; // IPU2-0
            }

            /* plug mbx8 only if IPU2-1 exists */
            if (MultiProc_getId("IPU2-1") != MultiProc_INVALIDID) {
                IntXbar_connect(30, 261);  // system mailbox 8 user 0
                NotifyDriverMbx_module->interruptTable[10] = 61; // IPU2-1
            }
        }
        else if (DNUM == 1) {          /* DSP2 */
            IntXbar_connect(24, 287);  // eve1 mailbox 1 user 1
            IntXbar_connect(25, 296);  // eve2 mailbox 1 user 1
            IntXbar_connect(26, 253);  // system mailbox 6 user 0

            NotifyDriverMbx_module->interruptTable[7] = 57; // IPU2-0
            NotifyDriverMbx_module->interruptTable[10] = 57; // IPU2-1

            /* plug eve3 and eve4 mbxs only if eve3 and eve4 exists */
            if ((MultiProc_getId("EVE3") != MultiProc_INVALIDID) ||
                (MultiProc_getId("EVE4") != MultiProc_INVALIDID)) {
                IntXbar_connect(27, 305);  // eve3 mailbox 1 user 1
                IntXbar_connect(28, 314);  // eve4 mailbox 1 user 1
            }

            /* plug mbx7 only if DSP1 or IPU1 exists */
            if ((MultiProc_getId("DSP1") != MultiProc_INVALIDID) ||
                (MultiProc_getId("IPU1") != MultiProc_INVALIDID) ||
                (MultiProc_getId("IPU1-0") != MultiProc_INVALIDID)) {
                IntXbar_connect(29, 258);  // system mailbox 7 user 1
                NotifyDriverMbx_module->interruptTable[6] = 60; // IPU1-0
            }

            /* plug mbx8 only if IPU1-1 exists */
            if (MultiProc_getId("IPU1-1") != MultiProc_INVALIDID) {
                IntXbar_connect(30, 262);  // system mailbox 8 user 1
                NotifyDriverMbx_module->interruptTable[9] = 61; // IPU1-1
            }
        }
        return (Startup_DONE);
    }
#elif defined(xdc_target__isaCompatible_v7M)

#else

#endif
    return (Startup_NOTDONE);
}

/*
 **************************************************************
 *                       Instance functions
 **************************************************************
 */

/*
 *  ======== NotifyDriverMbx_Instance_init ========
 */
Void NotifyDriverMbx_Instance_init(NotifyDriverMbx_Object *obj,
        const NotifyDriverMbx_Params *params)
{
    UInt        key;
    UInt16      selfVirtId;
    UInt16      index;

    /*
     * Check whether remote proc ID has been set and isn't the same as the
     * local proc ID
     */
    Assert_isTrue((params->remoteProcId != MultiProc_INVALIDID) &&
            (params->remoteProcId != MultiProc_self()),
            ti_sdo_ipc_Ipc_A_invParam);

    if (params->remoteProcId >= MultiProc_getNumProcessors() ||
        params->remoteProcId == MultiProc_INVALIDID) {
        return;    /* keep Coverity happy */
    }

    obj->evtRegMask = 0;
    obj->notifyHandle = NULL;
    obj->remoteProcId = params->remoteProcId;
    obj->remoteVirtId = PROCID(params->remoteProcId);
    obj->cpuIntrNum = params->intVectorId;

    /* disable global interrupts */
    key = Hwi_disable();

    /* clear mailbox of any old messages */
    selfVirtId = PROCID(MultiProc_self());
    index = (selfVirtId * NotifyDriverMbx_NUM_CORES) + obj->remoteVirtId;
    MAILBOX_INIT(index)

    /* must use processor virtual ID to store driver handle in table */
    NotifyDriverMbx_module->drvHandles[obj->remoteVirtId] = obj;

    /* plug the cpu interrupt */
    NotifySetup_plugHwi(params->remoteProcId, params->intVectorId,
            NotifyDriverMbx_isr);

    /* enable the mailbox interrupt from the remote core */
    NotifyDriverMbx_enable(obj);

    /* Restore global interrupts */
    Hwi_restore(key);
}

/*
 *  ======== NotifyDriverMbx_Instance_finalize ========
 */
Void NotifyDriverMbx_Instance_finalize(NotifyDriverMbx_Object *obj)
{

    /* disable the mailbox interrupt source */
    NotifyDriverMbx_disable(obj);

    /* unplug isr and unprogram the event dispatcher */
    NotifySetup_unplugHwi(obj->remoteProcId, obj->cpuIntrNum);

    /* must use processor virtual ID to remove driver handle from table */
    NotifyDriverMbx_module->drvHandles[obj->remoteVirtId] = NULL;
}

/*
 *  ======== NotifyDriverMbx_registerEvent ========
 */
Void NotifyDriverMbx_registerEvent(NotifyDriverMbx_Object *obj,
                                   UInt32 eventId)
{
    UInt hwiKey;

    /*
     *  Disable interrupt line to ensure that isr doesn't
     *  preempt registerEvent and encounter corrupt state
     */
    hwiKey = Hwi_disable();

    /* Set the 'registered' bit */
    SET_BIT(obj->evtRegMask, eventId);

    /* Restore the interrupt line */
    Hwi_restore(hwiKey);
}

/*
 *  ======== NotifyDriverMbx_unregisterEvent ========
 */
Void NotifyDriverMbx_unregisterEvent(NotifyDriverMbx_Object *obj,
                                     UInt32 eventId)
{
    UInt hwiKey;

    /*
     *  Disable interrupt line to ensure that isr doesn't
     *  preempt registerEvent and encounter corrupt state
     */
    hwiKey = Hwi_disable();

    /* Clear the registered bit */
    CLEAR_BIT(obj->evtRegMask, eventId);

    /* Restore the interrupt line */
    Hwi_restore(hwiKey);
}

/*
 *  ======== NotifyDriverMbx_sendEvent ========
 */
/*
 *  PUT_NOTIFICATION will spin waiting for enough room in the mailbox FIFO
 *  to store the number of messages needed for the notification ('numMsgs').
 *  If spinning is necesssary (i.e. if waitClear is TRUE and there isn't enough
 *  room in the FIFO) then PUT_NOTIFICATION will allow pre-emption while
 *  spinning.
 *
 *  PUT_NOTIFICATION needs to prevent another local thread from writing to the
 *  same mailbox after the current thread has
 *  1) determined that there is enough room to write the notification and
 *  2) written the first of two messages to the mailbox.
 *  This is needed to respectively prevent
 *  1) both threads from incorrectly assuming there is enough space in the FIFO
 *     for their own notifications
 *  2) the interrupting thread from writing a notification between two
 *     two messages that need to be successivly written by the preempted thread.
 *  Therefore, the check for enough FIFO room and one/both mailbox write(s)
 *  should all occur atomically (i.e. with interrupts disabled)
 */
#define PUT_NOTIFICATION(idx)                                               \
        key = Hwi_disable();                                                \
        while(MAILBOX_FIFOLENGTH - REG32(MAILBOX_STATUS(idx)) < numMsgs) {  \
            Hwi_restore(key);                                               \
            if (!waitClear) {                                               \
                return (Notify_E_FAIL);                                     \
            }                                                               \
            key = Hwi_disable();                                            \
        };                                                                  \
        REG32(MAILBOX_MESSAGE(idx)) = eventId + smallPayload;               \
        if (smallPayload == 0xFFFFFFE0) {                                   \
            REG32(MAILBOX_MESSAGE(idx)) = payload;                          \
        }                                                                   \
        Hwi_restore(key);

Int NotifyDriverMbx_sendEvent(NotifyDriverMbx_Object *obj, UInt32 eventId,
        UInt32 payload, Bool waitClear)
{
    UInt16 selfVirtId = PROCID(MultiProc_self());
    UInt key, numMsgs;
    UInt32 smallPayload;
    UInt16 index;

    /* Decide if the payload is small enough to fit in the first mbx msg */
    if (payload < 0x7FFFFFF) {
        smallPayload = (payload << 5);
        numMsgs = 1;
    }
    else {
        smallPayload = 0xFFFFFFE0;
        numMsgs = 2;
    }

#if defined(xdc_target__isaCompatible_64)
    index = (selfVirtId * NotifyDriverMbx_NUM_CORES) + obj->remoteVirtId;
    PUT_NOTIFICATION(index);

#elif defined(xdc_target__isaCompatible_v7M)

#if 0
    if (!(BIOS_smpEnabled) && (Core_getId())) {
        if (remoteProcId == NotifyDriverMbx_dspProcId) {
            PUT_NOTIFICATION(VPSS_TO_DSP)
        }
        else if (remoteProcId == NotifyDriverMbx_hostProcId) {
            PUT_NOTIFICATION(VPSS_TO_HOST)
        }
        else {
            PUT_NOTIFICATION(VPSS_TO_VIDEO)
        }
    }
    else {
        if (remoteProcId == NotifyDriverMbx_dspProcId) {
            PUT_NOTIFICATION(VIDEO_TO_DSP)
        }
        else if (remoteProcId == NotifyDriverMbx_hostProcId) {
            PUT_NOTIFICATION(VIDEO_TO_HOST)
        }
        else {
            PUT_NOTIFICATION(VIDEO_TO_VPSS)
        }
    }

#endif

#else

#if 0
    if (remoteProcId == NotifyDriverMbx_dspProcId) {
        PUT_NOTIFICATION(HOST_TO_DSP)
    }
    else if (remoteProcId == NotifyDriverMbx_videoProcId) {
        PUT_NOTIFICATION(HOST_TO_VIDEO)
    }
    else {
        PUT_NOTIFICATION(HOST_TO_VPSS)
    }

#endif

#endif

    return (Notify_S_SUCCESS);
}

/*
 *  ======== NotifyDriverMbx_disable ========
 */
Void NotifyDriverMbx_disable(NotifyDriverMbx_Object *obj)
{
    UInt16 selfVirtId = PROCID(MultiProc_self());
    UInt16 index;

#if defined(xdc_target__isaCompatible_64)
    index = (obj->remoteVirtId * NotifyDriverMbx_NUM_CORES) + selfVirtId;
    REG32(MAILBOX_IRQENABLE_CLR(index)) = MAILBOX_REG_VAL(SUBMBX_IDX(index));

#elif defined(xdc_target__isaCompatible_v7M)

#if 0
    if (!(BIOS_smpEnabled) && (Core_getId())) {
        if (remoteProcId == NotifyDriverMbx_hostProcId) {
            REG32(MAILBOX_IRQENABLE_CLR_VPSS) = MAILBOX_REG_VAL(HOST_TO_VPSS);
        }
        else if (remoteProcId == NotifyDriverMbx_dspProcId) {
            REG32(MAILBOX_IRQENABLE_CLR_VPSS) = MAILBOX_REG_VAL(DSP_TO_VPSS);
        }
        else {
            REG32(MAILBOX_IRQENABLE_CLR_VPSS) = MAILBOX_REG_VAL(VIDEO_TO_VPSS);
        }
    }
    else {
        if (remoteProcId == NotifyDriverMbx_hostProcId) {
            REG32(MAILBOX_IRQENABLE_CLR_VIDEO) = MAILBOX_REG_VAL(HOST_TO_VIDEO);
        }
        else if (remoteProcId == NotifyDriverMbx_dspProcId) {
            REG32(MAILBOX_IRQENABLE_CLR_VIDEO) = MAILBOX_REG_VAL(DSP_TO_VIDEO);
        }
        else {
            REG32(MAILBOX_IRQENABLE_CLR_VIDEO) = MAILBOX_REG_VAL(VPSS_TO_VIDEO);
        }
    }
#endif

#else

#if 0
    if (remoteProcId == NotifyDriverMbx_dspProcId) {
        REG32(MAILBOX_IRQENABLE_CLR_HOST) = MAILBOX_REG_VAL(DSP_TO_HOST);
    }
    else if (remoteProcId == NotifyDriverMbx_videoProcId) {
        REG32(MAILBOX_IRQENABLE_CLR_HOST) = MAILBOX_REG_VAL(VIDEO_TO_HOST);
    }
    else {
        REG32(MAILBOX_IRQENABLE_CLR_HOST) = MAILBOX_REG_VAL(VPSS_TO_HOST);
    }
#endif

#endif
}

/*
 *  ======== NotifyDriverMbx_enable ========
 */
Void NotifyDriverMbx_enable(NotifyDriverMbx_Object *obj)
{
    UInt16 selfVirtId = PROCID(MultiProc_self());
    UInt16 index;

#if defined(xdc_target__isaCompatible_64)
    index = (obj->remoteVirtId * NotifyDriverMbx_NUM_CORES) + selfVirtId;
    REG32(MAILBOX_IRQENABLE_SET(index)) = MAILBOX_REG_VAL(SUBMBX_IDX(index));

#elif defined(xdc_target__isaCompatible_v7M)

#if 0
    if (!(BIOS_smpEnabled) && (Core_getId())) {
        if (remoteProcId == NotifyDriverMbx_hostProcId) {
            REG32(MAILBOX_IRQENABLE_SET_VPSS) = MAILBOX_REG_VAL(HOST_TO_VPSS);
        }
        else if (remoteProcId == NotifyDriverMbx_dspProcId) {
            REG32(MAILBOX_IRQENABLE_SET_VPSS) = MAILBOX_REG_VAL(DSP_TO_VPSS);
        }
        else {
            REG32(MAILBOX_IRQENABLE_SET_VPSS) = MAILBOX_REG_VAL(VIDEO_TO_VPSS);
        }
    }
    else {
        if (remoteProcId == NotifyDriverMbx_hostProcId) {
            REG32(MAILBOX_IRQENABLE_SET_VIDEO) = MAILBOX_REG_VAL(HOST_TO_VIDEO);
        }
        else if (remoteProcId == NotifyDriverMbx_dspProcId) {
            REG32(MAILBOX_IRQENABLE_SET_VIDEO) = MAILBOX_REG_VAL(DSP_TO_VIDEO);
        }
        else {
            REG32(MAILBOX_IRQENABLE_SET_VIDEO) = MAILBOX_REG_VAL(VPSS_TO_VIDEO);
        }
    }
#endif

#else

#if 0
    if (remoteProcId == NotifyDriverMbx_dspProcId) {
        REG32(MAILBOX_IRQENABLE_SET_HOST) = MAILBOX_REG_VAL(DSP_TO_HOST);
    }
    else if (remoteProcId == NotifyDriverMbx_videoProcId) {
        REG32(MAILBOX_IRQENABLE_SET_HOST) = MAILBOX_REG_VAL(VIDEO_TO_HOST);
    }
    else {
        REG32(MAILBOX_IRQENABLE_SET_HOST) = MAILBOX_REG_VAL(VPSS_TO_HOST);
    }
#endif

#endif
}

/*
 *  ======== NotifyDriverMbx_disableEvent ========
 */
Void NotifyDriverMbx_disableEvent(NotifyDriverMbx_Object *obj, UInt32 eventId)
{
    /* NotifyDriverMbx_disableEvent not supported by this driver */
    Assert_isTrue(FALSE, NotifyDriverMbx_A_notSupported);
}

/*
 *  ======== NotifyDriverMbx_enableEvent ========
 */
Void NotifyDriverMbx_enableEvent(NotifyDriverMbx_Object *obj, UInt32 eventId)
{
    /* NotifyDriverMbx_enableEvent not supported by this driver */
    Assert_isTrue(FALSE, NotifyDriverMbx_A_notSupported);
}

/*
 *************************************************************************
 *                       Internal functions
 *************************************************************************
 */

/*
 *  ======== NotifyDriverMbx_isr ========
 */

/*  Read a message from the mailbox. The low 5 bits of the message
 *  contains the eventId. The high 27 bits of the message contains
 *  either:
 *      1) The payload if the payload is less than 0x7FFFFFF
 *      2) 0x7FFFFFF otherwise
 *  If the high 27 bits of the first message is 0x7FFFFFF, then the
 *  payload is in the next mailbox message.
 *
 *  idx = mailbox table index
 */
#define MESSAGE_DELIVERY(idx)                                               \
    msg = REG32(MAILBOX_MESSAGE(idx));                                      \
    eventId = (UInt16)(msg & 0x1F);                                         \
    payload = msg >> 5;                                                     \
    if (payload == 0x7FFFFFF) {                                             \
        while(REG32(MAILBOX_STATUS(idx)) == 0);                             \
        payload = REG32(MAILBOX_MESSAGE(idx));                              \
    }                                                                       \
    REG32(MAILBOX_IRQSTATUS_CLR(idx)) = MAILBOX_REG_VAL(SUBMBX_IDX(idx));   \
    obj = NotifyDriverMbx_module->drvHandles[srcVirtId];                    \
    Assert_isTrue(obj != NULL, ti_sdo_ipc_Notify_A_internal);               \
    if (TEST_BIT(obj->evtRegMask, eventId)) {                               \
        ti_sdo_ipc_Notify_exec(obj->notifyHandle, eventId, payload);        \
    }                                                                       \
    REG32(MAILBOX_EOI_REG(idx)) = 0x1;

Void NotifyDriverMbx_isr(UInt16 idx)
{
    NotifyDriverMbx_Object *obj;
    UInt32 msg, payload;
    UInt16 eventId;

#if defined(xdc_target__isaCompatible_64)
    UInt16 srcVirtId;

    srcVirtId = idx / NotifyDriverMbx_NUM_CORES;
    MESSAGE_DELIVERY(idx)

#elif defined(xdc_target__isaCompatible_v7M)

#if 0
    do {
        numProcessed = 0;
        if (!(BIOS_smpEnabled) && (Core_getId())) {
            GET_NOTIFICATION(VPSS, HOST)
            GET_NOTIFICATION(VPSS, DSP)
            GET_NOTIFICATION(VPSS, VIDEO)
        }
        else {
            GET_NOTIFICATION(VIDEO, HOST)
            GET_NOTIFICATION(VIDEO, DSP)
            GET_NOTIFICATION(VIDEO, VPSS)
        }
    }
    while (numProcessed != 0);
#endif

#else

#if 0
    do {
        numProcessed = 0;
        GET_NOTIFICATION(HOST, DSP)
        GET_NOTIFICATION(HOST, VPSS)
        GET_NOTIFICATION(HOST, VIDEO)
    }
    while (numProcessed != 0);
#endif

#endif
}

/*
 *  ======== NotifyDriverMbx_setNotifyHandle ========
 */
Void NotifyDriverMbx_setNotifyHandle(NotifyDriverMbx_Object *obj,
        Ptr notifyHandle)
{
    /* internally used, so no assert needed */
    obj->notifyHandle = (ti_sdo_ipc_Notify_Handle)notifyHandle;
}
