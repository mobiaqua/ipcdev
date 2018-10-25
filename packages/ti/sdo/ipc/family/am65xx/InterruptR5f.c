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
 *  ======== InterruptR5f.c ========
 *  AM65XX R5F based interupt manager
 */

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/family/arm/v7r/keystone3/Core.h>
#include <ti/sysbios/family/arm/v7r/keystone3/Hwi.h>

#include <ti/sdo/ipc/family/am65xx/NotifySetup.h>

#include <ti/sdo/ipc/notifyDrivers/IInterrupt.h>
#include <ti/sdo/utils/_MultiProc.h>

#include "package/internal/InterruptR5f.xdc.h"

/* Register access method. */
#define REG16(A)   (*(volatile UInt16 *) ((uintptr_t)A))
#define REG32(A)   (*(volatile UInt32 *) ((uintptr_t)A))

#define PROCID(IDX)               (InterruptR5f_procIdTable[(IDX)])
#define MBX_TABLE_IDX(SRC, DST)   ((PROCID(SRC) * InterruptR5f_NUM_CORES) + \
                                    PROCID(DST))
#define SUBMBX_IDX(IDX)           (InterruptR5f_mailboxTable[(IDX)] & 0xFF)
#define MBX_USER_IDX(IDX)         ((InterruptR5f_mailboxTable[(IDX)] >> 8) \
                                    & 0xFF)
#define MBX_BASEADDR_IDX(IDX)    ((InterruptR5f_mailboxTable[(IDX)] >> 16) \
                                    & 0xFFFF)

#define MAILBOX_REG_VAL(M)        (0x1 << (2 * M))

#define MAILBOX_MESSAGE(IDX)      (InterruptR5f_mailboxBaseAddr[  \
                                    MBX_BASEADDR_IDX(IDX)] + 0x40 +   \
                                    (0x4 * SUBMBX_IDX(IDX)))
#define MAILBOX_STATUS(IDX)       (InterruptR5f_mailboxBaseAddr[  \
                                    MBX_BASEADDR_IDX(IDX)] + 0xC0 +   \
                                    (0x4 * SUBMBX_IDX(IDX)))

#define MAILBOX_IRQSTATUS_CLR(IDX)   (InterruptR5f_mailboxBaseAddr[  \
                                        MBX_BASEADDR_IDX(IDX)] + (0x10 * \
                                        MBX_USER_IDX(IDX)) + 0x104)
#define MAILBOX_IRQENABLE_SET(IDX)   (InterruptR5f_mailboxBaseAddr[  \
                                        MBX_BASEADDR_IDX(IDX)] + (0x10 * \
                                        MBX_USER_IDX(IDX)) + 0x108)
#define MAILBOX_IRQENABLE_CLR(IDX)   (InterruptR5f_mailboxBaseAddr[  \
                                        MBX_BASEADDR_IDX(IDX)] + (0x10 * \
                                        MBX_USER_IDX(IDX)) + 0x10C)
#define MAILBOX_EOI_REG(IDX)         (InterruptR5f_mailboxBaseAddr[  \
                                        MBX_BASEADDR_IDX(IDX)] + 0x140)

#define WUGENIPU        19

/*
 *************************************************************************
 *                      Module functions
 *************************************************************************
 */

/*
 *  ======== InterruptR5f_intEnable ========
 *  Enable remote processor interrupt
 */
Void InterruptR5f_intEnable(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
    UInt16 index;
    Bool useMailbox = TRUE;
    UInt subMbxIdx;

    index = MBX_TABLE_IDX(remoteProcId, MultiProc_self());

    /*  If the remote processor communicates via mailboxes, we should enable
     *  the Mailbox IRQ instead of enabling the Hwi because multiple mailboxes
     *  share the same Hwi
     */
    if (useMailbox) {
        subMbxIdx = SUBMBX_IDX(index);
        REG32(MAILBOX_IRQENABLE_SET(index)) = MAILBOX_REG_VAL(subMbxIdx);
    }
}

/*
 *  ======== InterruptR5f_intDisable ========
 *  Disables remote processor interrupt
 */
Void InterruptR5f_intDisable(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
    UInt16 index;
    Bool useMailbox = TRUE;
    UInt subMbxIdx;

    index = MBX_TABLE_IDX(remoteProcId, MultiProc_self());

    /*  If the remote processor communicates via mailboxes, we should disable
     *  the Mailbox IRQ instead of disabling the Hwi because multiple mailboxes
     *  share the same Hwi
     */
    if (useMailbox) {
        subMbxIdx = SUBMBX_IDX(index);
        REG32(MAILBOX_IRQENABLE_CLR(index)) = MAILBOX_REG_VAL(subMbxIdx);
    }
}

/*
 *  ======== InterruptR5f_intRegister ========
 */
Void InterruptR5f_intRegister(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo,
        Fxn func, UArg arg)
{
    Hwi_Params  hwiAttrs;
    UInt        key;
    Int         index;
    InterruptR5f_FxnTable *table;

    Assert_isTrue(remoteProcId < ti_sdo_utils_MultiProc_numProcessors,
            ti_sdo_utils_MultiProc_A_invalidMultiProcId);

    /* index is the virtual id (invariant) */
    index = PROCID(remoteProcId);

    intInfo->localIntId = NotifySetup_interruptTable(index);

    /* Disable global interrupts */
    key = Hwi_disable();

    /* store callback function by virtual id */
    table = &(InterruptR5f_module->fxnTable[index]);
    table->func = func;
    table->arg  = arg;

    InterruptR5f_intClear(remoteProcId, intInfo);

    Hwi_Params_init(&hwiAttrs);
    hwiAttrs.maskSetting = Hwi_MaskingOption_LOWER;

    /* plug the cpu interrupt with notify setup dispatch isr */
    NotifySetup_plugHwi(remoteProcId, intInfo->localIntId,
        InterruptR5f_intShmMbxStub);

    InterruptR5f_intEnable(remoteProcId, intInfo);

    /* Restore global interrupts */
    Hwi_restore(key);
}

/*
 *  ======== InterruptR5f_intUnregister ========
 */
Void InterruptR5f_intUnregister(UInt16 remoteProcId,
        IInterrupt_IntInfo *intInfo)
{
    Int index;
    Hwi_Handle hwiHandle;
    InterruptR5f_FxnTable *table;

    /* Disable the mailbox interrupt source */
    InterruptR5f_intDisable(remoteProcId, intInfo);

    if ((remoteProcId == InterruptR5f_r5f_0ProcId) ||
        (remoteProcId == InterruptR5f_r5f_1ProcId)) {
        hwiHandle = Hwi_getHandle(WUGENIPU);
        Hwi_delete(&hwiHandle);
    }
    else {
        NotifySetup_unplugHwi(remoteProcId, intInfo->localIntId);
    }

    /* index is the virtual id (invariant) */
    index = PROCID(remoteProcId);

    /* Clear the FxnTable entry for the remote processor */
    table = &(InterruptR5f_module->fxnTable[index]);
    table->func = NULL;
    table->arg  = 0;
}


/*
 *  ======== InterruptR5f_intSend ========
 *  Send interrupt to the remote processor
 */
Void InterruptR5f_intSend(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo,
        UArg arg)
{
    UInt key;
    UInt16 index;

    index = MBX_TABLE_IDX(MultiProc_self(), remoteProcId);
    key = Hwi_disable();
    while (REG32(MAILBOX_STATUS(index)) != 0) {
        Hwi_restore(key);
        key = Hwi_disable();
    }
    REG32(MAILBOX_MESSAGE(index)) = arg;
    Hwi_restore(key);

}


/*
 *  ======== InterruptR5f_intPost ========
 *  Simulate an interrupt from a remote processor
 */
Void InterruptR5f_intPost(UInt16 srcProcId, IInterrupt_IntInfo *intInfo,
                             UArg arg)
{
    UInt key;
    UInt16 index;

    index = MBX_TABLE_IDX(srcProcId, MultiProc_self());
    key = Hwi_disable();
    if (REG32(MAILBOX_STATUS(index)) == 0) {
        REG32(MAILBOX_MESSAGE(index)) = arg;
    }
    Hwi_restore(key);

}

/*
 *  ======== InterruptR5f_intClear ========
 *  Clear interrupt
 */
UInt InterruptR5f_intClear(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
    UInt arg;
    UInt16 index;

    index = MBX_TABLE_IDX(remoteProcId, MultiProc_self());
    arg = REG32(MAILBOX_MESSAGE(index));
    REG32(MAILBOX_IRQSTATUS_CLR(index)) =
        MAILBOX_REG_VAL(SUBMBX_IDX(index));

    return (arg);
}

/*
 *************************************************************************
 *                      Internals functions
 *************************************************************************
 */

/*
 *  ======== InterruptR5f_intShmMbxStub ========
 */
Void InterruptR5f_intShmMbxStub(UInt16 idx)
{
    UInt16 srcVirtId;
    InterruptR5f_FxnTable *table;

    srcVirtId = idx / InterruptR5f_NUM_CORES;
    table = &(InterruptR5f_module->fxnTable[srcVirtId]);
    (table->func)(table->arg);
}
