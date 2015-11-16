/*
 * Copyright (c) 2014-2015 Texas Instruments Incorporated - http://www.ti.com
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== Interrupt.c ========
 */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Startup.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/hal/Hwi.h>

#include <ti/sdo/ipc/family/tci663x/MultiProcSetup.h>
#include <ti/sdo/ipc/notifyDrivers/IInterrupt.h>
#include <ti/sdo/utils/_MultiProc.h>

#include "package/internal/Interrupt.xdc.h"

#define ARM_SOURCE_OFFSET 31
#define ARM_HWI_OFFSET 32

#ifdef _TMS320C6X
    extern cregister volatile UInt DNUM;
#else
    /* ARM does not support DNUM */
#endif

/*
 *************************************************************************
 *                      Module functions
 *************************************************************************
 */

/*
 *  ======== Interrupt_Module_startup ========
 */
Int Interrupt_Module_startup(Int phase)
{
    UInt16 hostId;
    String name;
    UInt nameId;

    /* wait for Startup (because user function might set local procId) */
    if (!Startup_Module_startupDone()) {
        return (Startup_NOTDONE);
    }
    else if (MultiProc_self() == MultiProc_INVALIDID) {
        /* if user function is missing, this will eventually fail */
        return (Startup_NOTDONE);
    }

    if (!ti_sdo_utils_MultiProc_Module_startupDone()) {
        return (Startup_NOTDONE);
    }

    if (Interrupt_module->baseId == MultiProc_INVALIDID) {
        Interrupt_module->baseId = MultiProc_getBaseIdOfCluster();
    }
    Assert_isTrue(Interrupt_module->baseId != MultiProc_INVALIDID,
            Interrupt_A_clusterBaseId);

    /*  If this assert fails, then MultiProc config has changed to break
     *  an assumption in Linux rpmsg driver, that HOST is listed first in
     *  MultiProc name list configuration.
     */
    if ((hostId = MultiProc_getId("HOST")) != MultiProc_INVALIDID) {
        Assert_isTrue((hostId - Interrupt_module->baseId) == 0,
                Interrupt_A_hostConfig);
    }

#ifdef _TMS320C6X
    /*  Validate the running executable has been loaded onto the correct
     *  processor. In other words, make sure CORE0 was loaded onto DSP0
     *  (i.e. DNUM == 0), CORE1 loaded onto DSP1, etc.
     */
    name = MultiProc_getName(MultiProc_self());
    nameId = (UInt)(name[4] - '0');

    if (nameId != DNUM) {
        System_abort("incorrect executable loaded onto processor");
    }
#else
    /*  Host doesn't necessarily follow the same naming conventions
    */
#endif

    if (!Interrupt_enableKick) {
        /* do not unlock the kick registers */
        return (Startup_DONE);
    }

#ifdef _TMS320C6X
    /* only write KICK registers from CORE0 */
    if (DNUM == 0) {
        /* TODO: What if CORE0 is not started, but the others are? */
        if (Interrupt_KICK0 && Interrupt_KICK1) {
            volatile UInt32 *kick0 = (volatile UInt32 *)Interrupt_KICK0;
            volatile UInt32 *kick1 = (volatile UInt32 *)Interrupt_KICK1;

            /* unlock the KICK mechanism in the Bootcfg MMRs if defined */
            *kick0 = 0x83e70b13;        /* must be written with this value */
            *kick1 = 0x95a4f1e0;        /* must be written with this value */
        }
    }
#else
    /*  reserved for RTOS HOST
    */
#endif
    return (Startup_DONE);
}

/*
 *  ======== Interrupt_intEnable ========
 *  Enable interrupt
 *  TODO: fix this with interrupt mask
 */
Void Interrupt_intEnable(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
    Hwi_enableInterrupt(intInfo->intVectorId);
}

/*
 *  ======== Interrupt_intDisable ========
 *  Disables interrupts
 *  TODO: fix this with interrupt mask
 */
Void Interrupt_intDisable(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
    Hwi_disableInterrupt(intInfo->intVectorId);
}

/*
 *  ======== Interrupt_intRegister ========
 *  Register ISR for remote processor interrupt
 */
Void Interrupt_intRegister(UInt16 remoteProcId, IInterrupt_IntInfo *unused,
        Fxn func, UArg arg)
{
    UInt key;
    Hwi_Params hwiAttrs;
    UInt16 clusterId;
#ifdef _TMS320C6X
    volatile UInt32 *ipcar = (volatile UInt32 *)Interrupt_IPCAR0;
#else
    volatile UInt32 *ipcarh = (volatile UInt32 *)Interrupt_IPCARH;
#endif
    UInt32 val;

    /* disable global interrupts */
    key = Hwi_disable();

    /* setup the function table with client function and argument */
    clusterId = remoteProcId - Interrupt_module->baseId;
    Interrupt_module->clientTab[clusterId].func = func;
    Interrupt_module->clientTab[clusterId].arg = arg;

    /* make sure the interrupt gets plugged only once */
    if (Interrupt_module->numPlugged++ == 0) {

#ifdef _TMS320C6X
        /* clear all pending ipc interrupts */
        val = ipcar[DNUM];
        ipcar[DNUM] = val;
#else
        /* Verify that this works for ARM */
        val = *ipcarh;
        *ipcarh = val;
#endif

        /* register ipc interrupt */
        Hwi_Params_init(&hwiAttrs);
        hwiAttrs.maskSetting = Hwi_MaskingOption_SELF;
#ifdef _TMS320C6X
        hwiAttrs.eventId = Interrupt_INTERDSPINT,
        Interrupt_module->hwi = Hwi_create(Interrupt_ipcIntr, Interrupt_isr,
                &hwiAttrs, NULL);
#else
        Interrupt_module->hwi =
            Hwi_create(Interrupt_INTERDSPINT + ARM_HWI_OFFSET,
            Interrupt_isr, &hwiAttrs, NULL);
#endif
    }

    /* restore global interrupts */
    Hwi_restore(key);
}

/*
 *  ======== Interrupt_intUnregister ========
 */
Void Interrupt_intUnregister(UInt16 remoteProcId, IInterrupt_IntInfo *intInfo)
{
    UInt key;
    UInt16 clusterId;

    /* disable global interrupts */
    key = Hwi_disable();

    if (--Interrupt_module->numPlugged == 0) {
        Hwi_delete(&Interrupt_module->hwi);
    }

    /* clear function table entry */
    clusterId = remoteProcId - Interrupt_module->baseId;
    Interrupt_module->clientTab[clusterId].func = NULL;
    Interrupt_module->clientTab[clusterId].arg = (UArg)(-1);

    /* restore global interrupts */
    Hwi_restore(key);
}

/*
 *  ======== Interrupt_intSend ========
 *  Send interrupt to the remote processor
 */
Void Interrupt_intSend(UInt16 procId, IInterrupt_IntInfo *unused, UArg arg)
{
    UInt32 val;
    volatile UInt32 *ipcgr = (volatile UInt32 *)Interrupt_IPCGR0;
    volatile UInt32 *ipcgrh = (volatile UInt32 *)Interrupt_IPCGRH;
    int clusterId;
    UInt dnum;

#ifdef _TMS320C6X
    /*  bit 0 is set to generate interrupt.
     *  bits 4-7 is set to specify the interrupt generation source.
     *  The convention is that bit 4 (SRCS0) is used for core 0,
     *  bit 5 (SRCS1) for core 1, etc... .
     */
    val = (1 << (DNUM + Interrupt_SRCSx_SHIFT)) | 1;
#else
    /*  Host sets the first bit instead of using DNUM
    */
    val = (1 << ARM_SOURCE_OFFSET) | 1;
#endif

    if (procId == MultiProc_getId("HOST")) {
        /* interrupt the host processor,  use IPCGRH register */
        *ipcgrh = val;
    }
    else {
        /* compute ipcgr address for recipient processor */
        clusterId = procId - Interrupt_module->baseId;
        dnum = Interrupt_module->hwTab[clusterId].dnum;
        ipcgr[dnum] = val;
    }
}

/*
 *  ======== Interrupt_intPost ========
 *  Post an interrupt to local processor
 */
Void Interrupt_intPost(UInt16 srcProcId, IInterrupt_IntInfo *intInfo, UArg arg)
{
    int clusterId;
    int bit;
    UInt32 val;
#ifdef _TMS320C6X
    volatile UInt32 *ipcgr = (volatile UInt32 *)Interrupt_IPCGR0;
#else
    volatile UInt32 *ipcgrh = (volatile UInt32 *)Interrupt_IPCGRH;
#endif

    /* compute srcsx bit of source processor */
    clusterId = srcProcId - Interrupt_module->baseId;
    bit = Interrupt_module->hwTab[clusterId].srcsx;
    val = (1 << bit) | 1;

#ifdef _TMS320C6X
    /* raise the interrupt to myself */
    ipcgr[DNUM] = val;
#else
    *ipcgrh = val;
#endif
}

/*
 *  ======== Interrupt_intClear ========
 *  Acknowledge interrupt by clearing the corresponding source bit.
 *  Does not clear the IFR bit by way of ICR write because that should
 *  only be done during init time.
 */
UInt Interrupt_intClear(UInt16 remoteProcId, IInterrupt_IntInfo *unused)
{
    int clusterId;
    int pos;
#ifdef _TMS320C6X
    volatile UInt32 *ipcar = (volatile UInt32 *)Interrupt_IPCAR0;
#else
    volatile UInt32 *ipcarh = (volatile UInt32 *)Interrupt_IPCARH;
#endif
    UInt val;
    UInt stat = 0;

    /* compute srcsx bit of remote processor */
    clusterId = remoteProcId - Interrupt_module->baseId;
    pos = Interrupt_module->hwTab[clusterId].srcsx;

#ifdef _TMS320C6X
    /* read ipcar register to get source bits */
    val = ipcar[DNUM];
#else
    val = *ipcarh;
#endif

    if (val & (1 << pos)) {
#ifdef _TMS320C6X
        /* write ipc acknowledgement register to clear source bit */
        ipcar[DNUM] = (1 << pos);
#else
        *ipcarh = (1 << pos);
#endif
        stat = 1;
    }

    return (stat);
}

/*
 *************************************************************************
 *                      Internals functions
 *************************************************************************
 */

/*
 *  ======== Interrupt_isr ========
 */
Void Interrupt_isr(UArg unused)
{
    int clId;
    Interrupt_ClientEntry *entry;
#ifdef _TMS320C6X
    volatile UInt32 *ipcar = (volatile UInt32 *)Interrupt_IPCAR0;
#else
    volatile UInt32 *ipcarh = (volatile UInt32 *)Interrupt_IPCARH;
#endif
    UInt32 val;
    int bit;

#ifdef _TMS320C6X
    /* ipc acknowledgement register value */
    val = ipcar[DNUM];
#else
    val = *ipcarh;
#endif

    for (clId = 0; clId < ti_sdo_utils_MultiProc_numProcsInCluster; clId++) {
        bit = Interrupt_module->hwTab[clId].srcsx;

        if (val & (1 << bit)) {

#ifdef _TMS320C6X
            /* clear the interrupt source */
            ipcar[DNUM] = (1 << bit);
#else
            *ipcarh = (1 << bit);
#endif

            /* invoke the client isr */
            entry = &(Interrupt_module->clientTab[clId]);

            if (entry->func != NULL) {
                (entry->func)(entry->arg);
            }
        }
    }
}
