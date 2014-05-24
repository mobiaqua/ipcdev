/*
 * Copyright (c) 2012-2014 Texas Instruments Incorporated - http://www.ti.com
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
 *  ======== NotifySetup.xdc ========
 */
package ti.sdo.ipc.family.vayu;

import xdc.runtime.Assert;
import ti.sdo.utils.MultiProc;

/*!
 *  ======== NotifySetup ========
 *  Notify setup proxy for Vayu
 *
 *  This module creates and registers all drivers necessary for
 *  inter-processor notification on Vayu.
 */
@ModuleStartup
@Template("./NotifySetup.xdt")

module NotifySetup inherits ti.sdo.ipc.interfaces.INotifySetup
{
    /*
     *  ======== DriverIsr ========
     *  Notify driver isr function type definition
     *  param1 = mailbox table index
     */
    typedef Void (*DriverIsr)(UInt16);

    /*!
     *  ======== A_internal ========
     *  Internal implementation error.
     */
    config Assert.Id A_internal = {
        msg: "A_internal: internal implementation error"
    };

    /*!
     *  Interrupt vector id for Vayu/DSP.
     */
    config UInt dspIntVectId = 4;

    /*!
     *  Interrupt vector id for Vayu/EVE
     */
    config UInt eveIntVectId_INTC0 = 4;
    config UInt eveIntVectId_INTC1 = 8;

    /*!
     *  Available notify drivers.
     */
    enum Driver {
        Driver_SHAREDMEMORY = 0x01,     /*! shared memory */
        Driver_MAILBOX = 0x02           /*! hardware mailbox */
    };

    /*!
     *  Notify driver connection specification.
     */
    struct Connection {
        Driver driver;                  /*! notify driver */
        String procName;                /*! remote processor name */
    };

    /*!
     *  Specify notify driver for given processor connections.
     */
    metaonly config Connection connections[length];

    /*!
     *  ======== plugHwi ========
     *  Register an isr for the given interrupt and event.
     *
     *  @param(remoteProcId) The MutiProc Id of the remote processor
     *  which will raise the given interrupt.
     *
     *  @param(cpuIntrNum) The interrupt number which will be raised
     *  by the remote processor.
     *
     *  @param(isr) The ISR which should be invoked to service the
     *  given interrupt.
     */
    Void plugHwi(UInt16 remoteProcId, Int cpuIntrNum, DriverIsr isr);

    /*!
     *  ======== unplugHwi ========
     *  Unregister the isr for the given interrupt.
     */
    Void unplugHwi(UInt16 remoteProcId, Int cpuIntrNum);

    /*! @_nodoc
     *  ======== interruptTable ========
     *  Accessor method to return interrupt id for given virtual proc id
     */
    UInt16 interruptTable(Int srcVirtId);

internal:
    /* total number of cores on Vayu SoC */
    const UInt8 NUM_CORES = 11;

    /* number of cores in eve subsystem */
    const UInt8 NUM_EVES = 4;

    /* number of internal eve mailboxes */
    const UInt8 NUM_EVE_MBX = 12;

    /* number of system mailboxes (used by IPC) */
    const UInt8 NUM_SYS_MBX = 4;

    /*  Mailbox table for storing encoded base address, mailbox user ID,
     *  and sub-mailbox index.
     */
    config UInt32 mailboxTable[NUM_CORES * NUM_CORES];

    /* base address table for the mailbox subsystem */
    config UInt32 mailboxBaseAddr[NUM_EVE_MBX + NUM_SYS_MBX];

    /* map procId to discrete processor/core */
    config UInt eve1ProcId = MultiProc.INVALIDID;
    config UInt eve2ProcId = MultiProc.INVALIDID;
    config UInt eve3ProcId = MultiProc.INVALIDID;
    config UInt eve4ProcId = MultiProc.INVALIDID;
    config UInt dsp1ProcId = MultiProc.INVALIDID;
    config UInt dsp2ProcId = MultiProc.INVALIDID;
    config UInt ipu1_0ProcId = MultiProc.INVALIDID;  /* also used for ipu1 */
    config UInt ipu1_1ProcId = MultiProc.INVALIDID;
    config UInt ipu2_0ProcId = MultiProc.INVALIDID;  /* also used for ipu2 */
    config UInt ipu2_1ProcId = MultiProc.INVALIDID;
    config UInt hostProcId = MultiProc.INVALIDID;

    /* map MultiProc ID to virtual ID, virtId = procIdTable[procId] */
    config UInt32 procIdTable[NUM_CORES];

    /*
     *  ======== driverType ========
     */
    Driver driverType(UInt16 remoteProcId);

    /*
     *  ======== Shm_attach ========
     */
    Int Shm_attach(UInt16 remoteProcId, Ptr sharedAddr);

    /*
     *  ======== Shm_sharedMemReq ========
     */
    SizeT Shm_sharedMemReq(UInt16 remoteProcId, Ptr sharedAddr);

    /*
     *  ======== Mbx_attach ========
     */
    Int Mbx_attach(UInt16 remoteProcId, Ptr sharedAddr);

    /*
     *  ======== Mbx_sharedMemReq ========
     */
    SizeT Mbx_sharedMemReq(UInt16 remoteProcId, Ptr sharedAddr);

    /*
     *  ======== dispatchIsr ========
     *  Dispatch interrupt to notify driver instance.
     */
    Void dispatchIsr(UArg arg);

    /*
     *  ======== DrvBind ========
     */
    struct DrvBind {
        Driver driver;                  /*! notify driver */
        UInt16 procId;                  /*! remote processor ID */
    };

    /*
     *  ======== Module_State ========
     */
    struct Module_State {
        /* interrupt plug counter */
        UInt16 numPlugged[];

        /* connection array */
        DrvBind connAry[length];

        /*  Interrupt event IDs used to communicate with this processor.
         *  This table is indexed by virtual processor ID.
         */
        UInt16 interruptTable[NUM_CORES];

        /*  Notify driver isr dispatch table. This table is indexed
         *  by virtual processor ID.
         */
        DriverIsr isrDispatchTable[NUM_CORES];
    };
}
