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
 *  ======== InterruptR5f.xdc ========
 *
 */

import ti.sdo.utils.MultiProc;

/*!
 *  ======== InterruptR5f ========
 *  R5F interrupt manager
 */
module InterruptR5f inherits ti.sdo.ipc.notifyDrivers.IInterrupt
{
    /*!
     *  Maximum number of cores
     *
     *  @_nodoc
     */
    const UInt8 NUM_CORES = 3;

    /*!
     *  Maximum number of IPU cores
     *
     *  @_nodoc
     */
    const UInt8 NUM_R5f_CORES = 2;


    /*!
     *  Number of System mailboxes used by IPC
     *
     *  This represents the number of System mailboxes used by IPC.  IPC
     *  currently uses system mailbox 0
     */
    const UInt8 NUM_SYS_MBX = 3;

    /*!
     *  Base address for the mailbox subsystems
     *
     *  The `mailboxBaseAddr` array indicates the virtual addresses through
     *  which IPC will access various mailboxes.  The specific mailbox addresses
     *  each array index maps to follows:
     *  @p(blist)
     *    - 0  - System Mailbox 0
     *  @p
     *
     *  Note that these mailboxes are not accessible at their physical
     *  addresses (in the 0x4XXX_XXXX range).  So default virtual addresses
     *  through which these mailboxes will be accessed are assigned in the
     *  0x6XXX_XXXX range.  Users must ensure these virtual addresses are
     *  correctly mapped to the 0x4XXX_XXXX-based phys addrs in each IPUs AMMU.
     */
    config UInt32 mailboxBaseAddr[NUM_SYS_MBX];

    /*!
     * Mailbox table for storing encoded Base Address, mailbox user Id,
     * and sub-mailbox index.
     *
     *  @_nodoc
     */
    config UInt32 mailboxTable[NUM_CORES * NUM_CORES];

    /*!
     *  Base address for the R5F CTRL register
     */
    config UInt32 r5fCtrlBaseAddr = 0x40001000;

    /*!
     *  Processor Id table
     *
     *  @_nodoc
     */
    config UInt32 procIdTable[NUM_CORES];

internal:

    /*! Statically retrieve procIds to avoid doing this at runtime */
    config UInt r5f_0ProcId   = MultiProc.INVALIDID;
    config UInt hostProcId     = MultiProc.INVALIDID;
    config UInt r5f_1ProcId   = MultiProc.INVALIDID;

    /*! Function table */
    struct FxnTable {
        Fxn    func;
        UArg   arg;
    }

    /*! Stub to be plugged for inter R5F core interrupts */
    Void intShmR5fStub(UArg arg);

    /*! Stub to be plugged for intra R5F core interrupts */
    Void intShmMbxStub(UInt16 idx);

    struct Module_State {
        /*
         * Create a function table of length 8 (Total number of cores in the
         * System) for each M4 core.
         */
        FxnTable   fxnTable[NUM_CORES];
    };
}
