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
 *  ======== NotifySetup.xs ========
 *
 */
var MultiProc = null;

/*
 *  ======== module$use ========
 */
function module$use()
{
    var TableInit = xdc.useModule("ti.sdo.ipc.family.vayu.TableInit");

    /* load modules needed in meta domain and in target domain */
    MultiProc = xdc.useModule('ti.sdo.utils.MultiProc');

    xdc.useModule('xdc.runtime.Assert');

    /* initialize procIdTable */
    TableInit.initProcId(this);

    /* initialize mailboxTable */
    TableInit.generateTable(this);

    /* Initialize mailbox base address table */
    this.mailboxBaseAddr[0]  = 0x4208B000;  /* EVE1 Internal Mailbox 0 */
    this.mailboxBaseAddr[1]  = 0x4208C000;  /* EVE1 Internal Mailbox 1 */
    this.mailboxBaseAddr[2]  = 0x4208D000;  /* EVE1 Internal Mailbox 2 */
    this.mailboxBaseAddr[3]  = 0x4218B000;  /* EVE2 Internal Mailbox 0 */
    this.mailboxBaseAddr[4]  = 0x4218C000;  /* EVE2 Internal Mailbox 1 */
    this.mailboxBaseAddr[5]  = 0x4218D000;  /* EVE2 Internal Mailbox 2 */
    this.mailboxBaseAddr[6]  = 0x4228B000;  /* EVE3 Internal Mailbox 0 */
    this.mailboxBaseAddr[7]  = 0x4228C000;  /* EVE3 Internal Mailbox 1 */
    this.mailboxBaseAddr[8]  = 0x4228D000;  /* EVE3 Internal Mailbox 2 */
    this.mailboxBaseAddr[9]  = 0x4238B000;  /* EVE4 Internal Mailbox 0 */
    this.mailboxBaseAddr[10] = 0x4238C000;  /* EVE4 Internal Mailbox 1 */
    this.mailboxBaseAddr[11] = 0x4238D000;  /* EVE4 Internal Mailbox 2 */
    this.mailboxBaseAddr[12] = 0x48840000;  /* System Mailbox 5 */
    this.mailboxBaseAddr[13] = 0x48842000;  /* System Mailbox 6 */
    this.mailboxBaseAddr[14] = 0x48844000;  /* System Mailbox 7 */
    this.mailboxBaseAddr[15] = 0x48846000;  /* System Mailbox 8 */

    /* determine which notify drivers to include */
    this.$private.driverMask = 0;

    /* for unspecfied connections, the default is shared memory */
    if (this.connections.length < (MultiProc.numProcessors - 1)) {
        this.$private.driverMask |= this.Driver_SHAREDMEMORY;
    }

    /* remember which notify drivers have been specified */
    for (var i = 0; i < this.connections.length; i++) {
        if (this.connections[i].driver == this.Driver_SHAREDMEMORY) {
            this.$private.driverMask |= this.Driver_SHAREDMEMORY;
        }
        if (this.connections[i].driver == this.Driver_MAILBOX) {
            this.$private.driverMask |= this.Driver_MAILBOX;
        }
    }

    /* load notify drivers into configuration model */
    if (this.$private.driverMask & this.Driver_SHAREDMEMORY) {
        xdc.useModule('ti.sdo.ipc.notifyDrivers.NotifyDriverShm');
    }
    if (this.$private.driverMask & this.Driver_MAILBOX) {
        xdc.useModule('ti.sdo.ipc.family.vayu.NotifyDriverMbx');
    }
}

/*
 *  ======== module$static$init ========
 *  Initialize the target state object.
 */
function module$static$init(state, mod)
{
    var procId;

    state.numPlugged = 0;

    /* Initialize the state connAry from the config params. Translate
     * processor names into IDs for better runtime performance.
     */
    state.connAry.length = mod.connections.length;

    for (var i = 0; i < mod.connections.length; i++) {
        procId = MultiProc.getIdMeta(mod.connections[i].procName);
        state.connAry[i].procId = procId;
        state.connAry[i].driver = mod.connections[i].driver;
    }

    /* finish initializing the interrupt table */
    if (Program.build.target.isa == "v7M4") {
//      TODO
//      if (Core.id == 0) {
//          Hwi.construct(state.hwi, 53, NotifyDriverMbx.isr);
//      }
//      else {
//          Hwi.construct(state.hwi, 54, NotifyDriverMbx.isr);
//      }
        /* interrupt event IDs used by this processor */
        for (var i = 0; i < state.interruptTable.length; i++) {
            state.interruptTable[i] = 0xFFFF; /* TODO */
        }
    }
    else if (Program.build.target.isa == "arp32") {
        /* interrupt event IDs used by this processor */
        for (var i = 0; i < state.interruptTable.length; i++) {
            state.interruptTable[i] = 0xFFFF; /* TODO */
        }
    }
    else if (Program.build.target.isa == "66") {
        /* interrupt event IDs used by this processor */
        state.interruptTable[0] = 55; /* EVE1 -> DSP1 or DSP2 */
        state.interruptTable[1] = 56; /* EVE2 -> DSP1 or DSP2 */
        state.interruptTable[2] = 58; /* EVE3 -> DSP1 or DSP2 */
        state.interruptTable[3] = 59; /* EVE4 -> DSP1 or DSP2 */
        state.interruptTable[4] = 60; /* DSP1 -> DSP2 */
        state.interruptTable[5] = 60; /* DSP2 -> DSP1 */
        state.interruptTable[8] = 57; /* HOST -> DSP1 or DSP2 */

        /* these are not known at config time, set at runtime */
        state.interruptTable[6] = 0; /* IPU1 -> DSP1 or DSP2 */
        state.interruptTable[7] = 0; /* IPU2 -> DSP1 or DSP2 */
        state.interruptTable[9] = 0; /* IPU1-1 -> DSP1 or DSP2 */
        state.interruptTable[10] = 0; /* IPU2-1 -> DSP1 or DSP2 */
    }
    else if (Program.build.target.isa == "v7A15") {
        /* interrupt event IDs used by this processor */
        for (var i = 0; i < state.interruptTable.length; i++) {
            state.interruptTable[i] = 0xFFFF; /* TODO */
        }

        /* TODO */
        // Hwi.construct(state.hwi, 77, NotifyDriverMbx.isr);
    }
    else {
        throw("Invalid target: " + Program.build.target.$name);
    }

    /* initialize the driver table */
    for (var i = 0; i < state.isrDispatchTable.length; i++) {
        state.isrDispatchTable[i] = null;
    }
}
