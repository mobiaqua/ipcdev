/*
 * Copyright (c) 2012-2014, Texas Instruments Incorporated
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
 *  ======== NotifyDriverMbx.xs ================
 */

var NotifyDriverMbx = null;
var Core = null;

/*
 *  ======== module$use ========
 */
function module$use()
{
    var TableInit = xdc.useModule("ti.sdo.ipc.family.vayu.TableInit");

    NotifyDriverMbx = this;

    xdc.useModule("ti.sysbios.family.shared.vayu.IntXbar");
    xdc.useModule("ti.sysbios.family.c64p.EventCombiner");
    xdc.useModule("ti.sysbios.family.c64p.Hwi");
    xdc.useModule("ti.sdo.ipc.Notify");
    xdc.useModule("ti.sdo.utils.MultiProc");

    if (Program.build.target.$name.match(/M3/)) {
        Core = xdc.useModule("ti.sysbios.family.arm.ducati.Core");
    }

    /* initialize procIdTable */
    TableInit.initProcId(NotifyDriverMbx);

    /* Initialize mailboxTable */
    TableInit.generateTable(NotifyDriverMbx);

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
}


/*
 *  ======== module$static$init ========
 *  Initialize the target state object.
 */
function module$static$init(state, mod)
{

    for (var i = 0; i < state.drvHandles.length; i++) {
        state.drvHandles[i] = null;
    }

    /* TODO this code is specific to the DSP, need to add other processors */


    if (Program.build.target.$name.match(/M3/)) {
/* TODO */
if (0) {
        if (Core.id == 0) {
            Hwi.construct(state.hwi, 53, NotifyDriverMbx.isr);
        }
        else {
            Hwi.construct(state.hwi, 54, NotifyDriverMbx.isr);
        }
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
    else if (Program.build.target.$name.match(/A8/)) {
/* TODO */
if (0) {
        Hwi.construct(state.hwi, 77, NotifyDriverMbx.isr);
}
    }
    else {
        throw("Invalid target: " + Program.build.target.$name);
    }
}

/*
 *************************************************************************
 *                       ROV View functions
 *************************************************************************
 */

/*
 *  ======== viewInitBasic ========
 */
function viewInitBasic(view, obj)
{
    var Program = xdc.useModule('xdc.rov.Program');
    var ScalarStructs = xdc.useModule('xdc.rov.support.ScalarStructs');
    var MultiProc = xdc.useModule('ti.sdo.utils.MultiProc');
    var modCfg = Program.getModuleConfig(
            'ti.sdo.ipc.family.vayu.NotifyDriverMbx');

    /* translate the virtual id to a processor id */
    var remoteProcId = -1;

    for (var i = 0; i < modCfg.procIdTable.length; i++) {
        if (obj.remoteVirtId == modCfg.procIdTable[i]) {
            remoteProcId = i;
            break;
        }
    }

    /* view.remoteProc */
    try {
        view.remoteProc = MultiProc.getName$view(remoteProcId);
    }
    catch (e) {
        Program.displayError(view, 'remoteProc',
                "Problem retrieving proc name: " + e);
    }

    /* view.mailboxAddr */
    var selfVirtId = modCfg.procIdTable[MultiProc.self$view()];
    var idx = (obj.remoteVirtId * modCfg.NUM_CORES) + selfVirtId;
    var baseAddrIdx = (modCfg.mailboxTable[idx] >> 16) & 0xFFFF;
    var mailboxAddr = modCfg.mailboxBaseAddr[baseAddrIdx];
    view.mailboxAddr = "0x" + Number(mailboxAddr).toString(16);

    /* view.subMbxId */
    view.subMbxId = modCfg.mailboxTable[idx] & 0xFF;

    /* view.count */
    try {
        var MAILBOX_STATUS_IN = Program.fetchStruct(
                ScalarStructs.S_Bits32$fetchDesc,
                mailboxAddr + 0xC0 + (0x4 * view.subMbxId), false);
        view.count = MAILBOX_STATUS_IN.elem;
    }
    catch (e) {
        throw(e);
    }
}
