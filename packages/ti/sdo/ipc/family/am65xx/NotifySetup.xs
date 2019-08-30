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
 *  ======== NotifySetup.xs ========
 */
var MultiProc = null;
var Core = null;
var Mmu = null;
var isaChain = "";

const NAVSS_MAILBOX0_CLUSTER0_BASEADDR = 0x0031F80000;
const NAVSS_MAILBOX0_CLUSTER1_BASEADDR = 0x0031F81000;
const NAVSS_MAILBOX0_CLUSTER2_BASEADDR = 0x0031F82000;

/* The following are decided at the system level */
/* The following corresponds to M2M Router output 0 */
const R5F_0_INT_0_NO = 160;
/* The following corresponds to M2M Router output 1 */
const R5F_0_INT_1_NO = 161;
/* The following corresponds to M2M Router output 2 */
const R5F_1_INT_0_NO = 162;
/* The following corresponds to M2M Router output 3 */
const R5F_1_INT_1_NO = 163;

/* The following corresponds to NAVSS output 112 */
const A53_INT_0_NO = 496;
/* The following corresponds to NAVSS output 113 */
const A53_INT_1_NO = 497;

/*
 *  ======== isMbxDrv ========
 */
function isMbxDrv(mod, name)
{
    if (mod.connections == undefined) {
        return (false);
    }

    for (var i = 0; i < mod.connections.length; i++) {
        if (mod.connections[i].procName == name) {
            if (mod.connections[i].driver == mod.Driver_MAILBOX) {
                return (true);
            }
            else {
                return (false);
            }
        }
    }

    return (false);
}

/*
 *  ======== module$use ========
 */
function module$use()
{
    /* load modules needed in meta domain and in target domain */
    var TableInit = xdc.useModule("ti.sdo.ipc.family.am65xx.TableInit");
    var NotifySciClient = xdc.useModule("ti.sdo.ipc.family.am65xx.NotifySciClient");
    MultiProc = xdc.useModule('ti.sdo.utils.MultiProc');
    xdc.useModule('xdc.runtime.Assert');
    xdc.useModule('xdc.runtime.Error');
    xdc.useModule('xdc.runtime.Startup');

    /* The following csl & osal modules needed by sciclient module */
    var devType              = "am65xx"
    var Csl         	     = xdc.loadPackage('ti.csl');
    Csl.Settings.deviceType  = devType;

    var osType           = "tirtos";
    var Osal             = xdc.loadPackage('ti.osal');
    Osal.Settings.osType = osType;

    /* sciclient needed for interrupt routing and other resource allocation */
    var sciclientSettings = xdc.useModule ('ti.drv.sciclient.Settings');
    sciclientSettings.socType = "am65xx";
    sciclientSettings.boardType = "am65xx_evm";

    /* concatinate isa chain into single string for easier matching */
    isaChain = "#" + Program.build.target.getISAChain().join("#") + "#";

    if (isaChain.match(/#v7R#/)) {
        xdc.useModule("ti.sysbios.BIOS");
        Core = xdc.useModule("ti.sysbios.family.arm.v7r.keystone3.Core");
        xdc.useModule('ti.sysbios.family.arm.v7r.keystone3.Hwi');

        if(Program.platformName.match(/R5F0/)) {
            sciclientSettings.coreType = "mcu1_0";
        } else if (Program.platformName.match(/R5F1/)) {
            sciclientSettings.coreType = "mcu1_1";
        }
    }
    else if (isaChain.match(/#v8A#/)) {
        Mmu = xdc.useModule("ti.sysbios.family.arm.v8a.Mmu");
        xdc.useModule('ti.sysbios.family.arm.gicv3.Hwi');
        sciclientSettings.coreType = "mpu1_0";
    }

    /* initialize procIdTable */
    TableInit.initProcId(this);

    /* initialize mailboxTable */
    TableInit.generateTable(this);

    if (isaChain.match(/#v7R#/)) {
        if (this.mailboxBaseAddr[0] == undefined) {
            this.mailboxBaseAddr[0] = NAVSS_MAILBOX0_CLUSTER0_BASEADDR;  /* System Mailbox 0 */
        }
        if (this.mailboxBaseAddr[1] == undefined) {
            this.mailboxBaseAddr[1] = NAVSS_MAILBOX0_CLUSTER1_BASEADDR;  /* System Mailbox 1 */
        }
        if (this.mailboxBaseAddr[2] == undefined) {
            this.mailboxBaseAddr[2] = NAVSS_MAILBOX0_CLUSTER2_BASEADDR;  /* System Mailbox 2 */
        }
    }
    else if (isaChain.match(/#v8A#/)) {
        /* initialize mailbox base address table */
        this.mailboxBaseAddr[0] = NAVSS_MAILBOX0_CLUSTER0_BASEADDR;  /* System Mailbox 0 */
        this.mailboxBaseAddr[1] = NAVSS_MAILBOX0_CLUSTER1_BASEADDR;  /* System Mailbox 1 */
        this.mailboxBaseAddr[2] = NAVSS_MAILBOX0_CLUSTER2_BASEADDR;  /* System Mailbox 2 */
    }
    else {
        throw("Invalid target: " + Program.build.target.$name);
    }

    if (isaChain.match(/#v8A#/)) {
if (0) {
    /* TODO: Need to review if any MMU entry need to be added */
        /*  Add mailbox addresses to the Mmu table.
         *  Force mailbox addresses to be NON cacheable.
         */
        var peripheralAttrs = {
            type : Mmu.DescriptorType_BLOCK,  // BLOCK descriptor
            accPerm    : 0,                   // read/write at PL1
            noExecute  : true,                // not executable
            attrIndx   : 1                    // MAIR0 Byte1 describes mem attr
        };

        /* configure the corresponding MMU page descriptor */
        Mmu.setSecondLevelDescMeta(0x42000000, 0x42000000, peripheralAttrs);
        Mmu.setSecondLevelDescMeta(0x42200000, 0x42200000, peripheralAttrs);
        Mmu.setSecondLevelDescMeta(0x48800000, 0x48800000, peripheralAttrs);
}
    }

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

    /*  If Notify module is already used, then load notify drivers into
     *  configuration model. Do *not* useModule the Notify module. Some
     *  applications require this notify driver but do *not* want the
     *  ti.sdo.ipc package to be loaded.
     */
    if (xdc.module('ti.sdo.ipc.Notify').$used) {
        if (this.$private.driverMask & this.Driver_SHAREDMEMORY) {
            xdc.useModule('ti.sdo.ipc.notifyDrivers.NotifyDriverShm');
        }
        if (this.$private.driverMask & this.Driver_MAILBOX) {
            xdc.useModule('ti.sdo.ipc.family.am65xx.NotifyDriverMbx');
        }
    }
}

/*
 *  ======== module$static$init ========
 *  Initialize the target state object.
 */
function module$static$init(state, mod)
{
    var procId;

    /*  Initialize the state connAry from the config params. Translate
     *  processor names into IDs for better runtime performance.
     */
    state.connAry.length = mod.connections.length;

    for (var i = 0; i < mod.connections.length; i++) {
        procId = MultiProc.getIdMeta(mod.connections[i].procName);
        state.connAry[i].procId = procId;
        state.connAry[i].driver = mod.connections[i].driver;
    }

    if (isaChain.match(/#v7R#/)) {
        state.numPlugged.length =mod.NUM_SYS_MBX;
        for (var i = 0; i < state.numPlugged.length; i++) {
            state.numPlugged[i] = 0;
        }

        if (Core.id == 0) {
                state.interruptTable[0] = 0;   /* R5F-0 */
                state.interruptTable[1] = R5F_0_INT_0_NO;  /* HOST */
                state.interruptTable[2] = R5F_0_INT_1_NO;  /* R5F-1 */
if (0) { /* TODO: Understanding is this is not needed; Need to check */
                var mbxDrv = isMbxDrv(mod, "R5F-1");
                state.interruptTable[2] = mbxDrv ? 69 : 19;  /*R5F-1 */
}
        } else {
if (0) { /* TODO: Understanding is this is not needed; Need to check */
                var mbxDrv = isMbxDrv(mod, "R5F-0");
                state.interruptTable[0] = mbxDrv ? 76 : 19;  /* R5F-0 */
}
                state.interruptTable[0] = R5F_1_INT_0_NO;  /* HOST */
                state.interruptTable[1] = R5F_1_INT_1_NO;  /* R5F-0 */
                state.interruptTable[2] = 0;   /* R5F-1 */
        }
    }
    else if (isaChain.match(/#v8A#/)) {
        state.numPlugged.length = mod.NUM_SYS_MBX;
        for (var i = 0; i < state.numPlugged.length; i++) {
            state.numPlugged[i] = 0;
        }

        state.interruptTable[0] = A53_INT_0_NO;   /* R5F-0 */
        state.interruptTable[1] = 0;          /* HOST */
        state.interruptTable[2] = A53_INT_1_NO;   /* R5F-1 */
    }
    else {
        throw("Invalid target: " + Program.build.target.$name);
    }

    /* initialize the driver table */
    for (var i = 0; i < state.isrDispatchTable.length; i++) {
        state.isrDispatchTable[i] = null;
    }
}
