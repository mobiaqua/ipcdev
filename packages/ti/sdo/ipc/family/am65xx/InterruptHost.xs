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
 *  ======== InterruptHost.xs ========
 */

/*
 *  ======== module$use ========
 */
function module$use()
{
    xdc.useModule("xdc.runtime.Assert");

    xdc.useModule("ti.sysbios.family.arm.v8a.Mmu");
    xdc.useModule("ti.sysbios.family.arm.gicv3.Hwi");

    xdc.useModule("ti.sdo.ipc.Ipc");
    xdc.useModule("ti.sdo.ipc.family.am65xx.InterruptHost");
    xdc.useModule("ti.sdo.ipc.family.am65xx.NotifySetup");

    var TableInit = xdc.useModule("ti.sdo.ipc.family.am65xx.TableInit");

    /* initialize procIdTable */
    TableInit.initProcId(this);

    /* initialize mailboxTable */
    TableInit.generateTable(this);

    /* Initialize mailbox base address table */
    /* TODO: 0x0031F80000 is address from main domain memory map
     *       Need to find any addresss translation involved for v8A */
    this.mailboxBaseAddr[0] = 0x0031F80000;  /* System Mailbox 0 */
    /* Initialize mailbox base address table */
    /* TODO: 0x0031F80000 is address from main domain memory map
     *       Need to find any addresss translation involved for v8A */
    this.mailboxBaseAddr[1] = 0x0031F81000;  /* System Mailbox 1 */
    /* Initialize mailbox base address table */
    /* TODO: 0x0031F80000 is address from main domain memory map
     *       Need to find any addresss translation involved for v8A */
    this.mailboxBaseAddr[2] = 0x0031F82000;  /* System Mailbox 0 */
}

/*
 *  ======== module$static$init ========
 *  Initialize the target state object.
 */
function module$static$init(state, mod)
{
    for (var i = 0; i < this.procIdTable.length; i++) {
        state.fxnTable[i].func = null;
        state.fxnTable[i].arg = 0;
    }
}
