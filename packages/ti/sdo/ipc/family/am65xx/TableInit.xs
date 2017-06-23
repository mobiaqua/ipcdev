/*
 * Copyright (c) 2012-2018 Texas Instruments Incorporated - http://www.ti.com
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
 *  ======== TableInit.xs ========
 *
 */

/*
 * When assigning virtual indexes to each core make sure
 * to assign even virtual indexes to DSP/M4 cores with
 * even Core Ids, and assign odd virtual indexes to DSP/M4
 * cores with odd Core Ids.
 *
 * Example:
 *     DSP physical Core Id = 0 -> Virtual Index = 4;
 *     DSP physical Core Id = 1 -> Virtual Index = 5;
 *
 * Virtual Index Assignment:
 *
 * | R5F-0 -> 6 |
 * | HOST -> 8 | R5F-1 -> 9
 *
 */
var r5f_0VirtId  = 0;
var hostVirtId   = 1;
var r5f_1VirtId  = 2;

/*
 *  ======== initProcId ========
 *  Assign MultiProc ids and virtual processor ids.
 */
function initProcId(mod)
{
    var MultiProc = xdc.useModule("ti.sdo.utils.MultiProc");

    for (var i = 0; i < mod.procIdTable.length; i++) {
        mod.procIdTable[i] = -1;
    }

    mod.r5f_0ProcId = MultiProc.getIdMeta("R5F-0");
    mod.r5f_1ProcId = MultiProc.getIdMeta("R5F-1");
    mod.hostProcId   = MultiProc.getIdMeta("HOST");

    if (mod.r5f_0ProcId != MultiProc.INVALIDID) {
        mod.procIdTable[mod.r5f_0ProcId] = r5f_0VirtId;
    }

    if (mod.r5f_1ProcId != MultiProc.INVALIDID) {
        mod.procIdTable[mod.r5f_1ProcId] = r5f_1VirtId;
    }

    if (mod.hostProcId != MultiProc.INVALIDID) {
        mod.procIdTable[mod.hostProcId] = hostVirtId;
    }
}

/*
 * Function to generate mailbox table
 */
function generateTable(InterruptCore)
{
    var SYS_MBX0_OFFSET = 0;
    var SYS_MBX1_OFFSET = 1;
    var SYS_MBX2_OFFSET = 2;

    var index;
    var subMbxIdx;
    var tableEntry;
    var mbxUserIdx;
    var mbxBaseAddrIdx;

    /*
     * Each entry in the mailbox table stores 3 indexes.
     * The breakup of each entry is shown below:
     * Entry format : 0xAAAABBCC
     *         AAAA : Mailbox base address table index
     *           BB : Mailbox User Id
     *           CC : Sub-mailbox index
     *
     * In order to lookup the User Id, Sub-mailbox Index and mailbox base
     * address for a given src and dst core from the mailboxTable, the
     * procedure shown below is followed:
     *     1. Find the right entry for the given src and dst core.
     *        mailboxTable index is given by:
     *            Index = (src * NumCores) + dst
     *     2. Mbx BaseAddr Index = mailboxTable[Index] >> 16
     *     2. dst Mailbox UserId = (mailboxTable[Index] >> 8) & 0xFF
     *     3. Sub-Mailbox Index  = mailboxTable[Index] & 0xFF
     */

    /* TODO: Why not maintain mailboxTable as two dimentional array? */
    /*
     * 'i' is src core index, and
     * 'j' is dst core index
     */
    for (var i = 0; i < InterruptCore.NUM_CORES; i++) {
        for (var j = 0; j < InterruptCore.NUM_CORES; j++) {

            /* init mailboxTable */
            index = (i * InterruptCore.NUM_CORES) + j;
            InterruptCore.mailboxTable[index] = -1;

            /* No mailbox for same core */
            if (i == j)
                continue;

            /* System Mailbox 0 */
            /* For communication between HOST<->R5F0 */
            if (((i == r5f_0VirtId) || (i == hostVirtId)) &&
                ((j == r5f_0VirtId) || (j == hostVirtId))) {

                mbxBaseAddrIdx = (SYS_MBX0_OFFSET) << 16;

                if (j == hostVirtId) {
                    if (i == r5f_0VirtId) {
                        mbxUserIdx = 0 << 8;
                        subMbxIdx = 0;
                    }
                }
               else if (j == r5f_0VirtId) {
                    if (i == hostVirtId) {
                        mbxUserIdx = 1 << 8;
                        subMbxIdx = 1;
                    }
                }
            }
            /* System Mailbox 1 */
            /* For communication between HOST<->R5F1 */
            if (((i == hostVirtId) || (i == r5f_1VirtId)) &&
                ((j == hostVirtId) || (j == r5f_1VirtId))) {
                mbxBaseAddrIdx = (SYS_MBX1_OFFSET) << 16;

                if (j == hostVirtId) {
                    if (i == r5f_1VirtId) {
                        mbxUserIdx = 0 << 8;
                        subMbxIdx = 0;
                    }
                }
                else if (j == r5f_1VirtId) {
                    if (i == hostVirtId) {
                        mbxUserIdx = 1 << 8;
                        subMbxIdx = 1;
                    }
                }
            }

            /* System Mailbox 2 */
            /* For communication between R5F0<->R5F1 */
            if (((i == r5f_0VirtId) || (i == r5f_1VirtId)) &&
                ((j == r5f_0VirtId) || (j == r5f_1VirtId))) {
                mbxBaseAddrIdx = (SYS_MBX2_OFFSET) << 16;

                if (j == r5f_0VirtId) {
                    if (i == r5f_1VirtId) {
                        mbxUserIdx = 0 << 8;
                        subMbxIdx = 0;
                    }
                }
                else if (j == r5f_1VirtId) {
                    if (i == r5f_0VirtId) {
                        mbxUserIdx = 1 << 8;
                        subMbxIdx = 1;
                    }
                }
            }

            tableEntry = mbxBaseAddrIdx | mbxUserIdx | subMbxIdx;
            InterruptCore.mailboxTable[index] = tableEntry;
            continue;

        }
    }
}
