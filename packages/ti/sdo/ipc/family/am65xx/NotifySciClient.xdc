/*
 * Copyright (c) 2018, Texas Instruments Incorporated
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
package ti.sdo.ipc.family.am65xx;

module NotifySciClient
{
    enum CoreIndex {
        A53_0_CORE_INDEX = 0,
        /**< Core index for A53 core 0 */
        R5F_0_CORE_INDEX,
        /**< Core index for R5F core 0 */
        R5F_1_CORE_INDEX
        /**< Core index for R5F core 1 */
    } ;

    enum SourceIdIndex {
        MAILBOX_CLUSTER0_SRC_ID_INDEX = 0,
        /**< Source id index for mailbox cluster 0 */
        MAILBOX_CLUSTER1_SRC_ID_INDEX,
        /**< Source id index for mailbox cluster 1 */
        MAILBOX_CLUSTER2_SRC_ID_INDEX
        /**< Source id index for mailbox cluster 2 */
    } ;

    enum MailboxIndex {
        MAILBOX_USER_0= 0,
        /**< mailbox user 0 */
        MAILBOX_USER_1,
        /**<  mailbox user 1 */
        MAILBOX_USER_2,
        /**<  mailbox user 2 */
        MAILBOX_USER_3
        /**<  mailbox user 3 */
    } ;

    enum SecondaryHost {
        SECONDARYHOST_UNUSED = 0,
        /**< Secondary host unused */
        SECONDARYHOST_SPECIFIC_HOST,
        /**< Secondary host specific host */
        SECONDARYHOST_ALL
        /**< Secondary host all */
    } ;

    Int32 Init ();
    Int32 IrqSet(CoreIndex coreIndex,
                 SourceIdIndex mailboxClusterIndex,
                 MailboxIndex mailboxUserIndex,
                 UInt32 intNumber);
    Int32 IrqRelease(CoreIndex coreIndex,
                 SourceIdIndex mailboxClusterIndex,
                 MailboxIndex mailboxUserIndex,
                 UInt32 intNumber);
    Int32 getIntNumRange(CoreIndex coreIndex,
                         SecondaryHost secondaryHost,
                         UInt16 *rangeStartP,
                         UInt16 *rangeNumP);
};
