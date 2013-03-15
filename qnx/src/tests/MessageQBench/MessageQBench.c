/*
 * Copyright (c) 2012-2013, Texas Instruments Incorporated
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
/* =============================================================================
 *  @file   MessageQBench.c
 *
 *  @brief  Benchmark application for MessageQ module between MPU/Remote Proc
 *
 *  ============================================================================
 */

/*
 * opkg --tmp-dir ~/tmp install util-linux-ng
 * chrt -f ./MessageQBench
 */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/param.h>

#include <Std.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/MessageQ.h>

#define MessageQ_payload(m) ((void *)((char *)(m) + sizeof(MessageQ_MsgHeader)))
#define MINPAYLOADSIZE      (2 * sizeof(UInt32))

/* App defines: Must match on remote proc side: */
#define NUM_LOOPS_DFLT           1000  /* Number of transfers to be tested. */
#define HEAPID              0u
#define PROC_ID_DFLT        1     /* Host is zero, remote cores start at 1 */
#define SLAVE_MESSAGEQNAME  "SLAVE"
#define MPU_MESSAGEQNAME    "HOST"


long diff(struct timespec start, struct timespec end)
{
    struct timespec temp;

    if ((end.tv_nsec - start.tv_nsec) < 0) {
        temp.tv_sec = end.tv_sec - start.tv_sec-1;
        temp.tv_nsec = 1000000000UL + end.tv_nsec - start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec - start.tv_sec;
        temp.tv_nsec = end.tv_nsec - start.tv_nsec;
    }

    return (temp.tv_sec * 1000000UL + temp.tv_nsec / 1000);
}

Int MessageQApp_execute(UInt32 numLoops, UInt32 payloadSize, UInt16 procId)
{
    Int32                    status     = 0;
    MessageQ_Msg             msg        = NULL;
    MessageQ_Params          msgParams;
    UInt16                   i;
    MessageQ_QueueId         queueId = MessageQ_INVALIDMESSAGEQ;
    MessageQ_Handle          msgqHandle;
    char                     remoteQueueName[64];
    struct timespec          start, end;
    long                     elapsed;
    UInt32 *params;

    printf ("Entered MessageQApp_execute\n");

    /* Create the local Message Queue for receiving. */
    MessageQ_Params_init(&msgParams);
    msgqHandle = MessageQ_create(MPU_MESSAGEQNAME, &msgParams);
    if (msgqHandle == NULL) {
        printf ("Error in MessageQ_create\n");
        goto exit;
    }
    else {
        printf ("Local MessageQId: 0x%x\n", MessageQ_getQueueId(msgqHandle));
    }

    sprintf(remoteQueueName, "%s_%s", SLAVE_MESSAGEQNAME,
             MultiProc_getName(procId));

    /* Poll until remote side has its messageQ created before we send: */
    do {
        status = MessageQ_open(remoteQueueName, &queueId);
        sleep (1);
    } while (status == MessageQ_E_NOTFOUND);

    if (status < 0) {
        printf("Error in MessageQ_open [%d]\n", status);
        goto cleanup;
    }

    printf("Remote queueId  [0x%x]\n", queueId);

    msg = MessageQ_alloc(HEAPID, sizeof(MessageQ_MsgHeader) + payloadSize);
    if (msg == NULL) {
        printf("Error in MessageQ_alloc\n");
        MessageQ_close(&queueId);
        goto cleanup;
    }

    /* handshake with remote to set the number of loops */
    MessageQ_setReplyQueue(msgqHandle, msg);
    params = MessageQ_payload(msg);
    params[0] = numLoops;
    params[1] = FALSE;
    MessageQ_put(queueId, msg);
    MessageQ_get(msgqHandle, &msg, MessageQ_FOREVER);

    printf("Exchanging %d messages with remote processor %s...\n",
           numLoops, MultiProc_getName(procId));

    clock_gettime(CLOCK_REALTIME, &start);

    for (i = 0; i < numLoops; i++) {
        MessageQ_setMsgId(msg, i);

        /* Have the remote proc reply to this message queue */
        MessageQ_setReplyQueue(msgqHandle, msg);

        status = MessageQ_put(queueId, msg);
        if (status < 0) {
            printf("Error in MessageQ_put [%d]\n", status);
            break;
        }

        status = MessageQ_get(msgqHandle, &msg, MessageQ_FOREVER);
        if (status < 0) {
            printf("Error in MessageQ_get [%d]\n", status);
            break;
        }
        else {
            /* Validate the returned message. */
            if ((msg != NULL) && (MessageQ_getMsgId(msg) != i)) {
                printf ("Data integrity failure!\n"
                        "    Expected %d\n"
                        "    Received %d\n",
                        i, MessageQ_getMsgId (msg));
                break;
            }
        }
    }

    clock_gettime(CLOCK_REALTIME, &end);
    elapsed = diff(start, end);

    if (numLoops > 0) {
        printf("%s: Avg round trip time: %ld usecs\n",
               MultiProc_getName(procId), elapsed / numLoops);
    }

    MessageQ_free(msg);
    MessageQ_close(&queueId);

cleanup:
    status = MessageQ_delete(&msgqHandle);
    if (status < 0) {
        printf ("Error in MessageQ_delete [%d]\n", status);
    }

exit:
    printf("Leaving MessageQApp_execute\n\n");

    return (status);
}

int main (int argc, char * argv[])
{
    Int32 status = 0;
    UInt32 numLoops = NUM_LOOPS_DFLT;
    UInt32 payloadSize = MINPAYLOADSIZE;
    UInt16 procId = PROC_ID_DFLT;
    UInt32 argul = strtoul(argv[2], NULL, 0);

    /* Parse args: */
    if (argc > 1) {
        numLoops = strtoul(argv[1], NULL, 0);
    }

    if (argc > 2) {
        payloadSize = ((argul >= MINPAYLOADSIZE) ? argul : MINPAYLOADSIZE);
    }

    if (argc > 3) {
        procId  = atoi(argv[3]);
    }

    if (argc > 4) {
        printf("Usage: %s [<numLoops>] [<payloadSize>] [<ProcId>]\n", argv[0]);
        printf("\tDefaults: numLoops: %d; payloadSize: %d, ProcId: %d\n",
                   NUM_LOOPS_DFLT, MINPAYLOADSIZE, PROC_ID_DFLT);
        exit(0);
    }
    if (procId >= MultiProc_getNumProcessors()) {
        printf("ProcId must be less than %d\n", MultiProc_getNumProcessors());
        exit(0);
    }
    printf("Using numLoops: %d; payloadSize: %d, procId : %d\n",
            numLoops, payloadSize, procId);

    status = Ipc_start();

    if (status >= 0) {
        MessageQApp_execute(numLoops, payloadSize, procId);
        Ipc_stop();
    }
    else {
        fprintf(stderr, "Ipc_start failed: status = %d\n", status);
    }

    return (status);
}
