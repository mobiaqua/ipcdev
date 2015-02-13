/*
 * Copyright (c) 2014-2015 Texas Instruments Incorporated - http://www.ti.com
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
 *  ======== TransportRpmsg.c ========
 *  Implementation of functions specified in the IMessageQTransport interface.
 */

/* Socket Headers */
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/eventfd.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <pthread.h>

/* Socket Protocol Family */
#include <net/rpmsg.h>


/* IPC headers */
#include <ti/ipc/Std.h>
#include <SocketFxns.h>         /* Socket utils: */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/MultiProc.h>
#include <ti/ipc/transports/TransportRpmsg.h>
#include <_MessageQ.h>
#include <_lad.h>

/* More magic rpmsg port numbers: */
#define MESSAGEQ_RPMSG_PORT       61
#define MESSAGEQ_RPMSG_MAXSIZE   512

#define TransportRpmsg_GROWSIZE 32

/* traces in this file are controlled via _TransportMessageQ_verbose */
Bool _TransportMessageQ_verbose = FALSE;
#define verbose _TransportMessageQ_verbose

Int TransportRpmsg_bind(Void *handle, UInt32 queueId);
Int TransportRpmsg_unbind(Void *handle, UInt32 queueId);
Bool TransportRpmsg_put(Void *handle, Ptr msg);

typedef struct TransportRpmsg_Module {
    int             sock[MultiProc_MAXPROCESSORS];
    fd_set          rfds;
    int             maxFd;
    int             inFds[1024];
    int	            nInFds;
    pthread_mutex_t gate;
    int             unblockEvent;    /* eventFd for unblocking socket */
    pthread_t       threadId;        /* ID returned by pthread_create() */
    Bool            threadStarted;

    TransportRpmsg_Handle *inst;    /* array of instances */
} TransportRpmsg_Module;

IMessageQTransport_Fxns TransportRpmsg_fxns = {
    .bind    = TransportRpmsg_bind,
    .unbind  = TransportRpmsg_unbind,
    .put     = TransportRpmsg_put
};

typedef struct TransportRpmsg_Object {
    IMessageQTransport_Object base;
    Int status;
    UInt16 rprocId;
    int numQueues;
    int *qIndexToFd;
} TransportRpmsg_Object;

TransportRpmsg_Module TransportRpmsg_state = {
    .sock = {0},
    .threadStarted = FALSE,
    .inst = NULL
};
TransportRpmsg_Module *TransportRpmsg_module = &TransportRpmsg_state;

static Int attach(UInt16 rprocId);
static Int detach(UInt16 rprocId);
static void *rpmsgThreadFxn(void *arg);
static Int transportGet(int sock, MessageQ_Msg *retMsg);
static Void bindFdToQueueIndex(TransportRpmsg_Object *obj,
                               Int fd,
                               UInt16 qIndex);
static Void unbindQueueIndex(TransportRpmsg_Object *obj, UInt16 qIndex);
static Int queueIndexToFd(TransportRpmsg_Object *obj, UInt16 qIndex);

Int TransportRpmsg_Factory_create(Void);
Void TransportRpmsg_Factory_delete(Void);

Ipc_TransportFactoryFxns TransportRpmsg_Factory = {
    .createFxn = TransportRpmsg_Factory_create,
    .deleteFxn = TransportRpmsg_Factory_delete
};

/* -------------------------------------------------------------------------- */

/* instance convertors */
IMessageQTransport_Handle TransportRpmsg_upCast(TransportRpmsg_Handle handle)
{
    TransportRpmsg_Object *obj = (TransportRpmsg_Object *)handle;
    return ((IMessageQTransport_Handle)&obj->base);
}

TransportRpmsg_Handle TransportRpmsg_downCast(IMessageQTransport_Handle base)
{
    return ((TransportRpmsg_Handle)base);
}

TransportRpmsg_Handle TransportRpmsg_create(TransportRpmsg_Params *params,
                                            Int *attachStatus)
{
    TransportRpmsg_Object *obj;
    Int rv;

    rv = attach(params->rprocId);
    if (attachStatus) {
        *attachStatus = rv;
    }

    if (rv != MessageQ_S_SUCCESS) {
        return NULL;
    }

    obj = calloc(1, sizeof (TransportRpmsg_Object));

    /* structure copy */
    obj->base.base.interfaceType = IMessageQTransport_TypeId;
    obj->base.fxns = &TransportRpmsg_fxns;
    obj->rprocId = params->rprocId;

    obj->qIndexToFd = calloc(TransportRpmsg_GROWSIZE, sizeof (Int));
    obj->numQueues = TransportRpmsg_GROWSIZE;

    return (TransportRpmsg_Handle)obj;
}

Void TransportRpmsg_delete(TransportRpmsg_Handle *handlep)
{
    TransportRpmsg_Object *obj = *(TransportRpmsg_Object **)handlep;

    detach(obj->rprocId);

    free(obj->qIndexToFd);
    free(obj);

    *handlep = NULL;
}

static Int attach(UInt16 rprocId)
{
    Int     status = MessageQ_S_SUCCESS;
    int     sock;
    UInt16  clusterId;


    clusterId = rprocId - MultiProc_getBaseIdOfCluster();

    /* Create the socket for sending messages to the remote proc: */
    sock = socket(AF_RPMSG, SOCK_SEQPACKET, 0);
    if (sock < 0) {
        status = MessageQ_E_FAIL;
        printf("attach: socket failed: %d (%s)\n",
               errno, strerror(errno));

        goto exit;
    }

    PRINTVERBOSE1("attach: created send socket: %d\n", sock)

    /* Attempt to connect: */
    status = ConnectSocket(sock, rprocId, MESSAGEQ_RPMSG_PORT);
    if (status < 0) {
        /* is it ok to "borrow" this error code from MessageQ? */
        status = MessageQ_E_RESOURCE;

        /* don't hard-printf or exit since this is no longer fatal */
        PRINTVERBOSE1("attach: ConnectSocket(rprocId:%d) failed\n", rprocId)

        goto exitSock;
    }

    TransportRpmsg_module->sock[clusterId] = sock;

    if (TransportRpmsg_module->threadStarted == FALSE) {
        /* create a module wide event to unblock the socket select thread */
        TransportRpmsg_module->unblockEvent = eventfd(0, 0);
        if (TransportRpmsg_module->unblockEvent == -1) {
            printf("attach: unblock socket failed: %d (%s)\n",
                   errno, strerror(errno));
            status = MessageQ_E_FAIL;

            goto exitSock;
        }

        PRINTVERBOSE1("attach: created unblock event %d\n",
                      TransportRpmsg_module->unblockEvent)

        FD_ZERO(&TransportRpmsg_module->rfds);
        FD_SET(TransportRpmsg_module->unblockEvent,
               &TransportRpmsg_module->rfds);
        TransportRpmsg_module->maxFd = TransportRpmsg_module->unblockEvent;
        TransportRpmsg_module->nInFds = 0;

        pthread_mutex_init(&TransportRpmsg_module->gate, NULL);

        status = pthread_create(&TransportRpmsg_module->threadId, NULL,
                                &rpmsgThreadFxn, NULL);
        if (status < 0) {
            status = MessageQ_E_FAIL;
            printf("attach: failed to spawn thread\n");

            goto exitEvent;
        }
        else {
            TransportRpmsg_module->threadStarted = TRUE;
        }
    }

    goto exit;

exitEvent:
    close(TransportRpmsg_module->unblockEvent);

    FD_ZERO(&TransportRpmsg_module->rfds);
    TransportRpmsg_module->maxFd = 0;

exitSock:
    close(sock);
    TransportRpmsg_module->sock[clusterId] = 0;

exit:
    return status;
}

static Int detach(UInt16 rprocId)
{

    Int     status = -1;
    int     sock;
    UInt16  clusterId;

    clusterId = rprocId - MultiProc_getBaseIdOfCluster();
    sock = TransportRpmsg_module->sock[clusterId];

    if (sock) {
        PRINTVERBOSE1("detach: closing socket: %d\n", sock)

        status = close(sock);
    }

    return status;
}

Int TransportRpmsg_bind(Void *handle, UInt32 queueId)
{
    TransportRpmsg_Object *obj = (TransportRpmsg_Object *)handle;
    UInt16   queuePort = queueId & 0x0000ffff;
    int      fd;
    int      err;
    uint64_t buf;
    UInt16   rprocId;

    rprocId = obj->rprocId;

    PRINTVERBOSE2("TransportRpmsg_bind: creating endpoint for rprocId %d "
            "queuePort 0x%x\n", rprocId, queuePort)

    /*  Create the socket to receive messages for this messageQ. */
    fd = socket(AF_RPMSG, SOCK_SEQPACKET, 0);
    if (fd < 0) {
        printf("TransportRpmsg_bind: socket call failed: %d (%s)\n",
                errno, strerror(errno));
        goto exitClose;
    }

    PRINTVERBOSE1("TransportRpmsg_bind: created socket fd %d\n", fd)

    err = SocketBindAddr(fd, rprocId, (UInt32)queuePort);
    if (err < 0) {
        /* don't hard-printf since this is no longer fatal */
        PRINTVERBOSE2("TransportRpmsg_bind: bind failed: %d (%s)\n",
                      errno, strerror(errno))

        close(fd);

        return -1;
    }

    pthread_mutex_lock(&TransportRpmsg_module->gate);

    /* add to our fat fd array and update select() parameters */
    TransportRpmsg_module->inFds[TransportRpmsg_module->nInFds++] = fd;
    TransportRpmsg_module->maxFd = MAX(TransportRpmsg_module->maxFd, fd);
    FD_SET(fd, &TransportRpmsg_module->rfds);

    pthread_mutex_unlock(&TransportRpmsg_module->gate);

    bindFdToQueueIndex(obj, fd, queuePort);

    /*
     * Even though we use the unblock event as just a signalling event with
     * no related payload, we need to write some non-zero value.  Might as
     * well make it the fd (which the reader could decide to use if needed).
     */
    buf = fd;
    write(TransportRpmsg_module->unblockEvent, &buf, sizeof (buf));

    goto exit;

exitClose:
    TransportRpmsg_unbind(handle, fd);
    fd = 0;

exit:
    return fd;
}

Int TransportRpmsg_unbind(Void *handle, UInt32 queueId)
{
    TransportRpmsg_Object *obj = (TransportRpmsg_Object *)handle;
    UInt16 queuePort = queueId & 0x0000ffff;
    uint64_t buf;
    Int    status = MessageQ_S_SUCCESS;
    int    maxFd;
    int    fd;
    int    i;
    int    j;

    fd = queueIndexToFd(obj, queuePort);
    if (!fd) {
        PRINTVERBOSE1("TransportRpmsg_unbind: queueId 0x%x not bound\n",
                      queueId)

        return -1;
    }

    PRINTVERBOSE1("TransportRpmsg_unbind: closing socket %d\n", fd)

    pthread_mutex_lock(&TransportRpmsg_module->gate);

    /* remove from input fd array */
    for (i = 0; i < TransportRpmsg_module->nInFds; i++) {
        if (TransportRpmsg_module->inFds[i] == fd) {
            TransportRpmsg_module->nInFds--;

            /* shift subsequent elements down */
            for (j = i; j < TransportRpmsg_module->nInFds; j++) {
                TransportRpmsg_module->inFds[j] =
                    TransportRpmsg_module->inFds[j + 1];
            }
            TransportRpmsg_module->inFds[TransportRpmsg_module->nInFds] = 0;

            FD_CLR(fd, &TransportRpmsg_module->rfds);
            if (fd == TransportRpmsg_module->maxFd) {
                /* find new max fd */
                maxFd = TransportRpmsg_module->unblockEvent;
                for (j = 0; j < TransportRpmsg_module->nInFds; j++) {
                    maxFd = MAX(TransportRpmsg_module->inFds[j], maxFd);
                }
                TransportRpmsg_module->maxFd = maxFd;
            }

            /*
             * Even though we use the unblock event as just a signalling
             * event with no related payload, we need to write some non-zero
             * value.  Might as well make it the fd (which the reader could
             * decide to use if needed).
             */
            buf = fd;
            write(TransportRpmsg_module->unblockEvent, &buf, sizeof (buf));

            break;
        }

        close(fd);
    }

    unbindQueueIndex(obj, queuePort);

    pthread_mutex_unlock(&TransportRpmsg_module->gate);

    return status;
}

Bool TransportRpmsg_put(Void *handle, Ptr pmsg)
{
    MessageQ_Msg msg  = (MessageQ_Msg)pmsg;
    Int     status    = TRUE;
    int     sock;
    int     err;
    UInt16  clusterId;

    /*
     * Retrieve the socket for the AF_SYSLINK protocol associated with this
     * transport.
     */
    clusterId = msg->dstProc - MultiProc_getBaseIdOfCluster();
    sock = TransportRpmsg_module->sock[clusterId];
    if (!sock) {
        return FALSE;
    }

    PRINTVERBOSE2("Sending msgId: %d via sock: %d\n", msg->msgId, sock)

    err = send(sock, msg, msg->msgSize, 0);
    if (err < 0) {
        printf("TransportRpmsg_put: send failed: %d (%s)\n",
               errno, strerror(errno));
        status = FALSE;

        goto exit;
    }

    /*
     * Free the message, as this is a copy transport, we maintain MessageQ
     * semantics.
     */
    MessageQ_free(msg);

exit:
    return status;
}

Bool TransportRpmsg_control(Void *handle, UInt cmd, UArg cmdArg)
{
    return FALSE;
}

void *rpmsgThreadFxn(void *arg)
{
    static int lastFdx = 0;
    int      curFdx = 0;
    Int      status = MessageQ_S_SUCCESS;
    Int      tmpStatus;
    int      retval;
    uint64_t buf;
    fd_set   rfds;
    int      maxFd;
    int      nfds;
    MessageQ_Msg     retMsg;
    MessageQ_QueueId queueId;

    while (TRUE) {
        pthread_mutex_lock(&TransportRpmsg_module->gate);

        maxFd = TransportRpmsg_module->maxFd;
        rfds = TransportRpmsg_module->rfds;
        nfds = TransportRpmsg_module->nInFds;

        pthread_mutex_unlock(&TransportRpmsg_module->gate);

        PRINTVERBOSE3("rpmsgThreadFxn: maxFd %d rfds[1:0] 0x%08x%08x\n", maxFd,
                      (int)rfds.fds_bits[1], (int)rfds.fds_bits[0])

        retval = select(maxFd + 1, &rfds, NULL, NULL, NULL);
        if (retval) {
            if (FD_ISSET(TransportRpmsg_module->unblockEvent, &rfds)) {
                /*
                 * Our event was signalled by TransportRpmsg_bind()
                 * or TransportRpmsg_unbind() to tell us that the set of
                 * fds has changed.
                 */
                PRINTVERBOSE0("rpmsgThreadFxn: got unblock event\n")

                /* we don't need the written value */
                read(TransportRpmsg_module->unblockEvent, &buf, sizeof (buf));
            }
            else {
                /* start where we last left off */
                curFdx = lastFdx;

                /*
                 * The set of fds that's used by select has been recorded
                 * locally, but the array of fds that are scanned below is
                 * a changing set (MessageQ_create/delete() can change it).
                 * While this might present an issue in itself, one key
                 * takeaway is that 'nfds' must not be zero else the % below
                 * will cause a divide-by-zero exception.  We won't even get
                 * here if nfds == 0 since it's a local copy of the module's
                 * 'nInFds' which has to be > 0 for us to get here.  So, even
                 * though the module's 'nInFds' might go to 0 during this loop,
                 * the loop itself will still remain intact.
                 */
                do {
                    if (FD_ISSET(TransportRpmsg_module->inFds[curFdx], &rfds)) {

                        PRINTVERBOSE1("rpmsgThreadFxn: getting from fd %d\n",
                                      TransportRpmsg_module->inFds[curFdx])

                        /* transport input fd was signalled: get the message */
                        tmpStatus = transportGet(
                            TransportRpmsg_module->inFds[curFdx], &retMsg);
                        if (tmpStatus < 0) {
                            printf("rpmsgThreadFxn: transportGet failed.");
                            status = MessageQ_E_FAIL;
                        }
                        else {
                            queueId = MessageQ_getDstQueue(retMsg);

                            PRINTVERBOSE1("rpmsgThreadFxn: got message, "
                                    "delivering to queueId 0x%x\n", queueId)

                            MessageQ_put(queueId, retMsg);
                        }

                        lastFdx = (curFdx + 1) % nfds;

                        break;
                    }

                    curFdx = (curFdx + 1) % nfds;
                } while (curFdx != lastFdx);
            }
        }
    }

    return (void *)status;
}

/*
 * ======== transportGet ========
 *  Retrieve a message waiting in the socket's queue.
*/
static Int transportGet(int sock, MessageQ_Msg *retMsg)
{
    Int           status    = MessageQ_S_SUCCESS;
    MessageQ_Msg  msg;
    struct sockaddr_rpmsg fromAddr;  /* [Socket address of sender] */
    unsigned int  len;
    int           byteCount;

    /*
     * We have no way of peeking to see what message size we'll get, so we
     * allocate a message of max size to receive contents from the rpmsg socket
     * (currently, a copy transport)
     */
    msg = MessageQ_alloc(0, MESSAGEQ_RPMSG_MAXSIZE);
    if (!msg) {
        status = MessageQ_E_MEMORY;
        goto exit;
    }

    memset(&fromAddr, 0, sizeof (fromAddr));
    len = sizeof (fromAddr);

    byteCount = recvfrom(sock, msg, MESSAGEQ_RPMSG_MAXSIZE, 0,
                         (struct sockaddr *)&fromAddr, &len);
    if (len != sizeof (fromAddr)) {
        printf("recvfrom: got bad addr len (%d)\n", len);
        status = MessageQ_E_FAIL;
        goto exit;
    }
    if (byteCount < 0) {
        printf("recvfrom failed: %s (%d)\n", strerror(errno), errno);
        status = MessageQ_E_FAIL;
        goto exit;
    }
    else {
         /*
          * Update the allocated message size (even though this may waste
          * space when the actual message is smaller than the maximum rpmsg
          * size, the message will be freed soon anyway, and it avoids an
          * extra copy).
          */
         msg->msgSize = byteCount;

         /*
          * If the message received was statically allocated, reset the
          * heapId, so the app can free it.
          */
         if (msg->heapId == MessageQ_STATICMSG)  {
             msg->heapId = 0;  /* for a copy transport, heap id is 0. */
         }
    }

    PRINTVERBOSE1("transportGet: recvfrom socket: fd: %d\n", sock)
    PRINTVERBOSE3("\tReceived a msg: byteCount: %d, rpmsg addr: %d, rpmsg "
            "proc: %d\n", byteCount, fromAddr.addr, fromAddr.vproc_id)
    PRINTVERBOSE2("\tMessage Id: %d, Message size: %d\n", msg->msgId,
            msg->msgSize)

    *retMsg = msg;

exit:
    return status;
}

Void bindFdToQueueIndex(TransportRpmsg_Object *obj, Int fd, UInt16 queuePort)
{
    Int *queues;
    Int *oldQueues;
    UInt oldSize;
    UInt queueIndex;

    /* subtract port offset from queue index */
    queueIndex = queuePort - MessageQ_PORTOFFSET;

    if (queueIndex >= obj->numQueues) {
        PRINTVERBOSE1("TransportRpmsg_bind: growing numQueues to %d\n",
                queueIndex + TransportRpmsg_GROWSIZE)

        /* allocate larget table */
        oldSize = obj->numQueues * sizeof (Int);
        queues = calloc(queueIndex + TransportRpmsg_GROWSIZE, sizeof(Int));

        /* copy contents from old table int new table */
        memcpy(queues, obj->qIndexToFd, oldSize);

        /* swap in new table, delete old table */
        oldQueues = obj->qIndexToFd;
        obj->qIndexToFd = queues;
        obj->numQueues = queueIndex + TransportRpmsg_GROWSIZE;
        free(oldQueues);
    }

    /* add new entry */
    obj->qIndexToFd[queueIndex] = fd;
}

Void unbindQueueIndex(TransportRpmsg_Object *obj, UInt16 queuePort)
{
    UInt queueIndex;

    /* subtract port offset from queue index */
    queueIndex = queuePort - MessageQ_PORTOFFSET;

    /* clear table entry */
    obj->qIndexToFd[queueIndex] = 0;
}

Int queueIndexToFd(TransportRpmsg_Object *obj, UInt16 queuePort)
{
    UInt queueIndex;

    /* subtract port offset from queue index */
    queueIndex = queuePort - MessageQ_PORTOFFSET;

    /* return file descriptor */
    return (obj->qIndexToFd[queueIndex]);
}

/*
 *  ======== TransportRpmsg_Factory_create ========
 *  Create the transport instances
 *
 *  Attach to all remote processors. For now, must attach to
 *  at least one to tolerate MessageQ_E_RESOURCE failures.
 *
 *  This function implements the IPC Factory interface, so it
 *  returns Ipc status codes.
 */
Int TransportRpmsg_Factory_create(Void)
{
    Int     status;
    Int     attachStatus;
    Int     i;
    UInt16  procId;
    Int32   attachedAny;
    UInt16  clusterSize;
    UInt16  clusterBase;

    TransportRpmsg_Handle      *inst;
    TransportRpmsg_Handle       transport;
    TransportRpmsg_Params       params;
    IMessageQTransport_Handle   iMsgQTrans;


    status = Ipc_S_SUCCESS;
    attachedAny = FALSE;

    /* needed to enumerate processors in cluster */
    clusterSize = MultiProc_getNumProcsInCluster();
    clusterBase = MultiProc_getBaseIdOfCluster();

    /* allocate the instance array */
    inst = calloc(clusterSize, sizeof(TransportRpmsg_Handle));

    if (inst == NULL) {
        printf("Error: TransportRpmsg_Factory_create failed, no memory\n");
        status = Ipc_E_MEMORY;
        goto exit;
    }

    TransportRpmsg_module->inst = inst;

    /* create transport instance for all processors in cluster */
    for (i = 0, procId = clusterBase; i < clusterSize; i++, procId++) {

        if (MultiProc_self() == procId) {
            continue;
        }

        params.rprocId = procId;
        transport = TransportRpmsg_create(&params, &attachStatus);

        if (transport != NULL) {
            iMsgQTrans = TransportRpmsg_upCast(transport);
            MessageQ_registerTransport(iMsgQTrans, procId, 0);
            attachedAny = TRUE;
        }
        else {
            if (attachStatus == MessageQ_E_RESOURCE) {
                continue;
            }
            printf("TransportRpmsg_Factory_create: failed to attach to "
                    "procId=%d status=%d\n", procId, attachStatus);
            status = Ipc_E_FAIL;
            break;
        }

        TransportRpmsg_module->inst[i] = transport;
    }

    if (!attachedAny) {
        status = Ipc_E_FAIL;
    }

exit:
    return (status);
}

/*
 *  ======== TransportRpmsg_Factory_delete ========
 *  Finalize the transport instances
 */
Void TransportRpmsg_Factory_delete(Void)
{
    Int     i;
    UInt16  procId;
    UInt16  clusterSize;
    UInt16  clusterBase;

    /* needed to enumerate processors in cluster */
    clusterSize = MultiProc_getNumProcsInCluster();
    clusterBase = MultiProc_getBaseIdOfCluster();

    /* detach from all remote processors, assuming they are up */
    for (i = 0, procId = clusterBase; i < clusterSize; i++, procId++) {

        if (MultiProc_self() == procId) {
            continue;
        }

        if (TransportRpmsg_module->inst[i] != NULL) {
            MessageQ_unregisterTransport(procId, 0);
            TransportRpmsg_delete(&(TransportRpmsg_module->inst[i]));
        }
    }

    return;
}
