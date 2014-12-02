/*
 *  ======== INetworkTransport.h ========
 */

#ifndef INETWORKTRANSPORT_H
#define INETWORKTRANSPORT_H

#include <ITransport.h>

/* opaque instance handle */
typedef struct INetworkTransport_Object *INetworkTransport_Handle;

#define INetworkTransport_TypeId 0x03

/* virtual functions */
typedef struct INetworkTransport_Fxns {
    Int (*bind)(void *handle, UInt32 queueId);
    Int (*unbind)(void *handle, UInt32 queueId);
    Bool (*put)(void *handle, Ptr msg);
} INetworkTransport_Fxns;

/* abstract instance object */
typedef struct INetworkTransport_Object {
    ITransport_Object base;             /* inheritance */
    INetworkTransport_Fxns *fxns;       /* virtual functions */
} INetworkTransport_Object;

/* function stubs */
static inline
Bool INetworkTransport_bind(INetworkTransport_Handle inst, UInt32 queueId)
{
    INetworkTransport_Object *obj = (INetworkTransport_Object *)inst;
    return obj->fxns->bind((void *)inst, queueId);
}

static inline
Bool INetworkTransport_unbind(INetworkTransport_Handle inst, UInt32 queueId)
{
    INetworkTransport_Object *obj = (INetworkTransport_Object *)inst;
    return obj->fxns->unbind((void *)inst, queueId);
}

static inline
Bool INetworkTransport_put(INetworkTransport_Handle inst, Ptr msg)
{
    INetworkTransport_Object *obj = (INetworkTransport_Object *)inst;
    return obj->fxns->put((void *)inst, msg);
}

/* instance convertors */
ITransport_Handle INetworkTransport_upCast(INetworkTransport_Handle inst)
{
    INetworkTransport_Object *obj = (INetworkTransport_Object *)inst;
    return (ITransport_Handle)&obj->base;
}

INetworkTransport_Handle INetworkTransport_downCast(ITransport_Handle base)
{
    return (INetworkTransport_Handle)base;
}

#endif /* INETWORKTRANSPORT_H */

