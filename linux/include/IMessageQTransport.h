/*
 *  ======== IMessageQTransport.h ========
 */

#ifndef IMESSAGEQTRANSPORT_H
#define IMESSAGEQTRANSPORT_H

#include <ITransport.h>

/* opaque instance handle */
typedef struct IMessageQTransport_Object *IMessageQTransport_Handle;

#define IMessageQTransport_TypeId 0x02

/* virtual functions */
typedef struct IMessageQTransport_Fxns {
    Int (*bind)(void *handle, UInt32 queueId);
    Int (*unbind)(void *handle, UInt32 queueId);
    Bool (*put)(void *handle, Ptr msg);
} IMessageQTransport_Fxns;

/* abstract instance object */
typedef struct IMessageQTransport_Object {
    ITransport_Object base;             /* inheritance */
    IMessageQTransport_Fxns *fxns;      /* virtual functions */
} IMessageQTransport_Object;

/* function stubs */
static inline
Int IMessageQTransport_bind(IMessageQTransport_Handle inst, UInt32 queueId)
{
    IMessageQTransport_Object *obj = (IMessageQTransport_Object *)inst;
    return obj->fxns->bind((void *)inst, queueId);
}

static inline
Int IMessageQTransport_unbind(IMessageQTransport_Handle inst, UInt32 queueId)
{
    IMessageQTransport_Object *obj = (IMessageQTransport_Object *)inst;
    return obj->fxns->unbind((void *)inst, queueId);
}

static inline
Bool IMessageQTransport_put(IMessageQTransport_Handle inst, Ptr msg)
{
    IMessageQTransport_Object *obj = (IMessageQTransport_Object *)inst;
    return obj->fxns->put((void *)inst, msg);
}

/* instance convertors */
static inline
ITransport_Handle IMessageQTransport_upCast(IMessageQTransport_Handle inst)
{
    IMessageQTransport_Object *obj = (IMessageQTransport_Object *)inst;
    return (ITransport_Handle)&obj->base;
}

static inline
IMessageQTransport_Handle IMessageQTransport_downCast(ITransport_Handle base)
{
    return (IMessageQTransport_Handle)base;
}

#endif /* IMESSAGEQTRANSPORT_H */

