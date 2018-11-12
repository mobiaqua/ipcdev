/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* The following are required to avoid issues with types used in CSL */
#undef xdc__strict
/* Define SOC_AM65XX for header files below */
#define SOC_AM65XX

#include <ti/csl/csl_types.h>
#include <ti/drv/sciclient/sciclient.h>

#include "package/internal/NotifySciClient.xdc.h"

/* Ideally want to use fixed timeout. Work around is to use wait forever */
/* #define NOTIFY_SCICLIENT_RESP_TIMEOUT 100  */

#define NOTIFY_SCICLIENT_RESP_TIMEOUT SCICLIENT_SERVICE_WAIT_FOREVER

/*********************************************************************
 * @fn      NotifySciClient_Init
 *
 * @brief   Initialises NotifySciclient
 *          Currently only checking communication with system core.
 *
 *
 * @return  0 : Success; -1 for failures
 */
Int32 NotifySciClient_Init(void)
{
    int32_t status = 0;

    /* Setup Request for Version check */
    const Sciclient_ReqPrm_t      reqPrm =
    {
        .messageType = TISCI_MSG_VERSION,
        .flags = TISCI_MSG_FLAG_AOP,
        .pReqPayload = NULL,
        .reqPayloadSize = 0,
        .timeout = NOTIFY_SCICLIENT_RESP_TIMEOUT
    };

    struct tisci_msg_version_resp response;
    /* Setup Response parameters */
    Sciclient_RespPrm_t           respPrm =
    {
        .flags = 0,
        .pRespPayload = (uint8_t *) &response,
        .respPayloadSize = sizeof (response)
    };

    /* Check version check to TISCI connection */
    status = Sciclient_service(&reqPrm, &respPrm);
    if (CSL_PASS == status)
    {
        if (respPrm.flags != TISCI_MSG_FLAG_ACK)
        {
            return -1;
        }
    } else {
        return -1;
    }
    return status;
}

/*********************************************************************
 * @fn      NotifySciClient_IrqSet
 *
 * @brief   Configures interrupt routes by requesting the system core
 *
 * @param1  coreIndex: core index
 * @param2  mailboxClusterIndex: Mailbox Cluster index
 * @param3  mailboxUserIndex: Mailbox User index
 * @param4  intNumber: Local cpu interrupt number
 *
 * @return  0 : Success; -1 for failures
 */
Int32 NotifySciClient_IrqSet(NotifySciClient_CoreIndex coreIndex,
               NotifySciClient_SourceIdIndex mailboxClusterIndex,
               NotifySciClient_MailboxIndex mailboxUserIndex,
               UInt32 intNumber)
{
    int32_t status = 0;
    struct tisci_msg_rm_irq_set_resp resp;

    /* Indexed list of dst ids */
    const int32_t map_dst_id[] =
    {
        /* NOTE: This list should match the Core index */
        TISCI_DEV_GIC0,
        TISCI_DEV_MCU_ARMSS0_CPU0,
        TISCI_DEV_MCU_ARMSS0_CPU1
    };
    /* Indexed list of src ids */
    const uint16_t map_src_id[] =
    {
        TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER0,
        TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER1,
        TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER2
    };

    /* Indexed list of host ids */
    const uint16_t map_host_id[] =
    {
        TISCI_HOST_ID_A53_0,
        TISCI_HOST_ID_R5_0,
        TISCI_HOST_ID_R5_1
    };

    /* Initialize unused parameters */
    struct tisci_msg_rm_irq_set_req irq_set_req =
    {
        .ia_id          = 0,
        .vint           = 0,
        .global_event   = 0,
        .vint_status_bit_index = 0,
    };

    /* Request irq set for specified interrupt source */
    irq_set_req.valid_params = 0x80000003 ; /* Sets bits for secondary host, dst_id, dst_host_irq */
    irq_set_req.src_id = map_src_id[mailboxClusterIndex];
    irq_set_req.src_index = mailboxUserIndex;
    irq_set_req.dst_id = map_dst_id[coreIndex];
    irq_set_req.dst_host_irq = intNumber;
    irq_set_req.secondary_host = map_host_id[coreIndex];

    /* Call irq Set */
    if (CSL_PASS != Sciclient_rmIrqSet(&irq_set_req, &resp, NOTIFY_SCICLIENT_RESP_TIMEOUT ))
    {
        return -1;
    }

    return status;
}

/*********************************************************************
 * @fn      NotifySciClient_IrqRelease
 *
 * @brief   Releases interrupt routes by requesting the system core
 *
 * @param1  coreIndex: core index
 * @param2  mailboxClusterIndex: Mailbox Cluster index
 * @param3  mailboxUserIndex: Mailbox User index
 * @param4  intNumber: Local cpu interrupt number
 *
 * @return  0 : Success; -1 for failures
 */
Int32 NotifySciClient_IrqRelease(NotifySciClient_CoreIndex coreIndex,
               NotifySciClient_SourceIdIndex mailboxClusterIndex,
               NotifySciClient_MailboxIndex mailboxUserIndex,
               UInt32 intNumber)
{
    int32_t status = 0;

    /* Indexed list of dst ids */
    const int32_t map_dst_id[] =
    {
        /* NOTE: This list should match the Core index */
        TISCI_DEV_GIC0,
        TISCI_DEV_MCU_ARMSS0_CPU0,
        TISCI_DEV_MCU_ARMSS0_CPU1
    };
    /* Indexed list of src ids */
    const uint16_t map_src_id[] =
    {
        TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER0,
        TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER1,
        TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER2
    };

    /* Indexed list of host ids */
    const uint16_t map_host_id[] =
    {
        TISCI_HOST_ID_A53_0,
        TISCI_HOST_ID_R5_0,
        TISCI_HOST_ID_R5_1
    };

    /* Initialize unused parameters */
    struct tisci_msg_rm_irq_release_req irq_release_req =
    {
        .ia_id          = 0,
        .vint           = 0,
        .global_event   = 0,
        .vint_status_bit_index = 0,
    };

    /* Request irq release for specified interrupt source */
    irq_release_req.valid_params = 0x80000003 ; /* Sets bits for secondary host, dst_id, dst_host_irq */
    irq_release_req.src_id = map_src_id[mailboxClusterIndex];
    irq_release_req.src_index = mailboxUserIndex;
    irq_release_req.dst_id = map_dst_id[coreIndex];
    irq_release_req.dst_host_irq = intNumber;
    irq_release_req.secondary_host = map_host_id[coreIndex];

    /* Call irq Release */
    if (CSL_PASS != Sciclient_rmIrqRelease(&irq_release_req, NOTIFY_SCICLIENT_RESP_TIMEOUT ))
    {
        return -1;
    }

    return status;
}

/*********************************************************************
 * @fn      NotifySciClient_getIntNumRange
 *
 * @brief   Get range of interrupt Numbers available
 *
 * @param1  coreIndex: core index
 * @param2  coreIndex: secondary_host
 * @param1  rangeStartP: Pointer to range start
 * @param2  rangeNumP: Pointer to number of interrupts
 *
 * @return  0 : Success; -1 for failures
 */
Int32 NotifySciClient_getIntNumRange(NotifySciClient_CoreIndex coreIndex,
                                     NotifySciClient_SecondaryHost secondaryHost,
                                     UInt16 *rangeStartP,
                                     UInt16 *rangeNumP)
{
    int32_t status = 0;
    struct tisci_msg_rm_get_resource_range_resp resp;
    struct tisci_msg_rm_get_resource_range_req get_resource_range_req;
   /* Indexed list of req type */
    const uint16_t req_type[] =
    {
        /* NOTE: This list should match the Core index */
        TISCI_RESASG_TYPE_GIC_IRQ,
        TISCI_RESASG_TYPE_PULSAR_C0_IRQ,
        TISCI_RESASG_TYPE_PULSAR_C1_IRQ
    };
   /* Indexed list of req subtype */
    const uint16_t req_subtype[] =
    {
        /* NOTE: This list should match the Core index */
        TISCI_RESASG_SUBTYPE_GIC_IRQ_MAIN_NAV_SET1,
        TISCI_RESASG_SUBTYPE_PULSAR_C0_IRQ_MAIN2MCU_LVL,
        TISCI_RESASG_SUBTYPE_PULSAR_C1_IRQ_MAIN2MCU_LVL
    };

    /* Indexed list of host ids */
    const uint16_t map_host_id[] =
    {
        TISCI_HOST_ID_A53_0,
        TISCI_HOST_ID_R5_0,
        TISCI_HOST_ID_R5_1
    };

    get_resource_range_req.type = req_type[coreIndex];
    get_resource_range_req.subtype = req_subtype[coreIndex];
    if (secondaryHost == NotifySciClient_SECONDARYHOST_UNUSED) {
        get_resource_range_req.secondary_host = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;
    } else if (secondaryHost == NotifySciClient_SECONDARYHOST_SPECIFIC_HOST) {
        get_resource_range_req.secondary_host = map_host_id[coreIndex];
    } else {
        get_resource_range_req.secondary_host = TISCI_HOST_ID_ALL;
    }
    /* Get interrupt number range */
    status =  Sciclient_rmGetResourceRange(
                &get_resource_range_req,
                &resp,
                NOTIFY_SCICLIENT_RESP_TIMEOUT);
    if (CSL_PASS == status)
    {
        *rangeStartP = resp.range_start;
        *rangeNumP = resp.range_num;
    } else {
        return -1;
    }
    return status;
}
