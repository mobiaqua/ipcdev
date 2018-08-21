/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
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
 *  ======== rsc_table_am65xx_r5f.h ========
 *
 *  Define the resource table entries for all R5F cores. This will be
 *  incorporated into corresponding base images, and used by the remoteproc
 *  on the host-side to allocated/reserve resources.
 *
 */

#ifndef _RSC_TABLE_AM65XX_R5F_H_
#define _RSC_TABLE_AM65XX_R5F_H_

#include "rsc_types.h"

#define R5F_MEM_TEXT            0x9C200000
#define R5F_MEM_DATA            0x9C300000

#define R5F_MEM_IPC_DATA        0x9C100000
#define R5F_MEM_IPC_VRING       0x9C000000
#define R5F_MEM_RPMSG_VRING0    0x9C000000
#define R5F_MEM_RPMSG_VRING1    0x9C010000
#define R5F_MEM_VRING_BUFS0     0x9C040000
#define R5F_MEM_VRING_BUFS1     0x9C080000

#define R5F_MEM_IPC_VRING_SIZE  SZ_1M
#define R5F_MEM_IPC_DATA_SIZE   (SZ_1M)

#define R5F_MEM_TEXT_SIZE       (SZ_1M)

#define R5F_MEM_DATA_SIZE       (SZ_1M)

#define R5F_NUM_ENTRIES 6

/*
 * Assign fixed RAM addresses to facilitate a fixed MMU table.
 * PHYS_MEM_IPC_VRING & PHYS_MEM_IPC_DATA MUST be together.
 */
/* See CMA BASE addresses in Linux side: arch/arm/mach-omap2/remoteproc.c */
#define PHYS_MEM_IPC_VRING      0x9C000000

/*
 * Sizes of the virtqueues (expressed in number of buffers supported,
 * and must be power of 2)
 */
#define R5F_RPMSG_VQ0_SIZE      256
#define R5F_RPMSG_VQ1_SIZE      256

/* flip up bits whose indices represent features we support */
#define RPMSG_R5F_C0_FEATURES   1

struct my_resource_table {
    struct resource_table base;

    UInt32 offset[R5F_NUM_ENTRIES];  /* Should match 'num' in actual definition */

    /* rpmsg vdev entry */
    struct fw_rsc_vdev rpmsg_vdev;
    struct fw_rsc_vdev_vring rpmsg_vring0;
    struct fw_rsc_vdev_vring rpmsg_vring1;

    /* ipcdata carveout entry */
    struct fw_rsc_carveout ipcdata_cout;

    /* text carveout entry */
    struct fw_rsc_carveout text_cout;

    /* data carveout entry */
    struct fw_rsc_carveout data_cout;

    /* trace entry */
    struct fw_rsc_trace trace;

    /* devmem entry */
    struct fw_rsc_devmem devmem0;
};

#define TRACEBUFADDR (UInt32)&ti_trace_SysMin_Module_State_0_outbuf__A
#define TRACEBUFSIZE 0x8000

#pragma DATA_SECTION(ti_ipc_remoteproc_ResourceTable, ".resource_table")
#pragma DATA_ALIGN(ti_ipc_remoteproc_ResourceTable, 4096)

const struct my_resource_table ti_ipc_remoteproc_ResourceTable = {
    1,      /* we're the first version that implements this */
    R5F_NUM_ENTRIES,     /* number of entries in the table */
    0, 0,   /* reserved, must be zero */
    /* offsets to entries */
    {
        offsetof(struct my_resource_table, rpmsg_vdev),
        offsetof(struct my_resource_table, ipcdata_cout),
        offsetof(struct my_resource_table, text_cout),
        offsetof(struct my_resource_table, data_cout),
        offsetof(struct my_resource_table, trace),
        offsetof(struct my_resource_table, devmem0),
    },

    /* rpmsg vdev entry */
    {
        TYPE_VDEV, VIRTIO_ID_RPMSG, 0,
        RPMSG_R5F_C0_FEATURES, 0, 0, 0, 2, { 0, 0 },
        /* no config data */
    },
    /* the two vrings */
    { R5F_MEM_RPMSG_VRING0, 4096, R5F_RPMSG_VQ0_SIZE, 1, 0 },
    { R5F_MEM_RPMSG_VRING1, 4096, R5F_RPMSG_VQ1_SIZE, 2, 0 },

    {
        TYPE_CARVEOUT,
        R5F_MEM_IPC_DATA, 0,
        R5F_MEM_IPC_DATA_SIZE, 0, 0, "R5F_MEM_IPC_DATA",
    },

    {
        TYPE_CARVEOUT,
        R5F_MEM_TEXT, 0,
        R5F_MEM_TEXT_SIZE, 0, 0, "R5F_MEM_TEXT",
    },

    {
        TYPE_CARVEOUT,
        R5F_MEM_DATA, 0,
        R5F_MEM_DATA_SIZE, 0, 0, "R5F_MEM_DATA",
    },

    {
        TYPE_TRACE, TRACEBUFADDR, TRACEBUFSIZE, 0, "trace:r5f0",
    },

    {
        TYPE_DEVMEM,
        R5F_MEM_IPC_VRING, PHYS_MEM_IPC_VRING,
        R5F_MEM_IPC_VRING_SIZE, 0, 0, "R5F_MEM_IPC_VRING",
    },

};

#endif /* _RSC_TABLE_AM65XX_R5F_H_ */
