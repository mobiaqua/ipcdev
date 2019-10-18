/*
 * Copyright (c) 2012-2019, Texas Instruments Incorporated
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
 *  ======== rsc_table_vayu_ipu.h ========
 *
 *  Define the resource table entries for all IPU cores. This will be
 *  incorporated into corresponding base images, and used by the remoteproc
 *  on the host-side to allocated/reserve resources.
 *
 */

#ifndef _RSC_TABLE_VAYU_IPU_H_
#define _RSC_TABLE_VAYU_IPU_H_

#include "rsc_types.h"

/* IPU Memory Map */
#define L4_DRA7XX_BASE          0x4A000000

/* L4_CFG & L4_WKUP */
#define L4_PERIPHERAL_L4CFG     (L4_DRA7XX_BASE)
#define IPU_PERIPHERAL_L4CFG    0x6A000000

#define L4_PERIPHERAL_L4PER1    0x48000000
#define IPU_PERIPHERAL_L4PER1   0x68000000

#define L4_PERIPHERAL_L4PER2    0x48400000
#define IPU_PERIPHERAL_L4PER2   0x68400000

#define L4_PERIPHERAL_L4PER3    0x48800000
#define IPU_PERIPHERAL_L4PER3   0x68800000

#define L4_PERIPHERAL_L4EMU     0x54000000
#define IPU_PERIPHERAL_L4EMU    0x74000000

#define L3_PERIPHERAL_DMM       0x4E000000
#define IPU_PERIPHERAL_DMM      0x6E000000

#define L3_IVAHD_CONFIG         0x5A000000
#define IPU_IVAHD_CONFIG        0x7A000000

#define L3_IVAHD_SL2            0x5B000000
#define IPU_IVAHD_SL2           0x7B000000

#define L3_TILER_MODE_0_1       0x60000000
#define IPU_TILER_MODE_0_1      0xA0000000

#define L3_TILER_MODE_2         0x70000000
#define IPU_TILER_MODE_2        0xB0000000

#define L3_TILER_MODE_3         0x78000000
#define IPU_TILER_MODE_3        0xB8000000
#if defined(VAYU_IPU_1)
#define IPU_MEM_TEXT            0x0
#elif defined(VAYU_IPU_2)
#define IPU_MEM_TEXT0            0x0
#define IPU_MEM_TEXT1            0x0200000
#endif

#if defined(VAYU_IPU_1)
#define IPU_MEM_DATA0            0x80000000
#define IPU_MEM_DATA1            0x80100000
#elif defined(VAYU_IPU_2)
#define IPU_MEM_DATA0            0x80000000
#define IPU_MEM_DATA1            0x80800000
#define IPU_MEM_DATA2            0x81800000
#define IPU_MEM_DATA3            0x82800000
#endif

#define IPU_MEM_IPC_DATA        0x9F000000
#define IPU_MEM_IPC_VRING       0x60000000
#define IPU_MEM_RPMSG_VRING0    0x60000000
#define IPU_MEM_RPMSG_VRING1    0x60004000
#define IPU_MEM_VRING_BUFS0     0x60040000
#define IPU_MEM_VRING_BUFS1     0x60080000

#define IPU_MEM_IPC_VRING_SIZE  SZ_1M
#define IPU_MEM_IPC_DATA_SIZE   SZ_1M

#if defined(VAYU_IPU_1)
#define IPU_MEM_TEXT_SIZE       (SZ_1M)
#elif defined(VAYU_IPU_2)
#define IPU_MEM_TEXT0_SIZE       (SZ_1M * 2)
#define IPU_MEM_TEXT1_SIZE       (SZ_1M * 4)
#endif

#if defined(VAYU_IPU_1)
#define IPU_MEM_DATA0_SIZE       (SZ_1M * 1)
#define IPU_MEM_DATA1_SIZE       (SZ_1M * 4)
#elif defined(VAYU_IPU_2)
#define IPU_MEM_DATA0_SIZE       (SZ_1M * 8)
#define IPU_MEM_DATA1_SIZE       (SZ_1M * 16)
#define IPU_MEM_DATA2_SIZE       (SZ_1M * 16)
#define IPU_MEM_DATA3_SIZE       (SZ_1M * 8)
#endif

/*
 * NOTE:
 * To avoid issues with allocation failures with Linux carveout regions, need
 * to use the RSC_CARVEOUT entries with power of 2 page order sizes and aligned
 * on the same page order.
 * The size and the alignment order of entries in the resource table plays a
 * part in avoiding gaps in allocation
 */
#if defined(VAYU_IPU_1)
#define PHYS_MEM_IPC_VRING      0x9D000000
#elif defined (VAYU_IPU_2)
#define PHYS_MEM_IPC_VRING      0x95800000
#endif

/*
 * Sizes of the virtqueues (expressed in number of buffers supported,
 * and must be power of 2)
 */
#define IPU_RPMSG_VQ0_SIZE      256
#define IPU_RPMSG_VQ1_SIZE      256

/* flip up bits whose indices represent features we support */
#define RPMSG_IPU_C0_FEATURES   1

#if defined(VAYU_IPU_1)
#define NUM_RSC_ENTRIES 18
#elif defined (VAYU_IPU_2)
#define NUM_RSC_ENTRIES 21
#endif

struct my_resource_table {
    struct resource_table base;

    UInt32 offset[NUM_RSC_ENTRIES];  /* Should match 'num' in actual definition */

    /* rpmsg vdev entry */
    struct fw_rsc_vdev rpmsg_vdev;
    struct fw_rsc_vdev_vring rpmsg_vring0;
    struct fw_rsc_vdev_vring rpmsg_vring1;

    /* ipcdata carveout entry */
    struct fw_rsc_carveout ipcdata_cout;

    /* text carveout entry */
#if defined(VAYU_IPU_1)
    struct fw_rsc_carveout text_cout;
#elif defined (VAYU_IPU_2)
    struct fw_rsc_carveout text0_cout;
    struct fw_rsc_carveout text1_cout;
#endif
    /* data carveout entries */
    struct fw_rsc_carveout data0_cout;
    struct fw_rsc_carveout data1_cout;
#if defined(VAYU_IPU_2)
    struct fw_rsc_carveout data2_cout;
    struct fw_rsc_carveout data3_cout;
#endif

    /* trace entry */
    struct fw_rsc_trace trace;

    /* devmem entry */
    struct fw_rsc_devmem devmem0;

    /* devmem entry */
    struct fw_rsc_devmem devmem1;

    /* devmem entry */
    struct fw_rsc_devmem devmem2;

    /* devmem entry */
    struct fw_rsc_devmem devmem3;

    /* devmem entry */
    struct fw_rsc_devmem devmem4;

    /* devmem entry */
    struct fw_rsc_devmem devmem5;

    /* devmem entry */
    struct fw_rsc_devmem devmem6;

    /* devmem entry */
    struct fw_rsc_devmem devmem7;

    /* devmem entry */
    struct fw_rsc_devmem devmem8;

    /* devmem entry */
    struct fw_rsc_devmem devmem9;

    /* devmem entry */
    struct fw_rsc_devmem devmem10;

    /* devmem entry */
    struct fw_rsc_devmem devmem11;
};

#define TRACEBUFADDR (UInt32)&ti_trace_SysMin_Module_State_0_outbuf__A

#pragma DATA_SECTION(ti_ipc_remoteproc_ResourceTable, ".resource_table")
#pragma DATA_ALIGN(ti_ipc_remoteproc_ResourceTable, 4096)

struct my_resource_table ti_ipc_remoteproc_ResourceTable = {
    1,      /* we're the first version that implements this */
    NUM_RSC_ENTRIES,     /* number of entries in the table */
    0, 0,   /* reserved, must be zero */
    /* offsets to entries */
    {
        offsetof(struct my_resource_table, rpmsg_vdev),
        offsetof(struct my_resource_table, ipcdata_cout),
#if defined(VAYU_IPU_1)
        offsetof(struct my_resource_table, text_cout),
#elif defined (VAYU_IPU_2)
        offsetof(struct my_resource_table, text0_cout),
        offsetof(struct my_resource_table, text1_cout),
#endif
        offsetof(struct my_resource_table, data0_cout),
        offsetof(struct my_resource_table, data1_cout),
#if defined(VAYU_IPU_2)
        offsetof(struct my_resource_table, data2_cout),
        offsetof(struct my_resource_table, data3_cout),
#endif
        offsetof(struct my_resource_table, trace),
        offsetof(struct my_resource_table, devmem0),
        offsetof(struct my_resource_table, devmem1),
        offsetof(struct my_resource_table, devmem2),
        offsetof(struct my_resource_table, devmem3),
        offsetof(struct my_resource_table, devmem4),
        offsetof(struct my_resource_table, devmem5),
        offsetof(struct my_resource_table, devmem6),
        offsetof(struct my_resource_table, devmem7),
        offsetof(struct my_resource_table, devmem8),
        offsetof(struct my_resource_table, devmem9),
        offsetof(struct my_resource_table, devmem10),
        offsetof(struct my_resource_table, devmem11),
    },

    /* rpmsg vdev entry */
    {
        TYPE_VDEV, VIRTIO_ID_RPMSG, 0,
        RPMSG_IPU_C0_FEATURES, 0, 0, 0, 2, { 0, 0 },
        /* no config data */
    },
    /* the two vrings */
    { IPU_MEM_RPMSG_VRING0, 4096, IPU_RPMSG_VQ0_SIZE, 1, 0 },
    { IPU_MEM_RPMSG_VRING1, 4096, IPU_RPMSG_VQ1_SIZE, 2, 0 },

    {
        TYPE_CARVEOUT,
        IPU_MEM_IPC_DATA, 0,
        IPU_MEM_IPC_DATA_SIZE, 0, 0, "IPU_MEM_IPC_DATA",
    },

#if defined(VAYU_IPU_1)
    {
        TYPE_CARVEOUT,
        IPU_MEM_TEXT, 0,
        IPU_MEM_TEXT_SIZE, 0, 0, "IPU_MEM_TEXT",
    },
#elif defined (VAYU_IPU_2)
    {
        TYPE_CARVEOUT,
        IPU_MEM_TEXT0, 0,
        IPU_MEM_TEXT0_SIZE, 0, 0, "IPU_MEM_TEXT0",
    },

    {
        TYPE_CARVEOUT,
        IPU_MEM_TEXT1, 0,
        IPU_MEM_TEXT1_SIZE, 0, 0, "IPU_MEM_TEXT1",
    },
#endif

    {
        TYPE_CARVEOUT,
        IPU_MEM_DATA0, 0,
        IPU_MEM_DATA0_SIZE, 0, 0, "IPU_MEM_DATA0",
    },

    {
        TYPE_CARVEOUT,
        IPU_MEM_DATA1, 0,
        IPU_MEM_DATA1_SIZE, 0, 0, "IPU_MEM_DATA1",
    },

#if defined(VAYU_IPU_2)
    {
        TYPE_CARVEOUT,
        IPU_MEM_DATA2, 0,
        IPU_MEM_DATA2_SIZE, 0, 0, "IPU_MEM_DATA2",
    },

    {
        TYPE_CARVEOUT,
        IPU_MEM_DATA3, 0,
        IPU_MEM_DATA3_SIZE, 0, 0, "IPU_MEM_DATA3",
    },
#endif

    {
        TYPE_TRACE, TRACEBUFADDR, 0x8000, 0, "trace:sysm3",
    },

    {
        TYPE_DEVMEM,
        IPU_MEM_IPC_VRING, PHYS_MEM_IPC_VRING,
        IPU_MEM_IPC_VRING_SIZE, 0, 0, "IPU_MEM_IPC_VRING",
    },

    {
        TYPE_DEVMEM,
        IPU_TILER_MODE_0_1, L3_TILER_MODE_0_1,
        SZ_256M, 0, 0, "IPU_TILER_MODE_0_1",
    },

    {
        TYPE_DEVMEM,
        IPU_TILER_MODE_2, L3_TILER_MODE_2,
        SZ_128M, 0, 0, "IPU_TILER_MODE_2",
    },

    {
        TYPE_DEVMEM,
        IPU_TILER_MODE_3, L3_TILER_MODE_3,
        SZ_128M, 0, 0, "IPU_TILER_MODE_3",
    },

    {
        TYPE_DEVMEM,
        IPU_PERIPHERAL_L4CFG, L4_PERIPHERAL_L4CFG,
        SZ_16M, 0, 0, "IPU_PERIPHERAL_L4CFG",
    },

    {
        TYPE_DEVMEM,
        IPU_PERIPHERAL_L4PER1, L4_PERIPHERAL_L4PER1,
        SZ_2M, 0, 0, "IPU_PERIPHERAL_L4PER1",
    },

    {
        TYPE_DEVMEM,
        IPU_PERIPHERAL_L4PER2, L4_PERIPHERAL_L4PER2,
        SZ_4M, 0, 0, "IPU_PERIPHERAL_L4PER2",
    },

    {
        TYPE_DEVMEM,
        IPU_PERIPHERAL_L4PER3, L4_PERIPHERAL_L4PER3,
        SZ_8M, 0, 0, "IPU_PERIPHERAL_L4PER3",
    },

    {
        TYPE_DEVMEM,
        IPU_PERIPHERAL_L4EMU, L4_PERIPHERAL_L4EMU,
        SZ_16M, 0, 0, "IPU_PERIPHERAL_L4EMU",
    },

    {
        TYPE_DEVMEM,
        IPU_IVAHD_CONFIG, L3_IVAHD_CONFIG,
        SZ_16M, 0, 0, "IPU_IVAHD_CONFIG",
    },

    {
        TYPE_DEVMEM,
        IPU_IVAHD_SL2, L3_IVAHD_SL2,
        SZ_16M, 0, 0, "IPU_IVAHD_SL2",
    },

    {
        TYPE_DEVMEM,
        IPU_PERIPHERAL_DMM, L3_PERIPHERAL_DMM,
        SZ_1M, 0, 0, "IPU_PERIPHERAL_DMM",
    },
};

#endif /* _RSC_TABLE_VAYU_IPU_H_ */
