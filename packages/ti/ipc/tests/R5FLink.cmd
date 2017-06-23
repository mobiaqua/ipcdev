/*----------------------------------------------------------------------------*/
/* R5FLink.cmd                                                             */
/*                                                                            */
/* (c) Texas Instruments 2017-2018, All rights reserved.                           */
/*                                                                            */

/* USER CODE BEGIN (0) */
/* USER CODE END */

/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
--retain="*(.intvecs)"
--retain="*(.intvecsNew)"

/*----------------------------------------------------------------------------*/
/* Memory Map                                                                 */

/*  Memory Map for AM65X platform
 *
 *  --- External Memory ---
 *  Virtual     Physical        Size            Comment
 *  ------------------------------------------------------------------------
 *  9C20_0000   ???0_0000    10_0000  (   1 MB) EXT_CODE
 *  9C30_0000   ???0_0000    10_0000  (   2 MB) EXT_DATA
 *  9C40_0000   ???0_0000    30_0000  (   3 MB) EXT_HEAP
 *  9C10_0000   ???0_0000     6_0000  ( 384 kB) TRACE_BUF
 *  9C16_0000   ???6_0000     1_0000  (  64 kB) EXC_DATA
 *  9C17_0000   ???7_0000     2_0000  ( 128 kB) PM_DATA (Power mgmt)
 */

#define EXT_CODE_BASE 0x9C200000
#define EXT_DATA_BASE 0x9C300000
#define RAM0_ADDR 0x41C00000
#define ATCM_START 0x00000000

-e __VECS_ENTRY_POINT

MEMORY{
    ATCM       (RWX) : origin=ATCM_START, length=0x00008000
    BTCM       (RWX) : origin=0x41010000, length=0x00008000
    RAM0       (RW)  : origin=RAM0_ADDR  length=0x00080000
    EXT_CODE   (RWX) : origin=EXT_CODE_BASE length=0x00100000
    EXT_DATA   (RW)  : origin=EXT_DATA_BASE length=0x00100000
    EXT_HEAP   (RW)  : origin=0x9C400000 length=0x00300000
    TRACE_BUF  (RW)  : origin=0x9C100000 length=0x00060000
    EXC_DATA   (RW)  : origin=0x9C160000 length=0x00010000
    PM_DATA    (RW)  : origin=0x9C170000 length=0x00020000
}

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */
SECTIONS{
    .vecs             : {
        __VECS_ENTRY_POINT = .;
    } > ATCM_START

    .init_text        : {
                          boot.*(.text)
                          *(.text:ti_sysbios_family_arm_MPU_*)
                          *(.text:ti_sysbios_family_arm_v7r_Cache_*)
                          *(.text:xdc_runtime_Startup_reset*)
                        }  > ATCM
    .resource_table   : {
        __RESOURCE_TABLE = .;
    } > EXT_DATA_BASE

    .tracebuf  : {} > TRACE_BUF
}
/*----------------------------------------------------------------------------*/
