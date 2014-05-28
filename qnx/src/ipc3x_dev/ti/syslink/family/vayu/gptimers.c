/*
 * Copyright (c) 2014, Texas Instruments Incorporated
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

/* Standard headers */
#include <ti/syslink/Std.h>

/* QNX specific header include */
#include <errno.h>
#include <unistd.h>
#include <stdbool.h>

#include <hw/inout.h>
#include <sys/mman.h>

#include <OsalIsr.h>
#include <ti/syslink/ProcMgr.h>
#include <ti/syslink/utils/Trace.h>
#include <MultiProc.h>
#include <_MultiProc.h>
#include <gptimers.h>


/* Start a software forced wake-up */
#define SW_WKUP                           0x2

/* Start a software forced wake-up */
#define HW_AUTO                           0x3

/* Explicitly enable a module */
#define MOD_ENABLE                        0x2

/* Disable a module */
#define MOD_DISABLE                       0xFFFFFFFC

/*
 * Empirically determined delay that is necessary between when the clock domain
 * is enabled and when the status bit is updated
 */
#define DELAY                             100

/* Module idle status mask */
#define IDLEST_MASK                       0x30000

/* Module mode mask */
#define MODULEMODE_MASK                   0x3

/* Registers for clock management of timers in IPU power domain */
#define CM_CORE_AON__IPU_SIZE             0x100
#define CM_CORE_AON__IPU_BASE             0x4A005500
#define CM_IPU_CLKSTCTRL_OFFSET           0x40
#define CM_IPU_TIMER5_CLKCTRL_OFFSET      0x58
#define CM_IPU_TIMER6_CLKCTRL_OFFSET      0x60
#define CM_IPU_TIMER7_CLKCTRL_OFFSET      0x68
#define CM_IPU_TIMER8_CLKCTRL_OFFSET      0x70

/* Timer base addresses and size */
#define GPTIMER4_BASE 0x48036000
#define GPTIMER9_BASE 0x4803E000
#define TIMER_REG_SIZE 0x1000

/*
 * Interrupts for GPTimers used as watchdog
 * They correspond to MPU_IRQ numbers + 32
 */
#define VAYU_IRQ_GPT4 (40 + 32)
#define VAYU_IRQ_GPT9 (45 + 32)

typedef enum {
    GPTIMER_4 = 0,
    GPTIMER_9,
    GPTIMER_MAX
} gpt_Nums;

/* GPTimer registers */
typedef struct gpt_Regs {
    uint32_t tidr;
    uint32_t space[3];
    uint32_t tiocp_cfg;
    uint32_t space1[3];
    uint32_t eoi;
    uint32_t irqstatus_raw;
    uint32_t irqstatus;
    uint32_t irqenable_set;
    uint32_t irqenable_clr;
    uint32_t irqwakeen;
    uint32_t tclr;
    uint32_t tcrr;
    uint32_t tldr;
    uint32_t ttgr;
    uint32_t twps;
    uint32_t tmar;
    uint32_t tcar1;
    uint32_t tsicr;
    uint32_t tcar2;
    uint32_t tpir;
    uint32_t tnir;
    uint32_t tcvr;
    uint32_t tocr;
    uint32_t towr;
} gpt_Regs;

/*
 * Defines the state object, which contains all the module
 * specific information.
 */
struct gpt_module_object {
    uintptr_t cmCoreAonBaseVa;
    /* base address to CM_CORE_AON__IPU */
    bool isSetup;
    /* Indicates whether the ipu_pm module is setup. */
    OsalIsr_Handle gpt4IsrObject;
    /* ISR handle for gpt4 WDT */
    OsalIsr_Handle gpt9IsrObject;
    /* ISR handle for gpt9 WDT */
    void *         gpt4BaseAddr;
    /* Base address of GPTimer 4 registers */
    void *         gpt9BaseAddr;
    /* Base address of GPTimer 9 registers */
    ProcMgr_Handle proc_handles[MultiProc_MAXPROCESSORS];
    /* Array of processor handles */
};

static struct gpt_module_object gptState = {
    .cmCoreAonBaseVa = NULL,
    .isSetup = false
};

/* Enable a particular timer by setting its module mode */
static int enable(uintptr_t reg)
{
    int max_tries = DELAY;

    out32(reg, MOD_ENABLE);
    do {
        if (!(in32(reg) & IDLEST_MASK)) {
            break;
        }
    } while (--max_tries);

    if (max_tries == 0) {
        return -EIO;
    }
    else {
        return EOK;
    }
}

/* Disable a particular timer by setting its module mode */
static int disable(uintptr_t reg)
{
    uint32_t value = 0;

    /*Check if Clock is Enabled*/
    value = in32(reg);
    if ((value & MODULEMODE_MASK) == MOD_ENABLE) {
        /*Disable the Timer*/
        value &= MOD_DISABLE;
        out32(reg, value);
    }

    return EOK;
}

/* Enable GP timers (only the ones in PD_IPU for now) */
static int gpt_enable()
{
    /* make sure abe clock domain is enabled as it is source for gpt5-8 */
    out32((uintptr_t)(gptState.cmCoreAonBaseVa + CM_IPU_CLKSTCTRL_OFFSET),
        SW_WKUP);

    /* Set module mode for each timer */
    if (enable(gptState.cmCoreAonBaseVa + CM_IPU_TIMER5_CLKCTRL_OFFSET)
        != EOK) {
        return -EIO;
    }

    if (enable(gptState.cmCoreAonBaseVa + CM_IPU_TIMER6_CLKCTRL_OFFSET)
        != EOK) {
        return -EIO;
    }

    if (enable(gptState.cmCoreAonBaseVa + CM_IPU_TIMER7_CLKCTRL_OFFSET)
        != EOK) {
        return -EIO;
    }

    if (enable(gptState.cmCoreAonBaseVa + CM_IPU_TIMER8_CLKCTRL_OFFSET)
        != EOK) {
        return -EIO;
    }

    return EOK;
}

/* Disable GP timers (only the ones in PD_IPU for now) */
static int gpt_disable()
{
    disable(gptState.cmCoreAonBaseVa + CM_IPU_TIMER5_CLKCTRL_OFFSET);
    disable(gptState.cmCoreAonBaseVa + CM_IPU_TIMER6_CLKCTRL_OFFSET);
    disable(gptState.cmCoreAonBaseVa + CM_IPU_TIMER7_CLKCTRL_OFFSET);
    disable(gptState.cmCoreAonBaseVa + CM_IPU_TIMER8_CLKCTRL_OFFSET);

    /* put abe clock domain back to HW_AUTO mode */
    out32((uintptr_t)(gptState.cmCoreAonBaseVa + CM_IPU_CLKSTCTRL_OFFSET),
        HW_AUTO);

    return EOK;
}

/* Setup module */
int gpt_setup()
{
    int retval = EOK;

    if (gptState.isSetup == false) {
        gptState.isSetup = true;

        gptState.cmCoreAonBaseVa = (uintptr_t)mmap_device_io(
            CM_CORE_AON__IPU_SIZE, CM_CORE_AON__IPU_BASE);
        if((uintptr_t)gptState.cmCoreAonBaseVa == MAP_DEVICE_FAILED) {
            gptState.cmCoreAonBaseVa = NULL;
            retval = -errno;
            goto exit;
        }

        gpt_enable();

        gptState.gpt4BaseAddr = (void *)mmap_device_io(TIMER_REG_SIZE,
            GPTIMER4_BASE);
        if ((uintptr_t)gptState.gpt4BaseAddr == MAP_DEVICE_FAILED) {
            retval = -ENOMEM;
            gptState.gpt4BaseAddr = NULL;
            goto exit;
        }

        gptState.gpt9BaseAddr = (void *)mmap_device_io(TIMER_REG_SIZE,
            GPTIMER9_BASE);
        if ((uintptr_t)gptState.gpt9BaseAddr == MAP_DEVICE_FAILED) {
            retval = -ENOMEM;
            gptState.gpt9BaseAddr = NULL;
            goto exit;
        }
    }

exit:
    if (retval != EOK) {
        gpt_destroy();
    }
    return retval;
}

/* Finalize module */
int gpt_destroy()
{
    if (gptState.isSetup) {
        if (gptState.gpt9BaseAddr) {
            munmap((void *)gptState.gpt9BaseAddr, TIMER_REG_SIZE);
            gptState.gpt9BaseAddr = NULL;
        }

        if (gptState.gpt4BaseAddr) {
            munmap((void *)gptState.gpt4BaseAddr, TIMER_REG_SIZE);
            gptState.gpt4BaseAddr = NULL;
        }

        gpt_disable();

        if (gptState.cmCoreAonBaseVa) {
            munmap((void *)gptState.cmCoreAonBaseVa, CM_CORE_AON__IPU_SIZE);
            gptState.cmCoreAonBaseVa = NULL;
        }

        gptState.isSetup = false;
    }
    return EOK;
}

/* Watchdog interrupt handler */
static Bool gpt_interrupt(Ptr fxnArgs)
{
    int num;
    /* Using IPU2's id since that is the offically supported IPU */
    uint16_t ipu2_id = MultiProc_getId("IPU2");

    /* Trigger recovery by setting the ProcMgr state */
    switch ((uint32_t)fxnArgs) {
        case GPTIMER_4:
            num = 4;
            ProcMgr_setState(gptState.proc_handles[ipu2_id],
                ProcMgr_State_Watchdog);
            break;
        case GPTIMER_9:
            num = 9;
            ProcMgr_setState(gptState.proc_handles[ipu2_id],
                ProcMgr_State_Watchdog);
            break;
        default:
            num = 0;
            break;
    }

    GT_1trace(curTrace, GT_4CLASS,
              "ipu_pm_gptimer_interrupt: GPTimer %d expired!", num);

    return 0;
}

/* Interrupt clear function*/
static Bool gpt_clr_interrupt(Ptr fxnArgs)
{
    uintptr_t reg;
    uint32_t num = (uint32_t)fxnArgs;
    gpt_Regs *GPTRegs = NULL;

    if (num == GPTIMER_4) {
        GPTRegs = gptState.gpt4BaseAddr;
    }
    else if (num == GPTIMER_9) {
        GPTRegs = gptState.gpt9BaseAddr;
    }
    else {
        return TRUE;
    }

    reg = in32((uintptr_t)&GPTRegs->irqstatus);
    reg |= 0x2;

    /*Clear Overflow event */
    out32((uintptr_t)&GPTRegs->irqstatus, reg);
    reg = in32((uintptr_t)&GPTRegs->irqstatus);

    /*Always return TRUE for ISR*/
    return TRUE;
}


/* Wire the Watchdog interrupts to trigger recovery */
int gpt_wdt_attach(int proc_id)
{
    int retval = EOK;
    OsalIsr_Params isrParams;

    if (proc_id > MultiProc_MAXPROCESSORS) {
        return -EINVAL;
    }

    if ((proc_id == MultiProc_getId("IPU1")) ||
        (proc_id == MultiProc_getId("IPU2"))) {
        isrParams.checkAndClearFxn = gpt_clr_interrupt;
        isrParams.fxnArgs = (Ptr)GPTIMER_9;
        isrParams.intId = VAYU_IRQ_GPT9;
        isrParams.sharedInt = FALSE;
        gptState.gpt9IsrObject =
            OsalIsr_create(&gpt_interrupt,
                           isrParams.fxnArgs, &isrParams);
        if(gptState.gpt9IsrObject != NULL) {
            if (OsalIsr_install(gptState.gpt9IsrObject) < 0) {
                retval = -ENOMEM;
            }
        }
        else {
            retval = -ENOMEM;
        }

        isrParams.checkAndClearFxn = gpt_clr_interrupt;
        isrParams.fxnArgs = (Ptr)GPTIMER_4;
        isrParams.intId = VAYU_IRQ_GPT4;
        isrParams.sharedInt = FALSE;
        gptState.gpt4IsrObject =
            OsalIsr_create(&gpt_interrupt,
                           isrParams.fxnArgs, &isrParams);
        if(gptState.gpt4IsrObject != NULL) {
            if (OsalIsr_install(gptState.gpt4IsrObject) < 0) {
                retval = -ENOMEM;
            }
        }
        else {
            retval = -ENOMEM;
        }
    }

    if ((retval >= 0) &&
        (gptState.proc_handles[MultiProc_getId("IPU2")] == NULL)) {
        /* Using IPU2's entry since it is the offically supported IPU */
        retval = ProcMgr_open(&gptState.proc_handles[MultiProc_getId("IPU2")],
            proc_id);
    }
    else {
        if ((proc_id == MultiProc_getId("IPU1")) ||
            (proc_id == MultiProc_getId("IPU2"))) {
            if (gptState.gpt9IsrObject) {
                OsalIsr_uninstall(gptState.gpt9IsrObject);
                OsalIsr_delete(&gptState.gpt9IsrObject);
                gptState.gpt9IsrObject = NULL;
            }

            if (gptState.gpt4IsrObject) {
                OsalIsr_uninstall(gptState.gpt4IsrObject);
                OsalIsr_delete(&gptState.gpt4IsrObject);
                gptState.gpt4IsrObject = NULL;
            }
        }
    }

    return retval;
}

/* Un-hook the Watchdog interrupt handler */
int gpt_wdt_detach(int proc_id)
{
    int retval = EOK;

    if (proc_id > MultiProc_MAXPROCESSORS) {
        return -EINVAL;
    }

    if ((proc_id == MultiProc_getId("IPU1")) ||
        (proc_id == MultiProc_getId("IPU2"))) {
        OsalIsr_uninstall(gptState.gpt9IsrObject);
        OsalIsr_delete(&gptState.gpt9IsrObject);
        gptState.gpt9IsrObject = NULL;

        OsalIsr_uninstall(gptState.gpt4IsrObject);
        OsalIsr_delete(&gptState.gpt4IsrObject);
        gptState.gpt4IsrObject = NULL;
    }

    /* Using IPU2's entry since it is the offically supported IPU */
    if (gptState.proc_handles[MultiProc_getId("IPU2")]) {
        ProcMgr_close(&gptState.proc_handles[MultiProc_getId("IPU2")]);
        gptState.proc_handles[MultiProc_getId("IPU2")] = NULL;
    }

    return retval;
}
