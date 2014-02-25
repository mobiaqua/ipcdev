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

/*QNX specific header include */
#include <errno.h>
#include <unistd.h>
#include <stdbool.h>

#include <hw/inout.h>
#include <sys/mman.h>

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

/*
 * Defines the state object, which contains all the module
 * specific information.
 */
struct gpt_module_object {
    uintptr_t cmCoreAonBaseVa;
    /* base address to CM_CORE_AON__IPU */
    bool isSetup;
    /* Indicates whether the ipu_pm module is setup. */
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
        gptState.cmCoreAonBaseVa = (uintptr_t)mmap_device_io(
            CM_CORE_AON__IPU_SIZE, CM_CORE_AON__IPU_BASE);
        if((uintptr_t)gptState.cmCoreAonBaseVa == MAP_DEVICE_FAILED) {
            gptState.cmCoreAonBaseVa = NULL;
            retval = -errno;
            goto exit;
        }

        gpt_enable();

        gptState.isSetup = true;
    }

exit:
    if (retval != EOK) {
        gpt_disable();

        if (gptState.cmCoreAonBaseVa) {
            munmap((void *)gptState.cmCoreAonBaseVa, CM_CORE_AON__IPU_SIZE);
            gptState.cmCoreAonBaseVa = NULL;
        }
    }
    return retval;
}

/* Finalize module */
int gpt_destroy()
{
    if (gptState.isSetup) {
        gpt_disable();

        if (gptState.cmCoreAonBaseVa) {
            munmap((void *)gptState.cmCoreAonBaseVa, CM_CORE_AON__IPU_SIZE);
            gptState.cmCoreAonBaseVa = NULL;
        }

        gptState.isSetup = false;
    }
    return EOK;
}
