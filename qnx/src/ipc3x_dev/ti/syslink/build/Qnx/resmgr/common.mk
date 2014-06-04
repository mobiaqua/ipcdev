#
#   Copyright (c) 2013-2014, Texas Instruments Incorporated
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#   *  Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#   *  Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#   *  Neither the name of Texas Instruments Incorporated nor the names of
#      its contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#   THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#   EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

INSTALLDIR=bin

define PINFO
PINFO DESCRIPTION=IPC Resource Manager
endef
NAME=ipc

CCOPTS += -DSYSLINK_BUILDOS_QNX -DSYSLINK_BUILD_DEBUG -DSYSLINK_BUILD_HLOS $(QNX_CFLAGS)

ifeq ("$(SYSLINK_DEBUG)", "1")
#enable debug build
CCOPTS += -g -O0
endif # ifeq ("$(SYSLINK_DEBUG)", "1")

ifeq ("$(SYSLINK_PLATFORM)", "omap5430")
ifeq ("$(SMP)", "1")
CCOPTS += -DSYSLINK_SYSBIOS_SMP
endif # ifeq ("$(SMP)", "1")
CCOPTS += -DSYSLINK_PLATFORM_OMAP5430 -DARM_TARGET -DC60_TARGET -DSYSLINK_USE_IPU_PM
endif # ifeq ("$(SYSLINK_PLATFORM)", "omap5430")

ifeq ("$(SYSLINK_PLATFORM)", "vayu")
ifeq ("$(SMP)", "1")
CCOPTS += -DSYSLINK_SYSBIOS_SMP
endif # ifeq ("$(SMP)", "1")
CCOPTS += -DSYSLINK_PLATFORM_VAYU -DARM_TARGET -DC60_TARGET
endif # ifeq ("$(SYSLINK_PLATFORM)", "vayu")

#SYSLINK Product root path
SYSLINK_ROOT = $(PROJECT_ROOT)/../../../../..
SYSLINK_BUILDOS = Qnx

#PUBLIC_INCVPATH = $(PROJECT_ROOT)/public

EXTRA_INCVPATH = $(SYSLINK_ROOT)	\
		  $(SYSLINK_ROOT)/ti/syslink/inc	\
		  $(SYSLINK_ROOT)/ti/syslink/inc/ti/ipc	\
		  $(SYSLINK_ROOT)/ti/syslink/inc/knl	\
		  $(SYSLINK_ROOT)/ti/syslink/inc/$(SYSLINK_BUILDOS)	\
		  $(SYSLINK_ROOT)/ti/syslink/inc/knl/$(SYSLINK_BUILDOS) 	\
		  $(SYSLINK_ROOT)/ti/syslink/ipc/hlos/knl/family/$(SYSLINK_PLATFORM) 	\
		  $(SYSLINK_PKGPATH)	\
		  $(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/rprc 	\
		  $(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)	\
		  $(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)/DLOAD/DLOAD_API	\
		  $(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)/DLOAD/DLOAD	\
		  $(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)/DLOAD/C60_DLOAD_DYN	\
		  $(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)/DLOAD/C60_DLOAD_REL	\
		  $(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)/DLOAD/TMS470_DLOAD_DYN	\
		  $(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)/DLOAD/TMS470_DLOAD_REL	\
		  $(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)/DLOAD/DLOAD_SYM	\
		  $(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)/dlw_client	\
		  $(SYSLINK_ROOT)/ti/syslink/rpmsg-resmgr/hlos/knl/$(SYSLINK_BUILDOS)/family	\
		  $(SYSLINK_ROOT)/ti/syslink/resources	\
                  $(SYSLINK_ROOT)/ti/syslink/family/common \
		  $(IPC_REPO)/qnx/include \
                  $(IPC_REPO)/hlos_common/include \
		  $(IPC_REPO)/packages \

#devctl for ipc, procmgr, utils
EXTRA_SRCVPATH+=$(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl	\
		$(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/rprc	\
		$(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)	\
		$(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)/DLOAD/DLOAD	\
		$(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)/DLOAD/TMS470_DLOAD_DYN	\
		$(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)/DLOAD/TMS470_DLOAD_REL	\
		$(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)/DLOAD/C60_DLOAD_REL	\
		$(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)/DLOAD/C60_DLOAD_DYN	\
		$(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)/DLOAD/DLOAD_SYM	\
		$(SYSLINK_ROOT)/ti/syslink/procMgr/hlos/knl/loaders/Elf/$(SYSLINK_BUILDOS)/dlw_client	\
		$(SYSLINK_ROOT)/ti/syslink/resources	\
		$(SYSLINK_ROOT)/ti/syslink/ipc/hlos/knl/transports/virtio	\
		$(SYSLINK_ROOT)/ti/syslink/ipc/hlos/knl	\
		$(SYSLINK_ROOT)/ti/syslink/ipc/hlos/knl/arch	\
		$(SYSLINK_ROOT)/ti/syslink/ipc/hlos/knl/arch/$(SYSLINK_PLATFORM)	\
		$(SYSLINK_ROOT)/ti/syslink/ipc/hlos/knl/$(SYSLINK_BUILDOS)	\
		$(SYSLINK_ROOT)/ti/syslink/utils/common	\
		$(SYSLINK_ROOT)/ti/syslink/utils/hlos	\
		$(SYSLINK_ROOT)/ti/syslink/utils/hlos/knl	\
		$(SYSLINK_ROOT)/ti/syslink/utils/hlos/knl/$(SYSLINK_BUILDOS)	\
		$(SYSLINK_ROOT)/ti/syslink/utils/hlos/knl/osal/$(SYSLINK_BUILDOS)	\
		$(SYSLINK_ROOT)/ti/syslink/rpmsg-omx/hlos/knl/$(SYSLINK_BUILDOS)	\
		$(SYSLINK_ROOT)/ti/syslink/rpmsg-dce/hlos/knl/$(SYSLINK_BUILDOS)	\
		$(SYSLINK_ROOT)/ti/syslink/ti-ipc	\
		$(SYSLINK_ROOT)/ti/syslink/rpmsg-rpc	\
		$(SYSLINK_ROOT)/ti/syslink/rpmsg-resmgr/hlos/knl/$(SYSLINK_BUILDOS)/family/$(SYSLINK_PLATFORM)	\
		$(SYSLINK_ROOT)/ti/syslink/rpmsg-resmgr/hlos/knl/$(SYSLINK_BUILDOS)

ifeq ("$(SYSLINK_PLATFORM)", "omap5430")
EXTRA_SRCVPATH+=$(SYSLINK_ROOT)/ti/syslink/family/$(SYSLINK_PLATFORM)	\
		$(SYSLINK_ROOT)/ti/syslink/family/$(SYSLINK_PLATFORM)/ipu

EXCLUDE_OBJS = GateMP_daemon.o gatemp_devctl.o
endif

ifeq ("$(SYSLINK_PLATFORM)", "vayu")
EXTRA_SRCVPATH+=$(SYSLINK_ROOT)/ti/syslink/family/common	\
		$(SYSLINK_ROOT)/ti/syslink/family/common/$(SYSLINK_PLATFORM)/$(SYSLINK_PLATFORM)dsp	\
		$(SYSLINK_ROOT)/ti/syslink/family/common/$(SYSLINK_PLATFORM)/$(SYSLINK_PLATFORM)ipu	\
		$(SYSLINK_ROOT)/ti/syslink/family/common/$(SYSLINK_PLATFORM)/$(SYSLINK_PLATFORM)ipu/$(SYSLINK_PLATFORM)core0	\
		$(SYSLINK_ROOT)/ti/syslink/family/common/$(SYSLINK_PLATFORM)/$(SYSLINK_PLATFORM)ipu/$(SYSLINK_PLATFORM)core1	\
		$(SYSLINK_ROOT)/ti/syslink/family/$(SYSLINK_PLATFORM)	\
		$(SYSLINK_ROOT)/ti/syslink/family/$(SYSLINK_PLATFORM)/$(SYSLINK_PLATFORM)dsp	\
		$(SYSLINK_ROOT)/ti/syslink/family/$(SYSLINK_PLATFORM)/$(SYSLINK_PLATFORM)ipu	\
		$(SYSLINK_ROOT)/ti/syslink/family/$(SYSLINK_PLATFORM)/$(SYSLINK_PLATFORM)ipu/$(SYSLINK_PLATFORM)core1   \
                $(IPC_REPO)/qnx/src/api/gates

EXCLUDE_OBJS = GateMP.o
endif

include $(MKFILES_ROOT)/qtargets.mk

LDFLAGS += -M

EXTRA_LIBVPATH += $(INSTALL_ROOT_nto)/usr/lib

ifeq ("$(SYSLINK_DEBUG)", "1")
EXTRA_LIBVPATH += $(IPC_REPO)/qnx/src/utils/arm/a.g.le.v7
LIBS += utils_g
else
EXTRA_LIBVPATH += $(IPC_REPO)/qnx/src/utils/arm/a.le.v7
LIBS += utils
endif # ifeq ("$(SYSLINK_DEBUG)", "1")
