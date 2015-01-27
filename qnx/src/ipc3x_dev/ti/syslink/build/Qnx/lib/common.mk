ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

INSTALLDIR=/usr/lib

define PINFO
PINFO DESCRIPTION=IPC QNX Usr Libs
endef
NAME=ipc_client

# ---------------------------------------------------------------------------- #
# Defines                                                                      #
# ---------------------------------------------------------------------------- #
# Override definitions in base Makefile if required

#To Over ride the usage of ipc memory manager
IPC_USE_SYSMGR := 0
#To Over ride the build optimization flag
IPC_BUILD_OPTIMIZE := 0
#To override the debug build flag
IPC_BUILD_DEBUG := 0
#To override the TRACE flag
IPC_TRACE_ENABLE := 0

ifeq ("$(IPC_PLATFORM)", "omap5430")
CCOPTS += -DIPC_PLATFORM_OMAP5430
endif # ifeq ("$(IPC_PLATFORM)", "omap5430")

#default IPC Product root path and can be overridden from commandline
IPC_ROOT = $(PROJECT_ROOT)/../../../../..
IPC_BUILDOS = Qnx

#For SOURCE and include paths
#-include $(IPC_ROOT)/ti/syslink/buildutils/hlos/usr/Makefile.inc

#Add Resource Manager include path
#EXTRA_INCVPATH+=$(IPC_ROOT)/ti/syslink/utils/hlos/knl/Qnx/resMgr

#Add extra include path
EXTRA_INCVPATH+=$(IPC_ROOT)	\
				$(IPC_ROOT)/ti/syslink/inc	\
				$(IPC_ROOT)/ti/syslink/inc/usr/$(IPC_BUILDOS)	\
				$(IPC_ROOT)/ti/syslink/inc/usr	\
				$(IPC_ROOT)/ti/syslink/inc/$(IPC_BUILDOS) \
				$(IPC_REPO)/packages \
				$(IPC_REPO)/qnx/include \
                                $(IPC_REPO)/hlos_common/include
#SRCS:=$(CSRCS)

#SRCDIRS=$(sort $(foreach i,$(CSRCS),$(shell dirname $i)))
#EXTRA_SRCVPATH+=$(SRCDIRS)
EXTRA_SRCVPATH+=$(IPC_ROOT)/ti/syslink/ipc/hlos/usr \
				$(IPC_ROOT)/ti/syslink/ipc/hlos/usr/$(IPC_BUILDOS)	\
				$(IPC_ROOT)/ti/syslink/utils/hlos	\
				$(IPC_ROOT)/ti/syslink/utils/hlos/usr	\
				$(IPC_ROOT)/ti/syslink/utils/hlos/usr/$(IPC_BUILDOS)	\
				$(IPC_ROOT)/ti/syslink/utils/hlos/usr/osal/$(IPC_BUILDOS)

#Using the default build rules TODO: to selective pick and choose the compiler/linker/archiver & build rules
include $(MKFILES_ROOT)/qtargets.mk
CCOPTS += -DIPC_BUILDOS_QNX -DIPC_BUILD_DEBUG $(QNX_CFLAGS)
ifeq ("$(IPC_DEBUG)", "1")
#enable debug build
CCOPTS += -g -O0
endif # ifeq ("$(IPC_DEBUG)", "")
CCFLAGS += $(COMPILE_FLAGS)
CCFLAGS += -fPIC


