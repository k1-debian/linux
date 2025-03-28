BL_VERS_NUM=5.5.0.0

CONFIG_BL_FULLMAC ?= $(CONFIG_WIRELESS_BL602)

KERNELVERSION := 	$(shell uname -r)

# Support of P2P DebugFS for enabling/disabling NoA and OppPS
CONFIG_RWNX_P2P_DEBUGFS ?= y

subdir-ccflags-$(CONFIG_RWNX_P2P_DEBUGFS) += -DCONFIG_RWNX_P2P_DEBUGFS
subdir-ccflags-$(CONFIG_DEBUG_FS) += -DCONFIG_BL_DEBUGFS
subdir-ccflags-y += -I$(src)/fullmac

# FW VARS
subdir-ccflags-y += -DNX_VIRT_DEV_MAX=4
subdir-ccflags-y += -DNX_REMOTE_STA_MAX=10
subdir-ccflags-y += -DNX_CHAN_CTXT_CNT=3

#ARCH := arm
#CROSS_COMPILE = arm-gcc6.3-linux-uclibceabi-

obj-$(CONFIG_BL_FULLMAC) += fullmac/


