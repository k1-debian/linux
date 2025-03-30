# SPDX-License-Identifier: GPL-2.0
dtb-$(CONFIG_DT_HALLEY6_V20)		+= x1600/halley6_v20.dtb
dtb-$(CONFIG_DT_PANDA_V10)	        += x1600/panda_v10.dtb
dtb-$(CONFIG_DT_GEWU_V10)		+= m300/gewu_v10.dtb
dtb-$(CONFIG_DT_GEWU_V12)		+= m300/gewu_v12.dtb
dtb-$(CONFIG_DT_HALLEY5_V30)		+= x2000/halley5_v30.dtb
dtb-$(CONFIG_DT_ZEBRA)			+= x2000/zebra.dtb
dtb-$(CONFIG_DT_KALE_V10)		+= x2000/kale_v10.dtb
dtb-$(CONFIG_DT_CREALITY_K1)		+= x2000/creality_k1.dtb
dtb-$(CONFIG_DT_HIPPO_V12)		+= x2500/hippo_v12.dtb


dtb-$(CONFIG_DT_X2600_HALLEY7)		+= x2600/halley7.dtb
dtb-$(CONFIG_DT_X2670_HARE)		+= x2600/hare.dtb


# x2000_module_base.dts
module_driver_x2000_dst_file := $(shell echo $(CONFIG_DT_X2000_MODULE_BASE_DTS_FILE))
module_driver_x2000_dst_file := $(basename $(module_driver_x2000_dst_file)).dtb
dtb-$(CONFIG_DT_X2000_MODULE_BASE) += $(module_driver_x2000_dst_file)

# x2600_module_base.dts
module_driver_x2600_dst_file := $(shell echo $(CONFIG_DT_X2600_MODULE_BASE_DTS_FILE))
module_driver_x2600_dst_file := $(basename $(module_driver_x2600_dst_file)).dtb
dtb-$(CONFIG_DT_X2600_MODULE_BASE) += $(module_driver_x2600_dst_file)

# ad100_module_base.dts
module_driver_ad100_dst_file := $(shell echo $(CONFIG_DT_AD100_MODULE_BASE_DTS_FILE))
module_driver_ad100_dst_file := $(basename $(module_driver_ad100_dst_file)).dtb
dtb-$(CONFIG_DT_AD100_MODULE_BASE) += $(module_driver_ad100_dst_file)

obj-$(CONFIG_BUILTIN_DTB)	+= $(addsuffix .o, $(dtb-y))
