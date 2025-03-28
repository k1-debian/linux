#
# linux/drivers/mtd/devices/ingenic_sfc_v2/Makefile
#

obj-$(CONFIG_INGENIC_BUILTIN_PARAMS) += nor_device/

ccflags-y += -I$(srctree)/drivers/mtd/nand/raw
obj-$(CONFIG_MTD_INGENIC_SFC_NANDFLASH) += ingenic_sfc_common.o ingenic_sfc_nand.o nand_device/ ingenic_sfc_nor.o ingenic_sfc_ops.o ingenic_sfc_drv.o
obj-$(CONFIG_MTD_INGENIC_SFC_NORFLASH) += ingenic_sfc_common.o ingenic_sfc_nor.o ingenic_sfc_ops.o ingenic_sfc_drv.o
obj-$(CONFIG_MTD_INGENIC_SFC_BOTH_NAND_NOR) += ingenic_sfc_common.o ingenic_sfc_nand.o ingenic_sfc_nor.o ingenic_sfc_ops.o ingenic_sfc_drv.o nand_device/
