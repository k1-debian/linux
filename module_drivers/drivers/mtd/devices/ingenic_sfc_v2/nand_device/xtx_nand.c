#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mtd/partitions.h>
#include "../spinand.h"
#include "../ingenic_sfc_common.h"
#include "nand_common.h"

#define XTX_DEVICES_NUM         3
#define TSETUP      5
#define THOLD       5
#define TSHSL_R     20
#define TSHSL_W     20

#define TRD     240
#define TPP     1400
#define TBE     10

static struct ingenic_sfcnand_device *xtx_nand;

static struct ingenic_sfcnand_base_param xtx_param[XTX_DEVICES_NUM] = {

	[0] = {
		/*PN26G01AW*/
		.pagesize = 2 * 1024,
		.blocksize = 2 * 1024 * 64,
		.oobsize = 128,
		.flashsize = 2 * 1024 * 64 * 1024,

		.tSETUP  = TSETUP,
		.tHOLD   = THOLD,
		.tSHSL_R = TSHSL_R,
		.tSHSL_W = TSHSL_W,

		.tRD = TRD,
		.tPP = TPP,
		.tBE = TBE,

		.plane_select = 0,
		.ecc_max = 0x8,
		.need_quad = 1,
	},
	[1] = {
		/*PN26G02AW */
		.pagesize = 2 * 1024,
		.blocksize = 2 * 1024 * 64,
		.oobsize = 128,
		.flashsize = 2 * 1024 * 64 * 2048,

		.tSETUP  = TSETUP,
		.tHOLD   = THOLD,
		.tSHSL_R = TSHSL_R,
		.tSHSL_W = TSHSL_W,

		.tRD = TRD,
		.tPP = TPP,
		.tBE = TBE,

		.plane_select = 0,
		.ecc_max = 0x8,
		.need_quad = 1,
	},
	[2] = {
		/*PN26Q01AW */
		.pagesize = 2 * 1024,
		.blocksize = 2 * 1024 * 64,
		.oobsize = 128,
		.flashsize = 2 * 1024 * 64 * 1024,

		.tSETUP  = TSETUP,
		.tHOLD   = THOLD,
		.tSHSL_R = TSHSL_R,
		.tSHSL_W = TSHSL_W,

		.tRD = TRD,
		.tPP = TPP,
		.tBE = TBE,

		.plane_select = 0,
		.ecc_max = 0x8,
		.need_quad = 1,
	},

};

static struct device_id_struct device_id[XTX_DEVICES_NUM] = {
	DEVICE_ID_STRUCT(0xE1, "PN26G01AW", &xtx_param[0]),
	DEVICE_ID_STRUCT(0xE2, "PN26G02AW", &xtx_param[1]),
	DEVICE_ID_STRUCT(0xC1, "PN26Q01AW", &xtx_param[2]),
};

static cdt_params_t *xtx_get_cdt_params(struct sfc_flash *flash, uint16_t device_id)
{
	CDT_PARAMS_INIT(xtx_nand->cdt_params);

	switch (device_id) {
	case 0xE1:
	case 0xE2:
	case 0xC1:
		break;
	default:
		dev_err(flash->dev, "device_id err, please check your  device id: device_id = 0x%02x\n", device_id);
		return NULL;
	}

	return &xtx_nand->cdt_params;
}

static inline int deal_ecc_status(struct sfc_flash *flash, uint16_t device_id, uint8_t ecc_status)
{

	switch (device_id) {
	case 0xE1:
	case 0xE2:
	case 0xC1:
		switch ((ecc_status >> 4) & 0x3) {
		case 0x0:
		case 0x1:
			return 0;
		case 0x2:
			return -EBADMSG;
		case 0x3:
			return 8;
		default:
			break;
		}
		break;
	default:
		dev_err(flash->dev, "device_id err, it maybe don`t support this device, check your device id: device_id = 0x%02x\n", device_id);
		break;

	}
	return -EINVAL;
}

static int __init xtx_nand_init(void)
{

	xtx_nand = kzalloc(sizeof(*xtx_nand), GFP_KERNEL);
	if (!xtx_nand) {
		pr_err("alloc xtx_nand struct fail\n");
		return -ENOMEM;
	}

	xtx_nand->id_manufactory = 0xA1;
	xtx_nand->id_device_list = device_id;
	xtx_nand->id_device_count = XTX_DEVICES_NUM;

	xtx_nand->ops.get_cdt_params = xtx_get_cdt_params;
	xtx_nand->ops.deal_ecc_status = deal_ecc_status;

	/* use private get feature interface, please define it in this document */
	xtx_nand->ops.get_feature = NULL;

	return ingenic_sfcnand_register(xtx_nand);
}

fs_initcall(xtx_nand_init);
