#ifndef __SPINAND_H
#define __SPINAND_H
#include <linux/types.h>
#include "sfc_flash.h"
#include "spinand_cmd.h"

#define SPIFLASH_PARAMER_OFFSET 0x5800
#define SPINAND_MAGIC_NUM   0x646e616e

/* the max number of DMA Descriptor */
#define DESC_MAX_NUM        2

struct ingenic_sfcnand_base_param {
	uint32_t pagesize;
	uint32_t blocksize;
	uint32_t oobsize;
	uint32_t flashsize;

	uint16_t tHOLD;
	uint16_t tSETUP;
	uint16_t tSHSL_R;
	uint16_t tSHSL_W;

	uint16_t tRD;
	uint16_t tPP;
	uint16_t tBE;

	/*
	 * Indicates that NAND flash has a serial plane structure,
	 * needs to convert the plane by changing the column address
	 */
	uint8_t plane_select;

	uint8_t ecc_max;
	uint8_t need_quad;
};

struct flash_address {
	uint32_t pageaddr;
	uint32_t columnaddr;
	uint8_t ops_mode;
};

struct ingenic_sfcnand_partition_param {
	struct mtd_partition *partition;
	uint8_t num_partition;
};

struct device_id_struct {
	uint16_t id_device;
	char *name;
	struct ingenic_sfcnand_base_param *param;
};

struct spi_nand_cmd_info {
	unsigned short cmd;
	unsigned char dummy_byte;
	unsigned char addr_nbyte;
	unsigned char transfer_mode;

};

struct spi_nand_st_info {
	unsigned short cmd;
	unsigned char bit_shift;
	unsigned char mask;
	unsigned char val;
	unsigned char len; //length of byte to operate from register
	unsigned char dummy;
};

struct ingenic_sfcnand_cdt_params {
	/* general cmd info */
	struct spi_nand_cmd_info r_to_cache;
	struct spi_nand_cmd_info standard_r;
	struct spi_nand_cmd_info quad_r;
	struct spi_nand_cmd_info standard_w_cache;
	struct spi_nand_cmd_info quad_w_cache;
	struct spi_nand_cmd_info w_exec;
	struct spi_nand_cmd_info b_erase;
	struct spi_nand_cmd_info w_en;
	struct spi_nand_cmd_info ecc_r;

	/* status polling cmd info */
	struct spi_nand_st_info oip;
};
typedef struct ingenic_sfcnand_cdt_params cdt_params_t;

struct ingenic_sfcnand_ops {
	cdt_params_t *(*get_cdt_params)(struct sfc_flash *, uint16_t);
	int (*deal_ecc_status)(struct sfc_flash *, uint16_t, uint8_t);
	int32_t (*get_feature)(struct sfc_flash *, uint8_t);
};

struct ingenic_sfcnand_device {
	uint8_t id_manufactory;
	struct device_id_struct *id_device_list;
	uint8_t id_device_count;

	struct ingenic_sfcnand_ops ops;
	cdt_params_t cdt_params;

	struct list_head list;
};

struct ingenic_sfcnand_flashinfo {
	uint8_t id_manufactory;
	uint16_t id_device;

	struct ingenic_sfcnand_base_param param;
	struct ingenic_sfcnand_partition_param partition;
	struct ingenic_sfcnand_ops *ops;
	cdt_params_t *cdt_params;
};

struct ingenic_sfcnand_partition {
	char name[32];         /* identifier string */
	uint32_t size;          /* partition size */
	uint32_t offset;        /* offset within the master MTD space */
	uint32_t mask_flags;       /* master MTD flags to mask out for this partition */
	uint32_t manager_mode;     /* manager_mode mtd or ubi */
};

struct ingenic_sfcnand_burner_param {
	uint32_t magic_num;
	int32_t partition_num;
	struct ingenic_sfcnand_partition *partition;
};

int32_t ingenic_sfcnand_register(struct ingenic_sfcnand_device *flash);

/* SFC CDT Maximum INDEX number */
#define INDEX_MAX_NUM 32

/* SFC CDT INDEX */
enum {
	/* 1. reset */
	NAND_RESET,

	/* 2. try id */
	NAND_TRY_ID,

	/* 3. try id with dummy */
	NAND_TRY_ID_DMY,

	/* 4. set feature */
	NAND_SET_FEATURE,

	/* 5. get feature */
	NAND_GET_FEATURE,

	/* 6. nand standard read */
	NAND_STANDARD_READ_TO_CACHE,
	NAND_STANDARD_READ_GET_FEATURE,
	NAND_STANDARD_READ_FROM_CACHE,

	/* 7. nand quad read */
	NAND_QUAD_READ_TO_CACHE,
	NAND_QUAD_READ_GET_FEATURE,
	NAND_QUAD_READ_FROM_CACHE,

	/* 8. nand standard write */
	NAND_STANDARD_WRITE_ENABLE,
	NAND_STANDARD_WRITE_TO_CACHE,
	NAND_STANDARD_WRITE_EXEC,
	NAND_STANDARD_WRITE_GET_FEATURE,

	/* 9. nand quad write */
	NAND_QUAD_WRITE_ENABLE,
	NAND_QUAD_WRITE_TO_CACHE,
	NAND_QUAD_WRITE_EXEC,
	NAND_QUAD_WRITE_GET_FEATURE,

	/* 10. block erase */
	NAND_ERASE_WRITE_ENABLE,
	NAND_BLOCK_ERASE,
	NAND_ERASE_GET_FEATURE,

	/* 11. ecc status read */
	NAND_ECC_STATUS_READ,

	/* index count */
	NAND_MAX_INDEX,
};

#endif
