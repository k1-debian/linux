#include <linux/mtd/mtd.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include "spinor.h"
#include "ingenic_sfc_common.h"

int get_status(struct sfc_flash *flash, int command, int len)
{
	struct spinor_flashinfo *nor_info = flash->flash_info;
	struct spi_nor_info *spi_nor_info = nor_info->nor_flash_info;
	struct spi_nor_st_info *quad_get;
	struct spi_nor_st_info *busy;
	struct sfc_cdt_xfer xfer;
	static unsigned char buf[32];
	unsigned int val = 0, i = 0, ret = 0;

	busy = &spi_nor_info->busy;
	quad_get = &spi_nor_info->quad_get;

	memset(&xfer, 0, sizeof(xfer));
	memset(buf, 0, sizeof(buf));

	/* set index */
	if (command == busy->cmd) {
		xfer.cmd_index = NOR_GET_STATUS;
	} else if (command == quad_get->cmd) {
		xfer.cmd_index = NOR_GET_STATUS_1;
	} else {
		xfer.cmd_index = NOR_GET_STATUS_2;
	}

	/* set addr */
	xfer.rowaddr = 0;
	xfer.columnaddr = 0;

	/* set transfer config */
	xfer.dataen = ENABLE;
	xfer.config.datalen = len;
	xfer.config.data_dir = GLB0_TRAN_DIR_READ;
	xfer.config.ops_mode = CPU_OPS;
	xfer.config.buf = buf;

	if (sfc_sync_cdt(flash->sfc, &xfer)) {
		printk("sfc_sync error! %s %s %d\n", __FILE__, __func__, __LINE__);
		ret = -EIO;
	}

	for (i = 0; i < len; i++) {
		val |= buf[i] << (i * 8);
	}

	return val;
}

/* do nothing to set quad mode, use cmd directly */
static int set_quad_mode_cmd(struct sfc_flash *flash)
{
	return 0;
}

/* write nor flash status register QE bit to set quad mode */
static int set_quad_mode_reg(struct sfc_flash *flash)
{
	struct spinor_flashinfo *nor_info = flash->flash_info;
	struct spi_nor_info *spi_nor_info = nor_info->nor_flash_info;
	struct spi_nor_st_info *quad_set;
	struct spi_nor_st_info *quad_get;
	struct spi_nor_st_info *busy;
	unsigned int data;
	struct sfc_cdt_xfer xfer;
	int ret = 0;

	busy = &spi_nor_info->busy;
	quad_set = &spi_nor_info->quad_set;
	quad_get = &spi_nor_info->quad_get;
	data = (quad_set->val & quad_set->mask) << quad_set->bit_shift;

	memset(&xfer, 0, sizeof(xfer));

	/* set index */
	xfer.cmd_index = NOR_QUAD_SET_ENABLE;

	/* set addr */
	xfer.columnaddr = 0;
	xfer.rowaddr = 0;

	/* set transfer config */
	xfer.dataen = ENABLE;
	xfer.config.datalen = quad_set->len;
	xfer.config.data_dir = GLB0_TRAN_DIR_WRITE;
	xfer.config.ops_mode = CPU_OPS;
	xfer.config.buf = (uint8_t *)&data;

	if (sfc_sync_cdt(flash->sfc, &xfer)) {
		printk("sfc_sync error! %s %s %d\n", __FILE__, __func__, __LINE__);
		ret = -EIO;
	}

	return ret;
}

static int write_enable(struct sfc_flash *flash)
{
	struct sfc_cdt_xfer xfer;
	int32_t ret = 0;

	memset(&xfer, 0, sizeof(xfer));

	/* set index */
	xfer.cmd_index = NOR_WRITE_ENABLE;

	/* set addr */
	xfer.columnaddr = 0;

	/* set transfer config */
	xfer.dataen = DISABLE;

	if (sfc_sync_cdt(flash->sfc, &xfer)) {
		printk("sfc_sync error! %s %s %d\n", __FILE__, __func__, __LINE__);
		ret = -EIO;
	}

	return ret;
}

static int enter_4byte(struct sfc_flash *flash)
{
	struct sfc_cdt_xfer xfer;
	int32_t ret = 0;

	memset(&xfer, 0, sizeof(xfer));

	/* set index */
	xfer.cmd_index = NOR_EN_4BYTE;

	/* set addr */
	xfer.columnaddr = 0;

	/* set transfer config */
	xfer.dataen = DISABLE;

	if (sfc_sync_cdt(flash->sfc, &xfer)) {
		printk("sfc_sync error! %s %s %d\n", __FILE__, __func__, __LINE__);
		ret = -EIO;
	}

	return ret;
}

/* send 4byte command to enter 4byte mode */
static int set_4byte_mode_normal(struct sfc_flash *flash)
{
	int ret;
	ret = enter_4byte(flash);
	if (ret) {
		dev_err(flash->dev, "enter 4byte mode failed\n");
	}
	return ret;
}

/**
 * 1.send write enable command
 * 2.send 4byte command to enter 4byte mode
 **/
static int set_4byte_mode_wren(struct sfc_flash *flash)
{
	int ret;
	ret = write_enable(flash);
	if (ret) {
		dev_err(flash->dev, "enter 4byte mode failed\n");
		return ret;
	}

	ret = enter_4byte(flash);
	if (ret) {
		dev_err(flash->dev, "enter 4byte mode failed\n");
	}
	return ret;
}

static struct spi_nor_flash_ops nor_flash_ops;

static int noop(struct sfc_flash *flash)
{
	return 0;
}

int sfc_nor_get_special_ops(struct sfc_flash *flash)
{
	struct spinor_flashinfo *nor_info = flash->flash_info;
	struct spi_nor_info *spi_nor_info = nor_info->nor_flash_info;

	switch (spi_nor_info->quad_ops_mode) {
	case 0:
		nor_flash_ops.set_quad_mode = set_quad_mode_cmd;
		break;
	case 1:
		nor_flash_ops.set_quad_mode = set_quad_mode_reg;
		break;
	default:
		nor_flash_ops.set_quad_mode = noop;
		break;
	}

	switch (spi_nor_info->addr_ops_mode) {
	case 0:
		nor_flash_ops.set_4byte_mode = set_4byte_mode_normal;
		break;
	case 1:
		nor_flash_ops.set_4byte_mode = set_4byte_mode_wren;
		break;
	default:
		nor_flash_ops.set_4byte_mode = noop;
		break;
	}

	nor_info->nor_flash_ops = &nor_flash_ops;

	return 0;
}
