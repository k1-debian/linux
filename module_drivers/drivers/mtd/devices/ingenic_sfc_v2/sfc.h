#ifndef __SFC_H
#define __SFC_H

#include <linux/list.h>
#include <linux/mtd/mtd.h>
#include <linux/irqreturn.h>
#include <soc/sfc.h>

struct sfc {
	struct device *dev;
	void __iomem    *iomem;
	struct resource *ioarea;
	int         irq;
	struct clk      *clk;
	struct clk      *clk_gate;
	unsigned long src_clk;
	struct completion   done;
	spinlock_t      spin_lock;
	uint32_t        threshold;
	irqreturn_t (*irq_callback)(struct sfc *);
	unsigned long       phys;

	struct sfc_cdt_xfer *xfer;

	struct sfc_desc *desc;
	dma_addr_t desc_pyaddr;
	uint32_t desc_max_num;

	uint8_t *tmp_buffer;
	dma_addr_t tbuff_pyaddr;

	uint32_t retry_count;
};

struct data_config {

	uint32_t datalen;
	uint32_t cur_len;
	uint8_t data_dir;
	uint8_t ops_mode;
	uint8_t *buf;
};

struct sfc_cdt_xfer {

	unsigned short cmd_index;
	uint8_t dataen;
	struct data_config config;

	struct {
		uint32_t columnaddr;
		uint32_t rowaddr;
		uint32_t staaddr0;
		uint32_t staaddr1;
	};

};

struct sfc_desc {
	unsigned int next_des_addr;
	unsigned int mem_addr;
	unsigned int tran_len;
	unsigned int link;
};

enum {
	COL_ADDR,
	ROW_ADDR,
	STA_ADDR0,
	STA_ADDR1,
};

/*
 * create cdt table
 */
struct sfc_cdt {
	uint32_t link;
	uint32_t xfer;
	uint32_t staExp;
	uint32_t staMsk;
};

#define CMD_XFER(ADDR_WIDTH, POLL_EN, DMY_BITS, DATA_EN, CMD) (         \
        (ADDR_WIDTH << TRAN_CONF0_ADDR_WIDTH_OFFSET)                \
        | (POLL_EN << TRAN_CONF0_POLL_OFFSET)                   \
        | (TRAN_CONF0_CMDEN)                            \
        | (0 << TRAN_CONF0_FMAT_OFFSET)                     \
        | (DMY_BITS << TRAN_CONF0_DMYBITS_OFFSET)               \
        | (DATA_EN << TRAN_CONF0_DATEEN_OFFSET)                 \
        | CMD                                   \
                                                              )

#define CMD_LINK(LINK, ADDRMODE, TRAN_MODE) (                   \
        (LINK << 31) | (TRAN_MODE << TRAN_CONF1_TRAN_MODE_OFFSET) | (ADDRMODE)  \
                                            )

#define MK_CMD(cdt, cmd_info, LINK, ADDRMODE, DATA_EN)  {                       \
		cdt.link = CMD_LINK(LINK, ADDRMODE, cmd_info.transfer_mode);                    \
		cdt.xfer = CMD_XFER(cmd_info.addr_nbyte, DISABLE, cmd_info.dummy_byte, DATA_EN, cmd_info.cmd);  \
		cdt.staExp = 0;                                         \
		cdt.staMsk = 0;                                         \
	}

#define MK_ST(cdt, st_info, LINK, ADDRMODE, ADDR_WIDTH, POLL_EN, DATA_EN, TRAN_MODE)  {         \
		cdt.link = CMD_LINK(LINK, ADDRMODE, TRAN_MODE);                         \
		cdt.xfer = CMD_XFER(ADDR_WIDTH, POLL_EN, st_info.dummy, DATA_EN, st_info.cmd);          \
		cdt.staExp = (st_info.val << st_info.bit_shift);                        \
		cdt.staMsk = (st_info.mask << st_info.bit_shift);                       \
	}

#endif
