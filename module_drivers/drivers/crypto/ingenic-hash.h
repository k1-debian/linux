#ifndef __INGENIC_HASH_H__
#define __INGENIC_HASH_H__

#define ENTER()
#define EXIT()

#define INGENIC_HASH_QUEUE_LENGTH       (50)
#define INGENIC_HASH_CACHE_PAGE_SHIFT   (0)
#define DMA_MIN     4
#define HASH_BUFFER_LEN     PAGE_SIZE

/* HASH flags */
#define HASH_FLAGS_BUSY         BIT(0)
#define HASH_FLAGS_FINAL            BIT(1)
#define HASH_FLAGS_DMA_ACTIVE   BIT(2)
#define HASH_FLAGS_OUTPUT_READY BIT(3)
#define HASH_FLAGS_INIT         BIT(4)
#define HASH_FLAGS_CPU          BIT(5)
#define HASH_FLAGS_DMA_READY        BIT(6)

#define HASH_FLAGS_FINUP        BIT(16)
#define HASH_FLAGS_SG       BIT(17)
#define HASH_FLAGS_MD5      BIT(18)
#define HASH_FLAGS_SHA1     BIT(19)
#define HASH_FLAGS_SHA224   BIT(20)
#define HASH_FLAGS_SHA256   BIT(21)
#define HASH_FLAGS_SHA384   BIT(22)
#define HASH_FLAGS_SHA512   BIT(23)
#define HASH_FLAGS_ERROR        BIT(24)
#define HASH_FLAGS_PAD      BIT(25)

#define HASH_OP_UPDATE  1
#define HASH_OP_FINAL   2

#define INGENIC_HASH_DMA_THRESHOLD  56

#define MD5_DIGEST_SIZE         16
#define MD5_BLOCK_SIZE          64

#define SHA1_DIGEST_SIZE        20
#define SHA1_BLOCK_SIZE         64

#define SHA224_DIGEST_SIZE  28
#define SHA224_BLOCK_SIZE   64

#define SHA256_DIGEST_SIZE      32
#define SHA256_BLOCK_SIZE       64

#define SHA384_DIGEST_SIZE      48
#define SHA384_BLOCK_SIZE       128

#define SHA512_DIGEST_SIZE      64
#define SHA512_BLOCK_SIZE       128

#define HASH_HSCR                0
#define HASH_HSSR                4
#define HASH_HSINTM              8
#define HASH_HSSA                0xc
#define HASH_HSTC                0x10
#define HASH_HSDI                0x14
#define HASH_HSDO                0x18

struct ingenic_hash_reqctx {
	struct ingenic_hash_dev *hash;
	unsigned long   flags;
	unsigned long   op;

	u8  digest[SHA512_DIGEST_SIZE] __aligned(sizeof(u32));
	u64 digcnt[2];
	size_t  bufcnt;
	size_t  buflen;
	dma_addr_t  dma_addr;

	/* walk state */
	struct scatterlist  *sg;
	unsigned int    offset; /* offset in current sg */
	unsigned int    total;  /* total request */

	size_t block_size;
	u8 *buffer;

};

struct ingenic_hash_ctx {
	struct ingenic_hash_dev *hash;
	unsigned long       flags;
	/* fallback stuff */
	struct crypto_ahash *fallback;
	u8 buf[HASH_BUFFER_LEN] __aligned(sizeof(u32));
};

struct ingenic_hash_pdata {
	struct ahash_alg    *algs_list;
	unsigned int        size;
	unsigned int        registered;
};

struct ingenic_hash_dev {
	struct list_head    list;
	void __iomem    *io_base;
	int             irq;
	struct clk      *clk_gate;
	struct ingenic_hash_ctx *ctx;
	struct device       *dev;
	unsigned long       flags;
	spinlock_t      lock;
	struct crypto_queue queue;
	struct ahash_request    *req;
	struct ingenic_hash_pdata *pdata;
};

#define MCU_BOOT        0xb3422000

#define DMCS                    0xb3421030
#define boot_up_mcu()           *(volatile unsigned int *)(DMCS) = 0;

#endif  /*__INGENIC_HASH_H__*/
