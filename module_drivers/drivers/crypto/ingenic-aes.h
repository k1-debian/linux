#ifndef __INGENIC_AES_H__
#define __INGENIC_AES_H__

#define INGENIC_AES_QUEUE_LENGTH       (50)
#define INGENIC_AES_CACHE_PAGE_SHIFT   (0)
#define DMA_MIN     4

#define AES_ASCR      (0x00)
#define AES_ASSR      (0x04)
#define AES_ASINTM    (0x08)
#define AES_ASSA      (0x0c)
#define AES_ASDA      (0x10)
#define AES_ASTC      (0x14)
#define AES_ASDI      (0x18)
#define AES_ASDO      (0x1c)
#define AES_ASKY      (0x20)
#define AES_ASIV      (0x24)

#define ASCR_PS_DEF   (3 << 28)
#define ASCR_CLR      (1 << 10)

#define ASCR_DMAS     (1 << 9)
#define ASCR_DMAE     (1 << 8)

#define ASCR_KEYL(len)   ((len) << 6)

#define ECB_ALG       (0)
#define CBC_ALG       (1)
#define ASCR_ALGS(alg) ((alg) << 5)

#define ENCRYPTO_MODE  (0)
#define DECRYPTO_MODE  (1)
#define ASCR_DECE(m)  ((m) << 4)

#define ASCR_AESS     (1 << 3)
#define ASCR_KEYS     (1 << 2)
#define ASCR_INIT_IV  (1 << 1)
#define ASCR_EN       (1 << 0)

struct ingenic_aes_reqctx {
	unsigned long mode;
};

struct ingenic_aes_ctx {
	struct ingenic_aes_dev *aes;

	int     keylen;
	unsigned long       key[AES_KEYSIZE_256 / sizeof(unsigned long)];
	unsigned long   flags;
	struct crypto_skcipher  *fallback;
};

/* struct ingenic_aes_algs_info { */
/*  struct crypto_alg   *algs_list; */
/*  unsigned int        size; */
/*  unsigned int        registered; */
/* }; */
struct ingenic_aes_pdata {
	struct skcipher_alg *algs_list;
	unsigned int        size;
	unsigned int        registered;
};

struct ingenic_aes_dev {
	struct list_head    list;
	void __iomem    *io_base;
	int             irq;
	struct clk      *clk_gate;
	struct ingenic_aes_ctx  *ctx;
	struct device       *dev;
	unsigned long       flags;
	int         err;

	spinlock_t      lock;
	struct crypto_queue queue;
	struct skcipher_request     *req;
	size_t              total;

	struct scatterlist      *in_sg;

	struct scatterlist      in_sgl;
	size_t              in_offset;
	struct scatterlist      *out_sg;
	struct scatterlist      out_sgl;
	size_t              out_offset;

	size_t          buflen;
	void            *buf_in;
	void            *buf_out;
	dma_addr_t      dma_addr_in;
	dma_addr_t      dma_addr_out;

	size_t          dma_size;
	struct ingenic_aes_pdata *pdata;
};

#define FLAGS_MODE_MASK     0x000f
#define FLAGS_DECRYPT       BIT(0)
#define FLAGS_CBC       BIT(1)

#define FLAGS_INIT      BIT(4)
#define FLAGS_FAST      BIT(5)
#define FLAGS_BUSY      BIT(6)

#define MCU_BOOT        0xb3422000

#define DMCS                    0xb3421030
#define boot_up_mcu()           *(volatile unsigned int *)(DMCS) = 0;
#define reset_mcu()           *(volatile unsigned int *)(DMCS) = 1;

#endif /*__INGENIC_AES_H__*/
