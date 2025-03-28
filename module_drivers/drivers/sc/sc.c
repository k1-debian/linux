#include <linux/module.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <asm/io.h>

#include "secall.h"
#include "pdma.h"
#include "otp.h"
#include "sc.h"

/*
 * security decrypted.
 * 1. prepare a bin encrypted.
 *      |---------------|-------------------|                   |
 *      | SC KEY(2048)  |   CODE encrypted  |
 *      |---------------|-------------------|                   |
 *
 * */

#undef TCSM_CODE_ADDR
#undef TCSM_SC_KEY_ADDR
#undef MCU_TCSM_RETVAL
#undef MCU_TCSM_SECALL_MSG

#define TCSM_CODE_ADDR      (TCSM_BANK(7) + 0)
#define TCSM_SC_KEY_ADDR    (TCSM_BANK(7) + 2048)
#define MCU_TCSM_RETVAL     (TCSM_BANK(6) + 1108) /* cal from sc_interface. */
#define MCU_TCSM_SECALL_MSG (TCSM_BANK(6) + 128)  /* MCU_TCSM_SECALL_MSG */

/* size per round must be align with 64,
 * it doesn't matter when more than 2048 or less */
#define SC_MAX_SIZE_PERROUND    (2048)
/* should not change, depends on the signature tool */
#define SC_SIGNATURE_SIZE       (2048)

#define SC_MAGIC_SIZE       (512)
#define SC_KEY_SIZE         (1536)

#define SC_MAGIC_TAG        0x54424353

struct sc_info {
	unsigned char *src;
	unsigned char *dst;
	int binlen;
	int newround;
	int endround;
	int is_open;
	int is_setup;
};

static struct sc_info info;

static void secure_check(void *addr, int *issig)
{
	int *ddrptr = (int *)(addr);
	int p = (unsigned long)SC_MAGIC_TAG;

	if (*ddrptr++ == p) {
		*issig = 1;
	}
}

static int setup_sckeys(void *addr, unsigned int *len)
{
	volatile unsigned int *tcsmptr = (volatile unsigned int *)(TCSM_SC_KEY_ADDR);
	int iLoop = 0;
	int *ddrptr = (int *)(addr + SC_MAGIC_SIZE);

	/* 384 * 4 = 1536, sc_key */
	for (iLoop = 0; iLoop < SC_KEY_SIZE / 4; iLoop++) {
		tcsmptr[iLoop] = ddrptr[iLoop];
	}

	*len = tcsmptr[0]; /* image length */

	/* len must 4 wrod align */
	if ((*len) == 0 || (*len) % 64) {
		return -1;
	}

	return 0;
}

static int sc_decrypt(struct sc_info *p)
{
	struct sc_args *args = (struct sc_args *)(MCU_TCSM_SECALL_MSG);
	unsigned int ret;
	int *srcptr = (int *)(p->src);
	int *dstptr = (int *)(p->dst);
	int binlen = p->binlen;

	unsigned int *tcsmptr = (unsigned int *)(TCSM_CODE_ADDR);
	int iLoop = 0;
	int pos = 0;

	args->arg[0] = p->endround << 1 | p->newround;
	args->arg[1] = binlen;

	for (iLoop = 0; iLoop < binlen / 4; iLoop++) {
		tcsmptr[iLoop] = srcptr[pos++];
	}

	ret = secall(args, SC_FUNC_SCBOOT, 0, 1);

	for (iLoop = 0; iLoop < binlen / 4; iLoop++) {
		dstptr[pos - binlen / 4 + iLoop] = tcsmptr[iLoop];
	}

	ret = *(volatile unsigned int *)MCU_TCSM_RETVAL;
	ret &= 0xFFFF;

	return ret;
}

#define CMD_sc_setup    _IOWR('s', 0, void *)
#define CMD_sc_decrypt  _IOWR('s', 1, void *)

static void sc_clean(void)
{
	struct sc_args *args = (struct sc_args *)(MCU_TCSM_SECALL_MSG);

	if (info.is_setup) {
		info.is_setup = 0;
		/* delete previous round */
		args->arg[0] = 1 << 1 | 0;
		args->arg[1] = SC_MAX_SIZE_PERROUND;
		secall(args, SC_FUNC_SCBOOT, 0, 1);
	}

	info.newround = 0;
	info.endround = 0;
}

static void pdma_wait(void)
{
	__asm__ volatile(
	    "	.set	push		\n\t"
	    "	.set	noreorder	\n\t"
	    "	.set	mips32		\n\t"
	    "	li	$26, 0		\n\t"
	    "	mtc0	$26, $12	\n\t"
	    "	nop			\n\t"
	    "1:				\n\t"
	    "	wait			\n\t"
	    "	b	1b		\n\t"
	    "	nop			\n\t"
	    "	.set	reorder		\n\t"
	    "	.set	pop		\n\t"
	);
}

static int sc_open(struct inode *inode, struct file *filp)
{
	int i;
	struct clk *ddr_clk, *rsa_clk, *aes_clk, *pdma_clk, *efuse_clk, *dtrng_clk, *hash_clk;

	if (info.is_open) {
		printk(KERN_ERR "SC: it already be opened, please wait for closed\n");
		return -1;
	}

	ddr_clk = clk_get(NULL, "gate_ddr");
	clk_prepare_enable(ddr_clk);
	clk_put(ddr_clk);
	rsa_clk = clk_get(NULL, "gate_rsa");
	clk_prepare_enable(rsa_clk);
	clk_put(rsa_clk);
	aes_clk = clk_get(NULL, "gate_aes");
	clk_prepare_enable(aes_clk);
	clk_put(aes_clk);
	pdma_clk = clk_get(NULL, "gate_pdma");
	clk_prepare_enable(pdma_clk);
	clk_put(pdma_clk);
	efuse_clk = clk_get(NULL, "gate_efuse");
	clk_prepare_enable(efuse_clk);
	clk_put(efuse_clk);
	dtrng_clk = clk_get(NULL, "gate_dtrng");
	clk_prepare_enable(dtrng_clk);
	clk_put(dtrng_clk);
	hash_clk = clk_get(NULL, "gate_hash");
	clk_prepare_enable(hash_clk);
	clk_put(hash_clk);

	volatile unsigned int *pdma_bank0_off = (unsigned int *)TCSM_BANK0;
	unsigned int *pdma_ins = (unsigned int *)pdma_wait;

	for (i = 0; i < 6; i++) {
		pdma_bank0_off[i] = pdma_ins[i];
	}

	boot_up_mcu();
	info.is_open = 1;

	return 0;
}

static int sc_release(struct inode *inode, struct file *filp)
{
	sc_clean();
	info.is_open = 0;

	return 0;
}

static long sc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret;
	static int total_size = 0;

	switch (cmd) {
	case CMD_sc_setup: {
		unsigned long *array = (unsigned long *)arg;
		unsigned int src = array[0];
		unsigned int *size = array[1];
		int issig = 0;
		unsigned char keybuf[SC_SIGNATURE_SIZE];

		sc_clean();

		if (copy_from_user(keybuf, src, SC_SIGNATURE_SIZE)) {
			printk(KERN_ERR "SC: copy_from_user err!!\n");
			return -1;
		}

		secure_check(keybuf, &issig);
		if (EFUSTATE_SECBOOT_EN == 0 || issig == 0) {
			printk(KERN_ERR "SC: error! please sign your file !!\n");
			return -1;
		}

		ret = setup_sckeys(keybuf, &total_size);
		if (ret) {
			printk(KERN_ERR "SC: error! please check image size, ret = 0x%x !!\n", ret);
			return -1;
		}

		if (copy_to_user(size, &total_size, sizeof(int))) {
			printk(KERN_ERR "SC: copy_to_user err!!\n");
			return -1;
		}

		info.newround = 1;
		info.is_setup = 1;
		break;
	}

	case CMD_sc_decrypt: {
		unsigned long *array = (unsigned long *)arg;
		unsigned char *src = array[0];
		unsigned char *dst = array[1];
		unsigned int size = array[2];

		while (size > 0) {
			int len = size > SC_MAX_SIZE_PERROUND ? SC_MAX_SIZE_PERROUND : size;
			if (copy_from_user(info.src, src, len)) {
				printk(KERN_ERR "SC: copy_from_user err!\n");
				return -1;
			}

			if (total_size <= SC_MAX_SIZE_PERROUND && size == total_size) {
				info.endround = 1;
			}

			info.binlen = len;
			total_size -= info.binlen;

			ret = sc_decrypt(&info);
			if (ret) {
				printk(KERN_ERR "SC: error! please check your image, ret = 0x%x!!\n", ret);
				return -1;
			}

			if (copy_to_user(dst, info.dst, len)) {
				printk(KERN_ERR "SC: copy_to_user err!!\n");
				return -1;
			}

			src += len;
			dst += len;
			size -= len;
			if (info.is_open && info.is_setup && info.endround) {
				info.is_setup = 0;
			}
			info.newround = 0;
			info.endround = 0;
		}
		break;
	}

	default:
		printk(KERN_ERR "SC: do not support this cmd: %x\n", cmd);
		return -EINVAL;
	}

	return ret;
}

static struct file_operations sc_fops = {
	.owner          = THIS_MODULE,
	.open           = sc_open,
	.release        = sc_release,
	.unlocked_ioctl = sc_ioctl,
};

static struct miscdevice mdev = {
	.minor  = MISC_DYNAMIC_MINOR,
	.name   = "sc",
	.fops   = &sc_fops,
};

static int __init sc_init(void)
{
	int ret;

	ret = misc_register(&mdev);
	if (ret < 0) {
		printk(KERN_ERR "SC: %s register err!\n", mdev.name);
		return ret;
	}

	info.src = kmalloc(SC_MAX_SIZE_PERROUND, GFP_KERNEL);
	if (info.src == NULL) {
		printk(KERN_ERR "SC: kmalloc src failed, size: %d!!\n", SC_MAX_SIZE_PERROUND);
		return -1;
	}

	info.dst = kmalloc(SC_MAX_SIZE_PERROUND, GFP_KERNEL);
	if (info.dst == NULL) {
		printk(KERN_ERR "SC: kmalloc dst failed, size: %d!!\n", SC_MAX_SIZE_PERROUND);
		kfree(info.src);
		return -1;
	}

	return 0;
}

static void __exit sc_exit(void)
{
	kfree(info.src);
	kfree(info.dst);
	misc_deregister(&mdev);
}

module_init(sc_init);
module_exit(sc_exit);

MODULE_DESCRIPTION("INGENIC x2000 SC support");
MODULE_LICENSE("GPL");