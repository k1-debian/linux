#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/clk.h>
#include <linux/pwm.h>
#include <linux/file.h>
#include <linux/gpio.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kthread.h>
#include <linux/mfd/core.h>
#include <linux/mempolicy.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/vmalloc.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/resource.h>
#include <linux/dma-map-ops.h>

/* #include <gpio.h> */
#include <linux/gpio.h>

#include <soc/gpio.h>
#include <soc/base.h>
/* #include <soc/irq.h> */
#include <dt-bindings/interrupt-controller/x2500-irq.h>

/* #include <mach/platform.h> */
/* #include <mach/jzdma.h> */
/* #include <mach/jzsnd.h> */
#include <net/netlink.h>
#include <linux/spi/spi.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>
#include <ingenic_proc.h>
/* #include <linux/mfd/jz_tcu.h> */

#include "inc/txx-funcs.h"

/* -------------------debugfs interface------------------- */
static int print_level = ISP_WARNING_LEVEL;
module_param(print_level, int, S_IRUGO);
MODULE_PARM_DESC(print_level, "isp print level");

static int isp_clk = 200000000;
module_param(isp_clk, int, S_IRUGO);
MODULE_PARM_DESC(isp_clk, "isp clock freq");

int isp_printf(unsigned int level, unsigned char *fmt, ...)
{
	struct va_format vaf;
	va_list args;
	int r = 0;

	if (level >= print_level) {
		va_start(args, fmt);

		vaf.fmt = fmt;
		vaf.va = &args;

		r = printk("%pV", &vaf);
		va_end(args);
		//      if(level >= ISP_ERROR_LEVEL)
		//          dump_stack();
	}
	return r;
}
EXPORT_SYMBOL(isp_printf);

int get_isp_clk(void)
{
	return isp_clk;
}

static const unsigned int __pow2_lut[33] = {
	1073741824, 1097253708, 1121280436, 1145833280, 1170923762, 1196563654, 1222764986, 1249540052,
	1276901417, 1304861917, 1333434672, 1362633090, 1392470869, 1422962010, 1454120821, 1485961921,
	1518500250, 1551751076, 1585730000, 1620452965, 1655936265, 1692196547, 1729250827, 1767116489,
	1805811301, 1845353420, 1885761398, 1927054196, 1969251188, 2012372174, 2056437387, 2101467502,
	2147483648U
};

uint32_t private_math_exp2(uint32_t val, const unsigned char shift_in, const unsigned char shift_out)
{
	unsigned int fract_part = (val & ((1 << shift_in) - 1));
	unsigned int int_part = val >> shift_in;

	if (shift_in <= 5) {
		unsigned int lut_index = fract_part << (5 - shift_in);
		return __pow2_lut[lut_index] >> (30 - shift_out - int_part);
	} else {
		unsigned int lut_index = fract_part >> (shift_in - 5);
		unsigned int lut_fract = fract_part & ((1 << (shift_in - 5)) - 1);
		unsigned int a = __pow2_lut[lut_index];
		unsigned int b =  __pow2_lut[lut_index + 1];
		unsigned int res = ((unsigned long long)(b - a) * lut_fract) >> (shift_in - 5);
		res = (res + a) >> (30 - shift_out - int_part);

		return res;
	}
}
EXPORT_SYMBOL(private_math_exp2);

uint8_t private_leading_one_position(const uint32_t in)
{
	uint8_t pos = 0;
	uint32_t val = in;

	if (val >= 1 << 16) {
		val >>= 16;
		pos += 16;
	}
	if (val >= 1 << 8) {
		val >>=  8;
		pos +=  8;
	}
	if (val >= 1 << 4) {
		val >>=  4;
		pos +=  4;
	}
	if (val >= 1 << 2) {
		val >>=  2;
		pos +=  2;
	}
	if (val >= 1 << 1) {
		pos +=  1;
	}

	return pos;
}

int private_leading_one_position_64(uint64_t val)
{
	int pos = 0;

	if (val >= (uint64_t)1 << 32) {
		val >>= 32;
		pos += 32;
	}
	if (val >= 1 << 16) {
		val >>= 16;
		pos += 16;
	}
	if (val >= 1 << 8) {
		val >>=  8;
		pos +=  8;
	}
	if (val >= 1 << 4) {
		val >>=  4;
		pos +=  4;
	}
	if (val >= 1 << 2) {
		val >>=  2;
		pos +=  2;
	}
	if (val >= 1 << 1) {
		pos +=  1;
	}

	return pos;
}
//  y = log2(x)
//
//  input:  Integer: val
//  output: Fixed point x.y
//  y: out_precision
//
uint32_t private_log2_int_to_fixed(const uint32_t val, const uint8_t out_precision, const uint8_t shift_out)
{
	int i;
	int pos = 0;
	uint32_t a = 0;
	uint32_t b = 0;
	uint32_t in = val;
	uint32_t result = 0;
	const unsigned char precision = out_precision;

	if (0 == val) {
		return 0;
	}
	// integral part
	pos = private_leading_one_position(val);
	// fractional part
	a = (pos <= 15) ? (in << (15 - pos)) : (in >> (pos - 15));
	for (i = 0; i < precision; ++i) {
		b = a * a;
		if (b & (1 << 31))     {
			result = (result << 1) + 1;
			a = b >> 16;
		} else {
			result = (result << 1);
			a = b >> 15;
		}
	}

	return (((pos << precision) + result) << shift_out) | ((a & 0x7fff) >> (15 - shift_out));
}
EXPORT_SYMBOL(private_log2_int_to_fixed);

uint32_t private_log2_int_to_fixed_64(uint64_t val, uint8_t out_precision, uint8_t shift_out)
{
	int i;
	int pos = 0;
	uint64_t a = 0;
	uint64_t b = 0;
	uint64_t in = val;
	uint64_t result = 0;
	const unsigned char precision = out_precision;

	if (0 == val) {
		return 0;
	}
	// integral part
	pos = private_leading_one_position_64(val);
	// fractional part
	a = (pos <= 15) ? (in << (15 - pos)) : (in >> (pos - 15));
	for (i = 0; i < precision; ++i) {
		b = a * a;
		if (b & (1 << 31)) {
			result = (result << 1) + 1;
			a = b >> 16;
		} else {
			result = (result << 1);
			a = b >> 15;
		}
	}

	return (uint32_t)((((pos << precision) + result) << shift_out) | ((a & 0x7fff) >> (15 - shift_out)));
}

uint32_t private_log2_fixed_to_fixed(const uint32_t val, const int in_fix_point, const uint8_t out_fix_point)
{
	return private_log2_int_to_fixed(val, out_fix_point, 0) - (in_fix_point << out_fix_point);
}
EXPORT_SYMBOL(private_log2_fixed_to_fixed);

int32_t private_log2_fixed_to_fixed_64(uint64_t val, int32_t in_fix_point, uint8_t out_fix_point)
{
	return private_log2_int_to_fixed_64(val, out_fix_point, 0) - (in_fix_point << out_fix_point);
}

/* platform interfaces */
int private_platform_driver_register(struct platform_driver *drv)
{
	return platform_driver_register(drv);
}

void private_platform_driver_unregister(struct platform_driver *drv)
{
	platform_driver_unregister(drv);
}

void private_platform_set_drvdata(struct platform_device *pdev, void *data)
{
	platform_set_drvdata(pdev, data);
}

void *private_platform_get_drvdata(struct platform_device *pdev)
{
	return platform_get_drvdata(pdev);
}

int private_platform_device_register(struct platform_device *pdev)
{
	return platform_device_register(pdev);
}

void private_platform_device_unregister(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
}

struct resource *private_platform_get_resource(struct platform_device *dev,
        unsigned int type, unsigned int num)
{
	return platform_get_resource(dev, type, num);
}

void private_dev_set_drvdata(struct device *dev, void *data)
{
	dev_set_drvdata(dev, data);
}

void *private_dev_get_drvdata(const struct device *dev)
{
	return dev_get_drvdata(dev);
}

int private_platform_get_irq(struct platform_device *dev, unsigned int num)
{
	return platform_get_irq(dev, num);
}

struct resource *private_request_mem_region(resource_size_t start, resource_size_t n,
        const char *name)
{
	return request_mem_region(start, n, name);
}

void private_release_mem_region(resource_size_t start, resource_size_t n)
{
	release_mem_region(start, n);
}

void __iomem *private_ioremap(phys_addr_t offset, unsigned long size)
{
	return ioremap(offset, size);
}

void private_iounmap(const volatile void __iomem *addr)
{
	iounmap(addr);
}

/* interrupt interfaces */
int private_request_threaded_irq(unsigned int irq, irq_handler_t handler,
                                 irq_handler_t thread_fn, unsigned long irqflags,
                                 const char *devname, void *dev_id)
{
	return request_threaded_irq(irq, handler, thread_fn, irqflags, devname, dev_id);
}

void private_enable_irq(unsigned int irq)
{
	enable_irq(irq);
}

void private_disable_irq(unsigned int irq)
{
	disable_irq(irq);
}

void private_free_irq(unsigned int irq, void *dev_id)
{
	free_irq(irq, dev_id);
}

/* lock and mutex interfaces */
void __private_spin_lock_irqsave(spinlock_t *lock, unsigned long *flags)
{
	raw_spin_lock_irqsave(spinlock_check(lock), *flags);
}

void private_spin_unlock_irqrestore(spinlock_t *lock, unsigned long flags)
{
	spin_unlock_irqrestore(lock, flags);
}

void private_spin_lock_init(spinlock_t *lock)
{
	spin_lock_init(lock);
}

void private_mutex_lock(struct mutex *lock)
{
	mutex_lock(lock);
}

void private_mutex_unlock(struct mutex *lock)
{
	mutex_unlock(lock);
}

void private_raw_mutex_init(struct mutex *lock, const char *name, struct lock_class_key *key)
{
	__mutex_init(lock, name, key);
}

/* clock interfaces */
struct clk *private_clk_get(struct device *dev, const char *id)
{
	return clk_get(dev, id);
}
EXPORT_SYMBOL(private_clk_get);

struct clk *private_devm_clk_get(struct device *dev, const char *id)
{
	return devm_clk_get(dev, id);
}
EXPORT_SYMBOL(private_devm_clk_get);

int private_clk_enable(struct clk *clk)
{
	return clk_enable(clk);
}
EXPORT_SYMBOL(private_clk_enable);

int private_clk_prepare_enable(struct clk *clk)
{
	return clk_prepare_enable(clk);
}
EXPORT_SYMBOL(private_clk_prepare_enable);

#if 0
int private_clk_is_enabled(struct clk *clk)
{
	return clk_is_enabled(clk);
}
#endif

void private_clk_disable(struct clk *clk)
{
	clk_disable(clk);
}
EXPORT_SYMBOL(private_clk_disable);

void private_clk_disable_unprepare(struct clk *clk)
{
	clk_disable_unprepare(clk);
}
EXPORT_SYMBOL(private_clk_disable_unprepare);

unsigned long private_clk_get_rate(struct clk *clk)
{
	return clk_get_rate(clk);
}
EXPORT_SYMBOL(private_clk_get_rate);

void private_clk_put(struct clk *clk)
{
	return clk_put(clk);
}
EXPORT_SYMBOL(private_clk_put);

void private_devm_clk_put(struct device *dev, struct clk *clk)
{
	return devm_clk_put(dev, clk);
}
EXPORT_SYMBOL(private_devm_clk_put);

int private_clk_set_rate(struct clk *clk, unsigned long rate)
{
	return clk_set_rate(clk, rate);
}
EXPORT_SYMBOL(private_clk_set_rate);

/* i2c interfaces */
struct i2c_adapter *private_i2c_get_adapter(int nr)
{
	return i2c_get_adapter(nr);
}

void private_i2c_put_adapter(struct i2c_adapter *adap)
{
	i2c_put_adapter(adap);
}

int private_i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	return i2c_transfer(adap, msgs, num);
}
EXPORT_SYMBOL(private_i2c_transfer);

int private_i2c_register_driver(struct module *owner, struct i2c_driver *driver)
{
	return i2c_register_driver(owner, driver);
}

void private_i2c_del_driver(struct i2c_driver *drv)
{
	i2c_del_driver(drv);
}
EXPORT_SYMBOL(private_i2c_del_driver);

struct i2c_client *private_i2c_new_device(struct i2c_adapter *adap, struct i2c_board_info const *info)
{
	return i2c_new_client_device(adap, info);
}

void private_i2c_set_clientdata(struct i2c_client *dev, void *data)
{
	i2c_set_clientdata(dev, data);
}
EXPORT_SYMBOL(private_i2c_set_clientdata);

void *private_i2c_get_clientdata(const struct i2c_client *dev)
{
	return i2c_get_clientdata(dev);
}
EXPORT_SYMBOL(private_i2c_get_clientdata);

int private_i2c_add_driver(struct i2c_driver *drv)
{
	return i2c_add_driver(drv);
}
EXPORT_SYMBOL(private_i2c_add_driver);

void private_i2c_unregister_device(struct i2c_client *client)
{
	i2c_unregister_device(client);
}

/* gpio interfaces */
int private_gpio_request(unsigned gpio, const char *label)
{
	return gpio_request(gpio, label);
}
EXPORT_SYMBOL(private_gpio_request);

void private_gpio_free(unsigned gpio)
{
	gpio_free(gpio);
}
EXPORT_SYMBOL(private_gpio_free);

int private_gpio_direction_output(unsigned gpio, int value)
{
	return gpio_direction_output(gpio, value);
}
EXPORT_SYMBOL(private_gpio_direction_output);

int private_gpio_direction_input(unsigned gpio)
{
	return gpio_direction_input(gpio);
}

int private_gpio_set_debounce(unsigned gpio, unsigned debounce)
{
	return gpiod_set_debounce(gpio_to_desc(gpio), debounce);
}

int private_jzgpio_set_func(enum gpio_port port, enum gpio_function func, unsigned long pins)
{
	return jzgpio_set_func(port, func, pins);
}
EXPORT_SYMBOL(private_jzgpio_set_func);

#if 0
int private_jzgpio_ctrl_pull(enum gpio_port port, int enable_pull, unsigned long pins)
{
	return jzgpio_ctrl_pull(port, enable_pull, pins);
}
#endif

/* system interfaces */
void private_msleep(unsigned int msecs)
{
	msleep(msecs);
}
EXPORT_SYMBOL(private_msleep);

bool private_capable(int cap)
{
	return capable(cap);
}
EXPORT_SYMBOL(private_capable);

bool private_try_module_get(struct module *module)
{
	return try_module_get(module);
}

int private_request_module(bool wait, const char *fmt, ...)
{
	int ret = 0;
	struct va_format vaf;
	va_list args;
	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	ret =  __request_module(true, "%pV", &vaf);
	va_end(args);
	return ret;
}

void private_module_put(struct module *module)
{
	module_put(module);
}

/* wait interfaces */
void private_init_completion(struct completion *x)
{
	init_completion(x);
}

void private_complete(struct completion *x)
{
	complete(x);
}

int private_wait_for_completion_interruptible(struct completion *x)
{
	return wait_for_completion_interruptible(x);
}

unsigned long private_wait_for_completion_timeout(struct completion *x, unsigned long timeover)
{
	return wait_for_completion_timeout(x, timeover);
}

int private_wait_event_interruptible(wait_queue_head_t *q, int (*state)(void *), void *data)
{
	return wait_event_interruptible((*q), state(data));
}

void private_wake_up_all(wait_queue_head_t *q)
{
	wake_up_all(q);
}

void private_wake_up(wait_queue_head_t *q)
{
	wake_up(q);
}

void private_init_waitqueue_head(wait_queue_head_t *q)
{
	init_waitqueue_head(q);
}

/* misc driver interfaces */
int private_misc_register(struct miscdevice *mdev)
{
	return misc_register(mdev);
}

void private_misc_deregister(struct miscdevice *mdev)
{
	misc_deregister(mdev);
}

struct proc_dir_entry *private_proc_create_data(const char *name, umode_t mode,
        struct proc_dir_entry *parent,
        const struct proc_ops *proc_ops,
        void *data)
{
	return proc_create_data(name, mode, parent, proc_ops, data);
}

//malloc
void *private_vmalloc(unsigned long size)
{
	void *addr = vmalloc(size);
	return addr;
}

void private_vfree(const void *addr)
{
	vfree(addr);
}

void *private_kmalloc(size_t s, gfp_t gfp)
{
	void *addr = kmalloc(s, gfp);
	return addr;
}

void private_kfree(void *p)
{
	kfree(p);
}

//copy user
long private_copy_from_user(void *to, const void __user *from, long size)
{
	return copy_from_user(to, from, size);
}

long private_copy_to_user(void __user *to, const void *from, long size)
{
	return copy_to_user(to, from, size);
}

//netlink
struct sk_buff *private_nlmsg_new(size_t payload, gfp_t flags)
{
	return nlmsg_new(payload, flags);
}

struct nlmsghdr *private_nlmsg_put(struct sk_buff *skb, u32 portid, u32 seq,
                                   int type, int payload, int flags)
{
	return nlmsg_put(skb, portid, seq, type, payload, flags);
}

int private_netlink_unicast(struct sock *ssk, struct sk_buff *skb,
                            u32 portid, int nonblock)
{
	return netlink_unicast(ssk, skb, portid, nonblock);
}

struct sock *private_netlink_kernel_create(struct net *net, int unit, struct netlink_kernel_cfg *cfg)
{
	return netlink_kernel_create(net, unit, cfg);
}

void private_sock_release(struct socket *sock)
{
	sock_release(sock);
}

/* file ops */
struct file *private_filp_open(const char *filename, int flags, umode_t mode)
{
	return filp_open(filename, flags, mode);
}

int private_filp_close(struct file *filp, fl_owner_t id)
{
	return filp_close(filp, id);
}

ssize_t private_kernel_read(struct file *file, void *buf, size_t count, loff_t *pos)
{
	return kernel_read(file, buf, count, pos);
}
ssize_t private_vfs_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	return vfs_read(file, buf, count, pos);
}
ssize_t private_vfs_write(struct file *file, const char __user *buf, size_t count, loff_t *pos)
{
	return vfs_write(file, buf, count, pos);
}

loff_t private_vfs_llseek(struct file *file, loff_t offset, int whence)
{
	return vfs_llseek(file, offset, whence);
}

void private_dma_cache_sync(struct device *dev, void *vaddr, size_t size,
                            enum dma_data_direction direction)
{
	arch_sync_dma_for_device(virt_to_phys(vaddr), size, direction);
}

void private_getrawmonotonic(struct timespec64 *ts)
{
	ktime_get_raw_ts64(ts);
}

/* kthread interfaces */

bool private_kthread_should_stop(void)
{
	return kthread_should_stop();
}

struct task_struct *private_kthread_run(int (*threadfn)(void *data), void *data, const char namefmt[])
{
	return kthread_run(threadfn, data, namefmt);
}

int private_kthread_stop(struct task_struct *k)
{
	return kthread_stop(k);
}

/* proc file interfaces */
ssize_t private_seq_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	return seq_read(file, buf, size, ppos);
}

loff_t private_seq_lseek(struct file *file, loff_t offset, int whence)
{
	return seq_lseek(file, offset, whence);
}

int private_single_release(struct inode *inode, struct file *file)
{
	return single_release(inode, file);
}

int private_single_open_size(struct file *file, int (*show)(struct seq_file *, void *), void *data, size_t size)
{
	return single_open_size(file, show, data, size);
}

struct proc_dir_entry *private_jz_proc_mkdir(char *s)
{
	return jz_proc_mkdir(s);
}

void private_proc_remove(struct proc_dir_entry *de)
{
	proc_remove(de);
}

void private_seq_printf(struct seq_file *m, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;
	int r = 0;
	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	seq_printf(m, "%pV", &vaf);
	r = m->count;
	va_end(args);
}

unsigned long long private_simple_strtoull(const char *cp, char **endp, unsigned int base)
{
	return simple_strtoull(cp, endp,  base);
}
/*
extern unsigned long ispmem_base;
extern unsigned long ispmem_size;

static void get_isp_priv_mem(unsigned int *phyaddr, unsigned int *size)
{
    *phyaddr = ispmem_base;
    *size = ispmem_size;
}

void private_get_isp_priv_mem(unsigned int *phyaddr, unsigned int *size)
{
    get_isp_priv_mem(phyaddr, size);
}
*/
loff_t get_fsize(struct file *file)
{
	struct inode *inode = NULL;
	loff_t fsize;

	inode = file->f_inode;
	fsize = inode->i_size;

	return fsize;
}

/* struct net *private_get_init_net(void) */
/* { */
/*  return get_init_net(); */
/* } */
