#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/uio.h>
#include <linux/poll.h>
#include <soc/base.h>
#include <linux/workqueue.h>
#include <linux/sched.h>

#include "utils/clock.h"

#include "ring_mem.h"
#include "ingenic_ipc.h"


static void notify_remote(struct ingenic_ipc_device *ipc_dev)
{
	ipc_dev->ipc_ops->notify_remote(ipc_dev);
}

static int wait_ipc_wakeup(struct ingenic_ipc_device *ipc_dev, unsigned long *timeout)
{
	int ret = 0;
	if (!*timeout) {
		return 0;
	}

	ret = wait_event_interruptible_timeout(ipc_dev->wait, ipc_dev->msg_status, msecs_to_jiffies(*timeout));
	if (ret == 0) {
		printk(KERN_ERR "ipc: ipc read data timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static __poll_t ingenic_ipc_poll(struct file *filp, poll_table *wait)
{
	struct ingenic_ipc_device *ipc_dev = (struct ingenic_ipc_device *)filp->private_data;
	__poll_t mask = 0;

	poll_wait(filp, &ipc_dev->wait, wait);

	mask |= EPOLLIN | EPOLLRDNORM;
	return mask;
}

static ssize_t ingenic_ipc_write_iter(struct kiocb *iocb, struct iov_iter *from)
{
	struct file *filp = iocb->ki_filp;
	struct ingenic_ipc_device *ipc_dev = (struct ingenic_ipc_device *)filp->private_data;
	struct ring_mem *ring = ipc_dev->ipc_ops->get_ring_mem_for_host_write(ipc_dev->data);
	unsigned long timeout;
	unsigned long long now;
	int w_size;
	size_t len = iov_iter_count(from);
	int ret;

	ret = len;

	mutex_lock(&ipc_dev->m_lock);

	if (!ring || !ring->mem_size) {
		printk(KERN_ERR "ipc: ring mem is not inited\n");
		ret = -EBUSY;
		goto unlock;
	}

	if (!len) {
		goto unlock;
	}

	ring_mem_set_virt_addr_for_write(ring, ipc_dev->ipc_ops->to_host_addr(ring->mem_addr));

	if (filp->f_flags & O_NONBLOCK) {
		timeout = 0;
	} else {
		timeout = 100;
	}

	now = local_clock_us();
	while (len) {
		w_size = ring_mem_write(ring, from, len);
		if (!w_size) {
			if (!timeout) {
				break;
			}

			if (local_clock_us() - now > timeout) {
				printk(KERN_ERR "ipc: ipc write mem timeout\n");
				break;
			}
			usleep_range(timeout, timeout);
		}
		len -= w_size;
	}

	ret = ret - len;
	if (ret) {
		notify_remote(ipc_dev);
	}

unlock:
	mutex_unlock(&ipc_dev->m_lock);

	return ret;
}

static ssize_t ingenic_ipc_read_iter(struct kiocb *iocb, struct iov_iter *to)
{
	struct file *filp = iocb->ki_filp;
	struct ingenic_ipc_device *ipc_dev = (struct ingenic_ipc_device *)filp->private_data;
	struct ring_mem *ring = ipc_dev->ipc_ops->get_ring_mem_for_host_read(ipc_dev->data);
	unsigned long timeout = 100;
	int r_size;
	int size = iov_iter_count(to);
	int ret = size;

	mutex_lock(&ipc_dev->m_lock);

	if (!ring || !ring->mem_size) {
		printk(KERN_ERR "ipc: ring mem is not inited\n");
		ret = -EBUSY;
		goto unlock;
	}

	if (!size) {
		goto unlock;
	}

	ring_mem_set_virt_addr_for_read(ring, ipc_dev->ipc_ops->to_host_addr(ring->mem_addr));

	if (ring_mem_readable_size(ring)) {
		ipc_dev->msg_status = 1;
	}

	if (filp->f_flags & O_NONBLOCK) {
		timeout = 0;
	} else {
		timeout = 100;
	}

	if (wait_ipc_wakeup(ipc_dev, &timeout)) {
		ret = -ETIMEDOUT;
		goto unlock;
	}

	ipc_dev->msg_status = 0;

	while (size) {
		r_size = ring_mem_read(ring, to, size);
		if (!r_size) {
			if (!timeout) {
				break;
			}

			if (wait_ipc_wakeup(ipc_dev, &timeout)) {
				printk(KERN_ERR "ipc: ipc read mem timeout\n");
				break;
			}
		}
		size -= r_size;
	}
	ret = ret - size;

unlock:
	mutex_unlock(&ipc_dev->m_lock);
	return ret;
}

static int ingenic_ipc_open(struct inode *inode, struct file *filp)
{
	struct ingenic_ipc_device *ipc_dev = container_of(inode->i_cdev, struct ingenic_ipc_device, cdev);
	filp->private_data = ipc_dev;
	return 0;
}
static int ingenic_ipc_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static struct file_operations ingenic_ipc_fops = {
	.open = ingenic_ipc_open,
	.release = ingenic_ipc_release,
	.read_iter = ingenic_ipc_read_iter,
	.write_iter = ingenic_ipc_write_iter,
	.poll = ingenic_ipc_poll,
};

static int ingenic_ipc_nb_call(struct notifier_block *nb, unsigned long val, void *nouse)
{
	struct ingenic_ipc_device *ipc_dev = container_of(nb, struct ingenic_ipc_device, nb);

	ipc_dev->msg_status = val;
	wake_up_interruptible(&ipc_dev->wait);

	return 0;
}

#define DEVICE_NAME "ipc-dev"
#define MINOR_COUNT 2 //riscv mcu
static int current_minor = 0;
static dev_t dev_num;
static struct class *ipcdev_class = NULL;

struct ingenic_ipc_device *ingenic_ipc_device_register(void *data, struct ingenic_ipc_ops *ipc_ops, char *name)
{
	struct ingenic_ipc_device *ipc_dev = NULL;

	if ((!ipc_ops->notify_remote) ||
	    (!ipc_ops->get_ring_mem_for_host_write) ||
	    (!ipc_ops->get_ring_mem_for_host_read) ||
	    (!ipc_ops->to_host_addr) ||
	    (!ipc_ops->notifier_register) ||
	    (!ipc_ops->notifier_unregister)) {

		printk(KERN_ERR "please set ingenic_ipc_ops correctly\n");
		return NULL;
	}

	ipc_dev = kzalloc(sizeof(struct ingenic_ipc_device), GFP_KERNEL);

	//device create
	if (current_minor >= MINOR_COUNT) {
		printk(KERN_ERR "No more devices can be created\n");
		return NULL;
	}
	cdev_init(&ipc_dev->cdev, &ingenic_ipc_fops);
	if (cdev_add(&ipc_dev->cdev, MKDEV(MAJOR(dev_num), current_minor), 1) == -1) {
		printk(KERN_ERR "Failed to add cdev\n");
		return NULL;
	}
	if (device_create(ipcdev_class, NULL, MKDEV(MAJOR(dev_num), current_minor), NULL, "%s-%s", DEVICE_NAME, name) == NULL) {
		cdev_del(&ipc_dev->cdev);
		printk(KERN_ERR "Failed to create the device\n");
		return NULL;
	}
	ipc_dev->minor = current_minor;
	current_minor++;

	ipc_dev->ipc_ops = ipc_ops;

	//register notifier
	ipc_dev->nb.notifier_call = ingenic_ipc_nb_call;
	ipc_dev->nb.priority = 0;  //0 first
	ipc_dev->ipc_ops->notifier_register(&ipc_dev->nb);


	init_waitqueue_head(&ipc_dev->wait);

	mutex_init(&ipc_dev->m_lock);

	ipc_dev->data = data;

	printk("ingenic ipc device regist OK!\n");

	return ipc_dev;
}

void ingenic_ipc_device_unregister(struct ingenic_ipc_device *ipc_dev)
{
	ipc_dev->ipc_ops->notifier_unregister(&ipc_dev->nb);
	device_destroy(ipcdev_class, MKDEV(MAJOR(dev_num), ipc_dev->minor));
	current_minor--;
}

static int __init ingenic_ipc_driver_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&dev_num, 0, MINOR_COUNT, "ipc-dev");
	if (ret < 0) {
		printk(KERN_ERR "Failed to allocate a device number\n");
		return ret;
	}

	ipcdev_class = class_create("ipc-dev");
	if (IS_ERR(ipcdev_class)) {
		unregister_chrdev_region(dev_num, MINOR_COUNT);
		return PTR_ERR(ipcdev_class);
	}

	printk(KERN_INFO "ipc_dev: Device class created correctly\n");
	return 0;
}

static void __exit ingenic_ipc_driver_exit(void)
{
	class_destroy(ipcdev_class);
	unregister_chrdev_region(dev_num, MINOR_COUNT);
	printk(KERN_INFO "MyCharDevice: Goodbye from the LKM!\n");
}

module_init(ingenic_ipc_driver_init);
module_exit(ingenic_ipc_driver_exit);
MODULE_LICENSE("GPL");
