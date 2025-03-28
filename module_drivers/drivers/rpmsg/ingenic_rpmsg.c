#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/rpmsg.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <uapi/linux/rpmsg.h>

#include "rpmsg_internal.h"

#define INGENIC_RPMSG_DEV_MAX   (MINORMASK + 1)

static dev_t ingenic_rpmsg_major;
static struct class *ingenic_rpmsg_class;

static DEFINE_IDA(ingenic_rpmsg_minor_ida);

#define dev_to_ingenic_dev(dev) container_of(dev, struct ingenic_rpmsg_dev, dev)
#define cdev_to_ingenic_dev(i_cdev) container_of(i_cdev, struct ingenic_rpmsg_dev, cdev)

/**
 * struct ingenic_rpmsg - control device for instantiating endpoint devices
 * @rpdev:  underlaying rpmsg device
 * @cdev:   cdev for the ctrl device
 * @dev:    device for the ctrl device
 * @chinfo: info used to open the endpoint
 * @ept_lock:   synchronization of @ept modifications
 * @ept:    rpmsg endpoint reference, when open
 * @queue_lock: synchronization of @queue operations
 * @queue:  incoming message queue
 * @readq:  wait object for incoming queue
 */
struct ingenic_rpmsg_dev {
	struct rpmsg_device *rpdev;
	struct cdev cdev;
	struct device dev;

	struct rpmsg_channel_info chinfo;

	struct mutex ept_lock;
	struct rpmsg_endpoint *ept;

	spinlock_t queue_lock;
	struct sk_buff_head queue;
	wait_queue_head_t readq;
};

static int rpmsg_ept_cb(struct rpmsg_device *rpdev, void *buf, int len,
                        void *priv, u32 addr)
{
	struct ingenic_rpmsg_dev *ingenic_rpmsg = priv;
	struct sk_buff *skb;

	unsigned long flags;
	skb = alloc_skb(len, GFP_ATOMIC);
	if (!skb) {
		return -ENOMEM;
	}

	skb_put_data(skb, buf, len);

	spin_lock_irqsave(&ingenic_rpmsg->queue_lock, flags);
	skb_queue_tail(&ingenic_rpmsg->queue, skb);
	spin_unlock_irqrestore(&ingenic_rpmsg->queue_lock, flags);

	/* wake up any blocking processes, waiting for new data */
	wake_up_interruptible(&ingenic_rpmsg->readq);

	return 0;
}

static int ingenic_rpmsg_open(struct inode *inode, struct file *filp)
{
	struct ingenic_rpmsg_dev *ingenic_rpmsg = cdev_to_ingenic_dev(inode->i_cdev);

	filp->private_data = ingenic_rpmsg;

	return 0;
}

static int ingenic_rpmsg_release(struct inode *inode, struct file *filp)
{
	struct ingenic_rpmsg_dev *ingenic_rpmsg = cdev_to_ingenic_dev(inode->i_cdev);
	unsigned long flags;

	spin_lock_irqsave(&ingenic_rpmsg->queue_lock, flags);
	skb_queue_purge(&ingenic_rpmsg->queue);
	spin_unlock_irqrestore(&ingenic_rpmsg->queue_lock, flags);
	wake_up_interruptible(&ingenic_rpmsg->readq);

	return 0;
}

static ssize_t ingenic_rpmsg_read_iter(struct kiocb *iocb, struct iov_iter *to)
{
	struct file *filp = iocb->ki_filp;
	struct ingenic_rpmsg_dev *ingenic_rpmsg = filp->private_data;
	unsigned long flags;
	struct sk_buff *skb;
	int use;

	if (!ingenic_rpmsg->ept) {
		return -EPIPE;
	}

	spin_lock_irqsave(&ingenic_rpmsg->queue_lock, flags);

	/* Wait for data in the queue */
	if (skb_queue_empty(&ingenic_rpmsg->queue)) {
		spin_unlock_irqrestore(&ingenic_rpmsg->queue_lock, flags);

		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}

		/* Wait until we get data or the endpoint goes away */
		if (wait_event_interruptible(ingenic_rpmsg->readq,
		                             !skb_queue_empty(&ingenic_rpmsg->queue) ||
		                             !ingenic_rpmsg->ept)) {
			return -ERESTARTSYS;
		}

		/* We lost the endpoint while waiting */
		if (!ingenic_rpmsg->ept) {
			return -EPIPE;
		}

		spin_lock_irqsave(&ingenic_rpmsg->queue_lock, flags);
	}

	skb = skb_dequeue(&ingenic_rpmsg->queue);
	spin_unlock_irqrestore(&ingenic_rpmsg->queue_lock, flags);
	if (!skb) {
		return -EFAULT;
	}

	use = min_t(size_t, iov_iter_count(to), skb->len);
	if (copy_to_iter(skb->data, use, to) != use) {
		use = -EFAULT;
	}

	kfree_skb(skb);

	return use;
}

static ssize_t ingenic_rpmsg_write_iter(struct kiocb *iocb,
                                        struct iov_iter *from)
{
	struct file *filp = iocb->ki_filp;
	struct ingenic_rpmsg_dev *ingenic_rpmsg = filp->private_data;
	size_t len = iov_iter_count(from);
	void *kbuf;
	int ret;

	kbuf = kzalloc(len, GFP_KERNEL);
	if (!kbuf) {
		return -ENOMEM;
	}

	if (!copy_from_iter_full(kbuf, len, from)) {
		ret = -EFAULT;
		goto free_kbuf;
	}

	if (mutex_lock_interruptible(&ingenic_rpmsg->ept_lock)) {
		ret = -ERESTARTSYS;
		goto free_kbuf;
	}

	if (!ingenic_rpmsg->ept) {
		ret = -EPIPE;
		goto unlock_eptdev;
	}

	if (filp->f_flags & O_NONBLOCK) {
		ret = rpmsg_trysend(ingenic_rpmsg->ept, kbuf, len);
	} else {
		ret = rpmsg_send(ingenic_rpmsg->ept, kbuf, len);
	}

unlock_eptdev:
	mutex_unlock(&ingenic_rpmsg->ept_lock);

free_kbuf:
	kfree(kbuf);
	return ret < 0 ? ret : len;
}

static __poll_t ingenic_rpmsg_poll(struct file *filp, poll_table *wait)
{
	struct ingenic_rpmsg_dev *ingenic_rpmsg = filp->private_data;
	unsigned long flags;
	__poll_t mask = 0;

	if (!ingenic_rpmsg->ept) {
		return EPOLLERR;
	}

	poll_wait(filp, &ingenic_rpmsg->readq, wait);

	spin_lock_irqsave(&ingenic_rpmsg->queue_lock, flags);
	if (!skb_queue_empty(&ingenic_rpmsg->queue)) {
		mask |= EPOLLIN | EPOLLRDNORM;
	}

	spin_unlock_irqrestore(&ingenic_rpmsg->queue_lock, flags);
	mask |= rpmsg_poll(ingenic_rpmsg->ept, filp, wait);

	return mask;
}

static long ingenic_rpmsg_ioctl(struct file *fp, unsigned int cmd,
                                unsigned long arg)
{

	return 0;
}

static const struct file_operations ingenic_rpmsg_fops = {
	.owner = THIS_MODULE,
	.open = ingenic_rpmsg_open,
	.release = ingenic_rpmsg_release,
	.read_iter = ingenic_rpmsg_read_iter,
	.write_iter = ingenic_rpmsg_write_iter,
	.poll = ingenic_rpmsg_poll,
	.unlocked_ioctl = ingenic_rpmsg_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static void ingenic_rpmsg_dev_release_device(struct device *dev)
{
	struct ingenic_rpmsg_dev *ingenic_rpmsg = dev_to_ingenic_dev(dev);

	/* Close the endpoint*/
	mutex_lock(&ingenic_rpmsg->ept_lock);
	if (ingenic_rpmsg->ept) {
		rpmsg_destroy_ept(ingenic_rpmsg->ept);
		ingenic_rpmsg->ept = NULL;
	}
	mutex_unlock(&ingenic_rpmsg->ept_lock);

	/* wake up any blocked readers */
	wake_up_interruptible(&ingenic_rpmsg->readq);
	/* Discard all SKBs */
	skb_queue_purge(&ingenic_rpmsg->queue);

	put_device(dev);

	ida_simple_remove(&ingenic_rpmsg_minor_ida, MINOR(dev->devt));
	cdev_del(&ingenic_rpmsg->cdev);
	kfree(ingenic_rpmsg);
}

static int ingenic_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct ingenic_rpmsg_dev *ingenic_rpmsg;
	struct rpmsg_channel_info *chinfo = NULL;
	struct device *dev;
	int ret;

	ingenic_rpmsg = kzalloc(sizeof(*ingenic_rpmsg), GFP_KERNEL);
	if (!ingenic_rpmsg) {
		return -ENOMEM;
	}

	ingenic_rpmsg->rpdev = rpdev;

	dev = &ingenic_rpmsg->dev;
	device_initialize(dev);
	dev->parent = &rpdev->dev;
	dev->class = ingenic_rpmsg_class;

	cdev_init(&ingenic_rpmsg->cdev, &ingenic_rpmsg_fops);
	ingenic_rpmsg->cdev.owner = THIS_MODULE;

	ret = ida_simple_get(&ingenic_rpmsg_minor_ida, 0, INGENIC_RPMSG_DEV_MAX, GFP_KERNEL);
	if (ret < 0) {
		goto free_ctrldev;
	}
	dev->devt = MKDEV(MAJOR(ingenic_rpmsg_major), ret);

	dev->id = ret;

	dev_set_name(&ingenic_rpmsg->dev, "rpmsg-%d", rpdev->dst);

	ret = cdev_add(&ingenic_rpmsg->cdev, dev->devt, 1);
	if (ret) {
		goto free_ctrl_ida;
	}

	/* We can now rely on the release function for cleanup */
	dev->release = ingenic_rpmsg_dev_release_device;

	ret = device_add(dev);
	if (ret) {
		dev_err(&rpdev->dev, "device_add failed: %d\n", ret);
		put_device(dev);
	}

	mutex_init(&ingenic_rpmsg->ept_lock);
	spin_lock_init(&ingenic_rpmsg->queue_lock);
	skb_queue_head_init(&ingenic_rpmsg->queue);
	init_waitqueue_head(&ingenic_rpmsg->readq);

	dev_set_drvdata(&rpdev->dev, ingenic_rpmsg);

	get_device(dev);

	/*create ept*/
	chinfo = &ingenic_rpmsg->chinfo;
	memcpy(chinfo->name, rpdev->id.name, RPMSG_NAME_SIZE);
	chinfo->name[RPMSG_NAME_SIZE - 1] = '\0';
	/*一对一端点，使用相同地址*/
	chinfo->src = rpdev->dst;
	ingenic_rpmsg->ept = rpmsg_create_ept(rpdev, rpmsg_ept_cb, ingenic_rpmsg, ingenic_rpmsg->chinfo);
	if (!ingenic_rpmsg->ept) {
		dev_err(dev, "failed to open %s\n", ingenic_rpmsg->chinfo.name);
		put_device(dev);
		return -EINVAL;
	}

	return ret;

free_ctrl_ida:
	ida_simple_remove(&ingenic_rpmsg_minor_ida, MINOR(dev->devt));
free_ctrldev:
	put_device(dev);
	kfree(ingenic_rpmsg);

	return ret;
}

static void ingenic_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct ingenic_rpmsg_dev *ingenic_rpmsg = dev_get_drvdata(&rpdev->dev);

	mutex_lock(&ingenic_rpmsg->ept_lock);
	if (ingenic_rpmsg->ept) {
		rpmsg_destroy_ept(ingenic_rpmsg->ept);
		ingenic_rpmsg->ept = NULL;
	}
	mutex_unlock(&ingenic_rpmsg->ept_lock);

	device_del(&ingenic_rpmsg->dev);
	put_device(&ingenic_rpmsg->dev);
}

static struct rpmsg_device_id ingenic_rpmsg_id_table[] = {
	{.name = "ingenic_rpmsg"},
	{},
};

static struct rpmsg_driver ingenic_rpmsg_driver = {
	.probe = ingenic_rpmsg_probe,
	.remove = ingenic_rpmsg_remove,
	.id_table = ingenic_rpmsg_id_table,
	.drv = {
		.name = "ingenic_rpmsg",
	},
};

static int ingenic_rpmsg_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&ingenic_rpmsg_major, 0, INGENIC_RPMSG_DEV_MAX, "ingenic_rpmsg");
	if (ret < 0) {
		pr_err("ingenic_rpmsg: failed to allocate char dev region\n");
		return ret;
	}

	ingenic_rpmsg_class = class_create("ingenic_rpmsg");
	if (IS_ERR(ingenic_rpmsg_class)) {
		pr_err("failed to create ingenic_rpmsg class\n");
		unregister_chrdev_region(ingenic_rpmsg_major, INGENIC_RPMSG_DEV_MAX);
		return PTR_ERR(ingenic_rpmsg_class);
	}

	ret = register_rpmsg_driver(&ingenic_rpmsg_driver);
	if (ret < 0) {
		pr_err("failed to register ingenic_rpmsg driver\n");
		class_destroy(ingenic_rpmsg_class);
		unregister_chrdev_region(ingenic_rpmsg_major, INGENIC_RPMSG_DEV_MAX);
	}

	return ret;
}
postcore_initcall(ingenic_rpmsg_init);

static void ingenic_rpmsg_exit(void)
{
	unregister_rpmsg_driver(&ingenic_rpmsg_driver);
	class_destroy(ingenic_rpmsg_class);
	unregister_chrdev_region(ingenic_rpmsg_major, INGENIC_RPMSG_DEV_MAX);
}
module_exit(ingenic_rpmsg_exit);

MODULE_ALIAS("rpmsg:ingenic_rpmsg");
MODULE_LICENSE("GPL v2");
