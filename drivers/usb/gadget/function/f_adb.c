/*
 * Gadget Driver for Android ADB
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/configfs.h>
#include <linux/usb/composite.h>

#include "configfs.h"


/* String IDs */
#define INTERFACE_STRING_INDEX  0

#define DRIVER_NAME "adb"

#define ADB_BULK_BUFFER_SIZE           4096
/* number of tx requests to allocate */
#define TX_REQ_MAX 4
static const char adb_shortname[] = "android_adb";
struct adb_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;
	struct usb_ep *ep_in;
	struct usb_ep *ep_out;
	atomic_t online;
	atomic_t error;
	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;
	struct list_head tx_idle;
	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	struct usb_request *rx_req;
	int rx_done;
	bool notify_close;
	bool close_notified;
};
static struct usb_interface_descriptor adb_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = 0xFF,
	.bInterfaceSubClass     = 0x42,
	.bInterfaceProtocol     = 1,
};
static struct usb_endpoint_descriptor adb_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};
static struct usb_endpoint_descriptor adb_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};
static struct usb_endpoint_descriptor adb_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};
static struct usb_endpoint_descriptor adb_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};
static struct usb_descriptor_header *fs_adb_descs[] = {
	(struct usb_descriptor_header *) &adb_interface_desc,
	(struct usb_descriptor_header *) &adb_fullspeed_in_desc,
	(struct usb_descriptor_header *) &adb_fullspeed_out_desc,
	NULL,
};
static struct usb_descriptor_header *hs_adb_descs[] = {
	(struct usb_descriptor_header *) &adb_interface_desc,
	(struct usb_descriptor_header *) &adb_highspeed_in_desc,
	(struct usb_descriptor_header *) &adb_highspeed_out_desc,
	NULL,
};

static struct usb_string adb_string_defs[] = {
	[INTERFACE_STRING_INDEX].s  = "ADB",
	{  },   /* end of list */
};

static struct usb_gadget_strings adb_string_table = {
	.language       = 0x0409,   /* en-US */
	.strings        = adb_string_defs,
};

static struct usb_gadget_strings *adb_strings[] = {
	&adb_string_table,
	NULL,
};

static void adb_ready_callback(void)
{

}

static void adb_closed_callback(void)
{

}

struct adb_instance {
	struct usb_function_instance func_inst;
	struct adb_dev *dev;
	char adb_ext_compat_id[16];
	struct usb_os_desc adb_os_desc;
};

/* temporary variable used between adb_open() and adb_gadget_bind() */
static struct adb_dev *_adb_dev;
static inline struct adb_dev *func_to_adb(struct usb_function *f)
{
	return container_of(f, struct adb_dev, function);
}
static struct usb_request *adb_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req) {
		return NULL;
	}
	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}
	return req;
}
static void adb_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}
static inline int adb_lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}
static inline void adb_unlock(atomic_t *excl)
{
	if (atomic_dec_return(excl) < 0) {
		atomic_inc(excl);
	}
}
/* add a request to the tail of a list */
void adb_req_put(struct adb_dev *dev, struct list_head *head,
                 struct usb_request *req)
{
	unsigned long flags;
	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}
/* remove a request from the head of a list */
struct usb_request *adb_req_get(struct adb_dev *dev, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;
	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}
static void adb_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct adb_dev *dev = _adb_dev;
	if (req->status != 0) {
		atomic_set(&dev->error, 1);
	}
	adb_req_put(dev, &dev->tx_idle, req);
	wake_up(&dev->write_wq);
}
static void adb_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct adb_dev *dev = _adb_dev;
	dev->rx_done = 1;
	if (req->status != 0 && req->status != -ECONNRESET) {
		atomic_set(&dev->error, 1);
	}
	wake_up(&dev->read_wq);
}
static int adb_create_bulk_endpoints(struct adb_dev *dev,
                                     struct usb_endpoint_descriptor *in_desc,
                                     struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;
	DBG(cdev, "create_bulk_endpoints dev: %p\n", dev);
	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev;      /* claim the endpoint */
	dev->ep_in = ep;
	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for adb ep_out got %s\n", ep->name);
	ep->driver_data = dev;      /* claim the endpoint */
	dev->ep_out = ep;
	/* now allocate requests for our endpoints */
	req = adb_request_new(dev->ep_out, ADB_BULK_BUFFER_SIZE);
	if (!req) {
		goto fail;
	}
	req->complete = adb_complete_out;
	dev->rx_req = req;
	for (i = 0; i < TX_REQ_MAX; i++) {
		req = adb_request_new(dev->ep_in, ADB_BULK_BUFFER_SIZE);
		if (!req) {
			goto fail;
		}
		req->complete = adb_complete_in;
		adb_req_put(dev, &dev->tx_idle, req);
	}
	return 0;
fail:
	printk(KERN_ERR "adb_bind() could not allocate requests\n");
	return -1;
}

static void adb_free_bulk_endpoints(struct adb_dev *dev)
{
	struct usb_request *req;

	adb_request_free(dev->rx_req, dev->ep_out);
	while ((req = adb_req_get(dev, &dev->tx_idle))) {
		adb_request_free(req, dev->ep_in);
	}
}


static ssize_t adb_read(struct file *fp, char __user *buf,
                        size_t count, loff_t *pos)
{
	struct adb_dev *dev = fp->private_data;
	struct usb_request *req;
	int r = count, xfer;
	int ret;
	pr_debug("adb_read(%d)\n", count);
	if (!_adb_dev) {
		return -ENODEV;
	}
	if (count > ADB_BULK_BUFFER_SIZE) {
		return -EINVAL;
	}
	if (adb_lock(&dev->read_excl)) {
		return -EBUSY;
	}
	/* we will block until we're online */
	while (!(atomic_read(&dev->online) || atomic_read(&dev->error))) {
		pr_debug("adb_read: waiting for online state\n");
		ret = wait_event_interruptible(dev->read_wq,
		                               (atomic_read(&dev->online) ||
		                                atomic_read(&dev->error)));
		if (ret < 0) {
			adb_unlock(&dev->read_excl);
			return ret;
		}
	}
	if (atomic_read(&dev->error)) {
		r = -EIO;
		goto done;
	}
requeue_req:
	/* queue a request */
	req = dev->rx_req;
	req->length = ADB_BULK_BUFFER_SIZE;
	dev->rx_done = 0;
	ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
	if (ret < 0) {
		pr_debug("adb_read: failed to queue req %p (%d)\n", req, ret);
		r = -EIO;
		atomic_set(&dev->error, 1);
		goto done;
	} else {
		pr_debug("rx %p queue\n", req);
	}
	/* wait for a request to complete */
	ret = wait_event_interruptible(dev->read_wq, dev->rx_done ||
	                               atomic_read(&dev->error));
	if (ret < 0) {
		if (ret != -ERESTARTSYS) {
			atomic_set(&dev->error, 1);
		}
		r = ret;
		usb_ep_dequeue(dev->ep_out, req);
		goto done;
	}
	if (!atomic_read(&dev->error)) {
		/* If we got a 0-len packet, throw it back and try again. */
		if (req->actual == 0) {
			goto requeue_req;
		}
		pr_debug("rx %p %d\n", req, req->actual);
		xfer = (req->actual < count) ? req->actual : count;
		if (copy_to_user(buf, req->buf, xfer)) {
			r = -EFAULT;
		}
	} else {
		r = -EIO;
	}
done:
	if (atomic_read(&dev->error)) {
		wake_up(&dev->write_wq);
	}
	adb_unlock(&dev->read_excl);
	pr_debug("adb_read returning %d\n", r);
	return r;
}
static ssize_t adb_write(struct file *fp, const char __user *buf,
                         size_t count, loff_t *pos)
{
	struct adb_dev *dev = fp->private_data;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;
	if (!_adb_dev) {
		return -ENODEV;
	}
	pr_debug("adb_write(%d)\n", count);
	if (adb_lock(&dev->write_excl)) {
		return -EBUSY;
	}
	while (count > 0) {
		if (atomic_read(&dev->error)) {
			pr_debug("adb_write dev->error\n");
			r = -EIO;
			break;
		}
		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(dev->write_wq,
		                               ((req = adb_req_get(dev, &dev->tx_idle)) ||
		                                atomic_read(&dev->error)));
		if (ret < 0) {
			r = ret;
			break;
		}
		if (req != 0) {
			if (count > ADB_BULK_BUFFER_SIZE) {
				xfer = ADB_BULK_BUFFER_SIZE;
			} else {
				xfer = count;
			}
			if (copy_from_user(req->buf, buf, xfer)) {
				r = -EFAULT;
				break;
			}
			req->length = xfer;
			ret = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
			if (ret < 0) {
				pr_debug("adb_write: xfer error %d\n", ret);
				atomic_set(&dev->error, 1);
				r = -EIO;
				break;
			}
			buf += xfer;
			count -= xfer;
			/* zero this so we don't try to free it on error exit */
			req = 0;
		}
	}
	if (req) {
		adb_req_put(dev, &dev->tx_idle, req);
	}
	if (atomic_read(&dev->error)) {
		wake_up(&dev->read_wq);
	}
	adb_unlock(&dev->write_excl);
	pr_debug("adb_write returning %d\n", r);
	return r;
}
static int adb_open(struct inode *ip, struct file *fp)
{
	pr_debug("adb_open\n");
	if (!_adb_dev) {
		return -ENODEV;
	}
	if (adb_lock(&_adb_dev->open_excl)) {
		return -EBUSY;
	}
	fp->private_data = _adb_dev;
	/* clear the error latch */
	atomic_set(&_adb_dev->error, 0);
	if (_adb_dev->close_notified) {
		_adb_dev->close_notified = false;
		adb_ready_callback();
	}
	_adb_dev->notify_close = true;
	return 0;
}
static int adb_release(struct inode *ip, struct file *fp)
{
	pr_debug("adb_release\n");
	/*
	 * ADB daemon closes the device file after I/O error.  The
	 * I/O error happen when Rx requests are flushed during
	 * cable disconnect or bus reset in configured state.  Disabling
	 * USB configuration and pull-up during these scenarios are
	 * undesired.  We want to force bus reset only for certain
	 * commands like "adb root" and "adb usb".
	 */
	if (_adb_dev->notify_close) {
		adb_closed_callback();
		_adb_dev->close_notified = true;
	}
	adb_unlock(&_adb_dev->open_excl);
	return 0;
}
/* file operations for ADB device /dev/android_adb */
static const struct file_operations adb_fops = {
	.owner = THIS_MODULE,
	.read = adb_read,
	.write = adb_write,
	.open = adb_open,
	.release = adb_release,
};
static struct miscdevice adb_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = adb_shortname,
	.fops = &adb_fops,
};

static int adb_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct adb_dev  *dev = func_to_adb(f);
	struct usb_string   *us;
	int         id;
	int         ret;
	dev->cdev = cdev;
	DBG(cdev, "adb_function_bind dev: %p\n", dev);

	/* maybe allocate device-global string IDs, and patch descriptors */
	us = usb_gstrings_attach(cdev, adb_strings,
	                         ARRAY_SIZE(adb_string_defs));
	if (IS_ERR(us)) {
		return PTR_ERR(us);
	}
	adb_interface_desc.iInterface = us[INTERFACE_STRING_INDEX].id;

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0) {
		return id;
	}
	adb_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = adb_create_bulk_endpoints(dev, &adb_fullspeed_in_desc,
	                                &adb_fullspeed_out_desc);
	if (ret) {
		return ret;
	}

	adb_highspeed_in_desc.bEndpointAddress = adb_fullspeed_in_desc.bEndpointAddress;
	adb_highspeed_out_desc.bEndpointAddress = adb_fullspeed_out_desc.bEndpointAddress;

	ret = usb_assign_descriptors(f, fs_adb_descs, hs_adb_descs, NULL, NULL);
	if (ret) {
		goto fail;
	}

	DBG(cdev, "%s speed %s: IN/%s, OUT/%s\n",
	    gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
	    f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;

fail:
	adb_free_bulk_endpoints(dev);
	return ret;
}

static void adb_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct adb_dev  *dev = func_to_adb(f);
	atomic_set(&dev->online, 0);
	atomic_set(&dev->error, 1);
	wake_up(&dev->read_wq);
	adb_free_bulk_endpoints(dev);
}

static int adb_function_set_alt(struct usb_function *f,
                                unsigned intf, unsigned alt)
{
	struct adb_dev  *dev = func_to_adb(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;
	DBG(cdev, "adb_function_set_alt intf: %d alt: %d\n", intf, alt);
	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_in);
	if (ret) {
		dev->ep_in->desc = NULL;
		ERROR(cdev, "config_ep_by_speed failes for ep %s, result %d\n",
		      dev->ep_in->name, ret);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_in);
	if (ret) {
		ERROR(cdev, "failed to enable ep %s, result %d\n",
		      dev->ep_in->name, ret);
		return ret;
	}
	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_out);
	if (ret) {
		dev->ep_out->desc = NULL;
		ERROR(cdev, "config_ep_by_speed failes for ep %s, result %d\n",
		      dev->ep_out->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_out);
	if (ret) {
		ERROR(cdev, "failed to enable ep %s, result %d\n",
		      dev->ep_out->name, ret);
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	atomic_set(&dev->online, 1);
	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);
	return 0;
}
static void adb_function_disable(struct usb_function *f)
{
	struct adb_dev  *dev = func_to_adb(f);
	struct usb_composite_dev    *cdev = dev->cdev;
	DBG(cdev, "adb_function_disable cdev %p\n", cdev);
	/*
	 * Bus reset happened or cable disconnected.  No
	 * need to disable the configuration now.  We will
	 * set noify_close to true when device file is re-opened.
	 */
	dev->notify_close = false;
	atomic_set(&dev->online, 0);
	atomic_set(&dev->error, 1);
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);
	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);
	VDBG(cdev, "%s disabled\n", dev->function.name);
}

static int adb_setup(struct adb_instance *fi_adb)
{
	struct adb_dev *dev;
	int ret;

	if (fi_adb->dev) {
		return -EBUSY;
	}

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		return -ENOMEM;
	}
	spin_lock_init(&dev->lock);
	init_waitqueue_head(&dev->read_wq);
	init_waitqueue_head(&dev->write_wq);
	atomic_set(&dev->open_excl, 0);
	atomic_set(&dev->read_excl, 0);
	atomic_set(&dev->write_excl, 0);
	/* config is disabled by default if adb is present. */
	dev->close_notified = true;
	INIT_LIST_HEAD(&dev->tx_idle);

	_adb_dev = dev;
	ret = misc_register(&adb_device);
	if (ret) {
		goto err;
	}

	fi_adb->dev = dev;
	return 0;
err:
	kfree(dev);
	printk(KERN_ERR "adb gadget driver failed to initialize\n");
	return ret;
}

static void adb_cleanup(void)
{
	struct adb_dev *dev = _adb_dev;

	if (!dev) {
		return;
	}

	misc_deregister(&adb_device);
	kfree(_adb_dev);
	_adb_dev = NULL;
}

static struct adb_instance *to_adb_instance(struct config_item *item)
{
	return container_of(to_config_group(item), struct adb_instance, func_inst.group);
}

static void adb_attr_release(struct config_item *item)
{
	struct adb_instance *fi_adb = to_adb_instance(item);

	usb_put_function_instance(&fi_adb->func_inst);
}

static struct configfs_item_operations adb_item_ops = {
	.release        = adb_attr_release,
};

static struct config_item_type adb_func_type = {
	.ct_item_ops    = &adb_item_ops,
	.ct_owner       = THIS_MODULE,
};

static struct adb_instance *to_fi_adb(struct usb_function_instance *fi)
{
	return container_of(fi, struct adb_instance, func_inst);
}

static void adb_free_inst(struct usb_function_instance *fi)
{
	struct adb_instance *fi_adb;

	fi_adb = to_fi_adb(fi);
	adb_cleanup();
	kfree(fi_adb);
}


static struct usb_function_instance *adb_alloc_inst(void)
{
	struct adb_instance *fi_adb;
	int ret = 0;
	struct usb_os_desc *descs[1];
	char *names[1];

	fi_adb = kzalloc(sizeof(*fi_adb), GFP_KERNEL);
	if (!fi_adb) {
		return ERR_PTR(-ENOMEM);
	}
	fi_adb->func_inst.free_func_inst = adb_free_inst;

	fi_adb->adb_os_desc.ext_compat_id = fi_adb->adb_ext_compat_id;
	INIT_LIST_HEAD(&fi_adb->adb_os_desc.ext_prop);
	descs[0] = &fi_adb->adb_os_desc;
	names[0] = "ADB";
	config_group_init_type_name(&fi_adb->func_inst.group,
	                            "", &adb_func_type);
	usb_os_desc_prepare_interf_dir(&fi_adb->func_inst.group, 1,
	                               descs, names, THIS_MODULE);

	ret = adb_setup(fi_adb);
	if (ret) {
		kfree(fi_adb);
		pr_err("Error setting ADB\n");
		return ERR_PTR(ret);
	}

	return  &fi_adb->func_inst;
}

static int adb_ctrlreq_configfs(struct usb_function *f,
                                const struct usb_ctrlrequest *ctrl)
{
	return -EOPNOTSUPP;
}

static void adb_free(struct usb_function *f)
{
}

static struct usb_function *adb_alloc(struct usb_function_instance *fi)
{
	struct adb_instance *fi_adb = to_fi_adb(fi);
	struct adb_dev *dev;

	if (fi_adb->dev == NULL) {
		return ERR_PTR(-EINVAL);
	}

	dev = fi_adb->dev;
	dev->function.name = DRIVER_NAME;
	dev->function.strings = adb_strings;
	dev->function.fs_descriptors = fs_adb_descs;
	dev->function.hs_descriptors = hs_adb_descs;
	dev->function.bind = adb_function_bind;
	dev->function.unbind = adb_function_unbind;
	dev->function.set_alt = adb_function_set_alt;
	dev->function.disable = adb_function_disable;
	dev->function.setup = adb_ctrlreq_configfs;
	dev->function.free_func = adb_free;

	return &dev->function;
}

DECLARE_USB_FUNCTION_INIT(adb, adb_alloc_inst, adb_alloc);
MODULE_LICENSE("GPL");