/*  module_drivers/drivers/usb/mis/usbloader.c
 *
 * Ingenic usbloader driver to load firmware to Ingenic chips via usbboot
 *
 * Copyright (c) 2024 Ingenic
 * Author:qipengzhen <aric.pzqi@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/rwsem.h>
#include <linux/uaccess.h>
#include <linux/usb.h>

/*
    使用方法:
    1. A 芯片 USB 工作在host模式，X2600.
    2. B 芯片可以是君正的任意一个型号的芯片，例如:X1600/x2000/x2500/x2600.

    3. A系统启动进入 Linux系统
    4. 手动让B芯片进入usbboot模式。

    5. 此时，A系统可以识别USBBOOT device。

    6. A系统进入 /sys/class/usbmisc/usbloader0/device

    -- data_addr, data_length, send_data, flush_cache, stage1_run。  stage1_run 可以初始化B芯片的DDR.
    -- data_addr, data_length, send_data, flush_cache, stage2_run。  stage2_run 可以是任意的firmware.

*/

#define DRIVER_VERSION "USBLOADER Driver Version 1.00"

#define USBLOADER_MINOR     144

#define IOCTL_GET_HARD_VERSION  1
#define IOCTL_GET_DRV_VERSION   2

#define STAGE_ADDR_MSB(addr) ((addr) >> 16)
#define STAGE_ADDR_LSB(addr) ((addr) & 0xffff)

#define USBBOOT_VENDOR_ID   0xa108
#define USBBOOT_PRODUCT_ID  0xeaef

/* Vendor requests. */
#define EP0_GET_CPU_INFO                0x00
#define EP0_SET_DATA_ADDRESS            0x01
#define EP0_SET_DATA_LENGTH             0x02
#define EP0_FLUSH_CACHES                0x03
#define EP0_PROG_START1                 0x04
#define EP0_PROG_START2                 0x05

static const struct usb_device_id id_table[] = {
	{USB_DEVICE(USBBOOT_VENDOR_ID, USBBOOT_PRODUCT_ID) },
	{ },
};
MODULE_DEVICE_TABLE(usb, id_table);

#define MAX_CPU_INFO_SIZE   32

struct usb_loader {
	struct usb_device   *udev;
	struct usb_interface    *interface;     /* the interface for
                               this device */
	unsigned char       *bulk_in_buffer;    /* the buffer to receive
                               data */
	size_t          bulk_in_size;       /* the size of the
                               receive buffer */
	__u8            bulk_in_endpointAddr;   /* the address of the
                               bulk in endpoint */
	__u8            bulk_out_endpointAddr;  /* the address of the
                               bulk out endpoint */
	struct kref     kref;
	struct semaphore    limit_sem;      /* to stop writes at
                               full throttle from
                               using up all RAM */
	struct usb_anchor   submitted;      /* URBs to wait for
                               before suspend */
	struct rw_semaphore io_rwsem;
	unsigned long       disconnected: 1;

	unsigned char       cpu_info[MAX_CPU_INFO_SIZE];
	unsigned int        data_addr;
	unsigned int        data_length;
};
#define to_usbloader_dev(d) container_of(d, struct usb_loader, kref)

#define USB_LOADER_CONCURRENT_WRITES    5

static struct usb_driver usbloader_driver;

static void usbloader_delete(struct kref *kref)
{
	struct usb_loader *dev = to_usbloader_dev(kref);

	usb_put_dev(dev->udev);
	kfree(dev->bulk_in_buffer);
	kfree(dev);
}

static int usbloader_get_cpu_info(struct usb_loader *dev, char *cpu_info, int len)
{

	int status;
	status = usb_control_msg(dev->udev,
	                         usb_rcvctrlpipe(dev->udev, 0),
	                         EP0_GET_CPU_INFO,
	                         USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
	                         0, 0,
	                         cpu_info, len,
	                         USB_CTRL_GET_TIMEOUT);
	if (status < 0) {
		dev_err(&dev->interface->dev, "Failed to get CPU Info!, status: %d\n", status);
		return status;
	}

	return status;
}

static int usbloader_flush_cache(struct usb_loader *dev)
{
	int status;
	status = usb_control_msg(dev->udev,
	                         usb_sndctrlpipe(dev->udev, 0),
	                         EP0_FLUSH_CACHES,
	                         USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
	                         0, 0,
	                         NULL, 0,
	                         USB_CTRL_SET_TIMEOUT);

	if (status != 0) {
		dev_err(&dev->interface->dev, "Failed to Flush Cache %d\n", status);
		return status;
	}

	return 0;

}

static int usbloader_set_data_address(struct usb_loader *dev, unsigned int address)
{
	int status;
	status = usb_control_msg(dev->udev,
	                         usb_sndctrlpipe(dev->udev, 0),
	                         EP0_SET_DATA_ADDRESS,
	                         USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
	                         STAGE_ADDR_MSB(address), STAGE_ADDR_LSB(address),
	                         NULL, 0,
	                         USB_CTRL_SET_TIMEOUT * 20);

	if (status != 0) {
		dev_err(&dev->interface->dev, "Failed to set Data ADDR %d\n", status);
		return status;
	}

	return 0;
}

static int usbloader_set_data_length(struct usb_loader *dev, unsigned int length)
{
	int status;
	status = usb_control_msg(dev->udev,
	                         usb_sndctrlpipe(dev->udev, 0),
	                         EP0_SET_DATA_LENGTH,
	                         USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
	                         STAGE_ADDR_MSB(length), STAGE_ADDR_LSB(length),
	                         NULL, 0,
	                         USB_CTRL_SET_TIMEOUT * 20);

	if (status != 0) {
		dev_err(&dev->interface->dev, "Failed to set Data ADDR %d\n", status);
		return status;
	}

	return 0;
}

static int usbloader_send_data(struct usb_loader *dev, const char *buf, unsigned int length)
{
	unsigned int write_length = length;
	char *pbuf = (char *)buf;
	int result = 0;
	int ret_len = 0;

	while (write_length) {
		result = usb_bulk_msg(dev->udev,  usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
		                      pbuf, write_length, &ret_len,
		                      USB_CTRL_SET_TIMEOUT * 5);

		if (result < 0) {
			dev_err(&dev->interface->dev, "usb write bulk data error! %d \n", result);
			return result;
		}

		write_length -= ret_len;
		pbuf += ret_len;

	}
	return length;
}

static int usbloader_run(struct usb_loader *dev, int is_stage1)
{
	int status;
	unsigned long address = dev->data_addr;

	status = usb_control_msg(dev->udev,
	                         usb_sndctrlpipe(dev->udev, 0),
	                         is_stage1 ? EP0_PROG_START1 : EP0_PROG_START2,
	                         USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
	                         STAGE_ADDR_MSB(address), STAGE_ADDR_LSB(address),
	                         NULL, 0,
	                         USB_CTRL_SET_TIMEOUT * 5);

	if (status != 0) {
		dev_err(&dev->interface->dev, "run program error %d\n", status);
		return status;
	}

	return 0;
}

static ssize_t cpu_info_show(struct device *dev, struct device_attribute *attr,
                             char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct usb_loader *usb_loader_dev = usb_get_intfdata(intf);
	unsigned char *cpu_info = usb_loader_dev->cpu_info;

	usbloader_get_cpu_info(usb_loader_dev, cpu_info, MAX_CPU_INFO_SIZE);

	return sprintf(buf, "%s\n", cpu_info);
}

static ssize_t flush_cache_store(struct device *dev, struct device_attribute *attr,
                                 const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct usb_loader *usb_loader_dev = usb_get_intfdata(intf);

	usbloader_flush_cache(usb_loader_dev);

	return count;
}

static ssize_t data_addr_show(struct device *dev, struct device_attribute *attr,
                              char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct usb_loader *usb_loader_dev = usb_get_intfdata(intf);

	return sprintf(buf, "%d\n", usb_loader_dev->data_addr);
}

static ssize_t data_addr_store(struct device *dev, struct device_attribute *attr,
                               const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct usb_loader *usb_loader_dev = usb_get_intfdata(intf);
	unsigned long data_addr = 0;

	kstrtoul(buf, 16, &data_addr);

	usbloader_set_data_address(usb_loader_dev, data_addr);
	usb_loader_dev->data_addr =  data_addr;
	return count;
}

static ssize_t data_length_show(struct device *dev, struct device_attribute *attr,
                                char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct usb_loader *usb_loader_dev = usb_get_intfdata(intf);

	return sprintf(buf, "%d\n", usb_loader_dev->data_length);
}

static ssize_t data_length_store(struct device *dev, struct device_attribute *attr,
                                 const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct usb_loader *usb_loader_dev = usb_get_intfdata(intf);
	unsigned long data_length = 0;

	kstrtoul(buf, 10, &data_length);

	usbloader_set_data_length(usb_loader_dev, data_length);

	return count;
}

static ssize_t send_data_store(struct device *dev, struct device_attribute *attr,
                               const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct usb_loader *usb_loader_dev = usb_get_intfdata(intf);
	int ret = 0;

	ret = usbloader_send_data(usb_loader_dev, buf, count);
	if (ret < 0) {
		return ret;
	}

	return count;
}

static ssize_t stage1_run_store(struct device *dev, struct device_attribute *attr,
                                const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct usb_loader *usb_loader_dev = usb_get_intfdata(intf);
	int ret = 0;

	ret = usbloader_run(usb_loader_dev, 1);
	if (ret < 0) {
		return ret;
	}

	return count;
}

static ssize_t stage2_run_store(struct device *dev, struct device_attribute *attr,
                                const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct usb_loader *usb_loader_dev = usb_get_intfdata(intf);
	int ret = 0;

	ret = usbloader_run(usb_loader_dev, 0);
	if (ret < 0) {
		return ret;
	}

	return count;
}

static DEVICE_ATTR_RO(cpu_info);
static DEVICE_ATTR_WO(flush_cache);
static DEVICE_ATTR_RW(data_addr);
static DEVICE_ATTR_RW(data_length);
static DEVICE_ATTR_WO(send_data);
static DEVICE_ATTR_WO(stage1_run);
static DEVICE_ATTR_WO(stage2_run);

static struct attribute *boot_device_attrs[] = {
	&dev_attr_cpu_info.attr,
	&dev_attr_flush_cache.attr,
	&dev_attr_data_addr.attr,
	&dev_attr_data_length.attr,
	&dev_attr_send_data.attr,
	&dev_attr_stage1_run.attr,
	&dev_attr_stage2_run.attr,
	NULL,
};
ATTRIBUTE_GROUPS(boot_device);

static int usbloader_open(struct inode *inode, struct file *file)
{
	struct usb_loader *dev;
	struct usb_interface *interface;
	int subminor, r;

	subminor = iminor(inode);

	interface = usb_find_interface(&usbloader_driver, subminor);
	if (!interface) {
		pr_err("USBLOADER: %s - error, can't find device for minor %d\n",
		       __func__, subminor);
		return -ENODEV;
	}

	dev = usb_get_intfdata(interface);

	/* increment our usage count for the device */
	kref_get(&dev->kref);

	/* grab a power reference */
	r = usb_autopm_get_interface(interface);
	if (r < 0) {
		kref_put(&dev->kref, usbloader_delete);
		return r;
	}

	/* save our object in the file's private structure */
	file->private_data = dev;

	return 0;
}

static int usbloader_release(struct inode *inode, struct file *file)
{
	struct usb_loader *dev;

	dev = file->private_data;
	if (dev == NULL) {
		return -ENODEV;
	}

	/* decrement the count on our device */
	usb_autopm_put_interface(dev->interface);
	kref_put(&dev->kref, usbloader_delete);
	return 0;
}

static long usbloader_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
#if 0
	struct usb_loader *dev;
	u16 bcdDevice;
	char buf[30];

	dev = file->private_data;
	if (dev == NULL) {
		return -ENODEV;
	}

	switch (cmd) {
	case IOCTL_GET_HARD_VERSION:
		break;
	case IOCTL_GET_DRV_VERSION:
		break;
	default:
		return -ENOTTY;
	}
#endif
	return 0;
}

#if 0
static void usbloader_write_bulk_callback(struct urb *urb)
{
	struct usb_loader *dev;
	int status = urb->status;

	dev = urb->context;

	/* sync/async unlink faults aren't errors */
	if (status &&
	    !(status == -ENOENT ||
	      status == -ECONNRESET ||
	      status == -ESHUTDOWN)) {
		dev_dbg(&dev->interface->dev,
		        "nonzero write bulk status received: %d\n", status);
	}

	/* free up our allocated buffer */
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
	                  urb->transfer_buffer, urb->transfer_dma);
	up(&dev->limit_sem);
}
#endif

static ssize_t usbloader_write(struct file *file, const char __user *user_buffer,
                               size_t count, loff_t *ppos)
{
#if 0
	struct usb_loader *dev;
	int retval = 0, r;
	struct urb *urb = NULL;
	char *buf = NULL;

	dev = file->private_data;

	/* verify that we actually have some data to write */
	if (count == 0) {
		goto exit;
	}

	r = down_interruptible(&dev->limit_sem);
	if (r < 0) {
		return -EINTR;
	}

	down_read(&dev->io_rwsem);

	if (dev->disconnected) {
		retval = -ENODEV;
		goto err_up_io;
	}

	/* create a urb, and a buffer for it, and copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto err_up_io;
	}

	buf = usb_alloc_coherent(dev->udev, count, GFP_KERNEL,
	                         &urb->transfer_dma);
	if (!buf) {
		retval = -ENOMEM;
		goto error;
	}

	if (copy_from_user(buf, user_buffer, count)) {
		retval = -EFAULT;
		goto error;
	}

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, dev->udev,
	                  usb_sndbulkpipe(dev->udev,
	                                  dev->bulk_out_endpointAddr),
	                  buf, count, usbloader_write_bulk_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	usb_anchor_urb(urb, &dev->submitted);

	/* send the data out the bulk port */
	retval = usb_submit_urb(urb, GFP_KERNEL);
	if (retval) {
		dev_err(&dev->udev->dev,
		        "%s - failed submitting write urb, error %d\n",
		        __func__, retval);
		goto error_unanchor;
	}

	/* release our reference to this urb,
	   the USB core will eventually free it entirely */
	usb_free_urb(urb);

	up_read(&dev->io_rwsem);
exit:
	return count;
error_unanchor:
	usb_unanchor_urb(urb);
error:
	usb_free_coherent(dev->udev, count, buf, urb->transfer_dma);
	usb_free_urb(urb);
err_up_io:
	up_read(&dev->io_rwsem);
	up(&dev->limit_sem);
	return retval;
#endif
	return count;
}

static const struct file_operations usbloader_fops = {
	.owner =        THIS_MODULE,
	.write =        usbloader_write,
	.open =         usbloader_open,
	.unlocked_ioctl = usbloader_ioctl,
	.release =      usbloader_release,
	.llseek =    noop_llseek,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver usbloader_class = {
	.name =         "usbloader%d",
	.fops =         &usbloader_fops,
	.minor_base =   USBLOADER_MINOR,
};

static int usbloader_probe(struct usb_interface *interface,
                           const struct usb_device_id *id)
{
	struct usb_loader *dev = NULL;
	struct usb_endpoint_descriptor *bulk_in, *bulk_out;
	int i;
	int retval;

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		return -ENOMEM;
	}

	kref_init(&dev->kref);
	sema_init(&dev->limit_sem, USB_LOADER_CONCURRENT_WRITES);
	init_rwsem(&dev->io_rwsem);
	init_usb_anchor(&dev->submitted);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	/* set up the endpoint information */
	/* use only the first bulk-in and bulk-out endpoints */
	retval = usb_find_common_endpoints(interface->cur_altsetting,
	                                   &bulk_in, &bulk_out, NULL, NULL);
	if (retval) {
		dev_err(&interface->dev,
		        "Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}

	dev->bulk_out_endpointAddr = bulk_out->bEndpointAddress;

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(interface, &usbloader_class);
	if (retval) {
		/* something prevented us from registering this driver */
		dev_err(&interface->dev,
		        "Not able to get a minor for this device.\n");
		goto error;
	}

	i = le16_to_cpu(dev->udev->descriptor.bcdDevice);

	dev_info(&interface->dev, "USBLOADER Version %1d%1d.%1d%1d found "
	         "at address %d\n", (i & 0xF000) >> 12, (i & 0xF00) >> 8,
	         (i & 0xF0) >> 4, (i & 0xF), dev->udev->devnum);

	/* let the user know what node this device is now attached to */
	dev_info(&interface->dev, "USB LOADER device now attached to USBLOADER-%d\n",
	         interface->minor);
	return 0;

error:
	kref_put(&dev->kref, usbloader_delete);
	return retval;
}

static void usbloader_draw_down(struct usb_loader *dev)
{
	int time;

	time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
	if (!time) {
		usb_kill_anchored_urbs(&dev->submitted);
	}
}

static int usbloader_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_loader *dev = usb_get_intfdata(intf);

	if (!dev) {
		return 0;
	}
	usbloader_draw_down(dev);
	return 0;
}

static int usbloader_resume(struct usb_interface *intf)
{
	return 0;
}

static void usbloader_disconnect(struct usb_interface *interface)
{
	struct usb_loader *dev = usb_get_intfdata(interface);
	int minor = interface->minor;

	/* give back our minor */
	usb_deregister_dev(interface, &usbloader_class);

	down_write(&dev->io_rwsem);
	dev->disconnected = 1;
	up_write(&dev->io_rwsem);

	usb_kill_anchored_urbs(&dev->submitted);

	/* decrement our usage count */
	kref_put(&dev->kref, usbloader_delete);

	dev_info(&interface->dev, "USB LOADER #%d now disconnected\n", minor);
}

static struct usb_driver usbloader_driver = {
	.name =     "usbloader",
	.probe =    usbloader_probe,
	.disconnect =   usbloader_disconnect,
	.suspend =  usbloader_suspend,
	.resume =   usbloader_resume,
	.id_table = id_table,
	.dev_groups =   boot_device_groups,
	.supports_autosuspend = 1,
};

module_usb_driver(usbloader_driver);

MODULE_AUTHOR("qipengzhen <aric.pzqi@ingenic.com>");
MODULE_DESCRIPTION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
