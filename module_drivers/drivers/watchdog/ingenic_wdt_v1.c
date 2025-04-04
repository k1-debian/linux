/*
 *  Copyright (C) 2017, bo.liu <bo.liu@ingenic.com>
 *  INGENIC Watchdog driver
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ingenic-tcu_v1.h>
#include <uapi/linux/sched/types.h>
/* #include <soc/xburst/reboot.h> */
#include <ingenic_proc.h>
#include <soc/base.h>
#define DEFAULT_HEARTBEAT 5
#define MAX_HEARTBEAT     2048

enum restart_handler_priority {
	WDT_RESET_PROR = 0,
	RTC_HIBERNATE_RESET_PROR,
};

#define REBOOT_CMD_RECOVERY "recovery"
#define REBOOT_CMD_SOFTBURN "softburn"

void __weak ingenic_recovery_sign(void)
{
	pr_info("ingenic default recovery sign\n");
	return;
}

void __weak ingenic_reboot_sign(void)
{
	pr_info("ingenic default reboot sign\n");
	return;
}

void __weak ingenic_softburn_sign(void)
{
	pr_info("ingenic default softburn sign\n");
	return;
}

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
                 "Watchdog cannot be stopped once started (default="
                 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static unsigned int heartbeat = DEFAULT_HEARTBEAT;
module_param(heartbeat, uint, 0);
MODULE_PARM_DESC(heartbeat,
                 "Watchdog heartbeat period in seconds from 1 to "
                 __MODULE_STRING(MAX_HEARTBEAT) ", default "
                 __MODULE_STRING(DEFAULT_HEARTBEAT));

struct ingenic_wdt_drvdata {
	struct watchdog_device wdt;
	struct platform_device *pdev;
	const struct mfd_cell *cell;

	struct clk *ext_clk;
	struct notifier_block restart_handler;
};

static int ingenic_wdt_ping(struct watchdog_device *wdt_dev)
{
	ingenic_watchdog_set_count(0);
	return 0;
}

static inline int get_clk_div(unsigned int *timeout_value)
{
	int clk_div = TCU_PRESCALE_1;

	while (*timeout_value > 0xffff) {
		if (clk_div == TCU_PRESCALE_1024) {
			/* Requested timeout too high;
			 * use highest possible value. */
			*timeout_value = 0xffff;
			clk_div = -1;
			break;
		}
		*timeout_value >>= 2;
		clk_div += 1;
	}

	return clk_div;
}
static void wdt_config(struct ingenic_wdt_drvdata *drvdata,
                       unsigned int new_timeout)
{
	unsigned int rtc_clk_rate;
	//unsigned int ext_clk_rate;
	unsigned int timeout_value;
	unsigned int clk_src;
	int clk_div = 0;

	rtc_clk_rate = 24000000 / 512;
	clk_src = TCU_CLKSRC_RTC;
	//ext_clk_rate = clk_get_rate(drvdata->ext_clk);
	//timeout_value = ext_clk_rate * new_timeout;
	timeout_value = rtc_clk_rate * new_timeout;
	clk_div = get_clk_div(&timeout_value);
	if (clk_div < 0) {
		printk("Requested timeout too high, use highest possible value\n");
		clk_div = TCU_PRESCALE_1024;
		timeout_value = 0xffff;
	}

	ingenic_watchdog_config((clk_div << 3) | clk_src, timeout_value);
}

static int ingenic_wdt_set_timeout(struct watchdog_device *wdt_dev,
                                   unsigned int new_timeout)
{
	struct ingenic_wdt_drvdata *drvdata = watchdog_get_drvdata(wdt_dev);

	wdt_config(drvdata, new_timeout);
	wdt_dev->timeout = new_timeout;
	return 0;
}

static int ingenic_wdt_start(struct watchdog_device *wdt_dev)
{
	struct ingenic_wdt_drvdata *drvdata = watchdog_get_drvdata(wdt_dev);

	drvdata->cell->enable(drvdata->pdev);
	ingenic_wdt_set_timeout(wdt_dev, wdt_dev->timeout);
	return 0;
}

static int ingenic_wdt_stop(struct watchdog_device *wdt_dev)
{
	struct ingenic_wdt_drvdata *drvdata = watchdog_get_drvdata(wdt_dev);
	drvdata->cell->disable(drvdata->pdev);
	return 0;
}

static const struct watchdog_info ingenic_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "ingenic Watchdog",
};

static const struct watchdog_ops ingenic_wdt_ops = {
	.owner = THIS_MODULE,
	.start = ingenic_wdt_start,
	.stop = ingenic_wdt_stop,
	.ping = ingenic_wdt_ping,
	.set_timeout = ingenic_wdt_set_timeout,
};

static int ingenic_reset_handler(struct notifier_block *this, unsigned long mode,
                                 void *cmd)
{
	struct ingenic_wdt_drvdata *drvdata = container_of(this,
	                                      struct ingenic_wdt_drvdata, restart_handler);
	/* struct watchdog_device *wdd = &drvdata->wdt; */

	if (cmd && !strcmp(cmd, REBOOT_CMD_RECOVERY)) {
		ingenic_recovery_sign();
	} else {
		ingenic_reboot_sign();
	}

	if (cmd && !strcmp(cmd, REBOOT_CMD_SOFTBURN)) {
		ingenic_softburn_sign();
	}

	drvdata->cell->enable(drvdata->pdev);

	ingenic_watchdog_config((3 << 3) | TCU_CLKSRC_RTC, 4);
	while (1) {
		mdelay(500);
		pr_err("wdt reset failed, Never be here\n");
	}
	return NOTIFY_DONE;
}

#ifdef CONFIG_OF
static const struct of_device_id ingenic_wdt_of_matches[] = {
	{ .compatible = "ingenic,watchdog", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ingenic_wdt_of_matches)
#endif

static ssize_t watchdog_cmd_set(struct file *file, const char __user *buffer, size_t count, loff_t *f_pos);
static int watchdog_cmd_open(struct inode *inode, struct file *file);
static const struct proc_ops watchdog_cmd_fops = {
	.proc_read = seq_read,
	.proc_open = watchdog_cmd_open,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
	.proc_write = watchdog_cmd_set,
};

static struct watchdog_device *m_wdt;
static int ingenic_wdt_probe(struct platform_device *pdev)
{
	struct ingenic_wdt_drvdata *drvdata;
	struct watchdog_device *ingenic_wdt;
	int ret;
	struct proc_dir_entry *proc;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_wdt_drvdata),
	                       GFP_KERNEL);
	if (!drvdata) {
		dev_err(&pdev->dev, "Unable to alloacate watchdog device\n");
		return -ENOMEM;
	}

	drvdata->cell = mfd_get_cell(pdev);
	if (!drvdata->cell) {
		dev_err(&pdev->dev, "Failed to get mfd cell\n");
		return -ENOMEM;
	}
	drvdata->pdev = pdev;
	if (heartbeat < 1 || heartbeat > MAX_HEARTBEAT) {
		heartbeat = DEFAULT_HEARTBEAT;
	}

	m_wdt = ingenic_wdt = &drvdata->wdt;
	ingenic_wdt->info = &ingenic_wdt_info;
	ingenic_wdt->ops = &ingenic_wdt_ops;
	ingenic_wdt->timeout = heartbeat;
	ingenic_wdt->min_timeout = 1;
	ingenic_wdt->max_timeout = MAX_HEARTBEAT;
	ingenic_wdt->parent = &pdev->dev;
	watchdog_set_nowayout(ingenic_wdt, nowayout);
	watchdog_set_drvdata(ingenic_wdt, drvdata);

	drvdata->ext_clk = devm_clk_get(&pdev->dev, "ext");
	if (IS_ERR(drvdata->ext_clk)) {
		dev_err(&pdev->dev, "cannot find EXT clock\n");
		ret = PTR_ERR(drvdata->ext_clk);
		goto err_out;
	}

	ret = watchdog_register_device(&drvdata->wdt);
	if (ret < 0) {
		goto err_disable_ext_clk;
	}

	platform_set_drvdata(pdev, drvdata);

	drvdata->restart_handler.notifier_call = ingenic_reset_handler;
	drvdata->restart_handler.priority = WDT_RESET_PROR;
	ret = register_restart_handler(&drvdata->restart_handler);
	if (ret)
		dev_warn(&pdev->dev,
		         "cannot register restart handler (err=%d)\n", ret);

	/* proc info */
	proc = jz_proc_mkdir("watchdog");
	if (!proc) {
		printk("create mdio info failed!\n");
	}
	proc_create_data("reset", S_IRUGO, proc, &watchdog_cmd_fops, NULL);

	return 0;

err_disable_ext_clk:

	clk_put(drvdata->ext_clk);
err_out:
	return ret;
}

static int ingenic_wdt_remove(struct platform_device *pdev)
{
	struct ingenic_wdt_drvdata *drvdata = platform_get_drvdata(pdev);

	ingenic_wdt_stop(&drvdata->wdt);
	watchdog_unregister_device(&drvdata->wdt);
	clk_put(drvdata->ext_clk);
	devm_kfree(&pdev->dev, drvdata);
	return 0;
}

static struct platform_driver ingenic_wdt_driver = {
	.probe = ingenic_wdt_probe,
	.remove = ingenic_wdt_remove,
	.driver = {
		.name = "ingenic,watchdog",
		.of_match_table = of_match_ptr(ingenic_wdt_of_matches),
	},
};

module_platform_driver(ingenic_wdt_driver);

/**********************ingenic utils(deprecated)*****************************************/
struct wdt_utils {
	struct watchdog_device *wdt;
	unsigned msecs;

	unsigned stop;
	struct task_struct *task;
	unsigned short count;
};

static __deprecated int reset_task(void *data)
{
	struct wdt_utils *wdt_utils = data;
	const struct sched_param sc_param = {
		.sched_priority = MAX_RT_PRIO - 1,
	};

	sched_setscheduler(current, SCHED_RR, &sc_param);

	if (test_and_set_bit(WDOG_ACTIVE, &wdt_utils->wdt->status)) {
		pr_warn("watchdog conflict\n");
		wdt_utils->stop = 1;
		return 0;
	}

	ingenic_wdt_set_timeout(wdt_utils->wdt, (wdt_utils->msecs + 1000 - 1) / 1000);
	ingenic_wdt_start(wdt_utils->wdt);

	while (1) {
		if (kthread_should_stop()) {
			clear_bit(WDOG_ACTIVE, &wdt_utils->wdt->status);

			ingenic_wdt_stop(wdt_utils->wdt);

			break;
		}

		ingenic_wdt_ping(wdt_utils->wdt);

		msleep(wdt_utils->wdt->timeout / 3);
	}

	return 0;
}

static __deprecated ssize_t wdt_time_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
	struct wdt_utils *wdt = dev_get_drvdata(dev);
	return sprintf(buf, "%d msecs\n", wdt->msecs);
}
static __deprecated ssize_t wdt_time_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct wdt_utils *wdt = dev_get_drvdata(dev);
	unsigned int msecs;

	if (!wdt->stop) {
		return -EBUSY;
	}

	sscanf(buf, "%d\n", &msecs);

	if (msecs < 1000) {
		msecs = 1000;
	}
	if (msecs > 30000) {
		msecs = 30000;
	}

	wdt->msecs = msecs;
	return count;
}
static __deprecated DEVICE_ATTR_RW(wdt_time);

static __deprecated ssize_t wdt_control_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct wdt_utils *wdt = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", wdt->stop ? ">off<on\n" : "off>on<\n");
}

static __deprecated ssize_t wdt_control_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct wdt_utils *wdt = dev_get_drvdata(dev);

	if (!strncmp(buf, "on", 2) && (wdt->stop == 1)) {
		wdt->task = kthread_run(reset_task, wdt, "reset_task%d", wdt->count++);
		wdt->stop = 0;
	} else if (!strncmp(buf, "off", 3) && (wdt->stop == 0)) {
		kthread_stop(wdt->task);
		wdt->stop = 1;
	}
	return count;
}
static __deprecated DEVICE_ATTR_RW(wdt_control);

static __deprecated char *reset_command[] = {"wdt", "hibernate", "recovery", "burnerboot"};
static __deprecated ssize_t reset_show(struct device *dev, struct device_attribute *attr,
                                       char *buf)
{
	int len = 0, i;

	for (i = 0; i < ARRAY_SIZE(reset_command); i++) {
		len += sprintf(buf + len, "%s\t", reset_command[i]);
	}
	len += sprintf(buf + len, "\n");

	return len;
}

static __deprecated ssize_t reset_store(struct device *dev, struct device_attribute *attr,
                                        const char *buf, size_t count)
{
	int command_size = 0;
	int i;
	char *cmd = NULL;

	if (count == 0) {
		return -EINVAL;
	}

	command_size = ARRAY_SIZE(reset_command);
	for (i = 0; i < command_size; i++) {
		if (!strncmp(buf, reset_command[i], strlen(reset_command[i]))) {
			break;
		}
	}

	switch (i) {
	case 1:
		reboot_mode = REBOOT_HARD;
		break;
	case 2:
		cmd = "recovery";
		break;
	case 3:
		cmd = "softburn";
		break;
	default:
	case 0:
		break;
	}
	kernel_restart(cmd);
	return count;
}
static __deprecated DEVICE_ATTR_RW(reset);

static __deprecated struct attribute *reset_attrs[] = {
	&dev_attr_wdt_time.attr,
	&dev_attr_wdt_control.attr,
	&dev_attr_reset.attr,
	NULL
};

static __deprecated const struct attribute_group attr_group = {
	.attrs  = reset_attrs,
};

static __deprecated int wdt_utils_probe(struct platform_device *pdev)
{
	struct wdt_utils *wdt;
	int ret;

	wdt = devm_kzalloc(&pdev->dev, sizeof(struct wdt_utils), GFP_KERNEL);
	if (!wdt) {
		return -ENOMEM;
	}
	wdt->wdt = m_wdt;
	wdt->count = 0;
	wdt->stop = 1;
	wdt->msecs = heartbeat * 1000;
	dev_set_drvdata(&pdev->dev, wdt);
	ret = sysfs_create_group(&pdev->dev.kobj, &attr_group);
	if (ret) {
		printk("watchdog utils regist failed\n");
	}
	return ret;
}

static __deprecated void wdt_utils_shutdown(struct platform_device *pdev)
{
	struct wdt_utils *wdt_utils = dev_get_drvdata(&pdev->dev);
	ingenic_wdt_stop(wdt_utils->wdt);
}

static __deprecated int wdt_utils_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct wdt_utils *wdt = dev_get_drvdata(&pdev->dev);
	if (wdt->stop) {
		return 0;
	}
	kthread_stop(wdt->task);
	return 0;
}

static __deprecated int wdt_utils_resume(struct platform_device *pdev)
{
	struct wdt_utils *wdt = dev_get_drvdata(&pdev->dev);
	if (wdt->stop) {
		return 0;
	}
	wdt->task = kthread_run(reset_task, wdt, "reset_task%d", wdt->count++);
	return 0;
}

static __deprecated struct platform_device wdt_utils_pdev = {
	.name       = "wdt_utils",
};

static __deprecated struct platform_driver wdt_utils_pdrv = {
	.probe      = wdt_utils_probe,
	.shutdown   = wdt_utils_shutdown,
	.suspend    = wdt_utils_suspend,
	.resume     = wdt_utils_resume,
	.driver     = {
		.name   = "wdt_utils",
		.owner  = THIS_MODULE,
	},
};

static __deprecated int __init init_reset(void)
{
	platform_driver_register(&wdt_utils_pdrv);
	platform_device_register(&wdt_utils_pdev);
	return 0;
}
device_initcall_sync(init_reset);
MODULE_AUTHOR("bo.liu <bo.liu@ingenic.com>");
MODULE_DESCRIPTION("ingenic Watchdog Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-wdt");

/* cmd */
#define WATCHDOG_CMD_BUF_SIZE 100
static uint8_t watchdog_cmd_buf[100];
static int watchdog_cmd_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", watchdog_cmd_buf);
	return 0;
}

static ssize_t watchdog_cmd_set(struct file *file, const char __user *buffer, size_t count, loff_t *f_pos)
{
	int cmd_value = 0;
	unsigned int reg04 = 0, reglc = 0;

	char *buf = kzalloc((count + 1), GFP_KERNEL);
	if (!buf) {
		return -ENOMEM;
	}
	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return EFAULT;
	}
	cmd_value = simple_strtoull(buf, NULL, 0);

	mdelay(1000);
	if (1 == cmd_value) {
		reg04 = inl(TCU_IOBASE + 0x04);
		if (reg04 & 0x1) {
			outl(0 << 0, TCU_IOBASE + 0x4);
		}

		outl(0x1a, TCU_IOBASE + 0xc);

		reglc = inl(TCU_IOBASE + 0x1c);
		if (reglc & 0x10000) {
			outl(1 << 16, TCU_IOBASE + 0x3c);
			outl(1 << 16, TCU_IOBASE + 0x2c);
		}
		outl(0x0000, TCU_IOBASE + 0x0);
		outl(0x0000, TCU_IOBASE + 0x8);

		reg04 = inl(TCU_IOBASE + 0x04);
		if (!(reg04 & 0x1)) {
			outl(1 << 0, TCU_IOBASE + 0x4);
		}
		printk("watchdog reboot system!!!\n");
	}

	kfree(buf);
	return count;
}
static int watchdog_cmd_open(struct inode *inode, struct file *file)
{
	return single_open_size(file, watchdog_cmd_show, PDE_DATA(inode), 8192);
}
