#include <linux/device.h>
#include "ingenic_rproc_riscv.h"
#include "ingenic_rproc_riscv_sysfs.h"

struct pma_kobj_attr {
	struct kobj_attribute attr;
	unsigned int id;
};

#define to_pma_attr(attr)     \
	container_of(attr, struct pma_kobj_attr, attr)

#define to_jz_rproc_device(kobj) dev_get_drvdata(container_of(kobj->parent, struct device, kobj))

#define ADDR_OFFSET 2

unsigned int pma_addr[8] = {0};
unsigned int pma_cfg[8] = {0};
int pma_status[8] = {0};

static int pma_get_size(unsigned int val)
{
	int i = 0;
	for (i = 0; i < 32; i++) {
		if ((val & (1 << i)) == 0) {
			break;
		}
	}
	if (i > 30 || i < 8) { //support size 4K to 4G
		return 0;
	} else {
		return (1 << (i + 3));
	}
}

static unsigned int pma_get_addr(unsigned int val)
{
	int i = 0;
	for (i = 0; i < 32; i++) {
		if ((val & (1 << i)) == 0) {
			break;
		}
	}

	return ((val >> i) << (i + ADDR_OFFSET));
}

static ssize_t pma_start_addr_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int index = to_pma_attr(attr)->id;
	char *p = buf;
	unsigned int addr = 0;

	if (pma_status[index] == 0) {
		p += sprintf(p, "please echo 1 > status firstly\n");
		return p - buf;
	}

	addr = pma_get_addr(pma_addr[index]);

	p += sprintf(p, "0x%08x\n", addr);

	return p - buf;
}

static ssize_t pma_start_addr_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int index = to_pma_attr(attr)->id;
	unsigned int addr = simple_strtol(buf, NULL, 16);
	int size = 0;

	if (pma_status[index] == 0) {
		printk("please echo 1 > status firstly\n");
		return count;
	}

	size = pma_get_size(pma_addr[index]);

	if ((size > 0) && (addr % size)) {
		printk("start_addr illegal, unaligned to size(0x%x)\n", size);
	} else {
		/*clear addr*/
		if (size > 0) {
			pma_addr[index] &= ((size >> 3) - 1);
		} else {
			pma_addr[index] = 0;
		}
		/*set addr*/
		pma_addr[index] |= (addr >> ADDR_OFFSET);
		pma_cfg[index] |= (3 << 3); /*A = NAPOT*/
	}

	return count;
}

static ssize_t pma_size_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int index = to_pma_attr(attr)->id;
	char *p = buf;
	int size = 0;

	if (pma_status[index] == 0) {
		p += sprintf(p, "please echo 1 > status firstly\n");
		return p - buf;
	}

	size = pma_get_size(pma_addr[index]);

	if (size == 0) {
		p += sprintf(p, "pma size illegal!\n");
	} else {
		p += sprintf(p, "0x%x\n", size);
	}

	return p - buf;
}

static ssize_t pma_size_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int index = to_pma_attr(attr)->id;
	unsigned int size = simple_strtol(buf, NULL, 16);
	unsigned int addr = 0;

	if (pma_status[index] == 0) {
		printk("please echo 1 > status firstly\n");
		return count;
	}

	if ((size & (size - 1)) != 0) {
		printk("pma size illegal, it must pow of 2\n");
	} else if (size < 0x1000 || size > 0x100000000) {
		printk("pma size illegal, it must in a range of 4K to 4G\n");
	} else {
		addr = pma_get_addr(pma_addr[index]);
		if (addr % size) {
			printk("size illegal, start_addr(%x) is unaligned to this size\n", addr);
		} else {
			/*clear size*/
			pma_addr[index] &= (addr >> ADDR_OFFSET);
			/*set size*/
			pma_addr[index] |= (size >> 3) - 1;
		}

	}

	return count;
}

static ssize_t pma_readable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int index = to_pma_attr(attr)->id;
	char *p = buf;

	if (pma_status[index] == 0) {
		p += sprintf(p, "please echo 1 > status firstly\n");
		return p - buf;
	}

	p += sprintf(p, "%d\n", (pma_cfg[index] >> PMA_CFG_READ_BIT) & 0x01);

	return p - buf;
}

static ssize_t pma_readable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{

	int index = to_pma_attr(attr)->id;
	unsigned int enable = simple_strtol(buf, NULL, 10);

	if (pma_status[index] == 0) {
		printk("please echo 1 > status firstly\n");
		return count;
	}

	if (enable) {
		pma_cfg[index] |= 1 << PMA_CFG_READ_BIT;
	} else {
		pma_cfg[index] &= ~(1 << PMA_CFG_READ_BIT);
	}

	return count;
}

static ssize_t pma_writeable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int index = to_pma_attr(attr)->id;
	char *p = buf;

	if (pma_status[index] == 0) {
		p += sprintf(p, "please echo 1 > status firstly\n");
		return p - buf;
	}

	p += sprintf(p, "%d\n", (pma_cfg[index] >> PMA_CFG_WRITE_BIT) & 0x01);

	return p - buf;
}

static ssize_t pma_writeable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{

	int index = to_pma_attr(attr)->id;
	unsigned int enable = simple_strtol(buf, NULL, 10);

	if (pma_status[index] == 0) {
		printk("please echo 1 > status firstly\n");
		return count;
	}

	if (enable) {
		pma_cfg[index] |= 1 << PMA_CFG_WRITE_BIT;
	} else {
		pma_cfg[index] &= ~(1 << PMA_CFG_WRITE_BIT);
	}

	return count;
}

static ssize_t pma_executable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int index = to_pma_attr(attr)->id;
	char *p = buf;

	if (pma_status[index] == 0) {
		p += sprintf(p, "please echo 1 > status firstly\n");
		return p - buf;
	}

	p += sprintf(p, "%d\n", (pma_cfg[index] >> PMA_CFG_EXECUT_BIT) & 0x01);

	return p - buf;
}

static ssize_t pma_executable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int index = to_pma_attr(attr)->id;
	unsigned int enable = simple_strtol(buf, NULL, 10);

	if (pma_status[index] == 0) {
		printk("please echo 1 > status firstly\n");
		return count;
	}

	if (enable) {
		pma_cfg[index] |= 1 << PMA_CFG_EXECUT_BIT;
	} else {
		pma_cfg[index] &= ~(1 << PMA_CFG_EXECUT_BIT);
	}

	return count;
}

static ssize_t pma_cacheable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int index = to_pma_attr(attr)->id;
	char *p = buf;

	if (pma_status[index] == 0) {
		p += sprintf(p, "please echo 1 > status firstly\n");
		return p - buf;
	}

	p += sprintf(p, "%d\n", (pma_cfg[index] >> PMA_CFG_CACHE_BIT) & 0x01);

	return p - buf;
}

static ssize_t pma_cacheable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int index = to_pma_attr(attr)->id;
	unsigned int enable = simple_strtol(buf, NULL, 10);

	if (pma_status[index] == 0) {
		printk("please echo 1 > status firstly\n");
		return count;
	}

	if (enable) {
		pma_cfg[index] |= 1 << PMA_CFG_CACHE_BIT;
	} else {
		pma_cfg[index] &= ~(1 << PMA_CFG_CACHE_BIT);
	}

	return count;
}

static ssize_t pma_status_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct ingenic_riscv_rproc *jz_rproc = to_jz_rproc_device(kobj);
	int index = to_pma_attr(attr)->id;
	unsigned int enable = simple_strtol(buf, NULL, 10);

	if (enable) {
		pma_addr[index] = riscv_readl(jz_rproc, CCU_PMA_ADR(index));
		pma_cfg[index] = riscv_readl(jz_rproc, CCU_PMA_CFG(index));
		pma_status[index] = 1;
	} else {
		riscv_writel(jz_rproc, pma_addr[index], CCU_PMA_ADR(index));
		riscv_writel(jz_rproc, pma_cfg[index], CCU_PMA_CFG(index));
		pma_addr[index] = 0;
		pma_cfg[index] = 0;
		pma_status[index] = 0;
	}

	return count;
}

static ssize_t pma_help_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *p = buf;

	p += sprintf(p, "echo 1 > status to start pma setting\n");
	p += sprintf(p, "echo 0 > status to enable pma setting\n");

	return p - buf;
}

#define PMA_ATTR(nr, _name, _mode, _show, _store)       \
	{                                                       \
		.attr   = __ATTR(_name, _mode, _show, _store),  \
		          .id     = nr,                   \
	}

#define PMA_ATTRIBUTE(_name, _mode, _show, _store)                                                    \
	static struct pma_kobj_attr kobj_attr_##_name##pma0 = PMA_ATTR(0, _name, _mode, _show, _store); \
	static struct pma_kobj_attr kobj_attr_##_name##pma1 = PMA_ATTR(1, _name, _mode, _show, _store); \
	static struct pma_kobj_attr kobj_attr_##_name##pma2 = PMA_ATTR(2, _name, _mode, _show, _store); \
	static struct pma_kobj_attr kobj_attr_##_name##pma3 = PMA_ATTR(3, _name, _mode, _show, _store); \
	static struct pma_kobj_attr kobj_attr_##_name##pma4 = PMA_ATTR(4, _name, _mode, _show, _store); \
	static struct pma_kobj_attr kobj_attr_##_name##pma5 = PMA_ATTR(5, _name, _mode, _show, _store); \
	static struct pma_kobj_attr kobj_attr_##_name##pma6 = PMA_ATTR(6, _name, _mode, _show, _store); \
	static struct pma_kobj_attr kobj_attr_##_name##pma7 = PMA_ATTR(7, _name, _mode, _show, _store);

PMA_ATTRIBUTE(start_addr, S_IRUGO | S_IWUSR, pma_start_addr_show, pma_start_addr_store);
PMA_ATTRIBUTE(size, S_IRUGO | S_IWUSR, pma_size_show, pma_size_store);
PMA_ATTRIBUTE(readable, S_IRUGO | S_IWUSR, pma_readable_show, pma_readable_store);
PMA_ATTRIBUTE(writeable, S_IRUGO | S_IWUSR, pma_writeable_show, pma_writeable_store);
PMA_ATTRIBUTE(executable, S_IRUGO | S_IWUSR, pma_executable_show, pma_executable_store);
PMA_ATTRIBUTE(cacheable, S_IRUGO | S_IWUSR, pma_cacheable_show, pma_cacheable_store);
PMA_ATTRIBUTE(status, S_IRUGO | S_IWUSR, NULL, pma_status_store);
PMA_ATTRIBUTE(help, S_IRUGO | S_IWUSR, pma_help_show, NULL);

#define PMA_ATTRIBUTE_GROUP(name)                                     \
	static struct attribute *riscv_##name##_attrs[] = {               \
		&kobj_attr_start_addr##name.attr.attr,                     \
		&kobj_attr_size##name.attr.attr,                      \
		&kobj_attr_readable##name.attr.attr,                  \
		&kobj_attr_writeable##name.attr.attr,                  \
		&kobj_attr_executable##name.attr.attr,                  \
		&kobj_attr_cacheable##name.attr.attr,                  \
		&kobj_attr_status##name.attr.attr,                  \
		&kobj_attr_help##name.attr.attr,                  \
		NULL,                                                   \
	};

PMA_ATTRIBUTE_GROUP(pma0);
PMA_ATTRIBUTE_GROUP(pma1);
PMA_ATTRIBUTE_GROUP(pma2);
PMA_ATTRIBUTE_GROUP(pma3);
PMA_ATTRIBUTE_GROUP(pma4);
PMA_ATTRIBUTE_GROUP(pma5);
PMA_ATTRIBUTE_GROUP(pma6);
PMA_ATTRIBUTE_GROUP(pma7);

static struct attribute_group riscv_pma0_group = {
	.name = "pma0",
	.attrs = riscv_pma0_attrs,
};
static struct attribute_group riscv_pma1_group = {
	.name = "pma1",
	.attrs = riscv_pma1_attrs,
};
static struct attribute_group riscv_pma2_group = {
	.name = "pma2",
	.attrs = riscv_pma2_attrs,
};
static struct attribute_group riscv_pma3_group = {
	.name = "pma3",
	.attrs = riscv_pma3_attrs,
};
static struct attribute_group riscv_pma4_group = {
	.name = "pma4",
	.attrs = riscv_pma4_attrs,
};
static struct attribute_group riscv_pma5_group = {
	.name = "pma5",
	.attrs = riscv_pma5_attrs,
};
static struct attribute_group riscv_pma6_group = {
	.name = "pma6",
	.attrs = riscv_pma6_attrs,
};
static struct attribute_group riscv_pma7_group = {
	.name = "pma7",
	.attrs = riscv_pma7_attrs,
};

static const struct attribute_group *riscv_pmax_groups[] = {
	&riscv_pma0_group,
	&riscv_pma1_group,
	&riscv_pma2_group,
	&riscv_pma3_group,
	&riscv_pma4_group,
	&riscv_pma5_group,
	&riscv_pma6_group,
	&riscv_pma7_group,
	NULL,
};

int ingenic_rproc_riscv_sysfs_init(struct ingenic_riscv_rproc *jz_rproc)
{
	unsigned int ret = 0;
	/*create pma dir*/
	jz_rproc->sysfs_kobj = kobject_create_and_add("pma", &jz_rproc->dev->kobj);
	if (!jz_rproc->sysfs_kobj) {
		dev_err(jz_rproc->dev, "device create kobject failed\n");
		ret = -EINVAL;
		return ret;
	}

	ret = sysfs_create_groups(jz_rproc->sysfs_kobj, riscv_pmax_groups);
	if (ret) {
		dev_err(jz_rproc->dev, "device create sys groups failed\n");
		return ret;
	}

	return 0;
}

int ingenic_rproc_riscv_sysfs_deinit(struct ingenic_riscv_rproc *jz_rproc)
{
	sysfs_remove_groups(jz_rproc->sysfs_kobj, riscv_pmax_groups);
	kobject_del(jz_rproc->sysfs_kobj);

	return 0;
}
