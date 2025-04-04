#ifndef _LIBDMMU_
#define _LIBDMMU_

#include <linux/device.h>


struct dmmu_mm_ops {
	int (*mm_release)(void *data);
};

struct dmmu_mm_notifier {

	struct device *dev;
	void *data;
	struct list_head list;

	struct dmmu_mm_ops *ops;
};

int dmmu_register_mm_notifier(struct dmmu_mm_notifier *dmn);

unsigned long dmmu_map(struct device *dev, unsigned long vaddr, unsigned long len);
int dmmu_flush_cache(unsigned long vaddr, unsigned long len);
int dmmu_unmap(struct device *dev, unsigned long vaddr, int len);
int dmmu_unmap_all(struct device *dev);
void dmmu_dump_vaddr(unsigned long vaddr);
int dmmu_memory_smash(unsigned long vaddr, int smash_mode);
int dmmu_dump_map(unsigned long vaddr);

#endif
