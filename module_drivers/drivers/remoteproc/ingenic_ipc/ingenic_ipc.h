#ifndef __INGENIC_IPC_H__
#define __INGENIC_IPC_H__

#define CPM_SFTINT 0xbc

#ifdef CONFIG_SOC_X2000
	#define TCSM_MAX_SIZE (32 * 1024)
	#define TCSM_IO_BASE 0x13422000
	#define TCSM_MCU_VIRT_BASE 0xF4000000
#endif

//#include <linux/miscdevice.h>
#include <linux/cdev.h>
#include "ring_mem.h"


struct ingenic_ipc_device;

struct ingenic_ipc_ops {
	void (*notify_remote)(struct ingenic_ipc_device *ipc_dev);
	struct ring_mem *(*get_ring_mem_for_host_write)(void *data);
	struct ring_mem *(*get_ring_mem_for_host_read)(void *data);
	void *(*to_host_addr)(unsigned long addr);
	int (*notifier_register)(struct notifier_block *nb);
	int (*notifier_unregister)(struct notifier_block *nb);
};

struct ingenic_ipc_device {

	struct cdev cdev;
	int minor;

	struct mutex m_lock;

	wait_queue_head_t wait;
	int msg_status;

	struct notifier_block nb;

	struct ingenic_ipc_ops *ipc_ops;
	struct ingenic_ipc_rsc_table *rsc_table;
	void *data;
};

struct ingenic_ipc_rsc_table {
	void *data_from_host;
	void *data_to_host;
};


struct ingenic_ipc_device *ingenic_ipc_device_register(void *data, struct ingenic_ipc_ops *ops, char *name);
void ingenic_ipc_device_unregister(struct ingenic_ipc_device *ipc_dev);

#endif /*__INGENIC_IPC_H__*/
