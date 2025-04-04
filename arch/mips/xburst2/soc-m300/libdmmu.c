/*
 * This driver is used by VPU driver.
 *
 * Copyright (C) 2015 Ingenic Semiconductor Co., Ltd.
 *  http://www.ingenic.com
 * Author:  Yan Zhengting <zhengting.yan@ingenic.com>
 * Modify by:   Sun Jiwei <jiwei.sun@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/mempolicy.h>
#include <linux/mm_types.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/current.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/io.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <ingenic_proc.h>
#include <linux/module.h>
#include <linux/mmu_context.h>
#include <linux/mmu_notifier.h>


#include <libdmmu.h>
#define print_dbg(f, arg...) printk(KERN_INFO "dpu: %s, %d: " f "\n", __func__, __LINE__, ## arg)

#define MAP_COUNT       0x10
#define MAP_CONUT_MASK      0xff0

#define DMMU_PTE_VLD        0x01
#define DMMU_PMD_VLD        0x01

#define KSEG0_LOW_LIMIT     0x80000000
#define KSEG1_HEIGH_LIMIT   0xC0000000

enum mem_break {
	MODE1,  /* 13 24 57 68 911 1012 ... */
	MODE2,  /* 13 24 35 46 57 68 79 810 ... */
	MODE3   /* 12 34 56 78 ... */
};

LIST_HEAD(handle_list);
static unsigned long reserved_pte = 0;
static unsigned long res_pte_paddr;

struct pmd_node {
	unsigned int count;
	unsigned long index;
	unsigned long page;
	struct list_head list;
};

struct map_node {
	struct device *dev;
	unsigned long start;
	unsigned long len;
	struct list_head list;
};

struct dmmu_handle {
	pid_t tgid;
	unsigned long pdg;
	struct mutex lock;
	struct list_head list;
	struct list_head pmd_list;
	struct list_head map_list;
	struct list_head dev_notifier_list;

	struct mm_struct *handle_mm; /* mm struct used to match exit_mmap notify */

	struct mmu_notifier mn;
};


/* for test */
static struct dmmu_handle *find_handle(void);
static struct dmmu_handle *create_handle(void);
static void dmmu_cache_wback(struct dmmu_handle *h);
static int dmmu_mm_dev_release(struct dmmu_handle *h);

int dmmu_flush_cache(unsigned long vaddr, unsigned long len)
{
	struct list_head *pos, *next;
	struct dmmu_handle *h = NULL;
	struct map_node *n = NULL;
	unsigned long *pgd, *pte_phys, *pte;
	unsigned long virpage_phy;
	unsigned long v_end = vaddr + len;
	int found = 0;
	int index;

	h = find_handle();
	if (!h) {
		printk("%s find handle err!\n", __func__);
		return -1;
	}
	if (vaddr & 0xfff) {
		printk("%s addr align page err!\n", __func__);
		return -1;
	}
	list_for_each_safe(pos, next, &h->map_list) {
		n = list_entry(pos, struct map_node, list);
		if ((vaddr >= n->start) &&
		    ((vaddr + len) <= (n->start + n->len))) {
			found = 1;
			break;
		}
	}
	if (!found) {
		printk("%s addr err!\n", __func__);
		return -1;
	}
	pgd = (unsigned long *)h->pdg;
	while (vaddr < v_end) {
		index = vaddr >> 22;
		pte_phys = (unsigned long *)(pgd[index] & ~DMMU_PMD_VLD);
		pte = (unsigned long *)phys_to_virt((unsigned long)pte_phys);
		index = ((vaddr & 0x3ff000) >> 12);
		virpage_phy = pte[index] & ~DMMU_PTE_VLD;
		dma_cache_wback((unsigned long)phys_to_virt(virpage_phy), PAGE_SIZE);
		vaddr += PAGE_SIZE;
	}
	return 0;
}

EXPORT_SYMBOL(dmmu_flush_cache);
static int make_smash_list(struct dmmu_handle *h, unsigned long vaddr, unsigned long (**base)[2], unsigned int *len)
{
	unsigned long *pgd = (unsigned long *)h->pdg;
	unsigned long (*list_smash)[2] = NULL;
	unsigned int list_len = 0;
	unsigned long *pte, *pte_phys;
	unsigned int i, index;

	struct list_head *pos, *next;
	struct map_node *n;

	list_for_each_safe(pos, next, &h->map_list) {
		n = list_entry(pos, struct map_node, list);
		if (vaddr == n->start) {
			list_len = (n->len) >> 12;
			break;
		}
	}

	if (list_len) {
		list_smash = kmalloc(list_len * 2 * 4, GFP_KERNEL);
		if (list_smash) {
			i = 0;
			while (i < list_len) {
				list_smash[i][0] = (unsigned long)(vaddr & 0xfffff000);

				index = vaddr >> 22;
				pte_phys = (unsigned long *)(pgd[index] & ~DMMU_PMD_VLD);


				pte = (unsigned long *)phys_to_virt((unsigned long)pte_phys);

				index = ((vaddr & 0x3ff000) >> 12);
				//      printk(">>>>>>virt = 0x%lx, phys = 0x%lx\n", vaddr,
				//              *((unsigned long int *) (((unsigned long)pte_phys+index*4) | 0xa0000000)));
				list_smash[i][1] = pte[index];

				vaddr += 0x1000;
				i++;
			}
		}
	}

	if (list_smash) {
		*base = list_smash;
		*len = list_len;
		return 0;
	} else {
		return -1;
	}
}
EXPORT_SYMBOL(dmmu_dump_map);

int dmmu_dump_map(unsigned long vaddr)
{
	struct dmmu_handle *h = NULL;
	unsigned long (*list_smash)[2] = NULL;
	unsigned int list_len = 0;
	unsigned long *tmp;

	h = find_handle();
	if (!h) {
		h = create_handle();
	}
	if (!h) {
		return -1;
	}
	/* 检查4k对齐 */
	if (vaddr & 0xfff) {
		return -1;
	}

	mutex_lock(&h->lock);

	if (make_smash_list(h, vaddr, &list_smash, &list_len)) {
		mutex_unlock(&h->lock);
		return -1;
	} else {
		unsigned int i;
		for (i = 0; i < list_len; i++) {
			tmp = (unsigned long *)((list_smash[i][1] & ~1) | 0xa0000000);
			printk("[virt: 0x%08lx ---> phys: 0x%08lx] uncache: 0x%08lx\n",
			       list_smash[i][0], list_smash[i][1], (unsigned long)tmp);

			printk("value[0] = 0x%08lx, value[1] = 0x%08lx value[2] = 0x%08lx value[1013] = 0x%08lx\n",
			       tmp[0], tmp[1], tmp[2], tmp[1013]);
		}
	}

	mutex_unlock(&h->lock);
	kfree((void *)list_smash);

	return 0;
}

/* 13 24 57 68 911 1012 ... */
static int smash_mode1(unsigned long (*base)[2], unsigned int len)
{
	unsigned int i;
	unsigned long tmp;
	if (len >= 4) {
		for (i = 0; i < len; i += 2) {
			tmp = base[i][1];
			base[i][1] = base[i + 1][1];
			base[i + 1][1] = tmp;
		}
	}
	return 0;
}

/* 13 24 35 46 57 68 79 810 ... */
static int smash_mode2(unsigned long (*base)[2], unsigned int len)
{
	/* unsigned int i; */
	/* unsigned long tmp; */
	return 0;
}

/* 12 34 56 78 ... */
static int smash_mode3(unsigned long (*base)[2], unsigned int len)
{
	unsigned int i;
	unsigned long tmp;
	if (len >= 4) {
		for (i = 0; i < len / 2 - 1; i++) {
			tmp = base[i][1];
			base[i][1] = base[len - 1 - i][1];
			base[len - 1 - i][1] = tmp;
		}
		for (i = 0; i < len; i += 2) {
			tmp = base[i][1];
			base[i][1] = base[i + 1][1];
			base[i + 1][1] = tmp;
		}
	}
	return 0;
}

static int update_dmmu_pte(struct dmmu_handle *h, unsigned long vaddr,
                           unsigned long (**base)[2], unsigned int len)
{
	unsigned long *pgd = (unsigned long *)h->pdg;
	unsigned long *pte;
	unsigned int i, index;

	for (i = 0; i < len; i++) {
		index = vaddr >> 22;
		pte = (unsigned long *)(pgd[index] & ~DMMU_PMD_VLD);
		pte = (unsigned long *)phys_to_virt((unsigned long)pte);

		index = ((vaddr & 0x3ff000) >> 12);
		pte[index] = (*base)[i][1];
		vaddr += 0x1000;
	}
	return 0;
}

static int update_process_pte(unsigned long vaddr, unsigned long (**base)[2], unsigned int len)
{
	unsigned int i;
	pgd_t *pgdir;
	pmd_t *pmdir;
	pte_t *pte;
	spin_lock(&current->mm->page_table_lock);
	for (i = 0; i < len; i++) {
		pgdir = pgd_offset(current->mm, vaddr);
		if (pgd_none(*pgdir) || pgd_bad(*pgdir)) {
			return 0;
		}
		pmdir = pmd_offset((pud_t *)pgdir, vaddr);
		if (pmd_none(*pmdir) || pmd_bad(*pmdir)) {
			return 0;
		}
		pte = pte_offset_kernel(pmdir, vaddr);
		if (pte_present(*pte)) {
			pte->pte = (*base)[i][1];
		}
		vaddr += 4096;
	}
	spin_unlock(&current->mm->page_table_lock);
	return 0;
}

int dmmu_memory_smash(unsigned long vaddr, int smash_mode)
{
	struct dmmu_handle *h = NULL;
	unsigned long (*list_smash)[2] = NULL;
	unsigned int list_len = 0;
	struct vm_area_struct *vma;

	h = find_handle();
	if (!h) {
		h = create_handle();
	}
	if (!h) {
		return -1;
	}

	/* 检查4k对齐 */
	if (vaddr & 0xfff) {
		return -1;
	}

	mutex_lock(&h->lock);

	if (make_smash_list(h, vaddr, &list_smash, &list_len)) {
		mutex_unlock(&h->lock);
		return -1;
	} else {
		switch (smash_mode) {
		case MODE1:
			print_dbg("smash_mode = %u\n", smash_mode);
			smash_mode1(list_smash, list_len);
			break;
		case MODE2:
			print_dbg("smash_mode = %u\n", smash_mode);
			smash_mode2(list_smash, list_len);
			break;
		case MODE3:
			print_dbg("smash_mode = %u\n", smash_mode);
			smash_mode3(list_smash, list_len);
			break;
		default:
			break;
		}
	}
	update_dmmu_pte(h, vaddr, &list_smash, list_len);
	update_process_pte(vaddr, &list_smash, list_len);

	dmmu_cache_wback(h);
	vma = find_vma(current->mm, vaddr);
	flush_tlb_range(vma, vaddr, vaddr + (list_len * PAGE_SIZE));

	mutex_unlock(&h->lock);
	kfree(list_smash);

	return 0;
}
EXPORT_SYMBOL(dmmu_memory_smash);

static struct map_node *check_map(struct dmmu_handle *h, unsigned long vaddr, unsigned long len)
{
	struct list_head *pos, *next;
	struct map_node *n;
	list_for_each_safe(pos, next, &h->map_list) {
		n = list_entry(pos, struct map_node, list);
		if (vaddr == n->start && len == n->len) {
			return n;
		}
	}
	return NULL;
}

static void handle_add_map(struct device *dev, struct dmmu_handle *h, unsigned long vaddr, unsigned long len)
{
	struct map_node *n = kmalloc(sizeof(*n), GFP_KERNEL);
	if (n == NULL) {
		printk("malloc map_list node failed !!\n");
		return;
	}
	n->dev = dev;
	n->start = vaddr;
	n->len = len;
	INIT_LIST_HEAD(&n->list);
	list_add(&n->list, &h->map_list);
}

static unsigned int get_pfn(unsigned int vaddr)
{
	pgd_t *pgdir;
	pmd_t *pmdir;
	pte_t *pte;
	pgdir = pgd_offset(current->mm, vaddr);
	if (pgd_none(*pgdir) || pgd_bad(*pgdir)) {
		return 0;
	}
	pmdir = pmd_offset((pud_t *)pgdir, vaddr);
	if (pmd_none(*pmdir) || pmd_bad(*pmdir)) {
		return 0;
	}
	pte = pte_offset_kernel(pmdir, vaddr);
	if (pte_present(*pte)) {

		return pte_pfn(*pte) << PAGE_SHIFT;
	}

	return 0;
}

static unsigned long dmmu_v2pfn(unsigned long vaddr)
{
	if (vaddr < KSEG0_LOW_LIMIT) {
		return get_pfn(vaddr);
	}

	if (vaddr >= KSEG0_LOW_LIMIT && vaddr < KSEG1_HEIGH_LIMIT) {
		return virt_to_phys((void *)vaddr);
	}

	panic("dmmu_v2pfn error!");
	return 0;
}

static unsigned long unmap_node(struct pmd_node *n, unsigned long vaddr, unsigned long end, int check)
{
	unsigned int *pte = (unsigned int *)n->page;
	int index = ((vaddr & 0x3ff000) >> 12);
	int free = !check || (--n->count == 0);
	struct page *page = NULL;

	if (vaddr && end) {
		while (index < 1024 && vaddr < end) {
			if (pte[index] & MAP_CONUT_MASK) {
				pte[index] -= MAP_COUNT;
			} else {
				page = pfn_to_page(pte[index] >> PAGE_SHIFT);

				ClearPageReserved(page);
				pte[index] = reserved_pte;
			}
			index++;
			vaddr += 4096;
		}
	}

	if (free) {
		ClearPageReserved(virt_to_page((void *)n->page));
		free_page(n->page);
		list_del(&n->list);
		kfree(n);
	}

	return vaddr;
}

static unsigned long map_node(struct pmd_node *n, unsigned int vaddr, unsigned int end)
{
	unsigned int *pte = (unsigned int *)n->page;
	int index = ((vaddr & 0x3ff000) >> 12);
	struct page *page = NULL;
	unsigned long pfn = 0;


	while (index < 1024 && vaddr < end) {
		if (pte[index] == reserved_pte) {

			mmap_write_lock(current->mm);

			pfn = dmmu_v2pfn(vaddr) >> PAGE_SHIFT;
			page = pfn_to_page(pfn);

			SetPageReserved(page);

			pte[index] = (pfn << PAGE_SHIFT) | DMMU_PTE_VLD;

			mmap_write_unlock(current->mm);
		} else {
			pte[index] += MAP_COUNT;
		}
		index++;
		vaddr += 4096;
	}
	n->count++;
	return vaddr;
}

static struct pmd_node *find_node(struct dmmu_handle *h, unsigned int vaddr)
{
	struct list_head *pos, *next;
	struct pmd_node *n;

	list_for_each_safe(pos, next, &h->pmd_list) {
		n = list_entry(pos, struct pmd_node, list);
		if (n->index == (vaddr & 0xffc00000)) {
			return n;
		}
	}
	return NULL;
}

static struct pmd_node *add_node(struct dmmu_handle *h, unsigned int vaddr)
{
	int i;
	unsigned long *pte;
	unsigned long *pgd = (unsigned long *)h->pdg;
	struct pmd_node *n = kmalloc(sizeof(*n), GFP_KERNEL);
	INIT_LIST_HEAD(&n->list);
	n->count = 0;
	n->index = vaddr & 0xffc00000;
	n->page = __get_free_page(GFP_KERNEL);
	SetPageReserved(virt_to_page((void *)n->page));

	pte = (unsigned long *)n->page;
	for (i = 0; i < 1024; i++) {
		pte[i] = reserved_pte;
	}

	list_add(&n->list, &h->pmd_list);

	pgd[vaddr >> 22] = dmmu_v2pfn(n->page) | DMMU_PMD_VLD;
	return n;
}

static struct dmmu_handle *find_handle(void)
{
	struct list_head *pos, *next;
	struct dmmu_handle *h;


	list_for_each_safe(pos, next, &handle_list) {
		h = list_entry(pos, struct dmmu_handle, list);
		if (h->tgid == current->tgid) {
			return h;
		}
	}
	return NULL;
}

/**
* @brief unmap node from map list, clear each pmd_node in pmd_list,
*   Not free handle, let dmmu_unmap_all do it.
*
* @param h
*
* @return
*/
static int dmmu_unmap_node_unlock(struct dmmu_handle *h)
{
	struct map_node *cn;
	struct list_head *pos, *next;

	unsigned long vaddr;
	int len;
	unsigned long end;

	struct pmd_node *node;

	list_for_each_safe(pos, next, &h->map_list) {
		/* find and delete a map node from map_list */
		cn = list_entry(pos, struct map_node, list);
		vaddr = cn->start;
		len = cn->len;
		end = vaddr + len;

		while (vaddr < end) {
			node = find_node(h, vaddr);
			if (node) {
				vaddr = unmap_node(node, vaddr, end, 1);
			}
		}

		list_del(&cn->list);
		kfree(cn);
	}


	return 0;

}


void dmmu_mm_release(struct mmu_notifier *mn,
                     struct mm_struct *mm)
{
	struct dmmu_handle *h = container_of(mn, struct dmmu_handle, mn);

	printk("===== dmmu release mm (TODO: not checked) ====h: %p\n", h);

	if (h->handle_mm != mm) {
		return;
	}

	mutex_lock(&h->lock);

	dmmu_mm_dev_release(h);

	dmmu_unmap_node_unlock(h);

	mutex_unlock(&h->lock);
}

static const struct mmu_notifier_ops dmmu_mm_notifier_ops = {
	.release = dmmu_mm_release,
};


static struct dmmu_handle *create_handle(void)
{
	struct dmmu_handle *h;
	unsigned int pgd_index;
	unsigned long *pgd;

	h = kmalloc(sizeof(struct dmmu_handle), GFP_KERNEL);
	if (!h) {
		return NULL;
	}

	h->tgid = current->tgid;
	h->pdg = __get_free_page(GFP_KERNEL);
	if (!h->pdg) {
		pr_err("%s %d, Get free page for PGD error\n",
		       __func__, __LINE__);
		kfree(h);
		return NULL;
	}
	SetPageReserved(virt_to_page((void *)h->pdg));

	pgd = (unsigned long *)h->pdg;

	for (pgd_index = 0; pgd_index < PTRS_PER_PGD; pgd_index++) {
		pgd[pgd_index] = res_pte_paddr;
	}

	INIT_LIST_HEAD(&h->list);
	INIT_LIST_HEAD(&h->pmd_list);
	INIT_LIST_HEAD(&h->map_list);
	INIT_LIST_HEAD(&h->dev_notifier_list);
	mutex_init(&h->lock);

	/* register exit_mmap notify */
	h->handle_mm = current->mm;

	h->mn.ops = &dmmu_mm_notifier_ops;

	mmu_notifier_register(&h->mn, h->handle_mm);

	list_add(&h->list, &handle_list);

	return h;
}

static int dmmu_make_present(unsigned long addr, unsigned long end)
{
#if 0
	unsigned long i;
	for (i = addr; i < end; i += 4096) {
		*(volatile unsigned char *)(i) = 0;
	}
	*(volatile unsigned char *)(end - 1) = 0;
	return 0;
#else
	int ret, len, write;
	struct vm_area_struct *vma;
	unsigned long vm_page_prot;

	mmap_write_lock(current->mm);
	vma = find_vma(current->mm, addr);
	if (!vma) {
		printk("dmmu_make_present error. addr=%lx len=%lx\n", addr, end - addr);
		mmap_write_unlock(current->mm);
		return -1;
	}

	if (vma->vm_flags & VM_PFNMAP) {
		mmap_write_unlock(current->mm);
		return 0;
	}
	write = (vma->vm_flags & VM_WRITE) != 0;
	BUG_ON(addr >= end);
	BUG_ON(end > vma->vm_end);

	vm_page_prot = pgprot_val(vma->vm_page_prot);
	vma->vm_page_prot = __pgprot(vm_page_prot | _PAGE_VALID | _PAGE_ACCESSED | _PAGE_PRESENT);

	len = DIV_ROUND_UP(end, PAGE_SIZE) - addr / PAGE_SIZE;
	ret = get_user_pages(addr, len, write, NULL, NULL);
	vma->vm_page_prot = __pgprot(vm_page_prot);
	if (ret < 0) {
		printk("dmmu_make_present get_user_pages error(%d). addr=%lx len=%lx\n", 0 - ret, addr, end - addr);
		mmap_write_unlock(current->mm);
		return ret;
	}

	mmap_write_unlock(current->mm);
	return ret == len ? 0 : -1;
#endif
}

static void dmmu_cache_wback(struct dmmu_handle *h)
{
	struct list_head *pos, *next;
	struct pmd_node *n;

	dma_cache_wback(h->pdg, PAGE_SIZE);

	list_for_each_safe(pos, next, &h->pmd_list) {
		n = list_entry(pos, struct pmd_node, list);
		dma_cache_wback(n->page, PAGE_SIZE);
	}
}

static void dmmu_dump_handle(struct seq_file *m, void *v, struct dmmu_handle *h);

unsigned long dmmu_map(struct device *dev, unsigned long vaddr, unsigned long len)
{
	int end = vaddr + len;
	struct dmmu_handle *h;
	struct pmd_node *node;

	h = find_handle();
	if (!h) {
		h = create_handle();
	}
	if (!h) {
		return 0;
	}

	mutex_lock(&h->lock);
#ifdef DEBUG
	printk("(pid %d)dmmu_map %lx %lx================================================\n", h->tgid, vaddr, len);
#endif
	if (check_map(h, vaddr, len)) {
		mutex_unlock(&h->lock);
		return dmmu_v2pfn(h->pdg);
	}

	if (dmmu_make_present(vaddr, vaddr + len)) {
		mutex_unlock(&h->lock);
		return 0;
	}

	handle_add_map(dev, h, vaddr, len);

	while (vaddr < end) {
		node = find_node(h, vaddr);
		if (!node) {
			node = add_node(h, vaddr);
		}
		vaddr = map_node(node, vaddr, end);
	}
	dmmu_cache_wback(h);
#ifdef DEBUG
	dmmu_dump_handle(NULL, NULL, h);
#endif
	mutex_unlock(&h->lock);

	return dmmu_v2pfn(h->pdg);
}
EXPORT_SYMBOL(dmmu_map);

static int dmmu_mm_dev_release(struct dmmu_handle *h)
{
	struct dmmu_mm_notifier *n = NULL;
	struct list_head *pos, *next;

	list_for_each_safe(pos, next, &h->dev_notifier_list) {
		n = list_entry(pos, struct dmmu_mm_notifier, list);
		if (n && n->ops && n->ops->mm_release) {
			n->ops->mm_release(n->data);
		}

		list_del(&n->list);
	}


	return 0;
}


int dmmu_register_mm_notifier(struct dmmu_mm_notifier *dmn)
{
	int dev_already_registered = 0;
	struct dmmu_mm_notifier *n = NULL;
	struct dmmu_handle *h;
	struct list_head *pos, *next;

	h = find_handle();
	if (!h) {
		return -EINVAL;
	}

	mutex_lock(&h->lock);

	list_for_each_safe(pos, next, &h->dev_notifier_list) {
		n = list_entry(pos, struct dmmu_mm_notifier, list);

		if (n == dmn) {
			/* already in dev_notifier_list*/
			dev_already_registered = 1;
		}

	}

	if (dev_already_registered) {
		goto out;
	}

	list_add_tail(&dmn->list, &h->dev_notifier_list);
out:
	mutex_unlock(&h->lock);
	return 0;
}
EXPORT_SYMBOL(dmmu_register_mm_notifier);

int dmmu_unmap(struct device *dev, unsigned long vaddr, int len)
{
	unsigned long end = vaddr + len;
	struct dmmu_handle *h;
	struct pmd_node *node;
	struct map_node *n;

	h = find_handle();
	if (!h) {
		return 0;
	}

	mutex_lock(&h->lock);
#ifdef DEBUG
	printk("dmmu_unmap %lx %x**********************************************\n", vaddr, len);
#endif
	n = check_map(h, vaddr, len);
	if (!n) {
		mutex_unlock(&h->lock);
		return -EAGAIN;
	}
	if (n->dev != dev) {
		mutex_unlock(&h->lock);
		return -EAGAIN;
	}

	list_del(&n->list);
	kfree(n);

	while (vaddr < end) {
		node = find_node(h, vaddr);
		if (node) {
			vaddr = unmap_node(node, vaddr, end, 1);
		}
	}

	if (list_empty(&h->pmd_list) && list_empty(&h->map_list)) {
		list_del(&h->list);
		ClearPageReserved(virt_to_page((void *)h->pdg));
		free_page(h->pdg);
		mutex_unlock(&h->lock);

		mmu_notifier_unregister(&h->mn, h->handle_mm);

		kfree(h);
		return 0;
	}

	mutex_unlock(&h->lock);
	return 0;
}
EXPORT_SYMBOL(dmmu_unmap);

int dmmu_free_all(struct device *dev)
{
	struct dmmu_handle *h;
	struct pmd_node *node;
	struct map_node *cn;
	struct list_head *pos, *next;

	h = find_handle();
	if (!h) {
		return 0;
	}

	mutex_lock(&h->lock);

	list_for_each_safe(pos, next, &h->map_list) {
		cn = list_entry(pos, struct map_node, list);
		node = find_node(h, cn->start);
		if (node) {
			unmap_node(node, cn->start, cn->len, 1);
		}
		list_del(&cn->list);
		kfree(cn);
	}

	list_for_each_safe(pos, next, &h->pmd_list) {
		node = list_entry(pos, struct pmd_node, list);
		if (node) {
			printk("WARN: pmd list should NULL\n");
			unmap_node(node, 0, 0, 0);
		}
	}

	list_del(&h->list);
	ClearPageReserved(virt_to_page((void *)h->pdg));
	free_page(h->pdg);

	mutex_unlock(&h->lock);
	mmu_notifier_unregister(&h->mn, h->handle_mm);
	kfree(h);
	return 0;
}

/**
* @brief release all resources, which should be called by driver close
*
* @param dev
*
* @return
*/
int dmmu_unmap_all(struct device *dev)
{
	struct dmmu_handle *h;
	struct map_node *cn;
	struct list_head *pos, *next;

#ifdef DEBUG
	printk("dmmu_unmap_all\n");
#endif

	h = find_handle();
	if (!h) {
		printk("dmmu unmap all not find hindle, maybe it has benn freed already, name: %s\n", dev->kobj.name);
		return 0;
	}

	list_for_each_safe(pos, next, &h->map_list) {
		cn = list_entry(pos, struct map_node, list);
		if (dev == cn->dev) {
			dmmu_unmap(dev, cn->start, cn->len);
		}
	}


	if (list_empty(&h->map_list)) {
		dmmu_free_all(dev);
	}
	return 0;
}
EXPORT_SYMBOL(dmmu_unmap_all);
int __init dmmu_init(void)
{
	unsigned int pte_index;
	unsigned long res_page_paddr;
	unsigned long reserved_page;
	unsigned int *res_pte_vaddr;

	reserved_page = __get_free_page(GFP_KERNEL);
	if (!reserved_page) {
		pr_err("%s %d, Get reserved page error\n",
		       __func__, __LINE__);
		return ENOMEM;
	}
	SetPageReserved(virt_to_page((void *)reserved_page));
	reserved_pte = dmmu_v2pfn(reserved_page) | DMMU_PTE_VLD;

	res_page_paddr = virt_to_phys((void *)reserved_page) | 0xFFF;

	res_pte_vaddr = (unsigned int *)__get_free_page(GFP_KERNEL);
	if (!res_pte_vaddr) {
		pr_err("%s %d, Get free page for PTE error\n",
		       __func__, __LINE__);
		free_page(reserved_page);
		return ENOMEM;
	}
	SetPageReserved(virt_to_page(res_pte_vaddr));
	res_pte_paddr = virt_to_phys((void *)res_pte_vaddr) | DMMU_PTE_VLD;

	for (pte_index = 0; pte_index < PTRS_PER_PTE; pte_index++) {
		res_pte_vaddr[pte_index] = res_page_paddr;
	}



	return 0;
}
arch_initcall(dmmu_init);

void dmmu_dump_vaddr(unsigned long vaddr)
{
	struct dmmu_handle *h;
	struct pmd_node *node;

	unsigned long *pmd, *pte;
	h = find_handle();
	if (!h) {
		printk("dmmu_dump_vaddr %08lx error - h not found!\n", vaddr);
		return;
	}

	node = find_node(h, vaddr);
	if (!node) {
		printk("dmmu_dump_vaddr %08lx error - node not found!\n", vaddr);
		return;
	}

	pmd = (unsigned long *)h->pdg;
	pte = (unsigned long *)node->page;

	printk("pmd base = %p; pte base = %p\n", pmd, pte);
	printk("pmd = %08lx; pte = %08lx\n", pmd[vaddr >> 22], pte[(vaddr & 0x3ff000) >> 12]);
}

static void dmmu_dump_handle(struct seq_file *m, void *v, struct dmmu_handle *h)
{
	struct list_head *pos, *next;
	struct map_node *n;

	list_for_each_safe(pos, next, &h->map_list) {
		n = list_entry(pos, struct map_node, list);
		{
			int i = 0;
			int vaddr = n->start;
			struct pmd_node *pn = find_node(h, vaddr);
			unsigned int *pte = (unsigned int *)pn->page;

			while (vaddr < (n->start + n->len)) {
				if (i++ % 8 == 0) {
					printk("\nvaddr %08x : ", vaddr & 0xfffff000);
				}
				printk("%08x ", pte[(vaddr & 0x3ff000) >> 12]);
				vaddr += 4096;
			}
			printk("\n\n");
		}
	}
}

static int dmmu_proc_show(struct seq_file *m, void *v)
{
	struct list_head *pos, *next;
	struct dmmu_handle *h;
	volatile unsigned long flags;
	local_irq_save(flags);
	list_for_each_safe(pos, next, &handle_list) {
		h = list_entry(pos, struct dmmu_handle, list);
		dmmu_dump_handle(m, v, h);
	}
	local_irq_restore(flags);
	return 0;
}

static int dmmu_open(struct inode *inode, struct file *file)
{
	return single_open(file, dmmu_proc_show, PDE_DATA(inode));
}

static const struct proc_ops dmmus_proc_fops = {
	.proc_read = seq_read,
	.proc_open = dmmu_open,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static int __init init_dmmu_proc(void)
{
	struct proc_dir_entry *p;
	p = jz_proc_mkdir("dmmu");
	if (!p) {
		printk("create_proc_entry for common dmmu failed.\n");
		return -ENODEV;
	}
	proc_create("dmmus", 0600, p, &dmmus_proc_fops);

	return 0;
}

module_init(init_dmmu_proc);
