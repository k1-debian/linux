#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/dma-heap.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/sched/signal.h>
#include <asm/page.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>

/**
 * usage:
 * 1. dts:
 *      添加 reserved-memory 例如:
        reserved-memory {
             sharedmem_memory: sharedmem_mem@0x5E00000{
             compatible = "shared-dma-pool";
             //  startAddress  size
              reg = <0x05C00000 0x0400000>;
            };
        };
 *    添加设备节点,例如:
       sharedmem {
           status = "okay";
           compatible = "ingenic,sharedmem";
           ingenic,devname = "sharedmem";
           memory-region = <&sharedmem_memory>;
       };
  * 2. 用户层
  *       frameworks/reserveheap
  */

struct dma_buffer_addr {
	struct device *dev;
	void *vaddr;
	dma_addr_t paddr;
	int size;
};

struct attachmemlist {
	struct list_head list;
	struct dma_buf_attachment *attach;
	struct dma_buf *dbuf;
	struct sg_table *sgt;
	int fd;
};

struct sharedmem {
	struct miscdevice mdev;
	struct device *dev;
	struct dma_heap *sys_heap;
	const char *devname;
};

struct filedata {
	struct list_head phymap_top;
	struct mutex mtx;
	struct sharedmem *smem;
};

struct reservemem_heap_buffer {
	struct dma_heap *heap;
	struct list_head attachments;
	struct mutex lock;
	unsigned long len;
	struct sg_table sg_table;
	int vmap_cnt;
	void *vaddr;
	struct dma_buffer_addr *dma_buffer;
};

struct dma_heap_attachment {
	struct device *dev;
	struct sg_table *table;
	struct list_head list;
	bool mapped;
};

#define LOW_ORDER_GFP (GFP_HIGHUSER | __GFP_ZERO)
#define HIGH_ORDER_GFP  (((GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN \
                           | __GFP_NORETRY) & ~__GFP_RECLAIM) \
                         | __GFP_COMP)
static gfp_t order_flags[] = {HIGH_ORDER_GFP, HIGH_ORDER_GFP, LOW_ORDER_GFP};

static const unsigned int orders[] = {8, 4, 0};
#define NUM_ORDERS ARRAY_SIZE(orders)

static struct sg_table *dup_sg_table(struct sg_table *table)
{
	struct sg_table *new_table;
	int ret, i;
	struct scatterlist *sg, *new_sg;

	new_table = kzalloc(sizeof(*new_table), GFP_KERNEL);
	if (!new_table) {
		return ERR_PTR(-ENOMEM);
	}

	ret = sg_alloc_table(new_table, table->orig_nents, GFP_KERNEL);
	if (ret) {
		kfree(new_table);
		return ERR_PTR(-ENOMEM);
	}

	new_sg = new_table->sgl;
	for_each_sgtable_sg(table, sg, i) {
		sg_set_page(new_sg, sg_page(sg), sg->length, sg->offset);
		new_sg = sg_next(new_sg);
	}

	return new_table;
}

static int reservemem_heap_attach(struct dma_buf *dmabuf,
                                  struct dma_buf_attachment *attachment)
{
	struct reservemem_heap_buffer *buffer = dmabuf->priv;
	struct dma_heap_attachment *a;
	struct sg_table *table;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a) {
		return -ENOMEM;
	}

	table = dup_sg_table(&buffer->sg_table);
	if (IS_ERR(table)) {
		kfree(a);
		return -ENOMEM;
	}

	a->table = table;
	a->dev = attachment->dev;
	INIT_LIST_HEAD(&a->list);
	a->mapped = false;

	attachment->priv = a;

	mutex_lock(&buffer->lock);
	list_add(&a->list, &buffer->attachments);
	mutex_unlock(&buffer->lock);

	return 0;
}

static void reservemem_heap_detach(struct dma_buf *dmabuf,
                                   struct dma_buf_attachment *attachment)
{
	struct reservemem_heap_buffer *buffer = dmabuf->priv;
	struct dma_heap_attachment *a = attachment->priv;

	mutex_lock(&buffer->lock);
	list_del(&a->list);
	mutex_unlock(&buffer->lock);

	sg_free_table(a->table);
	kfree(a->table);
	kfree(a);
}

static struct sg_table *reservemem_heap_map_dma_buf(struct dma_buf_attachment *attachment,
        enum dma_data_direction direction)
{
	struct dma_heap_attachment *a = attachment->priv;
	struct sg_table *table = a->table;
	int ret;

	ret = dma_map_sgtable(attachment->dev, table, direction, 0);
	if (ret) {
		return ERR_PTR(ret);
	}

	a->mapped = true;
	return table;
}

static void reservemem_heap_unmap_dma_buf(struct dma_buf_attachment *attachment,
        struct sg_table *table,
        enum dma_data_direction direction)
{
	struct dma_heap_attachment *a = attachment->priv;

	a->mapped = false;
	dma_unmap_sgtable(attachment->dev, table, direction, 0);
}

static int reservemem_heap_dma_buf_begin_cpu_access(struct dma_buf *dmabuf,
        enum dma_data_direction direction)
{
	struct reservemem_heap_buffer *buffer = dmabuf->priv;
	struct dma_heap_attachment *a;

	mutex_lock(&buffer->lock);

	if (buffer->vmap_cnt) {
		invalidate_kernel_vmap_range(buffer->vaddr, buffer->len);
	}

	list_for_each_entry(a, &buffer->attachments, list) {
		if (!a->mapped) {
			continue;
		}
		dma_sync_sgtable_for_cpu(a->dev, a->table, direction);
	}
	mutex_unlock(&buffer->lock);

	return 0;
}

static int reservemem_heap_dma_buf_end_cpu_access(struct dma_buf *dmabuf,
        enum dma_data_direction direction)
{
	struct reservemem_heap_buffer *buffer = dmabuf->priv;
	struct dma_heap_attachment *a;

	mutex_lock(&buffer->lock);

	if (buffer->vmap_cnt) {
		flush_kernel_vmap_range(buffer->vaddr, buffer->len);
	}

	list_for_each_entry(a, &buffer->attachments, list) {
		if (!a->mapped) {
			continue;
		}
		dma_sync_sgtable_for_device(a->dev, a->table, direction);
	}
	mutex_unlock(&buffer->lock);

	return 0;
}

static int reservemem_heap_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct reservemem_heap_buffer *buffer = dmabuf->priv;
	struct sg_table *table = &buffer->sg_table;
	unsigned long addr = vma->vm_start;
	struct sg_page_iter piter;
	int ret;

	for_each_sgtable_page(table, &piter, vma->vm_pgoff) {
		struct page *page = sg_page_iter_page(&piter);

		ret = remap_pfn_range(vma, addr, page_to_pfn(page), PAGE_SIZE,
		                      vma->vm_page_prot);
		if (ret) {
			return ret;
		}
		addr += PAGE_SIZE;
		if (addr >= vma->vm_end) {
			return 0;
		}
	}
	return 0;
}

static void *reservemem_heap_do_vmap(struct reservemem_heap_buffer *buffer)
{
	struct sg_table *table = &buffer->sg_table;
	int npages = PAGE_ALIGN(buffer->len) / PAGE_SIZE;
	struct page **pages = vmalloc(sizeof(struct page *) * npages);
	struct page **tmp = pages;
	struct sg_page_iter piter;
	void *vaddr;

	if (!pages) {
		return ERR_PTR(-ENOMEM);
	}

	for_each_sgtable_page(table, &piter, 0) {
		WARN_ON(tmp - pages >= npages);
		*tmp++ = sg_page_iter_page(&piter);
	}

	vaddr = vmap(pages, npages, VM_MAP, PAGE_KERNEL);
	vfree(pages);

	if (!vaddr) {
		return ERR_PTR(-ENOMEM);
	}

	return vaddr;
}

static int reservemem_heap_vmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct reservemem_heap_buffer *buffer = dmabuf->priv;
	void *vaddr;
	int ret = 0;

	mutex_lock(&buffer->lock);
	if (buffer->vmap_cnt) {
		buffer->vmap_cnt++;
		iosys_map_set_vaddr(map, buffer->vaddr);
		goto out;
	}

	vaddr = reservemem_heap_do_vmap(buffer);
	if (IS_ERR(vaddr)) {
		ret = PTR_ERR(vaddr);
		goto out;
	}

	buffer->vaddr = vaddr;
	buffer->vmap_cnt++;
	iosys_map_set_vaddr(map, buffer->vaddr);
out:
	mutex_unlock(&buffer->lock);

	return ret;
}

static void reservemem_heap_vunmap(struct dma_buf *dmabuf, struct iosys_map *map)
{
	struct reservemem_heap_buffer *buffer = dmabuf->priv;

	mutex_lock(&buffer->lock);
	if (!--buffer->vmap_cnt) {
		vunmap(buffer->vaddr);
		buffer->vaddr = NULL;
	}
	mutex_unlock(&buffer->lock);
	iosys_map_clear(map);
}

static void reservemem_heap_dma_buf_release(struct dma_buf *dmabuf)
{
	struct reservemem_heap_buffer *buffer = dmabuf->priv;
	struct dma_buffer_addr *dma_buffer = buffer->dma_buffer;
	struct sg_table *table;
	struct scatterlist *sg;
	int i;

	table = &buffer->sg_table;
	for_each_sgtable_sg(table, sg, i) {
		struct page *page = sg_page(sg);

		__free_pages(page, compound_order(page));
	}
	dma_free_coherent(dma_buffer->dev, dma_buffer->size, dma_buffer->vaddr, dma_buffer->paddr);
	sg_free_table(table);
	kfree(buffer);
}

static const struct dma_buf_ops reservemem_heap_buf_ops = {
	.attach = reservemem_heap_attach,
	.detach = reservemem_heap_detach,
	.map_dma_buf = reservemem_heap_map_dma_buf,
	.unmap_dma_buf = reservemem_heap_unmap_dma_buf,
	.begin_cpu_access = reservemem_heap_dma_buf_begin_cpu_access,
	.end_cpu_access = reservemem_heap_dma_buf_end_cpu_access,
	.mmap = reservemem_heap_mmap,
	.vmap = reservemem_heap_vmap,
	.vunmap = reservemem_heap_vunmap,
	.release = reservemem_heap_dma_buf_release,
};
static struct page *alloc_largest_available(unsigned long size,
        unsigned int max_order)
{
	struct page *page;
	int i;

	for (i = 0; i < NUM_ORDERS; i++) {
		if (size < (PAGE_SIZE << orders[i])) {
			continue;
		}
		if (max_order < orders[i]) {
			continue;
		}

		page = alloc_pages(order_flags[i], orders[i]);
		if (!page) {
			continue;
		}
		return page;
	}
	return NULL;
}

static struct dma_buf *reservemem_heap_allocate(struct dma_heap *heap,
        unsigned long len,
        unsigned long fd_flags,
        unsigned long heap_flags)
{
	struct sharedmem *smem = dma_heap_get_drvdata(heap);
	struct reservemem_heap_buffer *buffer;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct dma_buf *dmabuf;
	void *vaddr;
	dma_addr_t paddr;
	struct sg_table *table;
	struct scatterlist *sg;
	struct list_head pages;
	struct page *page, *tmp_page;
	int i = 0, ret = -ENOMEM;
	unsigned int max_order = orders[0];
	unsigned long size_remaining = len;
	struct dma_buffer_addr *dma_buffer;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer) {
		return ERR_PTR(-ENOMEM);
	}

	INIT_LIST_HEAD(&buffer->attachments);
	mutex_init(&buffer->lock);
	buffer->heap = heap;
	buffer->len = len;

	INIT_LIST_HEAD(&pages);
	while (size_remaining > 0) {
		/*
		 * Avoid trying to allocate memory if the process
		 * has been killed by SIGKILL
		 */
		if (fatal_signal_pending(current)) {
			ret = -EINTR;
			goto free_buffer;
		}

		page = alloc_largest_available(size_remaining, max_order);
		if (!page) {
			goto free_buffer;
		}

		list_add_tail(&page->lru, &pages);
		size_remaining -= page_size(page);
		max_order = compound_order(page);
		i++;
	}
	table = &buffer->sg_table;
	if (sg_alloc_table(table, i, GFP_KERNEL)) {
		goto free_buffer;
	}

	sg = table->sgl;
	list_for_each_entry_safe(page, tmp_page, &pages, lru) {
		sg_set_page(sg, page, page_size(page), 0);
		sg = sg_next(sg);
		list_del(&page->lru);
	}

	vaddr = dma_alloc_coherent(smem->dev, len, &paddr, GFP_KERNEL);
	dma_buffer = kzalloc(sizeof(*dma_buffer), GFP_KERNEL);
	if (!dma_buffer) {
		ret = -ENOMEM;
		goto free_dma_buf;
	}
	dma_buffer->dev = smem->dev;
	dma_buffer->paddr = paddr;
	dma_buffer->vaddr = vaddr;
	dma_buffer->size = len;
	buffer->dma_buffer = dma_buffer;
	/* create the dmabuf */
	exp_info.exp_name = dma_heap_get_name(heap);
	exp_info.ops = &reservemem_heap_buf_ops;
	exp_info.size = buffer->len;
	exp_info.flags = fd_flags;
	exp_info.priv = buffer;
	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto free_pages;
	}
	return dmabuf;
free_pages:
	for_each_sgtable_sg(table, sg, i) {
		struct page *p = sg_page(sg);

		__free_pages(p, compound_order(p));
	}
	sg_free_table(table);

free_dma_buf:
	dma_free_coherent(smem->dev, len, vaddr, paddr);

free_buffer:
	list_for_each_entry_safe(page, tmp_page, &pages, lru)
	__free_pages(page, compound_order(page));
	kfree(buffer);



	return ERR_PTR(ret);
}

static const struct dma_heap_ops reservemem_heap_ops = {
	.allocate = reservemem_heap_allocate,
};

static int reservemem_heap_create(struct sharedmem *smem)
{
	struct dma_heap_export_info exp_info;
	int ret = 0;

	exp_info.name = smem->devname;
	exp_info.ops = &reservemem_heap_ops;
	exp_info.priv = smem;

	smem->sys_heap = dma_heap_add(&exp_info);
	if (IS_ERR(smem->sys_heap)) {
		ret = PTR_ERR(smem->sys_heap);
	}

	return ret;
}

static int sharedmem_dmabuf_attach_phy(struct device *dev, struct filedata *file_data, int fd, unsigned long *phyaddr)
{
	struct dma_buf_attachment *attach = NULL;
	struct dma_buf *dbuf  = NULL;
	struct sg_table *sgt  = NULL;
	struct attachmemlist *pos;
	int err = 0;
	mutex_lock(&file_data->mtx);
	list_for_each_entry(pos, &file_data->phymap_top, list) {
		if (pos->fd == fd) {
			attach = pos->attach;
			dbuf = pos->dbuf;
			sgt = pos->sgt;
			break;
		}
	}

	if (sgt) {
		*phyaddr = sg_dma_address(sgt->sgl);
		mutex_unlock(&file_data->mtx);
		return 0;
	}

	dbuf = dma_buf_get(fd);
	if (IS_ERR(dbuf)) {
		err = -EINVAL;
		goto error;
	}
	attach = dma_buf_attach(dbuf, dev);
	if (IS_ERR(attach)) {
		err = -EINVAL;
		goto error;
	}

	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		err = -EINVAL;
		goto error;
	}

	*phyaddr = sg_dma_address(sgt->sgl);

	pos = devm_kzalloc(dev, sizeof(struct attachmemlist), GFP_KERNEL);
	pos->attach = attach;
	pos->dbuf = dbuf;
	pos->fd = fd;
	pos->sgt = sgt;
	list_add_tail(&pos->list, &file_data->phymap_top);
	mutex_unlock(&file_data->mtx);
	return 0;
error:
	if (sgt) {
		dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);
	}
	if (attach) {
		dma_buf_detach(dbuf, attach);
	}
	if (dbuf) {
		dma_buf_put(dbuf);
	}
	mutex_unlock(&file_data->mtx);
	return err;
}

static int sharedmem_dmabuf_deattach_phy(struct device *dev, struct filedata *file_data, int fd)
{
	struct dma_buf_attachment *attach = NULL;
	struct dma_buf *dbuf  = NULL;
	struct sg_table *sgt  = NULL;
	struct attachmemlist *pos;
	mutex_lock(&file_data->mtx);
	list_for_each_entry(pos, &file_data->phymap_top, list) {
		if (pos->fd == fd) {
			attach = pos->attach;
			dbuf = pos->dbuf;
			sgt = pos->sgt;
			list_del(&pos->list);
			devm_kfree(dev, pos);
			break;
		}
	}
	if (sgt) {
		dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);
	}
	if (attach) {
		dma_buf_detach(dbuf, attach);
	}
	if (dbuf) {
		dma_buf_put(dbuf);
	}

	mutex_unlock(&file_data->mtx);
	return 0;
}

static int sharedmem_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct sharedmem *smem = container_of(mdev, struct sharedmem, mdev);
	struct filedata *file_data = devm_kzalloc(smem->dev, sizeof(struct filedata), GFP_KERNEL);
	if (IS_ERR_OR_NULL(file_data)) {
		dev_err(smem->dev, "file data alloc failed,no memory\n");
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&file_data->phymap_top);
	mutex_init(&file_data->mtx);
	file_data->smem = smem;
	file->private_data = file_data;
	// dev_info(smem->dev,"sharedmem open! %s:%d\n",current->comm,current->tgid);
	return 0;
}

static int sharedmem_release(struct inode *inode, struct file *file)
{
	struct filedata *file_data = (struct filedata *)file->private_data;
	struct sharedmem *smem = file_data->smem;
	struct attachmemlist *pos, *n;
	mutex_lock(&file_data->mtx);
	list_for_each_entry_safe(pos, n, &file_data->phymap_top, list) {
		if (pos->sgt) {
			dma_buf_unmap_attachment(pos->attach, pos->sgt, DMA_BIDIRECTIONAL);
		}
		if (pos->attach) {
			dma_buf_detach(pos->dbuf, pos->attach);
		}
		if (pos->dbuf) {
			dma_buf_put(pos->dbuf);
		}
		list_del(&pos->list);
		devm_kfree(smem->dev, pos);
	}

	mutex_unlock(&file_data->mtx);
	devm_kfree(smem->dev, file_data);
	// dev_info(smem->dev,"sharedmem close! %s:%d\n",current->comm,current->tgid);
	return 0;
}

#define SHAREDMEM_ATTACH_PHY_ADDR       _IOW('F', 0x1, unsigned int)
#define SHAREDMEM_DEATTACH_PHY_ADDR     _IOW('F', 0x2, unsigned int)
#define SHAREDMEM_AVAILABLE             _IOW('F', 0x3, unsigned int)
#define SHAREDMEM_TOTAL_SIZE            _IOW('F', 0x4, unsigned int)
int dma_coherent_mem_size(struct device *dev);
int dma_coherent_mem_available(struct device *dev);
static long sharedmem_ioctl(struct file *file, unsigned int cmd, unsigned long args)
{
	struct filedata *file_data = (struct filedata *)file->private_data;
	struct sharedmem *smem = file_data->smem;

	int fd = 0;
	int ret = 0;
	unsigned long phyaddr = 0;
	switch (cmd) {
		case SHAREDMEM_ATTACH_PHY_ADDR:
			if (copy_from_user(&fd, (void *)args, sizeof(fd))) {
				dev_err(smem->dev, "copy_from_user error!!!\n");
				ret = -EFAULT;
				break;
			}
			ret = sharedmem_dmabuf_attach_phy(smem->dev, file_data, fd, &phyaddr);
			if (copy_to_user((void *)args, &phyaddr, sizeof(phyaddr))) {
				return -EFAULT;
			}
			break;
		case SHAREDMEM_DEATTACH_PHY_ADDR:

			if (copy_from_user(&fd, (void *)args, sizeof(fd))) {
				dev_err(smem->dev, "copy_from_user error!!!\n");
				ret = -EFAULT;
				break;
			}
			ret = sharedmem_dmabuf_deattach_phy(smem->dev, file_data, fd);
			break;
		case SHAREDMEM_AVAILABLE:
			*(unsigned long *) args = dma_coherent_mem_available(smem->dev);
			break;
		case SHAREDMEM_TOTAL_SIZE:
			*(unsigned long *) args = dma_coherent_mem_size(smem->dev);
			break;
		default:
			dev_err(smem->dev, "No Support cmd:%x\n", cmd);
	}
	return ret;
}

static struct file_operations sharedmem_fops = {
	.owner = THIS_MODULE,
	.open = sharedmem_open,
	.unlocked_ioctl = sharedmem_ioctl,
	.release = sharedmem_release,
};

static int ingenic_sharedmem_probe(struct platform_device *pdev)
{
	struct sharedmem *smem;
	int ret;
	const char *devname;
	smem = devm_kzalloc(&pdev->dev, sizeof(struct sharedmem), GFP_KERNEL);
	if (IS_ERR_OR_NULL(smem)) {
		pr_err("Failed to alloc mem for sharedmem!\n");
		return -ENOMEM;
	}
	smem->dev = &pdev->dev;
	if (of_property_read_string(pdev->dev.of_node, "ingenic,devname", &devname)) {
		pr_err("unrecognized value for ingenic,devname");
		return -1;
	}
	smem->devname = devname;

	ret = reservemem_heap_create(smem);
	if (ret != 0) {
		pr_err("Failed to create dma-buf heap\n");
		return ret;
	}

	platform_set_drvdata(pdev, smem);

	smem->mdev.minor = MISC_DYNAMIC_MINOR;
	smem->mdev.name = smem->devname;
	smem->mdev.fops = &sharedmem_fops;

	ret = misc_register(&smem->mdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register misc driver!\n");
		return ret;
	}

	ret = of_reserved_mem_device_init(smem->dev);
	if (ret) {
		dev_warn(smem->dev, "failed to init reserved mem check dts\n");
	}

	dev_info(&pdev->dev, "sharedmem driver probe ok!\n");

	return 0;

}

static int ingenic_sharedmem_remove(struct platform_device *pdev)
{
	struct sharedmem *smem = platform_get_drvdata(pdev);
	misc_deregister(&smem->mdev);
	pr_err("the driver isn't support remove\n");
	return 0;
}

static const struct of_device_id ingenic_sharedmem_dt_match[] = {
	{ .compatible = "ingenic,sharedmem", .data = NULL },
	{},
};

MODULE_DEVICE_TABLE(of, ingenic_sharedmem_dt_match);

static struct platform_driver ingenic_sharedmem_driver = {
	.probe  = ingenic_sharedmem_probe,
	.remove = ingenic_sharedmem_remove,
	.driver = {
		.name   = "sharedmem",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(ingenic_sharedmem_dt_match),
	},
};

module_platform_driver(ingenic_sharedmem_driver);

MODULE_LICENSE("GPL v2");
