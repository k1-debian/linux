#include "ipu_dmabuf.h"

#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-resv.h>
#include <linux/slab.h>
#include <linux/anon_inodes.h>

#include <linux/kobject.h>

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Kevin Grandemange");
MODULE_AUTHOR("Sebastien Alaiwan");
MODULE_AUTHOR("Antoine Gruzelle");
MODULE_DESCRIPTION("JZ Common");

struct ipu_dmabuf_priv {
	struct ipu_dma_buffer *buffer;

	/* DMABUF related */
	struct device *dev;
	struct sg_table *sgt_base;
	enum dma_data_direction dma_dir;

};

struct ipu_dmabuf_attachment {
	struct sg_table sgt;
	enum dma_data_direction dma_dir;
};

struct ipu_dma_buffer *ipu_alloc_dma(struct device *dev, size_t size)
{
	struct ipu_dma_buffer *buf =
	    kmalloc(sizeof(struct ipu_dma_buffer),
	            GFP_KERNEL);

	if (!buf) {
		return NULL;
	}

	buf->size = size;
	buf->cpu_handle = dma_alloc_coherent(dev, buf->size,
	                                     &buf->dma_handle,
	                                     GFP_KERNEL | GFP_DMA);

	if (!buf->cpu_handle) {
		kfree(buf);
		return NULL;
	}

	return buf;
}

void ipu_free_dma(struct device *dev, struct ipu_dma_buffer *buf)
{
	if (buf)
		dma_free_coherent(dev, buf->size, buf->cpu_handle,
		                  buf->dma_handle);
	kfree(buf);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
	/* device argument was removed */
	static int ipu_dmabuf_attach(struct dma_buf *dbuf, struct dma_buf_attachment *dbuf_attach)
#else
	static int ipu_dmabuf_attach(struct dma_buf *dbuf, struct device *dev, struct dma_buf_attachment *dbuf_attach)
#endif
{
	struct ipu_dmabuf_priv *dinfo = dbuf->priv;

	struct ipu_dmabuf_attachment *attach;

	struct scatterlist *rd, *wr;
	struct sg_table *sgt;
	int ret, i;

	attach = kzalloc(sizeof(*attach), GFP_KERNEL);
	if (!attach) {
		return -ENOMEM;
	}

	sgt = &attach->sgt;

	ret = sg_alloc_table(sgt, dinfo->sgt_base->orig_nents, GFP_KERNEL);
	if (ret) {
		kfree(attach);
		return -ENOMEM;
	}

	rd = dinfo->sgt_base->sgl;
	wr = sgt->sgl;

	for (i = 0; i < sgt->orig_nents; ++i) {
		sg_set_page(wr, sg_page(rd), rd->length, rd->offset);
		rd = sg_next(rd);
		wr = sg_next(wr);
	}

	attach->dma_dir = DMA_NONE;

	dbuf_attach->priv = attach;

	return 0;
}

static void ipu_dmabuf_detach(struct dma_buf *dbuf,
                              struct dma_buf_attachment *db_attach)
{
	struct ipu_dmabuf_attachment *attach = db_attach->priv;
	struct sg_table *sgt;

	if (!attach) {
		return;
	}

	sgt = &attach->sgt;

	/* release the scatterlist cache */
	if (attach->dma_dir != DMA_NONE)
		dma_unmap_sg(db_attach->dev, sgt->sgl, sgt->orig_nents,
		             attach->dma_dir);

	sg_free_table(sgt);
	kfree(attach);
	db_attach->priv = NULL;
}

static struct sg_table *ipu_dmabuf_map(struct dma_buf_attachment *db_attach,
                                       enum dma_data_direction dma_dir)
{
	struct ipu_dmabuf_attachment *attach = db_attach->priv;
	struct sg_table *sgt;

	sgt = &attach->sgt;

	if (attach->dma_dir == dma_dir) {
		return sgt;
	}

	if (attach->dma_dir != DMA_NONE) {
		dma_unmap_sg(db_attach->dev, sgt->sgl, sgt->orig_nents,
		             attach->dma_dir);
		attach->dma_dir = DMA_NONE;
	}

	sgt->nents = dma_map_sg(db_attach->dev, sgt->sgl, sgt->orig_nents,
	                        dma_dir);

	if (!sgt->nents) {
		pr_err("failed to map scatterlist\n");
		return ERR_PTR(-EIO);
	}

	attach->dma_dir = dma_dir;


	return sgt;
}

static void ipu_dmabuf_unmap(struct dma_buf_attachment *at,
                             struct sg_table *sg, enum dma_data_direction dir)
{
}

// this API copy from v4l2-loopback
static int dma_mmap_noncoherent(struct device *dev, struct vm_area_struct *vma,
                                void *cpu_addr, dma_addr_t dma_addr, size_t size)
{
	int ret = -ENXIO;
	unsigned long user_count = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
	unsigned long count = PAGE_ALIGN(size) >> PAGE_SHIFT;
	unsigned long pfn = page_to_pfn(virt_to_page(cpu_addr));
	unsigned long off = vma->vm_pgoff;

	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;

	if (off < count && user_count <= (count - off)) {

		ret = remap_pfn_range(vma, vma->vm_start,
		                      pfn + off,
		                      user_count << PAGE_SHIFT,
		                      vma->vm_page_prot);
	}

	return ret;
}

static int ipu_dmabuf_mmap(struct dma_buf *buf, struct vm_area_struct *vma)
{
	struct ipu_dmabuf_priv *dinfo = buf->priv;
	unsigned long start = vma->vm_start;
	unsigned long vsize = vma->vm_end - start;
	struct ipu_dma_buffer *buffer = dinfo->buffer;
	int ret;

	if (!dinfo) {
		pr_err("No buffer to map\n");
		return -EINVAL;
	}

	vma->vm_pgoff = 0;

	ret = dma_mmap_noncoherent(dinfo->dev, vma, buffer->cpu_handle,
	                           buffer->dma_handle, vsize);

	if (ret < 0) {
		pr_err("Remapping memory failed, error: %d\n", ret);
		return ret;
	}

	vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP);

	return 0;
}

static void ipu_dmabuf_release(struct dma_buf *buf)
{
	struct ipu_dmabuf_priv *dinfo = buf->priv;
	struct ipu_dma_buffer *buffer = dinfo->buffer;

	if (dinfo->sgt_base) {
		sg_free_table(dinfo->sgt_base);
		kfree(dinfo->sgt_base);
	}

	dma_free_coherent(dinfo->dev, buffer->size, buffer->cpu_handle,
	                  buffer->dma_handle);

	put_device(dinfo->dev);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10, 0)
	kfree_sensitive(buffer);
#else
	kzfree(buffer);
#endif
	kfree(dinfo);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,10, 0)
static void *ipu_dmabuf_kmap(struct dma_buf *dmabuf, unsigned long page_num)
{
	struct ipu_dmabuf_priv *dinfo = dmabuf->priv;
	void *vaddr = dinfo->buffer->cpu_handle;

	return vaddr + page_num * PAGE_SIZE;
}
#endif

int ipu_dmabuf_vmap(struct dma_buf *dbuf, struct iosys_map *map)
{
	struct ipu_dmabuf_priv *dinfo = dbuf->priv;
	iosys_map_set_vaddr(map, dinfo->buffer->cpu_handle);

	return 0;
}

static const struct dma_buf_ops ipu_dmabuf_ops = {
	.attach     = ipu_dmabuf_attach,
	.detach     = ipu_dmabuf_detach,
	.map_dma_buf    = ipu_dmabuf_map,
	.unmap_dma_buf  = ipu_dmabuf_unmap,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
	/* the map_atomic interface was removed after 4.19 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,10, 0)
	.map_atomic = ipu_dmabuf_kmap,
	.map        = ipu_dmabuf_kmap,
#endif
#else
	.kmap_atomic    = ipu_dmabuf_kmap,
	.kmap       = ipu_dmabuf_kmap,
#endif
	.vmap       = ipu_dmabuf_vmap,
	.mmap       = ipu_dmabuf_mmap,
	.release    = ipu_dmabuf_release,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
static void define_export_info(struct dma_buf_export_info *exp_info, int size, void *priv)
{
	exp_info->owner = THIS_MODULE;
	exp_info->exp_name = KBUILD_MODNAME;
	exp_info->ops = &ipu_dmabuf_ops;
	exp_info->flags = O_RDWR;
	exp_info->resv = NULL;
	exp_info->size = size;
	exp_info->priv = priv;
}
#endif

static struct sg_table *ipu_get_base_sgt(struct ipu_dmabuf_priv *dinfo)
{
	int ret;
	struct sg_table *sgt;
	struct ipu_dma_buffer *buf = dinfo->buffer;
	struct device *dev = dinfo->dev;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt) {
		return NULL;
	}

	ret = dma_get_sgtable(dev, sgt, buf->cpu_handle, buf->dma_handle,
	                      buf->size);
	if (ret < 0) {
		kfree(sgt);
		return NULL;
	}

	return sgt;

}

static struct dma_buf *ipu_get_dmabuf(void *dma_info_priv)
{
	struct dma_buf *dbuf;
	struct ipu_dmabuf_priv *dinfo = dma_info_priv;
	struct ipu_dma_buffer *buf = dinfo->buffer;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
	struct dma_buf_export_info exp_info;
#endif

	if (!dinfo->sgt_base) {
		dinfo->sgt_base = ipu_get_base_sgt(dinfo);
	}

	if (WARN_ON(!dinfo->sgt_base)) {
		return NULL;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
	define_export_info(&exp_info, buf->size, (void *)dinfo);
	dbuf = dma_buf_export(&exp_info);
	if (IS_ERR(dbuf)) {
		pr_err("couldn't export dma buf\n");
		return NULL;
	}
#else
	dbuf = dma_buf_export((void *)dinfo, &ipu_dmabuf_ops, buf->size, O_RDWR);
	if (IS_ERR(buf)) {
		pr_err("couldn't export dma buf\n");
		return NULL;
	}
#endif
	return dbuf;
}

static void *ipu_dmabuf_wrap(struct device *dev, unsigned long size,
                             struct ipu_dma_buffer *buffer)
{
	struct ipu_dmabuf_priv *dinfo;
	struct dma_buf *dbuf;

	dinfo = kzalloc(sizeof(*dinfo), GFP_KERNEL);
	if (!dinfo) {
		return ERR_PTR(-ENOMEM);
	}

	dinfo->dev = get_device(dev);
	dinfo->buffer = buffer;
	dinfo->dma_dir = DMA_BIDIRECTIONAL;
	dinfo->sgt_base = ipu_get_base_sgt(dinfo);

	dbuf = ipu_get_dmabuf(dinfo);
	if (IS_ERR_OR_NULL(dbuf)) {
		return ERR_PTR(-EINVAL);
	}

	return dbuf;
}

int ipu_create_dmabuf_fd(struct device *dev, unsigned long size,
                         struct ipu_dma_buffer *buffer)
{
	struct dma_buf *dbuf = ipu_dmabuf_wrap(dev, size, buffer);

	if (IS_ERR(dbuf)) {
		return PTR_ERR(dbuf);
	}
	return dma_buf_fd(dbuf, O_RDWR);
}

int ipu_allocate_dmabuf(struct device *dev, int size, u32 *fd)
{
	struct ipu_dma_buffer *buffer;

	buffer = ipu_alloc_dma(dev, size);
	if (!buffer) {
		dev_err(dev, "Can't alloc DMA buffer\n");
		return -ENOMEM;
	}

	*fd = ipu_create_dmabuf_fd(dev, size, buffer);
	return 0;
}

int ipu_dmabuf_get_address(struct device *dev, u32 fd, u32 *bus_address)
{
	struct dma_buf *dbuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	int err = 0;

	dbuf = dma_buf_get(fd);
	if (IS_ERR(dbuf)) {
		return -EINVAL;
	}
	attach = dma_buf_attach(dbuf, dev);
	if (IS_ERR(attach)) {
		err = -EINVAL;
		goto fail_attach;
	}
	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		err = -EINVAL;
		goto fail_map;
	}

	*bus_address = sg_dma_address(sgt->sgl);

	dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);
fail_map:
	dma_buf_detach(dbuf, attach);
fail_attach:
	dma_buf_put(dbuf);
	return err;
}

int ipu_ioctl_get_dma_fd(struct device *dev, unsigned long arg)
{
	struct ipu_dma_info info;
	int err;

	if (copy_from_user(&info, (struct ipu_dma_info *)arg, sizeof(info))) {
		return -EFAULT;
	}

	err = ipu_allocate_dmabuf(dev, info.size, &info.fd);
	if (err) {
		return err;
	}

	err = ipu_dmabuf_get_address(dev, info.fd, &info.phy_addr);
	if (err) {
		return err;
	}

	if (copy_to_user((void *)arg, &info, sizeof(info))) {
		return -EFAULT;
	}

	return 0;
}

int ipu_ioctl_get_dmabuf_dma_addr(struct device *dev, unsigned long arg)
{
	struct ipu_dma_info info;
	int err;

	if (copy_from_user(&info, (struct ipu_dma_info *)arg, sizeof(info))) {
		return -EFAULT;
	}

	err = ipu_dmabuf_get_address(dev, info.fd, &info.phy_addr);
	if (err) {
		return err;
	}

	if (copy_to_user((void *)arg, &info, sizeof(info))) {
		return -EFAULT;
	}

	return 0;
}

typedef enum {
	DMA_SYNC_FOR_READ,
	DMA_SYNC_FOR_WRITE,
} Ipu_DMA_SyncType_t;

typedef struct {
	int fd;
	Ipu_DMA_SyncType_t sync_type;
} IPU_DmaBuf_SyncInfo_t;

int ipu_ioctl_dmabuf_cache_sync(struct device *dev, unsigned long arg)
{
	struct dma_buf *dbuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	int err = 0;
	struct ipu_dmabuf_priv *dinfo;

	IPU_DmaBuf_SyncInfo_t info;
	if (copy_from_user(&info, (IPU_DmaBuf_SyncInfo_t *)arg, sizeof(info))) {
		return -EFAULT;
	}
	dbuf = dma_buf_get(info.fd);
	if (IS_ERR(dbuf)) {
		return -EINVAL;
	}
	dinfo = dbuf->priv;
	if (!dinfo) {
		return -EINVAL;
	}
	attach = dma_buf_attach(dbuf, dev);
	if (IS_ERR(attach)) {
		err = -EINVAL;
		goto fail_attach;
	}
	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		err = -EINVAL;
		goto fail_map;
	}

	if (info.sync_type == DMA_SYNC_FOR_READ) {
		dma_sync_sg_for_cpu(dev, sgt->sgl, sgt->orig_nents, DMA_BIDIRECTIONAL);
	} else {
		dma_sync_sg_for_device(dev, sgt->sgl, sgt->orig_nents, DMA_BIDIRECTIONAL);
	}

	dma_buf_unmap_attachment(attach, sgt, DMA_BIDIRECTIONAL);
fail_map:
	dma_buf_detach(dbuf, attach);
fail_attach:
	dma_buf_put(dbuf);
	return err;
}
