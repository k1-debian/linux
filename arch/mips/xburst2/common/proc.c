#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/page-flags.h>
#include <asm/uaccess.h>
#include <ingenic_proc.h>
static struct proc_dir_entry *proc_jz_root;
struct proc_dir_entry *jz_proc_mkdir(char *s)
{
	struct proc_dir_entry *p;
	if (!proc_jz_root) {
		proc_jz_root = proc_mkdir("jz", 0);
		if (!proc_jz_root) {
			return NULL;
		}
	}
	p = proc_mkdir(s, proc_jz_root);
	return p;
}

EXPORT_SYMBOL(jz_proc_mkdir);

static int jz_proc_show(struct seq_file *filq, void *v)
{
	int ret = 1;
	struct jz_single_file_ops *proc_fops = filq->private;
	filq->private = proc_fops->data;

	if (proc_fops->read) {
		ret = proc_fops->read(filq, v);
	}

	filq->private = proc_fops;
	return ret;
}
static int jz_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, jz_proc_show, pde_data(inode));
}
static ssize_t jz_proc_write(struct file *file, const char __user *buffer, size_t usize, loff_t *off)
{
	size_t ret;
	struct jz_single_file_ops *proc_fops = ((struct seq_file *)file->private_data)->private;
	((struct seq_file *)file->private_data)->private = proc_fops->data;

	ret = proc_fops->write(file, buffer, usize, off);
	((struct seq_file *)file->private_data)->private = proc_fops;

	return ret;
}

struct proc_dir_entry *jz_proc_create_data(
    const char *name, umode_t mode, struct proc_dir_entry *parent,
    struct jz_single_file_ops *proc_fops, void *data)
{
	struct proc_ops *jz_proc_fops;

	jz_proc_fops = kzalloc(sizeof(struct proc_ops), GFP_KERNEL);
	if (!jz_proc_fops) {
		return NULL;
	}

	jz_proc_fops->proc_read = seq_read;
	jz_proc_fops->proc_lseek = seq_lseek;
	jz_proc_fops->proc_release = single_release;
	jz_proc_fops->proc_open = jz_proc_open;
	jz_proc_fops->proc_write = jz_proc_write;
	proc_fops->data = data;

	return proc_create_data(name, mode, parent, jz_proc_fops, proc_fops);
}

struct proc_dir_entry *jz_proc_create(
    const char *name, umode_t mode, struct proc_dir_entry *parent,
    struct jz_single_file_ops *proc_fops)
{
	return jz_proc_create_data(name, mode, parent, proc_fops, NULL);
}

struct proc_dir_entry *get_jz_proc_root(void)
{
	if (!proc_jz_root) {
		proc_jz_root = proc_mkdir("jz", 0);
		if (!proc_jz_root) {
			return NULL;
		}
	}
	return proc_jz_root;
}
