#include "mpsys-funcs.h"

/* semaphore and mutex interfaces */
int mpsys_down_interruptible(struct semaphore *sem)
{
	return down_interruptible(sem);
}

void mpsys_up(struct semaphore *sem)
{
	up(sem);
}

void mpsys_mutex_lock(struct mutex *lock)
{
	mutex_lock(lock);
}

void mpsys_mutex_unlock(struct mutex *lock)
{
	mutex_unlock(lock);
}

int mpsys_mutex_lock_interruptible(struct mutex *lock)
{
	return mutex_lock_interruptible(lock);
}

/* wait interfaces */
void mpsys_init_completion(struct completion *x)
{
	init_completion(x);
}

void mpsys_complete(struct completion *x)
{
	complete(x);
}

int mpsys_wait_for_completion_interruptible(struct completion *x)
{
	return wait_for_completion_interruptible(x);
}

int mpsys_wait_event_interruptible(wait_queue_head_t *wq, int (* state)(void))
{
	return wait_event_interruptible((*wq), state());
}

void mpsys_wake_up(wait_queue_head_t *q)
{
	wake_up(q);
}

/* mem ops */
void *mpsys_kzalloc(size_t s, gfp_t gfp)
{
	return kzalloc(s, gfp);
}

void mpsys_kfree(void *p)
{
	kfree(p);
}

long mpsys_copy_from_user(void *to, const void __user *from, long size)
{
	return copy_from_user(to, from, size);
}

long mpsys_copy_to_user(void __user *to, const void *from, long size)
{
	return copy_to_user(to, from, size);
}

/* file ops */
struct file *mpsys_filp_open(const char *filename, int flags, umode_t mode)
{
	return filp_open(filename, flags, mode);
}

int mpsys_filp_close(struct file *filp, fl_owner_t id)
{
	return filp_close(filp, id);
}

int mpsys_kernel_read(struct file *file, loff_t *offset, char *addr, unsigned long count)
{
	return kernel_read(file, addr, count, offset);
}

/* string ops */
char *mpsys_strstr(const char *s1, const char *s2)
{
	return strstr(s1, s2);
}

size_t mpsys_strlen(const char *s)
{
	return strlen(s);
}

int mpsys_kstrtoint(const char *s, unsigned int base, int *res)
{
	return kstrtoint(s, base, res);
}

/* misc driver interfaces */
int mpsys_misc_register(struct miscdevice *misc)
{
	return misc_register(misc);
}

void mpsys_misc_deregister(struct miscdevice *misc)
{
	misc_deregister(misc);
}

/* system interfaces */
void mpsys_msleep(unsigned int msecs)
{
	msleep(msecs);
}

/* proc file interfaces */
struct proc_dir_entry *mpsys_proc_create_data(const char *name, umode_t mode,
        struct proc_dir_entry *parent,
        const struct proc_ops *proc_fops,
        void *data)
{
	return proc_create_data(name, mode, parent, proc_fops, data);
}

ssize_t mpsys_seq_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	return seq_read(file, buf, size, ppos);
}

loff_t mpsys_seq_lseek(struct file *file, loff_t offset, int whence)
{
	return seq_lseek(file, offset, whence);
}

int mpsys_single_release(struct inode *inode, struct file *file)
{
	return single_release(inode, file);
}

int mpsys_single_open_size(struct file *file, int (*show)(struct seq_file *, void *), void *data, size_t size)
{
	return single_open_size(file, show, data, size);
}

struct proc_dir_entry *mpsys_jz_proc_mkdir(char *s)
{
	return jz_proc_mkdir(s);
}

void mpsys_proc_remove(struct proc_dir_entry *de)
{
	proc_remove(de);
}

void mpsys_remove_proc_entry(const char *name, struct proc_dir_entry *parent)
{
	remove_proc_entry(name, parent);
}

void mpsys_seq_printf(struct seq_file *m, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;
	int r = 0;
	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	seq_printf(m, "%pV", &vaf);
	r = m->count;
	va_end(args);
}
