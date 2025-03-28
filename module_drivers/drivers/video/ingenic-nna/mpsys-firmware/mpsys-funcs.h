#ifndef __PRIVATE_FUNCS_H__
#define __PRIVATE_FUNCS_H__
#include <linux/init.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/semaphore.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <ingenic_proc.h>

/* semaphore and mutex interfaces */
int mpsys_down_interruptible(struct semaphore *sem);
void mpsys_up(struct semaphore *sem);
void mpsys_mutex_lock(struct mutex *lock);
void mpsys_mutex_unlock(struct mutex *lock);
int mpsys_mutex_lock_interruptible(struct mutex *lock);

/* wait interfaces */
void mpsys_init_completion(struct completion *x);
void mpsys_complete(struct completion *x);
int mpsys_wait_for_completion_interruptible(struct completion *x);
int mpsys_wait_event_interruptible(wait_queue_head_t *wq, int (* state)(void));
void mpsys_wake_up(wait_queue_head_t *q);

/* mem ops */
void *mpsys_kzalloc(size_t s, gfp_t gfp);
void mpsys_kfree(void *p);
long mpsys_copy_from_user(void *to, const void __user *from, long size);
long mpsys_copy_to_user(void __user *to, const void *from, long size);

/* file ops */
struct file *mpsys_filp_open(const char *, int, umode_t);
int mpsys_filp_close(struct file *filp, fl_owner_t id);
int mpsys_kernel_read(struct file *file, loff_t *offset, char *addr, unsigned long count);

/* string ops */
size_t mpsys_strlen(const char *s);
int mpsys_kstrtoint(const char *s, unsigned int base, int *res);
char *mpsys_strstr(const char *s1, const char *s2);

/* misc driver interfaces */
int mpsys_misc_register(struct miscdevice *misc);
void mpsys_misc_deregister(struct miscdevice *misc);

/* system interfaces */
void mpsys_msleep(unsigned int msecs);

/* proc file interfaces */
struct proc_dir_entry *mpsys_proc_create_data(const char *name, umode_t mode, struct proc_dir_entry *parent, const struct proc_ops *proc_fops, void *data);
ssize_t mpsys_seq_read(struct file *file, char __user *buf, size_t size, loff_t *ppos);
loff_t mpsys_seq_lseek(struct file *file, loff_t offset, int whence);
int mpsys_single_release(struct inode *inode, struct file *file);
int mpsys_single_open_size(struct file *file, int (*show)(struct seq_file *, void *), void *data, size_t size);
struct proc_dir_entry *mpsys_jz_proc_mkdir(char *s);
void mpsys_proc_remove(struct proc_dir_entry *de);
void mpsys_remove_proc_entry(const char *name, struct proc_dir_entry *parent);
void mpsys_seq_printf(struct seq_file *m, const char *f, ...);

#endif
