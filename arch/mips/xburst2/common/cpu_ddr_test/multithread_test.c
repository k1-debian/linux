#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/syscore_ops.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include "multithread_test.h"
struct multithread_test {
	struct dentry       *root;
	const char *name;
	int run;
	struct mutex        lock;
	struct mutex        resultlock;
	struct list_head    thread_top;
	unsigned int thread_count;
	struct list_head    func_top;
};
struct func_list {
	struct list_head list;
	const char *test_name;
	TEST_FUNC    testfunc;
	void           *param;
	atomic_t testcount;
};
struct thread_test {
	struct multithread_test *mt;
	struct list_head    list;
	struct task_struct  *task;
	char thread_name[20];
	TEST_FUNC test_func;
	void *param;
	struct mutex        *lock;
	volatile int done;
};
static int mt_test_func(void *data);
static int mt_test_add_threads(struct multithread_test *mt)
{
	struct thread_test *thread;
	struct func_list *l_func;
	struct func_list *min_func = NULL;
	unsigned int testcount;
	int retry = 100000;

	if (list_empty(&mt->func_top)) {
		printk("test func list is empty,please add func for multithread test.\n");
		return -1;
	}

	thread = (struct thread_test *)vmalloc(sizeof(struct thread_test));
	if (IS_ERR_OR_NULL(thread)) {
		printk("%s:thread kmalloc failed!\n", mt->name);
		return -1;
	}
	memset(thread, 0, sizeof(struct thread_test));
	thread->done = -1;

	testcount = 0xffffffff;
	list_for_each_entry(l_func, &mt->func_top, list) {
		unsigned int tc;
		tc = atomic_read(&l_func->testcount);
		if (testcount >= tc) {
			min_func = l_func;
			testcount = tc;
		}
	}
	if (!min_func) {
		printk("Not find func %s %d\n", __FILE__, __LINE__);
		goto err_exit;
	}
	atomic_inc(&min_func->testcount);

	thread->mt = mt;
	thread->test_func = min_func->testfunc;
	thread->param = min_func->param;
	thread->lock = &mt->resultlock;
	snprintf(thread->thread_name, sizeof(thread->thread_name), "%s-%d\n", min_func->test_name, atomic_read(&min_func->testcount));

	do {
		if (retry <= 0) {
			break;
		}
		thread->task = kthread_run(mt_test_func, thread, thread->thread_name);
		msleep(1);

		retry--;

	} while ((int)thread->task == -EAGAIN);

	if (IS_ERR(thread->task) || (retry == 0)) {
		printk("%s: Failed to run thread func:%s, retry: %d\n", mt->name, thread->thread_name, retry);
		goto err_exit;
	}
	list_add_tail(&thread->list, &mt->thread_top);
	return 0;
err_exit:
	kfree(thread);
	return -1;
}

static int mt_test_func(void *data)
{
	struct thread_test *thread = data;
	while (!kthread_should_stop()) {
		if (thread->test_func(thread->param) == 0) {
			mutex_lock(&thread->mt->lock);
			mt_test_add_threads(thread->mt);
			mutex_unlock(&thread->mt->lock);
		}
		break;
	}

	mutex_lock(&thread->mt->lock);
	list_del(&thread->list);
	thread->done = 0;
	mutex_unlock(&thread->mt->lock);

	vfree(thread);
	return 0;
}

static int run_mt_test(struct multithread_test *mt)
{
	int i = 0;
	if (mt->thread_count > 0) {
		for (i = 0; i < mt->thread_count; i++) {
			mt_test_add_threads(mt);
		}
	}
	return 0;
}
static void stop_threaded_test(struct multithread_test *mt)
{
	struct thread_test *thread, *_thread;

	list_for_each_entry_safe(thread, _thread, &mt->thread_top, list) {
		list_del(&thread->list);
		kthread_stop(thread->task);
		while (thread->done);
		vfree(thread);
	}
}

static ssize_t mt_test_read_run(struct file *file, char __user *user_buf,
                                size_t count, loff_t *ppos)
{
	struct multithread_test *mt = file->private_data;
	char *buf;
	mutex_lock(&mt->lock);
	if (!list_empty(&mt->thread_top)) {
		buf = "Runing\n";
	} else {
		buf = "Stop\n";
	}
	mutex_unlock(&mt->lock);
	return simple_read_from_buffer(user_buf, count, ppos, buf, strlen(buf));
}
static ssize_t mt_test_write_run(struct file *file, const char __user *user_buf,
                                 size_t count, loff_t *ppos)
{
	struct multithread_test *mt = file->private_data;
	char buf[16];
	bool bv;
	int ret = 0;
	if (copy_from_user(buf, user_buf, min(count, (sizeof(buf) - 1)))) {
		return -EFAULT;
	}

	if (strtobool(buf, &bv) == 0) {
		mutex_lock(&mt->lock);
		if (bv) {
			run_mt_test(mt);
		} else {
			stop_threaded_test(mt);
		}
		mutex_unlock(&mt->lock);
	}
	return ret ? ret : count;
}
static const struct file_operations mt_test_run_fops = {
	.read   = mt_test_read_run,
	.write  = mt_test_write_run,
	.open   = simple_open,
	.llseek = default_llseek,
};
int multithread_test_add_func(void *handle, const char *name, TEST_FUNC testfunc, void *param)
{
	struct multithread_test *mt = (struct multithread_test *)handle;
	struct func_list *funclist = vmalloc(sizeof(struct multithread_test));
	if (funclist == NULL) {
		printk("add func failed,%s %d\n", __FILE__, __LINE__);
		return -1;
	}
	funclist->test_name = name;
	funclist->testfunc = testfunc;
	funclist->param = param;
	atomic_set(&funclist->testcount, 0);
	list_add_tail(&funclist->list, &mt->func_top);
	return 0;
}
void *multithread_test_init(const char *name, int threadcount)
{
	struct dentry *d;
	struct multithread_test *mt = vmalloc(sizeof(struct multithread_test));
	if (!mt) {
		pr_err("%s:%s %d alloc struct multithread_test failed!\n", name, __FILE__, __LINE__);
		return NULL;
	}
	memset(mt, 0, sizeof(struct multithread_test));
	mutex_init(&mt->lock);
	mutex_init(&mt->resultlock);
	INIT_LIST_HEAD(&mt->thread_top);
	INIT_LIST_HEAD(&mt->func_top);
	mt->thread_count = threadcount;
	mt->name = name;
	d = debugfs_create_dir(name, NULL);
	if (IS_ERR(d)) {
		pr_err("create debugfs for %s failed.\n", name);
		return NULL;
	}
	mt->root = d;

	debugfs_create_u32("thread_count", S_IWUSR | S_IRUGO, mt->root,
	                   (u32 *)&mt->thread_count);

	d = debugfs_create_file("run", S_IWUSR | S_IRUGO, mt->root,
	                        mt, &mt_test_run_fops);
	if (IS_ERR_OR_NULL(d)) {
		pr_err("%s:debugfs create run node failed\n", name);
		goto err_node;
	}
	pr_info("%s:debugfs init ok.\n", name);
	return (void *)mt;
err_node:
	debugfs_remove_recursive(mt->root);
	pr_err("%s:debugfs init failed.\n", name);
	return NULL;
}

void multithread_test_deinit(void *handle)
{
	struct multithread_test *mt = (struct multithread_test *)handle;
	struct func_list *funclist, *_funclist;
	stop_threaded_test(mt);
	debugfs_remove_recursive(mt->root);
	list_for_each_entry_safe(funclist, _funclist, &mt->thread_top, list) {
		list_del(&funclist->list);
		vfree(funclist);
	}
	vfree(mt);
}
