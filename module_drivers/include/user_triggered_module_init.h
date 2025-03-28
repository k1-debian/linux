#ifndef __USER_TRIGGERED_MODULE_INIT_H__
#define __USER_TRIGGERED_MODULE_INIT_H__

#include <linux/platform_device.h>

static int is_insmod = 0;
static inline int module_init_func(void);
static inline int user_triggered_set(const char *arg, const struct kernel_param *kp)
{
	int start_insmod = (int)arg;
	if (!start_insmod) {
		return 0;
	}

	if (is_insmod) {
		return 0;
	}

	int ret;
	ret = module_init_func();
	if (!ret) {
		is_insmod = 1;
	}

	*(int *)kp->arg = is_insmod;

	return 0;
}

static inline int user_triggered_get(char *buffer, const struct kernel_param *kp)
{
	sprintf(buffer, "%d\n", *(int *)kp->arg);
	return strlen(buffer);
}

static struct kernel_param_ops user_triggered_ops = {
	.set = user_triggered_set,
	.get = user_triggered_get,
};

#define user_triggered_module_init_named(initfn, name) \
	static inline int __init module_init_func (void){return initfn();}\
	param_check_int(name, &(is_insmod));               \
	module_param_cb(name, &user_triggered_ops, &(is_insmod), 0644)    \

#define user_triggered_module_init(initfn) \
	user_triggered_module_init_named(initfn, is_insmod) \

#endif