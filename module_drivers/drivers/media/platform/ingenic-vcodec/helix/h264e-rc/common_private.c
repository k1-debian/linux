#include <linux/mm.h>
#include <linux/slab.h>

//#include <common_private.h>

int common_private_printk(unsigned char *fmt, ...)
{
	va_list args;
	int r = 0;

	va_start(args, fmt);
	r = vprintk(fmt, args);
	va_end(args);

	return r;
}

/* malloc */
void *common_private_malloc(unsigned long size)
{
	return kmalloc(size, GFP_KERNEL);
}

void *common_private_realloc(void *p, unsigned long new_size)
{
	return krealloc(p, new_size, GFP_KERNEL);
}

void common_private_free(void *p)
{
	return kfree(p);
}
