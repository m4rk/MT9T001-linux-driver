#include <linux/init.h>
#include <linux/module.h>
MODULE_LICENSE("Dual BDS/GPL");

static int __init initialization(void)
{
	printk(KERN_ALERT "CEID_CAMERA: module loaded\n");
	return 0;
}

static void __exit cleanup(void)
{
	printk(KERN_ALERT "CEID_CAMERA: module unloaded\n");
}

module_init(initialization);
module_exit(cleanup);
