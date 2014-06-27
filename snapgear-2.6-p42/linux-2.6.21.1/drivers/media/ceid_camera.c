/* Mandatory Driver module header files */
#include <linux/init.h>
#include <linux/module.h>

MODULE_LICENSE("GPL");

static int __init ceid_camera_init(void)
{
	printk(KERN_ALERT "CEID_CAMERA: module loaded\n");
	return 0;
}

static void __exit ceid_camera_exit(void)
{
	printk(KERN_ALERT "CEID_CAMERA: module unloaded\n");
}

module_init(ceid_camera_init);
module_exit(ceid_camera_exit);
