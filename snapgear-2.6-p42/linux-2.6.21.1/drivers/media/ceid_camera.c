#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h> // various structures related to file_operations 
#include <linux/device.h> //  struct of type "device"
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h> // several kernel macros and functions

MODULE_LICENSE("GPL");

// file operations fuctions prototypes
static int dev_open(struct inode *, struct file *);
static int dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

static dev_t dev_num; // device number
static struct cdev c_dev; //char device structure
static struct class *cl; //device class 


static struct file_operations fops =
{
	.owner	= THIS_MODULE,
	.read	= dev_read, 
	.open	= dev_open,
	.write	= dev_write,
	.release= dev_release,
};
	
static int __init ceid_camera_init(void)
{
	int ret;
	printk(KERN_INFO "CEID_CAM: Module loaded\n");
	printk(KERN_INFO "CEID_CAM: Device registration -> ");
	
	if ((ret = alloc_chrdev_region(&dev_num, 0, 1, "ceidCam")) < 0) {
		printk("failed!\n");
		return ret;
	}else {
		printk("succeded!\n");
		printk(KERN_INFO "CEID_CAM: <Major, Minor>: <%d, %d>\n", MAJOR(dev_num), MINOR(dev_num));
	}

	// add device to system
	cdev_init(&c_dev, &fops);
	printk(KERN_INFO "CEID_CAM: cdev_add -> ");
	if (ret = cdev_add(&c_dev,dev_num,1) < 0) {
		printk("failed\n");
		goto error;
	}else{
		printk("succeded!\n");
	}

	// create device class
	printk(KERN_INFO "CEID_CAM: class_create -> ");
	if (IS_ERR(cl = class_create(THIS_MODULE, "media"))) {
		printk("failed\n");
		ret = PTR_ERR(cl);
		goto error;
	}else{
		printk("succeded \n");
	}

	return 0;

error:
	cdev_del(&c_dev);
	unregister_chrdev_region(dev_num, 1);
	return ret;
}

static void __exit ceid_camera_exit(void)
{
	class_destroy(cl);
	cdev_del(&c_dev);
	unregister_chrdev_region(dev_num, 1);
	printk(KERN_INFO "CEID_CAM: module unloaded\n");
}

static int dev_open(struct inode *inod, struct file *fil)
{
	printk(KERN_INFO "CEID_CAM: Device Opened\n");
	return 0;
}

static ssize_t dev_read(struct file* filp, char __user *buff, size_t len, loff_t *off)
{
	printk(KERN_INFO "CEID_CAM: Reading from device\n");
	return 0;
}

static ssize_t dev_write(struct file *filp, const char __user *buff, size_t len, loff_t *off)
{
	printk(KERN_INFO "CEID_CAM: Writing to device\n");
	return 0;
}

static int dev_release(struct inode *inod, struct file *fil)
{
	//printk(KERN_INFO "CEID_CAM: Device Closed\n");
	return 0;
}

module_init(ceid_camera_init);
module_exit(ceid_camera_exit);
