#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/types.h>
//#include <linux/kernel.h>

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
	printk(KERN_ALERT "CEID_CAM: Module loaded\n");
	printk(KERN_ALERT "CEID_CAM: Device registration -> ");
	
	if(alloc_chrdev_region(&dev_num, 0, 1, "ceidCam") < 0){
		printk("failed!\n");
	} else {
		printk("succeded!\n");
		printk(KERN_ALERT "CEID_CAM: <Major, Minor>: <%d, %d>\n", MAJOR(dev_num), MINOR(dev_num));
	}

	// create device class
	printk(KERN_ALERT "CEID_CAM: class_create -> ");

	if((cl = class_create(THIS_MODULE, "media") == NULL))
		printk("failed\n");
	else
		printk("succeded \n");

	printk(KERN_ALERT "CEID_CAM: device_create -> ");

	if(device_create(cl,NULL,dev_num,NULL,"ceidCam") == NULL)
		printk("failed\n");
	else
		printk("succeded \n");
		
	cdev_init(&c_dev, &fops);

	printk(KERN_ALERT "CEID_CAM: cdev_add -> ");

	if(cdev_add(&c_dev,dev_num,1) == -1)
		printk("failed\n");
	else
		printk("succeded!\n");


	return 0;
}

static void __exit ceid_camera_exit(void)
{
	cdev_del(&c_dev);
	device_destroy(cl,dev_num);
	class_destroy(cl);
	unregister_chrdev_region(dev_num, 1);
	printk(KERN_ALERT "CEID_CAM: module unloaded\n");
}

static int dev_open(struct inode *inod, struct file *fil)
{
	printk(KERN_ALERT "CEID_CAM: Device Opened\n");
	return 0;
}

static ssize_t dev_read(struct file* filp, char *buff, size_t len, loff_t *off)
{
	short count=1;
	printk(KERN_ALERT "CEID_CAM: Reading from device\n");
	return count;
}

static ssize_t dev_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	short count=2;
	printk(KERN_ALERT "CEID_CAM: Writing to device\n");
	return count;
}

static int dev_release(struct inode *inod, struct file *fil)
{
	printk(KERN_ALERT "CEID_CAM: Device Closed\n");
	return 0;
}

module_init(ceid_camera_init);
module_exit(ceid_camera_exit);
