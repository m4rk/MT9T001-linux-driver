/* Mandatory Driver module header files */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

MODULE_LICENSE("GPL");

// file operations fuctions prototypes
static int dev_open(struct inode *, struct file *);
static int dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);


static struct file_operations fops =
{
	.read	= dev_read, 
	.open	= dev_open,
	.write	= dev_write,
	.release= dev_release,
};
	
static int __init ceid_camera_init(void)
{
	printk(KERN_ALERT "CEID_CAMERA: module loaded\n");
	
	// TODO: get MAJOR dynamically, hardcode for now
	int t = register_chrdev(23,"ceid_cam",&fops);
	if(t<0)
		printk(KERN_ALERT "CEID_CAM: Device registration -> failed!\n");
	else
		printk(KERN_ALERT "CEID_CAM: Device registration -> succeded!\n");
	
	return t;
}

static void __exit ceid_camera_exit(void)
{
	unregister_chrdev(23,"ceid_cam");
	printk(KERN_ALERT "CEID_CAMERA: module unloaded\n");
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
	printk(KERN_ALERT "Device Closed\n");
	return 0;
}

module_init(ceid_camera_init);
module_exit(ceid_camera_exit);
