#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h> // various structures related to file_operations 
#include <linux/device.h> //  struct of type "device"
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h> // several kernel macros and functions
#include <linux/uaccess.h>
#include <asm/io.h>
#include <asm/leon.h> // LEON_BYPASS assembly functions

#include "ceid_camera.h"

// file operations fuctions prototypes
static int dev_open(struct inode *, struct file *);
static int dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

static dev_t dev_num; // device number
static struct cdev c_dev; //char device structure
static struct class *cl; //device class 

// Registers & Signals
#define I2C_PRER_LO        0xE0000000       //     = 3'b000;
#define I2C_PRER_HI        0xE0000004       //     = 3'b001;
#define I2C_CTR            0xE0000008       //     = 3'b010;

#define I2C_RXR            0xE000000C       //     = 3'b011;
#define I2C_TXR            0xE000000C       //     = 3'b011;

#define I2C_CR             0xE0000010       //     = 3'b100;
#define I2C_SR             0xE0000010       //     = 3'b100;
#define I2C_TXR_R          0xE0000014       //     = 3'b101; // undocumented / reserved output
#define I2C_CR_R           0xE0000014       //     = 3'b110; // undocumented / reserved output

#define I2C_RD             1
#define I2C_WR             0
#define I2C_SADR           0xBA

#define I2C_CMD_STA        0x80
#define I2C_CMD_STO        0x40
#define I2C_CMD_RD         0x20
#define I2C_CMD_WR         0x10
#define I2C_CMD_NACK       0x08
#define I2C_CMD_ACK        0x00
#define I2C_CMD_IACK       0x01

#define I2C_STATUS_RxACK   0x80
#define I2C_STATUS_BUSY    0x40
#define I2C_STATUS_AL      0x20
#define I2C_STATUS_TIP     0x02
#define I2C_STATUS_IF      0x01



static inline void init_ahb_i2c(void)
{
	LEON_BYPASS_STORE_PA(I2C_PRER_LO,0x90);
	LEON_BYPASS_STORE_PA(I2C_PRER_HI,0x01);
	LEON_BYPASS_STORE_PA(I2C_CTR,0x80);
}

static unsigned int i2c_read(int pos)
{
    
    volatile unsigned temp, data;
    //int i;
    
    LEON_BYPASS_STORE_PA(I2C_TXR, I2C_SADR | I2C_WR);             //
    LEON_BYPASS_STORE_PA(I2C_CR, I2C_CMD_STA | I2C_CMD_WR);      //
    while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_TIP);     //

    LEON_BYPASS_STORE_PA(I2C_TXR, (pos & 0xFF)) ;                     //
    LEON_BYPASS_STORE_PA(I2C_CR, I2C_CMD_WR);                    //
    while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_TIP);

    temp = 100; while(temp--);


    LEON_BYPASS_STORE_PA(I2C_TXR, I2C_SADR | I2C_RD);             //
    LEON_BYPASS_STORE_PA(I2C_CR, I2C_CMD_STA | I2C_CMD_WR);      //
    while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_TIP);     //

    LEON_BYPASS_STORE_PA(I2C_CR,I2C_CMD_RD | I2C_CMD_ACK);      //
    while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_TIP);     //

    data = LEON_BYPASS_LOAD_PA(I2C_RXR) & 0xFF;

    LEON_BYPASS_STORE_PA(I2C_CR, I2C_CMD_RD | I2C_CMD_NACK);     //
    while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_TIP);    //

    data = (data<<8) | (LEON_BYPASS_LOAD_PA(I2C_RXR) & 0xFF);

    // read 1 or 2 bytes...

    LEON_BYPASS_STORE_PA(I2C_CR,I2C_CMD_STO);
    while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_BUSY);

    return data;
    
}

static void i2c_write(int pos, int val){
    
    //int i;
    
    LEON_BYPASS_STORE_PA(I2C_TXR, I2C_SADR | I2C_WR);
    LEON_BYPASS_STORE_PA(I2C_CR, I2C_CMD_STA | I2C_CMD_WR);
    while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_TIP);

    LEON_BYPASS_STORE_PA(I2C_TXR, pos&0xFF) ;
    LEON_BYPASS_STORE_PA(I2C_CR, I2C_CMD_WR);
    while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_TIP); 

    // write MSB
    LEON_BYPASS_STORE_PA(I2C_TXR, (val&0xFF00)>>8 );
    LEON_BYPASS_STORE_PA(I2C_CR, I2C_CMD_WR);
    while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_TIP); 


    // write LSB
    LEON_BYPASS_STORE_PA(I2C_TXR, val&0xFF );
    LEON_BYPASS_STORE_PA(I2C_CR, I2C_CMD_WR);
    while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_TIP); 


    LEON_BYPASS_STORE_PA(I2C_CR, I2C_CMD_STO);
    while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_BUSY);
}

static long dev_ioctl(struct file *f, unsigned int cmd, unsigned long data)
{
	reg_struct r;
	//r.addr = 0;
	//r.val = 0;
	//printk(KERN_INFO "DRV:BEFORE: r.addr->%x r.val->%x\n\n",r.addr,r.val);
	switch(cmd)
	{
		case CEIDCAM_RD_REG:
			if (copy_from_user(&r, (reg_struct *)data, sizeof(reg_struct)))
				return -EACCES;
			//printk(KERN_INFO "DRV: r.addr: %x\n",r.addr);
			r.val = i2c_read(r.addr);
			//printk(KERN_INFO "DRV: r.val: %x\n",r.val);
			if (copy_to_user((reg_struct *)data, &r, sizeof(reg_struct)))
				return -EACCES;
		break;
		case CEIDCAM_WR_REG:
			if (copy_from_user(&r, (reg_struct *)data, sizeof(reg_struct)))
				return -EACCES;
			i2c_write(r.addr, r.val);
		break;
		default:
			return -ENOTTY;
	}

	return 0;
}

static struct file_operations fops =
{
	.owner	= THIS_MODULE,
	.read	= dev_read,
	.write	= dev_write,
	.unlocked_ioctl	= dev_ioctl,
	.open	= dev_open,
	.release= dev_release
};

// Device initialization/registration
static int __init ceid_camera_init(void)
{
	int ret;
	printk(KERN_INFO "CEID_CAM: Module loaded\n");
	
	// request major/minor numbers
	if ((ret = alloc_chrdev_region(&dev_num, 0, 1, "ceidCam")) < 0) {
		printk(KERN_INFO "CEID_CAM: Device registration -> failed!\n");
		return ret;
	}

	// add device to system
	cdev_init(&c_dev, &fops);
	if ((ret = cdev_add(&c_dev,dev_num,1)) < 0) {
		printk(KERN_INFO "CEID_CAM: cdev_add -> failed\n");
		goto error;
	}

	// create device class
	if (IS_ERR(cl = class_create(THIS_MODULE, "media"))) {
		printk(KERN_INFO "CEID_CAM: class_create -> failed\n");
		ret = PTR_ERR(cl);
		goto error;
	}

	init_ahb_i2c();

	return 0;

error:
	cdev_del(&c_dev);
	unregister_chrdev_region(dev_num, 1);
	return ret;
}

// Cleanup
static void __exit ceid_camera_exit(void)
{
	class_destroy(cl);
	cdev_del(&c_dev);
	unregister_chrdev_region(dev_num, 1);
	printk(KERN_INFO "CEID_CAM: module unloaded\n");
}

// Open
static int dev_open(struct inode *inod, struct file *fil)
{
	//printk(KERN_INFO "CEID_CAM: Device Opened\n");
	return 0;
}

// Reading
static ssize_t dev_read(struct file* filp, char __user *buff, size_t len, loff_t *off)
{
	unsigned int ret;
	ret = i2c_read(0x00);

	return ret;
}

// Writing
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

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Markellos Orfanos <mark.orfanos@gmail.com>");
MODULE_DESCRIPTION("Aptina MT9T001 Image Sensor Driver");
