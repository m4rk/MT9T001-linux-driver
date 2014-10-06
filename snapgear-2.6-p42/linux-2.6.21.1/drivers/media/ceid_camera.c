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
#include "ceid_camera_regs.h"

// file operations fuctions prototypes
static int dev_open(struct inode *, struct file *);
static int dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

static dev_t dev_num; // device number
static struct cdev c_dev; //char device structure
static struct class *cl; //device class 

static inline void init_ahb_i2c(void)
{
	LEON_BYPASS_STORE_PA(I2C_PRER_LO,0x90);
	LEON_BYPASS_STORE_PA(I2C_PRER_HI,0x01);
	LEON_BYPASS_STORE_PA(I2C_CTR,0x80);
}

static unsigned int i2c_read(int pos)
{
    	volatile unsigned temp, data;
        
	LEON_BYPASS_STORE_PA(I2C_TXR, I2C_SADR | I2C_WR);
	LEON_BYPASS_STORE_PA(I2C_CR, I2C_CMD_STA | I2C_CMD_WR);
	while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_TIP);

	LEON_BYPASS_STORE_PA(I2C_TXR, (pos & 0xFF));
	LEON_BYPASS_STORE_PA(I2C_CR, I2C_CMD_WR);
	while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_TIP);

	temp = 100; while(temp--);

	LEON_BYPASS_STORE_PA(I2C_TXR, I2C_SADR | I2C_RD);
	LEON_BYPASS_STORE_PA(I2C_CR, I2C_CMD_STA | I2C_CMD_WR);
	while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_TIP);

	LEON_BYPASS_STORE_PA(I2C_CR,I2C_CMD_RD | I2C_CMD_ACK);
	while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_TIP);

	data = LEON_BYPASS_LOAD_PA(I2C_RXR) & 0xFF;

	LEON_BYPASS_STORE_PA(I2C_CR, I2C_CMD_RD | I2C_CMD_NACK);
	while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_TIP);

	data = (data<<8) | (LEON_BYPASS_LOAD_PA(I2C_RXR) & 0xFF);

	// read 1 or 2 bytes...
	LEON_BYPASS_STORE_PA(I2C_CR,I2C_CMD_STO);
	while ( LEON_BYPASS_LOAD_PA(I2C_SR) & I2C_STATUS_BUSY);

	return data;
}

static void i2c_write(int pos, int val)
{
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

static void reset_sensor(void)
{
	i2c_write(MT9T_RESET, 0x01);
	i2c_write(MT9T_RESET, 0x00);
}

static void setup_sensor(void)
{
	unsigned int frame_vertical_skip	= 1;	// 0:no skip, 1:2x, 2:3x etc. 
	unsigned int frame_vertical_bin		= 2;	// 0:no bin, 1:2x, 2:3x .	
	unsigned int frame_horizontal_skip	= 2;	// 0:no skip, 1:2x, 2:3x etc.
	unsigned int frame_horizontal_bin	= 2;	// 0:no bin, 1:2x, 2:3x .
	unsigned int frame_top	= 20+768-TEST_Y*(1+frame_vertical_skip);
	unsigned int frame_left	= 20+1024-TEST_X*(1+frame_horizontal_skip);
	unsigned int frame_width	= (1+frame_horizontal_skip)*2*TEST_X-1; // 2048;// 1535;
	unsigned int frame_height	= (1+frame_vertical_skip)*2*TEST_Y-1;   // 1535; //767;
	unsigned int sensor_gain = 0xff ; // 0x0051; // Maximum analog gain 7F.... and no digital gain
	unsigned int skipbinv;
	unsigned int skipbinh;

	// Enable top rows from the sensor
	unsigned int total_rows_in_push_master;
	unsigned int total_cols_in_push_master;
	unsigned int rows_cols_in_push_master;

	i2c_write(MT9T_READ_MODE1,0x8040);  //by recomendation in the datasheet
	i2c_write(0x4E,0x0020);  //by recomendation in the datasheet
	i2c_write(MT9T_TEST_DATA,0x04A4);  //test data
	i2c_write( MT9T_OUT_CONTROL, MT9T_OUTPUT_NORMAL);  //don't generate test_data, just enable data output 
	i2c_write(MT9T_GLOBAL_GAIN,sensor_gain);  // gain	
	i2c_write(MT9T_RESTART,0x1); 
	i2c_write(MT9T_ROW_START,frame_top);    // First Row 
	i2c_write(MT9T_COL_START,frame_left);    // First Collumn
	i2c_write(MT9T_HEIGHT,frame_height);    // Set Window Height (up to 5FF)
	i2c_write(MT9T_WIDTH,frame_width+EXTRA_COLUMNS);    // Set Window Width  (up to 7FF)
	
	skipbinv = (frame_vertical_bin<<4  ) + (frame_vertical_skip & 0xF  );
	skipbinh = (frame_horizontal_bin<<4) + (frame_horizontal_skip & 0xF);
	
	i2c_write(MT9T_ROW_ADDR_MODE,skipbinv);    // Row Skip and Bin
	i2c_write(MT9T_COL_ADDR_MODE,skipbinh);    // Collumn Skip and Bin
	i2c_write(MT9T_SHUT_WIDTH_L,1000);  //128
	i2c_write(MT9T_BLACK_GAIN,0x2A<<2);   // black level
	i2c_write(MT9T_HOR_BLANK,0x008E); 
	i2c_write(MT9T_VER_BLANK,0x0019);
   
	total_rows_in_push_master = (frame_height + 1)/(4*(frame_vertical_skip   + 1))-1;
	total_cols_in_push_master = 512/4-1;
	rows_cols_in_push_master = (total_rows_in_push_master<<8) | total_cols_in_push_master;
	rows_cols_in_push_master = rows_cols_in_push_master & 0x0000FFFF;
    
	LEON_BYPASS_STORE_PA(0xF0000004, rows_cols_in_push_master);
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
	reset_sensor();
	setup_sensor();

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
