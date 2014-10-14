#ifndef CEID_CAMERA_H
#define CEID_CAMERA_H

#include <linux/ioctl.h>

#define DRVNAME		"MT9T001"
#define BUFFER_SIZE	256 //image buffer (KB)

// internal driver struct to use for i2c_read/write
typedef struct
{
    unsigned int addr, val;
} reg_struct;

// ioctl defines
#define IOC_MAGIC 'm'
#define CEIDCAM_RD_REG _IOR(IOC_MAGIC, 1, reg_struct *)
#define CEIDCAM_WR_REG _IOW(IOC_MAGIC, 2, reg_struct *)

#endif
