#ifndef CEID_CAMERA_REGS_H
#define CEID_CAMERA_REGS_H

// Registers & Signals
#define I2C_PRER_LO		0xE0000000 // = 3'b000;
#define I2C_PRER_HI		0xE0000004 // = 3'b001;
#define I2C_CTR			0xE0000008 // = 3'b010;
#define I2C_RXR			0xE000000C // = 3'b011;
#define I2C_TXR			0xE000000C // = 3'b011;
#define I2C_CR			0xE0000010 // = 3'b100;
#define I2C_SR			0xE0000010 // = 3'b100;
#define I2C_TXR_R		0xE0000014 // = 3'b101; // undocumented / reserved output
#define I2C_CR_R		0xE0000014 // = 3'b110; // undocumented / reserved output

#define I2C_RD			1
#define I2C_WR			0
#define I2C_SADR		0xBA

#define I2C_CMD_STA		0x80
#define I2C_CMD_STO		0x40
#define I2C_CMD_RD		0x20
#define I2C_CMD_WR		0x10
#define I2C_CMD_NACK		0x08
#define I2C_CMD_ACK		0x00
#define I2C_CMD_IACK		0x01

#define I2C_STATUS_RxACK	0x80
#define I2C_STATUS_BUSY		0x40
#define I2C_STATUS_AL		0x20
#define I2C_STATUS_TIP		0x02
#define I2C_STATUS_IF		0x01

// DEFINITION OF I2C REGISTERS IN THE SENSOR
#define MT9T_CHIP_VERSION_0		0x00
#define MT9T_ROW_START			0x01
#define MT9T_COL_START			0x02
#define MT9T_HEIGHT			0x03
#define MT9T_WIDTH			0x04
#define MT9T_HOR_BLANK			0x05
#define MT9T_VER_BLANK			0x06
#define MT9T_OUT_CONTROL		0x07
#define MT9T_SHUT_WIDTH_U		0x08
#define MT9T_SHUT_WIDTH_L		0x09
#define MT9T_PIXEL_CLOCK		0x0A
#define MT9T_RESTART			0x0B
#define MT9T_SHUT_DELAY			0x0C
#define MT9T_RESET			0x0D
#define MT9T_READ_MODE1			0x1E
#define MT9T_READ_MODE2			0x20
#define MT9T_READ_MODE3			0x21
#define MT9T_ROW_ADDR_MODE		0x22
#define MT9T_COL_ADDR_MODE		0x23
#define MT9T_GREEN1_GAIN		0x2B
#define MT9T_BLUE_GAIN			0x2C
#define MT9T_RED_GAIN			0x2D
#define MT9T_GREEN2_GAIN		0x2E
#define MT9T_TEST_DATA			0x32
#define MT9T_GLOBAL_GAIN		0x35
#define MT9T_BLACK_GAIN			0x49
#define MT9T_ROW_BLACK_DEFAULT_OFFSET	0x4B
#define MT9T_BLC_DELTA_THRESHOLDS	0x5D
#define MT9T_CAL_THRESHOLD		0x5F
#define MT9T_GREEN1_OFFSET		0x60
#define MT9T_GREEN2_OFFSET		0x61
#define MT9T_BLACK_LEVEL_CALIBRATION	0x62
#define MT9T_RED_OFFSET			0x63
#define MT9T_BLUE_OFFSET		0x64
#define MT9T_CHIP_ENABLE		0xF8
#define MT9T_CHIP_VERSION_1		0xFF

#define MT9T_OUTPUT_TEST    0x0042
#define MT9T_OUTPUT_NORMAL  0x0002

// DEFINITIONS OF PROJECT_IS_NEW_SIF
#define TEST_X          256
#define TEST_Y          256
#define EXTRA_COLUMNS   32
#define IMAGE_REG_SKIP  0
#define TEST_EXTRA_Y    0

#endif
