/*
 * @file include/linux/dmt10.h
 * @brief DMT g-sensor Linux device driver
 * @author Domintech Technology Co., Ltd (http://www.domintech.com.tw)
 * @version 1.00
 * @date 2012/9/21
 *
 * @section LICENSE
 *
 *  Copyright 2012 Domintech Technology Co., Ltd
 *
 * 	This software is licensed under the terms of the GNU General Public
 * 	License version 2, as published by the Free Software Foundation, and
 * 	may be copied, distributed, and modified under those terms.
 *
 * 	This program is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * 	GNU General Public License for more details.
 *
 */
#ifndef DMT10_H
#define DMT10_H
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/syscalls.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#define AUTO_CALIBRATION	0

#define DMT_DEBUG_DATA
#define GSE_TAG                  "[DMT_Gsensor]"
#ifdef DMT_DEBUG_DATA
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(KERN_INFO GSE_TAG fmt, ##args)
#define GSE_FUN(f)               printk(KERN_INFO GSE_TAG" %s: %s: %i\n", __FILE__, __func__, __LINE__)
#define DMT_DATA(dev, ...)		 dev_dbg((dev), ##__VA_ARGS__)
#else
#define GSE_ERR(fmt, args...)
#define GSE_LOG(fmt, args...)
#define GSE_FUN(f)
#define DMT_DATA(dev, format, ...)
#endif

#define INPUT_NAME_ACC			"DMT_accel"	/* Input Device Name  */
#define DEVICE_I2C_NAME 		"dmt"		/* Device name for DMARD10 misc. device */
#define REG_ACTR 				0x00
#define REG_TAPNS				0x0f
#define REG_MISC2				0x1f
#define REG_AFEM 				0x0c
#define REG_CKSEL 				0x0d
#define REG_INTC 				0x0e
#define REG_STADR 				0x12
#define REG_STAINT 				0x1C
#define REG_PD					0x21
#define REG_X_OUT 				0x41

#define MODE_Off				0x00
#define MODE_ResetAtOff			0x01
#define MODE_Standby			0x02
#define MODE_ResetAtStandby		0x03
#define MODE_Active				0x06
#define MODE_Trigger			0x0a
#define MODE_ReadOTP			0x12
#define MODE_WriteOTP			0x22
#define MODE_ResetDataPath		0x82

#define VALUE_STADR					0x55
#define VALUE_STAINT 				0xAA
#define VALUE_AFEM_AFEN_Normal		0x8f// AFEN set 1 , ATM[2:0]=b'000(normal),EN_Z/Y/X/T=1
#define VALUE_AFEM_Normal			0x0f// AFEN set 0 , ATM[2:0]=b'000(normal),EN_Z/Y/X/T=1
#define VALUE_INTC					0x00// INTC[6:5]=b'00 
#define VALUE_INTC_Interrupt_En		0x20// INTC[6:5]=b'01 (Data ready interrupt enable, active high at INT0)
#define VALUE_CKSEL_ODR_0_204		0x04// ODR[3:0]=b'0000 (0.78125Hz), CCK[3:0]=b'0100 (204.8kHZ)
#define VALUE_CKSEL_ODR_1_204		0x14// ODR[3:0]=b'0001 (1.5625Hz), CCK[3:0]=b'0100 (204.8kHZ)
#define VALUE_CKSEL_ODR_3_204		0x24// ODR[3:0]=b'0010 (3.125Hz), CCK[3:0]=b'0100 (204.8kHZ)
#define VALUE_CKSEL_ODR_6_204		0x34// ODR[3:0]=b'0011 (6.25Hz), CCK[3:0]=b'0100 (204.8kHZ)
#define VALUE_CKSEL_ODR_12_204		0x44// ODR[3:0]=b'0100 (12.5Hz), CCK[3:0]=b'0100 (204.8kHZ)
#define VALUE_CKSEL_ODR_25_204		0x54// ODR[3:0]=b'0101 (25Hz), CCK[3:0]=b'0100 (204.8kHZ)
#define VALUE_CKSEL_ODR_50_204		0x64// ODR[3:0]=b'0110 (50Hz), CCK[3:0]=b'0100 (204.8kHZ)
#define VALUE_CKSEL_ODR_100_204		0x74// ODR[3:0]=b'0111 (100Hz), CCK[3:0]=b'0100 (204.8kHZ)

#define VALUE_TAPNS_NoFilter	0x00	// TAP1/TAP2	NO FILTER
#define VALUE_TAPNS_Ave_2		0x11	// TAP1/TAP2	Average 2
#define VALUE_TAPNS_Ave_4		0x22	// TAP1/TAP2	Average 4
#define VALUE_TAPNS_Ave_8		0x33	// TAP1/TAP2	Average 8
#define VALUE_TAPNS_Ave_16		0x44	// TAP1/TAP2	Average 16
#define VALUE_TAPNS_Ave_32		0x55	// TAP1/TAP2	Average 32
#define VALUE_MISC2_OSCA_EN		0x08
#define VALUE_PD_RST			0x52

#define CONFIG_GSEN_CALIBRATION_GRAVITY_ON_Z_NEGATIVE 1
#define CONFIG_GSEN_CALIBRATION_GRAVITY_ON_Z_POSITIVE 2
#define CONFIG_GSEN_CALIBRATION_GRAVITY_ON_Y_NEGATIVE 3
#define CONFIG_GSEN_CALIBRATION_GRAVITY_ON_Y_POSITIVE 4
#define CONFIG_GSEN_CALIBRATION_GRAVITY_ON_X_NEGATIVE 5
#define CONFIG_GSEN_CALIBRATION_GRAVITY_ON_X_POSITIVE 6

#define AVG_NUM 				16
#define SENSOR_DATA_SIZE 		3 
#define DEFAULT_SENSITIVITY 	1024

#define IOCTL_MAGIC  0x09
#define SENSOR_RESET    		_IO(IOCTL_MAGIC, 0)
#define SENSOR_CALIBRATION   	_IOWR(IOCTL_MAGIC,  1, int[SENSOR_DATA_SIZE])
#define SENSOR_GET_OFFSET  		_IOR(IOCTL_MAGIC,  2, int[SENSOR_DATA_SIZE])
#define SENSOR_SET_OFFSET  		_IOWR(IOCTL_MAGIC,  3, int[SENSOR_DATA_SIZE])
#define SENSOR_READ_ACCEL_XYZ  	_IOR(IOCTL_MAGIC,  4, int[SENSOR_DATA_SIZE])
#define SENSOR_SETYPR  			_IOW(IOCTL_MAGIC,  5, int[SENSOR_DATA_SIZE])
#define SENSOR_GET_OPEN_STATUS	_IO(IOCTL_MAGIC,  6)
#define SENSOR_GET_CLOSE_STATUS	_IO(IOCTL_MAGIC,  7)
#define SENSOR_GET_DELAY		_IOR(IOCTL_MAGIC,  8, unsigned int*)
#define SENSOR_MAXNR 8

/* g-senor layout configuration, choose one of the following configuration */
#define CONFIG_GSEN_LAYOUT_PAT_1	1
#define CONFIG_GSEN_LAYOUT_PAT_2	0
#define CONFIG_GSEN_LAYOUT_PAT_3	0
#define CONFIG_GSEN_LAYOUT_PAT_4	0
#define CONFIG_GSEN_LAYOUT_PAT_5	0
#define CONFIG_GSEN_LAYOUT_PAT_6	0
#define CONFIG_GSEN_LAYOUT_PAT_7	0
#define CONFIG_GSEN_LAYOUT_PAT_8	0

s16 sensorlayout[3][3] = {
#if CONFIG_GSEN_LAYOUT_PAT_1
    { 1, 0, 0},	{ 0, 1,	0}, { 0, 0, 1},
#elif CONFIG_GSEN_LAYOUT_PAT_2
    { 0, 1, 0}, {-1, 0,	0}, { 0, 0, 1},
#elif CONFIG_GSEN_LAYOUT_PAT_3
    {-1, 0, 0},	{ 0,-1,	0}, { 0, 0, 1},
#elif CONFIG_GSEN_LAYOUT_PAT_4
    { 0,-1, 0},	{ 1, 0,	0}, { 0, 0, 1},
#elif CONFIG_GSEN_LAYOUT_PAT_5
    {-1, 0, 0},	{ 0, 1,	0}, { 0, 0,-1},
#elif CONFIG_GSEN_LAYOUT_PAT_6
    { 0,-1, 0}, {-1, 0,	0}, { 0, 0,-1},
#elif CONFIG_GSEN_LAYOUT_PAT_7
    { 1, 0, 0},	{ 0,-1,	0}, { 0, 0,-1},
#elif CONFIG_GSEN_LAYOUT_PAT_8
    { 0, 1, 0},	{ 1, 0,	0}, { 0, 0,-1},
#endif
};

typedef union {
	struct {
		s16	x;
		s16	y;
		s16	z;
	} u;
	s16	v[SENSOR_DATA_SIZE];
} raw_data;

struct dmt_data {
	dev_t 					devno;
	struct cdev 			cdev;
	struct device			*class_dev;
  	struct class 			*class;
  	struct input_dev 		*input;
	struct i2c_client 		*client;
	struct delayed_work 	delaywork;	//work;
	struct work_struct 		work;	//irq_work;
	struct mutex 			sensor_mutex;
	wait_queue_head_t		open_wq;
	atomic_t				active;
	atomic_t 				delay;
	atomic_t 				enable;
};

#define ACC_DATA_FLAG		0
#define MAG_DATA_FLAG		1
#define ORI_DATA_FLAG		2
#define DMT_NUM_SENSORS		3
#endif               
