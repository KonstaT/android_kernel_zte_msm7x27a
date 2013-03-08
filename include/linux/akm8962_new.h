/*
 * Definitions for akm8962 compass chip.
 */
#ifndef AKM8962_H
#define AKM8962_H

#include <linux/ioctl.h>

#define AKM8962_I2C_NAME "akm8962"

#define SENSOR_DATA_SIZE	8
#define YPR_DATA_SIZE		12
#define RWBUF_SIZE			16

#define ACC_DATA_FLAG		0
#define MAG_DATA_FLAG		1
#define ORI_DATA_FLAG		2
#define AKM_NUM_SENSORS		3

#define ACC_DATA_READY		(1<<(ACC_DATA_FLAG))
#define MAG_DATA_READY		(1<<(MAG_DATA_FLAG))
#define ORI_DATA_READY		(1<<(ORI_DATA_FLAG))

/*! \name AK8962 constant definition
 \anchor AK8962_Def
 Constant definitions of the AK8962.*/
#define AK8962_MEASUREMENT_TIME_US	10000

/*! \name AK8962 operation mode
 \anchor AK8962_Mode
 Defines an operation mode of the AK8962.*/
/*! @{*/
#define AK8962_MODE_SNG_MEASURE	0x01
#define	AK8962_MODE_SELF_TEST	0x08
#define	AK8962_MODE_FUSE_ACCESS	0x0F
#define	AK8962_MODE_POWERDOWN	0x00
/*! @}*/

/*! \name AK8962 register address
\anchor AK8962_REG
Defines a register address of the AK8962.*/
/*! @{*/
#define AK8962_REG_WIA		0x00
#define AK8962_REG_INFO		0x01
#define AK8962_REG_ST1		0x02
#define AK8962_REG_HXL		0x03
#define AK8962_REG_HXH		0x04
#define AK8962_REG_HYL		0x05
#define AK8962_REG_HYH		0x06
#define AK8962_REG_HZL		0x07
#define AK8962_REG_HZH		0x08
#define AK8962_REG_ST2		0x09
#define AK8962_REG_CNTL		0x0A
#define AK8962_REG_RSV		0x0B
#define AK8962_REG_ASTC		0x0C
#define AK8962_REG_TS1		0x0D
#define AK8962_REG_TS2		0x0E
#define AK8962_REG_I2CDIS	0x0F
/*! @}*/

/*! \name AK8962 fuse-rom address
\anchor AK8962_FUSE
Defines a read-only address of the fuse ROM of the AK8962.*/
/*! @{*/
#define AK8962_FUSE_ASAX	0x10
#define AK8962_FUSE_ASAY	0x11
#define AK8962_FUSE_ASAZ	0x12
/*! @}*/

#define AKMIO                   0xA1

/* IOCTLs for AKM library */
#define ECS_IOCTL_READ              _IOWR(AKMIO, 0x01, char*)
#define ECS_IOCTL_WRITE             _IOW(AKMIO, 0x02, char*)
#define ECS_IOCTL_SET_MODE          _IOW(AKMIO, 0x03, short)
#define ECS_IOCTL_GETDATA           _IOR(AKMIO, 0x04, char[SENSOR_DATA_SIZE])
#define ECS_IOCTL_SET_YPR           _IOW(AKMIO, 0x05, int[YPR_DATA_SIZE])
#define ECS_IOCTL_GET_OPEN_STATUS   _IOR(AKMIO, 0x06, int)
#define ECS_IOCTL_GET_CLOSE_STATUS  _IOR(AKMIO, 0x07, int)
#define ECS_IOCTL_GET_DELAY         _IOR(AKMIO, 0x08, long long int[AKM_NUM_SENSORS])
#define ECS_IOCTL_GET_LAYOUT        _IOR(AKMIO, 0x09, char)
#define ECS_IOCTL_GET_ACCEL			_IOR(AKMIO, 0x30, short[3])

struct akm8962_platform_data {
	char layout;
	int gpio_DRDY;
};

#endif

