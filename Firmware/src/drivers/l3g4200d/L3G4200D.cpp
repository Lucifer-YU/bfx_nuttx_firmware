/*
 * L3G4200D.cpp
 *
 *  Created on: Apr 2, 2015
 *      Author: Lucifer
 */

#include "L3G4200D.h"

///////
/// Accelerometer ADXL345 register definitions
#define ADXL345_ACCELEROMETER_ADDRESS                  0x53
#define ADXL345_ACCELEROMETER_XL345_DEVID              0xe5
#define ADXL345_ACCELEROMETER_ADXLREG_BW_RATE          0x2c
#define ADXL345_ACCELEROMETER_ADXLREG_POWER_CTL        0x2d
#define ADXL345_ACCELEROMETER_ADXLREG_DATA_FORMAT      0x31
#define ADXL345_ACCELEROMETER_ADXLREG_DEVID            0x00
#define ADXL345_ACCELEROMETER_ADXLREG_DATAX0           0x32
#define ADXL345_ACCELEROMETER_ADXLREG_FIFO_CTL         0x38
#define ADXL345_ACCELEROMETER_ADXLREG_FIFO_CTL_STREAM     0x9F
#define ADXL345_ACCELEROMETER_ADXLREG_FIFO_STATUS      0x39

// ADXL345 accelerometer scaling
// Result will be scaled to 1m/s/s
// ADXL345 in Full resolution mode (any g scaling) is 256 counts/g, so scale by 9.81/256 = 0.038320312
#define ADXL345_ACCELEROMETER_SCALE_M_S    (GRAVITY_MSS / 256.0f)

/// Gyro ITG3205 register definitions
#define L3G4200D_I2C_ADDRESS                       0x69

#define L3G4200D_REG_WHO_AM_I                      0x0f
#define L3G4200D_REG_WHO_AM_I_VALUE                     0xd3

#define L3G4200D_REG_CTRL_REG1                     0x20
#define L3G4200D_REG_CTRL_REG1_DRBW_800_110             0xf0
#define L3G4200D_REG_CTRL_REG1_PD                       0x08
#define L3G4200D_REG_CTRL_REG1_XYZ_ENABLE               0x07

#define L3G4200D_REG_CTRL_REG4                     0x23
#define L3G4200D_REG_CTRL_REG4_FS_2000                  0x30

#define L3G4200D_REG_CTRL_REG5                     0x24
#define L3G4200D_REG_CTRL_REG5_FIFO_EN                  0x40

#define L3G4200D_REG_FIFO_CTL                      0x2e
#define L3G4200D_REG_FIFO_CTL_STREAM                    0x40

#define L3G4200D_REG_FIFO_SRC                      0x2f
#define L3G4200D_REG_FIFO_SRC_ENTRIES_MASK              0x1f
#define L3G4200D_REG_FIFO_SRC_EMPTY                     0x20
#define L3G4200D_REG_FIFO_SRC_OVERRUN                   0x40

#define L3G4200D_REG_XL                            0x28

// this bit is ORd into the register to enable auto-increment mode
#define L3G4200D_REG_AUTO_INCREMENT		           0x80

// L3G4200D Gyroscope scaling
// running at 2000 DPS full range, 16 bit signed data, datasheet
// specifies 70 mdps per bit
#define L3G4200D_GYRO_SCALE_R_S (DEG_TO_RAD * 70.0f * 0.001f)

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
#undef ERROR
#endif
#define ERROR (-1)

////////////////////////////////////////////////////////////////////////////////

namespace device {

L3G4200D::L3G4200D(int bus, const char* path) :
		I2C("L3G4200D", path, bus, L3G4200D_I2C_ADDRESS, 400000) {
	_device_id.devid_s.devtype = DRV_GYRO_DEVTYPE_L3G4200D;

	_class_instance = -1;
	// TODO
}

L3G4200D::~L3G4200D() {

	// TODO

	if (_class_instance != -1)
		unregister_class_devname(MAG_DEVICE_PATH, _class_instance);

}

int L3G4200D::init() {
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;


	// TODO

    // Init the Gyro
    // Expect to read the right 'WHO_AM_I' value
	uint8_t data = 0;
	ret = read_reg(L3G4200D_REG_WHO_AM_I, data);
	if (OK != ret) {
		log("could not to read L3G4200D who am I register");
		goto out;
	}
	if (data != L3G4200D_REG_WHO_AM_I_VALUE) {
		log("could not find L3G4200D sensor");
		goto out;
	}

	_class_instance = register_class_devname(GYRO_DEVICE_PATH);

	ret = OK;
	/* sensor is OK, but not calibrated */
	// _sensor_ok = true;
out:
	return ret;
}

ssize_t L3G4200D::read(struct file *filp, char *buffer, size_t buflen) {
	return -ERROR;
}

int L3G4200D::ioctl(struct file *filp, int cmd, unsigned long arg) {
	return -ERROR;
}

void L3G4200D::start() {

}

void L3G4200D::stop() {

}

} /* namespace device */
