/*
 * L3G4200D.h
 *
 *  Created on: Apr 2, 2015
 *      Author: Lucifer
 */

#ifndef FIRMWARE_SRC_DRIVERS_L3G4200D_L3G4200D_H_
#define FIRMWARE_SRC_DRIVERS_L3G4200D_L3G4200D_H_

#include "../device/I2C.h"
#include "../drv_device.h"

namespace device {

class L3G4200D: public device::I2C {
public:
	L3G4200D(int bus, const char* path);
	virtual ~L3G4200D();

	virtual int init();
	virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);
	virtual int ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	/*
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 * 		to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/*
	 * Stop the automatic measurement state machine.
	 */
	void stop();

	/**
	 * Write a register.
	 *
	 * @param reg
	 * 		The register to write.
	 * @param val
	 * 		The value to write.
	 * @return OK on write success.
	 */
	int write_reg(uint8_t reg, uint8_t val);

	/**
	 * Read a register.
	 *
	 * @param reg
	 * 		The register to read.
	 * @param val
	 * 		The value read.
	 * @return OK on read success.
	 */
	int read_reg(uint8_t reg, uint8_t &val);

private:
	int	_class_instance;
};

} /* namespace device */

#endif /* FIRMWARE_SRC_DRIVERS_L3G4200D_L3G4200D_H_ */
