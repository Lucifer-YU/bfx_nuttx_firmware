/*
 * I2C.h
 *
 *  Created on: Mar 30, 2015
 *      Author: Lucifer
 */

#ifndef FIRMWARE_SRC_DRIVERS_DEVICE_I2C_H_
#define FIRMWARE_SRC_DRIVERS_DEVICE_I2C_H_

#include "CharDevice.h"

#include <nuttx/i2c.h>

namespace device {

/**
 * Abstract class for character device on I2C
 */
class __EXPORT I2C: public CharDevice {
public:
	/**
	 * Get the address
	 */
	int16_t get_address() const {
		return _address;
	}

protected:
	/**
	 *
	 * @param name
	 * 		Driver name
	 * @param devname
	 * 		Device node name
	 * @param bus
	 * 		I2C bus on which the device lives
	 * @param address
	 * 		I2C bus address, or zero if set_address will be used
	 * @param frequency
	 * 		I2C bus frequency for the device (currently not used)
	 * @param irq
	 * 		Interrupt assigned to the device (or zero if none)
	 */
	I2C(const char *name, const char *devname, int bus, uint16_t address,
			uint32_t frequency, int irq = 0);
	virtual ~I2C();

	virtual int init();

	/**
	 * Check for the presence of the device on the bus.
	 */
	virtual int probe();

	/**
	 * Perform an I2C transaction to the device.
	 *
	 * At least one of send_len and recv_len must be non-zero.
	 *
	 * @param send
	 * 		Pointer to bytes to send.
	 * @param send_len
	 * 		Number of bytes to send.
	 * @param recv
	 * 		Pointer to buffer for bytes received.
	 * @param recv_len
	 * 		Number of bytes to receive.
	 * @return OK if the transfer was successful, -errno otherwise.
	 */
	int transfer(const uint8_t *send, unsigned int send_len, uint8_t *recv,
			unsigned int recv_len);

	/**
	 * Perform a multi-part I2C transaction to the device.
	 *
	 * @param msgv
	 * 		An I2C message vector.
	 * @param msgs
	 * 		The number of entries in the message vector.
	 * @return OK if the transfer was successful, -errno otherwise.
	 */
	int transfer(i2c_msg_s *msgv, unsigned int msgs);

	/**
	 * Change the bus address.
	 *
	 * Most often useful during probe() when the driver is testing
	 * several possible bus addresses.
	 *
	 * @param address
	 * 		The new bus address to set.
	 */
	void set_address(uint16_t address) {
		_address = address;
		_device_id.devid_s.address = _address;
	}

private:
	I2C(const device::I2C&);
	I2C operator=(const device::I2C&);

protected:
	/**
	 * The number of times a read or write operation will be retried on error.
	 */
	unsigned int _retries;

	/**
	 * The I2C bus number the device is attached to.
	 */
	int _bus;

private:
	uint16_t _address;
	uint32_t _frequency;
	struct i2c_dev_s *_dev;
};

} /* namespace device */

#endif /* FIRMWARE_SRC_DRIVERS_DEVICE_I2C_H_ */
