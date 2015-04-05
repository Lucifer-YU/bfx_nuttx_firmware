/*
 * I2C.cpp
 *
 *  Created on: Mar 30, 2015
 *      Author: Lucifer
 */

#include "I2C.h"

namespace device {

I2C::I2C(const char *name, const char *devname, int bus, uint16_t address,
		uint32_t frequency, int irq) :
		CharDevice(name, devname, irq) {
	_retries = 0;
	_bus = bus;
	_address = address;
	_frequency = frequency;
	_dev = nullptr;

	// fill in _device_id fields for a I2C device
	_device_id.devid_s.bus_type = DeviceBusType_I2C;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = address;
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;
}

I2C::~I2C() {
	if (_dev) {
		up_i2cuninitialize(_dev);
		_dev = nullptr;
	}
}

int I2C::init() {
	int ret = OK;

	// attach to the i2c bus
	_dev = up_i2cinitialize(_bus);
	if (_dev == nullptr) {
		debug("failed to init I2C");
		ret = -ENOENT;
		goto out;
	}

	// call the probe function to check whether the device is present
	ret = probe();
	if (ret != OK) {
		debug("probe failed");
		goto out;
	}

	// do base class init, which will create device node, etc
	ret = CharDevice::init();
	if (ret != OK) {
		debug("cdev init failed");
		goto out;
	}

	// tell the world where we are
	log("on I2C bus %d at 0x%02x", _bus, _address);

out:
	if ((ret != OK) && (_dev != nullptr)) {
		up_i2cuninitialize(_dev);
		_dev = nullptr;
	}
	return ret;
}

int I2C::probe() {
	// Assume the device is too stupid to be discoverable.
	return OK;
}

int I2C::transfer(const uint8_t *send, unsigned int send_len, uint8_t *recv,
		unsigned int recv_len) {
	struct i2c_msg_s msgv[2];
	unsigned msgs;
	int ret;
	unsigned retry_count = 0;

	do {
		debug("transfer out %p/%u  in %p/%u", send, send_len, recv, recv_len);
		msgs = 0;

		if (send_len > 0) {
			msgv[msgs].addr = _address;
			msgv[msgs].flags = 0;
			msgv[msgs].buffer = const_cast<uint8_t *>(send);
			msgv[msgs].length = send_len;
			msgs++;
		}

		if (recv_len > 0) {
			msgv[msgs].addr = _address;
			msgv[msgs].flags = I2C_M_READ;
			msgv[msgs].buffer = recv;
			msgv[msgs].length = recv_len;
			msgs++;
		}

		if (msgs == 0)
			return -EINVAL;

		/*
		 * I2C architecture means there is an unavoidable race here
		 * if there are any devices on the bus with a different frequency
		 * preference.  Really, this is pointless.
		 */
		I2C_SETFREQUENCY(_dev, _frequency);
		ret = I2C_TRANSFER(_dev, &msgv[0], msgs);

		/* success */
		if (ret == OK)
			break;

		/* if we have already retried once, or we are going to give up, then reset the bus */
		if ((retry_count >= 1) || (retry_count >= _retries))
			up_i2creset(_dev);

	} while (retry_count++ < _retries);

	return ret;
}

int I2C::transfer(i2c_msg_s *msgv, unsigned int msgs) {
	int ret;
	unsigned retry_count = 0;

	/* force the device address into the message vector */
	for (unsigned i = 0; i < msgs; i++)
		msgv[i].addr = _address;

	do {
		/*
		 * I2C architecture means there is an unavoidable race here
		 * if there are any devices on the bus with a different frequency
		 * preference.  Really, this is pointless.
		 */
		I2C_SETFREQUENCY(_dev, _frequency);
		ret = I2C_TRANSFER(_dev, msgv, msgs);

		/* success */
		if (ret == OK)
			break;

		/* if we have already retried once, or we are going to give up, then reset the bus */
		if ((retry_count >= 1) || (retry_count >= _retries))
			up_i2creset(_dev);

	} while (retry_count++ < _retries);

	return ret;
}

} /* namespace device */
