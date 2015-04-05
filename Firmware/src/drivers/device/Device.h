/*
 * Device.h
 *
 *  Created on: Mar 27, 2015
 *      Author: Lucifer
 */

#ifndef FIRMWARE_SRC_DRIVERS_DEVICE_DEVICE_H_
#define FIRMWARE_SRC_DRIVERS_DEVICE_DEVICE_H_

/*
 * Includes here should only cover the needs of the framework definitions.
 */
#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <poll.h>

#include "visibility.h"

namespace device {

/**
 * Fundamental base class for all device drivers.
 * This class handles the basic "being a driver" things, including interrupt
 * registration and dispatch.
 */
class __EXPORT Device {
protected:
	/**
	 * @param name
	 * 		Driver name
	 * @param irq
	 * 		Interrupt assigned to the device.
	 */
	Device(const char *name, int irq = 0);

public:
	/**
	 * Destructor
	 *
	 * Public so that anonymous devices can be destroyed.
	 */
	virtual ~Device();

	/**
	 * Initialise the driver and make it ready for use.
	 * @return OK if the driver initialized OK, negative errno otherwise;
	 */
	virtual int init();

	/**
	 * @param ctx
	 */
	virtual void interrupt(void *ctx);

	/**
	 * Read directly from the device.
	 * The actual size of each unit quantity is device-specific.
	 *
	 * @param offset
	 * 		The device address at which to start reading
	 * @param data
	 * 		The buffer into which the read values should be placed.
	 * @param count
	 * 		The number of items to read.
	 * @return The number of items read on success, negative errno otherwise.
	 */
	virtual int read(unsigned int address, void *data, unsigned int count);

	/**
	 * Write directly to the device.
	 *
	 * The actual size of each unit quantity is device-specific.
	 *
	 * @param address
	 * 		The device address at which to start writing.
	 * @param data
	 * 		The buffer from which values should be read.
	 * @param count
	 * 		The number of items to write.
	 * @return The number of items written on success, negative errno otherwise.
	 */
	virtual int write(unsigned int address, void *data, unsigned int count);

	/**
	 * Perform a device-specific operation.
	 *
	 * @param operation
	 * 		The operation to perform.
	 * @param arg
	 * 		An argument to the operation.
	 * @return Negative errno on error, OK or positive value on success.
	 */
	virtual int ioctl(unsigned int operation, unsigned int &arg);

	/**
	 device bus types for DEVID
	 */
	enum DeviceBusType {
		DeviceBusType_UNKNOWN = 0,
		DeviceBusType_I2C = 1,
		DeviceBusType_SPI = 2,
		DeviceBusType_UAVCAN = 3,
	};

	/**
	 broken out device elements. The bit-fields are used to keep
	 the overall value small enough to fit in a float accurately,
	 which makes it possible to transport over the MAVLink
	 parameter protocol without loss of information.
	 */
	struct DeviceStructure {
		enum DeviceBusType bus_type :3;
		uint8_t bus :5;		/// which instance of the bus type
		uint8_t address;	/// address on the bus (eg. I2C address)
		uint8_t devtype;	/// device class specific device type
	};

	union DeviceId {
		struct DeviceStructure devid_s;
		uint32_t devid;
	};

protected:
	/**
	 * Enable the device interrupt
	 */
	void interrupt_enable();

	/**
	 * Disable the device interrupt
	 */
	void interrupt_disable();

	/**
	 * Take the driver lock.
	 *
	 * Each driver instance has its own lock/semaphore.
	 *
	 * Note that we must loop as the wait may be interrupted by a signal.
	 */
	void lock() {
		do {
		} while (sem_wait(&_lock) != 0);
	}

	/**
	 * Release the driver lock.
	 */
	void unlock() {
		sem_post(&_lock);
	}

	/**
	 * Log a message.
	 *
	 * The message is prefixed with the driver name, and followed
	 * by a newline.
	 */
	void log(const char *fmt, ...);

	/**
	 * Print a debug message.
	 *
	 * The message is prefixed with the driver name, and followed
	 * by a newline.
	 */
	void debug(const char *fmt, ...);

private:
	/*
	 * disable copy construction for this and all subclasses
	 */
	Device(const Device &);
	/*
	 * disable assignment for this and all subclasses
	 */
	Device &operator =(const Device &);

protected:
	const char* _name;			///< device name (weak).
	union DeviceId _device_id;	///< device identifier information
	bool _debug_enabled;		///< if true, debug messages are printed

private:
	int _irq;
	bool _irq_attached;
	sem_t _lock;
};

} /* namespace device */

#endif /* FIRMWARE_SRC_DRIVERS_DEVICE_DEVICE_H_ */
