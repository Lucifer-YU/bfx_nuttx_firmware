/*
 * CharDevice.h
 *
 *  Created on: Mar 30, 2015
 *      Author: Lucifer
 */

#ifndef FIRMWARE_SRC_DRIVERS_DEVICE_CHARDEVICE_H_
#define FIRMWARE_SRC_DRIVERS_DEVICE_CHARDEVICE_H_

#include <nuttx/fs/fs.h>

#include "Device.h"

namespace device {

/**
 * Abstract class for any character device
 */
class __EXPORT CharDevice : public Device {
public:
	/**
	 * Constructor
	 *
	 * @param name
	 * 		Driver name
	 * @param devname
	 * 		Device node name
	 * @param irq
	 * 		Interrupt assigned to the device
	 */
	CharDevice(const char *name, const char *devname, int irq = 0);

	/**
	 * Destructor
	 */
	virtual ~CharDevice();

	virtual int init();

	/**
	 * Handle an open of the device.
	 *
	 * This function is called for every open of the device. The default
	 * implementation maintains _open_count and always returns OK.
	 *
	 * @param filp
	 * 		Pointer to the NuttX file structure.
	 * @return OK if the open is allowed, -errno otherwise.
	 */
	virtual int open(struct file *filp);

	/**
	 * Handle a close of the device.
	 *
	 * This function is called for every close of the device. The default
	 * implementation maintains _open_count and returns OK as long as it is not zero.
	 *
	 * @param filp
	 * 		Pointer to the NuttX file structure.
	 * @return OK if the close was successful, -errno otherwise.
	 */
	virtual int close(struct file *filp);

	/**
	 * Perform a read from the device.
	 *
	 * The default implementation returns -ENOSYS.
	 *
	 * @param filp
	 * 		Pointer to the NuttX file structure.
	 * @param buffer
	 * 		Pointer to the buffer into which data should be placed.
	 * @param buflen
	 * 		The number of bytes to be read.
	 * @return The number of bytes read or -errno otherwise.
	 */
	virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);

	/**
	 * Perform a write to the device.
	 *
	 * The default implementation returns -ENOSYS.
	 *
	 * @param filp
	 * 		Pointer to the NuttX file structure.
	 * @param buffer
	 * 		Pointer to the buffer from which data should be read.
	 * @param buflen
	 * 		The number of bytes to be written.
	 * @return The number of bytes written or -errno otherwise.
	 */
	virtual ssize_t write(struct file *filp, const char *buffer, size_t buflen);

	/**
	 * Perform a logical seek operation on the device.
	 *
	 * The default implementation returns -ENOSYS.
	 *
	 * @param filp
	 * 		Pointer to the NuttX file structure.
	 * @param offset
	 * 		The new file position relative to whence.
	 * @param whence
	 * 		SEEK_OFS, SEEK_CUR or SEEK_END.
	 * @return The previous offset, or -errno otherwise.
	 */
	virtual off_t seek(struct file *filp, off_t offset, int whence);

	/**
	 * Perform an ioctl operation on the device.
	 *
	 * The default implementation handles DIOC_GETPRIV, and otherwise
	 * returns -ENOTTY. Subclasses should call the default implementation
	 * for any command they do not handle themselves.
	 *
	 * @param filp
	 * 		Pointer to the NuttX file structure.
	 * @param cmd
	 * 		The ioctl command value.
	 * @param arg
	 * 		The ioctl argument value.
	 * @return OK on success, or -errno otherwise.
	 */
	virtual int ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Perform a poll setup/teardown operation.
	 *
	 * This is handled internally and should not normally be overridden.
	 *
	 * @param filp
	 * 		Pointer to the NuttX file structure.
	 * @param fds
	 * 		Poll descriptor being waited on.
	 * @param arg
	 * 		True if this is establishing a request, false if it is being torn
	 * 		down.
	 * @return OK on success, or -errno otherwise.
	 */
	virtual int poll(struct file *filp, struct pollfd *fds, bool setup);

	/**
	 * Test whether the device is currently open.
	 *
	 * This can be used to avoid tearing down a device that is still active.
	 * Note - not virtual, cannot be overridden by a subclass.
	 *
	 * @return True if the device is currently open.
	 */
	bool is_open() const {
		return _open_count > 0;
	}

protected:
	/**
	 * Register a class device name, automatically adding device
	 * class instance suffix if need be.
	 *
	 * @param class_devname
	 * 		Device class name
	 * @return class_instamce Class instance created, or -errno on failure
	 */
	virtual int register_class_devname(const char *class_devname);

	/**
	 * Register a class device name, automatically adding device
	 * class instance suffix if need be.
	 *
	 * @param class_devname
	 * 		Device class name
	 * @param class_instance
	 * 		Device class instance from register_class_devname()
	 * @return OK on success, -errno otherwise
	 */
	virtual int unregister_class_devname(const char *class_devname,
			unsigned class_instance);

	/**
	 * Notification of the first open.
	 *
	 * This function is called when the device open count transitions from zero
	 * to one.  The driver lock is held for the duration of the call.
	 *
	 * The default implementation returns OK.
	 *
	 * @param filp
	 * 		Pointer to the NuttX file structure.
	 * @return OK if the open should proceed, -errno otherwise.
	 */
	virtual int open_first(struct file *filp);

	/**
	 * Notification of the last close.
	 *
	 * This function is called when the device open count transitions from
	 * one to zero.  The driver lock is held for the duration of the call.
	 *
	 * The default implementation returns OK.
	 *
	 * @param filp
	 * 		Pointer to the NuttX file structure.
	 * @return OK if the open should return OK, -errno otherwise.
	 */
	virtual int close_last(struct file *filp);

	/**
	 * Check the current state of the device for poll events from the
	 * perspective of the file.
	 *
	 * This function is called by the default poll() implementation when
	 * a poll is set up to determine whether the poll should return immediately.
	 *
	 * The default implementation returns no events.
	 *
	 * @param filp
	 * 		The file that's interested.
	 * @return The current set of poll events.
	 */
	virtual pollevent_t poll_state(struct file *filp);

	/**
	 * Report new poll events.
	 *
	 * This function should be called anytime the state of the device changes
	 * in a fashion that might be interesting to a poll waiter.
	 *
	 * @param events
	 * 		The new event(s) being announced.
	 */
	virtual void poll_notify(pollevent_t events);

	/**
	 * Internal implementation of poll_notify.
	 *
	 * @param fds
	 * 		A poll waiter to notify.
	 * @param events
	 * 		The event(s) to send to the waiter.
	 */
	virtual void poll_notify_one(struct pollfd *fds, pollevent_t events);

	/**
	 * Get the device name.
	 *
	 * @return The file system string of the device handle
	 */
	const char* get_devname() {
		return _devname;
	}

private:
	/* do not allow copying this class */
	CharDevice(const CharDevice&);
	CharDevice operator=(const CharDevice&);

	/*
	 * Store a pollwaiter in a slot where we can find it later.
	 * Expands the pollset as required.  Must be called with the driver locked.
	 *
	 * @return OK, or -errno on error.
	 */
	int add_poll_waiter(struct pollfd *fds);

	/*
	 * Remove a poll waiter.
	 *
	 * @return OK, or -errno on error.
	 */
	int remove_poll_waiter(struct pollfd *fds);

protected:
	/**
	 * Pointer to the default CharDevice file operations table; useful for
	 * registering clone devices etc.
	 */
	static const struct file_operations _fops;

	bool _pub_blocked;			///< true if publishing should be blocked

private:
	const char *_devname;		// device node name
	bool _registered;			// true if device name was registered
	unsigned _open_count;		// number of successful opens

	static const unsigned int _max_pollwaiters = 8;
	struct pollfd *_pollset[_max_pollwaiters];
};

} /* namespace device */

#endif /* FIRMWARE_SRC_DRIVERS_DEVICE_CHARDEVICE_H_ */
