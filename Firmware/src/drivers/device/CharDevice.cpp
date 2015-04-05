/*
 * CharDevice1.cpp
 *
 *  Created on: Mar 30, 2015
 *      Author: Lucifer
 */

#include "CharDevice.h"

#include <stdio.h>
#include <sys/ioctl.h>
#include "../drv_device.h"

namespace device {

#ifdef CONFIG_DISABLE_POLL
# error This requires !CONFIG_DISABLE_POLL.
#endif

////////////////////////////////////////////////////////////////////////////////

/*
 * The standard NuttX operation dispatch table can't call C++ member functions
 * directly, so we have to bounce them through this dispatch table.
 */
static int cdev_open(struct file *filp) {
	CharDevice *cdev = (CharDevice *) (filp->f_inode->i_private);
	return cdev->open(filp);
}

static int cdev_close(struct file *filp) {
	CharDevice *cdev = (CharDevice *) (filp->f_inode->i_private);
	return cdev->close(filp);
}

static ssize_t cdev_read(struct file *filp, char *buffer, size_t buflen) {
	CharDevice *cdev = (CharDevice *) (filp->f_inode->i_private);
	return cdev->read(filp, buffer, buflen);
}

static ssize_t cdev_write(struct file *filp, const char *buffer,
		size_t buflen) {
	CharDevice *cdev = (CharDevice *) (filp->f_inode->i_private);
	return cdev->write(filp, buffer, buflen);
}

static off_t cdev_seek(struct file *filp, off_t offset, int whence) {
	CharDevice *cdev = (CharDevice *) (filp->f_inode->i_private);
	return cdev->seek(filp, offset, whence);
}

static int cdev_ioctl(struct file *filp, int cmd, unsigned long arg) {
	CharDevice *cdev = (CharDevice *) (filp->f_inode->i_private);
	return cdev->ioctl(filp, cmd, arg);
}

static int cdev_poll(struct file *filp, struct pollfd *fds, bool setup) {
	CharDevice *cdev = (CharDevice *) (filp->f_inode->i_private);
	return cdev->poll(filp, fds, setup);
}

////////////////////////////////////////////////////////////////////////////////

/**
 * Character device indirection table.
 *
 * Every CharDevice we register gets the same function table; we use the private
 * data field in the inode to store the instance pointer.
 *
 * Note that we use the GNU extension syntax here because we don't get designated
 * initializers in GCC 4.6.
 */
const struct file_operations CharDevice::_fops = {
		cdev_open,	/* open */
		cdev_close,	/* close */
		cdev_read,	/* read */
		cdev_write,	/* write */
		cdev_seek,	/* seek */
		cdev_ioctl,	/* ioctl */
		cdev_poll,	/* poll */
		nullptr	/* unlink */
};

CharDevice::CharDevice(const char *name, const char *devname, int irq/* = 0*/) :
		Device(name, irq) {
	_pub_blocked = false;
	_devname = devname;
	_registered = false;
	_open_count = 0;

	for (unsigned i = 0; i < _max_pollwaiters; i++)
		_pollset[i] = nullptr;
}

CharDevice::~CharDevice() {
	if (_registered)
		unregister_driver(_devname);
}

int CharDevice::init() {
	// base class initialization first
	int ret = Device::init();
	if (ret == OK) {
		// now register the driver
		if (_devname != nullptr) {
			ret = register_driver(_devname, &_fops, 0666, (void *) this);

			if (ret == OK)
				_registered = true;
		}
	}
	return ret;
}

int CharDevice::open(struct file *filp) {
	int ret = OK;

	lock();
	/* increment the open count */
	_open_count++;

	if (_open_count == 1) {

		/* first-open callback may decline the open */
		ret = open_first(filp);

		if (ret != OK)
			_open_count--;
	}

	unlock();

	return ret;
}

int CharDevice::close(struct file *filp) {
	int ret = OK;

	lock();

	if (_open_count > 0) {
		/* decrement the open count */
		_open_count--;

		/* callback cannot decline the close */
		if (_open_count == 0)
			ret = close_last(filp);

	} else {
		ret = -EBADF;
	}

	unlock();

	return ret;
}

ssize_t CharDevice::read(struct file *filp, char *buffer, size_t buflen) {
	return -ENOSYS;
}

ssize_t CharDevice::write(struct file *filp, const char *buffer, size_t buflen) {
	return -ENOSYS;
}

off_t CharDevice::seek(struct file *filp, off_t offset, int whence) {
	return -ENOSYS;
}

int CharDevice::ioctl(struct file *filp, int cmd, unsigned long arg) {
	switch (cmd) {

	/* fetch a pointer to the driver's private data */
	case DIOC_GETPRIV:
		*(void **) (uintptr_t) arg = (void *) this;
		return OK;
		break;
	case DEVIOCSPUBBLOCK:
		_pub_blocked = (arg != 0);
		return OK;
		break;
	case DEVIOCGPUBBLOCK:
		return _pub_blocked;
		break;
	}

	/* try the superclass. The different ioctl() function form
	 * means we need to copy arg */
	unsigned int arg2 = (unsigned int) arg;
	int ret = Device::ioctl(cmd, arg2);
	if (ret != -ENODEV)
		return ret;

	return -ENOTTY;
}

int CharDevice::poll(struct file *filp, struct pollfd *fds, bool setup) {
	int ret = OK;

	/*
	 * Lock against pollnotify() (and possibly other callers)
	 */
	lock();

	if (setup) {
		/*
		 * Save the file pointer in the pollfd for the subclass'
		 * benefit.
		 */
		fds->priv = (void *) filp;

		/*
		 * Handle setup requests.
		 */
		ret = add_poll_waiter(fds);

		if (ret == OK) {

			/*
			 * Check to see whether we should send a poll notification
			 * immediately.
			 */
			fds->revents |= fds->events & poll_state(filp);

			/* yes? post the notification */
			if (fds->revents != 0)
				sem_post(fds->sem);
		}

	} else {
		/*
		 * Handle a teardown request.
		 */
		ret = remove_poll_waiter(fds);
	}

	unlock();

	return ret;
}

int CharDevice::register_class_devname(const char *class_devname) {
	if (class_devname == nullptr) {
		return -EINVAL;
	}
	int class_instance = 0;
	int ret = -ENOSPC;
	while (class_instance < 4) {
		if (class_instance == 0) {
			ret = register_driver(class_devname, &_fops, 0666, (void *) this);
			if (ret == OK)
				break;
		} else {
			char name[32];
			snprintf(name, sizeof(name), "%s%d", class_devname, class_instance);
			ret = register_driver(name, &_fops, 0666, (void *) this);
			if (ret == OK)
				break;
		}
		class_instance++;
	}
	if (class_instance == 4)
		return ret;
	return class_instance;
}

int CharDevice::unregister_class_devname(const char *class_devname,
		unsigned class_instance) {
	if (class_instance > 0) {
		char name[32];
		snprintf(name, sizeof(name), "%s%u", class_devname, class_instance);
		return unregister_driver(name);
	}
	return unregister_driver(class_devname);
}

int CharDevice::open_first(struct file *filp) {
	return OK;
}

int CharDevice::close_last(struct file *filp) {
	return OK;
}

pollevent_t CharDevice::poll_state(struct file *filp) {
	/* by default, no poll events to report */
	return 0;
}

void CharDevice::poll_notify(pollevent_t events) {
	/* lock against poll() as well as other wakeups */
	irqstate_t state = irqsave();

	for (unsigned int i = 0; i < _max_pollwaiters; i++)
		if (nullptr != _pollset[i])
			poll_notify_one(_pollset[i], events);

	irqrestore(state);
}

void CharDevice::poll_notify_one(struct pollfd *fds, pollevent_t events) {
	/* update the reported event set */
	fds->revents |= fds->events & events;

	/* if the state is now interesting, wake the waiter if it's still asleep */
	/* XXX semcount check here is a vile hack; counting semphores should not be abused as cvars */
	if ((fds->revents != 0) && (fds->sem->semcount <= 0))
		sem_post(fds->sem);
}

int CharDevice::add_poll_waiter(struct pollfd *fds) {
	/*
	 * Look for a free slot.
	 */
	for (unsigned int i = 0; i < _max_pollwaiters; i++) {
		if (nullptr == _pollset[i]) {
			/* save the pollfd */
			_pollset[i] = fds;

			return OK;
		}
	}

	return ENOMEM;
}

int CharDevice::remove_poll_waiter(struct pollfd *fds) {
	for (unsigned int i = 0; i < _max_pollwaiters; i++) {
		if (fds == _pollset[i]) {
			_pollset[i] = nullptr;
			return OK;
		}
	}

	puts("poll: bad fd state");
	return -EINVAL;
}

} /* namespace device */
