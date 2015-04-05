/*
 * Device.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: Lucifer
 */

#include <nuttx/arch.h>
#include <stdio.h>
#include <sys/types.h>
#include "../drv_device.h"
#include "Device.h"

namespace device {

////////////////////////////////////////////////////////////////////////////////

/*
 * Interrupt dispatch table entry.
 */
struct irq_entry {
	int irq;
	Device *owner;
};

/*
 * size of the interrupt dispatch table
 */
static const unsigned __irq_nentries = 8;
/*
 * interrupt dispatch table (XXX should be a vector)
 */
static irq_entry __irq_entries[__irq_nentries];

/*
 * Handle an interrupt.
 *
 * @param irq
 * 		The interrupt being invoked.
 * @param context
 * 		The interrupt register context.
 * @return Always returns OK.
 */
static int interrupt(int irq, void *context) {
	for (unsigned i = 0; i < __irq_nentries; i++) {
		if (__irq_entries[i].irq == irq) {
			__irq_entries[i].owner->interrupt(context);
			break;
		}
	}

	return OK;
}

/*
 * Register an interrupt to a specific device.
 *
 * @param irq
 * 		The interrupt number to register.
 * @param owner
 * 		The device receiving the interrupt.
 * @return OK if the interrupt was registered.
 */
static int register_interrupt(int irq, Device *owner) {
	int ret = -ENOMEM;

	// look for a slot where we can register the interrupt
	for (unsigned i = 0; i < __irq_nentries; i++) {
		if (__irq_entries[i].irq == 0) {

			// great, we could put it here; try attaching it
			ret = irq_attach(irq, &interrupt);

			if (ret == OK) {
				__irq_entries[i].irq = irq;
				__irq_entries[i].owner = owner;
			}

			break;
		}
	}

	return ret;
}

/*
 * Unregister an interrupt.
 *
 * @param irq
 * 		The previously-registered interrupt to be de-registered.
 */
static void unregister_interrupt(int irq) {
	for (unsigned i = 0; i < __irq_nentries; i++) {
		if (__irq_entries[i].irq == irq) {
			__irq_entries[i].irq = 0;
			__irq_entries[i].owner = nullptr;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

Device::Device(const char *name, int irq/* = 0*/) {
	_debug_enabled = false;
	_name = name;
	_irq = irq;
	_irq_attached = false;

	sem_init(&_lock, 0, 1);

	// setup a default device ID. When bus_type is UNKNOWN the other fields
	// are invalid
	_device_id.devid = 0;
	_device_id.devid_s.bus_type = DeviceBusType_UNKNOWN;
	_device_id.devid_s.bus = 0;
	_device_id.devid_s.address = 0;
	_device_id.devid_s.devtype = 0;
}

Device::~Device() {
	sem_destroy(&_lock);

	if (_irq_attached)
		unregister_interrupt(_irq);
}

int Device::init() {
	int ret = OK;

	// If assigned an interrupt, connect it
	if (_irq) {
		/* ensure it's disabled */
		up_disable_irq(_irq);

		/* register */
		ret = register_interrupt(_irq, this);

		if (ret == OK)
			_irq_attached = true;
	}

	return ret;
}

void Device::interrupt_enable() {
	if (_irq_attached)
		up_enable_irq(_irq);
}

void Device::interrupt_disable() {
	if (_irq_attached)
		up_disable_irq(_irq);
}

void Device::interrupt(void *ctx) {
	// default action is to disable the interrupt so we don't get called again
	interrupt_disable();
}

int Device::read(unsigned int address, void *data, unsigned int count) {
	return -ENODEV;
}

int Device::write(unsigned int address, void *data, unsigned int count) {
	return -ENODEV;
}

int Device::ioctl(unsigned int operation, unsigned int &arg) {
	switch (operation) {
	case DEVIOCGDEVICEID:
		return (int) _device_id.devid;
	}
	return -ENODEV;
}

void Device::log(const char *fmt, ...) {
	va_list ap;

	printf("[%s] ", _name);
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	printf("\n");
	fflush (stdout);
}

void Device::debug(const char *fmt, ...) {
	va_list ap;

	if (_debug_enabled) {
		printf("<%s> ", _name);
		va_start(ap, fmt);
		vprintf(fmt, ap);
		va_end(ap);
		printf("\n");
		fflush (stdout);
	}
}

} /* namespace device */
