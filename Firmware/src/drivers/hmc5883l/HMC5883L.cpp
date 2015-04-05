/*
 * HMC5883L.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: Lucifer
 */

#include "HMC5883L.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <wchar.h>	// wint_t
#include <math.h>
#include <float.h>
#include <stdlib.h>

#include "../drv_device.h"
#include "lib/system/err.h"

#define HMC5883L_ADDRESS		0x1e

/* Max measurement rate is 160Hz, however with 160 it will be set to 166 Hz, therefore workaround using 150 */
#define HMC5883_CONVERSION_INTERVAL	(1000000 / 150)	/* microseconds */

#define ADDR_CONF_A			0x00
#define ADDR_CONF_B			0x01
#define ADDR_MODE				0x02
#define ADDR_DATA_OUT_X_MSB	0x03
#define ADDR_DATA_OUT_X_LSB	0x04
#define ADDR_DATA_OUT_Z_MSB	0x05
#define ADDR_DATA_OUT_Z_LSB	0x06
#define ADDR_DATA_OUT_Y_MSB	0x07
#define ADDR_DATA_OUT_Y_LSB	0x08
#define ADDR_STATUS			0x09
#define ADDR_ID_A				0x0a
#define ADDR_ID_B				0x0b
#define ADDR_ID_C				0x0c

/* modes not changeable outside of driver */
#define HMC5883L_MODE_NORMAL		(0 << 0)  /* default */
#define HMC5883L_MODE_POSITIVE_BIAS	(1 << 0)  /* positive bias */
#define HMC5883L_MODE_NEGATIVE_BIAS	(1 << 1)  /* negative bias */

#define HMC5883L_AVERAGING_1		(0 << 5) /* conf a register */
#define HMC5883L_AVERAGING_2		(1 << 5)
#define HMC5883L_AVERAGING_4		(2 << 5)
#define HMC5883L_AVERAGING_8		(3 << 5)

#define MODE_REG_CONTINOUS_MODE		(0 << 0)
#define MODE_REG_SINGLE_MODE		(1 << 0) /* default */

#define STATUS_REG_DATA_OUT_LOCK	(1 << 1) /* page 16: set if data is only partially read, read device to reset */
#define STATUS_REG_DATA_READY		(1 << 0) /* page 16: set if all axes have valid measurements */

#define ID_A_WHO_AM_I			'H'
#define ID_B_WHO_AM_I			'4'
#define ID_C_WHO_AM_I			'3'

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
#undef ERROR
#endif
#define ERROR (-1)

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

////////////////////////////////////////////////////////////////////////////////

namespace device {

HMC5883L::HMC5883L(int bus, const char* path) :
		I2C("HMC5883", path, bus, HMC5883L_ADDRESS, 400000) {
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_HMC5883;

	_measure_ticks = 0;
	_reports = nullptr;

	_range_scale = 0; /* default range scale from counts to gauss */
	_range_bits = 0;
	_range_ga = 1.3f;
	_conf_reg = 0;
	_collect_phase = false;
	_class_instance = -1;

	_sensor_ok = false;
	_calibrated = false;

	// allocate perf counters
	_sample_perf = perf_alloc(PC_ELAPSED, "hmc5883_read");
	_comms_errors = perf_alloc(PC_COUNT, "hmc5883_comms_errors");
	_buffer_overflows = perf_alloc(PC_COUNT, "hmc5883_buffer_overflows");
	_range_errors = perf_alloc(PC_COUNT, "hmc5883_range_errors");
	_conf_errors = perf_alloc(PC_COUNT, "hmc5883_conf_errors");

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

HMC5883L::~HMC5883L() {
	/* make sure we are truly inactive */
	stop();

	if (_reports != nullptr)
		delete _reports;

	if (_class_instance != -1)
		unregister_class_devname(MAG_DEVICE_PATH, _class_instance);

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
	perf_free(_range_errors);
	perf_free(_conf_errors);
}

int HMC5883L::init() {
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_reports = new RingBuffer(2, sizeof(mag_report));
	if (_reports == nullptr)
		goto out;

	/* reset the device configuration */
	reset();

	_class_instance = register_class_devname(MAG_DEVICE_PATH);

	ret = OK;
	/* sensor is ok, but not calibrated */
	_sensor_ok = true;
out:
	return ret;
}

ssize_t HMC5883L::read(struct file *filp, char *buffer, size_t buflen) {
	unsigned int count = buflen / sizeof(struct mag_report);
	struct mag_report *mag_buf = reinterpret_cast<struct mag_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(mag_buf)) {
				ret += sizeof(struct mag_report);
				mag_buf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep (HMC5883_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		if (_reports->get(mag_buf)) {
			ret = sizeof(struct mag_report);
		}
	} while (0);

	return ret;
}

int HMC5883L::ioctl(struct file *filp, int cmd, unsigned long arg) {
	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
		switch (arg) {

		/* switching to manual polling */
		case SENSOR_POLLRATE_MANUAL:
			stop();
			_measure_ticks = 0;
			return OK;

			/* external signalling (DRDY) not supported */
		case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
		case 0:
			return -EINVAL;

			/* set default/max polling rate */
		case SENSOR_POLLRATE_MAX:
		case SENSOR_POLLRATE_DEFAULT: {
			/* do we need to start internal polling? */
			bool want_start = (_measure_ticks == 0);

			/* set interval for next measurement to minimum legal value */
			_measure_ticks = USEC2TICK(HMC5883_CONVERSION_INTERVAL);

			/* if we need to start the poll state machine, do it */
			if (want_start)
				start();

			return OK;
		}

			/* adjust to a legal polling interval in Hz */
		default: {
			/* do we need to start internal polling? */
			bool want_start = (_measure_ticks == 0);

			/* convert hz to tick interval via microseconds */
			unsigned ticks = USEC2TICK(1000000 / arg);

			/* check against maximum rate */
			if (ticks < USEC2TICK(HMC5883_CONVERSION_INTERVAL))
				return -EINVAL;

			/* update interval for next measurement */
			_measure_ticks = ticks;

			/* if we need to start the poll state machine, do it */
			if (want_start)
				start();

			return OK;
		}
		}
		ASSERT(0);	// NOTE: should never touches here.
		break;
	}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0)
			return SENSOR_POLLRATE_MANUAL;

		return 1000000 / TICK2USEC(_measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
		/* lower bound is mandatory, upper bound is a sanity check */
		if ((arg < 1) || (arg > 100))
			return -EINVAL;

		irqstate_t flags = irqsave();
		if (!_reports->resize(arg)) {
			irqrestore(flags);
			return -ENOMEM;
		}
		irqrestore(flags);

		return OK;
	}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		return reset();

	case MAGIOCSSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return ioctl(filp, SENSORIOCSPOLLRATE, arg);

	case MAGIOCGSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return 1000000 / TICK2USEC(_measure_ticks);

	case MAGIOCSRANGE:
		return set_range(arg);

	case MAGIOCGRANGE:
		return _range_ga;

	case MAGIOCSLOWPASS:
	case MAGIOCGLOWPASS:
		/* not supported, no internal filtering */
		return -EINVAL;

	case MAGIOCSSCALE:
		/* set new scale factors */
		memcpy(&_scale, (mag_scale *) arg, sizeof(_scale));
		/* check calibration, but not actually return an error */
		(void) check_calibration();
		return 0;

	case MAGIOCGSCALE:
		/* copy out scale factors */
		memcpy((mag_scale *) arg, &_scale, sizeof(_scale));
		return 0;

	case MAGIOCCALIBRATE:
		return calibrate(filp, arg);

	case MAGIOCEXSTRAP:
		return set_excitement(arg);

	case MAGIOCSELFTEST:
		return check_calibration();

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

void HMC5883L::start() {
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t) & HMC5883L::cycle_trampoline, this, 1);
}

void HMC5883L::stop() {
	work_cancel(HPWORK, &_work);
}

int HMC5883L::reset() {
	/* set range */
	return set_range(_range_ga);
}

int HMC5883L::set_range(unsigned int range) {
	if (range < 1) {
		_range_bits = 0x00;
		_range_scale = 1.0f / 1370.0f;
		_range_ga = 0.88f;

	} else if (range <= 1) {
		_range_bits = 0x01;
		_range_scale = 1.0f / 1090.0f;
		_range_ga = 1.3f;

	} else if (range <= 2) {
		_range_bits = 0x02;
		_range_scale = 1.0f / 820.0f;
		_range_ga = 1.9f;

	} else if (range <= 3) {
		_range_bits = 0x03;
		_range_scale = 1.0f / 660.0f;
		_range_ga = 2.5f;

	} else if (range <= 4) {
		_range_bits = 0x04;
		_range_scale = 1.0f / 440.0f;
		_range_ga = 4.0f;

	} else if (range <= 4.7f) {
		_range_bits = 0x05;
		_range_scale = 1.0f / 390.0f;
		_range_ga = 4.7f;

	} else if (range <= 5.6f) {
		_range_bits = 0x06;
		_range_scale = 1.0f / 330.0f;
		_range_ga = 5.6f;

	} else {
		_range_bits = 0x07;
		_range_scale = 1.0f / 230.0f;
		_range_ga = 8.1f;
	}

	int ret;

	/*
	 * Send the command to set the range
	 */
	ret = write_reg(ADDR_CONF_B, (_range_bits << 5));
	if (OK != ret){}
	//	perf_count (_comms_errors);

	uint8_t range_bits_in;
	ret = read_reg(ADDR_CONF_B, range_bits_in);
	//if (OK != ret)
	//	perf_count (_comms_errors);

	return !(range_bits_in == (_range_bits << 5));
}

int HMC5883L::set_excitement(unsigned int enable) {
	int ret;
	/* arm the excitement strap */
	ret = read_reg(ADDR_CONF_A, _conf_reg);
	if (OK != ret){}
	//	perf_count (_comms_errors);

	if (((int) enable) < 0) {
		_conf_reg |= 0x01;
	} else if (enable > 0) {
		_conf_reg |= 0x02;
	} else {
		_conf_reg &= ~0x03;
	}

	// log("set_excitement enable=%d regA=0x%x\n", (int)enable, (unsigned int)_conf_reg);

	ret = write_reg(ADDR_CONF_A, _conf_reg);
	//if (OK != ret)
	//	perf_count (_comms_errors);

	uint8_t conf_reg_ret;
	read_reg(ADDR_CONF_A, conf_reg_ret);

	//print_info();

	return !(_conf_reg == conf_reg_ret);
}

void HMC5883L::cycle_trampoline(void *arg) {
	HMC5883L *dev = (HMC5883L *) arg;

	dev->cycle();
}

void HMC5883L::cycle() {
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			log("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(HMC5883_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK, &_work, (worker_t) & HMC5883L::cycle_trampoline,
					this,
					_measure_ticks - USEC2TICK(HMC5883_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure())
		log("measure error");

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK, &_work, (worker_t) & HMC5883L::cycle_trampoline, this,
			USEC2TICK(HMC5883_CONVERSION_INTERVAL));
}

int HMC5883L::calibrate(struct file *filp, unsigned int enable) {
	struct mag_report report;
	ssize_t sz;
	int ret = 1;
	uint8_t good_count = 0;

	// XXX do something smarter here
	int fd = (int) enable;

	struct mag_scale mscale_previous = { 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, };

	struct mag_scale mscale_null = { 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, };

	float sum_excited[3] = { 0.0f, 0.0f, 0.0f };

	/* expected axis scaling. The datasheet says that 766 will
	 * be places in the X and Y axes and 713 in the Z
	 * axis. Experiments show that in fact 766 is placed in X,
	 * and 713 in Y and Z. This is relative to a base of 660
	 * LSM/Ga, giving 1.16 and 1.08 */
	float expected_cal[3] = { 1.16f, 1.08f, 1.08f };

	warnx("starting mag scale calibration");

	/* start the sensor polling at 50 Hz */
	if (OK != ioctl(filp, SENSORIOCSPOLLRATE, 50)) {
		warn("failed to set 2Hz poll rate");
		ret = 1;
		goto out;
	}

	/* Set to 2.5 Gauss. We ask for 3 to get the right part of
	 * the chained if statement above. */
	if (OK != ioctl(filp, MAGIOCSRANGE, 3)) {
		warnx("failed to set 2.5 Ga range");
		ret = 1;
		goto out;
	}

	if (OK != ioctl(filp, MAGIOCEXSTRAP, 1)) {
		warnx("failed to enable sensor calibration mode");
		ret = 1;
		goto out;
	}

	if (OK != ioctl(filp, MAGIOCGSCALE, (unsigned long) &mscale_previous)) {
		warn("WARNING: failed to get scale / offsets for mag");
		ret = 1;
		goto out;
	}

	if (OK != ioctl(filp, MAGIOCSSCALE, (unsigned long) &mscale_null)) {
		warn("WARNING: failed to set null scale / offsets for mag");
		ret = 1;
		goto out;
	}

	// discard 10 samples to let the sensor settle
	for (uint8_t i = 0; i < 10; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("timed out waiting for sensor data");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("periodic read failed");
			ret = -EIO;
			goto out;
		}
	}

	/* read the sensor up to 50x, stopping when we have 10 good values */
	for (uint8_t i = 0; i < 50 && good_count < 10; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("timed out waiting for sensor data");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("periodic read failed");
			ret = -EIO;
			goto out;
		}
		float cal[3] = { fabsf(expected_cal[0] / report.x), fabsf(
				expected_cal[1] / report.y), fabsf(expected_cal[2] / report.z) };

		if (cal[0] > 0.7f && cal[0] < 1.35f && cal[1] > 0.7f && cal[1] < 1.35f
				&& cal[2] > 0.7f && cal[2] < 1.35f) {
			good_count++;
			sum_excited[0] += cal[0];
			sum_excited[1] += cal[1];
			sum_excited[2] += cal[2];
		}

		//warnx("periodic read %u", i);
		//warnx("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
		//warnx("cal: %.6f  %.6f  %.6f", (double)cal[0], (double)cal[1], (double)cal[2]);
	}

	if (good_count < 5) {
		warn("failed calibration");
		ret = -EIO;
		goto out;
	}

	float scaling[3];

	scaling[0] = sum_excited[0] / good_count;
	scaling[1] = sum_excited[1] / good_count;
	scaling[2] = sum_excited[2] / good_count;

	warnx("axes scaling: %.6f  %.6f  %.6f", (double) scaling[0],
			(double) scaling[1], (double) scaling[2]);

	/* set scaling in device */
	mscale_previous.x_scale = scaling[0];
	mscale_previous.y_scale = scaling[1];
	mscale_previous.z_scale = scaling[2];

	ret = OK;

out:
	if (OK != ioctl(filp, MAGIOCSSCALE,
					(unsigned long) &mscale_previous)) {
		warn("WARNING: failed to set new scale / offsets for mag");
	}

	/* set back to normal mode */
	/* Set to 1.1 Gauss */
	if (OK != ::ioctl(fd, MAGIOCSRANGE, 1)) {
		warnx("failed to set 1.1 Ga range");
	}

	if (OK != ::ioctl(fd, MAGIOCEXSTRAP, 0)) {
		warnx("failed to disable sensor calibration mode");
	}

	if (ret == OK) {
		if (!check_scale()) {
			warnx("mag scale calibration successfully finished.");
		} else {
			warnx("mag scale calibration finished with invalid results.");
			ret = ERROR;
		}

	} else {
		warnx("mag scale calibration failed.");
	}

	return ret;
}

int HMC5883L::measure() {
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	ret = write_reg(ADDR_MODE, MODE_REG_SINGLE_MODE);
	if (OK != ret)
		perf_count (_comms_errors);

	return ret;
}

int HMC5883L::collect() {
#pragma pack(push, 1)
	struct { /* status register and data as read back from the device */
		uint8_t x[2];
		uint8_t z[2];
		uint8_t y[2];
	} hmc_report;
#pragma pack(pop)
	struct {
		int16_t x, y, z;
	} report;
	int ret = -EIO;
	uint8_t cmd;
	uint8_t check_counter;

	perf_begin (_sample_perf);
	struct mag_report new_report;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	new_report.timestamp = perf_absolute_usec();
	new_report.error_count = perf_event_count(_comms_errors);

	/*
	 * @note  We could read the status register here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device */
	cmd = ADDR_DATA_OUT_X_MSB;
	ret = transfer(&cmd, 1, (uint8_t *) &hmc_report, sizeof(hmc_report));

	if (ret != OK) {
		perf_count (_comms_errors);
		debug("data/status read error");
		goto out;
	}

	/* swap the data we just received */
	report.x = (((int16_t) hmc_report.x[0]) << 8) + hmc_report.x[1];
	report.y = (((int16_t) hmc_report.y[0]) << 8) + hmc_report.y[1];
	report.z = (((int16_t) hmc_report.z[0]) << 8) + hmc_report.z[1];

	/*
	 * If any of the values are -4096, there was an internal math error in the sensor.
	 * Generalise this to a simple range check that will also catch some bit errors.
	 */
	if ((abs(report.x) > 2048) || (abs(report.y) > 2048)
			|| (abs(report.z) > 2048)) {
		perf_count (_comms_errors);
		goto out;
	}

	/*
	 * RAW outputs
	 *
	 * to align the sensor axes with the board, x and y need to be flipped
	 * and y needs to be negated
	 */
	new_report.x_raw = report.y;
	new_report.y_raw = -report.x;
	/* z remains z */
	new_report.z_raw = report.z;

	/* the standard external mag by 3DR has x pointing to the
	 * right, y pointing backwards, and z down, therefore switch x
	 * and y and invert y */
	new_report.x = ((-report.y * _range_scale) - _scale.x_offset)
			* _scale.x_scale;
	/* flip axes and negate value for y */
	new_report.y = ((report.x * _range_scale) - _scale.y_offset)
			* _scale.y_scale;
	/* z remains z */
	new_report.z = ((report.z * _range_scale) - _scale.z_offset)
			* _scale.z_scale;

	// apply user specified rotation
	// rotate_3f(_rotation, new_report.x, new_report.y, new_report.z);
#if 0
	if (_class_instance == CLASS_DEVICE_PRIMARY && !(_pub_blocked)) {

		if (_mag_topic != -1) {
			/* publish it */
			orb_publish(ORB_ID(sensor_mag), _mag_topic, &new_report);
		} else {
			_mag_topic = orb_advertise(ORB_ID(sensor_mag), &new_report);

			if (_mag_topic < 0)
				debug("failed to create sensor_mag publication");
		}
	}
#endif

	_last_report = new_report;

	/* post a report to the ring */
	if (_reports->force(&new_report)) {
		perf_count (_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/*
	 periodically check the range register and configuration
	 registers. With a bad I2C cable it is possible for the
	 registers to become corrupt, leading to bad readings. It
	 doesn't happen often, but given the poor cables some
	 vehicles have it is worth checking for.
	 */
	check_counter = perf_event_count(_sample_perf) % 256;
	if (check_counter == 0) {
		check_range();
	}
	if (check_counter == 128) {
		check_conf();
	}

	ret = OK;

out:
	perf_end(_sample_perf);
	return ret;
}

void HMC5883L::check_conf() {
	int ret;

	uint8_t conf_reg_in;
	ret = read_reg(ADDR_CONF_A, conf_reg_in);
	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}
	if (conf_reg_in != _conf_reg) {
		perf_count(_conf_errors);
		ret = write_reg(ADDR_CONF_A, _conf_reg);
		if (OK != ret)
			perf_count(_comms_errors);
	}
}

int HMC5883L::check_offset() {
	bool offset_valid;

	if ((-2.0f * FLT_EPSILON < _scale.x_offset
			&& _scale.x_offset < 2.0f * FLT_EPSILON)
			&& (-2.0f * FLT_EPSILON < _scale.y_offset
					&& _scale.y_offset < 2.0f * FLT_EPSILON)
			&& (-2.0f * FLT_EPSILON < _scale.z_offset
					&& _scale.z_offset < 2.0f * FLT_EPSILON)) {
		/* offset is zero */
		offset_valid = false;
	} else {
		offset_valid = true;
	}

	/* return 0 if calibrated, 1 else */
	return !offset_valid;
}

void HMC5883L::check_range() {
	int ret;

	uint8_t range_bits_in;
	ret = read_reg(ADDR_CONF_B, range_bits_in);
	if (OK != ret) {
		perf_count (_comms_errors);
		return;
	}
	if (range_bits_in != (_range_bits << 5)) {
		perf_count (_range_errors);
		ret = write_reg(ADDR_CONF_B, (_range_bits << 5));
		if (OK != ret)
			perf_count (_comms_errors);
	}
}

int HMC5883L::check_calibration() {
	bool offset_valid = (check_offset() == OK);
	bool scale_valid = (check_scale() == OK);

	if (_calibrated != (offset_valid && scale_valid)) {
		warnx("mag cal status changed %s%s",
				(scale_valid) ? "" : "scale invalid ",
				(offset_valid) ? "" : "offset invalid");
		_calibrated = (offset_valid && scale_valid);

		// XXX Change advertisement
#if 0
		/* notify about state change */
		struct subsystem_info_s info = {
				true,
				true, _calibrated, SUBSYSTEM_TYPE_MAG
		};
		static orb_advert_t pub = -1;

		if (!(_pub_blocked)) {
			if (pub > 0) {
				orb_publish(ORB_ID(subsystem_info), pub, &info);
			} else {
				pub = orb_advertise(ORB_ID(subsystem_info), &info);
			}
		}
#endif
	}

	/* return 0 if calibrated, 1 else */
	return (!_calibrated);
}

int HMC5883L::check_scale() {
	bool scale_valid;

	if ((-FLT_EPSILON + 1.0f < _scale.x_scale
			&& _scale.x_scale < FLT_EPSILON + 1.0f)
			&& (-FLT_EPSILON + 1.0f < _scale.y_scale
					&& _scale.y_scale < FLT_EPSILON + 1.0f)
			&& (-FLT_EPSILON + 1.0f < _scale.z_scale
					&& _scale.z_scale < FLT_EPSILON + 1.0f)) {
		/* scale is one */
		scale_valid = false;
	} else {
		scale_valid = true;
	}

	/* return 0 if calibrated, 1 else */
	return !scale_valid;
}

int HMC5883L::write_reg(uint8_t reg, uint8_t val) {
	uint8_t cmd[] = { reg, val };

	return transfer(&cmd[0], 2, nullptr, 0);
}

int HMC5883L::read_reg(uint8_t reg, uint8_t &val) {
	return transfer(&reg, 1, &val, 1);
}

} /* namespace device */
