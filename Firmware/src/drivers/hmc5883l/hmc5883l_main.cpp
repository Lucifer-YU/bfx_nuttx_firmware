/*
 * hmc5883l_main.cpp
 *
 *  Created on: Mar 30, 2015
 *      Author: Lucifer
 */

#include <nuttx/config.h>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "HMC5883L.h"
#include "../lib/system/err.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
#undef ERROR
#endif
#define ERROR (-1)

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

////////////////////////////////////////////////////////////////////////////////

using namespace device;

#define HMC5883L_DEVICE_PATH	"/dev/hmc5883l"

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int hmc5883l_main(int argc, char *argv[]);

static HMC5883L*__dev_ptr = nullptr;

static int hmc5883l_start() {
	int fd = -1;

	// create the driver
	if (__dev_ptr != nullptr) {
		warnx("already started");
		return OK;	// NOTE: not a error
	}
	__dev_ptr = new HMC5883L(0, HMC5883L_DEVICE_PATH);
	if (__dev_ptr == nullptr) {
		warnx("unable to allocate device instance");
		return ERROR;
	} else if (!__dev_ptr->init()) {
		warnx("failed to initialize device");
		delete __dev_ptr;
		__dev_ptr = nullptr;
		return ERROR;
	}

	// set the poll rate to default, starts automatic data collection
	fd = open(HMC5883L_DEVICE_PATH, O_RDONLY);
	if (fd < 0) {
		warnx("%s open failed", HMC5883L_DEVICE_PATH);
		return ERROR;
	}
	if (0 > ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		close(fd);
		warnx("failed to setup poll rate");
		return ERROR;
	}
	close(fd);

	return 0;
}

static int hmc5883l_calibrate() {
	int fd, ret;

	fd = open(HMC5883L_DEVICE_PATH, O_RDONLY);
	if (fd < 0)
		err(0, "%s open failed, (try 'hmc5883 start' if the driver is not running",
				HMC5883L_DEVICE_PATH);
	if (0 > (ret = ioctl(fd, MAGIOCCALIBRATE, fd))) {
		warnx("failed to enable sensor calibration mode");
	}
	return 0;
}

static void hmc5883l_usage(const char *reason) {
	if (reason)
		warnx("%s\n", reason);

	warnx("usage: hmc5883 [<options>] {start|stop|test|reset|info|calibrate}\n");
	warnx("options:");
	warnx("    -R rotation");
	warnx("    -C calibrate on start");
}

int hmc5883l_main(int argc, char *argv[]) {
	int ch;
	bool calibrate = false;

	if (argc < 1) {
		hmc5883l_usage("missing command");
		exit(1);
	}

	while ((ch = getopt(argc, argv, "XISR:C")) != EOF) {
		switch (ch) {
		case 'R':
			// TODO set rotation
			break;
		case 'C':
			calibrate = true;
			break;
		default:
			hmc5883l_usage("unrecognized option");
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (OK != hmc5883l_start()) {
			errx(1, "start failed");
		}
		if (calibrate) {
			if (OK == hmc5883l_calibrate()) {
				warnx("calibration successful");
			} else {
				errx(1, "calibration failed");
			}
		} else {
			exit(0);
		}
	}

	// TODO

	hmc5883l_usage("unrecognized command");

	exit(1);
}


