/*
 * l3g4200d_main.cpp
 *
 *  Created on: Apr 2, 2015
 *      Author: Lucifer
 */

#include <nuttx/config.h>

#include "L3G4200D.h"
#include "../lib/system/err.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
#undef ERROR
#endif
#define ERROR (-1)

////////////////////////////////////////////////////////////////////////////////

using namespace device;

#define L3G4200D_DEVICE_PATH	"/dev/l3g4200d"

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int l3g4200d_main(int argc, char *argv[]);

// static L3G4200D*__dev_ptr = nullptr;

int l3g4200d_main(int argc, char *argv[]) {
	if (argc < 1) {
		l3g4200d_usage("missing command");
		exit(1);
	}
	// TODO options

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (OK != l3g4200d_start()) {
			errx(1, "start failed");
		}
	}

	// TODO

	hmc5883l_usage("unrecognized command");

	exit(1);
}
