/*
 * HMC5883L.h
 *
 *  Created on: Mar 30, 2015
 *      Author: Lucifer
 */

#ifndef FIRMWARE_SRC_DRIVERS_HMC5883L_HMC5883L_H_
#define FIRMWARE_SRC_DRIVERS_HMC5883L_HMC5883L_H_

#include "../device/I2C.h"
#include "../drv_device.h"
#include "../device/RingBuffer.h"
#include "../../lib/system/perf_counter.h"

#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

namespace device {

class HMC5883L: public device::I2C {
public:
	HMC5883L(int bus, const char* path);
	virtual ~HMC5883L();

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

	/*
	 * Reset the device
	 */
	int reset();

	/*
	 * Sets the internal sensor range to handle at least the argument in Gauss.
	 *
	 * @param range
	 * @return
	 */
	int set_range(unsigned int range);

	/*
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable
	 * 		Set to 1 to enable self-test positive strap, -1 to enable
	 *        negative strap, 0 to set to normal mode
	 * @return
	 */
	int set_excitement(unsigned int enable);

	/*
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void cycle();

	/*
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg
	 * 		Instance pointer for the driver that is polling.
	 */
	static void cycle_trampoline(void *arg);

	/*
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 * @param filp
	 * 		Pointer to the NuttX file structure.
	 * @param enable
	 * 		Set to 1 to enable self-test strap, 0 to disable
	 */
	int calibrate(struct file *filp, unsigned int enable);

	/*
	 * Issue a measurement command.
	 *
	 * @return OK if the measurement command was successful.
	 */
	int measure();

	/*
	 * Collect the result of the most recent measurement.
	 *
	 * @return OK if the measurement command was successful.
	 */
	int collect();

	/*
	 * check the sensor configuration.
	 *
	 * checks that the config of the sensor is correctly set, to
	 * cope with communication errors causing the configuration to
	 * change
	 */
	void check_conf();

	/*
	 * Check the current offset calibration
	 *
	 * @return 0 if offset calibration is OK, 1 else
	 */
	int check_offset();

	 /*
	 * checks that the range of the sensor is correctly set, to
	 * cope with communication errors causing the range to change
	 */
	void check_range();

	/*
	 * Check the current calibration and update device status
	 *
	 * @return 0 if calibration is OK, 1 else
	 */
	int check_calibration();

	/*
	 * Check the current scale calibration
	 *
	 * @return 0 if scale calibration is OK, 1 else
	 */
	int check_scale();

	/*
	 * Write a register.
	 *
	 * @param reg
	 * 		The register to write.
	 * @param val
	 * 		The value to write.
	 * @return OK on write success.
	 */
	int write_reg(uint8_t reg, uint8_t val);

	/*
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
	work_s _work;
	unsigned int _measure_ticks;

	RingBuffer* _reports;

	mag_scale _scale;
	uint8_t _range_bits;
	float _range_scale;
	float _range_ga;
	uint8_t _conf_reg;
	bool _collect_phase;
	int _class_instance;

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;
	perf_counter_t _buffer_overflows;
	perf_counter_t _range_errors;
	perf_counter_t _conf_errors;

	/* status reporting */
	bool _sensor_ok; // sensor was found and reports OK
	bool _calibrated; // the calibration is valid

	struct mag_report _last_report; // used for info()
};

} /* namespace device */

#endif /* FIRMWARE_SRC_DRIVERS_HMC5883L_HMC5883L_H_ */
