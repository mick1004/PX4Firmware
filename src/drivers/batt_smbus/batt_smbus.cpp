/****************************************************************************
 *
 *   Copyright (C) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Randy Mackay <rmackay9@yahoo.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file batt_smbus.cpp
 *
 * Driver for a battery monitor connected via SMBus (I2C).
 *
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sched.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <ctype.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/battery_status.h>

#include <float.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_batt_smbus.h>
#include <drivers/device/ringbuffer.h>

#define BATT_SMBUS_I2C_BUS				PX4_I2C_BUS_EXPANSION
#define BATT_SMBUS_ADDR					0x0B	/* I2C address */
#define BATT_SMBUS_TEMP					0x08	/* temperature register */
#define BATT_SMBUS_VOLTAGE				0x09	/* voltage register */
#define BATT_SMBUS_DESIGN_CAPACITY		0x18	/* design capacity register */
#define BATT_SMBUS_DESIGN_VOLTAGE		0x19	/* design voltage register */
#define BATT_SMBUS_SERIALNUM			0x1c	/* serial number register */
#define BATT_SMBUS_MANUFACTURE_INFO		0x25	/* cell voltage register */
#define BATT_SMBUS_CURRENT				0x2a	/* current register */
#define BATT_SMBUS_MEASUREMENT_INTERVAL_MS	(1000000 / 10)	/* time in microseconds, measure at 10hz */

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class BATT_SMBUS : public device::I2C
{
public:
	BATT_SMBUS(int bus = PX4_I2C_BUS_EXPANSION, uint16_t batt_smbus_addr = BATT_SMBUS_ADDR);
	virtual ~BATT_SMBUS();

	virtual int		init();
	virtual int		test();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	// read battery information, updates internal variables and returns OK if successful
	int				get_voltage();
	int				get_current();
	int				get_capacity();
	int				get_design_voltage();
	int				get_cell_voltage();

protected:
	virtual int		probe();		// check if the device can be contacted

private:

	// start periodic reads from the battery
	void			start();

	// stop periodic reads from the battery
	void			stop();

	// static function that is called by worker queue
	static void		cycle_trampoline(void *arg);

	// perform a read from the battery
	void			cycle();

	// read_reg - read a word from specified register
	int				read_reg(uint8_t reg, uint16_t &val);

	// internal variables
	bool			_sensor_ok;		// true if battery was found and reports ok
	work_s			_work;			// work queue for scheduling reads
	unsigned		_measure_ticks;
	RingBuffer		*_reports;		// buffer of recorded voltages, currents
	struct battery_status_s _last_report;	// last published report, used for test()
	orb_advert_t	_batt_topic;
	orb_id_t		_batt_orb_id;
	uint16_t		_voltage;	// voltage in millivolts
	int16_t 		_current;	// current in milliamps (negative is discharging, positive is charging)
	uint16_t		_capacity;	// total pack capacity in mAh
	uint16_t		_design_voltage;	// maximum designed voltage of battery
	batt_smbus_cell_voltage	_cell_voltage;	// individual cell voltages
};

/* for now, we only support one BATT_SMBUS */
namespace
{
BATT_SMBUS *g_batt_smbus;
}

void batt_smbus_usage();

extern "C" __EXPORT int batt_smbus_main(int argc, char *argv[]);

// constructor
BATT_SMBUS::BATT_SMBUS(int bus, uint16_t batt_smbus_addr) :
	I2C("batt_smbus", BATT_SMBUS_DEVICE_PATH, bus, batt_smbus_addr, 100000),
	_sensor_ok(false),
	_work{},
	_measure_ticks(0),
	_reports(nullptr),
	_batt_topic(-1),
	_batt_orb_id(nullptr),
	_voltage(0),
	_current(0),
	_capacity(0),
	_design_voltage(0)
{
	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));

	// initialise cell voltage
	memset(_cell_voltage, 0, sizeof(_cell_voltage));
}

// destructor
BATT_SMBUS::~BATT_SMBUS()
{
	/* make sure we are truly inactive */
	stop();

	if (_reports != nullptr) {
		delete _reports;
	}
}

int
BATT_SMBUS::init()
{
	int ret = ENOTTY;

	// attempt to initialise I2C bus
	ret = I2C::init();

	if (ret != OK) {
		errx(1,"failed to init I2C");
		return ret;
	} else {
		/* allocate basic report buffers */
		_reports = new RingBuffer(2, sizeof(struct battery_status_s));
		if (_reports == nullptr) {
			ret = ENOTTY;
		} else {
			// start work queue
			start();
			_sensor_ok = true;
		}
	}

	// init orb id
	_batt_orb_id = ORB_ID(battery_status);

	return ret;
}

int
BATT_SMBUS::test()
{
	int sub = orb_subscribe(ORB_ID(battery_status));
	struct battery_status_s status;

	for (uint8_t i=0; i<10; i++) {
		if (orb_copy(ORB_ID(battery_status), sub, &status) == OK) {
			printf("V=%f C=%f\n", status.voltage_v, status.current_a);
		}
		// sleep for 0.2 seconds
		usleep(200000);
	}

	return OK;
}

int
BATT_SMBUS::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret = ENOTTY;

	switch (cmd) {
	case BATT_SMBUS_READ_VOLTAGE:
		/* read voltage */
		get_voltage();
		return _voltage;

	case BATT_SMBUS_READ_CURRENT:
		/* read current */
		get_current();
		return _current;

	case BATT_SMBUS_READ_CAPACITY:
		/* read capacity */
		get_capacity();
		return _capacity;

	case BATT_SMBUS_READ_DESIGN_VOLTAGE:
		/* read design_voltage */
		get_design_voltage();
		return _design_voltage;

	case BATT_SMBUS_READ_CELL_VOLTAGE:
		/* get cell voltages */
		get_cell_voltage();
		/* copy voltages out */
		memcpy((struct batt_smbus_cell_voltages *) arg, &_cell_voltage, sizeof(_cell_voltage));
		return 0;

	default:
		/* see if the parent class can make any use of it */
		ret = CDev::ioctl(filp, cmd, arg);
		break;
	}

	return ret;
}

int
BATT_SMBUS::get_voltage()
{
	return read_reg(BATT_SMBUS_VOLTAGE, _voltage);
}

int
BATT_SMBUS::get_current()
{
	uint16_t tmp;
	// To-Do: change to use read_block
	return read_reg(BATT_SMBUS_CURRENT, tmp);
	_current = (int16_t)tmp;
}

int
BATT_SMBUS::get_capacity()
{
	return read_reg(BATT_SMBUS_DESIGN_CAPACITY, _capacity);
}

int
BATT_SMBUS::get_design_voltage()
{
	return read_reg(BATT_SMBUS_DESIGN_VOLTAGE, _design_voltage);
}

int
BATT_SMBUS::get_cell_voltage()
{
	uint8_t result[13];
	int ret;

	// read manufacturer info
	uint8_t reg = BATT_SMBUS_MANUFACTURE_INFO;
	ret = transfer(&reg, 1, result, 13);
	if (ret == OK) {
	    // extract individual cell voltages
		_cell_voltage[0] = (uint16_t)result[12]<<8 | (uint16_t)result[11];
		_cell_voltage[1] = (uint16_t)result[10]<<8 | (uint16_t)result[9];
		_cell_voltage[2] = (uint16_t)result[8]<<8 | (uint16_t)result[7];
		_cell_voltage[3] = (uint16_t)result[6]<<8 | (uint16_t)result[5];
	}

	// return success or failure
	return ret;
}

int
BATT_SMBUS::probe()
{
	// always return OK to ensure device starts
	return OK;
}

// start periodic reads from the battery
void
BATT_SMBUS::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&BATT_SMBUS::cycle_trampoline, this, 1);
}

// stop periodic reads from the battery
void
BATT_SMBUS::stop()
{
	work_cancel(HPWORK, &_work);
}

// static function that is called by worker queue
void
BATT_SMBUS::cycle_trampoline(void *arg)
{
	BATT_SMBUS *dev = (BATT_SMBUS *)arg;

	dev->cycle();
}

// perform a read from the battery
void
BATT_SMBUS::cycle()
{
	// read data from sensor
	struct battery_status_s new_report;

	// set time of reading
	new_report.timestamp = hrt_absolute_time();

    // read voltage and current
	uint16_t tmp;
    if (read_reg(BATT_SMBUS_VOLTAGE, tmp) == OK) {
    	new_report.voltage_v = (float)tmp;
    } else {
    	new_report.voltage_v = 0.0f;
    }

    // public readings to orb
	if (_batt_topic != -1) {
		orb_publish(_batt_orb_id, _batt_topic, &new_report);
	} else {
		_batt_topic = orb_advertise(_batt_orb_id, &new_report);
		if (_batt_topic < 0) {
			errx(1, "ADVERT FAIL");
		}
	}

    // copy report for test()
    _last_report = new_report;

	/* post a report to the ring */
	_reports->force(&new_report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK, &_work, (worker_t)&BATT_SMBUS::cycle_trampoline, this, USEC2TICK(BATT_SMBUS_MEASUREMENT_INTERVAL_MS));
}

int
BATT_SMBUS::read_reg(uint8_t reg, uint16_t &val)
{
	int ret = ENOTTY;
	uint8_t buff[2];

	// read current
    ret = transfer(&reg, 1, buff, 2);
	if (ret == OK) {
		val = (uint16_t)buff[1] << 8 | (uint16_t)buff[0];
	}

	// return success or failure
	return ret;
}

void
batt_smbus_usage()
{
	warnx("missing command: try 'start', 'test', 'stop'");
	warnx("options:");
		warnx("    -b i2cbus (%d)", BATT_SMBUS_I2C_BUS);
		warnx("    -a addr (0x%x)", BATT_SMBUS_ADDR);
}

int
batt_smbus_main(int argc, char *argv[])
{
	int i2cdevice = BATT_SMBUS_I2C_BUS;
	int batt_smbusadr = BATT_SMBUS_ADDR; /* 7bit */

	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "a:b:")) != EOF) {
		switch (ch) {
		case 'a':
			batt_smbusadr = strtol(optarg, NULL, 0);
			break;

		case 'b':
			i2cdevice = strtol(optarg, NULL, 0);
			break;

		default:
			batt_smbus_usage();
			exit(0);
		}
	}

	if (optind >= argc) {
		batt_smbus_usage();
		exit(1);
	}

	const char *verb = argv[optind];

	if (!strcmp(verb, "start")) {
		if (g_batt_smbus != nullptr) {
			errx(1, "already started");
		} else {
			// create new global object
			g_batt_smbus = new BATT_SMBUS(i2cdevice, batt_smbusadr);

			if (g_batt_smbus == nullptr)
				errx(1, "new failed");

			if (OK != g_batt_smbus->init()) {
				delete g_batt_smbus;
				g_batt_smbus = nullptr;
				errx(1, "init failed xxx");
			} else {
				warnx("init succeeded");
			}
		}

		exit(0);
	}

	/* need the driver past this point */
	if (g_batt_smbus == nullptr) {
		warnx("not started");
		batt_smbus_usage();
		exit(1);
	}

	if (!strcmp(verb, "test")) {
		g_batt_smbus->test();
		exit(0);
	}

	if (!strcmp(verb, "stop")) {
		delete g_batt_smbus;
		g_batt_smbus = nullptr;
		exit(0);
	}

	batt_smbus_usage();
	exit(0);
}
