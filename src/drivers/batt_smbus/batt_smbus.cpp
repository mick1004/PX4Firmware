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

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>

#include <nuttx/wqueue.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/drv_batt_smbus.h>

#define BATT_SMBUS_I2C_BUS				PX4_I2C_BUS_EXPANSION
#define BATT_SMBUS_ADDR					0x0B	/* I2C address */
#define BATT_SMBUS_TEMP					0x08	/* temperature register */
#define BATT_SMBUS_VOLTAGE				0x09	/* voltage register */
#define BATT_SMBUS_DESIGN_CAPACITY		0x18	/* design capacity register */
#define BATT_SMBUS_DESIGN_VOLTAGE		0x19	/* design voltage register */
#define BATT_SMBUS_SERIALNUM			0x1c	/* serial number register */
#define BATT_SMBUS_MANUFACTURE_INFO		0x25	/* cell voltage register */
#define BATT_SMBUS_CURRENT				0x2a	/* current register */

class BATT_SMBUS : public device::I2C
{
public:
	BATT_SMBUS(int bus = PX4_I2C_BUS_EXPANSION, int batt_smbus = BATT_SMBUS_ADDR);
	virtual ~BATT_SMBUS();

	virtual int		init();
	virtual int		probe();
	virtual int		info();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

private:

	// read battery information, updates internal variables and returns OK if successful
	int				get_voltage();
	int				get_current();
	int				get_capacity();
	int				get_design_voltage();
	int				get_cell_voltage();

	// internal variables
	unsigned int		_voltage;	// voltage in millivolts
	unsigned int		_current;	// current in milliamps
	unsigned int		_capacity;	// total pack capacity in mAh
	unsigned int		_design_voltage;	// maximum designed voltage of battery
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
BATT_SMBUS::BATT_SMBUS(int bus, int batt_smbus) :
	I2C("batt_smbus", BATT_SMBUS_DEVICE_PATH, bus, batt_smbus, 100000),
	_voltage(0),
	_current(0),
	_capacity(0),
	_design_voltage(0)
{
	// initialise cell voltage
	memset(_cell_voltage, 0, sizeof(_cell_voltage));
}

// destructor
BATT_SMBUS::~BATT_SMBUS()
{
}

int
BATT_SMBUS::init()
{
	int ret = I2C::init();

	if (ret != OK) {
		errx(1,"failed to init I2C");
		return ret;
	}

	/* read battery capacity */
	ret = get_capacity();

	return ret;
}

int
BATT_SMBUS::probe()
{
	int ret;

	/* read battery capacity to ensure we can communicate successfully */
	ret = get_capacity();

	//return ret;
	return OK;
}

int
BATT_SMBUS::info()
{
	int volt_ret = get_voltage();
	int curr_ret = get_current();

	if (volt_ret == OK && curr_ret == OK) {
		/* we don't care about power-save mode */
		log("voltage: %u, current: %u", (unsigned)_voltage, (unsigned)_current);
		return OK;
	} else {
		warnx("failed to read from battery");
		return ENOTTY;
	}
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
	uint8_t result[2];
	int ret;

	// read voltage
	ret = transfer(nullptr, BATT_SMBUS_VOLTAGE, &result[0], 2);
	if (ret == OK) {
		_voltage = (unsigned int)result[0] << 8 || (unsigned int)result[1];
	}

	// return success or failure
	return ret;
}

int
BATT_SMBUS::get_current()
{
	uint8_t result[2];
	int ret;

	// read current
	ret = transfer(nullptr, BATT_SMBUS_CURRENT, &result[0], 2);
	if (ret == OK) {
		_current = (unsigned int)result[0] << 8 || (unsigned int)result[1];
	}

	// return success or failure
	return ret;
}

int
BATT_SMBUS::get_capacity()
{
	uint8_t result[2];
	int ret;

	// read current
	ret = transfer(nullptr, BATT_SMBUS_DESIGN_CAPACITY, &result[0], 2);
	if (ret == OK) {
		_capacity = (unsigned int)result[0] << 8 || (unsigned int)result[1];
	}

	// return success or failure
	return ret;
}

int
BATT_SMBUS::get_design_voltage()
{
	uint8_t result[2];
	int ret;

	// read current
	ret = transfer(nullptr, BATT_SMBUS_DESIGN_VOLTAGE, &result[0], 2);
	if (ret == OK) {
		_design_voltage = (unsigned int)result[0] << 8 || (unsigned int)result[1];
	}

	// return success or failure
	return ret;
}

int
BATT_SMBUS::get_cell_voltage()
{
	uint8_t result[13];
	int ret;

	// read current
	ret = transfer(nullptr, BATT_SMBUS_MANUFACTURE_INFO, &result[0], 13);
	if (ret == OK) {
	    // extract individual cell voltages
		_cell_voltage[0] = (unsigned int)result[12]<<8 | (unsigned int)result[11];
		_cell_voltage[1] = (unsigned int)result[10]<<8 | (unsigned int)result[9];
		_cell_voltage[2] = (unsigned int)result[8]<<8 | (unsigned int)result[7];
		_cell_voltage[3] = (unsigned int)result[6]<<8 | (unsigned int)result[5];
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

	int fd;
	int ret;

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
		fd = open(BATT_SMBUS_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " BATT_SMBUS_DEVICE_PATH);
		}

		// read voltage
		int voltage = ioctl(fd, BATT_SMBUS_READ_VOLTAGE, (unsigned long)0);

		// display voltage
		warnx("volt:%d", voltage);

		close(fd);
		exit(ret);
	}

	if (!strcmp(verb, "info")) {
		g_batt_smbus->info();
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
