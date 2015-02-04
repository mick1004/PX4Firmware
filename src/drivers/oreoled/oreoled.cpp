/****************************************************************************
 *
 *   Copyright (C) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Julian Oes <joes@student.ethz.ch>
 *           Anton Babushkin <anton.babushkin@me.com>
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
 * @file oreoled.cpp
 *
 * Driver for the onboard RGB LED controller (TCA62724FMG) connected via I2C.
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

#include <drivers/drv_oreoled.h>

#define OREOLED_NUM_LEDS		4		// maximum number of LEDs the oreo led driver can support
#define OREOLED_BASE_I2C_ADDR	0x68	// base i2c address (7-bit)
#define OREOLED_GENERALCALL_MS	4000	// general call sent every 4 seconds
#define OREOLED_GENERALCALL_CMD	0x00	// general call command sent at regular intervals

class OREOLED : public device::I2C
{
public:
	OREOLED(int bus, int i2c_addr);
	virtual ~OREOLED();

	virtual int		init();
	virtual int		probe();
	virtual int		info();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/* send general call on I2C bus to syncronise all LEDs */
	int				send_general_call();

	/* send a string of bytes to an LEDs (used for testing only) */
	int				send_bytes(oreoled_sendbytes_t sb);

private:
	bool			_overall_health;				// overall health of sensor (true if at least one LED is responding)
	bool			_healthy[OREOLED_NUM_LEDS];		// health of each LED
	oreoled_pattern	_pattern[OREOLED_NUM_LEDS];		// pattern of each LED
	uint8_t			_r[OREOLED_NUM_LEDS];
	uint8_t			_g[OREOLED_NUM_LEDS];
	uint8_t			_b[OREOLED_NUM_LEDS];
	oreoled_macro   _macro[OREOLED_NUM_LEDS];

	/* send latest requested rgb values to all oreo LEDs */
	int				send_rgb();

	/* send latest requested macro to all oreo LEDs */
	int				send_macro();

	/* get latest requested rgb values sent to an individual led*/
	int				get(uint8_t instance, oreoled_pattern &pattern, uint8_t &r, uint8_t &g, uint8_t &b);
};

/* for now, we only support one OREOLED */
namespace
{
OREOLED *g_oreoled = nullptr;
}

void oreoled_usage();

extern "C" __EXPORT int oreoled_main(int argc, char *argv[]);

/* constructor */
OREOLED::OREOLED(int bus, int i2c_addr) :
	I2C("oreoled", OREOLED_DEVICE_PATH, bus, i2c_addr, 100000),
	_overall_health(false)
{
	// initialise to unhealthy
	memset(_healthy, false, sizeof(_healthy));

	// initialise patterns and colours
	memset(_pattern, OREOLED_PATTERN_OFF, sizeof(_pattern));
	memset(_r, 0, sizeof(_r));
	memset(_g, 0, sizeof(_g));
	memset(_b, 0, sizeof(_b));
	memset(_macro, 0, sizeof(_macro));
}

/* destructor */
OREOLED::~OREOLED()
{
}

int
OREOLED::init()
{
	int ret;

	/* initialise to unhealthy */
	_overall_health = false;

	/* initialise I2C bus */
	ret = I2C::init();
	if (ret != OK) {
		return ret;
	}

	/* prepare command to turn off LED*/
	uint8_t msg[] = {OREOLED_PATTERN_OFF};

	/* switch off each LED and set health */
	for (uint8_t i=0; i<OREOLED_NUM_LEDS; i++) {
		/* set I2C address */
		set_address(OREOLED_BASE_I2C_ADDR+i);
		/* send I2C command and record health*/
		_healthy[i] = (transfer(msg, sizeof(msg), nullptr, 0) == OK);
		usleep(1);
		/* set overall sensor to healthy if at least one LED is responding */
		if (_healthy[i]) {
			_overall_health = true;
			ret = OK;
			warnx("OREO %d ok",(unsigned)i);
		} else {
			warnx("OREO %d bad",(unsigned)i);
		}
	}

	return OK;
}

int
OREOLED::probe()
{
	/* always return true */
	return OK;
}

int
OREOLED::info()
{
	int ret;
	oreoled_pattern pattern;
	uint8_t r, g, b;

	// print info on each LED
	for (uint8_t i=0; i<OREOLED_NUM_LEDS; i++) {
		ret = get(i, pattern, r, g, b);
		if (ret == OK) {
			log("oreo %u: pattern:%u, red:%u, green:%u, blue: %u", (unsigned)i, (unsigned)pattern, (unsigned)r, (unsigned)g, (unsigned)b);
		} else {
			warnx("oreo %u: failed to read", (unsigned)i);
		}
	}

	return OK;
}

int
OREOLED::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret = ENOTTY;
	uint8_t instance;

	switch (cmd) {
	case OREOLED_SET_RGB:
		/* set the specified color */
		instance = ((oreoled_rgbset_t *) arg)->instance;

		/* special handling for request to set all instances rgb values */
		if (instance == OREOLED_ALL_INSTANCES) {
			memset(_pattern, ((oreoled_rgbset_t *) arg)->pattern, sizeof(_pattern));
			memset(_r, ((oreoled_rgbset_t *) arg)->red, sizeof(_r));
			memset(_g, ((oreoled_rgbset_t *) arg)->green, sizeof(_g));
			memset(_b, ((oreoled_rgbset_t *) arg)->blue, sizeof(_b));

		/* request to set individual instance's rgb value */
		} else if (instance < OREOLED_NUM_LEDS) {
			_pattern[instance] = ((oreoled_rgbset_t *) arg)->pattern;
			_r[instance] = ((oreoled_rgbset_t *) arg)->red;
			_g[instance] = ((oreoled_rgbset_t *) arg)->green;
			_b[instance] = ((oreoled_rgbset_t *) arg)->blue;
		}

		/* send I2C updates */
		send_rgb();
		return OK;

	case OREOLED_RUN_MACRO:
		/* run a macro */
		instance = ((oreoled_macrorun_t *) arg)->instance;

		/* special handling for request to set all instances rgb values */
		if (instance == OREOLED_ALL_INSTANCES) {
			memset(_macro, ((oreoled_macrorun_t *) arg)->macro, sizeof(_macro));

		/* request to set individual instance's rgb value */
		} else if (instance < OREOLED_NUM_LEDS) {
			memset(_macro, ((oreoled_macrorun_t *) arg)->macro, sizeof(_macro));
		}

		/* send I2C updates */
		send_macro();
		return OK;

	default:
		/* see if the parent class can make any use of it */
		ret = CDev::ioctl(filp, cmd, arg);
		break;
	}

	return ret;
}

/**
 * Send RGB to OREO LED driver according to current color
 */
int
OREOLED::send_rgb()
{
	int ret = ENOTTY;

	/* return immediately if not healthy */
	if (!_overall_health) {
		return ret;
	}

	/* for each healthy led */
	for (uint8_t i=0; i<OREOLED_NUM_LEDS; i++) {
		if (_healthy[i]) {
			/* set I2C address */
			set_address(OREOLED_BASE_I2C_ADDR+i);
			/* prepare command */
			uint8_t msg[] = {_pattern[i], OREOLED_PARAM_BIAS_RED, _r[i], OREOLED_PARAM_BIAS_GREEN, _g[i],OREOLED_PARAM_BIAS_BLUE, _b[i]};
			/* send I2C command */
			if (transfer(msg, sizeof(msg), nullptr, 0) == OK) {
				ret = OK;
			}
			usleep(1);
		}
	}

	return ret;
}

/**
 * send latest requested macro to all oreo LEDs
 */
int
OREOLED::send_macro()
{
	int ret = ENOTTY;

	/* return immediately if not healthy */
	if (!_overall_health) {
		return ret;
	}

	/* for each healthy led */
	for (uint8_t i=0; i<OREOLED_NUM_LEDS; i++) {
		if (_healthy[i]) {
			/* set I2C address */
			set_address(OREOLED_BASE_I2C_ADDR+i);
			/* prepare command */
			uint8_t macro_num = (uint8_t)_macro[i];
			uint8_t msg[] = {OREOLED_PATTERN_PARAMUPDATE, OREOLED_PARAM_MACRO, macro_num};
			/* send I2C command */
			if (transfer(msg, sizeof(msg), nullptr, 0) == OK) {
				ret = OK;
			}
			usleep(1);
		}
	}

	return ret;
}

/* send general call on I2C bus to syncronise all LEDs */
int
OREOLED::send_general_call()
{
	int ret = ENOTTY;

	/* return immediately if not healthy */
	if (!_overall_health) {
		return ret;
	}

	/* set I2C address to zero */
	set_address(0);

	/* prepare command : 0x01 = general hardware call, 0x00 = I2C address of master (but we don't act as a slave so set to zero)*/
	uint8_t msg[] = {0x01,0x00};

	/* send I2C command */
	if (transfer(msg, sizeof(msg), nullptr, 0) == OK) {
		ret = OK;
	}

	return ret;
}

/* send a string of bytes to an LEDs (used for testing only) */
int
OREOLED::send_bytes(oreoled_sendbytes_t sb)
{
	int ret = ENOTTY;

	/* return immediately if not healthy */
	if (!_overall_health) {
		return ret;
	}

	/* set I2C address to zero */
	set_address(OREOLED_BASE_I2C_ADDR+sb.led_num);

	/* sanity check number of bytes */
	if (sb.num_bytes > 20) {
		return ret;
	}

	/* send bytes */
	ret = transfer(sb.buff, sb.num_bytes, nullptr, 0);

	return ret;
}

int
OREOLED::get(uint8_t instance, oreoled_pattern &pattern, uint8_t &r, uint8_t &g, uint8_t &b)
{
	// sanity check instance
	if (instance < OREOLED_NUM_LEDS) {
		pattern = _pattern[instance];
		r = _r[instance];
		g = _g[instance];
		b = _b[instance];
		return OK;
	} else {
		return EINVAL;
	}
}

void
oreoled_usage()
{
	warnx("missing command: try 'start', 'test', 'info', 'off', 'stop', 'rgb 30 40 50' 'macro 4' 'gencall' 'bytes <lednum> 7 9 6'");
	warnx("options:");
	warnx("    -b i2cbus (%d)", PX4_I2C_BUS_LED);
	warnx("    -a addr (0x%x)", OREOLED_BASE_I2C_ADDR);
}

int
oreoled_main(int argc, char *argv[])
{
	int i2cdevice = -1;
	int i2c_addr = OREOLED_BASE_I2C_ADDR; /* 7bit */

	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "a:b:")) != EOF) {
		switch (ch) {
		case 'a':
			i2c_addr = strtol(optarg, NULL, 0);
			break;

		case 'b':
			i2cdevice = strtol(optarg, NULL, 0);
			break;

		default:
			oreoled_usage();
			exit(0);
		}
	}

        if (optind >= argc) {
            oreoled_usage();
            exit(1);
        }

	const char *verb = argv[optind];

	int fd;
	int ret;

	if (!strcmp(verb, "start")) {
		if (g_oreoled != nullptr)
			errx(1, "already started");

		if (i2cdevice == -1) {
			// try the external bus first
			i2cdevice = PX4_I2C_BUS_EXPANSION;
			g_oreoled = new OREOLED(PX4_I2C_BUS_EXPANSION, i2c_addr);

			if (g_oreoled != nullptr && OK != g_oreoled->init()) {
				delete g_oreoled;
				g_oreoled = nullptr;
			}

			if (g_oreoled == nullptr) {
				// fall back to default bus
				if (PX4_I2C_BUS_LED == PX4_I2C_BUS_EXPANSION) {
					errx(1, "init failed");
				}
				i2cdevice = PX4_I2C_BUS_LED;
			}
		}

		if (g_oreoled == nullptr) {
			g_oreoled = new OREOLED(i2cdevice, i2c_addr);

			if (g_oreoled == nullptr)
				errx(1, "new failed");

			if (OK != g_oreoled->init()) {
				delete g_oreoled;
				g_oreoled = nullptr;
				errx(1, "init failed");
			}
		}

		exit(0);
	}

	/* need the driver past this point */
	if (g_oreoled == nullptr) {
		warnx("not started");
		oreoled_usage();
		exit(1);
	}

	if (!strcmp(verb, "test")) {
		fd = open(OREOLED_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED_DEVICE_PATH);
		}

		/* structure to hold desired colour */
		oreoled_rgbset_t rgb_set_red = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_SOLID, 0xFF, 0x0, 0x0};
		oreoled_rgbset_t rgb_set_blue = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_SOLID, 0x0, 0x0, 0xFF};
		oreoled_rgbset_t rgb_set_off = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_OFF, 0x0, 0x0, 0x0};

		/* flash red and blue for 3 seconds */
		for (uint8_t i=0; i<30; i++) {
			/* red */
			ret = ioctl(fd, OREOLED_SET_RGB, (unsigned long)&rgb_set_red);
			/* sleep for 0.05 seconds */
			usleep(50000);
			/* blue */
			ret = ioctl(fd, OREOLED_SET_RGB, (unsigned long)&rgb_set_blue);
			/* sleep for 0.05 seconds */
			usleep(50000);
		}
		/* turn off LED */
		ret = ioctl(fd, OREOLED_SET_RGB, (unsigned long)&rgb_set_off);

		close(fd);
		exit(ret);
	}

	if (!strcmp(verb, "info")) {
		g_oreoled->info();
		exit(0);
	}

	if (!strcmp(verb, "off") || !strcmp(verb, "stop")) {
		fd = open(OREOLED_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED_DEVICE_PATH);
		}

		/* turn off LED */
		oreoled_rgbset_t rgb_set_off = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_OFF, 0x0, 0x0, 0x0};
		ret = ioctl(fd, OREOLED_SET_RGB, (unsigned long)&rgb_set_off);

		close(fd);
		/* delete the oreoled object if stop was requested, in addition to turning off the LED. */
		if (!strcmp(verb, "stop")) {
			delete g_oreoled;
			g_oreoled = nullptr;
			exit(0);
		}
		exit(ret);
	}

	if (!strcmp(verb, "rgb")) {
		if (argc < 5) {
			errx(1, "Usage: oreoled rgb <red> <green> <blue>");
		}

		fd = open(OREOLED_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED_DEVICE_PATH);
		}

		uint8_t red = strtol(argv[2], NULL, 0);
		uint8_t green = strtol(argv[3], NULL, 0);
		uint8_t blue = strtol(argv[4], NULL, 0);
		oreoled_rgbset_t rgb_set = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_SOLID, red, green, blue};
		ret = ioctl(fd, OREOLED_SET_RGB, (unsigned long)&rgb_set);
		close(fd);
		exit(ret);
	}

	if (!strcmp(verb, "macro")) {
		if (argc < 3) {
			errx(1, "Usage: oreoled macro <macro_num>");
		}

		fd = open(OREOLED_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED_DEVICE_PATH);
		}

		uint8_t macro = strtol(argv[2], NULL, 0);

		/* sanity check macro number */
		if (macro > OREOLED_PARAM_MACRO_ENUM_COUNT) {
			errx(1, "invalid macro number %d",(int)macro);
			exit(ret);
		}

		oreoled_macrorun_t macro_run = {OREOLED_ALL_INSTANCES, (enum oreoled_macro)macro};
		ret = ioctl(fd, OREOLED_RUN_MACRO, (unsigned long)&macro_run);
		close(fd);
		exit(ret);
	}

	/* send general hardware call to all LEDS */
	if (!strcmp(verb, "gencall")) {
		ret = g_oreoled->send_general_call();
		warnx("sent general call");
		exit(ret);
	}

	/* send a string of bytes to an LED using send_bytes function */
	if (!strcmp(verb, "bytes")) {
		if (argc < 3) {
			errx(1, "Usage: oreoled bytes <led_num> <byte1> <byte2> <byte3> ...");
		}

		// structure to be sent
		oreoled_sendbytes_t sendb;

		// maximum of 20 bytes can be sent
		if (argc > 23) {
			errx(1, "Max of 20 bytes can be sent");
		}

		// check led num
		sendb.led_num = strtol(argv[2], NULL, 0);
		if (sendb.led_num > 3) {
			errx(1, "led number must be between 0 ~ 3");
		}

		// get bytes
		sendb.num_bytes = argc-3;
		uint8_t byte_count;
		for (byte_count=0; byte_count<sendb.num_bytes; byte_count++) {
			sendb.buff[byte_count] = strtol(argv[byte_count+3], NULL, 0);
		}

		// send bytes
		ret = g_oreoled->send_bytes(sendb);
		warnx("sent %d bytes 0:%x 1:%x 2:%x 3:%x 4:%x 5:%x 6:%x 7:%x",
				(int)sendb.num_bytes,
				(int)sendb.buff[0],
				(int)sendb.buff[1],
				(int)sendb.buff[2],
				(int)sendb.buff[3],
				(int)sendb.buff[4],
				(int)sendb.buff[5],
				(int)sendb.buff[6],
				(int)sendb.buff[7]);

		warnx("sent %d bytes",(int)sendb.num_bytes);
		exit(ret);
	}

	oreoled_usage();
	exit(0);
}
