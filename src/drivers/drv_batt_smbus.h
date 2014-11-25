/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
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
 * @file drv_batt_smbus.h
 *
 * SMBus battery monitor device API
 */

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>

/* device path */
#define BATT_SMBUS_DEVICE_PATH "/dev/batt_smbus"

/* maximum number of supported cells */
#define BATT_SMBUS_MAX_CELLS			4

/*
 * ioctl() definitions
 */

#define _BATT_SMBUS_IOCBASE		(0x2a00)
#define _BATT_SMBUS_IOC(_n)		(_IOC(_BATT_SMBUS_IOCBASE, _n))

/** read battery voltage */
#define BATT_SMBUS_READ_VOLTAGE		_BATT_SMBUS_IOC(1)

/** read battery current */
#define BATT_SMBUS_READ_CURRENT		_BATT_SMBUS_IOC(2)

/** read pack capacity */
#define BATT_SMBUS_READ_CAPACITY	_BATT_SMBUS_IOC(3)

/** read design voltage */
#define BATT_SMBUS_READ_DESIGN_VOLTAGE	_BATT_SMBUS_IOC(4)

/** read individual cell voltages */
#define BATT_SMBUS_READ_CELL_VOLTAGE	_BATT_SMBUS_IOC(5)

/* structure to hold individual cell voltages */
typedef unsigned int batt_smbus_cell_voltage[BATT_SMBUS_MAX_CELLS];
