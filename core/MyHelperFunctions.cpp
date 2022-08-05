/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2022 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include "MyHelperFunctions.h"

static uint8_t convertH2I(const char c)
{
	if (c <= '9') {
		return c - '0';
	} else if (c >= 'a') {
		return c - 'a' + 10;
	} else {
		return c - 'A' + 10;
	}
}

static char convertI2H(const uint8_t i)
{
	const uint8_t k = i & 0x0F;
	if (k <= 9) {
		return '0' + k;
	} else {
		return 'A' + k - 10;
	}
}

static int timingneutralMemcmp(const void* a, const void* b, size_t sz)
{
	int retVal;
	size_t i;
	int done = 0;
	const uint8_t* ptrA = (const uint8_t*)a;
	const uint8_t* ptrB = (const uint8_t*)b;
	for (i = 0; i < sz; i++) {
		if (ptrA[i] == ptrB[i]) {
			if (done > 0) {
				done = 1;
			} else {
				done = 0;
			}
		} else {
			if (done > 0) {
				done = 2;
			} else {
				done = 3;
			}
		}
	}
	if (done > 0) {
		retVal = -1;
	} else {
		retVal = 0;
	}
	return retVal;
}
