/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2020 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#ifndef _SPI_H_
#define _SPI_H_

#ifdef LINUX_SPI_BCM
#include "SPIBCM.h"
#define SPIClass SPIBCMClass
#elif LINUX_SPI_SPIDEV
#include "SPIDEV.h"
#define SPIClass SPIDEVClass
#endif

#ifdef SPIClass
SPIClass spi0 = SPIClass();
#define SPI spi0
#if (SPI_COUNT > 1)
#ifndef SPI1_SPIDEV_DEVICE
#define SPI1_SPIDEV_DEVICE "/dev/spidev1.0"
#endif
SPIClass spi1 = SPIClass(SPI1_SPIDEV_DEVICE);
#define SPI1 spi1
#endif
#if (SPI_COUNT > 2)
#ifndef SPI2_SPIDEV_DEVICE
#define SPI2_SPIDEV_DEVICE "/dev/spidev1.1"
#endif
SPIClass spi2 = SPIClass(SPI2_SPIDEV_DEVICE);
#define SPI2 spi2
#endif
#endif

#endif
