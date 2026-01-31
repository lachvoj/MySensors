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
 *
 * Based on wiringPi Copyright (c) 2012 Gordon Henderson.
 */

#ifndef interrupt_h
#define interrupt_h

#include <pthread.h>
#include <gpiod.h>
#include <stdint.h>

#include "GPIOD.hpp"

#define CHANGE 1
#define FALLING 2
#define RISING 3
#define NONE 4

class GPIODInterruptClass
{
  private:
    struct ThreadArgs
    {
        void (*func)();
        int pin;
#ifdef LIBGPIOD_V2
        struct gpiod_line_request *line_request;
#else
        struct gpiod_line *line;
#endif
    };

    volatile bool interruptsEnabled = true;
    pthread_mutex_t intMutex = PTHREAD_MUTEX_INITIALIZER;

    pthread_t *threadIds[GPIOD_MAX_LINE_DEFINITIONS];
#ifdef LIBGPIOD_V2
    struct gpiod_line_request *intLineRequests[GPIOD_MAX_LINE_DEFINITIONS];
#endif
    int sysFds[GPIOD_MAX_LINE_DEFINITIONS];

    int piHiPri(const int pri);

    //This is ok since using signgleton instance of GPIODInterruptClass
    static void *interruptHandler(void *args);

  public:
    GPIODInterruptClass();
    ~GPIODInterruptClass();

    void attachInterrupt(uint8_t pin, void (*func)(), uint8_t mode);
    void detachInterrupt(uint8_t pin);
    void interrupts();
    void noInterrupts();
};

extern GPIODInterruptClass GPIODInterrupt;

#endif
