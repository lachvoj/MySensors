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

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "GPIODInterrupt.hpp"
#include "log.h"

// Declare a single default instance
GPIODInterruptClass GPIODInterrupt = GPIODInterruptClass();

GPIODInterruptClass::GPIODInterruptClass()
{
    memset(threadIds, 0, sizeof(threadIds));
    memset(sysFds, -1, sizeof(sysFds));
#ifdef LIBGPIOD_V2
    memset(intLineRequests, 0, sizeof(intLineRequests));
#endif
}

GPIODInterruptClass::~GPIODInterruptClass()
{
#ifdef LIBGPIOD_V2
    for (int i = 0; i < GPIOD_MAX_LINE_DEFINITIONS; ++i)
    {
        if (intLineRequests[i] != NULL)
        {
            gpiod_line_request_release(intLineRequests[i]);
        }
    }
#endif
}

/*
 * Part of wiringPi: Simple way to get your program running at high priority
 * with realtime schedulling.
 */
int GPIODInterruptClass::piHiPri(const int pri)
{
    struct sched_param sched;

    memset(&sched, 0, sizeof(sched));

    if (pri > sched_get_priority_max(SCHED_RR))
    {
        sched.sched_priority = sched_get_priority_max(SCHED_RR);
    }
    else
    {
        sched.sched_priority = pri;
    }

    return sched_setscheduler(0, SCHED_RR, &sched);
}

void *GPIODInterruptClass::interruptHandler(void *args)
{
    int fd;
    struct ThreadArgs *arguments = (struct ThreadArgs *)args;
    int pin = arguments->pin;
#ifdef LIBGPIOD_V2
    struct gpiod_line_request *line_request = arguments->line_request;
#else
    struct gpiod_line *line = arguments->line;
#endif
    void (*func)() = arguments->func;
    delete arguments;

    (void)GPIODInterrupt.piHiPri(55); // Only effective if we run as root

    if ((fd = GPIODInterrupt.sysFds[pin]) == -1)
    {
        logError("GPIODInterruptClass::interruptHandler: Failed to attach interrupt for pin %d\n", pin);
        return NULL;
    }

#ifdef LIBGPIOD_V2
    struct gpiod_edge_event_buffer *event_buffer = gpiod_edge_event_buffer_new(1);
    if (event_buffer == NULL)
    {
        logError("GPIODInterruptClass::interruptHandler: Failed to create event buffer\n");
        return NULL;
    }

    while (1)
    {
        // Wait for event
        int ret = gpiod_line_request_wait_edge_events(line_request, -1);
        if (ret < 0)
        {
            logError("GPIODInterruptClass::interruptHandler: Error waiting for interrupt: %s\n", strerror(errno));
            break;
        }

        // Read the event
        ret = gpiod_line_request_read_edge_events(line_request, event_buffer, 1);
        if (ret < 0)
        {
            logError("GPIODInterruptClass::interruptHandler: Error reading edge event: %s\n", strerror(errno));
            continue;
        }

#ifdef MY_DEBUG_VERBOSE_CORE
        struct gpiod_edge_event *event = gpiod_edge_event_buffer_get_event(event_buffer, 0);
        if (gpiod_edge_event_get_event_type(event) == GPIOD_EDGE_EVENT_RISING_EDGE)
        {
            logInfo("GPIODInterruptClass::interruptHandler: RISING Edge on pin %d\n", pin);
        }
        else
        {
            logInfo("GPIODInterruptClass::interruptHandler: FALLING Edge on pin %d\n", pin);
        }
#endif
        pthread_mutex_lock(&GPIODInterrupt.intMutex);
        if (GPIODInterrupt.interruptsEnabled)
        {
            pthread_mutex_unlock(&GPIODInterrupt.intMutex);
            func();
        }
        else
        {
            pthread_mutex_unlock(&GPIODInterrupt.intMutex);
        }
    }

    gpiod_edge_event_buffer_free(event_buffer);
    gpiod_line_request_release(line_request);
#else
    while (1)
    {
        // Wait for it ...
        int ret = gpiod_line_event_wait(line, NULL);
        if (ret < 0)
        {
            logError("GPIODInterruptClass::interruptHandler: Error waiting for interrupt: %s\n", strerror(errno));
            break;
        }

#ifdef MY_DEBUG_VERBOSE_CORE
        struct gpiod_line_event event;
        int event_read_result = gpiod_line_event_read(line, &event);
        if (event_read_result == 0)
        {
            if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE)
            {
                logInfo("GPIODInterruptClass::interruptHandler: RISING Edge on line offset %d, name %s\n", gpiod_line_offset(line), gpiod_line_name(line));
            }
            else
            {
                logInfo("GPIODInterruptClass::interruptHandler: FALLING Edge on line offset %d, name %s\n", gpiod_line_offset(line), gpiod_line_name(line));
            }
        }
#endif
        pthread_mutex_lock(&GPIODInterrupt.intMutex);
        if (GPIODInterrupt.interruptsEnabled)
        {
            pthread_mutex_unlock(&GPIODInterrupt.intMutex);
            func();
        }
        else
        {
            pthread_mutex_unlock(&GPIODInterrupt.intMutex);
        }
    }

    gpiod_line_release(line);
#endif
    close(fd);

    return NULL;
}

void GPIODInterruptClass::attachInterrupt(uint8_t pin, void (*func)(), uint8_t mode)
{
    if (threadIds[pin] == NULL)
    {
        threadIds[pin] = new pthread_t;
    }
    else
    {
        // Cancel the existing thread for that pin
        pthread_cancel(*threadIds[pin]);
        // Wait a bit
        usleep(1000);
    }

#ifdef LIBGPIOD_V2
    // Release any existing interrupt request for this pin
    if (intLineRequests[pin] != NULL)
    {
        gpiod_line_request_release(intLineRequests[pin]);
        intLineRequests[pin] = NULL;
    }

    // Release any existing GPIOD request for this pin (e.g., from pinMode called before attachInterrupt)
    if (GPIOD.line_requests[pin] != NULL)
    {
        gpiod_line_request_release(GPIOD.line_requests[pin]);
        GPIOD.line_requests[pin] = NULL;
    }

    struct gpiod_line_settings *settings = gpiod_line_settings_new();
    if (settings == NULL)
    {
        logError("GPIODInterruptClass::attachInterrupt: Failed to create line settings for pin %d\n", pin);
        exit(1);
    }

    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);

    switch (mode)
    {
    case CHANGE: gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_BOTH); break;
    case FALLING: gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_FALLING); break;
    case RISING: gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING); break;
    case NONE: break;
    default:
        gpiod_line_settings_free(settings);
        logError("GPIODInterruptClass::attachInterrupt: Invalid mode\n");
        return;
    }

    struct gpiod_line_config *line_cfg = gpiod_line_config_new();
    if (line_cfg == NULL)
    {
        gpiod_line_settings_free(settings);
        logError("GPIODInterruptClass::attachInterrupt: Failed to create line config for pin %d\n", pin);
        exit(1);
    }

    unsigned int offsets[] = {pin};
    if (gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings) != 0)
    {
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        logError("GPIODInterruptClass::attachInterrupt: Failed to add line settings for pin %d\n", pin);
        exit(1);
    }

    struct gpiod_request_config *req_cfg = gpiod_request_config_new();
    if (req_cfg == NULL)
    {
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        logError("GPIODInterruptClass::attachInterrupt: Failed to create request config for pin %d\n", pin);
        exit(1);
    }
    gpiod_request_config_set_consumer(req_cfg, "gpiointerrupt");

    intLineRequests[pin] = gpiod_chip_request_lines(GPIOD.chip, req_cfg, line_cfg);

    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(settings);

    if (intLineRequests[pin] == NULL)
    {
        logError("GPIODInterruptClass::attachInterrupt: Unable to register event listener on pin %d: %s\n", pin, strerror(errno));
        exit(1);
    }

    if (sysFds[pin] == -1)
    {
        if ((sysFds[pin] = gpiod_line_request_get_fd(intLineRequests[pin])) < 0)
        {
            logError("Error reading pin %d: %s\n", pin, strerror(errno));
            exit(1);
        }
    }

    struct ThreadArgs *threadArgs = new struct ThreadArgs;
    threadArgs->func = func;
    threadArgs->pin = pin;
    threadArgs->line_request = intLineRequests[pin];
#else
    if (GPIOD.gpiod_lines[pin] != NULL)
    {
        gpiod_line_release(GPIOD.gpiod_lines[pin]);
    }
    GPIOD.gpiod_lines[pin] = gpiod_chip_get_line(GPIOD.chip, pin);
    struct gpiod_line *line = GPIOD.gpiod_lines[pin];

    int reqRet = -1;
    switch (mode)
    {
    case CHANGE: reqRet = gpiod_line_request_both_edges_events(line, "gpiointerrupt"); break;
    case FALLING: reqRet = gpiod_line_request_falling_edge_events(line, "gpiointerrupt"); break;
    case RISING: reqRet = gpiod_line_request_rising_edge_events(line, "gpiointerrupt"); break;
    case NONE: break;
    default: logError("GPIODInterruptClass::attachInterrupt: Invalid mode\n"); return;
    }

    if (reqRet != 0)
    {
        logError("GPIODInterruptClass::attachInterrupt: Unable to register event listener on pin %d.", pin);
        exit(1);
    }

    if (sysFds[pin] == -1)
    {
        if ((sysFds[pin] = gpiod_line_event_get_fd(line)) < 0)
        {
            logError("Error reading pin %d: %s\n", pin, strerror(errno));
            exit(1);
        }
    }

    struct ThreadArgs *threadArgs = new struct ThreadArgs;
    threadArgs->func = func;
    threadArgs->pin = pin;
    threadArgs->line = line;
#endif

    // Create a thread passing the pin and function
    pthread_create(threadIds[pin], NULL, &GPIODInterruptClass::interruptHandler, (void *)threadArgs);
}

void GPIODInterruptClass::detachInterrupt(uint8_t pin)
{
    // Cancel the thread
    if (threadIds[pin] != NULL)
    {
        pthread_cancel(*threadIds[pin]);
        delete threadIds[pin];
        threadIds[pin] = NULL;
    }

    // Close filehandle
    if (sysFds[pin] != -1)
    {
        close(sysFds[pin]);
        sysFds[pin] = -1;
    }

#ifdef LIBGPIOD_V2
    if (intLineRequests[pin] != NULL)
    {
        gpiod_line_request_release(intLineRequests[pin]);
        intLineRequests[pin] = NULL;
    }
#endif
}

void GPIODInterruptClass::interrupts()
{
    pthread_mutex_lock(&intMutex);
    interruptsEnabled = true;
    pthread_mutex_unlock(&intMutex);
}

void GPIODInterruptClass::noInterrupts()
{
    pthread_mutex_lock(&intMutex);
    interruptsEnabled = false;
    pthread_mutex_unlock(&intMutex);
}
