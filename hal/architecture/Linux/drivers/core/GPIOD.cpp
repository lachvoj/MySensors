#include <dirent.h>
#include <gpiod.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include "GPIOD.hpp"
#include "log.h"

// Declare a single default instance
GPIODClass GPIOD = GPIODClass();

GPIODClass::GPIODClass()
{
#ifdef LIBGPIOD_V2
    chip = gpiod_chip_open(chipdevname);
#else
    chip = gpiod_chip_open(chipdevname);
#endif
    if (chip == NULL)
    {
        logError("GPIODClass: Failed to open gpio chip %s\n", chipdevname);
        exit(1);
    }
#ifdef LIBGPIOD_V2
    memset(line_requests, 0, sizeof(line_requests));
#else
    memset(gpiod_lines, 0, sizeof(gpiod_lines));
#endif
}

GPIODClass::GPIODClass(const GPIODClass &other)
{
    chipdevname = other.chipdevname;
    chip = other.chip;
}

GPIODClass::~GPIODClass()
{
#ifdef LIBGPIOD_V2
    for (int i = 0; i < GPIOD_MAX_LINE_DEFINITIONS; ++i)
    {
        if (line_requests[i] != NULL)
        {
            gpiod_line_request_release(line_requests[i]);
        }
    }
#endif
    gpiod_chip_close(chip);
}

void GPIODClass::pinMode(uint8_t pin, uint8_t mode)
{
    if (pin >= GPIOD_MAX_LINE_DEFINITIONS)
    {
        logError("GPIODClass::pinMode: Pin number too big: %d >= %d\n", pin, GPIOD_MAX_LINE_DEFINITIONS);
        return;
    }

#ifdef LIBGPIOD_V2
    if (line_requests[pin] != NULL)
    {
#ifdef MY_DEBUG_VERBOSE_CORE
        logWarning("GPIODClass::pinMode: Pin %d was already reserved.\n", pin);
#endif
        gpiod_line_request_release(line_requests[pin]);
        line_requests[pin] = NULL;
    }

    struct gpiod_line_settings *settings = gpiod_line_settings_new();
    if (settings == NULL)
    {
        logError("GPIODClass::pinMode: Failed to create line settings for pin %d\n", pin);
        exit(1);
    }

    if (mode == OUTPUT)
    {
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);
    }
    else
    {
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    }

    struct gpiod_line_config *line_cfg = gpiod_line_config_new();
    if (line_cfg == NULL)
    {
        gpiod_line_settings_free(settings);
        logError("GPIODClass::pinMode: Failed to create line config for pin %d\n", pin);
        exit(1);
    }

    unsigned int offsets[] = {pin};
    if (gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings) != 0)
    {
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        logError("GPIODClass::pinMode: Failed to add line settings for pin %d\n", pin);
        exit(1);
    }

    struct gpiod_request_config *req_cfg = gpiod_request_config_new();
    if (req_cfg == NULL)
    {
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        logError("GPIODClass::pinMode: Failed to create request config for pin %d\n", pin);
        exit(1);
    }
    gpiod_request_config_set_consumer(req_cfg, "mysgw");

    line_requests[pin] = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(settings);

    if (line_requests[pin] == NULL)
    {
        logError("GPIODClass::pinMode: Failed to request line for pin %d\n", pin);
        exit(1);
    }
#else
    if (gpiod_lines[pin] != NULL)
    {
#ifdef MY_DEBUG_VERBOSE_CORE
        logWarning("GPIODClass::pinMode: Pin %d was already reserved.\n", pin);
#endif
        gpiod_line_release(gpiod_lines[pin]);
    }

    gpiod_lines[pin] = gpiod_chip_get_line(chip, pin);
    if (mode == OUTPUT)
    {
        int reqRet = gpiod_line_request_output(gpiod_lines[pin], "mysgw", 0);
        if (reqRet != 0)
        {
            logError("GPIODClass::pinMode: Failure gpiod_line_request_output for pin %d\n", pin);
            exit(1);
        }
    }
    else
    {
        if (gpiod_line_request_input(gpiod_lines[pin], "mysgw") != 0)
        {
            logError("GPIODClass::pinMode: Failure gpiod_line_request_input for pin %d\n", pin);
            exit(1);
        }
    }
#endif
}

void GPIODClass::digitalWrite(uint8_t pin, uint8_t value)
{
#ifdef LIBGPIOD_V2
    if (line_requests[pin] == NULL)
    {
        pinMode(pin, OUTPUT);
    }
    enum gpiod_line_value val = value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
    if (gpiod_line_request_set_value(line_requests[pin], pin, val) != 0)
    {
        logError("GPIODClass::digitalWrite: Failure setting pin %d to value %d\n", pin, value);
        exit(1);
    }
#else
    if (gpiod_lines[pin] == NULL)
    {
        pinMode(pin, OUTPUT);
    }
    if (gpiod_line_set_value(gpiod_lines[pin], value) != 0)
    {
        logError("GPIODClass::digitalWrite: Failure setting pin %d to value %d\n", pin, value);
        exit(1);
    }
#endif
}

uint8_t GPIODClass::digitalRead(uint8_t pin)
{
#ifdef LIBGPIOD_V2
    if (line_requests[pin] == NULL)
    {
        pinMode(pin, INPUT);
    }
    enum gpiod_line_value val = gpiod_line_request_get_value(line_requests[pin], pin);
    if (val == GPIOD_LINE_VALUE_ERROR)
    {
        logError("GPIODClass::digitalRead: Failure getting value from pin %d\n", pin);
        exit(1);
    }
    return (val == GPIOD_LINE_VALUE_ACTIVE) ? HIGH : LOW;
#else
    if (gpiod_lines[pin] == NULL)
    {
        pinMode(pin, INPUT);
    }
    uint8_t value;
    value = gpiod_line_get_value(gpiod_lines[pin]);
    if (value > 1)
    {
        logError("GPIODClass::digitalRead: Failure getting value from pin %d\n", pin);
        exit(1);
    }
    return value;
#endif
}

uint8_t GPIODClass::digitalPinToInterrupt(uint8_t pin)
{
    return pin;
}

GPIODClass &GPIODClass::operator=(const GPIODClass &other)
{
    if (this != &other)
    {
        chip = other.chip;
#ifdef LIBGPIOD_V2
        for (int i = 0; i < GPIOD_MAX_LINE_DEFINITIONS; ++i)
        {
            line_requests[i] = other.line_requests[i];
        }
#else
        for (int i = 0; i < GPIOD_MAX_LINE_DEFINITIONS; ++i)
        {
            gpiod_lines[i] = other.gpiod_lines[i];
        }
#endif
    }
    return *this;
}
