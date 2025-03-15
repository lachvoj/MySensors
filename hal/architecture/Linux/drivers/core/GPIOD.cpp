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
    chip = gpiod_chip_open(chipdevname);
    if (chip == NULL)
    {
        logError("GPIODClass: Failed to open gpio chip %s\n", chipdevname);
        exit(1);
    }
    memset(gpiod_lines, NULL, sizeof(gpiod_lines));
}

GPIODClass::GPIODClass(const GPIODClass &other)
{
    chipdevname = other.chipdevname;
    chip = other.chip;
}

GPIODClass::~GPIODClass()
{
    gpiod_chip_close(chip);
}

void GPIODClass::pinMode(uint8_t pin, uint8_t mode)
{
    if (pin >= GPIOD_MAX_LINE_DEFINITIONS)
    {
        logError("GPIODClass::pinMode: Pin number too big: %d >= %d\n", pin, GPIOD_MAX_LINE_DEFINITIONS);
        return;
    }

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
}

void GPIODClass::digitalWrite(uint8_t pin, uint8_t value)
{
    if (gpiod_lines[pin] == NULL)
    {
        pinMode(pin, OUTPUT);
    }
    if (gpiod_line_set_value(gpiod_lines[pin], value) != 0)
    {
        logError("GPIODClass::digitalWrite: Failure setting pin %d to value %d\n", pin, value);
        exit(1);
    }
}

uint8_t GPIODClass::digitalRead(uint8_t pin)
{
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
        for (int i = 0; i < GPIOD_MAX_LINE_DEFINITIONS; ++i)
        {
            gpiod_lines[i] = other.gpiod_lines[i];
        }
    }
    return *this;
}
