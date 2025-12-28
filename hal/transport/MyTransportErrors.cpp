/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2025 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * Transport Error Logging Implementation
 */

#include "MyTransportErrors.h"

#ifdef MY_TRANSPORT_ERROR_LOG

#if defined(ARDUINO)
#include <Arduino.h>
#elif defined(__linux__)
// Linux: millis() is provided by MySensors' Arduino compatibility layer
// (hal/architecture/Linux/drivers/core/Arduino.h)
#else
#error "Unsupported platform for MY_TRANSPORT_ERROR_LOG"
#endif

// Ring buffer storage
static TransportErrorLogEntry_t _errorLog[MY_TRANSPORT_ERROR_LOG_SIZE];
static uint8_t _errorLogHead = 0;       // Next write position
static uint8_t _errorLogCount = 0;      // Number of valid entries (0 to SIZE)
static uint32_t _totalErrorCount = 0;   // Total errors logged (including overwritten)

void transportLogError(uint8_t errorCode, uint8_t channel, uint8_t extra)
{
    _errorLog[_errorLogHead].timestamp = millis();
    _errorLog[_errorLogHead].errorCode = errorCode;
    _errorLog[_errorLogHead].channel = channel;
    _errorLog[_errorLogHead].extra = extra;
    _errorLog[_errorLogHead].reserved = 0;
    
    _errorLogHead = (_errorLogHead + 1) % MY_TRANSPORT_ERROR_LOG_SIZE;
    
    if (_errorLogCount < MY_TRANSPORT_ERROR_LOG_SIZE) {
        _errorLogCount++;
    }
    
    _totalErrorCount++;  // Always increment (will wrap at 2^32)
}

uint8_t transportGetErrorLogCount(void)
{
    return _errorLogCount;
}

bool transportGetErrorLogEntry(uint8_t index, TransportErrorLogEntry_t *entry)
{
    if (index >= _errorLogCount || entry == NULL) {
        return false;
    }
    
    // Calculate actual position in ring buffer
    uint8_t pos;
    if (_errorLogCount < MY_TRANSPORT_ERROR_LOG_SIZE) {
        // Buffer not full yet, entries start at 0
        pos = index;
    } else {
        // Buffer is full, oldest entry is at _errorLogHead
        pos = (_errorLogHead + index) % MY_TRANSPORT_ERROR_LOG_SIZE;
    }
    
    *entry = _errorLog[pos];
    return true;
}

void transportClearErrorLog(void)
{
    _errorLogHead = 0;
    _errorLogCount = 0;
    // Note: _totalErrorCount is NOT cleared - it's a lifetime counter
}

uint8_t transportGetLastError(void)
{
    if (_errorLogCount == 0) {
        return TSP_ERR_NONE;
    }
    
    uint8_t lastPos = (_errorLogHead == 0) ? (MY_TRANSPORT_ERROR_LOG_SIZE - 1) : (_errorLogHead - 1);
    return _errorLog[lastPos].errorCode;
}

uint32_t transportGetLastErrorTimestamp(void)
{
    if (_errorLogCount == 0) {
        return 0;
    }
    
    uint8_t lastPos = (_errorLogHead == 0) ? (MY_TRANSPORT_ERROR_LOG_SIZE - 1) : (_errorLogHead - 1);
    return _errorLog[lastPos].timestamp;
}

uint32_t transportGetTotalErrorCount(void)
{
    return _totalErrorCount;
}

#endif // MY_TRANSPORT_ERROR_LOG
