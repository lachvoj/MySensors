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
 */

/**
 * @file MyHwSTM32.cpp
 * @brief Hardware abstraction layer for STM32 microcontrollers using STM32duino core
 *
 * This implementation uses the official STM32duino Arduino core which provides
 * STM32Cube HAL underneath. It supports a wide range of STM32 families including
 * F0, F1, F4, L0, L4, G0, G4, H7, and more.
 *
 * Tested on:
 * - STM32F401CC/CE Black Pill
 * - STM32F411CE Black Pill
 *
 * Pin Mapping Example (STM32F4 Black Pill):
 *
 * nRF24L01+ Radio (SPI1):
 * - SCK:  PA5
 * - MISO: PA6
 * - MOSI: PA7
 * - CSN:  PA4
 * - CE:   PB0 (configurable via MY_RF24_CE_PIN)
 *
 * RFM69/RFM95 Radio (SPI1):
 * - SCK:  PA5
 * - MISO: PA6
 * - MOSI: PA7
 * - CS:   PA4
 * - IRQ:  PA3 (configurable)
 * - RST:  PA2 (configurable)
 */

#include "MyHwSTM32.h"

// ============================================================================
// Lightweight ADC using STM32 LL (Low-Level) inline functions
// All LL_ADC_* functions are __STATIC_INLINE - no .c file linkage!
// Avoids linking HAL ADC driver (~1KB savings vs analogRead/HAL)
//
// Used by: hwCPUTemperature(), hwCPUVoltage(), hwAnalogRead()
// ============================================================================

// Include LL headers (inline functions only, no .c linkage)
// These provide LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_CHANNEL_VREFINT and all LL_ADC_* functions
#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"

// ADC initialization state
static bool s_bAdcInitialized = false;

/**
 * @brief Initialize ADC1 using LL (one-time setup)
 * Enables clock, powers on ADC, waits for stabilization, and calibrates.
 * Safe to call multiple times - will only initialize once.
 */
static void hwADCInit(void)
{
    // Enable ADC1 peripheral clock
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

    // Enable ADC
    LL_ADC_Enable(ADC1);

    // Wait for ADC stabilization (tSTAB ~1µs)
    for (volatile uint32_t i = 0; i < 72; i++) {} // ~1µs at 72MHz

    // Calibrate ADC (required for STM32F1)
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) {}

    s_bAdcInitialized = true;
}

/**
 * @brief Convert Arduino pin to LL ADC channel using framework's digitalPinToAnalogInput()
 * @param pin Arduino pin number (PA0, PA1, etc.)
 * @return LL_ADC_CHANNEL_x constant, or LL_ADC_CHANNEL_0 if invalid
 * @note Uses __LL_ADC_DECIMAL_NB_TO_CHANNEL macro from LL header
 */
static uint32_t hwPinToADCChannel(uint8_t pin)
{
    uint32_t analogIdx = digitalPinToAnalogInput(pin);
    if (analogIdx >= NUM_ANALOG_INPUTS)
    {
        return LL_ADC_CHANNEL_0; // Invalid pin, fallback
    }
    return __LL_ADC_DECIMAL_NB_TO_CHANNEL(analogIdx);
}

/**
 * @brief Read ADC channel using LL inline functions (internal implementation)
 * @param channel LL_ADC_CHANNEL_x constant
 * @param enableInternalPath true for temp sensor/vrefint, false for GPIO
 * @return 12-bit ADC value
 */
static uint16_t hwReadADCChannel(uint32_t channel, bool enableInternalPath)
{
    // Ensure ADC is initialized

    if (!s_bAdcInitialized)
    {
        hwADCInit();
    }

    // Enable internal paths if needed (temp sensor + vrefint)
    if (enableInternalPath)
    {
        LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);
    }

    // Configure sampling time for the channel (239.5 cycles for accuracy)
    LL_ADC_SetChannelSamplingTime(ADC1, channel, LL_ADC_SAMPLINGTIME_239CYCLES_5);

    // Configure regular channel sequence: 1 conversion
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, channel);

    // Set software trigger (required for STM32F1)
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);

    // Start conversion
    LL_ADC_REG_StartConversionSWStart(ADC1);

    // Wait for conversion complete
    // Note: On STM32F1, LL_ADC_IsActiveFlag_EOS checks the EOC bit (no separate EOS on F1)
    while (!LL_ADC_IsActiveFlag_EOS(ADC1)) {}

    // Read result
    uint16_t result = LL_ADC_REG_ReadConversionData12(ADC1);

    // Clear flag
    LL_ADC_ClearFlag_EOS(ADC1);

    // Disable temp sensor path to save power
    if (enableInternalPath)
    {
        LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);
    }

    return result;
}

#if defined(TEMP_SENSOR_AVAILABLE) || defined(VREF_AVAILABLE)
/**
 * @brief Read internal ADC channel (temp sensor or vrefint)
 */
static uint16_t hwReadInternalADC(uint32_t channel)
{
    return hwReadADCChannel(channel, true);
}
#endif

/**
 * @brief Read external GPIO ADC pin using LL (no HAL dependency)
 * @param pin Arduino pin number (PA0, PA1, etc.)
 * @return 12-bit ADC value (0-4095)
 *
 * This function uses STM32 LL library to read ADC without linking HAL ADC.
 * Can be used by pinCfgC AnalogWrapper via MySensorsWrapper.
 */
uint16_t hwAnalogRead(uint8_t pin)
{
    uint32_t channel = hwPinToADCChannel(pin);
    return hwReadADCChannel(channel, false);
}

bool hwInit(void)
{
#if !defined(MY_DISABLED_SERIAL)
    MY_SERIALDEVICE.begin(MY_BAUD_RATE);
#if defined(MY_GATEWAY_SERIAL)
    // Wait for serial port to connect (needed for native USB)
    while (!MY_SERIALDEVICE)
    {
        ; // Wait for serial port connection
    }
#endif
#endif

    // STM32duino EEPROM library auto-initializes on first use
    // No explicit initialization required
    return true;
}

void hwReadConfigBlock(void *buf, void *addr, size_t length)
{
    uint8_t *dst = static_cast<uint8_t *>(buf);
    int pos = reinterpret_cast<int>(addr);

    for (size_t i = 0; i < length; i++)
    {
        dst[i] = EEPROM.read(pos + i);
    }
}

void hwWriteConfigBlock(void *buf, void *addr, size_t length)
{
    uint8_t *src = static_cast<uint8_t *>(buf);
    int pos = reinterpret_cast<int>(addr);

    for (size_t i = 0; i < length; i++)
    {
        EEPROM.update(pos + i, src[i]);
    }
}

uint8_t hwReadConfig(const int addr)
{
    return EEPROM.read(addr);
}

void hwWriteConfig(const int addr, uint8_t value)
{
    EEPROM.update(addr, value);
}

void hwWatchdogReset(void)
{
#if defined(HAL_IWDG_MODULE_ENABLED) && defined(IWDG)
    // Reset independent watchdog if enabled
    // Use direct register write to reload watchdog counter
    // This works whether IWDG was initialized by HAL or LL drivers
    IWDG->KR = IWDG_KEY_RELOAD;
#endif
}

void hwReboot(void)
{
    NVIC_SystemReset();
}

void hwRandomNumberInit(void)
{
    // Use internal temperature sensor and ADC noise as entropy source
    // This provides reasonably good random seed values

#ifdef ADC1
    uint32_t seed = 0;

    // Read multiple samples from different sources for entropy
    for (uint8_t i = 0; i < 32; i++)
    {
        uint32_t value = 0;

#ifdef TEMP_SENSOR_AVAILABLE
        // Try to read internal temperature sensor if available
        // Use LL inline functions to avoid HAL ADC dependency
        value ^= hwReadInternalADC(LL_ADC_CHANNEL_TEMPSENSOR);
#endif

        // Mix in current time
        value ^= hwMillis();

        // Mix in system tick
        value ^= micros();

        // Accumulate into seed
        seed ^= (value & 0x7) << (i % 29);

        // Small delay to ensure values change
        delayMicroseconds(100);
    }

    randomSeed(seed);
#else
    // Fallback: use millis as weak entropy source
    randomSeed(hwMillis());
#endif // ADC1
}

bool hwUniqueID(unique_id_t *uniqueID)
{
#ifdef UID_BASE
    // STM32 unique device ID is stored at a fixed address
    // Length is 96 bits (12 bytes) but we store 16 bytes for compatibility
    (void)memcpy((uint8_t *)uniqueID, (uint32_t *)UID_BASE, 12);
    (void)memset(static_cast<void *>(uniqueID + 12), MY_HWID_PADDING_BYTE, 4); // padding
    return true;
#else
    // Unique ID not available on this variant
    return false;
#endif
}

uint16_t hwCPUVoltage(void)
{
#if defined(VREF_AVAILABLE) && defined(ADC1)
    // Read internal voltage reference using LL (no HAL ADC dependency)
    // VREFINT is typically 1.2V (1200mV) at all temperatures

    uint32_t vrefint = hwReadInternalADC(LL_ADC_CHANNEL_VREFINT);

    if (vrefint > 0)
    {
        // Calculate VDD in millivolts
        // Formula: VDD = VREFINT_CAL_VREF * VREFINT_CAL / vrefint_reading
        // For STM32F1: VREFINT ≈ 1.2V, measured at 3.3V
        // VDD = 1200mV * 4096 / vrefint_reading (assuming 12-bit, 3.3V ref during cal)
        return (uint16_t)((1200UL * 4096UL) / vrefint);
    }
#endif

    // Return typical 3.3V if measurement not available
    return 3300;
}

uint16_t hwCPUFrequency(void)
{
    // Return CPU frequency in 0.1 MHz units
    // F_CPU is defined by the build system (e.g., 84000000 for 84 MHz)
    return F_CPU / 100000UL;
}

int8_t hwCPUTemperature(void)
{
#if defined(TEMP_SENSOR_AVAILABLE) && defined(ADC1)
    // Read internal temperature sensor using LL inline functions
    // Bypasses HAL ADC to save ~1KB flash

    int32_t temp_raw = hwReadInternalADC(LL_ADC_CHANNEL_TEMPSENSOR);

#ifdef TEMP110_CAL_ADDR
    // Use factory calibration if available (STM32F4, L4, etc.)
    uint16_t *temp30_cal = (uint16_t *)TEMP30_CAL_ADDR;
    uint16_t *temp110_cal = (uint16_t *)TEMP110_CAL_ADDR;

    if (temp30_cal && temp110_cal && *temp110_cal != *temp30_cal)
    {
        // Calculate temperature using two-point calibration
        // Formula: T = ((110-30) / (CAL_110 - CAL_30)) * (raw - CAL_30) + 30
        int32_t temp = 30 + ((110 - 30) * (temp_raw - *temp30_cal)) / (*temp110_cal - *temp30_cal);

        // Apply user calibration
        temp = (temp - MY_STM32_TEMPERATURE_OFFSET) / MY_STM32_TEMPERATURE_GAIN;

        return (int8_t)temp;
    }
#endif // TEMP110_CAL_ADDR

    // Fallback: use typical values from STM32F1 datasheet (no factory calibration available)
    // STM32F1 specs: V25 = 1.43V (1430mV), Avg_Slope = 4.3 mV/°C (4300 µV/°C)
    //
    // Use ST LL macro: __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(slope_uV, V25_mV, cal_temp, vdd_mV, adc, resolution)
    // Note: Using fixed 3300mV for VDD - variation is minor compared to ±5°C chip tolerance
    int32_t temp = __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(4300, 1430, 25, 3300, temp_raw, LL_ADC_RESOLUTION_12B);

    return (int8_t)((temp - MY_STM32_TEMPERATURE_OFFSET) / MY_STM32_TEMPERATURE_GAIN);
#else
    // Temperature sensor not available
    return FUNCTION_NOT_SUPPORTED;
#endif
}

uint16_t hwFreeMem(void)
{
    // Calculate free heap memory
    // This uses newlib's mallinfo if available

#ifdef STACK_TOP
    extern char *__brkval;
    extern char __heap_start;

    char *heap_end = __brkval ? __brkval : &__heap_start;
    char stack_var;

    // Calculate space between heap and stack
    return (uint16_t)(&stack_var - heap_end);
#else
    // Alternative method: try to allocate and measure
    // Not implemented to avoid fragmentation
    return FUNCTION_NOT_SUPPORTED;
#endif
}

int8_t hwSleep(uint32_t ms)
{
    // TODO: Implement low-power sleep mode
    // For now, use simple delay
    // Future: Use STM32 STOP or STANDBY mode with RTC wakeup

    (void)ms;
    return MY_SLEEP_NOT_POSSIBLE;
}

int8_t hwSleep(const uint8_t interrupt, const uint8_t mode, uint32_t ms)
{
    // TODO: Implement interrupt-based sleep
    // Future: Configure EXTI and enter STOP mode

    (void)interrupt;
    (void)mode;
    (void)ms;
    return MY_SLEEP_NOT_POSSIBLE;
}

int8_t hwSleep(
    const uint8_t interrupt1,
    const uint8_t mode1,
    const uint8_t interrupt2,
    const uint8_t mode2,
    uint32_t ms)
{
    // TODO: Implement dual-interrupt sleep

    (void)interrupt1;
    (void)mode1;
    (void)interrupt2;
    (void)mode2;
    (void)ms;
    return MY_SLEEP_NOT_POSSIBLE;
}
