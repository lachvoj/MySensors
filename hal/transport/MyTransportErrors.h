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
 * Transport Error Logging - Compile-time optional ring buffer for transport errors
 */

#ifndef MyTransportErrors_h
#define MyTransportErrors_h

#include <stdint.h>

// ============================================================================
// Transport Error Code Definitions
// ============================================================================
// Error code format: 0xTEE where T=Transport type, EE=Error code
// Transport types: 0x0=Generic, 0x1=RF24, 0x2=RFM69, 0x3=RFM95, 0x4=NRF5, 
//                  0x5=SX126x, 0x6=RS485, 0x7=PJON, 0x8=CAN

// ---- Generic Transport Errors (0x00-0x0F) ----
#define TSP_ERR_NONE                0x00    // No error
#define TSP_ERR_INIT_FAILED         0x01    // Transport initialization failed
#define TSP_ERR_SANITY_FAILED       0x02    // Sanity check failed
#define TSP_ERR_TX_FAILED           0x03    // Transmission failed
#define TSP_ERR_TX_TIMEOUT          0x04    // Transmission timeout
#define TSP_ERR_TX_NO_ACK           0x05    // No ACK received
#define TSP_ERR_RX_FAILED           0x06    // Reception failed
#define TSP_ERR_RX_OVERFLOW         0x07    // RX buffer overflow
#define TSP_ERR_RX_INVALID_LEN      0x08    // Invalid message length
#define TSP_ERR_RX_QUEUE_FULL       0x09    // RX queue full
#define TSP_ERR_HW_NOT_READY        0x0A    // Hardware not ready
#define TSP_ERR_ENCRYPTION          0x0B    // Encryption/decryption error
#define TSP_ERR_RECOVERY_OK         0x0C    // Recovery successful
#define TSP_ERR_RECOVERY_FAILED     0x0D    // Recovery failed

// ---- RF24 Specific Errors (0x10-0x1F) ----
#define TSP_ERR_RF24_INIT           0x10    // RF24 init failed
#define TSP_ERR_RF24_TX_FIFO        0x11    // TX FIFO error
#define TSP_ERR_RF24_RX_FIFO        0x12    // RX FIFO error
#define TSP_ERR_RF24_CHANNEL        0x13    // Invalid channel

// ---- RFM69 Specific Errors (0x20-0x2F) ----
#define TSP_ERR_RFM69_INIT          0x20    // RFM69 init failed
#define TSP_ERR_RFM69_SYNC          0x21    // Sync word error
#define TSP_ERR_RFM69_MODE          0x22    // Mode change error
#define TSP_ERR_RFM69_RSSI          0x23    // RSSI read error

// ---- RFM95/LoRa Specific Errors (0x30-0x3F) ----
#define TSP_ERR_RFM95_INIT          0x30    // RFM95 init failed
#define TSP_ERR_RFM95_CRC           0x31    // CRC error
#define TSP_ERR_RFM95_HEADER        0x32    // Header error
#define TSP_ERR_RFM95_CAD           0x33    // CAD failed

// ---- NRF5 ESB Specific Errors (0x40-0x4F) ----
#define TSP_ERR_NRF5_INIT           0x40    // NRF5 init failed
#define TSP_ERR_NRF5_RADIO          0x41    // Radio error

// ---- SX126x Specific Errors (0x50-0x5F) ----
#define TSP_ERR_SX126X_INIT         0x50    // SX126x init failed
#define TSP_ERR_SX126X_BUSY         0x51    // Radio busy
#define TSP_ERR_SX126X_CMD          0x52    // Command error

// ---- RS485 Specific Errors (0x60-0x6F) ----
#define TSP_ERR_RS485_INIT          0x60    // RS485 init failed
#define TSP_ERR_RS485_FRAMING       0x61    // Framing error
#define TSP_ERR_RS485_PARITY        0x62    // Parity error
#define TSP_ERR_RS485_COLLISION     0x63    // Bus collision

// ---- PJON Specific Errors (0x70-0x7F) ----
#define TSP_ERR_PJON_INIT           0x70    // PJON init failed
#define TSP_ERR_PJON_CONNECTION     0x71    // Connection error
#define TSP_ERR_PJON_CONTENT        0x72    // Content error

// ---- CAN Specific Errors (0x80-0x9F) ----
#define TSP_ERR_CAN_INIT            0x80    // CAN init failed
#define TSP_ERR_CAN_INIT_FILTER     0x81    // Filter init failed
#define TSP_ERR_CAN_TX_FAILED       0x82    // CAN TX failed
#define TSP_ERR_CAN_TX_TIMEOUT      0x83    // CAN TX timeout
#define TSP_ERR_CAN_TX_BUFFER_FULL  0x84    // TX buffer full
#define TSP_ERR_CAN_TX_ARBITRATION  0x85    // Lost arbitration
#define TSP_ERR_CAN_RX_LEN_INVALID  0x86    // Invalid frame length
#define TSP_ERR_CAN_RX_FRAME_OOB    0x87    // Frame out of bounds
#define TSP_ERR_CAN_RX_DUP_FRAME    0x88    // Duplicate frame
#define TSP_ERR_CAN_RX_LEN_OVERFLOW 0x89    // Message length overflow
#define TSP_ERR_CAN_RX_BUFFER_OVF   0x8A    // RX buffer overflow (HW)
#define TSP_ERR_CAN_BUS_OFF         0x8B    // Bus-off state
#define TSP_ERR_CAN_BUS_PASSIVE     0x8C    // Error passive state
#define TSP_ERR_CAN_BUS_WARNING     0x8D    // Error warning threshold
#define TSP_ERR_CAN_BUS_STUFF       0x8E    // Stuff error
#define TSP_ERR_CAN_BUS_FORM        0x8F    // Form error
#define TSP_ERR_CAN_BUS_CRC         0x90    // CRC error
#define TSP_ERR_CAN_BUS_BIT         0x91    // Bit error
#define TSP_ERR_CAN_SLOT_FULL       0x92    // No free slots
#define TSP_ERR_CAN_SLOT_TIMEOUT    0x93    // Slot timed out
#define TSP_ERR_CAN_SLOT_EVICTED    0x94    // Slot evicted

// ============================================================================
// Error Logging Ring Buffer (compile-time optional)
// ============================================================================
// Configuration macros MY_TRANSPORT_ERROR_LOG and MY_TRANSPORT_ERROR_LOG_SIZE
// are defined in MyConfig.h

#ifdef MY_TRANSPORT_ERROR_LOG

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Error log entry structure
 */
typedef struct {
    uint32_t timestamp;     // micros() when error occurred
    uint8_t errorCode;      // Error code from TSP_ERR_* defines
    uint8_t channel;        // Transport channel (transportChannelID_t)
    uint8_t extra;          // Extra info (TEC, REC, frame#, nodeId, etc.)
    uint8_t reserved;       // Padding for alignment / future use
} TransportErrorLogEntry_t;

/**
 * @brief Log a transport error
 * @param errorCode Error code from TSP_ERR_* defines
 * @param channel Transport channel that generated the error
 * @param extra Extra info byte (context-dependent)
 */
void transportLogError(uint8_t errorCode, uint8_t channel, uint8_t extra);

/**
 * @brief Get number of logged errors
 * @return Number of entries in error log (0 to MY_TRANSPORT_ERROR_LOG_SIZE)
 */
uint8_t transportGetErrorLogCount(void);

/**
 * @brief Get error log entry by index (0 = oldest)
 * @param index Entry index (0 to count-1)
 * @param entry Pointer to store entry data
 * @return true if entry exists, false if index out of range
 */
bool transportGetErrorLogEntry(uint8_t index, TransportErrorLogEntry_t *entry);

/**
 * @brief Clear error log
 */
void transportClearErrorLog(void);

/**
 * @brief Get most recent error code (0 if no errors)
 * @return Most recent error code or TSP_ERR_NONE
 */
uint8_t transportGetLastError(void);

/**
 * @brief Get timestamp of most recent error
 * @return Timestamp in micros() or 0 if no errors
 */
uint32_t transportGetLastErrorTimestamp(void);

/**
 * @brief Get total error count since boot (includes overwritten entries)
 * @return Total error count (may wrap around)
 */
uint32_t transportGetTotalErrorCount(void);

#ifdef __cplusplus
}
#endif

// Macro to log error (expands to nothing when disabled)
#define TRANSPORT_LOG_ERROR(code, channel, extra) transportLogError(code, channel, extra)

#else
// Error logging disabled - macros expand to nothing
#define TRANSPORT_LOG_ERROR(code, channel, extra) ((void)0)
#endif // MY_TRANSPORT_ERROR_LOG

// ============================================================================
// Helper macros for transport-specific error logging
// ============================================================================
#ifdef MY_RADIO_RF24
#define RF24_LOG_ERROR(code, extra) TRANSPORT_LOG_ERROR(code, TRANSPORT_RF24_CHANNEL_ID, extra)
#endif

#ifdef MY_RADIO_RFM69
#define RFM69_LOG_ERROR(code, extra) TRANSPORT_LOG_ERROR(code, TRANSPORT_RFM69_CHANNEL_ID, extra)
#endif

#ifdef MY_RADIO_RFM95
#define RFM95_LOG_ERROR(code, extra) TRANSPORT_LOG_ERROR(code, TRANSPORT_RFM95_CHANNEL_ID, extra)
#endif

#ifdef MY_RADIO_NRF5_ESB
#define NRF5_LOG_ERROR(code, extra) TRANSPORT_LOG_ERROR(code, TRANSPORT_NRF5_ESB_CHANNEL_ID, extra)
#endif

#ifdef MY_RADIO_SX126x
#define SX126X_LOG_ERROR(code, extra) TRANSPORT_LOG_ERROR(code, TRANSPORT_SX126x_CHANNEL_ID, extra)
#endif

#ifdef MY_RS485
#define RS485_LOG_ERROR(code, extra) TRANSPORT_LOG_ERROR(code, TRANSPORT_RS485_CHANNEL_ID, extra)
#endif

#ifdef MY_PJON
#define PJON_LOG_ERROR(code, extra) TRANSPORT_LOG_ERROR(code, TRANSPORT_PJON_CHANNEL_ID, extra)
#endif

#ifdef MY_CAN
#define CAN_LOG_ERROR(code, extra) TRANSPORT_LOG_ERROR(code, TRANSPORT_CAN_CHANNEL_ID, extra)
#endif

// Generic error logging (channel 0 = all/unknown)
#define TSP_LOG_ERROR(code, extra) TRANSPORT_LOG_ERROR(code, 0, extra)

#endif // MyTransportErrors_h
