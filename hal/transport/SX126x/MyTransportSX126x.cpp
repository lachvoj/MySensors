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

#include "hal/transport/SX126x/driver/SX126x.h"

bool SX126x_transportInit(void)
{
	const bool result = SX126x_initialise();
#if !defined(MY_GATEWAY_FEATURE) && !defined(MY_SX126x_ATC_MODE_DISABLED)
	SX126x_setATC(true, SX126x_TARGET_RSSI);
#endif
	return result;
}

void SX126x_transportSetAddress(const uint8_t address)
{
	SX126x_setAddress(address);
}

uint8_t SX126x_transportGetAddress(void)
{
	return SX126x_getAddress();
}

bool SX126x_transportSend(const uint8_t to, const void *data, const uint8_t len, const bool noACK)
{
	return SX126x_sendWithRetry(to, data, len, noACK);
}

bool SX126x_transportDataAvailable(void)
{
	SX126x_handle();
	return SX126x_packetAvailable();
}

void SX126x_transportTask(void)
{
#if defined(MY_TRANSPORT_RX_QUEUE)
    while (SX126x_transportDataAvailable())
    {
        RXQueuedMessage_t *msgIn = transportHALGetQueueBuffer();
        if (msgIn != NULL)
        {
            msgIn->channel = TRANSPORT_SX126x_CHANNEL_ID;
            msgIn->length = SX126x_transportReceive((void *)&msgIn->data, sizeof(msgIn->data));
            (void)transportHALPushQueueBuffer(msgIn);
        }
    }
#endif
}

bool SX126x_transportSanityCheck(void)
{
	return SX126x_sanityCheck();
}

uint8_t SX126x_transportReceive(void *data)
{
	return SX126x_getData(static_cast<uint8_t *>(data), MAX_MESSAGE_SIZE);
}

void SX126x_transportSleep(void)
{
	SX126x_sleep();
}

void SX126x_transportStandBy(void)
{
	SX126x_standBy();
}

void SX126x_transportPowerDown(void)
{
	SX126x_powerDown();
}

void SX126x_transportPowerUp(void)
{
	SX126x_powerUp();
}

void SX126x_transportToggleATCmode(const bool OnOff, const int16_t targetRSSI)
{
	SX126x_setATC(OnOff, targetRSSI);
}

int16_t SX126x_transportGetSendingRSSI(void)
{
	return SX126x_getSendingRSSI();
}

int16_t SX126x_transportGetReceivingRSSI(void)
{
	return SX126x_getReceivingRSSI();
}

int16_t SX126x_transportGetSendingSNR(void)
{
	return SX126x_getSendingSNR();
}

int16_t SX126x_transportGetReceivingSNR(void)
{
	return SX126x_getReceivingSNR();
}

int16_t SX126x_transportGetTxPowerPercent(void)
{
	return SX126x_getTxPowerPercent();
}

int16_t SX126x_transportGetTxPowerLevel(void)
{
	return SX126x_getTxPowerLevel();
}

bool SX126x_transportSetTxPowerPercent(const uint8_t powerPercent)
{
	return SX126x_setTxPowerPercent(powerPercent);
}
