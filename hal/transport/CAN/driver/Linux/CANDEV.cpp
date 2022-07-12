#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "CANDEV.h"

CANDEVClass::CANDEVClass(char *canDevice)
{
    strcpy(_canDevice, canDevice);
}

uint8_t CANDEVClass::begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset)
{
    struct ifreq ifr;
    struct sockaddr_can addr;

    _s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (_s < 0)
        return CAN_FAILINIT;

    strcpy(ifr.ifr_name, _canDevice);
    if (ioctl(_s, SIOCGIFINDEX, &ifr) < 0)
        return CAN_FAILINIT;
    
    int opt = 1;
    if (ioctl(_s, FIONBIO, &opt))
         return CAN_FAILINIT;

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(_s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        return CAN_FAILINIT;

    int loopback = 0;
    if (setsockopt(_s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0)
        return CAN_FAILINIT;

    _initialized = true;
    return CAN_OK;
}

uint8_t CANDEVClass::setFilterMask(can_filter *filters, uint8_t count)
{
    int ret = 0;

    errno = 0;
    if (setsockopt(_s, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(can_filter) * count) < 0)
        ret++;

    return ret;
}

uint8_t CANDEVClass::sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf)
{
    if (!_initialized)
        return CAN_FAILINIT;

    struct can_frame CAN_tx_msg;
    CAN_tx_msg.can_id = id;
    if (ext == 0)
        CAN_tx_msg.can_id &= CAN_SFF_MASK;
    else
    {
        CAN_tx_msg.can_id &= CAN_EFF_MASK;
        CAN_tx_msg.can_id |= CAN_EFF_FLAG;
    }
    if (len == 0)
    {
        CAN_tx_msg.can_id |= CAN_RTR_FLAG;
        if (write(_s, &CAN_tx_msg, sizeof(CAN_tx_msg)) < 0)
            return CAN_FAILTX;
        return CAN_OK;
    }

    uint8_t noOfFrames = len / 8;
    if (len % 8 != 0)
        noOfFrames++;

    for (uint8_t currentFrame = 0; currentFrame < noOfFrames; ++currentFrame)
    {
        CAN_tx_msg.can_dlc = (currentFrame * 8 <= len) ? 8 : len % 8;
        memcpy(CAN_tx_msg.data, buf + (currentFrame * 8), CAN_tx_msg.can_dlc);
        if (write(_s, &CAN_tx_msg, sizeof(CAN_tx_msg)) < 0)
            return CAN_FAILTX;
    }

    return CAN_OK;
}

uint8_t CANDEVClass::sendMsgBuf(uint32_t id, uint8_t len, uint8_t *buf)
{
    uint8_t ext = 0;
    if ((id & CAN_IS_EXTENDED) == CAN_IS_EXTENDED)
        ext = 1;

    return sendMsgBuf(id, ext, len, buf);
}

uint8_t CANDEVClass::readMsgBuf(uint32_t *id, uint8_t *ext, uint8_t *len, uint8_t *buf)
{
    if (!_initialized)
        return CAN_FAILINIT;

    struct can_frame CAN_rx_msg;
    ssize_t nbytes = read(_s, &CAN_rx_msg, sizeof(CAN_rx_msg));
    if (nbytes <= 0)
        return 0;

    *id = CAN_rx_msg.can_id;
    *ext = (CAN_rx_msg.can_id & CAN_EFF_FLAG) ? 1 : 0;
    *len = CAN_rx_msg.can_dlc;
    memcpy(buf, CAN_rx_msg.data, CAN_rx_msg.can_dlc);

    return nbytes;
}

uint8_t CANDEVClass::readMsgBuf(uint32_t *id, uint8_t *len, uint8_t *buf)
{
    uint8_t ext;

    return readMsgBuf(id, &ext, len, buf);
}

uint8_t CANDEVClass::checkReceive(void)
{
    int count;
    ioctl(_s, FIONREAD, &count);

    return (uint8_t)count;
}

uint8_t CANDEVClass::checkError(void)
{
    return 0;
}

uint8_t CANDEVClass::getError(void)
{
    return 0;
}

uint8_t CANDEVClass::errorCountRX(void)
{
    return 0;
}

uint8_t CANDEVClass::errorCountTX(void)
{
    return 0;
}

uint8_t CANDEVClass::enOneShotTX(void)
{
    return 0;
}

uint8_t CANDEVClass::disOneShotTX(void)
{
    return 0;
}

uint8_t CANDEVClass::abortTX(void)
{
    return 0;
}

uint8_t CANDEVClass::setGPO(uint8_t data)
{
    return 0;
}

uint8_t CANDEVClass::getGPI(void)
{
    return 0;
}
