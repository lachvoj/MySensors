#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "CANDEV.h"

// WARNING: will work only for one can if
// TODO: expand to more can interfaces

#ifndef MY_CAN_CANDEV_RB_SZ
#define MY_CAN_CANDEV_RB_SZ (CAN_BUF_SIZE * (MAX_MESSAGE_SIZE / CAN_MAX_CHAR_IN_MESSAGE)) * 100
#endif

static int _s;

#ifdef MY_CAN_CANDEV_RB
typedef struct
{
    struct can_frame frames[MY_CAN_CANDEV_RB_SZ];
    struct can_frame *rdPt;
    struct can_frame *wrPt;
} CANDEV_RingBuffer_t;

static CANDEV_RingBuffer_t rxRb;

void irqHandle(int val)
{
    ssize_t nbytes;
    while (nbytes = read(_s, rxRb.wrPt, sizeof(can_frame)) > 0)
    {
        rxRb.wrPt++;
        if (rxRb.wrPt == rxRb.rdPt)
        {
            // Drop newest message
            rxRb.wrPt--;
        }
        if (rxRb.wrPt > ((can_frame *)(&rxRb.frames + 1) - 1))
        {
            rxRb.wrPt = rxRb.frames;
        }
    }
}
#endif

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

    // int opt = 1;
    // if (ioctl(_s, FIONBIO, &opt))
    //     return CAN_FAILINIT;

    int fileFlags = fcntl(_s, F_GETFL, 0);
    if (fileFlags < 0)
        return CAN_FAILINIT;

    if (fcntl(_s, F_SETFL, fileFlags | O_NONBLOCK) < 0)
        return CAN_FAILINIT;

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(_s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        return CAN_FAILINIT;

    int loopback = 0;
    if (setsockopt(_s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0)
        return CAN_FAILINIT;

    _initialized = true;

#ifdef MY_CAN_CANDEV_RB
    if (signal(SIGIO, irqHandle) == SIG_ERR)
        return CAN_FAILINIT;

    if (fcntl(_s, F_SETOWN, getpid()) < 0)
        return CAN_FAILINIT;

    fileFlags = fcntl(_s, F_GETFL, 0);
    if (fileFlags < 0)
        return CAN_FAILINIT;

    if (fcntl(_s, F_SETFL, fileFlags | FNDELAY | FASYNC) < 0)
        return CAN_FAILINIT;
#endif

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

    if (len > CAN_MAX_CHAR_IN_MESSAGE)
        return CAN_FAILTX;

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
    }

    CAN_tx_msg.can_dlc = len;
    memcpy(CAN_tx_msg.data, buf, len);
    if (write(_s, &CAN_tx_msg, sizeof(CAN_tx_msg)) < 0)
        return CAN_FAILTX;

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
    
    struct can_frame *CAN_rx_msg;

#ifdef MY_CAN_CANDEV_RB
    if (rxRb.rdPt == rxRb.wrPt)
        return 0;

    CAN_rx_msg = rxRb.rdPt;
#else
    struct can_frame CAN_rx_msg_buf;
    ssize_t nbytes = read(_s, &CAN_rx_msg_buf, sizeof(can_frame));
    if (nbytes <= 0)
        return 0;

    CAN_rx_msg = &CAN_rx_msg_buf;
#endif

    *id = CAN_rx_msg->can_id;
    *ext = (CAN_rx_msg->can_id & CAN_EFF_FLAG) ? 1 : 0;
    *len = CAN_rx_msg->can_dlc;
    memcpy(buf, CAN_rx_msg->data, CAN_rx_msg->can_dlc);

#ifdef MY_CAN_CANDEV_RB
    rxRb.rdPt++;
    if (rxRb.rdPt > ((can_frame *)(&rxRb.frames + 1) - 1))
    {
        rxRb.rdPt = rxRb.frames;
    }
#endif

    return CAN_rx_msg->can_dlc;
}

uint8_t CANDEVClass::readMsgBuf(uint32_t *id, uint8_t *len, uint8_t *buf)
{
    uint8_t ext;

    return readMsgBuf(id, &ext, len, buf);
}

uint8_t CANDEVClass::checkReceive(void)
{
    return 0;
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
