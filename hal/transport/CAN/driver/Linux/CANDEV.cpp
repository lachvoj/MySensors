#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "CANDEV.h"

// WARNING: will work only for one can if
// TODO: expand to more can interfaces
static int _s;

#ifdef MY_CAN_CANDEV_RB
typedef struct
{
    struct can_frame frames[MY_CAN_CANDEV_RB_SZ];
    struct can_frame *rdPt;
    struct can_frame *wrPt;
} CANDEV_RingBuffer_t;

static CANDEV_RingBuffer_t rxRb;

inline static void _readDataToRb()
{
    static const can_frame *rxRbLastElem = ((can_frame *)(&rxRb.frames + 1) - 1);
    while (true)
    {
        can_frame *nextWrPt = rxRb.wrPt + 1;
        // dont't read if next message will overwrite rdPt
        if (nextWrPt == rxRb.rdPt || (nextWrPt > rxRbLastElem && rxRb.rdPt == rxRb.frames))
            return;

        ssize_t nbytes = read(_s, rxRb.wrPt, sizeof(can_frame));
        if (nbytes < 1)
            return;

        rxRb.wrPt = nextWrPt;
        if (rxRb.wrPt > rxRbLastElem)
        {
            rxRb.wrPt = rxRb.frames;
        }
    }
}

inline static void _cleanRb()
{
    rxRb.rdPt = rxRb.frames;
    rxRb.wrPt = rxRb.frames;
}
#endif

#ifdef MY_CAN_CANDEV_ASYNC
// void _sigactionHandler(int val, siginfo_t *sigInfo, void *context)
// {
//     if (sigInfo->si_code != POLL_IN &&  sigInfo->si_code != POLL_MSG)
//         return;

//     _readDataToRb();
// }
inline static void _sigactionHandler(int val)
{
    _readDataToRb();
}
#endif

CANDEVClass::CANDEVClass(char *canDevice)
{
    strcpy(_canDevice, canDevice);
}

CANDEVClass::~CANDEVClass()
{
    close(_s);
}

uint8_t CANDEVClass::begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset)
{
    _s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (_s < 0)
        return CAN_FAILINIT;

    int fileFlags = fcntl(_s, F_GETFL, 0);
    if (fileFlags < 0)
        return CAN_FAILINIT;

#ifdef MY_CAN_CANDEV_ASYNC
    // struct sigaction act;
    // memset(&act, 0, sizeof(act));
    // if (sigfillset(&act.sa_mask) < 0)
    //     return CAN_FAILINIT;

    // act.sa_flags = SA_SIGINFO;
    // act.sa_sigaction = _sigactionHandler;
    // if (sigaction(SIGIO, &act, NULL) < 0)
    //     return CAN_FAILINIT;

    if (signal(SIGIO, _sigactionHandler))
        return CAN_FAILINIT;

    if (fcntl(_s, F_SETOWN, getpid()) < 0)
        return CAN_FAILINIT;

    fileFlags |= O_ASYNC;
#endif
#ifdef MY_CAN_CANDEV_RB
    _cleanRb();
#endif

    if (fcntl(_s, F_SETFL, fileFlags | O_NONBLOCK) < 0)
        return CAN_FAILINIT;

    int loopback = 0;
    if (setsockopt(_s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback)) < 0)
        return CAN_FAILINIT;

    struct ifreq ifr;
    strcpy(ifr.ifr_name, _canDevice);
    if (ioctl(_s, SIOCGIFINDEX, &ifr) < 0)
        return CAN_FAILINIT;

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(_s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
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

uint8_t CANDEVClass::sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, const uint8_t *buf)
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

uint8_t CANDEVClass::sendMsgBuf(uint32_t id, uint8_t len, const uint8_t *buf)
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
#ifndef MY_CAN_CANDEV_ASYNC
    _readDataToRb();
#endif
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
