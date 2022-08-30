#include "CANDEV.h"

#ifndef TEST_MSG_BUF_SZ
#define TEST_MSG_BUF_SZ 10000
#endif

static can_frame canMsgBuf[TEST_MSG_BUF_SZ];
static uint32_t canMsgBufWrPt;
static uint32_t canMsgBufRdPt;

CANDEVClass::CANDEVClass(const char *canDevice)
{}

CANDEVClass::~CANDEVClass()
{}

uint8_t CANDEVClass::begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset)
{
    return 0;
}

uint8_t CANDEVClass::setFilterMask(can_filter *filters, uint8_t count)
{
    return 0;
}

uint8_t CANDEVClass::sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, const uint8_t *buf)
{
    canMsgBuf[canMsgBufWrPt].can_id = id;
    canMsgBuf[canMsgBufWrPt].can_dlc = len;
    memcpy(canMsgBuf[canMsgBufWrPt].data, buf, len);

    canMsgBufWrPt++;
    if (canMsgBufWrPt >= TEST_MSG_BUF_SZ)
        canMsgBufWrPt = 0;

    return CAN_OK;
}

uint8_t CANDEVClass::readMsgBuf(uint32_t *id, uint8_t *len, uint8_t *buf)
{
    *id = canMsgBuf[canMsgBufRdPt].can_id;
    *len = canMsgBuf[canMsgBufRdPt].can_dlc;
    memcpy(buf, canMsgBuf[canMsgBufRdPt].data, canMsgBuf[canMsgBufRdPt].can_dlc);

    if (*len != 0)
        canMsgBufRdPt++;

    if (canMsgBufRdPt >= TEST_MSG_BUF_SZ)
        canMsgBufRdPt = 0;

    return *len;
}
