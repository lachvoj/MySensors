#include "MyTransportCAN.h"

#if defined(ARDUINO_ARCH_STM32F1) && !defined(MCP_CAN)
#include "driver/STM32Fx/STM32FxCAN.cpp"
STM32FxCAN CAN0;
#elif defined(__linux__) && defined(MY_CAN_LINUX_CANDEV)
#include "driver/Linux/CANDEV.cpp"
CANDEVClass CAN0(MY_CAN_LINUX_CANDEV_DEVICE);
#else
#include "hal/transport/CAN/driver/mcp_can.cpp"
#include "hal/transport/CAN/driver/mcp_can.h"
MCP_CAN CAN0(MY_CAN_CS);
#endif

#if defined(MY_DEBUG_VERBOSE_CAN)
#define CAN_DEBUG(x, ...) DEBUG_OUTPUT(x, ##__VA_ARGS__) //!< Debug print
#else
#define CAN_DEBUG(x, ...) //!< DEBUG null
#endif

#ifdef MY_CAN_MAX_MSG_ID_SEND_DELAY_MS
#define MY_CAN_MAX_MSG_ID_SEND_DELAY_US MY_CAN_MAX_MSG_ID_SEND_DELAY_MS * 1000
#endif

#if (MAX_MESSAGE_SIZE > 32)
#error MAX_MESSAGE_SIZE si more than 32 this component will not work.
#endif

bool canInitialized = false;

// input buffer for raw data (from library).
uint8_t _nodeId;

// buffer element
typedef struct CAN_Slot
{
    uint8_t data[MAX_MESSAGE_SIZE];
    uint32_t timestamp;
    //search id bits 4-31 frameReceived 0-3
    union
    {
        uint32_t searchId;
        uint8_t frameReceived;
    };
    uint8_t totalFrames;
    uint8_t len;
#ifdef MY_CAN_FAST_SLOT_ACCESS
    union
    {
        CAN_Slot *next;
        CAN_Slot *newer;
    };
    union
    {
        CAN_Slot *prev;
        CAN_Slot *older;
    };
#else
    bool ready;
#endif
} CAN_Slot_t;

// assembly buffer
static CAN_Slot_t canSlots[MY_CAN_BUF_SIZE];

// free slots counter
static uint8_t freeCanSlots;

static const uint32_t findSlotMask =        0x1FFFFFF0U;
static const uint32_t fromSearchIdMask =    0x001FFFF0U;
static const uint32_t toAddrMask =          0x1FE00000U;
static const uint32_t msgIdMask =           0x00001FF0U;

#ifdef MY_CAN_FAST_SLOT_ACCESS
static CAN_Slot *searchListFirst;
static CAN_Slot *oldestReady;

static inline void _addToSearchList(CAN_Slot_t *slot)
{
    if (searchListFirst != nullptr)
    {
        searchListFirst->prev = slot;
    }
    slot->next = searchListFirst;
    searchListFirst = slot;
}

static inline void _moveFromSearchToOldestReadyList(CAN_Slot_t *slot)
{
    if (slot->prev != nullptr)
    {
        slot->prev->next = slot->next;
    }
    else
    {
        searchListFirst = slot->next;
    }
    if (slot->next != nullptr)
    {
        slot->next->prev = slot->prev;
    }
    slot->prev = nullptr;
    slot->next = nullptr;

    CAN_Slot_t *current = oldestReady;
    if (current == nullptr)
    {
        oldestReady = slot;
    }
    else
    {
        while(true)
        {
            if ((slot->searchId & fromSearchIdMask) < (current->searchId & fromSearchIdMask))
            {
                if (current->older != nullptr)
                {
                    slot->older = current->older;
                    slot->older->newer = slot;
                }
                else
                {
                    oldestReady = slot;
                }
                current->older = slot;
                slot->newer = current;

                break;
            }


            if (current->newer == nullptr)
            {
                current->newer = slot;
                slot->older = current;
                
                break;
            }

            current = current->newer;
        }
    }
}
#endif

#if defined(ARDUINO_ARCH_STM32F1) && !defined(MCP_CAN)
bool _initFilters()
{
    if (!canInitialized)
    {
        return false;
    }
    uint8_t err = 0;

    err += CAN0.setFilterMask32(0, BROADCAST_ADDRESS << 21, toAddrMask);
    err += CAN0.setFilterMask32(1, _nodeId << 21, toAddrMask);

    return err == 0;
}
#elif defined(__linux__) && defined(MY_CAN_LINUX_CANDEV)
bool _initFilters()
{
    if (!canInitialized)
    {
        return false;
    }

    uint8_t err = 0;

    struct can_filter rfilter[2];
    rfilter[0].can_id = BROADCAST_ADDRESS << 21;
    rfilter[0].can_mask = toAddrMask;
    rfilter[1].can_id = _nodeId << 21;
    rfilter[1].can_mask = toAddrMask;
    err += CAN0.setFilterMask(rfilter, 2);

    return err == 0;
}
#else
// filter incoming messages (MCP2515 feature).
bool _initFilters()
{
    if (!canInitialized)
    {
        return false;
    }
    uint8_t err = 0;
    err += CAN0.setMode(MODE_CONFIG);

    err += CAN0.init_Mask(0, 1, toAddrMask); // Init first mask. Only dest address will be used to filter messages.
    err += CAN0.init_Filt(0, 1, BROADCAST_ADDRESS << 21); // Init first filter. Accept broadcast messages.
    err += CAN0.init_Filt(1, 1, _nodeId << 21);           // Init second filter. Accept messages send to this node.
    // second mask and filters need to be set. Otherwise all messages would be accepted.
    err += CAN0.init_Mask(1, 1, 0xFFFFFFFF); // Init second mask.
    err += CAN0.init_Filt(2, 1, 0xFFFFFFFF); // Init third filter.
    err += CAN0.init_Filt(3, 1, 0xFFFFFFFF); // Init fourth filter.
    err += CAN0.init_Filt(4, 1, 0xFFFFFFFF); // Init fifth filter.
    err += CAN0.init_Filt(5, 1, 0xFFFFFFFF); // Init sixth filter.
    err += CAN0.setMode(MCP_NORMAL);
    hwPinMode(MY_CAN_INT, INPUT);
    return err == 0;
}
#endif

// clear single slot in buffer.
static inline void _cleanSlot(CAN_Slot_t *slot)
{
#ifdef MY_CAN_FAST_SLOT_ACCESS
    if (slot->prev != nullptr)
    {
        slot->prev->next = slot->next;
    }

    if (slot->next != nullptr)
    {
        slot->next->prev = slot->prev;
    }

    if (searchListFirst == slot)
    {
        searchListFirst = slot->next;
    }
    else if (oldestReady == slot)
    {
        oldestReady = slot->newer;
    }
#endif

    memset(slot, 0x0U, sizeof(CAN_Slot_t));
    freeCanSlots++;
}

// find empty slot in buffer
static inline CAN_Slot_t *_findEmptyCanSlot()
{
    CAN_Slot_t *slot = nullptr;
    for (uint8_t i = 0; i < MY_CAN_BUF_SIZE; i++)
    {
        if (canSlots[i].searchId == 0x0U)
        {
            slot = &canSlots[i];
            break;
        }
    }

    return slot;
}

static inline CAN_Slot_t *_getOldestSlot()
{
    CAN_Slot_t *slot = &canSlots[0];
    for (uint8_t i = 1; i < MY_CAN_BUF_SIZE; i++)
    {
        if (canSlots[i].timestamp > slot->timestamp)
        {
            slot = &canSlots[i];
        }
    }

    return slot;
}

static inline CAN_Slot_t *_getOldestReadySlot()
{

#ifdef MY_CAN_FAST_SLOT_ACCESS

    CAN_Slot_t *slot = oldestReady;
    if (oldestReady != nullptr)
    {
        oldestReady = slot->newer;
        slot->newer = nullptr;
        if (oldestReady != nullptr)
        {
            oldestReady->older = nullptr;
        }
    }

#else

    CAN_Slot_t *slot = nullptr;
    uint8_t i;
    for (i = 0; i < MY_CAN_BUF_SIZE; i++)
    {
        if (canSlots[i].ready)
        {
            slot = &canSlots[i];
            break;
        }
    }

    if (slot != nullptr)
    {
        for (i++; i < MY_CAN_BUF_SIZE; i++)
        {
            if (canSlots[i].ready && (canSlots[i].searchId & fromSearchIdMask) < (slot->searchId & fromSearchIdMask))
            {
                slot = &canSlots[i];
            }
        }
    }

#endif

    return slot;
}

inline void _removeTimedOutSlots()
{
    uint32_t time = (uint32_t)micros();
    for (uint8_t i = 0; i < MY_CAN_BUF_SIZE; i++)
    {
        if ((canSlots[i].searchId != 0x0U) && (time - canSlots[i].timestamp) > MY_CAN_SLOT_MAX_AGE_MS)
        {
            CAN_DEBUG(
                PSTR("!CAN:RCV:SLOT=%" PRIu8 ":SEARCHID=0x%" PRIX32 " message dropped (max age)\n"),
                i,
                canSlots[i].searchId);
            _cleanSlot(&canSlots[i]);
        }
    }
}


// find slot with previous data parts.
static inline CAN_Slot_t *_findCanSlot(const uint32_t rxId, const uint8_t currentFrameMask)
{
#ifdef MY_CAN_FAST_SLOT_ACCESS

    CAN_Slot_t *slot = searchListFirst;
    while (slot != nullptr)
    {
        if ((slot->searchId & findSlotMask) == (rxId & findSlotMask) && (slot->frameReceived & currentFrameMask) == 0)
        {
            break;
        }
        else
        {
            slot = slot->next;
        }
    }

#else

    CAN_Slot_t *slot = nullptr;
    for (uint8_t i = 0; i < MY_CAN_BUF_SIZE; i++)
    {
        if (!canSlots[i].ready && (canSlots[i].searchId & findSlotMask) == (rxId & findSlotMask) &&
            (canSlots[i].frameReceived & currentFrameMask) == 0)
        {
            slot = &canSlots[i];
            break;
        }
    }
#endif

    return slot;
}

static inline bool _checkDataAvailable()
{
#ifdef MY_CAN_FAST_SLOT_ACCESS

    return (oldestReady != nullptr);

#else

    for (uint8_t i = 0; i < MY_CAN_BUF_SIZE; i++)
    {
        if (canSlots[i].ready)
            return true;
    }

    return false;

#endif
}

// 1 bit is last flag (D)
// 3 bits current part number  (C)
// 9 bits message_id (E)
// 8 bits from address (A)
// 8 bits to address (B)
// 1 bit require ack (F)
// 1 bit is ack (G)
// header model (32 bits)
// HGFB BBBB BBBA AAAA AAAE EEEE EEEE CCCD
long unsigned int _buildHeader(
    uint16_t messageId,
    uint8_t totalFramesCount,
    uint8_t currentFrameNumber,
    uint8_t toAddress,
    uint8_t fromAddress)
{
    long unsigned int header = 0x0U; // set, G=0 (To be implemented), F=0 (To be implemented)
    header += toAddress;
    header = header << 8;
    header += fromAddress;
    header = header << 9;
    header += (messageId & 0x01FFU); // set messageId
    header = header << 3;
    header += (currentFrameNumber & 0x07U); // set current part number
    header = header << 1;
    header += ((uint8_t)(totalFramesCount - 1 == currentFrameNumber) & 0x01U); // set is last/total flag
    CAN_DEBUG(
        PSTR("CAN:SND:CANH=%" PRIu32 ",ID=%" PRIu16 ",TOTAL=%" PRIu8 ",CURR=%" PRIu8 ",TO=%" PRIu8 ",FROM=%" PRIu8 "\n"),
        header,
        messageId,
        totalFramesCount,
        currentFrameNumber,
        toAddress,
        fromAddress);
    return header;
}

void _parseHeader(
    const long unsigned int rxId,
    uint16_t *messageId,
    bool *isLast,
    uint8_t *currentFrameNumber,
    uint8_t *toAddress,
    uint8_t *fromAddress)
{
    *isLast = (bool)(rxId & 0x00000001U);
    *currentFrameNumber = (uint8_t)((rxId & 0x0000000EU) >> 1);
    *messageId = (uint16_t)((rxId & msgIdMask) >> 4);
    *fromAddress = (uint8_t)((rxId & 0x001FE000U) >> 13);
    *toAddress = (uint8_t)((rxId & toAddrMask) >> 21);
}

bool CAN_transportInit(void)
{
    CAN_DEBUG(
        PSTR("CAN:INIT:CS=%" PRIu8 ",INT=%" PRIu8 ",SPE=%" PRIu8 ",CLO=%" PRIu8 "\n"),
        MY_CAN_CS,
        MY_CAN_INT,
        MY_CAN_SPEED,
        MY_CAN_CLOCK);

    freeCanSlots = MY_CAN_BUF_SIZE;

#ifdef MY_CAN_FAST_SLOT_ACCESS
    searchListFirst = nullptr;
    oldestReady = nullptr;
#endif

    if (CAN0.begin(MCP_STDEXT, MY_CAN_SPEED, MY_CAN_CLOCK) != CAN_OK)
    {
        canInitialized = false;
        return false;
    }
    canInitialized = true;

    

    return _initFilters();
}

bool CAN_transportSend(const uint8_t to, const void *data, const uint8_t len, const bool noACK)
{
    (void)noACK; // some ack is provided by CAN itself. TODO implement application layer ack.
    const uint8_t *datap = static_cast<const uint8_t *>(data);
    // calculate number of frames
    uint8_t noOfFrames = len / CAN_MAX_CHAR_IN_MESSAGE;
    if (len % CAN_MAX_CHAR_IN_MESSAGE != 0)
    {
        noOfFrames++;
    }
    // message id updated for every outgoing mesage
    static uint16_t message_id;

#ifdef MY_CAN_MAX_MSG_ID_SEND_DELAY_MS
    static uint32_t lastMsgSentTimeStamp;

    uint32_t tstDiff = ((uint32_t)micros() - lastMsgSentTimeStamp);
    if (message_id == 0x01FF && tstDiff < MY_CAN_MAX_MSG_ID_SEND_DELAY_US)
        wait((uint32_t)(MY_CAN_MAX_MSG_ID_SEND_DELAY_US - tstDiff));

    lastMsgSentTimeStamp = (uint32_t)micros();
#endif

    // update message_id
    message_id++;
    // make sure message_id isn't longer than 9 bits.
    message_id &= 0x01FF;

    CAN_DEBUG(PSTR("CAN:SND:LN=%" PRIu8 ",NOF=%" PRIu8 "\n"), len, noOfFrames);
    uint8_t currentFrame;
    for (currentFrame = 0; currentFrame < noOfFrames; currentFrame++)
    {
        uint8_t partLen;
        if (len <= CAN_MAX_CHAR_IN_MESSAGE)
        {
            partLen = len;
        }
        else if (currentFrame * CAN_MAX_CHAR_IN_MESSAGE + CAN_MAX_CHAR_IN_MESSAGE <= len)
        {
            partLen = CAN_MAX_CHAR_IN_MESSAGE;
        }
        else
        {
            partLen = len % CAN_MAX_CHAR_IN_MESSAGE;
        }

        CAN_DEBUG(
            PSTR("CAN:SND:LN=%" PRIu8 ",DTA0=%" PRIu8 ",DTA1=%" PRIu8 ",DTA2=%" PRIu8 ",DTA3=%" PRIu8 ",DTA4=%" PRIu8
                 ",DTA5=%" PRIu8 ",DTA6=%" PRIu8 ",DTA7=%" PRIu8 "\n"),
            partLen,
            *(datap + (currentFrame * CAN_MAX_CHAR_IN_MESSAGE) + 0),
            *(datap + (currentFrame * CAN_MAX_CHAR_IN_MESSAGE) + 1),
            *(datap + (currentFrame * CAN_MAX_CHAR_IN_MESSAGE) + 2),
            *(datap + (currentFrame * CAN_MAX_CHAR_IN_MESSAGE) + 3),
            *(datap + (currentFrame * CAN_MAX_CHAR_IN_MESSAGE) + 4),
            *(datap + (currentFrame * CAN_MAX_CHAR_IN_MESSAGE) + 5),
            *(datap + (currentFrame * CAN_MAX_CHAR_IN_MESSAGE) + 6),
            *(datap + (currentFrame * CAN_MAX_CHAR_IN_MESSAGE) + 7));

        if (CAN0.sendMsgBuf(
                _buildHeader(message_id, noOfFrames, currentFrame, to, _nodeId),
                1,
                partLen,
                datap + (currentFrame * CAN_MAX_CHAR_IN_MESSAGE)) != CAN_OK)
        {
            CAN_DEBUG(PSTR("!CAN:SND:FAIL\n"));
            return false;
        }
    }

    CAN_DEBUG(PSTR("CAN:SND:OK\n"));
    return true;
}

bool CAN_transportDataAvailable(void)
{
    bool ret = false;

    long unsigned int rxId;
    uint8_t len = 0;
    uint8_t rxBuf[CAN_MAX_CHAR_IN_MESSAGE];
#if defined(ARDUINO_ARCH_STM32F1) && !defined(MCP_CAN)
    while (CAN0.checkReceive())
    {
        CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
#elif defined(__linux__) && defined(MY_CAN_LINUX_CANDEV)
    while (CAN0.readMsgBuf((uint32_t *)&rxId, &len, rxBuf) > 0)
    {
#else
    while (!hwDigitalRead(MY_CAN_INT)) // If MY_CAN_INT pin is low, read receive buffer
    {
        CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
#endif
        uint16_t messageId;
        uint8_t currentFrame, to, from;
        bool isLast;
        _parseHeader(rxId, &messageId, &isLast, &currentFrame, &to, &from);
        uint8_t currentFrameMask = 0x01U << currentFrame;
        CAN_DEBUG(
            PSTR("CAN:RCV:CANH=0X%" PRIX32 ",ID=%" PRIu8 ",ISLAST=%" PRIu8 ",CURR=%" PRIu8 ",TO=%" PRIu8 ",FROM=%" PRIu8
                 "\n"),
            rxId,
            messageId,
            isLast,
            currentFrame,
            to,
            from);
        CAN_DEBUG(
            PSTR("CAN:RCV:LN=%" PRIu8 ",DTA0=%" PRIu8 ",DTA1=%" PRIu8 ",DTA2=%" PRIu8 ",DTA3=%" PRIu8 ",DTA4=%" PRIu8
                 ",DTA5=%" PRIu8 ",DTA6=%" PRIu8 ",DTA7=%" PRIu8 "\n"),
            len,
            rxBuf[0],
            rxBuf[1],
            rxBuf[2],
            rxBuf[3],
            rxBuf[4],
            rxBuf[5],
            rxBuf[6],
            rxBuf[7]);

        CAN_Slot_t *slot = _findCanSlot(rxId, currentFrameMask);
        if (slot == nullptr)
        {
            slot = _findEmptyCanSlot();
            if (slot == nullptr)
            {
                _removeTimedOutSlots();
                slot = _findEmptyCanSlot();
                if (slot == nullptr)
                {
                    slot = _getOldestSlot();
                    CAN_DEBUG(
                        PSTR("!CAN:RCV:SLOT:FROM=%" PRIu8 ":MSGID=%" PRIu8
                             " message dropped (no space) !!!Increase MY_CAN_BUF_SIZE if you seeing this too "
                             "often!!!\n"),
                        from,
                        messageId);
                    _cleanSlot(slot);
                }
            }
            freeCanSlots--;
            slot->searchId = rxId & findSlotMask;
            slot->timestamp = (uint32_t)micros();
#ifdef MY_CAN_FAST_SLOT_ACCESS
            _addToSearchList(slot);
#endif
        }
        memcpy(slot->data + currentFrame * CAN_MAX_CHAR_IN_MESSAGE, rxBuf, len);
        slot->len += len;
        slot->frameReceived |= currentFrameMask;
        if (isLast)
            slot->totalFrames = currentFrame + 1;
        CAN_DEBUG(
            PSTR("CAN:RCV:SLOT:FROM=%" PRIu8 ":MSGID=%" PRIu8 ":PARTS=%" PRIX8 ":LEN=%" PRIu8 "\n"),
            from,
            messageId,
            slot->frameReceived & 0X0FU,
            slot->len);
        if (slot->totalFrames > 0 && ((slot->frameReceived & 0x0FU) ^ ((1 << slot->totalFrames) - 1)) == 0)
        {
#ifdef MY_CAN_FAST_SLOT_ACCESS
            _moveFromSearchToOldestReadyList(slot);
#else
            slot->ready = true;
#endif
            CAN_DEBUG(PSTR("CAN:RCV:SLOT:FROM=%" PRIu8 ":MSGID=%" PRIu8 " complete\n"), from, messageId);
            ret = true;
        }
        if (freeCanSlots < 2 && _checkDataAvailable())
        {
            ret = true;
            break;
        }
    }

    return ret;
}

void CAN_transportTask(void)
{
#if defined(MY_TRANSPORT_RX_QUEUE)
    while (CAN_transportDataAvailable())
    {
        RXQueuedMessage_t *msgIn = transportHALGetQueueBuffer();
        if (msgIn != NULL)
        {
            msgIn->channel = TRANSPORT_CAN_CHANNEL_ID;
            msgIn->length = CAN_transportReceive((void *)&msgIn->data, sizeof(msgIn->data));
            (void)transportHALPushQueueBuffer(msgIn);
        }
    }
#endif
}

uint8_t CAN_transportReceive(void *data, const uint8_t maxBufSize)
{
    CAN_Slot_t *slot = _getOldestReadySlot();

    if (slot == nullptr)
    {
        return (0);
    }

    uint8_t len = slot->len;

    memcpy(data, slot->data, len);
    _cleanSlot(slot);

    return len;
}

void CAN_transportSetAddress(const uint8_t address)
{
    if (_nodeId == address)
        return;

    _nodeId = address;
    _initFilters();
}

uint8_t CAN_transportGetAddress(void)
{
    return _nodeId;
}

bool CAN_transportSanityCheck(void)
{
    // not implemented yet
    return true;
}

void CAN_transportPowerDown(void)
{
    // Nothing to shut down here
}

void CAN_transportPowerUp(void)
{
    // not implemented
}

void CAN_transportSleep(void)
{
    // not implemented
}

void CAN_transportStandBy(void)
{
    // not implemented
}

int16_t CAN_transportGetSendingRSSI(void)
{
    // not implemented
    return INVALID_RSSI;
}

int16_t CAN_transportGetReceivingRSSI(void)
{
    // not implemented
    return INVALID_RSSI;
}

int16_t CAN_transportGetSendingSNR(void)
{
    // not implemented
    return INVALID_SNR;
}

int16_t CAN_transportGetReceivingSNR(void)
{
    // not implemented
    return INVALID_SNR;
}

int16_t CAN_transportGetTxPowerPercent(void)
{
    // not implemented
    return static_cast<int16_t>(100);
}

int16_t CAN_transportGetTxPowerLevel(void)
{
    // not implemented
    return static_cast<int16_t>(100);
}

bool CAN_transportSetTxPowerPercent(const uint8_t powerPercent)
{
    // not possible
    (void)powerPercent;
    return false;
}
