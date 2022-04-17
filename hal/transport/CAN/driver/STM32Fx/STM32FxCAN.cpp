#include <stm32_def.h>

#include "STM32FxCAN.h"

/* Symbolic names for formats of CAN message                                 */
typedef enum
{
    STANDARD_FORMAT = 0,
    EXTENDED_FORMAT
} CAN_FORMAT;

/* Symbolic names for type of CAN message                                    */
typedef enum
{
    DATA_FRAME = 0,
    REMOTE_FRAME
} CAN_FRAME;

typedef struct
{
    uint32_t id;     /* 29 bit identifier                               */
    uint8_t data[8]; /* Data field                                      */
    uint8_t len;     /* Length of data field in bytes                   */
    uint8_t ch;      /* Object channel(Not use)                         */
    uint8_t format;  /* 0 - STANDARD, 1- EXTENDED IDENTIFIER            */
    uint8_t type;    /* 0 - DATA FRAME, 1 - REMOTE FRAME                */
} CAN_msg_t;

typedef const struct
{
    uint8_t TS2;
    uint8_t TS1;
    uint8_t BRP;
} CAN_bit_timing_config_t;

CAN_bit_timing_config_t can_configs[6] = {{2, 13, 45}, {2, 15, 20}, {2, 13, 18}, {2, 13, 9}, {2, 15, 4}, {2, 15, 2}};

/**
 * Initializes the CAN filter registers.
 *
 * @preconditions   - This register can be written only when the filter initialization mode is set (FINIT=1) in the
 * CAN_FMR register.
 * @params: index   - Specified filter index. index 27:14 are available in connectivity line devices only.
 * @params: scale   - Select filter scale.
 *                    0: Dual 16-bit scale configuration
 *                    1: Single 32-bit scale configuration
 * @params: mode    - Select filter mode.
 *                    0: Two 32-bit registers of filter bank x are in Identifier Mask mode
 *                    1: Two 32-bit registers of filter bank x are in Identifier List mode
 * @params: fifo    - Select filter assigned.
 *                    0: Filter assigned to FIFO 0
 *                    1: Filter assigned to FIFO 1
 * @params: bank1   - Filter bank register 1
 * @params: bank2   - Filter bank register 2
 *
 */
uint8_t CANSetFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2)
{
    if (index > 27)
        return 1;

    CAN1->FMR |= 0x1UL; // Set to filter initialization mode

    CAN1->FA1R &= ~(0x1UL << index); // Deactivate filter

    if (scale == 0)
    {
        CAN1->FS1R &= ~(0x1UL << index); // Set filter to Dual 16-bit scale configuration
    }
    else
    {
        CAN1->FS1R |= (0x1UL << index); // Set filter to single 32 bit configuration
    }
    if (mode == 0)
    {
        CAN1->FM1R &= ~(0x1UL << index); // Set filter to Mask mode
    }
    else
    {
        CAN1->FM1R |= (0x1UL << index); // Set filter to List mode
    }

    if (fifo == 0)
    {
        CAN1->FFA1R &= ~(0x1UL << index); // Set filter assigned to FIFO 0
    }
    else
    {
        CAN1->FFA1R |= (0x1UL << index); // Set filter assigned to FIFO 1
    }

    CAN1->sFilterRegister[index].FR1 = bank1; // Set filter bank registers1
    CAN1->sFilterRegister[index].FR2 = bank2; // Set filter bank registers2

    CAN1->FA1R |= (0x1UL << index); // Activate filter

    CAN1->FMR &= ~(0x1UL); // Deactivate initialization mode

    return 0;
}

#define CAN_EXT_ID_MASK 0x1FFFFFFFU
#define CAN_STD_ID_MASK 0x000007FFU
#define STM32_CAN_TIR_TXRQ (1U << 0U) // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE (1U << 2U)  // Bit 2: Identifier Extension

/**
 * Encodes CAN messages using the CAN message struct and populates the
 * data registers with the sent.
 *
 * @params CAN_tx_msg - CAN message structure for transmission
 *
 */
uint8_t CANSend(CAN_msg_t *CAN_tx_msg)
{
    volatile int count = 0;

    uint32_t out = 0;
    if (CAN_tx_msg->format == EXTENDED_FORMAT)
    { // Extended frame format
        out = ((CAN_tx_msg->id & CAN_EXT_ID_MASK) << 3U) | STM32_CAN_TIR_IDE;
    }
    else
    { // Standard frame format
        out = ((CAN_tx_msg->id & CAN_STD_ID_MASK) << 21U);
    }

    // Remote frame
    if (CAN_tx_msg->type == REMOTE_FRAME)
    {
        out |= STM32_CAN_TIR_RTR;
    }

    CAN1->sTxMailBox[0].TDTR &= ~(0xF);
    CAN1->sTxMailBox[0].TDTR |= CAN_tx_msg->len & 0xFUL;

    CAN1->sTxMailBox[0].TDLR =
        (((uint32_t)CAN_tx_msg->data[3] << 24) | ((uint32_t)CAN_tx_msg->data[2] << 16) |
         ((uint32_t)CAN_tx_msg->data[1] << 8) | ((uint32_t)CAN_tx_msg->data[0]));
    CAN1->sTxMailBox[0].TDHR =
        (((uint32_t)CAN_tx_msg->data[7] << 24) | ((uint32_t)CAN_tx_msg->data[6] << 16) |
         ((uint32_t)CAN_tx_msg->data[5] << 8) | ((uint32_t)CAN_tx_msg->data[4]));

    // Send Go
    CAN1->sTxMailBox[0].TIR = out | STM32_CAN_TIR_TXRQ;

    // Wait until the mailbox is empty
    while (CAN1->sTxMailBox[0].TIR & 0x1UL && count++ < 1000000) {};

    // The mailbox don't becomes empty while loop
    if (CAN1->sTxMailBox[0].TIR & 0x1UL)
        return CAN_FAILTX;

    return CAN_OK;
}

STM32FxCAN::STM32FxCAN(uint8_t _CS)
{
}

bool STM32FxCAN::isInitialized()
{
    if (_initialized)
        return _initialized;

    begin(0, _speed);

    return _initialized;
}

uint8_t STM32FxCAN::begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset)
{
    uint8_t bitrate;
    _speed = speedset;
    switch (speedset)
    {
    case CAN_50KBPS: bitrate = 0; break;
    case CAN_100KBPS: bitrate = 1; break;
    case CAN_125KBPS: bitrate = 2; break;
    case CAN_250KBPS: bitrate = 3; break;
    case CAN_500KBPS: bitrate = 4; break;
    case CAN_1000KBPS: bitrate = 5; break;

    default: return CAN_FAILINIT; break;
    }

    RCC->APB1ENR |= 0x2000000UL; // Enable CAN clock
    RCC->APB2ENR |= 0x1UL;       // Enable AFIO clock
    AFIO->MAPR &= 0xFFFF9FFF;    // reset CAN remap
                                 // CAN_RX mapped to PA11, CAN_TX mapped to PA12

#if (STM32FxCAN_BUS_TYPE == 0)
    {
        RCC->APB2ENR |= 0x4UL;      // Enable GPIOA clock
        GPIOA->CRH &= ~(0xFF000UL); // Configure PA12(0b0000) and PA11(0b0000)
                                    // 0b0000
                                    //   MODE=00(Input mode)
                                    //   CNF=00(Analog mode)

        GPIOA->CRH |= 0xB8FFFUL; // Configure PA12(0b1011) and PA11(0b1000)
                                 // 0b1011
                                 //   MODE=11(Output mode, max speed 50 MHz)
                                 //   CNF=10(Alternate function output Push-pull
                                 // 0b1000
                                 //   MODE=00(Input mode)
                                 //   CNF=10(Input with pull-up / pull-down)

#if (STM32FxCAN_BUS_PULLUP > 0)
        GPIOA->ODR |= 0x1UL << 12; // PA12 Upll-up
#endif
    }
#endif

#if (STM32FxCAN_BUS_TYPE == 2)
    {
        AFIO->MAPR |= 0x00004000; // set CAN remap
                                  // CAN_RX mapped to PB8, CAN_TX mapped to PB9
                                  // (not available on 36-pin package)

        RCC->APB2ENR |= 0x8UL;   // Enable GPIOB clock
        GPIOB->CRH &= ~(0xFFUL); // Configure PB9(0b0000) and PB8(0b0000)
                                 // 0b0000
                                 //   MODE=00(Input mode)
                                 //   CNF=00(Analog mode)

        GPIOB->CRH |= 0xB8UL; // Configure PB9(0b1011) and PB8(0b1000)
                              // 0b1011
                              //   MODE=11(Output mode, max speed 50 MHz)
                              //   CNF=10(Alternate function output Push-pull
                              // 0b1000
                              //   MODE=00(Input mode)
                              //   CNF=10(Input with pull-up / pull-down)
#if (STM32FxCAN_BUS_PULLUP > 0)
        GPIOB->ODR |= 0x1UL << 8; // PB8 Upll-up
#endif
    }
#endif

#if (STM32FxCAN_BUS_TYPE == 3)
    {
        AFIO->MAPR |= 0x00005000; // set CAN remap
                                  // CAN_RX mapped to PD0, CAN_TX mapped to PD1
                                  // (available on 100-pin and 144-pin package)

        RCC->APB2ENR |= 0x20UL;  // Enable GPIOD clock
        GPIOD->CRL &= ~(0xFFUL); // Configure PD1(0b0000) and PD0(0b0000)
                                 // 0b0000
                                 //   MODE=00(Input mode)
                                 //   CNF=00(Analog mode)

        GPIOD->CRH |= 0xB8UL; // Configure PD1(0b1011) and PD0(0b1000)
                              // 0b1000
                              //   MODE=00(Input mode)
                              //   CNF=10(Input with pull-up / pull-down)
                              // 0b1011
                              //   MODE=11(Output mode, max speed 50 MHz)
                              //   CNF=10(Alternate function output Push-pull
#if (STM32FxCAN_BUS_PULLUP > 0)
        GPIOD->ODR |= 0x1UL << 0; // PD0 Upll-up
#endif
    }
#endif

    CAN1->MCR |= 0x1UL; // Require CAN1 to Initialization mode
    while (!(CAN1->MSR & 0x1UL))
        ; // Wait for Initialization mode

    // CAN1->MCR = 0x51UL;                 // Hardware initialization(No
    // automatic retransmission)
    CAN1->MCR = 0x41UL; // Hardware initialization(With automatic retransmission)

    // Set bit rates
    CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF));
    CAN1->BTR |= (((can_configs[bitrate].TS2 - 1) & 0x07) << 20) | (((can_configs[bitrate].TS1 - 1) & 0x0F) << 16) |
                 ((can_configs[bitrate].BRP - 1) & 0x1FF);

    // Configure Filters to default values
    CAN1->FMR |= 0x1UL;      // Set to filter initialization mode
    CAN1->FMR &= 0xFFFFC0FF; // Clear CAN2 start bank

    // bxCAN has 28 filters.
    // These filters are used for both CAN1 and CAN2.
    // STM32F103 has only CAN1, so all 28 are used for CAN1
    CAN1->FMR |= 0x1C << 8; // Assign all filters to CAN1

    // Set filter 0
    // Single 32-bit scale configuration
    // Two 32-bit registers of filter bank x are in Identifier Mask mode
    // Filter assigned to FIFO 0
    // Filter bank register to all 0
    CANSetFilter(0, 1, 0, 0, 0x0UL, 0x0UL);

    CAN1->FMR &= ~(0x1UL); // Deactivate initialization mode

    uint16_t TimeoutMilliseconds = 1000;
    bool can1 = false;
    CAN1->MCR &= ~(0x1UL); // Require CAN1 to normal mode

    // Wait for normal mode
    // If the connection is not correct, it will not return to normal mode.
    for (uint16_t wait_ack = 0; wait_ack < TimeoutMilliseconds; wait_ack++)
    {
        if ((CAN1->MSR & 0x1UL) == 0)
        {
            can1 = true;
            break;
        }
        delayMicroseconds(1000);
    }

    if (!can1)
        return CAN_FAILINIT;

    _initialized = true;
    return CAN_OK;
}

uint8_t STM32FxCAN::init_Mask(uint8_t num, uint8_t ext, uint32_t ulData)
{
    _actualMask = ulData;
    return 0;
}

uint8_t STM32FxCAN::init_Mask(uint8_t num, uint32_t ulData)
{
    return init_Mask(num, 1, ulData);
}

uint8_t STM32FxCAN::init_Filt(uint8_t num, uint8_t ext, uint32_t ulData)
{
    switch (ext)
    {
    case 0:
    case 1: break;

    default: return CAN_FAILINIT;
    }

    CANSetFilter(num, ext, 0, 0, _actualMask, ulData);

    return CAN_OK;
}

uint8_t STM32FxCAN::init_Filt(uint8_t num, uint32_t ulData)
{
    uint8_t ext = 0;
    if ((ulData & CAN_IS_EXTENDED) == CAN_IS_EXTENDED)
        ext = 1;

    return init_Filt(num, ext, ulData);
}

uint8_t STM32FxCAN::setMode(uint8_t opMode)
{
    return 0;
}

uint8_t STM32FxCAN::sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf)
{
    if (!isInitialized())
        return CAN_FAILINIT;

    CAN_msg_t CAN_tx_msg;
    CAN_tx_msg.id = id;
    CAN_tx_msg.format = (ext == 0) ? STANDARD_FORMAT : EXTENDED_FORMAT;
    if (len == 0)
    {
        CAN_tx_msg.type = REMOTE_FRAME;
        return CANSend(&CAN_tx_msg);
    }
    CAN_tx_msg.type = DATA_FRAME;

    uint8_t lenM1 = len - 1;
    for (uint8_t i = 0; i < len; ++i)
    {
        uint8_t imod8 = i % 8;
        CAN_tx_msg.data[imod8] = buf[i];

        if (imod8 == 7 || i == lenM1)
        {
            if (i == lenM1)
                CAN_tx_msg.len = imod8;
            else
                CAN_tx_msg.len = 8;
            if (CANSend(&CAN_tx_msg) != CAN_OK)
                return CAN_FAILTX;
        }
    }

    return CAN_OK;
}

uint8_t STM32FxCAN::sendMsgBuf(uint32_t id, uint8_t len, uint8_t *buf)
{
    uint8_t ext = 0;
    if ((id & CAN_IS_EXTENDED) == CAN_IS_EXTENDED)
        ext = 1;

    return sendMsgBuf(id, ext, len, buf);
}

uint8_t STM32FxCAN::readMsgBuf(uint32_t *id, uint8_t *ext, uint8_t *len, uint8_t *buf)
{
    if (!isInitialized())
        return CAN_FAILINIT;

    *id = CAN1->sFIFOMailBox[0].RIR;
    if ((*id & STM32_CAN_RIR_IDE) == 0)
        *ext = 0;
    else
        *ext = 1;

    *len = (CAN1->sFIFOMailBox[0].RDTR) & 0xFUL;
    buf[0] = 0xFFUL & CAN1->sFIFOMailBox[0].RDLR;
    buf[1] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 8);
    buf[2] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 16);
    buf[3] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDLR >> 24);
    buf[4] = 0xFFUL & CAN1->sFIFOMailBox[0].RDHR;
    buf[5] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 8);
    buf[6] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 16);
    buf[7] = 0xFFUL & (CAN1->sFIFOMailBox[0].RDHR >> 24);

    // Release FIFO 0 output mailbox.
    // Make the next incoming message available.
    CAN1->RF0R |= 0x20UL;

    return CAN_OK;
}

uint8_t STM32FxCAN::readMsgBuf(uint32_t *id, uint8_t *len, uint8_t *buf)
{
    uint8_t ext;

    return readMsgBuf(id, &ext, len, buf);
}

uint8_t STM32FxCAN::checkReceive(void)
{
    // Check for pending FIFO 0 messages
    return CAN1->RF0R & 0x3UL;
}

uint8_t STM32FxCAN::checkError(void)
{
    return 0;
}

uint8_t STM32FxCAN::getError(void)
{
    return 0;
}

uint8_t STM32FxCAN::errorCountRX(void)
{
    return 0;
}

uint8_t STM32FxCAN::errorCountTX(void)
{
    return 0;
}

uint8_t STM32FxCAN::enOneShotTX(void)
{
    return 0;
}

uint8_t STM32FxCAN::disOneShotTX(void)
{
    return 0;
}

uint8_t STM32FxCAN::abortTX(void)
{
    return 0;
}

uint8_t STM32FxCAN::setGPO(uint8_t data)
{
    return 0;
}

uint8_t STM32FxCAN::getGPI(void)
{
    return 0;
}
