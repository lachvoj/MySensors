#include "STM32FxCAN.h"

#define CAN_EXT_ID_MASK 0x1FFFFFFFU
#define CAN_STD_ID_MASK 0x000007FFU
#define STM32_CAN_TIR_TXRQ (1U << 0U) // Bit 0: Transmit Mailbox Request
#define STM32_CAN_RIR_RTR (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE (1U << 2U)  // Bit 2: Identifier Extension
#define STM32_CAN_TIR_RTR (1U << 1U)  // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE (1U << 2U)  // Bit 2: Identifier Extension

STM32FxCAN::STM32FxCAN(uint8_t canDevice)
{
    switch (canDevice)
    {
    case 1: _canDev = CAN1; break;
    }
}

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
uint8_t STM32FxCAN::setFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2)
{
    if (index > 27)
        return CAN_FAILINIT;

    _canDev->FMR |= 0x1UL; // Set to filter initialization mode

    _canDev->FA1R &= ~(0x1UL << index); // Deactivate filter

    if (scale == 0)
    {
        _canDev->FS1R &= ~(0x1UL << index); // Set filter to Dual 16-bit scale configuration
    }
    else
    {
        _canDev->FS1R |= (0x1UL << index); // Set filter to single 32 bit configuration
    }
    if (mode == 0)
    {
        _canDev->FM1R &= ~(0x1UL << index); // Set filter to Mask mode
    }
    else
    {
        _canDev->FM1R |= (0x1UL << index); // Set filter to List mode
    }

    if (fifo == 0)
    {
        _canDev->FFA1R &= ~(0x1UL << index); // Set filter assigned to FIFO 0
    }
    else
    {
        _canDev->FFA1R |= (0x1UL << index); // Set filter assigned to FIFO 1
    }

    _canDev->sFilterRegister[index].FR1 = bank1; // Set filter bank registers1
    _canDev->sFilterRegister[index].FR2 = bank2; // Set filter bank registers2

    _canDev->FA1R |= (0x1UL << index); // Activate filter

    _canDev->FMR &= ~(0x1UL); // Deactivate initialization mode

    return CAN_OK;
}

bool STM32FxCAN::isInitialized()
{
    if (_initialized)
        return _initialized;

    begin(0, _speed);

    return _initialized;
}

typedef const struct
{
    uint8_t TS2;
    uint8_t TS1;
    uint8_t BRP;
} CAN_bit_timing_config_t;

static const CAN_bit_timing_config_t can_configs[6] =
    {{2, 13, 45}, {2, 15, 20}, {2, 13, 18}, {2, 13, 9}, {2, 15, 4}, {2, 15, 2}};

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

    _canDev->MCR |= 0x1UL; // Require CAN1 to Initialization mode
    while (!(_canDev->MSR & 0x1UL))
        ; // Wait for Initialization mode

    // _canDev->MCR = 0x51UL;  // Hardware initialization(No automatic retransmission)
    _canDev->MCR = 0x41UL; // Hardware initialization(With automatic retransmission)

    // _canDev->MCR |= 0x4UL; //Tx FIFO priority by request order

    // Set bit rates
    _canDev->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF));
    _canDev->BTR |= (((can_configs[bitrate].TS2 - 1) & 0x07) << 20) | (((can_configs[bitrate].TS1 - 1) & 0x0F) << 16) |
                    ((can_configs[bitrate].BRP - 1) & 0x1FF);

    // Configure Filters to default values
    _canDev->FMR |= 0x1UL;      // Set to filter initialization mode
    _canDev->FMR &= 0xFFFFC0FF; // Clear CAN2 start bank

    // bxCAN has 28 filters.
    // These filters are used for both CAN1 and CAN2.
    // STM32F103 has only CAN1, so all 28 are used for CAN1
    _canDev->FMR |= 0x1C << 8; // Assign all filters to CAN1

    // Set filter 0
    // Single 32-bit scale configuration
    // Two 32-bit registers of filter bank x are in Identifier Mask mode
    // Filter assigned to FIFO 0
    // Filter bank register to all 0
    setFilter(0, 1, 0, 0, 0x0UL, 0x0UL);

    _canDev->FMR &= ~(0x1UL); // Deactivate initialization mode

    uint16_t TimeoutMilliseconds = 1000;
    bool can1 = false;
    _canDev->MCR &= ~(0x1UL); // Require CAN1 to normal mode

    // Wait for normal mode
    // If the connection is not correct, it will not return to normal mode.
    for (uint16_t wait_ack = 0; wait_ack < TimeoutMilliseconds; wait_ack++)
    {
        if ((_canDev->MSR & 0x1UL) == 0)
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

uint8_t STM32FxCAN::setFilterMask32(uint8_t index, uint32_t filter, uint32_t mask, uint8_t fifo)
{
    return setFilter(index, 1, 0, fifo, (filter << 3U), (mask << 3U));
}

uint8_t STM32FxCAN::setFilterList32(uint8_t index, uint32_t filter1, uint32_t filter2, uint8_t fifo)
{
    return setFilter(index, 1, 1, fifo, (filter1 << 3U), (filter2 << 3U));
}

uint8_t STM32FxCAN::setFilterMask16(
    uint8_t index,
    uint16_t filter1,
    uint16_t mask1,
    uint16_t filter2,
    uint16_t mask2,
    uint8_t fifo)
{
    uint32_t f1 = (mask1 << 21) | (filter1 << 5);
    uint32_t f2 = (mask2 << 21) | (filter2 << 5);

    return setFilter(index, 0, 0, fifo, f1, f2);
}

uint8_t STM32FxCAN::setFilterList16(
    uint8_t index,
    uint16_t filter1,
    uint16_t filter2,
    uint16_t filter3,
    uint16_t filter4,
    uint8_t fifo)
{
    uint32_t f1 = (filter2 << 21) | (filter1 << 5);
    uint32_t f2 = (filter4 << 21) | (filter3 << 5);

    return setFilter(index, 0, 1, fifo, f1, f2);
}

uint8_t STM32FxCAN::setMode(uint8_t opMode)
{
    return 0;
}

uint8_t STM32FxCAN::sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, const uint8_t *buf)
{
    if (!isInitialized() || len > CAN_MAX_CHAR_IN_MESSAGE)
    {
        return CAN_FAILINIT;
    }

    // get free mailbox
    uint8_t mbIdx = 0;
    volatile int count = 0;
    while (true)
    {
        // any of mailboxes becomes empty while loop
        if (count >= 1000000)
            return CAN_FAILTX;
        count++;

        uint8_t tme = ((_canDev->TSR & 0x1C000000UL) >> 26);
        if ((tme & 0x01U) == 1)
        {
            mbIdx = 0;
            break;
        }
        if ((tme & 0x02U) == 2)
        {
            mbIdx = 1;
            break;
        }
        if ((tme & 0x04U) == 4)
        {
            mbIdx = 2;
            break;
        }
    }

    uint32_t out = 0;
    if (ext != 0)
    { // Extended frame format
        out = ((id & CAN_EXT_ID_MASK) << 3U) | STM32_CAN_TIR_IDE;
    }
    else
    { // Standard frame format
        out = ((id & CAN_STD_ID_MASK) << 21U);
    }

    // Remote frame
    if (len == 0)
    {
        out |= STM32_CAN_TIR_RTR;
    }

    _canDev->sTxMailBox[mbIdx].TDTR &= ~(0xF);
    _canDev->sTxMailBox[mbIdx].TDTR |= len & 0xFUL;

    _canDev->sTxMailBox[mbIdx].TDLR =
        (((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[0]));
    _canDev->sTxMailBox[mbIdx].TDHR =
        (((uint32_t)buf[7] << 24) | ((uint32_t)buf[6] << 16) | ((uint32_t)buf[5] << 8) | ((uint32_t)buf[4]));

    // Send Go
    _canDev->sTxMailBox[mbIdx].TIR = out | STM32_CAN_TIR_TXRQ;

    return CAN_OK;
}

uint8_t STM32FxCAN::sendMsgBuf(uint32_t id, uint8_t len, const uint8_t *buf)
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

    *ext = ((_canDev->sFIFOMailBox[0].RIR & STM32_CAN_RIR_IDE) >> 2);
    if (*ext)
        *id = _canDev->sFIFOMailBox[0].RIR >> 3; // extended id
    else
        *id = _canDev->sFIFOMailBox[0].RIR >> 21; // standard id

    *len = (_canDev->sFIFOMailBox[0].RDTR) & 0xFUL;
    ((uint32_t *)buf)[0] = _canDev->sFIFOMailBox[0].RDLR;
    ((uint32_t *)buf)[1] = _canDev->sFIFOMailBox[0].RDHR;

    // Release FIFO 0 output mailbox.
    // Make the next incoming message available.
    _canDev->RF0R |= 0x20UL;

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
    return _canDev->RF0R & 0x3UL;
}

uint8_t STM32FxCAN::checkError(void)
{
    return 0;
}

uint8_t STM32FxCAN::getError(void)
{
    return (uint8_t)((_canDev->ESR & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos);
}

uint8_t STM32FxCAN::errorCountRX(void)
{
    return (uint8_t)((_canDev->ESR & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos);
}

uint8_t STM32FxCAN::errorCountTX(void)
{
    return (uint8_t)((_canDev->ESR & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos);
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

uint8_t STM32FxCAN::enableRxInterrupt(void)
{
    _canDev->IER |= CAN_IER_FMPIE0;
    return 0;
}

uint8_t STM32FxCAN::disableRxInterrupt(void)
{
    _canDev->IER &= ~(CAN_IER_FMPIE0);

    return 0;
}

#define MMIO32(x) (*(volatile uint32_t *)(x))
uint8_t STM32FxCAN::attachRxInterrupt(void func())
{
    static uint8_t newTbl[0xF0] __attribute__((aligned(0x100)));
    uint8_t *pNewTbl = newTbl;
    int origTbl = MMIO32(SCB_BASE + 0x008);
    for (int j = 0; j < 0x3c; j++) // table length = 60 integers
        MMIO32((pNewTbl + (j << 2))) = MMIO32((origTbl + (j << 2)));

    uint32_t canVectTblAdr = reinterpret_cast<uint32_t>(pNewTbl) + (36 << 2); // calc new ISR addr in new vector tbl
    MMIO32(canVectTblAdr) = reinterpret_cast<uint32_t>(func);       // set new CAN/USB ISR jump addr into new table
    MMIO32(SCB_BASE + 0x008) = reinterpret_cast<uint32_t>(pNewTbl); // load vtor register with new tbl location

    return 0;
}
