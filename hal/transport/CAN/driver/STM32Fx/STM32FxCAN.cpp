#include "STM32FxCAN.h"

// ============================================================================
// Most CAN register definitions come from CMSIS headers (stm32f103xb.h):
//   CAN_MCR_*, CAN_MSR_*, CAN_TSR_*, CAN_RF0R_*, CAN_BTR_*, CAN_FMR_*,
//   CAN_IER_*, CAN_ESR_*, RCC_APB*ENR_*, AFIO_MAPR_CAN_REMAP_*
// The following are driver-specific definitions not in CMSIS.
// ============================================================================

// CAN ID masks (29-bit extended, 11-bit standard)
#define CAN_EXT_ID_MASK         0x1FFFFFFFU
#define CAN_STD_ID_MASK         0x000007FFU

// CAN TIR (Transmit Identifier Register) bits - not in CMSIS
#define STM32_CAN_TIR_TXRQ      (1U << 0U)      // Bit 0: Transmit Mailbox Request
#define STM32_CAN_TIR_RTR       (1U << 1U)      // Bit 1: Remote Transmission Request
#define STM32_CAN_TIR_IDE       (1U << 2U)      // Bit 2: Identifier Extension

// CAN RIR (Receive Identifier Register) bits - not in CMSIS
#define STM32_CAN_RIR_RTR       (1U << 1U)      // Bit 1: Remote Transmission Request
#define STM32_CAN_RIR_IDE       (1U << 2U)      // Bit 2: Identifier Extension

// Timeout values
#define CAN_INIT_TIMEOUT        (10000U)        // Init mode timeout iterations
#define CAN_TX_TIMEOUT_MS       (100U)          // TX timeout in milliseconds
#define CAN_NORMAL_TIMEOUT_MS   (1000U)         // Normal mode timeout in milliseconds

// Composite mask for clearing BTR timing fields (using CMSIS definitions)
#define CAN_BTR_TIMING_CLEAR    (CAN_BTR_SJW | CAN_BTR_TS2 | CAN_BTR_TS1 | CAN_BTR_BRP)

// Filter configuration constants
#define CAN_MAX_FILTER_INDEX    (27U)           // Maximum filter bank index (0-27)
#define CAN_FILTER_BANK_ALL_CAN1 (28U)          // Assign all 28 filter banks to CAN1

// GPIO configuration values for CAN pins
// CRH/CRL: 4 bits per pin [CNF1:CNF0:MODE1:MODE0]
// For TX: AF push-pull, 50MHz = CNF=10, MODE=11 = 0xB
// For RX: Input with pull-up/down = CNF=10, MODE=00 = 0x8
#define GPIO_CAN_TX_CONFIG      (0xBUL)         // AF push-pull, 50MHz
#define GPIO_CAN_RX_CONFIG      (0x8UL)         // Input with pull-up/down

// GPIO pin bit positions in CRH/CRL (4 bits per pin)
#define GPIO_PIN_POS(pin)       ((pin) * 4U)    // Calculate bit position for pin 0-7

// CAN data length code mask
#define CAN_DLC_MASK            (0xFUL)         // DLC is 4 bits (0-15, valid 0-8)

// CAN RX interrupt callback (weak symbol approach for C6 fix)
static void (*_canRxCallback)(void) = nullptr;

STM32FxCAN::STM32FxCAN(uint8_t canDevice)
{
    switch (canDevice)
    {
    case 1: _canDev = CAN1; break;
    default: _canDev = CAN1; break;  // Safe default for invalid device number
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
    if (index > CAN_MAX_FILTER_INDEX)
        return CAN_FAILINIT;

    _canDev->FMR |= CAN_FMR_FINIT; // Set to filter initialization mode

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

    _canDev->FMR &= ~CAN_FMR_FINIT; // Deactivate initialization mode

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

    default: return CAN_FAILINIT;
    }

    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;   // Enable CAN clock
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   // Enable AFIO clock
    AFIO->MAPR &= ~AFIO_MAPR_CAN_REMAP;   // Reset CAN remap (PA11/PA12 default)

#if (STM32FxCAN_BUS_TYPE == 0)
    {
        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // Enable GPIOA clock
        // Clear config for PA11 (RX) and PA12 (TX) in CRH (pins 8-15, so PA11=bits 12-15, PA12=bits 16-19)
        GPIOA->CRH &= ~(0xFFUL << GPIO_PIN_POS(11 - 8));
        // PA12: TX - AF push-pull 50MHz (0xB), PA11: RX - Input with pull-up/down (0x8)
        GPIOA->CRH |= (GPIO_CAN_TX_CONFIG << GPIO_PIN_POS(12 - 8)) | (GPIO_CAN_RX_CONFIG << GPIO_PIN_POS(11 - 8));

#if (STM32FxCAN_BUS_PULLUP > 0)
        GPIOA->ODR |= (1UL << 12);  // PA12 Pull-up
#endif
    }
#endif

#if (STM32FxCAN_BUS_TYPE == 2)
    {
        AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP2;  // CAN remap to PB8/PB9 (partial remap)
                                                    // (not available on 36-pin package)

        RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;  // Enable GPIOB clock
        // Clear config for PB8 (RX) and PB9 (TX) in CRH (pins 8-15, so PB8=bits 0-3, PB9=bits 4-7)
        GPIOB->CRH &= ~(0xFFUL << GPIO_PIN_POS(8 - 8));
        // PB9: TX - AF push-pull 50MHz (0xB), PB8: RX - Input with pull-up/down (0x8)
        GPIOB->CRH |= (GPIO_CAN_TX_CONFIG << GPIO_PIN_POS(9 - 8)) | (GPIO_CAN_RX_CONFIG << GPIO_PIN_POS(8 - 8));

#if (STM32FxCAN_BUS_PULLUP > 0)
        GPIOB->ODR |= (1UL << 8);  // PB8 Pull-up
#endif
    }
#endif

#if (STM32FxCAN_BUS_TYPE == 3)
    {
        AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP3;  // CAN remap to PD0/PD1 (full remap)
                                                    // (available on 100-pin and 144-pin package)

        RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;  // Enable GPIOD clock
        // Clear config for PD0 (RX) and PD1 (TX) in CRL (pins 0-7)
        GPIOD->CRL &= ~(0xFFUL << GPIO_PIN_POS(0));
        // PD1: TX - AF push-pull 50MHz (0xB), PD0: RX - Input with pull-up/down (0x8)
        GPIOD->CRL |= (GPIO_CAN_TX_CONFIG << GPIO_PIN_POS(1)) | (GPIO_CAN_RX_CONFIG << GPIO_PIN_POS(0));

#if (STM32FxCAN_BUS_PULLUP > 0)
        GPIOD->ODR |= (1UL << 0);  // PD0 Pull-up
#endif
    }
#endif

    _canDev->MCR |= CAN_MCR_INRQ; // Require CAN1 to Initialization mode
    
    // Wait for Initialization mode with timeout
    uint32_t initTimeout = CAN_INIT_TIMEOUT;
    while (!(_canDev->MSR & CAN_MSR_INAK) && --initTimeout)
        ;
    if (initTimeout == 0)
        return CAN_FAILINIT;  // Timeout waiting for init mode

    // Hardware initialization

    // _canDev->MCR = 0x41UL; // Hardware initialization(With automatic retransmission)

    // 0x80UL Time triggered comunication mode

    // 0x40UL Automatic buss-off reset

    // ~(0x10UL) Automatic retransmission

    // 0x04UL Tx FIFO priority by request order

    _canDev->MCR = (CAN_MCR_INRQ | CAN_MCR_ABOM | CAN_MCR_TXFP);

    // Set bit rates - clear then set TS2, TS1, and BRP fields
    _canDev->BTR &= ~CAN_BTR_TIMING_CLEAR;
    _canDev->BTR |= (((can_configs[bitrate].TS2 - 1) & 0x07) << CAN_BTR_TS2_Pos) | 
                    (((can_configs[bitrate].TS1 - 1) & 0x0F) << CAN_BTR_TS1_Pos) |
                    ((can_configs[bitrate].BRP - 1) & CAN_BTR_BRP);

    // Configure Filters to default values
    _canDev->FMR |= CAN_FMR_FINIT;        // Set to filter initialization mode
    _canDev->FMR &= ~CAN_FMR_CAN2SB;      // Clear CAN2 start bank

    // bxCAN has 28 filters.
    // These filters are used for both CAN1 and CAN2.
    // STM32F103 has only CAN1, so all 28 are used for CAN1
    _canDev->FMR |= (CAN_FILTER_BANK_ALL_CAN1 << CAN_FMR_CAN2SB_Pos);

    // Set filter 0
    // Single 32-bit scale configuration
    // Two 32-bit registers of filter bank x are in Identifier Mask mode
    // Filter assigned to FIFO 0
    // Filter bank register to all 0
    setFilter(0, 1, 0, 0, 0x0UL, 0x0UL);

    _canDev->FMR &= ~CAN_FMR_FINIT; // Deactivate initialization mode

    uint16_t timeoutMs = CAN_NORMAL_TIMEOUT_MS;
    bool can1 = false;
    _canDev->MCR &= ~CAN_MCR_INRQ; // Require CAN1 to normal mode

    // Wait for normal mode
    // If the connection is not correct, it will not return to normal mode.
    for (uint16_t wait_ack = 0; wait_ack < timeoutMs; wait_ack++)
    {
        if ((_canDev->MSR & CAN_MSR_INAK) == 0)
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

    // Build CAN ID field
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

    // Build TX data registers - only read bytes up to len to avoid buffer over-read
    uint32_t tdlr = 0, tdhr = 0;
    for (uint8_t i = 0; i < len && i < 4; i++)
        tdlr |= ((uint32_t)buf[i]) << (i * 8);
    for (uint8_t i = 4; i < len && i < 8; i++)
        tdhr |= ((uint32_t)buf[i]) << ((i - 4) * 8);

    // Wait for free mailbox with timeout
    // Note: No critical section needed - TX mailboxes are independent from RX,
    // and only main loop context calls sendMsgBuf() (no ISR sends)
    uint8_t mbIdx = 0;
    uint32_t startTime = millis();
    while (true)
    {
        if ((millis() - startTime) >= CAN_TX_TIMEOUT_MS)
        {
            return CAN_FAILTX;
        }

        uint8_t tme = ((_canDev->TSR & CAN_TSR_TME) >> CAN_TSR_TME_Pos);
        if (tme & (CAN_TSR_TME0 >> CAN_TSR_TME_Pos))
        {
            mbIdx = 0;
            break;
        }
        if (tme & (CAN_TSR_TME1 >> CAN_TSR_TME_Pos))
        {
            mbIdx = 1;
            break;
        }
        if (tme & (CAN_TSR_TME2 >> CAN_TSR_TME_Pos))
        {
            mbIdx = 2;
            break;
        }
        
        // No mailbox free, yield briefly before retry
        delayMicroseconds(10);
    }

    // Populate and send - writing TIR with TXRQ atomically claims the mailbox
    _canDev->sTxMailBox[mbIdx].TDTR &= ~CAN_DLC_MASK;
    _canDev->sTxMailBox[mbIdx].TDTR |= len & CAN_DLC_MASK;
    _canDev->sTxMailBox[mbIdx].TDLR = tdlr;
    _canDev->sTxMailBox[mbIdx].TDHR = tdhr;
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
    // Validate pointers
    if (!id || !ext || !len || !buf)
        return CAN_FAILINIT;
    
    if (!isInitialized())
        return CAN_FAILINIT;

    // Check if FIFO has messages (FMP0 bits)
    if ((_canDev->RF0R & CAN_RF0R_FMP0) == 0)
        return CAN_NOMSG;  // FIFO empty

    *ext = ((_canDev->sFIFOMailBox[0].RIR & STM32_CAN_RIR_IDE) >> 2);
    if (*ext)
        *id = _canDev->sFIFOMailBox[0].RIR >> 3; // extended id
    else
        *id = _canDev->sFIFOMailBox[0].RIR >> 21; // standard id

    // Get DLC and clamp to max valid CAN length
    uint8_t dlc = (_canDev->sFIFOMailBox[0].RDTR) & CAN_DLC_MASK;
    *len = (dlc > CAN_MAX_CHAR_IN_MESSAGE) ? CAN_MAX_CHAR_IN_MESSAGE : dlc;
    
    // Safe byte-by-byte copy to avoid alignment issues and buffer overflow
    uint32_t rdlr = _canDev->sFIFOMailBox[0].RDLR;
    uint32_t rdhr = _canDev->sFIFOMailBox[0].RDHR;
    for (uint8_t i = 0; i < *len && i < 4; i++)
        buf[i] = (rdlr >> (i * 8)) & 0xFF;
    for (uint8_t i = 4; i < *len && i < 8; i++)
        buf[i] = (rdhr >> ((i - 4) * 8)) & 0xFF;

    // Release FIFO 0 output mailbox.
    // Make the next incoming message available.
    _canDev->RF0R |= CAN_RF0R_RFOM0;

    return CAN_OK;
}

uint8_t STM32FxCAN::readMsgBuf(uint32_t *id, uint8_t *len, uint8_t *buf)
{
    uint8_t ext;

    return readMsgBuf(id, &ext, len, buf);
}

uint8_t STM32FxCAN::checkReceive(void)
{
    uint32_t rf0r = _canDev->RF0R;
    
    // Check and clear FIFO overflow flag (FOVR0)
    if (rf0r & CAN_RF0R_FOVR0)
    {
        _canDev->RF0R = CAN_RF0R_FOVR0;  // Clear overflow flag by writing 1
        // Note: One or more messages were lost due to FIFO overflow
    }
    
    // Return pending message count (FMP0 bits)
    return rf0r & CAN_RF0R_FMP0;
}

uint8_t STM32FxCAN::checkError(void)
{
    uint32_t esr = _canDev->ESR;
    
    // Check for critical error conditions:
    // - BOFF: Bus-off state
    // - EPVF: Error passive flag  
    // - EWGF: Error warning flag
    if (esr & (CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_EWGF))
        return 1;
    
    // Check for last error code (LEC != 0 means an error occurred)
    if (esr & CAN_ESR_LEC_Msk)
        return 1;
    
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
    // Enable one-shot mode by setting NART (No Automatic Retransmission) bit
    _canDev->MCR |= CAN_MCR_NART;
    return CAN_OK;
}

uint8_t STM32FxCAN::disOneShotTX(void)
{
    // Disable one-shot mode by clearing NART bit (enable auto retransmission)
    _canDev->MCR &= ~CAN_MCR_NART;
    return CAN_OK;
}

uint8_t STM32FxCAN::abortTX(void)
{
    // Request abort for all three TX mailboxes
    _canDev->TSR |= (CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2);
    return CAN_OK;
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
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5);  // Set appropriate priority
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    return CAN_OK;
}

uint8_t STM32FxCAN::disableRxInterrupt(void)
{
    NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    _canDev->IER &= ~(CAN_IER_FMPIE0);
    return CAN_OK;
}

/**
 * @brief Attach a callback function to the CAN RX interrupt
 * @param func Pointer to the callback function
 * @return CAN_OK on success
 * 
 * @note This function uses a callback pointer approach instead of direct vector
 *       table manipulation for improved safety. The actual ISR handler
 *       (USB_LP_CAN1_RX0_IRQHandler) should be defined elsewhere and call
 *       the registered callback.
 * 
 * @warning On STM32F103, the CAN RX interrupt (USB_LP_CAN1_RX0_IRQn) is shared
 *          with USB LP. If USB is also used, additional handling is required.
 */
uint8_t STM32FxCAN::attachRxInterrupt(void (*func)(void))
{
    // Register the callback
    // Note: Single pointer write is atomic on ARM Cortex-M, and this is
    // typically called during setup before interrupts are enabled anyway
    _canRxCallback = func;
    
    return CAN_OK;
}

/**
 * @brief Get the registered CAN RX callback function
 * @return Pointer to the callback function, or nullptr if none registered
 * 
 * @note This function can be called from the ISR to invoke the registered callback:
 * @code
 * extern "C" void USB_LP_CAN1_RX0_IRQHandler(void) {
 *     void (*callback)(void) = STM32FxCAN::getRxCallback();
 *     if (callback) callback();
 * }
 * @endcode
 */
void (*STM32FxCAN::getRxCallback(void))(void)
{
    return _canRxCallback;
}

// Provide a weak default ISR that calls the registered callback
// This can be overridden by user code if needed
extern "C" __attribute__((weak)) void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if (_canRxCallback)
    {
        _canRxCallback();
    }
}
