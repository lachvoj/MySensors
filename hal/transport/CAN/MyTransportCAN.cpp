#include "MyTransportCAN.h"
#include "hal/transport/MyTransportErrors.h"

#if defined(ARDUINO_ARCH_STM32) && !defined(MCP_CAN)
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

// Critical section macros for protecting shared slot data structures
// Memory barriers ensure pending memory accesses complete before/after critical section
#if defined(ARDUINO_ARCH_STM32) && !defined(MCP_CAN)
// STM32: Use NVIC to disable only CAN RX interrupt (finer-grained than noInterrupts)
// __DSB() ensures all memory accesses complete, __ISB() flushes pipeline
#define CAN_ENTER_CRITICAL()  do { NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn); __DSB(); __ISB(); } while(0)
#define CAN_EXIT_CRITICAL()   do { __DSB(); NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn); } while(0)
#elif defined(__linux__) && defined(MY_CAN_LINUX_CANDEV)
// Linux: No interrupt context, critical sections not needed
#define CAN_ENTER_CRITICAL()  do {} while(0)
#define CAN_EXIT_CRITICAL()   do {} while(0)
#else
// MCP2515 and others: Use global interrupt disable with memory barrier
#define CAN_ENTER_CRITICAL()  do { noInterrupts(); __asm__ __volatile__("" ::: "memory"); } while(0)
#define CAN_EXIT_CRITICAL()   do { __asm__ __volatile__("" ::: "memory"); interrupts(); } while(0)
#endif

// Convert slot max age from milliseconds to microseconds for comparison with micros()
#define MY_CAN_SLOT_MAX_AGE_US  (MY_CAN_SLOT_MAX_AGE_MS * 1000UL)

// Maximum number of CAN frames per message (32 bytes / 8 bytes per frame = 4 frames max)
#define CAN_MAX_FRAMES  ((MAX_MESSAGE_SIZE + CAN_MAX_CHAR_IN_MESSAGE - 1) / CAN_MAX_CHAR_IN_MESSAGE)

#if (MAX_MESSAGE_SIZE > 32)
#error MAX_MESSAGE_SIZE is more than 32 this component will not work.
#endif

// Error counter threshold for warning
#define CAN_ERROR_WARNING_THRESHOLD 96

// Bus-off recovery backoff constants
#define CAN_BUSOFF_INITIAL_BACKOFF_MS   100   // Initial retry delay
#define CAN_BUSOFF_MAX_BACKOFF_MS       5000  // Maximum retry delay (5 seconds)
#define CAN_BUSOFF_BACKOFF_MULTIPLIER   2     // Exponential backoff multiplier

// Bus-off recovery state (volatile - accessed by error check which may be called from ISR)
static volatile uint32_t busOffBackoffMs = 0;          // Current backoff delay
static volatile uint32_t busOffLastAttemptMs = 0;      // Timestamp of last recovery attempt
static volatile bool busOffRecoveryPending = false;    // Recovery in progress

// CAN error state enumeration
typedef enum {
    CAN_ERR_NONE = 0,      // No error, operational
    CAN_ERR_BUS_OFF,       // Bus-off state (TEC >= 256)
    CAN_ERR_PASSIVE,       // Error passive (TEC/REC >= 128)
    CAN_ERR_WARNING,       // High error count warning
    CAN_ERR_RX_OVERFLOW    // RX buffer overflow
} CAN_ErrorState_t;

static volatile bool canInitialized = false;

// input buffer for raw data (from library).
uint8_t _nodeId;

// Slot status flags (bit field)
#define SLOT_FLAG_IN_USE        0x01    // Slot is allocated and in use
#define SLOT_FLAG_READY         0x02    // Message is complete and ready for consumption
#define SLOT_FLAG_IN_SEARCH     0x04    // Slot is in search list (assembling)
#define SLOT_FLAG_IN_READY      0x08    // Slot is in ready list (complete, waiting for read)

// buffer element
// Field order optimized to minimize struct padding on 32-bit ARM
typedef struct CAN_Slot
{
    // 4-byte aligned fields first
    uint8_t data[MAX_MESSAGE_SIZE];  // 32 bytes
    uint32_t timestamp;              // 4 bytes
    uint32_t searchId;               // 4 bytes - Slot identifier (from-addr + msg-id)
#ifdef MY_CAN_FAST_SLOT_ACCESS
    CAN_Slot *next;                  // 4 bytes - next in search/ready/free list
    CAN_Slot *prev;                  // 4 bytes - prev in search/ready list
#endif
    // 1-byte fields grouped together at end to minimize padding
    uint8_t frameReceived;           // 1 byte - Bitmask of received frames (bits 0-7)
    uint8_t totalFrames;             // 1 byte
    uint8_t len;                     // 1 byte
    uint8_t flags;                   // 1 byte - SLOT_FLAG_IN_USE | SLOT_FLAG_READY
} CAN_Slot_t;

// Helper macros for slot flags
#define SLOT_IS_IN_USE(slot)      ((slot)->flags & SLOT_FLAG_IN_USE)
#define SLOT_IS_READY(slot)       ((slot)->flags & SLOT_FLAG_READY)
#define SLOT_IS_IN_SEARCH(slot)   ((slot)->flags & SLOT_FLAG_IN_SEARCH)
#define SLOT_IS_IN_READY(slot)    ((slot)->flags & SLOT_FLAG_IN_READY)
#define SLOT_SET_IN_USE(slot)     ((slot)->flags |= SLOT_FLAG_IN_USE)
#define SLOT_SET_READY(slot)      ((slot)->flags |= SLOT_FLAG_READY)
#define SLOT_SET_IN_SEARCH(slot)  ((slot)->flags |= SLOT_FLAG_IN_SEARCH)
#define SLOT_SET_IN_READY(slot)   ((slot)->flags |= SLOT_FLAG_IN_READY)
#define SLOT_CLEAR_IN_SEARCH(slot) ((slot)->flags &= ~SLOT_FLAG_IN_SEARCH)
#define SLOT_CLEAR_IN_READY(slot)  ((slot)->flags &= ~SLOT_FLAG_IN_READY)
#define SLOT_CLEAR_FLAGS(slot)    ((slot)->flags = 0)

// Free list head for O(1) slot allocation
#ifdef MY_CAN_FAST_SLOT_ACCESS
static CAN_Slot_t * volatile freeListHead;
#endif

// assembly buffer
static CAN_Slot_t canSlots[MY_CAN_BUF_SIZE];

// free slots counter
static volatile uint8_t freeCanSlots;

static const uint32_t findSlotMask =        0x1FFFFFF0U;
static const uint32_t fromSearchIdMask =    0x001FFFF0U;
static const uint32_t toAddrMask =          0x1FE00000U;
static const uint32_t msgIdMask =           0x00001FF0U;
static const uint32_t fromAddrMask =        0x001FE000U;
static const uint32_t frameNumMask =        0x0000000EU;
static const uint32_t isLastMask =          0x00000001U;

// Bit shift amounts for header fields
#define CAN_HDR_FRAME_NUM_SHIFT    1
#define CAN_HDR_MSG_ID_SHIFT       4
#define CAN_HDR_FROM_ADDR_SHIFT    13
#define CAN_HDR_TO_ADDR_SHIFT      21

#ifdef MY_CAN_FAST_SLOT_ACCESS
static CAN_Slot_t * volatile searchListFirst;
static CAN_Slot_t * volatile oldestReady;
static CAN_Slot_t * volatile newestReady;  // Tail pointer for O(1) append

/**
 * @brief Add slot to the head of the search list
 * @param slot Pointer to slot to add
 * @note Caller must hold critical section
 */
static inline void _addToSearchList(CAN_Slot_t *slot)
{
    slot->prev = nullptr;
    if (searchListFirst != nullptr)
    {
        searchListFirst->prev = slot;
    }
    slot->next = searchListFirst;
    searchListFirst = slot;
    SLOT_SET_IN_SEARCH(slot);
}

/**
 * @brief Move completed slot from search list to ready list (FIFO order)
 * @param slot Pointer to slot to move
 * @note Caller must hold critical section. Appends to tail for FIFO ordering.
 */
static inline void _moveFromSearchToOldestReadyList(CAN_Slot_t *slot)
{
    // Unlink from search list
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
    SLOT_CLEAR_IN_SEARCH(slot);
    
    // O(1) FIFO append to ready list (append at tail, consume from head)
    // Ready list: oldestReady is head (consume from), newestReady is tail (append to)
    // prev points toward older (head), next points toward newer (tail)
    slot->prev = newestReady;
    slot->next = nullptr;
    
    if (newestReady != nullptr)
    {
        newestReady->next = slot;
    }
    else
    {
        // List was empty, this is also the oldest
        oldestReady = slot;
    }
    newestReady = slot;
    SLOT_SET_IN_READY(slot);
}
#endif

#if defined(ARDUINO_ARCH_STM32) && !defined(MCP_CAN)
/**
 * @brief Initialize CAN hardware filters for STM32 bxCAN
 * @return true if filters initialized successfully
 * 
 * Configures filters to accept only broadcast messages and messages
 * addressed to this node.
 */
bool _initFilters()
{
    if (!canInitialized)
    {
        return false;
    }
    uint8_t err = 0;

    err += CAN0.setFilterMask32(0, BROADCAST_ADDRESS << 21, toAddrMask);
    err += CAN0.setFilterMask32(1, _nodeId << 21, toAddrMask);

    return (err == 0);
}
#elif defined(__linux__) && defined(MY_CAN_LINUX_CANDEV)
/**
 * @brief Initialize CAN socket filters for Linux socketCAN
 * @return true if filters initialized successfully
 * 
 * Configures kernel-level filters to accept only broadcast messages
 * and messages addressed to this node.
 */
bool _initFilters()
{
    if (!canInitialized)
    {
        return false;
    }

    uint8_t err = 0;

    struct can_filter rfilter[2];
    // CAN_EFF_FLAG must be set for extended frame ID filtering
    rfilter[0].can_id = (BROADCAST_ADDRESS << 21) | CAN_EFF_FLAG;
    rfilter[0].can_mask = toAddrMask | CAN_EFF_FLAG;
    rfilter[1].can_id = (_nodeId << 21) | CAN_EFF_FLAG;
    rfilter[1].can_mask = toAddrMask | CAN_EFF_FLAG;
    err += CAN0.setFilterMask(rfilter, 2);

    return err == 0;
}
#else
/**
 * @brief Initialize CAN hardware filters for MCP2515
 * @return true if filters initialized successfully
 * 
 * Configures MCP2515 masks and filters to accept only broadcast messages
 * and messages addressed to this node. Unused filters are set to reject all.
 */
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

/**
 * @brief Clear and release a slot back to the free pool
 * @param slot Pointer to slot to clean
 * @note Caller must hold critical section OR call _cleanSlotSafe() instead.
 *       Unlinks from search/ready lists and returns to free list.
 */
static inline void _cleanSlot(CAN_Slot_t *slot)
{
    // Guard against double-free
    if (!SLOT_IS_IN_USE(slot))
    {
        return;
    }
    
#ifdef MY_CAN_FAST_SLOT_ACCESS
    // Use explicit flags instead of heuristic detection
    if (SLOT_IS_IN_SEARCH(slot))
    {
        // Unlink from search list
        if (slot->prev != nullptr)
        {
            slot->prev->next = slot->next;
        }
        else if (searchListFirst == slot)
        {
            searchListFirst = slot->next;
        }
        
        if (slot->next != nullptr)
        {
            slot->next->prev = slot->prev;
        }
        SLOT_CLEAR_IN_SEARCH(slot);
    }
    else if (SLOT_IS_IN_READY(slot))
    {
        // Unlink from ready list
        // Ready list: oldestReady is head, newestReady is tail
        // prev points toward older (head), next points toward newer (tail)
        if (oldestReady == slot)
        {
            oldestReady = slot->next;
            if (oldestReady != nullptr)
            {
                oldestReady->prev = nullptr;
            }
            else
            {
                newestReady = nullptr;
            }
        }
        else if (newestReady == slot)
        {
            newestReady = slot->prev;
            if (newestReady != nullptr)
            {
                newestReady->next = nullptr;
            }
            else
            {
                oldestReady = nullptr;
            }
        }
        else
        {
            // Slot is in middle of ready list
            if (slot->prev != nullptr)
            {
                slot->prev->next = slot->next;
            }
            if (slot->next != nullptr)
            {
                slot->next->prev = slot->prev;
            }
        }
        SLOT_CLEAR_IN_READY(slot);
    }
    // else: slot is not in any list (already removed)
    
    // Clear pointers before adding to free list
    slot->next = nullptr;
    slot->prev = nullptr;
    
    // Add to free list for O(1) allocation
    slot->next = freeListHead;
    freeListHead = slot;
#endif

    // Clear slot and mark as not in use
    slot->searchId = 0;
    slot->frameReceived = 0;
    slot->totalFrames = 0;
    slot->len = 0;
    SLOT_CLEAR_FLAGS(slot);
    freeCanSlots++;
}

/**
 * @brief Thread-safe version of _cleanSlot
 * @param slot Pointer to slot to clean
 * 
 * Acquires critical section before cleaning the slot.
 */
static inline void _cleanSlotSafe(CAN_Slot_t *slot)
{
    CAN_ENTER_CRITICAL();
    _cleanSlot(slot);
    CAN_EXIT_CRITICAL();
}

/**
 * @brief Find an empty slot in the assembly buffer
 * @return Pointer to empty slot, or nullptr if none available
 * @note Caller must hold critical section.
 *       With MY_CAN_FAST_SLOT_ACCESS: O(1) from free list.
 *       Without: O(n) linear search.
 */
static inline CAN_Slot_t *_findEmptyCanSlot()
{
#ifdef MY_CAN_FAST_SLOT_ACCESS
    // O(1) allocation from free list
    if (freeListHead != nullptr)
    {
        CAN_Slot_t *slot = freeListHead;
        freeListHead = slot->next;
        slot->next = nullptr;
        return slot;
    }
    return nullptr;
#else
    // O(n) fallback: linear search using flags
    for (uint8_t i = 0; i < MY_CAN_BUF_SIZE; i++)
    {
        if (!SLOT_IS_IN_USE(&canSlots[i]))
        {
            return &canSlots[i];
        }
    }
    return nullptr;
#endif
}

/**
 * @brief Find the oldest (by timestamp) slot in the buffer
 * @return Pointer to oldest slot, or nullptr if all slots empty
 * @note Caller must hold critical section. Used for eviction when buffer is full.
 */
static inline CAN_Slot_t *_getOldestSlot()
{
    CAN_Slot_t *slot = nullptr;
    uint32_t oldestTime = UINT32_MAX;
    
    for (uint8_t i = 0; i < MY_CAN_BUF_SIZE; i++)
    {
        // Find slot with smallest (oldest) timestamp using flags
        if (SLOT_IS_IN_USE(&canSlots[i]) && canSlots[i].timestamp < oldestTime)
        {
            oldestTime = canSlots[i].timestamp;
            slot = &canSlots[i];
        }
    }

    return slot;
}

/**
 * @brief Get the oldest completed message slot (thread-safe)
 * @return Pointer to oldest ready slot, or nullptr if none available
 * 
 * With MY_CAN_FAST_SLOT_ACCESS: O(1) from ready list head.
 * Without: O(n) search, returns slot with lowest from-address for fairness.
 * Automatically unlinks the slot from the ready list.
 */
static inline CAN_Slot_t *_getOldestReadySlot()
{
    CAN_Slot_t *slot = nullptr;
    
    CAN_ENTER_CRITICAL();

#ifdef MY_CAN_FAST_SLOT_ACCESS

    slot = oldestReady;
    if (oldestReady != nullptr)
    {
        oldestReady = slot->next;
        slot->next = nullptr;
        slot->prev = nullptr;  // Clear back pointer to prevent _cleanSlot corruption
        SLOT_CLEAR_IN_READY(slot);  // Clear flag since we removed from ready list
        if (oldestReady != nullptr)
        {
            oldestReady->prev = nullptr;
        }
        else
        {
            // List is now empty, clear tail pointer too
            newestReady = nullptr;
        }
    }

#else

    uint8_t i;
    for (i = 0; i < MY_CAN_BUF_SIZE; i++)
    {
        if (SLOT_IS_READY(&canSlots[i]))
        {
            slot = &canSlots[i];
            break;
        }
    }

    if (slot != nullptr)
    {
        for (i++; i < MY_CAN_BUF_SIZE; i++)
        {
            if (SLOT_IS_READY(&canSlots[i]) && (canSlots[i].searchId & fromSearchIdMask) < (slot->searchId & fromSearchIdMask))
            {
                slot = &canSlots[i];
            }
        }
    }

#endif

    CAN_EXIT_CRITICAL();
    return slot;
}

/**
 * @brief Remove slots that have exceeded MY_CAN_SLOT_MAX_AGE_MS
 * 
 * Scans all slots and cleans any that have timed out waiting for
 * remaining frames. Uses single critical section for thread safety.
 */
inline void _removeTimedOutSlots()
{
    uint32_t time = (uint32_t)micros();
    
    CAN_ENTER_CRITICAL();
    for (uint8_t i = 0; i < MY_CAN_BUF_SIZE; i++)
    {
        if (SLOT_IS_IN_USE(&canSlots[i]) && !SLOT_IS_READY(&canSlots[i]) &&
            (time - canSlots[i].timestamp) > MY_CAN_SLOT_MAX_AGE_US)
        {
            CAN_DEBUG(
                PSTR("!CAN:RCV:SLOT=%" PRIu8 ":SEARCHID=0x%" PRIX32 " message dropped (max age)\n"),
                i,
                canSlots[i].searchId);
            CAN_LOG_ERROR(TSP_ERR_CAN_SLOT_TIMEOUT, i);
            _cleanSlot(&canSlots[i]);
        }
    }
    CAN_EXIT_CRITICAL();
}


/**
 * @brief Find existing slot matching the given CAN ID (for frame assembly)
 * @param rxId CAN extended ID from received frame
 * @param currentFrameMask Bitmask for current frame number (to detect duplicates)
 * @return Pointer to matching slot, or nullptr if not found
 * @note Caller must hold critical section.
 *       Searches for slot with same source+msgId that hasn't received this frame yet.
 */
static inline CAN_Slot_t *_findCanSlot(const uint32_t rxId, const uint8_t currentFrameMask)
{
#ifdef MY_CAN_FAST_SLOT_ACCESS
    // O(m): linear search through active slots list (m = number of slots being assembled)
    CAN_Slot_t *slot = searchListFirst;
    while (slot != nullptr)
    {
        if ((slot->searchId & findSlotMask) == (rxId & findSlotMask) && (slot->frameReceived & currentFrameMask) == 0)
        {
            break;
        }
        slot = slot->next;
    }
#else

    CAN_Slot_t *slot = nullptr;
    for (uint8_t i = 0; i < MY_CAN_BUF_SIZE; i++)
    {
        // Use flags: check in use and not ready yet (still assembling)
        if (SLOT_IS_IN_USE(&canSlots[i]) && !SLOT_IS_READY(&canSlots[i]) && 
            (canSlots[i].searchId & findSlotMask) == (rxId & findSlotMask) &&
            (canSlots[i].frameReceived & currentFrameMask) == 0)
        {
            slot = &canSlots[i];
            break;
        }
    }
#endif

    return slot;
}

/**
 * @brief Check if any complete messages are available for reading
 * @return true if at least one message is ready
 * 
 * With MY_CAN_FAST_SLOT_ACCESS: O(1) check of ready list.
 * Without: O(n) scan of all slots.
 */
static inline bool _checkDataAvailable()
{
    bool result = false;
    
    CAN_ENTER_CRITICAL();
#ifdef MY_CAN_FAST_SLOT_ACCESS
    result = (oldestReady != nullptr);
#else
    for (uint8_t i = 0; i < MY_CAN_BUF_SIZE; i++)
    {
        if (SLOT_IS_READY(&canSlots[i]))
        {
            result = true;
            break;
        }
    }
#endif
    CAN_EXIT_CRITICAL();
    
    return result;
}

/**
 * @brief Find or allocate a slot for incoming frame (thread-safe)
 * @param rxId CAN extended ID from received frame
 * @param currentFrameMask Bitmask for current frame number
 * @param isNewSlot Output: true if a new slot was allocated
 * @return Pointer to slot, or nullptr if allocation failed
 * 
 * This function handles all critical section management internally.
 */
static CAN_Slot_t *_findOrAllocateSlot(const uint32_t rxId, const uint8_t currentFrameMask, bool *isNewSlot)
{
    *isNewSlot = false;
    CAN_Slot_t *slot = nullptr;
    
    // Single critical section to prevent TOCTOU race between find and allocate
    CAN_ENTER_CRITICAL();
    
    // First, try to find existing slot for this message
    slot = _findCanSlot(rxId, currentFrameMask);
    if (slot != nullptr)
    {
        CAN_EXIT_CRITICAL();
        return slot;
    }
    
    // Try to allocate a new slot
    slot = _findEmptyCanSlot();
    
    if (slot == nullptr)
    {
        // Try inline timeout cleanup (avoid releasing critical section)
        uint32_t currentTime = (uint32_t)micros();
        for (uint8_t i = 0; i < MY_CAN_BUF_SIZE && slot == nullptr; i++)
        {
            if (SLOT_IS_IN_USE(&canSlots[i]) && !SLOT_IS_READY(&canSlots[i]) &&
                (currentTime - canSlots[i].timestamp) > MY_CAN_SLOT_MAX_AGE_US)
            {
                // Log only error code inside critical section (lightweight)
                // Verbose debug output would block interrupts for 100µs+
                CAN_LOG_ERROR(TSP_ERR_CAN_SLOT_TIMEOUT, i);
                _cleanSlot(&canSlots[i]);
                slot = &canSlots[i];
            }
        }
    }
    
    if (slot == nullptr)
    {
        // Try allocation again after cleanup
        slot = _findEmptyCanSlot();
    }
    
    if (slot == nullptr)
    {
        // Evict oldest slot as last resort
        slot = _getOldestSlot();
        if (slot != nullptr)
        {
            _cleanSlot(slot);
#ifdef MY_CAN_FAST_SLOT_ACCESS
            // CRITICAL: _cleanSlot added slot to free list head, but we're about to reuse it.
            // Pop it from free list to prevent double-allocation.
            if (freeListHead == slot)
            {
                freeListHead = slot->next;
                slot->next = nullptr;
            }
#endif
        }
        else
        {
            // Truly no slots available (should not happen)
            CAN_EXIT_CRITICAL();
            CAN_LOG_ERROR(TSP_ERR_CAN_SLOT_FULL, 0);
            return nullptr;
        }
    }
    
    // Initialize the new slot (still in critical section)
    // Note: freeCanSlots was already decremented by pop from free list (eviction case)
    // or needs decrement for fresh allocation
    if (freeCanSlots > 0)
    {
        freeCanSlots--;
    }
    SLOT_SET_IN_USE(slot);
    slot->searchId = rxId & findSlotMask;
    slot->frameReceived = 0;
    slot->len = 0;
    slot->totalFrames = 0;
    slot->timestamp = (uint32_t)micros();
#ifdef MY_CAN_FAST_SLOT_ACCESS
    _addToSearchList(slot);
#endif
    
    CAN_EXIT_CRITICAL();
    
    *isNewSlot = true;
    return slot;
}

/**
 * @brief Update slot with received frame data (thread-safe)
 * @param slot Pointer to the slot to update
 * @param rxBuf Frame data buffer
 * @param len Frame data length
 * @param currentFrame Frame number (0-3)
 * @param isLast True if this is the last frame
 * @return True if message is now complete
 */
static bool _updateSlotWithFrame(CAN_Slot_t *slot, const uint8_t *rxBuf, uint8_t len, 
                                  uint8_t currentFrame, bool isLast)
{
    uint8_t frameMask = (0x01U << currentFrame);
    
    CAN_ENTER_CRITICAL();
    
    // Check for duplicate frame (security fix: prevents len overflow)
    if (slot->frameReceived & frameMask)
    {
        CAN_EXIT_CRITICAL();
        CAN_DEBUG(PSTR("!CAN:RCV:DUP_FRAME=%" PRIu8 "\n"), currentFrame);
        CAN_LOG_ERROR(TSP_ERR_CAN_RX_DUP_FRAME, currentFrame);
        return false;  // Ignore duplicate frame
    }
    
    // Prevent len overflow: check before adding
    if ((uint16_t)slot->len + len > MAX_MESSAGE_SIZE)
    {
        CAN_EXIT_CRITICAL();
        CAN_DEBUG(PSTR("!CAN:RCV:LEN_OVERFLOW\n"));
        CAN_LOG_ERROR(TSP_ERR_CAN_RX_LEN_OVERFLOW, slot->len);
        return false;
    }
    
    memcpy(slot->data + currentFrame * CAN_MAX_CHAR_IN_MESSAGE, rxBuf, len);
    slot->len += len;
    slot->frameReceived |= frameMask;
    if (isLast)
    {
        slot->totalFrames = currentFrame + 1;
    }
    
    // Check if message is complete (all frames received)
    bool messageComplete = (slot->totalFrames > 0 && 
        ((slot->frameReceived & 0x0FU) ^ ((1 << slot->totalFrames) - 1)) == 0);
    
    if (messageComplete)
    {
#ifdef MY_CAN_FAST_SLOT_ACCESS
        _moveFromSearchToOldestReadyList(slot);
#else
        SLOT_SET_READY(slot);
#endif
    }
    
    CAN_EXIT_CRITICAL();
    
    return messageComplete;
}

// CAN Extended ID Header Format (29 bits)
// 
// Bit layout (MSB to LSB):
//   B: 8 bits - to address (destination node, 0-255)
//   A: 8 bits - from address (source node, 0-255)
//   E: 9 bits - message_id (0-511, unique per multi-frame message)
//   C: 3 bits - current frame number (0-7)
//   D: 1 bit  - is last frame flag
//
// Binary: BBBB BBBB AAAA AAAA EEEE EEEE ECCC D
//
/**
 * @brief Build CAN extended ID header for transmission
 * @param messageId Unique message identifier (9 bits, 0-511)
 * @param totalFramesCount Total number of frames in this message (1-4)
 * @param currentFrameNumber Current frame index (0-3)
 * @param toAddress Destination node address (0-255)
 * @param fromAddress Source node address (0-255)
 * @return 29-bit CAN extended ID
 */
static uint32_t _buildHeader(
    uint16_t messageId,
    uint8_t totalFramesCount,
    uint8_t currentFrameNumber,
    uint8_t toAddress,
    uint8_t fromAddress)
{
    uint32_t header = 0x0U;
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

/**
 * @brief Parse CAN extended ID header from received frame
 * @param rxId 29-bit CAN extended ID
 * @param[out] messageId Extracted message identifier
 * @param[out] isLast True if this is the last frame
 * @param[out] currentFrameNumber Frame index (0-7)
 * @param[out] toAddress Destination node address
 * @param[out] fromAddress Source node address
 */
static void _parseHeader(
    const uint32_t rxId,
    uint16_t *messageId,
    bool *isLast,
    uint8_t *currentFrameNumber,
    uint8_t *toAddress,
    uint8_t *fromAddress)
{
    *isLast = (bool)(rxId & isLastMask);
    *currentFrameNumber = (uint8_t)((rxId & frameNumMask) >> CAN_HDR_FRAME_NUM_SHIFT);
    *messageId = (uint16_t)((rxId & msgIdMask) >> CAN_HDR_MSG_ID_SHIFT);
    *fromAddress = (uint8_t)((rxId & fromAddrMask) >> CAN_HDR_FROM_ADDR_SHIFT);
    *toAddress = (uint8_t)((rxId & toAddrMask) >> CAN_HDR_TO_ADDR_SHIFT);
}

/**
 * @brief Get current CAN controller error state
 * @return CAN_ErrorState_t indicating the most severe error condition
 * 
 * Checks controller registers for bus-off, error passive, and overflow.
 * Returns the most severe condition found (bus-off > passive > warning > overflow > none).
 */
static CAN_ErrorState_t _getErrorState(void)
{
#if defined(ARDUINO_ARCH_STM32) && !defined(MCP_CAN)
    uint8_t tec = CAN0.errorCountTX();
    uint8_t rec = CAN0.errorCountRX();
    
#if defined(CAN_HAS_ISBUSOFF) || defined(STM32_CAN_HAS_ISBUSOFF)
    if (CAN0.isBusOff()) {
        return CAN_ERR_BUS_OFF;
    }
#else
    if (tec >= 255) {
        return CAN_ERR_BUS_OFF;
    }
#endif
    
    if (tec >= 128 || rec >= 128) {
        return CAN_ERR_PASSIVE;
    }
    
    if (tec > CAN_ERROR_WARNING_THRESHOLD || rec > CAN_ERROR_WARNING_THRESHOLD) {
        return CAN_ERR_WARNING;
    }
    
#elif defined(__linux__) && defined(MY_CAN_LINUX_CANDEV)
    // Linux: Kernel handles errors
    (void)0;
    
#else
    // MCP2515
    uint8_t eflg = CAN0.getError();
    
    if (eflg & MCP_EFLG_TXBO) {
        return CAN_ERR_BUS_OFF;
    }
    
    if (eflg & (MCP_EFLG_TXEP | MCP_EFLG_RXEP)) {
        return CAN_ERR_PASSIVE;
    }
    
    if (eflg & (MCP_EFLG_RX0OVR | MCP_EFLG_RX1OVR)) {
        return CAN_ERR_RX_OVERFLOW;
    }
#endif
    
    return CAN_ERR_NONE;
}

/**
 * @brief Attempt to reinitialize the CAN controller
 * @return true if reinitialization successful
 */
static bool _reinitController(void)
{
    canInitialized = false;
    if (CAN0.begin(MCP_STDEXT, MY_CAN_SPEED, MY_CAN_CLOCK) == CAN_OK) {
        if (!_initFilters()) {
            CAN_LOG_ERROR(TSP_ERR_CAN_INIT_FILTER, 0);
            return false;
        }
        canInitialized = true;
        return true;
    }
    return false;
}

/**
 * @brief Check CAN controller error state and attempt recovery
 * @return true if transport is operational, false if unrecoverable error
 * 
 * Checks for bus-off, error passive, and RX overflow conditions.
 * Attempts automatic recovery from bus-off by reinitializing the controller.
 * Uses exponential backoff for recovery attempts (100ms initial, capped at 5s).
 * Logs errors via CAN_LOG_ERROR when MY_TRANSPORT_ERROR_LOG is enabled.
 */
static bool _checkAndRecoverErrors(void)
{
    if (!canInitialized && !busOffRecoveryPending) {
        return false;
    }
    
    CAN_ErrorState_t errState = _getErrorState();
    
    if (errState == CAN_ERR_BUS_OFF) {
        CAN_DEBUG(PSTR("!CAN:ERR:BUS_OFF\n"));
        CAN_LOG_ERROR(TSP_ERR_CAN_BUS_OFF, 0);
        canInitialized = false;
        
        // Initialize or continue exponential backoff
        if (!busOffRecoveryPending) {
            busOffRecoveryPending = true;
            busOffBackoffMs = CAN_BUSOFF_INITIAL_BACKOFF_MS;
            busOffLastAttemptMs = hwMillis();
        }
        
        // Check if enough time has passed since last attempt
        uint32_t now = hwMillis();
        if ((now - busOffLastAttemptMs) < busOffBackoffMs) {
            // Still waiting for backoff period
            return false;
        }
        
        // Attempt recovery
        busOffLastAttemptMs = now;
        if (_reinitController()) {
            busOffRecoveryPending = false;
            busOffBackoffMs = 0;
            CAN_DEBUG(PSTR("CAN:RECOVERED\n"));
            CAN_LOG_ERROR(TSP_ERR_RECOVERY_OK, 0);
            return true;
        }
        
        // Recovery failed, increase backoff (capped at max)
        busOffBackoffMs *= CAN_BUSOFF_BACKOFF_MULTIPLIER;
        if (busOffBackoffMs > CAN_BUSOFF_MAX_BACKOFF_MS) {
            busOffBackoffMs = CAN_BUSOFF_MAX_BACKOFF_MS;
        }
        CAN_DEBUG(PSTR("!CAN:ERR:RECOVERY_FAILED backoff=%" PRIu32 "ms\n"), busOffBackoffMs);
        CAN_LOG_ERROR(TSP_ERR_RECOVERY_FAILED, (uint8_t)(busOffBackoffMs / 100));
        return false;
    }
    
    // Log non-fatal errors
    if (errState == CAN_ERR_PASSIVE) {
        CAN_DEBUG(PSTR("!CAN:ERR:PASSIVE\n"));
        CAN_LOG_ERROR(TSP_ERR_CAN_BUS_PASSIVE, 0);
    } else if (errState == CAN_ERR_WARNING) {
        CAN_DEBUG(PSTR("!CAN:ERR:WARNING\n"));
        CAN_LOG_ERROR(TSP_ERR_CAN_BUS_WARNING, 0);
    } else if (errState == CAN_ERR_RX_OVERFLOW) {
        CAN_DEBUG(PSTR("!CAN:ERR:RX_OVF\n"));
        CAN_LOG_ERROR(TSP_ERR_CAN_RX_BUFFER_OVF, 0);
    }
    
    return true;
}

/**
 * @brief Initialize CAN transport
 * @return true if initialization successful
 * 
 * Initializes the CAN controller, slot buffer, and hardware filters.
 * Must be called before any other CAN transport functions.
 */
bool CAN_transportInit(void)
{
    CAN_DEBUG(
        PSTR("CAN:INIT:CS=%" PRIu8 ",INT=%" PRIu8 ",SPE=%" PRIu8 ",CLO=%" PRIu8 "\n"),
        MY_CAN_CS,
        MY_CAN_INT,
        MY_CAN_SPEED,
        MY_CAN_CLOCK);

    freeCanSlots = MY_CAN_BUF_SIZE;
    
    // Initialize all slots as not in use
    for (uint8_t i = 0; i < MY_CAN_BUF_SIZE; i++)
    {
        SLOT_CLEAR_FLAGS(&canSlots[i]);
        canSlots[i].searchId = 0;
    }

#ifdef MY_CAN_FAST_SLOT_ACCESS
    searchListFirst = nullptr;
    oldestReady = nullptr;
    newestReady = nullptr;
    
    // Initialize free list: link all slots together
    freeListHead = &canSlots[0];
    for (uint8_t i = 0; i < MY_CAN_BUF_SIZE - 1; i++)
    {
        canSlots[i].next = &canSlots[i + 1];
    }
    canSlots[MY_CAN_BUF_SIZE - 1].next = nullptr;
#endif

    if (CAN0.begin(MCP_STDEXT, MY_CAN_SPEED, MY_CAN_CLOCK) != CAN_OK)
    {
        canInitialized = false;
        CAN_LOG_ERROR(TSP_ERR_CAN_INIT, 0);
        return false;
    }
    canInitialized = true;

    if (!_initFilters()) {
        CAN_LOG_ERROR(TSP_ERR_CAN_INIT_FILTER, 0);
        return false;
    }

    return true;
}

/**
 * @brief Send message via CAN transport
 * @param to Destination node address (0-255, 255=broadcast)
 * @param data Pointer to message data
 * @param len Length of message (1-32 bytes)
 * @param noACK Unused (CAN provides hardware ACK)
 * @return true if message sent successfully
 * 
 * Fragments messages >8 bytes into multiple CAN frames.
 * Retries with error recovery on transmission failure.
 */
bool CAN_transportSend(const uint8_t to, const void *data, const uint8_t len, const bool noACK)
{
    (void)noACK; // some ack is provided by CAN itself. TODO implement application layer ack.
    
    // Validate input parameters
    if (len == 0 || len > MAX_MESSAGE_SIZE || data == nullptr)
    {
        CAN_DEBUG(PSTR("!CAN:SND:INVALID_PARAM len=%" PRIu8 "\n"), len);
        return false;
    }
    
    const uint8_t *datap = static_cast<const uint8_t *>(data);
    // calculate number of frames
    uint8_t noOfFrames = (len + CAN_MAX_CHAR_IN_MESSAGE - 1) / CAN_MAX_CHAR_IN_MESSAGE;
    // message id updated for every outgoing mesage
    static uint16_t message_id;

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
            // Retry with error recovery
            uint8_t retries = MY_CAN_SEND_RETRIES;
            bool success = false;
            
            while (retries-- > 0 && !success) {
                // Check for bus errors and attempt recovery
                if (!_checkAndRecoverErrors()) {
                    CAN_DEBUG(PSTR("!CAN:SND:RECOVERY_FAILED\n"));
                    continue;
                }
                
                // Retry sending
                if (CAN0.sendMsgBuf(
                        _buildHeader(message_id, noOfFrames, currentFrame, to, _nodeId),
                        1,
                        partLen,
                        datap + (currentFrame * CAN_MAX_CHAR_IN_MESSAGE)) == CAN_OK)
                {
                    success = true;
                }
            }
            
            if (!success) {
                CAN_DEBUG(PSTR("!CAN:SND:FAIL after %" PRIu8 " retries\n"), MY_CAN_SEND_RETRIES);
                CAN_LOG_ERROR(TSP_ERR_CAN_TX_FAILED, currentFrame);
                return false;
            }
        }
    }

    CAN_DEBUG(PSTR("CAN:SND:OK\n"));
    return true;
}

/**
 * @brief Check for and process incoming CAN frames
 * @return true if complete message(s) available for reading
 * 
 * Reads all pending frames from CAN controller, assembles multi-frame
 * messages into slots, and marks complete messages as ready.
 * Should be called frequently from main loop.
 */
bool CAN_transportDataAvailable(void)
{
    uint32_t rxId;
    uint8_t len = 0;
    uint8_t rxBuf[CAN_MAX_CHAR_IN_MESSAGE];
#if defined(ARDUINO_ARCH_STM32) && !defined(MCP_CAN)
    while (CAN0.checkReceive())
    {
        CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
#elif defined(__linux__) && defined(MY_CAN_LINUX_CANDEV)
    while (CAN0.readMsgBuf(&rxId, &len, rxBuf) > 0)
    {
#else
    while (!hwDigitalRead(MY_CAN_INT)) // If MY_CAN_INT pin is low, read receive buffer
    {
        CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
#endif
        // Validate len from driver to prevent buffer overread (security fix)
        if (len > CAN_MAX_CHAR_IN_MESSAGE)
        {
            CAN_DEBUG(PSTR("!CAN:RCV:LEN_INVALID=%" PRIu8 "\n"), len);
            CAN_LOG_ERROR(TSP_ERR_CAN_RX_LEN_INVALID, len);
            continue;
        }
        
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

        // Early bounds check BEFORE slot allocation (security fix)
        // Prevents allocating slots for invalid frames that would never complete
        if (currentFrame >= CAN_MAX_FRAMES || 
            ((uint8_t)(currentFrame * CAN_MAX_CHAR_IN_MESSAGE) + len) > MAX_MESSAGE_SIZE)
        {
            CAN_DEBUG(
                PSTR("!CAN:RCV:FRAME_OOB:FRAME=%" PRIu8 ":LEN=%" PRIu8 "\n"),
                currentFrame,
                len);
            CAN_LOG_ERROR(TSP_ERR_CAN_RX_FRAME_OOB, currentFrame);
            continue;
        }

        // Find existing slot or allocate new one (handles critical sections internally)
        bool isNewSlot;
        CAN_Slot_t *slot = _findOrAllocateSlot(rxId, currentFrameMask, &isNewSlot);
        
        if (slot == nullptr)
        {
            CAN_DEBUG(
                PSTR("!CAN:RCV:SLOT:FROM=%" PRIu8 ":MSGID=%" PRIu8
                     " message dropped (no space) !!!Increase MY_CAN_BUF_SIZE if you seeing this too "
                     "often!!!\n"),
                from,
                messageId);
            CAN_LOG_ERROR(TSP_ERR_CAN_SLOT_FULL, from);
            continue;
        }
        
        // Update slot with frame data (handles critical sections internally)
        bool messageComplete = _updateSlotWithFrame(slot, rxBuf, len, currentFrame, isLast);
        
        CAN_DEBUG(
            PSTR("CAN:RCV:SLOT:FROM=%" PRIu8 ":MSGID=%" PRIu8 ":PARTS=%" PRIX8 ":LEN=%" PRIu8 "\n"),
            from,
            messageId,
            slot->frameReceived & 0X0FU,
            slot->len);
        if (messageComplete)
        {
            CAN_DEBUG(PSTR("CAN:RCV:SLOT:FROM=%" PRIu8 ":MSGID=%" PRIu8 " complete\n"), from, messageId);
        }
        
        // Check if running low on slots - atomic read with critical section
        CAN_ENTER_CRITICAL();
        uint8_t slotsAvailable = freeCanSlots;
        CAN_EXIT_CRITICAL();
        if (slotsAvailable < 2)
        {
            break;
        }
    }

    return _checkDataAvailable();
}

/**
 * @brief Transport task for RX queue mode
 * 
 * When MY_TRANSPORT_RX_QUEUE is enabled, this function processes
 * incoming messages and pushes them to the transport HAL queue.
 */
void CAN_transportTask(void)
{
#if defined(MY_TRANSPORT_RX_QUEUE)
    while (CAN_transportDataAvailable())
    {
        RXQueuedMessage_t *msgIn = transportHALGetQueueBuffer();
        if (msgIn == NULL)
        {
            // Queue full or allocation failed - break to prevent infinite loop
            // Messages will be processed on next call when queue has space
            break;
        }
        msgIn->channel = TRANSPORT_CAN_CHANNEL_ID;
        msgIn->length = CAN_transportReceive((void *)&msgIn->data, sizeof(msgIn->data));
        (void)transportHALPushQueueBuffer(msgIn);
    }
#endif
}

/**
 * @brief Receive a complete message from CAN transport
 * @param data Buffer to store received message
 * @param maxBufSize Maximum bytes to copy
 * @return Number of bytes received, or 0 if no message available
 * 
 * Returns the oldest complete message and releases its slot.
 */
uint8_t CAN_transportReceive(void *data, const uint8_t maxBufSize)
{
    CAN_Slot_t *slot = _getOldestReadySlot();

    if (slot == nullptr)
    {
        return (0);
    }

    // Limit copy length to prevent buffer overflow
    uint8_t len = (slot->len > maxBufSize) ? maxBufSize : slot->len;

    memcpy(data, slot->data, len);
    
    _cleanSlotSafe(slot);

    return len;
}

/**
 * @brief Set this node's address and update filters
 * @param address New node address (0-255)
 * 
 * Updates hardware filters to accept messages for the new address.
 */
void CAN_transportSetAddress(const uint8_t address)
{
    CAN_ENTER_CRITICAL();
    if (_nodeId == address)
    {
        CAN_EXIT_CRITICAL();
        return;
    }

    _nodeId = address;
    CAN_EXIT_CRITICAL();
    
    // Filter reconfiguration outside critical section (takes longer)
    if (!_initFilters()) {
        CAN_LOG_ERROR(TSP_ERR_CAN_INIT_FILTER, address);
    }
}

/**
 * @brief Get this node's address
 * @return Current node address
 */
uint8_t CAN_transportGetAddress(void)
{
    return _nodeId;
}

/**
 * @brief Check CAN transport health
 * @return true if transport is operational
 * 
 * Checks for bus-off and error passive conditions.
 * Called periodically by MySensors to verify transport health.
 */
bool CAN_transportSanityCheck(void)
{
    if (!canInitialized) {
        return false;
    }
    
    CAN_ErrorState_t errState = _getErrorState();
    
    if (errState == CAN_ERR_BUS_OFF) {
        CAN_DEBUG(PSTR("!CAN:SANITY:BUS_OFF\n"));
        CAN_LOG_ERROR(TSP_ERR_CAN_BUS_OFF, 0);
        return false;
    }
    
    // Error passive state (degraded but still functional)
    if (errState == CAN_ERR_PASSIVE) {
        CAN_DEBUG(PSTR("!CAN:SANITY:PASSIVE\n"));
        CAN_LOG_ERROR(TSP_ERR_CAN_BUS_PASSIVE, 0);
        // Still return true as we can communicate, but log warning
    }
    
    return true;
}

/**
 * @brief Power down CAN transport (not implemented)
 */
void CAN_transportPowerDown(void)
{
    // Nothing to shut down here
}

/**
 * @brief Power up CAN transport (not implemented)
 */
void CAN_transportPowerUp(void)
{
    // not implemented
}

/**
 * @brief Put CAN transport to sleep (not implemented)
 */
void CAN_transportSleep(void)
{
    // not implemented
}

/**
 * @brief Put CAN transport in standby (not implemented)
 */
void CAN_transportStandBy(void)
{
    // not implemented
}

/**
 * @brief Get sending RSSI (not applicable for CAN)
 * @return INVALID_RSSI
 */
int16_t CAN_transportGetSendingRSSI(void)
{
    // not implemented
    return INVALID_RSSI;
}

/**
 * @brief Get receiving RSSI (not applicable for CAN)
 * @return INVALID_RSSI
 */
int16_t CAN_transportGetReceivingRSSI(void)
{
    // not implemented
    return INVALID_RSSI;
}

/**
 * @brief Get sending SNR (not applicable for CAN)
 * @return INVALID_SNR
 */
int16_t CAN_transportGetSendingSNR(void)
{
    // not implemented
    return INVALID_SNR;
}

/**
 * @brief Get receiving SNR (not applicable for CAN)
 * @return INVALID_SNR
 */
int16_t CAN_transportGetReceivingSNR(void)
{
    // not implemented
    return INVALID_SNR;
}

/**
 * @brief Get TX power as percentage (not applicable for CAN)
 * @return 100 (always full power)
 */
int16_t CAN_transportGetTxPowerPercent(void)
{
    // not implemented
    return static_cast<int16_t>(100);
}

/**
 * @brief Get TX power level (not applicable for CAN)
 * @return 100
 */
int16_t CAN_transportGetTxPowerLevel(void)
{
    // not implemented
    return static_cast<int16_t>(100);
}

/**
 * @brief Set TX power as percentage (not applicable for CAN)
 * @param powerPercent Ignored
 * @return false (not supported)
 */
bool CAN_transportSetTxPowerPercent(const uint8_t powerPercent)
{
    // not possible
    (void)powerPercent;
    return false;
}
