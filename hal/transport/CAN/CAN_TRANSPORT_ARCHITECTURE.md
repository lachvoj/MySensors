# CAN Transport Architecture

## Overview

The MySensors CAN transport layer handles multi-frame message assembly over CAN 2.0B extended frames. Messages up to 32 bytes are fragmented into 8-byte CAN frames and reassembled at the receiver using a slot-based buffer system.

## CAN Extended ID Header Format (29 bits)

```
┌─────────────────────────────────────────────────────────────┐
│  Bit 28-21  │  Bit 20-13  │  Bit 12-4   │ Bit 3-1 │  Bit 0  │
├─────────────┼─────────────┼─────────────┼─────────┼─────────┤
│  TO (8b)    │  FROM (8b)  │ MSG_ID (9b) │FRAME(3b)│ LAST(1b)│
│  Dest Addr  │  Src Addr   │  Message ID │Frame Num│Is Last  │
└─────────────────────────────────────────────────────────────┘
```

- **TO**: Destination node address (0-255, 0=broadcast)
- **FROM**: Source node address (0-255)
- **MSG_ID**: Unique message identifier (0-511), increments per message
- **FRAME**: Current frame number (0-7, typically 0-3 for 32-byte max)
- **LAST**: 1 if this is the final frame of the message

### Design Rationale & Considered Alternatives

The current header format was analyzed for potential optimization:

| Field | Current | Alternative | Trade-off |
|-------|---------|-------------|-----------|
| MSG_ID | 9 bits (512 IDs) | 8 bits (256 IDs) | Frees 1 bit, doubles collision risk |
| MSG_ID | 9 bits | 7 bits (128 IDs) | Frees 2 bits for new features |
| FRAME | 3 bits (8 frames) | 4 bits (16 frames) | Supports 128-byte messages |

**Potential uses for freed bits:**
- **Priority bit (P)**: Lower CAN ID wins arbitration. P=0 (high) beats P=1 (normal). Useful for alarms/emergency.
- **Transport ACK bit (A)**: Request immediate transport-level acknowledgment.
- **Message Class (2 bits)**: Distinguish data/control/discovery traffic.

**Why no change was made:**
1. **MSG_ID collision is not a real concern** - The ID is only used to reassemble multi-frame messages from the same sender. A sender would need to transmit 512 complete messages before wrap-around, by which time all previous slots are long consumed.
2. **MySensors limits to 32 bytes** - Only 4 frames needed, so 4-bit FRAME field provides no benefit.
3. **Priority not supported by MySensors** - Would require changes to MySensors core API.
4. **ACK already in MySensors** - Application-level echo/ACK exists via `setRequestEcho()`.
5. **Backward compatibility** - Breaking change would require coordinated upgrade of all nodes.

### Why LAST Bit is Required

The LAST bit enables **out-of-order frame reception**:

```
Frames can arrive:  Frame 2 (LAST) → Frame 0 → Frame 1
                         ↓
              totalFrames = 3 (learned from LAST frame)
```

Completion is detected via bitmask: `frameReceived == (1 << totalFrames) - 1`

Without LAST, the receiver wouldn't know how many frames to expect.

## Slot Structure

```c
typedef struct CAN_Slot {
    uint8_t data[32];        // Message data buffer
    uint32_t timestamp;      // Slot allocation time (micros)
    uint32_t searchId;       // Identifier for slot matching
#ifdef MY_CAN_FAST_SLOT_ACCESS
    CAN_Slot *next/newer;    // Linked list pointers
    CAN_Slot *prev/older;    // (union: dual purpose)
#endif
    uint8_t frameReceived;   // Bitmask of received frames
    uint8_t totalFrames;     // Total frames expected
    uint8_t len;             // Accumulated data length
    uint8_t flags;           // SLOT_FLAG_IN_USE | SLOT_FLAG_READY
} CAN_Slot_t;
```

## Configuration Comparison

| Feature | `MY_CAN_FAST_SLOT_ACCESS` Disabled | `MY_CAN_FAST_SLOT_ACCESS` Enabled |
|---------|-----------------------------------|-----------------------------------|
| Slot size | 44 bytes | 48 bytes (+8 for pointers) |
| Find empty slot | O(n) array scan | O(1) free list pop |
| Find existing slot | O(n) array scan | O(m) linked list scan |
| Mark ready | Set flag | Move to ready list O(1) |
| Get ready slot | O(n) scan + compare | O(1) list head |
| Check data available | O(n) scan | O(1) pointer check |
| Memory overhead | Minimal | +8 bytes/slot + 3 pointers |

Where:
- `n` = MY_CAN_BUF_SIZE (total slots)
- `m` = active slots being assembled (typically 1-4)

---

## Flow Diagrams

### 1. Receive Frame Flow

```
                    ┌──────────────────┐
                    │  CAN Frame RX    │
                    │  (from driver)   │
                    └────────┬─────────┘
                             │
                             ▼
                    ┌──────────────────┐
                    │ Validate len ≤ 8 │
                    └────────┬─────────┘
                             │ valid
                             ▼
                    ┌──────────────────┐
                    │  Parse Header    │
                    │  (from, msgId,   │
                    │   frame#, last)  │
                    └────────┬─────────┘
                             │
                             ▼
                    ┌──────────────────┐
                    │ Bounds Check     │
                    │ frame < 4 &&     │
                    │ offset+len ≤ 32  │
                    └────────┬─────────┘
                             │ valid
                             ▼
              ┌──────────────────────────────┐
              │    _findOrAllocateSlot()     │
              └──────────────┬───────────────┘
                             │
                             ▼
              ┌──────────────────────────────┐
              │    _updateSlotWithFrame()    │
              │  - Check duplicate frame     │
              │  - Check len overflow        │
              │  - Copy data to slot         │
              │  - Update frameReceived      │
              │  - Check if complete         │
              └──────────────┬───────────────┘
                             │
                             ▼
                    ┌──────────────────┐
                    │ Message Complete?│
                    └───────┬──────────┘
                       no   │   yes
                    ┌───────┴───────┐
                    │               │
                    ▼               ▼
              ┌──────────┐   ┌──────────────┐
              │ Continue │   │ Move to      │
              │ waiting  │   │ ready list   │
              └──────────┘   └──────────────┘
```

### 2. Slot Allocation Flow (`_findOrAllocateSlot`)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         _findOrAllocateSlot()                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────────────────────┐                                    │
│  │ _findCanSlot(rxId, frameMask)   │                                    │
│  │ Look for existing slot          │                                    │
│  └───────────────┬─────────────────┘                                    │
│                  │                                                      │
│         ┌────────┴────────┐                                             │
│         │  Found slot?    │                                             │
│         └───────┬─────────┘                                             │
│            yes  │   no                                                  │
│         ┌───────┴───────┐                                               │
│         │               │                                               │
│         ▼               ▼                                               │
│   ┌──────────┐   ┌─────────────────────────┐                            │
│   │  Return  │   │  _findEmptyCanSlot()    │                            │
│   │   slot   │   └───────────┬─────────────┘                            │
│   └──────────┘               │                                          │
│                     ┌────────┴────────┐                                 │
│                     │  Found empty?   │                                 │
│                     └───────┬─────────┘                                 │
│                        yes  │   no                                      │
│                     ┌───────┴───────┐                                   │
│                     │               │                                   │
│                     ▼               ▼                                   │
│               ┌──────────┐   ┌─────────────────────────┐                │
│               │ Init &   │   │ _removeTimedOutSlots()  │                │
│               │ return   │   │ Try again...            │                │
│               └──────────┘   └───────────┬─────────────┘                │
│                                          │                              │
│                                 ┌────────┴────────┐                     │
│                                 │  Found empty?   │                     │
│                                 └───────┬─────────┘                     │
│                                    yes  │   no                          │
│                                 ┌───────┴───────┐                       │
│                                 │               │                       │
│                                 ▼               ▼                       │
│                           ┌──────────┐   ┌─────────────────────────┐    │
│                           │ Init &   │   │ _getOldestSlot()        │    │
│                           │ return   │   │ Evict & return          │    │
│                           └──────────┘   └─────────────────────────┘    │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### 3. Implementation Comparison: Find Empty Slot

#### Without `MY_CAN_FAST_SLOT_ACCESS` (O(n) scan):

```
┌─────────────────────────────────────┐
│      _findEmptyCanSlot()            │
├─────────────────────────────────────┤
│                                     │
│   for i = 0 to BUF_SIZE:            │
│       ┌───────────────────┐         │
│       │ canSlots[i].flags │         │
│       │ & SLOT_FLAG_IN_USE│         │
│       │     == 0 ?        │         │
│       └─────────┬─────────┘         │
│            yes  │  no               │
│       ┌─────────┴─────────┐         │
│       ▼                   ▼         │
│   ┌────────┐        ┌─────────┐     │
│   │ return │        │  next   │     │
│   │ slot   │        │   i++   │     │
│   └────────┘        └─────────┘     │
│                                     │
│   return nullptr (buffer full)      │
└─────────────────────────────────────┘
```

#### With `MY_CAN_FAST_SLOT_ACCESS` (O(1) free list):

```
┌─────────────────────────────────────┐
│      _findEmptyCanSlot()            │
├─────────────────────────────────────┤
│                                     │
│   ┌───────────────────────┐         │
│   │ freeListHead != NULL? │         │
│   └───────────┬───────────┘         │
│          yes  │  no                 │
│   ┌───────────┴───────────┐         │
│   ▼                       ▼         │
│ ┌─────────────────┐   ┌─────────┐   │
│ │slot = freeListHead│ │ return  │   │
│ │freeListHead =   │   │ nullptr │   │
│ │  slot->next     │   └─────────┘   │
│ │return slot      │                 │
│ └─────────────────┘                 │
│                                     │
└─────────────────────────────────────┘
```

### 4. Implementation Comparison: Find Existing Slot

#### Without `MY_CAN_FAST_SLOT_ACCESS` (O(n) scan):

```
┌─────────────────────────────────────┐
│      _findCanSlot()                 │
├─────────────────────────────────────┤
│                                     │
│   for i = 0 to BUF_SIZE:            │
│       ┌───────────────────┐         │
│       │ IN_USE &&         │         │
│       │ !READY &&         │         │
│       │ searchId match && │         │
│       │ frame not recv'd? │         │
│       └─────────┬─────────┘         │
│            yes  │  no               │
│       ┌─────────┴─────────┐         │
│       ▼                   ▼         │
│   ┌────────┐        ┌─────────┐     │
│   │ return │        │  next   │     │
│   │ slot   │        │   i++   │     │
│   └────────┘        └─────────┘     │
│                                     │
│   return nullptr (not found)        │
└─────────────────────────────────────┘
```

#### With `MY_CAN_FAST_SLOT_ACCESS` (O(m) linked list):

```
┌─────────────────────────────────────┐
│      _findCanSlot()                 │
├─────────────────────────────────────┤
│                                     │
│   slot = searchListFirst            │
│                                     │
│   while slot != NULL:               │
│       ┌───────────────────┐         │
│       │ searchId match && │         │
│       │ frame not recv'd? │         │
│       └─────────┬─────────┘         │
│            yes  │  no               │
│       ┌─────────┴─────────┐         │
│       ▼                   ▼         │
│   ┌────────┐        ┌─────────────┐ │
│   │ return │        │slot = slot->│ │
│   │ slot   │        │    next     │ │
│   └────────┘        └─────────────┘ │
│                                     │
│   return nullptr (not found)        │
└─────────────────────────────────────┘
```

**Key difference**: With FAST_SLOT_ACCESS, we only scan `m` active slots (being assembled), not `n` total slots.

### 5. Implementation Comparison: Get Ready Message

#### Without `MY_CAN_FAST_SLOT_ACCESS` (O(n) scan):

```
┌─────────────────────────────────────┐
│     _getOldestReadySlot()           │
├─────────────────────────────────────┤
│                                     │
│   oldest = nullptr                  │
│                                     │
│   for i = 0 to BUF_SIZE:            │
│       ┌───────────────────┐         │
│       │ SLOT_IS_READY &&  │         │
│       │ older than best?  │         │
│       └─────────┬─────────┘         │
│            yes  │  no               │
│       ┌─────────┴─────────┐         │
│       ▼                   ▼         │
│   ┌────────────┐    ┌─────────┐     │
│   │oldest=slot │    │  next   │     │
│   └────────────┘    │   i++   │     │
│                     └─────────┘     │
│                                     │
│   return oldest                     │
└─────────────────────────────────────┘
```

#### With `MY_CAN_FAST_SLOT_ACCESS` (O(1) list head):

```
┌─────────────────────────────────────┐
│     _getOldestReadySlot()           │
├─────────────────────────────────────┤
│                                     │
│   slot = oldestReady                │
│                                     │
│   if slot != NULL:                  │
│       oldestReady = slot->newer     │
│       if oldestReady != NULL:       │
│           oldestReady->older = NULL │
│       else:                         │
│           newestReady = NULL        │
│                                     │
│   return slot                       │
│                                     │
└─────────────────────────────────────┘
```

---

## Data Structure Visualization

### Without `MY_CAN_FAST_SLOT_ACCESS`

```
canSlots[] array (linear scan for all operations):

┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
│Slot 0│Slot 1│Slot 2│Slot 3│Slot 4│Slot 5│Slot 6│Slot 7│
│ FREE │IN_USE│READY │ FREE │IN_USE│ FREE │READY │ FREE │
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
   ↑       ↑      ↑      ↑      ↑      ↑      ↑      ↑
   └───────┴──────┴──────┴──────┴──────┴──────┴──────┘
              Must scan ALL for any operation
```

### With `MY_CAN_FAST_SLOT_ACCESS`

```
Three separate linked lists for O(1) operations:

FREE LIST (for allocation):
┌──────────────┐
│ freeListHead │──▶ Slot 0 ──▶ Slot 3 ──▶ Slot 5 ──▶ Slot 7 ──▶ NULL
└──────────────┘

SEARCH LIST (active slots being assembled):
┌─────────────────┐
│ searchListFirst │──▶ Slot 1 ◀──▶ Slot 4 ──▶ NULL
└─────────────────┘

READY LIST (complete messages, FIFO):
┌─────────────┐                              ┌─────────────┐
│ oldestReady │──▶ Slot 2 ◀──▶ Slot 6 ◀──────│ newestReady │
└─────────────┘    (head)        (tail)      └─────────────┘
```

---

## Memory Usage Summary

### Per-Slot Memory

| Field | Size | Notes |
|-------|------|-------|
| data[32] | 32 bytes | Message payload |
| timestamp | 4 bytes | Slot allocation time |
| searchId | 4 bytes | From-addr + msg-id |
| next/prev | 8 bytes | Only with FAST_SLOT_ACCESS |
| frameReceived | 1 byte | Bitmask (4 frames max) |
| totalFrames | 1 byte | Expected frame count |
| len | 1 byte | Accumulated length |
| flags | 1 byte | IN_USE + READY flags |
| **Total** | **44/52 bytes** | Without/with FAST_SLOT |

### Global Memory

| Variable | Without FAST | With FAST |
|----------|-------------|-----------|
| canSlots[n] | n × 44 bytes | n × 52 bytes |
| freeCanSlots | 1 byte | 1 byte |
| searchListFirst | - | 4 bytes |
| oldestReady | - | 4 bytes |
| newestReady | - | 4 bytes |
| freeListHead | - | 4 bytes |
| **Overhead** | **0 bytes** | **16 bytes** |

### Example: MY_CAN_BUF_SIZE = 8

| Config | Slot Memory | Overhead | Total |
|--------|-------------|----------|-------|
| Without FAST | 352 bytes | 0 | 352 bytes |
| With FAST | 416 bytes | 16 | 432 bytes |
| **Difference** | | | **+80 bytes** |

---

## When to Use Each Mode

### Use Standard Mode (no `MY_CAN_FAST_SLOT_ACCESS`):
- Very memory-constrained systems
- Small buffer sizes (≤ 4 slots)
- Low message throughput
- Simple deployment (single node communication)

### Use Fast Slot Access Mode:
- Higher message throughput requirements
- Larger buffer sizes (8+ slots)
- Multiple concurrent multi-frame messages
- Gateway nodes handling many sources
- Real-time requirements (consistent O(1) operations)

---

## Thread Safety

All slot manipulation functions use critical sections to protect against CAN RX interrupts:

```c
CAN_ENTER_CRITICAL();    // Disable CAN RX interrupt
// ... slot operations ...
CAN_EXIT_CRITICAL();     // Re-enable CAN RX interrupt
```

Platform-specific implementations:
- **STM32**: `NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn)` (fine-grained)
- **MCP2515**: `noInterrupts()` / `interrupts()` (global)
- **Linux**: No-op (no ISR context)

---

## Error Handling

| Error | Detection | Action |
|-------|-----------|--------|
| Invalid frame length | `len > 8` | Skip frame, log warning |
| Frame out of bounds | `frame >= 4` or overflow | Skip frame, log warning |
| Duplicate frame | `frameReceived & mask` | Skip frame, log warning |
| Length overflow | `len + new > 32` | Skip frame, log warning |
| Buffer full | No free slots | Evict oldest slot |
| Slot timeout | `micros() - timestamp > MAX_AGE` | Clean slot |
| Bus-off | TEC >= 255 or BOFF flag | Attempt recovery |
