#ifndef STM32FxCAN_h
#define STM32FxCAN_h

#include <stm32_def.h>

#include "../mcp_can_dfs.h"

#ifndef STM32FxCAN_BUS_TYPE
#define STM32FxCAN_BUS_TYPE 2
#endif

#ifndef STM32FxCAN_BUS_PULLUP
#define STM32FxCAN_BUS_PULLUP 0
#endif

class STM32FxCAN
{
  private:
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

    bool _initialized = false;
    uint8_t _speed;
    CAN_TypeDef *_canDev;

    bool isInitialized();
    uint8_t send(CAN_msg_t *CAN_tx_msg);
    uint8_t setFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2);

  public:
    STM32FxCAN(uint8_t canDevice = 1);
    uint8_t begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset = 0); // Initialize controller parameters
    uint8_t setFilterMask32(uint8_t index, uint32_t filter, uint32_t mask, uint8_t fifo = 0);
    uint8_t setFilterList32(uint8_t index, uint32_t filter1, uint32_t filter2, uint8_t fifo = 0);
    uint8_t setFilterMask16(
        uint8_t index,
        uint16_t filter1,
        uint16_t mask1,
        uint16_t filter2,
        uint16_t mask2,
        uint8_t fifo = 0);
    uint8_t setFilterList16(
        uint8_t index,
        uint16_t filter1,
        uint16_t filter2,
        uint16_t filter3,
        uint16_t filter4,
        uint8_t fifo = 0);
    uint8_t setMode(uint8_t opMode);                                            // Set operational mode
    uint8_t sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf);    // Send message to transmit buffer
    uint8_t sendMsgBuf(uint32_t id, uint8_t len, uint8_t *buf);                 // Send message to transmit buffer
    uint8_t readMsgBuf(uint32_t *id, uint8_t *ext, uint8_t *len, uint8_t *buf); // Read message from receive buffer
    uint8_t readMsgBuf(uint32_t *id, uint8_t *len, uint8_t *buf);               // Read message from receive buffer
    uint8_t checkReceive(void);                                                 // Check for received data
    uint8_t checkError(void);                                                   // Check for errors
    uint8_t getError(void);                                                     // Check for errors
    uint8_t errorCountRX(void);                                                 // Get error count
    uint8_t errorCountTX(void);                                                 // Get error count
    uint8_t enOneShotTX(void);                                                  // Enable one-shot transmission
    uint8_t disOneShotTX(void);                                                 // Disable one-shot transmission
    uint8_t abortTX(void);                                                      // Abort queued transmission(s)
    uint8_t setGPO(uint8_t data);                                               // Sets GPO
    uint8_t getGPI(void);                                                       // Reads GPI
    uint8_t enableRxInterrupt(void);
    uint8_t disableRxInterrupt(void);
    uint8_t attachRxInterrupt(void func());
};

#endif // STM32FxCAN_h