#ifndef STM32FxCAN_h
#define STM32FxCAN_h

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
    uint32_t _actualMask = 0;
    bool _initialized = false;
    uint8_t _speed;

    bool isInitialized();

  public:
    STM32FxCAN(uint8_t _CS = -1);
    uint8_t begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset = 0);   // Initialize controller parameters
    uint8_t init_Mask(uint8_t num, uint8_t ext, uint32_t ulData);               // Initialize Mask(s)
    uint8_t init_Mask(uint8_t num, uint32_t ulData);                            // Initialize Mask(s)
    uint8_t init_Filt(uint8_t num, uint8_t ext, uint32_t ulData);               // Initialize Filter(s)
    uint8_t init_Filt(uint8_t num, uint32_t ulData);                            // Initialize Filter(s)
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
};

#endif // STM32FxCAN_h