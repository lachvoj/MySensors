#ifndef CANDEV_h
#define CANDEV_h

#include <inttypes.h>
#include <linux/can.h>

class CANDEVClass
{
  private:
    bool _initialized = false;
    char _canDevice[10];

  public:
    CANDEVClass(const char *canDevice);
    ~CANDEVClass();
    uint8_t begin(uint8_t idmodeset, uint8_t speedset, uint8_t clockset = 0);      // Initialize controller parameters
    uint8_t setFilterMask(can_filter *filters, uint8_t count);                     // Set filter and its mask
    uint8_t sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, const uint8_t *buf); // Send message to transmit buffer
    uint8_t sendMsgBuf(uint32_t id, uint8_t len, const uint8_t *buf);              // Send message to transmit buffer
    uint8_t readMsgBuf(uint32_t *id, uint8_t *ext, uint8_t *len, uint8_t *buf);    // Read message from receive buffer
    uint8_t readMsgBuf(uint32_t *id, uint8_t *len, uint8_t *buf);                  // Read message from receive buffer
    uint8_t checkReceive(void);                                                    // Check for received data
    uint8_t checkError(void);                                                      // Check for errors
    uint8_t getError(void);                                                        // Check for errors
    uint8_t errorCountRX(void);                                                    // Get error count
    uint8_t errorCountTX(void);                                                    // Get error count
    uint8_t enOneShotTX(void);                                                     // Enable one-shot transmission
    uint8_t disOneShotTX(void);                                                    // Disable one-shot transmission
    uint8_t abortTX(void);                                                         // Abort queued transmission(s)
    uint8_t setGPO(uint8_t data);                                                  // Sets GPO
    uint8_t getGPI(void);                                                          // Reads GPI
};

#endif // CANDEV_h