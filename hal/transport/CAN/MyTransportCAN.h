
bool CAN_transportInit(void);

bool CAN_transportSend(const uint8_t to, const void* data, const uint8_t len, const bool noACK);

bool CAN_transportDataAvailable(void);

uint8_t CAN_transportReceive(void *data, const uint8_t maxBufSize);

void CAN_transportTask(void);

void CAN_transportSetAddress(const uint8_t address);

uint8_t CAN_transportGetAddress(void);

bool CAN_transportSanityCheck(void);

void CAN_transportPowerDown(void);

void CAN_transportPowerUp(void);

void CAN_transportSleep(void);

void CAN_transportStandBy(void);

int16_t CAN_transportGetSendingRSSI(void);

int16_t CAN_transportGetReceivingRSSI(void);

int16_t CAN_transportGetSendingSNR(void);

int16_t CAN_transportGetReceivingSNR(void);

int16_t CAN_transportGetTxPowerPercent(void);

int16_t CAN_transportGetTxPowerLevel(void);

bool CAN_transportSetTxPowerPercent(const uint8_t powerPercent);

