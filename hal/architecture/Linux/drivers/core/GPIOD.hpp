#ifndef GPIOD_hpp
#define GPIOD_hpp

#include <stdint.h>

#define INPUT 0
#define OUTPUT 1

#define LOW 0
#define HIGH 1

#define GPIOD_MAX_LINE_DEFINITIONS 64


#ifndef STRINGIFY
#define STRINGIFY(s) #s
#endif

#ifndef GPIOD_DEV
#define GPIOD_DEV STRINGIFY(/dev/gpiochip0)
#endif

/**
 * @brief GPIOD class
 */
class GPIODClass
{
  private:
    const char *chipdevname = GPIOD_DEV;

  public:
    struct gpiod_line *gpiod_lines[GPIOD_MAX_LINE_DEFINITIONS];
    struct gpiod_chip *chip;
    /**
     * @brief GPIODClass constructor.
     */
    GPIODClass();
    /**
     * @brief GPIODClass copy constructor.
     */
    GPIODClass(const GPIODClass &other);
    /**
     * @brief GPIODClass destructor.
     */
    ~GPIODClass();
    /**
     * @brief Configures the specified pin to behave either as an input or an output.
     *
     * @param pin The number of the pin.
     * @param mode INPUT or OUTPUT.
     */
    void pinMode(uint8_t pin, uint8_t mode);
    /**
     * @brief Write a high or a low value for the given pin.
     *
     * @param pin number.
     * @param value HIGH or LOW.
     */
    void digitalWrite(uint8_t pin, uint8_t value);
    /**
     * @brief Reads the value from a specified pin.
     *
     * @param pin The number of the pin.
     * @return HIGH or LOW.
     */
    uint8_t digitalRead(uint8_t pin);
    /**
     * @brief Arduino compatibility function, returns the same given pin.
     *
     * @param pin The number of the pin.
     * @return The same parameter pin number.
     */
    uint8_t digitalPinToInterrupt(uint8_t pin);
    /**
     * @brief Overloaded assign operator.
     *
     */
    GPIODClass &operator=(const GPIODClass &other);
};

extern GPIODClass GPIOD;

#endif
