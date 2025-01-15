#pragma once
#include "gpio.h"
#include <cstdint>

namespace drivers {

class Gpio {
  public:
    Gpio(uint32_t gpio_pin, uint32_t dir);
    void init();
    void set();
    void reset();

  private:
    const uint32_t pin_;
    const uint32_t dir_;
};

} // namespace drivers
