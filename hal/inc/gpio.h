#pragma once
#include "gpio.h"
#include <cstdint>

namespace drivers {

class Gpio {
  public:
    Gpio(uint32_t gpio, bool out);
    Gpio(const uint32_t gpio);    
    void input();
    void output();
    void pull_up();
    void pull_down();
    void no_pulls();
    void set();
    void reset();
    uint32_t get_gpio();

  private:
    const uint32_t gpio_;
    bool out_ = false;
};

} // namespace drivers
