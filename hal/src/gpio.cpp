#include "gpio.h"
#include "hardware/gpio.h"

namespace drivers {

Gpio::Gpio(uint32_t gpio, bool  out) : gpio_{gpio}, out_{out} {}

Gpio::Gpio(uint32_t gpio) : gpio_{gpio} {}

void Gpio::input() {
    out_ = false;
    gpio_init(gpio_);
    gpio_set_dir(gpio_, out_);
}

void Gpio::output() {
    out_ = true;
    gpio_init(gpio_);
    gpio_set_dir(gpio_, out_);
}

void Gpio::pull_up() {
    gpio_pull_up(gpio_);
}

void Gpio::pull_down() {
    gpio_pull_down(gpio_);
}

void Gpio::no_pulls() {
    gpio_disable_pulls(gpio_);
}

void Gpio::set() { gpio_put(gpio_, 1); }

void Gpio::reset() { gpio_put(gpio_, 0); }

uint32_t Gpio::get_gpio() {
    return gpio_;
};

} // namespace drivers
