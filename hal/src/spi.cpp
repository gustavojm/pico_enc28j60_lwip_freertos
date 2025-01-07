#include "spi.h"
#include "hardware/gpio.h"

namespace drivers {

Spi::Spi(const Config config) : config_{config} {}

bool Spi::init() {

    spi_init(config_.spi_handle, config_.baudrate_Hz);
    spi_set_format(config_.spi_handle, 8, config_.clock_polarization, config_.clock_phase,
                   config_.bit_order);

    gpio_set_function(config_.MISO_gpio, GPIO_FUNC_SPI);
    gpio_set_function(config_.MOSI_gpio, GPIO_FUNC_SPI);
    gpio_set_function(config_.CLK_gpio, GPIO_FUNC_SPI);

    return true;
}

size_t Spi::read(uint8_t *dst, const size_t len) {
    return spi_read_blocking(config_.spi_handle, 0, dst, len);
}

bool Spi::write(const uint8_t *src, const size_t len) {
    if (spi_write_blocking(config_.spi_handle, src, len) != len) {
        return false;
    }
    return true;
}

bool Spi::transceive(const uint8_t *src, uint8_t *dst, const size_t tx_len,
                            const size_t rx_len) {
    if (not write(src, tx_len)) {
        return false;
    }
    if (not read(dst, rx_len)) {
        return false;
    }

    return true;
}

}; // namespace drivers
