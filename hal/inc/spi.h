#pragma once

#include "hardware/spi.h"
#include "spi.h"

namespace drivers {

class Spi  {
  public:
    struct Config {
        spi_inst_t *spi_handle;
        uint8_t CLK_gpio;
        uint8_t MOSI_gpio;
        uint8_t MISO_gpio;
        uint32_t baudrate_Hz;
        spi_cpol_t clock_polarization = SPI_CPOL_0;
        spi_cpha_t clock_phase = SPI_CPHA_0;
        spi_order_t bit_order = SPI_MSB_FIRST;
    };
    explicit Spi(const Config config);

    bool init();
    size_t read(uint8_t *dst, const size_t len);
    bool write(const uint8_t *src, const size_t len);
    bool transceive(const uint8_t *src, uint8_t *dst, const size_t tx_len,
                    const size_t rx_len);

  private:
    const Config config_;
};

} // namespace drivers
