#pragma once

/**
 * @note This driver bases on EtherCard repository.
 * License: GPL-2.0 license
 * URL: https://github.com/njh/EtherCard
 */

#include "gpio.h"
#include "spi.h"
#include <array>
#include <cinttypes>

#include "FreeRTOS.h"
#include "lwip/err.h"
#include "lwip/netif.h"
#include "semphr.h"

#include <cinttypes>

inline SemaphoreHandle_t irq_loop_sem;

namespace drivers {

class enc28j60 {
    static constexpr uint16_t RXSTART_INIT = 0x0;
    static constexpr uint16_t RXSTOP_INIT = (0x1FFF - 0x0600 - 1);
    static constexpr uint16_t TXSTART_INIT = (0x1FFF - 0x0600);
    static constexpr uint16_t TXEND_INIT = 0x1FFF;
    static constexpr uint16_t TXSTOP_INIT = 0x1FFF;
    static constexpr uint32_t AFTER_RESET_DELAY_MS = 100;
    static constexpr size_t ETHERNET_MTU = 1500;

  public:
    struct Config {
        Gpio CS_gpio;
        Gpio RST_gpio;
        Gpio IRQ_gpio;
        Spi &spi;
        SemaphoreHandle_t mutex;
    };
    
    using MacAddress = std::array<uint8_t, 6>;

    struct __attribute__((packed)) PacketMetaInfo {
        uint16_t next_packet_pointer;
        uint16_t byte_count;
        uint16_t long_drop_event : 1;
        uint16_t reserved : 1;
        uint16_t carrier_event_previously_seen : 1;
        uint16_t reserved_2 : 1;
        uint16_t crc_err : 1;
        uint16_t length_check_err : 1;
        uint16_t length_out_of_range : 1;
        uint16_t received_ok : 1;
        uint16_t receive_multicast_packet : 1;
        uint16_t receive_broadcast_packet : 1;
        uint16_t dribble_nibble : 1;
        uint16_t receive_control_frame : 1;
        uint16_t receive_pause_control_frame : 1;
        uint16_t receive_unknown_opcode : 1;
        uint16_t receive_vlan_type_detected : 1;
        uint16_t zero : 1;
    };

    enc28j60(Config config);

    void lock();
    void unlock();
    void send_packet(uint16_t len);
    bool init(const MacAddress &mac_address);
    bool is_link_up();
    uint8_t get_number_of_packets();
    size_t get_incoming_packet(const PacketMetaInfo &info, uint8_t *dst, const size_t max_length);
    PacketMetaInfo get_incoming_packet_info();
    bool send_packet(const uint8_t *src, const size_t len);
    bool link_state_changed();

  public:
    Config config_;
    MacAddress mac_;

    void irq_deferred_handler();

    void write_op(uint8_t operation, const uint8_t reg, const uint8_t data);
    uint8_t read_op(uint8_t operation, const uint8_t reg);
    void select_bank(const uint8_t address);

    void write_reg(const uint8_t addr, const uint8_t data);
    void write_reg16(const uint8_t addr, const uint16_t data);

    uint8_t read_reg(const uint8_t reg);
    uint16_t read_reg16(const uint8_t reg);

    void write_phy(const uint8_t reg, const uint16_t data);
    uint16_t read_phy(const uint8_t reg);

    size_t read_buff(uint8_t *dst, size_t len);
    void write_buff(const uint8_t *src, size_t len);

    uint8_t current_register_bank;
    uint16_t next_packet_pointer;
    bool current_link_state;

  public:
    int wait_phy_ready();
    int poll_ready(uint8_t reg, uint8_t mask, uint8_t val);
    void txfifo_init(uint16_t start, uint16_t end);
    bool enc28j60_irq(int irq);
    void tx_clear(bool err);
    int get_free_rxfifo();
    int rx_interrupt();
    void enable_interupts();
    static err_t eth_netif_init(struct netif *netif);
    static err_t eth_packet_output(struct netif *netif, struct pbuf *p);
};

} // namespace drivers
