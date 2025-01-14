#include "FreeRTOS.h"
#include "task.h"

#include "netif/etharp.h"
#include "enc28j60.h"
#include "enc28j60_registers.h"
#include "enc28j60_FreeRtos_lwip.h"
#include "errno.h"
#include "pico/stdlib.h"
#include "stdio.h"
#include "utils.h"
#include <cstring>

namespace drivers {
    
enc28j60::enc28j60(Config &config) : config_{config} {}

void enc28j60::lock() { xSemaphoreTakeRecursive(config_.mutex, portMAX_DELAY); }

void enc28j60::unlock() { xSemaphoreGiveRecursive(config_.mutex); }

void enc28j60::irq_deferred_handler() {

    while (true) {
        if (xSemaphoreTake(irq_loop_sem, portMAX_DELAY) == pdPASS) {
            int intflags;
            /* disable further interrupts */
            write_op(ENC28J60_BIT_FIELD_CLR, EIE, EIE_INTIE);

            intflags = read_reg(EIR);
            /* DMA interrupt handler (not currently used) */
            if ((intflags & EIR_DMAIF) != 0) {
                write_op(ENC28J60_BIT_FIELD_CLR, EIR, EIR_DMAIF);
            }
            /* LINK changed handler */
            if ((intflags & EIR_LINKIF) != 0) {
                if (is_link_up()) {
                    netif_set_link_up(&net_if);
                } else {
                    netif_set_link_down(&net_if);
                }

                // enc28j60_check_link_status(ndev);
                /* read PHIR to clear the flag */
                read_phy(PHIR);
            }
            /* TX complete handler */
            if (((intflags & EIR_TXIF) != 0) && ((intflags & EIR_TXERIF) == 0)) {
                bool err = false;
                //printf("intTX\n");
                if (read_reg(ESTAT) & ESTAT_TXABRT) {
                    printf("Tx Error (aborted)\n");
                    err = true;
                }
                tx_clear(err);
                write_op(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXIF);
            }
            /* TX Error handler */
            if ((intflags & EIR_TXERIF) != 0) {
                // uint8_t tsv[TSV_SIZE];                
                printf("intTXErr\n");
                write_op(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRTS);
                //  enc28j60_read_tsv(priv, tsv);                
                //  enc28j60_dump_tsv(priv, "Tx Error", tsv);
                    printf("TX Error");
                    LINK_STATS_INC(link.err);
                /* Reset TX logic */
                lock();
                write_op(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
                write_op(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
                txfifo_init(TXSTART_INIT, TXEND_INIT);
                unlock();
                /* Transmit Late collision check for retransmit */
                // if (TSV_GETBIT(tsv, TSV_TXLATECOLLISION)) {
                // 	if (netif_msg_tx_err(priv))
                // 		netdev_printk(KERN_DEBUG, ndev,
                // 			      "LateCollision TXErr (%d)\n",
                // 			      priv->tx_retry_count);
                // 	if (priv->tx_retry_count++ < MAX_TX_RETRYCOUNT)
                // 		write_op(ENC28J60_BIT_FIELD_SET ECON1,
                // 				   ECON1_TXRTS);
                // 	else
                // 		tx_clear(ndev, true);
                // } else
                // 	tx_clear(ndev, true);
                write_op(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF | EIR_TXIF); // should be
                                                                              // locked
            }
            /* RX Error handler */
            if ((intflags & EIR_RXERIF) != 0) {
                printf("intRXErr\n");
                /* Check free FIFO space to flag RX overrun */
                if (get_free_rxfifo() <= 0) {
                    printf("RX Overrun\n");
                }
                write_op(ENC28J60_BIT_FIELD_CLR, EIR, EIR_RXERIF); // should be locked
            }
            /* RX handler */
            int pk_counter = read_reg(EPKTCNT);
            while (pk_counter-- > 0) {
                auto packet_info = get_incoming_packet_info();

                pbuf *ptr = pbuf_alloc(PBUF_RAW, packet_info.byte_count, PBUF_RAM);
                if (ptr != nullptr) {
                    get_incoming_packet(packet_info, (uint8_t *)ptr->payload,
                                        packet_info.byte_count);

                    LINK_STATS_INC(link.recv);
#ifdef ENC_DEBUG_ON
                    printf("Received packet with len %d!\r\n",
                            packet_info.byte_count);
#endif

                    if (net_if.input(ptr, &net_if) != ERR_OK) {
                        printf("Error processing frame input\r\n");
                        pbuf_free(ptr);
                    }
                }
            }

            /* re-enable interrupts */
            write_op(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE); // should be locked
        }
    }
}

extern "C" void enc28j60_irq_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Give a semaphore for irq_loop
    xSemaphoreGiveFromISR(irq_loop_sem, &xHigherPriorityTaskWoken);
    gpio_acknowledge_irq(gpio, events);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    // Acknowledge the interrupt
    return;
}

bool enc28j60::init(const MacAddress &mac_address) {
    config_.mutex = xSemaphoreCreateRecursiveMutex();
    if (config_.mutex) {
        printf(" * * * * MUTEX CREATED * * * *");
    }
    config_.Rst.init();
    config_.Cs.init();

    config_.Cs.set();
    config_.Rst.reset();
    vTaskDelay(pdMS_TO_TICKS(AFTER_RESET_DELAY_MS));
    config_.Rst.set();

    write_op(ENC28J60_SOFT_RESET, 0x00, ENC28J60_SOFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(2));
    /** Oscillator ready */
    while (!(read_reg(ESTAT) & ESTAT_CLKRDY))
        ;

    /** RX buffer ptr */
    write_reg16(ERXST, RXSTART_INIT);
    write_reg16(ERXRDPT, RXSTART_INIT);
    write_reg16(ERXND, RXSTOP_INIT);

    /** TX buffer ptr */
    write_reg16(ETXST, TXSTART_INIT);
    write_reg16(ETXND, TXSTOP_INIT);

    write_phy(PHLCON, 0x476);

    read_phy(PHHID1);
    /** FILTERS setup */
    write_reg(ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_PMEN | ERXFCON_BCEN);

    write_reg16(EPMM0, 0x303f);
    write_reg16(EPMCS, 0xf7f9);

    /** Set the MARXEN bit in MACON1 to enable the MAC to receive frames. If using full duplex, most
     * applications should also set TXPAUS and RXPAUS to allow IEEE defined flow control to function
     */
    select_bank(MACON1);
    write_op(ENC28J60_BIT_FIELD_SET, MACON1, MACON1_MARXEN);
    /** Configure the PADCFG, TXCRCEN and FULDPX bits of MACON3. */
    write_op(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN);

    /** Program the MAMXFL registers with the maxi- mum frame length to be permitted to be received
     * or transmitted. MAX PDU - offset */
    write_reg16(MAMXFL, 1500);

    /** Configure the Back-to-Back Inter-Packet Gap register, MABBIPG. Most applications will pro-
     * gram this register with 15h when Full-Duplex mode is used and 12h when Half-Duplex mode is
     * used. */
    write_reg(MABBIPG, 0x12);
    read_reg(MABBIPG);

    /** Configure the Non-Back-to-Back Inter-Packet Gap register low byte, MAIPGL. Most applications
     * will program this register with 12h. If half duplex is used, the Non-Back-to-Back
     * Inter-Packet Gap register high byte, MAIPGH, should be programmed. Most applications will
     * program this register to 0Ch.*/
    write_reg16(MAIPG, 0x0C12);

    read_reg(MAIPG);
    read_reg(0x07 | 0x40 | 0x80);

    write_reg(MAADR5, mac_address[0]);
    write_reg(MAADR4, mac_address[1]);
    write_reg(MAADR3, mac_address[2]);
    write_reg(MAADR2, mac_address[3]);
    write_reg(MAADR1, mac_address[4]);
    write_reg(MAADR0, mac_address[5]);
    mac_ = mac_address;

    write_phy(PHCON2, PHCON2_HDLDIS);

    /** Start receiving */
    select_bank(ECON1);
    write_op(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE | EIE_PKTIE | EIR_LINKIF);
    write_op(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);

    uint8_t rev = read_reg(EREVID);
    unlock();

    return rev > 0;
}

void enc28j60::enable_interupts() {
        // Enabling Interrupts

    lock();
    #define IRQ_GPIO 22
    gpio_init(IRQ_GPIO);
    gpio_set_dir(IRQ_GPIO, GPIO_IN);
    gpio_pull_up(IRQ_GPIO);
    // gpio_disable_pulls(IRQ_GPIO);
    gpio_set_irq_enabled_with_callback(IRQ_GPIO, GPIO_IRQ_EDGE_FALL, true, &enc28j60_irq_callback);

    write_phy(PHIE, PHIE_PGEIE | PHIE_PLNKIE);
    
    write_op(ENC28J60_BIT_FIELD_CLR, EIR,
             EIR_DMAIF | EIR_LINKIF | EIR_TXIF | EIR_TXERIF | EIR_RXERIF | EIR_PKTIF);
    write_reg(EIE, EIE_INTIE | EIE_PKTIE | EIE_LINKIE | EIE_TXIE | EIE_TXERIE | EIE_RXERIE);

    /* enable receive logic */
    write_op(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
    write_op(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE); // should be locked

    unlock();
}

bool enc28j60::is_link_up() { return (read_phy(PHSTAT2) & PHSTAT2_LSTAT); }

void enc28j60::write_op(const uint8_t op, const uint8_t addr, const uint8_t data) {
    lock();
    config_.Cs.reset();
    const uint8_t operation = op | (addr & ENC_ADDR_MASK);
    config_.spi.write(&operation, sizeof(operation));
    config_.spi.write(&data, sizeof(data));
    config_.Cs.set();
    unlock();
}

uint8_t enc28j60::read_op(const uint8_t op, const uint8_t reg) {
    lock();
    config_.Cs.reset();
    const uint8_t operation = op | (reg & ENC_ADDR_MASK);
    uint8_t incoming_data{};

    config_.spi.write(&operation, 1);
    config_.spi.read(&incoming_data, 1);

    if (reg & 0x80) {
        /** @note If this is MAC register, then read dummy byte first */
        config_.spi.read(&incoming_data, 1);
    }

    config_.Cs.set();
    unlock();
    return incoming_data;
}

void enc28j60::select_bank(const uint8_t address) {
    if (current_register_bank != (address & BANK_MASK)) {
        write_op(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_BSEL0 | ECON1_BSEL1);
        write_op(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK) >> 5);
        current_register_bank = address & BANK_MASK;
    }
}

void enc28j60::write_reg(const uint8_t addr, const uint8_t data) {
    lock();
    select_bank(addr);
    write_op(ENC28J60_WRITE_CTRL_REG, addr, data);
    unlock();
}

void enc28j60::write_reg16(const uint8_t addr, const uint16_t data) {
    //    enc28j60::write_op_16bit(ENC28J60_WRITE_CTRL_REG, addr, data);
    lock();
    write_reg(addr, data & 0xff);
    write_reg(addr + 1, data >> 8);
    unlock();
}

uint8_t enc28j60::read_reg(const uint8_t reg) {
    lock();
    select_bank(reg);
    uint8_t res = read_op(ENC28J60_READ_CTRL_REG, reg);
    unlock();
    return res;
}

uint16_t enc28j60::read_reg16(const uint8_t reg) {
    lock();
    uint8_t res = read_reg(reg) + (read_reg(reg + 1) << 8);
    unlock();
    return res;
}

void enc28j60::write_phy(const uint8_t reg, const uint16_t data) {
    /** 1. Write the address of the PHY register to write to into the MIREGADR register. */
    write_reg(MIREGADR, reg);
    /** 2. Write the lower 8 bits of data to write into the MIWRL register. */
    write_reg16(MIWR, data);
    /** 3. Write the upper 8 bits of data to write into the MIWRH register.
     * Writing to this register auto- matically begins the MIIM transaction, so it must be written
     * to after MIWRL. The MISTAT.BUSY bit becomes set. */
    while (enc28j60::read_reg(MISTAT) & MISTAT_BUSY)
        ;
}

uint16_t enc28j60::read_phy(const uint8_t reg) {
    /** 1. Write the address of the PHY register to read from into the MIREGADR register.  */
    enc28j60::write_reg(MIREGADR, reg);
    uint8_t xd = enc28j60::read_reg(MIREGADR);

    /** 2. Set the MICMD.MIIRD bit. The read operation begins and the MISTAT.BUSY bit is set. */
    enc28j60::write_reg(MICMD, MICMD_MIIRD);

    /** 3. Wait 10.24 μs. Poll the MISTAT.BUSY bit to be certain that the operation is complete.
     While busy, the host controller should not start any MIISCAN operations or write to the MIWRH
    register. When the MAC has obtained the register contents, the BUSY bit will clear itself.  */
    while (enc28j60::read_reg(MISTAT) & MISTAT_BUSY)
        ;

    /** 4. Clear the MICMD.MIIRD bit. */
    enc28j60::write_reg(MICMD, 0x00);

    /** 5. Read the desired data from the MIRDL and MIRDH registers. The order that these bytes are
     * accessed is unimportant. */
    uint8_t out_L = enc28j60::read_reg(MIRDL);
    uint8_t out_H = enc28j60::read_reg(MIRDH);
    return (out_H << 8) | out_L;
}

size_t enc28j60::read_buff(uint8_t *src, size_t len) {
    lock();
    config_.Cs.reset();
    const uint8_t operation = ENC28J60_READ_BUF_MEM;
    config_.spi.write(&operation, 1);
    auto ret = config_.spi.read(src, len);
    config_.Cs.set();
    unlock();
    return ret;
}

void enc28j60::write_buff(const uint8_t *src, size_t len) {
    lock();
    config_.Cs.reset();
    const uint8_t operation = ENC28J60_WRITE_BUF_MEM;
    config_.spi.write(&operation, 1);
    config_.spi.write(src, len);
    config_.Cs.set();
    unlock();
}

uint8_t enc28j60::get_number_of_packets() {
    uint8_t n = read_reg(EPKTCNT);
    return n;
}

size_t enc28j60::get_incoming_packet(const PacketMetaInfo &info, uint8_t *dst,
                                     const size_t length) {

    size_t bytes_read;
    if (dst != nullptr) {
        bytes_read = read_buff(dst, length);
    }

    next_packet_pointer = info.next_packet_pointer;
    write_reg16(ERXRDPT, info.next_packet_pointer);
    write_op(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);

    return bytes_read;
}

struct transmit_status_vector {
    uint8_t bytes[7];
};

#define ETHERCARD_SEND_PIPELINING 1

#if ETHERCARD_SEND_PIPELINING
#define BREAKORCONTINUE                                                                            \
    retry = 0;                                                                                     \
    continue;
#else
#define BREAKORCONTINUE break;
#endif

bool enc28j60::send_packet(const uint8_t *src, const size_t len) {
    uint8_t retry = 0;

#if ETHERCARD_SEND_PIPELINING
    goto resume_last_transmission;
#endif
    while (1) {
        // latest errata sheet: DS80349C
        // always reset transmit logic (Errata Issue 12)
        // the Microchip TCP/IP stack implementation used to first check
        // whether TXERIF is set and only then reset the transmit logic
        // but this has been changed in later versions; possibly they
        // have a reason for this; they don't mention this in the errata
        // sheet
        write_op(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
        write_op(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
        write_op(ENC28J60_BIT_FIELD_CLR, EIR, EIR_TXERIF | EIR_TXIF);

        // prepare new transmission
        if (retry == 0) {
            // Set the write pointer to start of transmit buffer area
            write_reg16(EWRPT, TXSTART_INIT);

            // Set the TXND pointer to correspond to the packet size given
            write_reg16(ETXND, TXSTART_INIT + len);

            // Write per-packet control byte
            write_op(ENC28J60_WRITE_BUF_MEM, 0, 0x00);

            // Copy the packet into the transmit buffer
            write_buff(src, len);
        }

        // Initiate transmission
        write_op(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
#if ETHERCARD_SEND_PIPELINING
        if (retry == 0)
            return true;
#endif

    resume_last_transmission:

        // wait until transmission has finished; referring to the data sheet and
        // to the errata (Errata Issue 13; Example 1) you only need to wait until either
        // TXIF or TXERIF gets set; however this leads to hangs; apparently Microchip
        // realized this and in later implementations of their tcp/ip stack they introduced
        // a counter to avoid hangs; of course they didn't update the errata sheet
        uint16_t count = 0;
        while ((read_reg(EIR) & (EIR_TXIF | EIR_TXERIF)) == 0 && ++count < 1000U) {
        }

        if (!(read_reg(EIR) & EIR_TXERIF) && count < 1000U) {
            // no error; start new transmission
            BREAKORCONTINUE
        }

        // Cancel previous transmission if stuck
        write_op(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRTS);

#if ETHERCARD_RETRY_LATECOLLISIONS == 0
        BREAKORCONTINUE
#endif

        // Check whether the chip thinks that a late collision occurred; the chip
        // may be wrong (Errata Issue 13); therefore we retry. We could check
        // LATECOL in the ESTAT register in order to find out whether the chip
        // thinks a late collision occurred but (Errata Issue 15) tells us that
        // this is not working. Therefore we check TSV
        transmit_status_vector tsv;
        uint16_t etxnd = read_reg16(ETXND);
        write_reg16(ERDPT, etxnd + 1);
        read_buff((uint8_t *)&tsv, sizeof(transmit_status_vector));
        // LATECOL is bit number 29 in TSV (starting from 0)

        if (!((read_reg(EIR) & EIR_TXERIF) &&
              (tsv.bytes[3] & 1 << 5) /*tsv.transmitLateCollision*/) ||
            retry > 16U) {
            // there was some error but no LATECOL so we do not repeat
            BREAKORCONTINUE
        }

        retry++;
    }
    return false;
}

enc28j60::PacketMetaInfo enc28j60::get_incoming_packet_info() {
    PacketMetaInfo ret{};
    write_reg16(ERDPT, next_packet_pointer);
    read_buff(reinterpret_cast<uint8_t *>(&ret), sizeof(PacketMetaInfo));

    return ret;
}

bool enc28j60::link_state_changed() {
    if (current_link_state != is_link_up()) {
        current_link_state = !current_link_state;
        return true;
    }

    return false;
}

int enc28j60::poll_ready(uint8_t reg, uint8_t mask, uint8_t val) {
    TickType_t start = xTaskGetTickCount();

    /* 20 msec timeout read */
    while ((read_reg(reg) & mask) != val) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(20)) {
            return -ETIMEDOUT;
        }
        asm("nop");
    }
    return 0;
}

/*
 * Wait until the PHY operation is complete.
 */
int enc28j60::wait_phy_ready() { return poll_ready(MISTAT, MISTAT_BUSY, 0) ? 0 : 1; }

void enc28j60::txfifo_init(uint16_t start, uint16_t end) {
    if (start > 0x1FFF || end > 0x1FFF || start > end) {
        // printf("%s(%d, %d) TXFIFO bad parameters!\n",
        // 		__func__, start, end);
        return;
    }
    /* set transmit buffer start + end */
    write_reg16(ETXST, start); // ETXSTL
    write_reg16(ETXND, end);   // ETXNDL
}

void enc28j60::tx_clear(bool err) {
    write_op(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRTS); // Should be locked                                                          
}

/*
 * Calculate free space in RxFIFO
 */
int enc28j60::get_free_rxfifo() {
    int epkcnt, erxst, erxnd, erxwr, erxrd;
    int free_space;

    lock();
    epkcnt = read_reg(EPKTCNT);
    if (epkcnt >= 255)
        free_space = -1;
    else {
        erxst = read_reg16(ERXST);
        erxnd = read_reg16(ERXND);
        erxwr = read_reg16(ERXWRPT);
        erxrd = read_reg16(ERXRDPT);

        if (erxwr > erxrd)
            free_space = (erxnd - erxst) - (erxwr - erxrd);
        else if (erxwr == erxrd)
            free_space = (erxnd - erxst);
        else
            free_space = erxrd - erxwr - 1;
    }
    unlock();
    // printf("%s() free_space = %d\n", __func__, free_space);
    return free_space;
}

err_t enc28j60::eth_packet_output(struct netif *netif, struct pbuf *p) {
    LINK_STATS_INC(link.xmit);
    drivers::enc28j60 *me = static_cast<drivers::enc28j60 *>(netif->state);

    struct pbuf *q;
    for (q = p; q != nullptr; q = q->next) {
        // print_pbuf_payload(q);

        if (!me->send_packet(static_cast<uint8_t *>(q->payload), q->len)) {
            printf("Cannot send fragment of length %d\r\n", q->len);
            return ERR_ABRT;
        }
    }
    //
#if ENC_DEBUG_ON
    printf("Sent packet with len %d[%d]!\r\n", p->len, p->tot_len);
#endif
    return ERR_OK;
}


err_t enc28j60::eth_netif_init(struct netif *netif) {    
    drivers::enc28j60 *me = static_cast<drivers::enc28j60 *>(netif->state);

    netif->linkoutput = drivers::enc28j60::eth_packet_output;
    netif->output = etharp_output;
    netif->mtu = ETHERNET_MTU;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET |
                   NETIF_FLAG_IGMP | NETIF_FLAG_MLD6;
    memcpy(netif->hwaddr, me->mac_.data(), sizeof(netif->hwaddr));
    netif->hwaddr_len = sizeof(netif->hwaddr);

    printf("LWIP Init \n");

    return ERR_OK;
}

} // namespace drivers::enc28j60
