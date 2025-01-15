#include "enc_os_lwip_glue.h"
#include "enc28j60.h"
#include "lwip/err.h"

#include "lwip/dhcp.h"
#include "lwip/inet.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/stats.h"
#include "lwip/tcpip.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"

#include "FreeRTOS.h"
#include "pico/stdlib.h"
#include "semphr.h"
#include "task.h"
#include <cstring>

#include "gpio_wrapper.h"
#include "spi.h"
#include "utils.h"

#ifdef ENC_DEBUG_ON
#include <pico/stdio.h>
#endif

constexpr uint32_t NETWORKING_CORE_ID = 1 << 1; // pin all networking functions to core1
constexpr size_t ETHERNET_MTU = 1500;

static SemaphoreHandle_t worker_sem{};

void print_pbuf_payload(struct pbuf *p) {
    struct pbuf *current = p;
    while (current != NULL) {
        // Print the current pbuf's payload as a string (ensure it's null-terminated if needed)
        printf("Payload (len=%d): ", current->len);
        fwrite(current->payload, 1, current->len, stdout);
        printf("\n");

        // Move to the next pbuf in the chain
        current = current->next;
    }
}

err_t enc_eth_packet_output(struct netif *netif, struct pbuf *p) {
    LINK_STATS_INC(link.xmit);
    auto &controller = *static_cast<drivers::enc28j60::enc28j60 *>(netif->state);

    struct pbuf *q;
    for (q = p; q != nullptr; q = q->next) {
        // print_pbuf_payload(q);

        if (!controller.send_packet(static_cast<uint8_t *>(q->payload), q->len)) {
            printf("Cannot send fragment of length %d\r\n", q->len);
            return ERR_ABRT;
        }
    }
    //
#ifdef ENC_DEBUG_ON
    printf("Sent packet with len %d[%d]!\r\n", p->len, p->tot_len);
#endif
    return ERR_OK;
}

static void tcpip_init_done(void *arg) { xSemaphoreGive(static_cast<SemaphoreHandle_t>(arg)); }

static err_t enc_eth_netif_init(struct netif *netif) {

    netif->linkoutput = enc_eth_packet_output;
    netif->output = etharp_output;
    netif->mtu = ETHERNET_MTU;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET |
                   NETIF_FLAG_IGMP | NETIF_FLAG_MLD6;
    memcpy(netif->hwaddr, mac.data(), sizeof(netif->hwaddr));
    netif->hwaddr_len = sizeof(netif->hwaddr);

    printf("LWIP Init \n");

    return ERR_OK;
}

static void netif_status_callback(struct netif *netif) {
    printf("netif status changed %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
}

static void netif_link_callback(struct netif *netif) { printf("netif link changed\n"); }

static void enc_worker_thread(void *param) {
    xSemaphoreTake(worker_sem, portMAX_DELAY);

    drivers::enc28j60::enc28j60 &eth_driver =
        *static_cast<drivers::enc28j60::enc28j60 *>(net_if.state);
    pbuf *ptr = nullptr;

    while (true) {
//         if (eth_driver.link_state_changed()) {
//             if (eth_driver.is_link_up()) {
//                 netif_set_link_up(&net_if);
//                 printf("**** NETIF: LINK IS UP!\r\n");
//             } else {
//                 netif_set_link_down(&net_if);
//                 printf("**** NETIF: LINK IS DOWN!\r\n");
//             }
//         }

//         while (eth_driver.get_number_of_packets() > 0) {
//             auto packet_info = eth_driver.get_incoming_packet_info();

//             ptr = pbuf_alloc(PBUF_RAW, packet_info.byte_count, PBUF_RAM);
//             if (ptr != nullptr) {
//                 eth_driver.get_incoming_packet(packet_info, (uint8_t *)ptr->payload,
//                                                packet_info.byte_count);

//                 LINK_STATS_INC(link.recv);
// #ifdef ENC_DEBUG_ON
//                 // printf("Received packet with len %d!\r\n",
//                 //        packet_info.byte_count);
// #endif

//                 if (net_if.input(ptr, &net_if) != ERR_OK) {
//                     printf("Error processing frame input\r\n");
//                     pbuf_free(ptr);
//                 }
//             }
//         }
         vTaskDelay(pdMS_TO_TICKS(20));
    }
}

err_t enc_driver_os_init() {

    constexpr uint8_t MISO_PIN = 16;
    constexpr uint8_t MOSI_PIN = 19;
    constexpr uint8_t CLK_PIN = 18;
    constexpr uint8_t ENC_IRQ = 22;

    static drivers::gpio::Gpio EncRstPin{21, GPIO_OUT};
    static drivers::gpio::Gpio EncCsPin{17, GPIO_OUT};
    static drivers::spi::Config spi0Config{spi0, CLK_PIN, MOSI_PIN, MISO_PIN, 25 * 1000000};

    EncRstPin.init();
    EncCsPin.init();

    static drivers::spi::SpiWrapper spi0_{spi0Config};
    spi0_.init();

    static drivers::enc28j60::Config EncConfig{EncCsPin, EncRstPin, ENC_IRQ, spi0_};
    static drivers::enc28j60::enc28j60 eth_driver{EncConfig};
    if (!eth_driver.init(mac)) {
        hal::panic();
    }

    while (!eth_driver.is_link_up())
        ;

    ip4_addr_t ipaddr, netmask, gw;
    IP4_ADDR(&ipaddr, 10, 30, 113, 199);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gw, 10, 30, 113, 1);

    if (netif_add(&net_if, &ipaddr, &netmask, &gw, static_cast<void *>(&eth_driver),
                  enc_eth_netif_init, tcpip_input) == nullptr) {
        printf("netif_add failed\n");
        return ERR_ABRT;
    }

    printf("netif_add ADDED\n");

    net_if.name[0] = 'e';
    net_if.name[1] = '0';

    netif_set_status_callback(&net_if, netif_status_callback);
    netif_set_link_callback(&net_if, netif_link_callback);
    netif_set_hostname(&net_if, "PICO");

    netif_set_default(&net_if);
    netif_set_up(&net_if);
    // dhcp_start(&net_if);

    // printf("netif DHCP STARTED\n");

    SemaphoreHandle_t init_sem = xSemaphoreCreateBinary();
    tcpip_init(tcpip_init_done, init_sem);
    xSemaphoreTake(init_sem, portMAX_DELAY);

    worker_sem = xSemaphoreCreateBinary();
    TaskHandle_t enc_worker_task_handle{};

    if (xTaskCreate(enc_worker_thread, "enc_worker", 2048, nullptr, tskIDLE_PRIORITY + 4,
                    &enc_worker_task_handle) != pdPASS) {
        return ERR_ABRT;
    }

    // drivers::enc28j60::enc28j60 *eth_driver =
    //         static_cast<drivers::enc28j60::enc28j60 *>(net_if.state);

    irq_loop_sem = xSemaphoreCreateBinary();

    TaskHandle_t irq_loop_task_handle{};
    if (xTaskCreate([](void *me) { static_cast<drivers::enc28j60::enc28j60 *>(me)->irq_loop(); },
                    "irq_loop", 2048, (void *)&eth_driver, configMAX_PRIORITIES - 1,
                    &irq_loop_task_handle) != pdPASS) {
        return ERR_ABRT;
    }
    
#if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
    vTaskCoreAffinitySet(enc_worker_task_handle, NETWORKING_CORE_ID);
    vTaskCoreAffinitySet(irq_loop_task_handle, NETWORKING_CORE_ID);
#endif

    xSemaphoreGive(worker_sem);
    eth_driver.enable_interupts();

    return ERR_OK;
}

err_t enc_driver_lwip_init() { return ERR_OK; }
