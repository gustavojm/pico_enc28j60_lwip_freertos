#include "enc28j60_FreeRtos_lwip.h"
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

#include "gpio.h"
#include "spi.h"
#include "utils.h"

#ifdef ENC_DEBUG_ON
#include <pico/stdio.h>
#endif

constexpr uint32_t NETWORKING_CORE_ID = 1 << 1; // pin all networking functions to core1

static void tcpip_init_done(void *arg) { xSemaphoreGive(static_cast<SemaphoreHandle_t>(arg)); }

static void netif_status_callback(struct netif *netif) {
    printf("NETIF status changed %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
}

static void netif_link_callback(struct netif *netif) {
    drivers::enc28j60 *me = static_cast<drivers::enc28j60 *>(netif->state);
    printf("NETIF link changed: ");
    if (me->is_link_up()) {
        printf("LINK IS UP!\n");
    } else {
        printf("LINK IS DOWN!\n");
    }
}

err_t enc28j60_driver_os_init() {

    constexpr uint8_t MISO_PIN = 16;
    constexpr uint8_t MOSI_PIN = 19;
    constexpr uint8_t CLK_PIN = 18;
    constexpr uint8_t ENC_IRQ = 22;

    static drivers::Gpio EncRstPin{21, GPIO_OUT};
    static drivers::Gpio EncCsPin{17, GPIO_OUT};
    static drivers::Spi::Config spi0Config{spi0, CLK_PIN, MOSI_PIN, MISO_PIN, 25 * 1000000};

    static drivers::Spi spi0_{spi0Config};
    spi0_.init();

    static drivers::enc28j60::Config EncConfig{EncCsPin, EncRstPin, ENC_IRQ, spi0_};
    static drivers::enc28j60 eth_driver{EncConfig};
    constexpr drivers::enc28j60::MacAddress mac{0x0a, 0xbd, 0x7d, 0x95, 0xd3, 0xa5};

    if (!eth_driver.init(mac)) {
        hal::panic();
    }

    ip4_addr_t ipaddr, netmask, gw;
    IP4_ADDR(&ipaddr, 10, 30, 113, 199);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gw, 10, 30, 113, 1);

    if (netif_add(&net_if, &ipaddr, &netmask, &gw, static_cast<void *>(&eth_driver),
                  drivers::enc28j60::eth_netif_init, tcpip_input) == nullptr) {
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

    irq_loop_sem = xSemaphoreCreateBinary();

    TaskHandle_t irq_loop_task_handle{};
    if (xTaskCreate([](void *me) { static_cast<drivers::enc28j60 *>(me)->irq_deferred_handler(); },
                    "irq_loop", 2048, (void *)&eth_driver, configMAX_PRIORITIES - 1,
                    &irq_loop_task_handle) != pdPASS) {
        return ERR_ABRT;
    }

#if configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
    vTaskCoreAffinitySet(irq_loop_task_handle, NETWORKING_CORE_ID);
#endif

    eth_driver.enable_interupts();

    return ERR_OK;
}
