#include "FreeRTOS.h"
#include "task.h"

#include "enc28j60.h"
#include "enc28j60_FreeRtos_lwip.h"
#include "gpio_wrapper.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "spi.h"
#include "utils.h"

#include "lwip/apps/httpd.h"

#include "tcp_server_command.h"

tcp_server_command cmd_server(456);

void main_task(void *params) {
 
    enc28j60_driver_os_init();

    cmd_server.start();
    httpd_init();

    while (true) {
        vTaskDelay(100);
    }
}

int main() {
    stdio_init_all();

    TaskHandle_t task_handle{};
    xTaskCreate(main_task, "TestMainThread", configMINIMAL_STACK_SIZE * 4, nullptr,
                tskIDLE_PRIORITY + 1, &task_handle);

#if NO_SYS && configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
    // If NO_SYS is set, then we must bind the main task to one core (at least while the init is
    // called)
    vTaskCoreAffinitySet(task_handle, 1);
#endif
    vTaskStartScheduler();

    return 0;
}

