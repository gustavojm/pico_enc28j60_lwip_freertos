#include "FreeRTOS.h"
#include "task.h"

#include "enc28j60.h"
#include "enc28j60_LWIP_FreeRTOS.h"
#include "gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "spi.h"
#include "tcpecho.h"
#include "utils.h"

#include "lwip/apps/httpd.h"

void main_task(void *params) {
    vTaskDelay(1000); // Wait to have usb ttyACM ready
    enc28j60_driver_os_init();

    tcpecho_init();

    vTaskDelete(NULL);
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
