#include "FreeRTOS.h"
#include "task.h"

#include "enc28j60.h"
#include "enc_os_lwip_glue.h"
#include "gpio_wrapper.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "spi.h"
#include "utils.h"

#include "lwip/apps/httpd.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "tcp_server_command.h"

#define DEBUG_printf printf

#if !NO_SYS
void hal::sleep_milli(const uint32_t time_ms) { vTaskDelay(pdMS_TO_TICKS(time_ms)); }
#endif

constexpr uint8_t MISO_PIN = 16;
constexpr uint8_t MOSI_PIN = 19;
constexpr uint8_t CLK_PIN = 18;
// constexpr uint8_t ENC_IRQ = 22;

drivers::gpio::Gpio EncRstPin{21, GPIO_OUT};
drivers::gpio::Gpio EncCsPin{17, GPIO_OUT};
drivers::spi::Config spi0Config{spi0, CLK_PIN, MOSI_PIN, MISO_PIN, 25 * 1000000};
drivers::spi::SpiWrapper spi0_{spi0Config};

drivers::enc28j60::Config EncConfig{EncCsPin, EncRstPin, spi0_};
drivers::enc28j60::enc28j60 eth_driver{EncConfig};

// drivers::gpio::Gpio BoardLed{LED_PIN, GPIO_OUT};

// #define MQTT_SERVER_HOST "telemetria"
#define MQTT_SERVER_PORT 1883

#define PUBLISH_TOPIC "/pruebas/texto"
#define WILL_TOPIC "/pruebas/uvyt"       /*uvyt: ultima voluntad y testamentp*/
#define WILL_MSG "sin conexion a broker" /*uvyt: ultima voluntad y testamentp*/

#define MS_PUBLISH_PERIOD 5000
#define MQTT_TLS 0 // needs to be 1 for AWS IoT

tcp_server_command cmd_server(456);
// tcp_server_command otro_server(678);

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status != 0) {
        DEBUG_printf("Error during connection: err %d.\n", status);
    } else {
        DEBUG_printf("MQTT connected.\n");
    }
}

/*
 * Called when publish is complete either with success or failure
 */
void mqtt_pub_request_cb(void *arg, err_t err) {
    mqtt_client_t *client = static_cast<mqtt_client_t *>(arg);
    // printf("request publish ERROR %d \n", err);
}

err_t mqtt_app_publish(mqtt_client_t *client, char *payload) {
    uint8_t qos = 0; /* 0 1 or 2, see MQTT specification */
    uint8_t retain = 0;
    err_t err = mqtt_publish(client, PUBLISH_TOPIC, payload, strlen(payload), qos, retain,
                       mqtt_pub_request_cb, client);

    if (err != ERR_OK) {
        DEBUG_printf("**** Publish fail***** %d\n", err);
    }
    return err;
}

/* Initiate client and connect to server, if this fails immediately an error code is returned
 * otherwise mqtt_connection_cb will be called with connection result after attempting to
 * to establish a connection with the server. For now MQTT version 3.1.1 is always used
 */
err_t mqtt_app_connect(mqtt_client_t *client, ip_addr_t broker_addr) {
    mqtt_connect_client_info_t ci{};    
    ci.client_id = "PicoW";
    ci.client_user = NULL;
    ci.client_pass = NULL;
    ci.keep_alive = 0;
    ci.will_topic = WILL_TOPIC;
    ci.will_msg = WILL_MSG;
    ci.will_retain = 0;
    ci.will_qos = 2;

    err_t err = mqtt_client_connect(client, &broker_addr, MQTT_SERVER_PORT,
                              mqtt_connection_cb, client, &ci);

    if (err != ERR_OK) {
        DEBUG_printf("ERROR en mqtt_app_connect() %d \n", err);
    }

    return err;
}

void perform_payload(char *p) {     
    time_t s = time(NULL);
    struct tm * current_time = localtime(&s);

    sprintf(p, "%02d:%02d:%02d", current_time->tm_hour, current_time->tm_min, current_time->tm_sec);
}

void mqtt_publish_task(void *pvParameters) {    
    DEBUG_printf("Inicio publish_task\n");
    ip_addr_t broker_addr;
    IP4_ADDR(&broker_addr, 192, 168, 2, 2);    

    int counter = 0;
    mqtt_client_t mqtt_client{};
    // run_dns_lookup(mqtt_client);

    vTaskDelay(2000);

    err_t error = mqtt_app_connect(&mqtt_client, broker_addr);
    if (error == ERR_OK) {
        DEBUG_printf("mqtt_app_connect() OK\n");
    } else {
        DEBUG_printf("mqtt_app_connect() ERROR=%d\n", error);
    }

    while (true) {
        if (mqtt_client_is_connected(&mqtt_client)) {
            char payload[16];
            perform_payload(payload);
            error = mqtt_app_publish(&mqtt_client, payload);

            if (error == ERR_OK) {
                DEBUG_printf("%s publicacion: %d\n", payload, counter);
                counter++;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MS_PUBLISH_PERIOD));
    }
}

void main_task(void *params) {
    if (!eth_driver.init(mac)) {
        hal::panic();
    }

    while (!eth_driver.is_link_up())
        ;

    enc_driver_os_init();
    enc_driver_lwip_init(eth_driver);

    //cmd_server.start();
    //httpd_init();

    xTaskCreate(mqtt_publish_task,      // Task to be run
                "MQTTPublish",          // Name of the Task for debugging and managing its Task Handleos
                2048,                   // Stack depth to be allocated for use with task's stack (see docs)
                nullptr,                // Arguments needed by the Task (NULL because we don't have any)
                (configMAX_PRIORITIES - 2),        // Task Priority - Higher the number the more priority [max is
                                        // (configMAX_PRIORITIES - 1) provided in FreeRTOSConfig.h]
                NULL                    // Task Handle if available for managing the task
    );

    vTaskDelete(NULL);
}

int main() {
    stdio_init_all();

    EncRstPin.init();
    EncCsPin.init();
    spi0_.init();

    TaskHandle_t task{};
    xTaskCreate(main_task, "MainThread", configMINIMAL_STACK_SIZE, nullptr,
                tskIDLE_PRIORITY + 1, &task);

#if NO_SYS && configUSE_CORE_AFFINITY && configNUM_CORES > 1
    // If NO_SYS is set, then we must bind the main task to one core (at least while the init is
    // called)
    vTaskCoreAffinitySet(task, 1);
#endif
    vTaskStartScheduler();

    return 0;
}
