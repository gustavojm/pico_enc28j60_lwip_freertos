#include "pico/stdlib.h"

#include "lwip/apps/lwiperf.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "ArduinoJson.h"

#include <lwip/sockets.h>

#include "tcp_server.h"

#define TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)

tcp_server::tcp_server(const char *name, int port, int simultaneous_connections) : port(port), simultaneous_connections(simultaneous_connections) {
    connectionSemaphore = xSemaphoreCreateCounting(simultaneous_connections, simultaneous_connections);    
    memset(task_name, 0, sizeof(task_name));
    strncat(task_name, name, sizeof(task_name) - strlen(task_name) - 1);
    strncat(task_name, "_task", sizeof(task_name) - strlen(task_name) - 1);
}

void tcp_server::start() {
    xTaskCreate([](void *me) { static_cast<tcp_server *>(me)->task(); }, task_name, 1024, this, TASK_PRIORITY, NULL);
    printf("%s: created", task_name);
}

struct TaskParams {
    tcp_server *server;
    int conn_sock;
};

void do_handle_connection(void *pvParams) {
    auto *task_params = static_cast<TaskParams *>(pvParams);
    int conn_sock = task_params->conn_sock;
    while (!task_params->server->reply_fn(conn_sock)) {
    }

    closesocket(conn_sock);
    xSemaphoreGive(task_params->server->connectionSemaphore);
    delete task_params; // Clean up dynamically allocated parameters

    vTaskDelete(NULL);
}


void tcp_server::handle_connection(int conn_sock) {
    TaskHandle_t task;
    xSemaphoreTake(connectionSemaphore, portMAX_DELAY);

    auto *params = new TaskParams{this, conn_sock}; // New was used here... memory leak?

    xTaskCreate(
        do_handle_connection,
        "Connection Thread",
        configMINIMAL_STACK_SIZE,
        (void *)params,
        TASK_PRIORITY,
        &task
    );
}

void tcp_server::send_message(int socket, char const *msg) {
    int len = strlen(msg);
    int done = 0;
    while (done < len) {
        int done_now = send(socket, msg + done, len - done, 0);
        if (done_now <= 0)
            return;

        done += done_now;
    }
}

void tcp_server::task() {
    server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    struct sockaddr_in listen_addr =
        {
            .sin_len = sizeof(struct sockaddr_in),
            .sin_family = AF_INET,
            .sin_port = htons(port),
            .sin_addr = 0,
        };
    
    if (server_sock < 0) {
        printf("Unable to create socket: error %d", errno);
        goto CLEAN_UP;
    }

    if (bind(server_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
        printf("Unable to bind socket: error %d\n", errno);
        goto CLEAN_UP;
    }

    if (listen(server_sock, simultaneous_connections * 2) < 0) {        // Why times 2... Not sure why...
        printf("Unable to listen on socket: error %d\n", errno);
        goto CLEAN_UP;
    }

    printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), ntohs(listen_addr.sin_port));

    while (true) {
        struct sockaddr_storage remote_addr;
        socklen_t len = sizeof(remote_addr);
        int conn_sock = accept(server_sock, (struct sockaddr *)&remote_addr, &len);
        if (conn_sock >= 0)
            handle_connection(conn_sock);
    }

CLEAN_UP:
    close(server_sock);
    vTaskDelete(NULL);
}
