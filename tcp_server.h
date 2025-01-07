#pragma once

#include <cstdint>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "lwip/apps/lwiperf.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"


class tcp_server {
  public:
    explicit tcp_server(const char *name, int port, int thread_count);

    void start();

    virtual ~tcp_server() {
    } // Virtual destructor

    void send_message(int socket, char const *msg);

    void handle_connection(int conn_sock);

    virtual bool reply_fn(int sock) = 0;

    xSemaphoreHandle connectionSemaphore;

    void task();

    char task_name[configMAX_TASK_NAME_LEN];
    int port;
    int server_sock;
    int simultaneous_connections;
};
