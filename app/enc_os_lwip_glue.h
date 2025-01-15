#pragma once
#include "enc28j60.h"
#include "lwip/err.h"
#include "lwip/netif.h"

err_t enc_driver_lwip_init();
err_t enc28j60_driver_os_init();

inline netif net_if{};
