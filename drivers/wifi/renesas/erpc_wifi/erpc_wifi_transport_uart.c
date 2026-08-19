/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_erpc_wifi_uart

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wifi_erpc_wifi_transport, CONFIG_WIFI_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <erpc_transport_setup.h>

#include "erpc_wifi_transport.h"

static erpc_transport_t g_active_transport;

static K_MUTEX_DEFINE(g_transport_io_mutex);

void erpc_wifi_transport_lock(void)
{
	k_mutex_lock(&g_transport_io_mutex, K_FOREVER);
}

void erpc_wifi_transport_unlock(void)
{
	k_mutex_unlock(&g_transport_io_mutex);
}

erpc_transport_t erpc_wifi_transport_init(void)
{
    const struct device *dev = DEVICE_DT_GET(DT_INST_BUS(0));

	if (!device_is_ready(dev)) {
		LOG_ERR("Bus device is not ready");
		return NULL;
	}

    g_active_transport = erpc_transport_zephyr_uart_init((void *)dev);
    return g_active_transport;
}

erpc_transport_t erpc_wifi_transport_get(void)
{
	return g_active_transport;
}

void erpc_wifi_transport_deinit(erpc_transport_t transport)
{
    erpc_transport_zephyr_uart_deinit(transport);
	if (g_active_transport == transport) {
		g_active_transport = NULL;
	}
}
