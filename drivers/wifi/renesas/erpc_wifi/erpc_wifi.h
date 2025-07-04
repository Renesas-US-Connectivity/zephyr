/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_WIFI_RENESAS_ERPC_WIFI_H_
#define ZEPHYR_INCLUDE_DRIVERS_WIFI_RENESAS_ERPC_WIFI_H_

#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/drivers/spi.h>
#include <wifi_host_to_ra_common.h>
#include <erpc_server_setup.h>

#ifdef __cplusplus
extern "C" {
#endif

struct erpc_wifi_data {
	struct net_if *net_iface;
	enum wifi_iface_state state;
	scan_result_cb_t scan_cb;
	uint16_t scan_max_bss_cnt;
	struct WIFINetworkParams_t drv_nwk_params;
	erpc_server_t erpc_server;

	struct k_work_q workq;
	struct k_work scan_work;
	struct k_work connect_work;
	struct k_work disconnect_work;

	// TODO - get this to work instead of using bool...?
	//struct k_sem sem_if_ready;	
	volatile bool if_ready;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_WIFI_RENESAS_ERPC_WIFI_H_ */
