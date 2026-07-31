# Copyright (c) 2025 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_WIFI_ERPC_WIFI)
  zephyr_library_sources(
    erpc_wifi.c
	erpc_wifi_socket_offload.c
    service/wifi/c_erpcdemo_wifi_host_to_ra_client.cpp
    service/wifi/erpcdemo_wifi_host_to_ra_client.cpp
    service/wifi/erpcdemo_wifi_host_to_ra_interface.cpp

    service/wifi/c_erpcdemo_wifi_ra_to_host_server.cpp
    service/wifi/erpcdemo_wifi_ra_to_host_server.cpp
    service/wifi/erpcdemo_wifi_ra_to_host_interface.cpp

    service/ble/c_erpcdemo_ble_host_to_ra_client.cpp
    service/ble/erpcdemo_ble_host_to_ra_client.cpp
    service/ble/erpcdemo_ble_host_to_ra_interface.cpp

    service/ble/c_erpcdemo_ble_ra_to_host_server.cpp
    service/ble/erpcdemo_ble_ra_to_host_server.cpp
    service/ble/erpcdemo_ble_ra_to_host_interface.cpp
  )

  zephyr_library_sources_ifdef(CONFIG_ERPC_TRANSPORT_UART erpc_wifi_transport_uart.c)
  zephyr_library_sources_ifdef(CONFIG_ERPC_TRANSPORT_SPI erpc_wifi_transport_spi.c)

  zephyr_include_directories(
	.
	service/wifi
	service/ble
	${ZEPHYR_BASE}/subsys/net/lib/sockets
  )
endif()
