/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host-side dedicated data path bypassing the eRPC codec for the hot
 * send/recv socket plane. The wire frame (data_path_msg_t) is carried as the
 * body of the standard eRPC FramedTransport message over SPI.
 *
 * The wire layout MUST stay byte-compatible with the server-side
 * ra6fw_data_path.h. DP requests are queued to a dedicated host worker that
 * owns each synchronous transport request-response exchange.
 *
 * Build gate: CONFIG_ERPC_DATA_PATH (default n).
 */

#ifndef ZEPHYR_DRIVERS_WIFI_RENESAS_ERPC_WIFI_DATA_PATH_H_
#define ZEPHYR_DRIVERS_WIFI_RENESAS_ERPC_WIFI_DATA_PATH_H_

#include <stdint.h>
#include <stddef.h>

#include "erpc_wifi_cmd.h"

/* Wire-protocol signature — must match server ra6fw_data_path.h. */
#define DATA_PATH_MSG_SIGNATURE (0xDA1AU)

/*
 * Packed wire frame. Only the send/recv/sendto/recvfrom union members are
 * declared here (the current host data-path scope); the select/ioctl members
 * of the server-side struct are omitted because all union members share the
 * same base offset and every frame length is computed from the specific
 * member used, so omission does not change any offset or wire layout.
 */
typedef struct __attribute__((packed)) data_path_msg {
	struct {
		uint16_t id;
		uint16_t seq_id;
		uint16_t worker_id;
		uint16_t api_id;
	} ctrl;

	union {
		struct {
			int32_t s;
			uint32_t size;
			int32_t flags;
			uint8_t data[];
		} lwip_send;

		struct {
			int32_t s;
			uint32_t size;
			int32_t flags;
		} lwip_recv;

		struct {
			int32_t s;
			uint32_t size;
			uint32_t count;
			int32_t flags;
			uint32_t tolen;
			uint8_t to[16];
			uint8_t data[];
		} lwip_sendto;

		struct {
			int32_t s;
			uint32_t size;
			int32_t flags;
		} lwip_recvfrom;

		/* return values */
		struct {
			int32_t ret;
			int32_t _errno;
		} lwip_send_ret;

		struct {
			int32_t ret;
			int32_t _errno;
			uint8_t data[];
		} lwip_recv_ret;

		struct {
			int32_t ret;
			int32_t _errno;
		} lwip_sendto_ret;

		struct {
			int32_t ret;
			int32_t _errno;
			uint32_t fromlen;
			uint8_t from[16];
			uint8_t data[];
		} lwip_recvfrom_ret;
	} u;
} data_path_msg_t;

typedef enum {
	/* APIs */
	DATA_PATH_API_ID_LWIP_SEND = 0,
	DATA_PATH_API_ID_LWIP_RECV,
	DATA_PATH_API_ID_LWIP_SENDTO,
	DATA_PATH_API_ID_LWIP_RECVFROM,
	DATA_PATH_API_ID_LWIP_SELECT,
	DATA_PATH_API_ID_LWIP_IOCTL,
	DATA_PATH_API_ID_SENDTO_REPEAT,
	DATA_PATH_API_ID_RECVFROM_REPEAT,
	DATA_PATH_API_ID_SENDTO_BATCH,
	DATA_PATH_API_ID_SENDTO_NORESP,

	/* return codes */
	DATA_PATH_API_RET_ID_LWIP_SEND = 100,
	DATA_PATH_API_RET_ID_LWIP_RECV,
	DATA_PATH_API_RET_ID_LWIP_SENDTO,
	DATA_PATH_API_RET_ID_LWIP_RECVFROM,
	DATA_PATH_API_RET_ID_LWIP_SELECT,
	DATA_PATH_API_RET_ID_LWIP_IOCTL,
	DATA_PATH_API_RET_ID_SENDTO_REPEAT,
	DATA_PATH_API_RET_ID_RECVFROM_REPEAT,
	DATA_PATH_API_RET_ID_SENDTO_BATCH,
	DATA_PATH_API_RET_ID_SENDTO_NORESP,
} data_path_id_t;

/*
 * Command-queue handlers (run in the erpc_wifi_cmd.c handler thread).
 * Each takes the same parameter struct the eRPC path uses and returns the
 * socket result using the -errno convention (>=0 on success).
 */
int erpc_wifi_dp_send_process(void *data);
int erpc_wifi_dp_recv_process(void *data);
int erpc_wifi_dp_sendto_process(void *data);
int erpc_wifi_dp_recvfrom_process(void *data);

/*
 * Queue-based host data-path APIs. Requests are executed by a dedicated
 * worker that owns transport send/receive sequencing for DP traffic.
 */
int erpc_wifi_dp_send(int fd, const uint8_t *buff, size_t buff_size, uint32_t flags);
int erpc_wifi_dp_recv(int fd, uint8_t *buff, size_t buff_size, uint32_t flags);
int erpc_wifi_dp_sendto(int fd, const uint8_t *buff, size_t buff_size, uint32_t flags,
			 const struct ra_erpc_sockaddr *addr);
int erpc_wifi_dp_recvfrom(int fd, uint8_t *buff, size_t buff_size, uint32_t flags,
		       ra_erpc_sockaddr *src_addr, uint32_t *src_addrlen);

/* Register the data-path command handlers. Called from cmd handler init. */
int erpc_wifi_dp_handlers_init(void);

#endif /* ZEPHYR_DRIVERS_WIFI_RENESAS_ERPC_WIFI_DATA_PATH_H_ */
