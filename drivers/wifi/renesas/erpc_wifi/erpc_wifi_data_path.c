/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host-side data-path handlers and worker queue implementation.
 * Socket hot paths enqueue DP requests to a dedicated worker thread. The
 * worker performs synchronous transport request-response transactions
 * (build frame -> send -> receive -> parse), bypassing the eRPC codec for
 * the hot send/recv socket plane.
 *
 * Build gate: CONFIG_ERPC_DATA_PATH.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/sys/atomic.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "erpc_wifi_data_path.h"
#include "erpc_wifi_transport.h"

LOG_MODULE_REGISTER(erpc_wifi_data_path, CONFIG_WIFI_LOG_LEVEL);

#define DP_CTRL_SIZE (sizeof(((data_path_msg_t *)0)->ctrl))
#define DP_REQ_TIMEOUT K_SECONDS(30)
#define DP_QUEUE_DEPTH 16

enum dp_req_type {
	DP_REQ_SEND,
	DP_REQ_RECV,
	DP_REQ_SENDTO,
	DP_REQ_RECVFROM,
};

struct dp_req {
	enum dp_req_type type;
	union {
		erpc_wifi_msg_send_t send;
		erpc_wifi_msg_recv_t recv;
		erpc_wifi_msg_sendto_t sendto;
		erpc_wifi_msg_recvfrom_t recvfrom;
	} u;
	struct k_sem done;
	int ret;
};

K_MSGQ_DEFINE(g_dp_req_msgq, sizeof(struct dp_req *), DP_QUEUE_DEPTH, 4);
K_THREAD_STACK_DEFINE(g_dp_worker_stack, 3072);
static struct k_thread g_dp_worker_thread;
static k_tid_t g_dp_worker_tid;
static atomic_t g_dp_worker_started;
static atomic_t g_dp_trace_seq;

static int dp_send_impl(erpc_wifi_msg_send_t *msg);
static int dp_recv_impl(erpc_wifi_msg_recv_t *msg);
static int dp_sendto_impl(erpc_wifi_msg_sendto_t *msg);
static int dp_recvfrom_impl(erpc_wifi_msg_recvfrom_t *msg);

static void erpc_wifi_dp_worker_main(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		struct dp_req *req = NULL;
		int64_t op_start;

		if (k_msgq_get(&g_dp_req_msgq, &req, K_FOREVER) != 0 || req == NULL) {
			continue;
		}

		op_start = k_uptime_get();
		erpc_wifi_transport_lock();
		switch (req->type) {
		case DP_REQ_SEND:
			req->ret = dp_send_impl(&req->u.send);
			break;
		case DP_REQ_RECV:
			req->ret = dp_recv_impl(&req->u.recv);
			break;
		case DP_REQ_SENDTO:
			req->ret = dp_sendto_impl(&req->u.sendto);
			break;
		case DP_REQ_RECVFROM:
			req->ret = dp_recvfrom_impl(&req->u.recvfrom);
			break;
		default:
			req->ret = -EINVAL;
			break;
		}
		erpc_wifi_transport_unlock();

		k_sem_give(&req->done);
	}
}

static int dp_ensure_worker_started(void)
{
	if (atomic_get(&g_dp_worker_started) != 0) {
		return 0;
	}

	if (!atomic_cas(&g_dp_worker_started, 0, 1)) {
		return 0;
	}

	g_dp_worker_tid = k_thread_create(&g_dp_worker_thread,
					 g_dp_worker_stack,
					 K_THREAD_STACK_SIZEOF(g_dp_worker_stack),
					 erpc_wifi_dp_worker_main,
					 NULL, NULL, NULL,
					 K_PRIO_PREEMPT(9), 0, K_NO_WAIT);
	if (!g_dp_worker_tid) {
		atomic_set(&g_dp_worker_started, 0);
		return -ENOMEM;
	}

	k_thread_name_set(g_dp_worker_tid, "erpc_wifi_dp_worker");
	return 0;
}

static int dp_submit_req(struct dp_req *req)
{
	int rc = dp_ensure_worker_started();
	if (rc != 0) {
		return rc;
	}

	k_sem_init(&req->done, 0, 1);
	req->ret = -EIO;

	if (k_msgq_put(&g_dp_req_msgq, &req, K_MSEC(1000)) != 0) {
		LOG_ERR("dp: worker queue full");
		return -EAGAIN;
	}

	if (k_sem_take(&req->done, DP_REQ_TIMEOUT) != 0) {
		LOG_ERR("dp: worker request timeout");
		return -ETIMEDOUT;
	}

	return req->ret;
}

static uint16_t dp_next_seq_id(void)
{
	static uint16_t seq;

	return seq++;
}

static void dp_fill_ctrl(data_path_msg_t *m, uint16_t api_id)
{
	m->ctrl.id = DATA_PATH_MSG_SIGNATURE;
	m->ctrl.seq_id = dp_next_seq_id();
	m->ctrl.worker_id = 0U;
	m->ctrl.api_id = api_id;
}

/*
 * Send a request frame and receive the response into resp. Validates the
 * response signature and expected return API id. Returns 0 on success with
 * *resp_len set to the response body length, or a negative errno.
 */
static int dp_do_xact(const uint8_t *req, uint16_t req_len,
		      uint8_t *resp, uint16_t resp_cap, uint16_t *resp_len,
		      uint16_t expect_api)
{
	erpc_transport_t transport = erpc_wifi_transport_get();
	int rc = erpc_transport_zephyr_spi_master_dp_send(transport, req, req_len);
	if (rc != 0) {
		return rc;
	}

	uint16_t rlen = 0U;
	rc = erpc_transport_zephyr_spi_master_dp_recv(transport, resp, resp_cap, &rlen);
	if (rc != 0) {
		return rc;
	}

	data_path_msg_t *r = (data_path_msg_t *)resp;
	if (rlen < DP_CTRL_SIZE || r->ctrl.id != DATA_PATH_MSG_SIGNATURE) {
		LOG_ERR("dp: bad response (len=%u id=0x%04x)", rlen, r->ctrl.id);
		return -EPROTO;
	}
	if (r->ctrl.api_id != expect_api) {
		LOG_ERR("dp: unexpected api_id %u (want %u)", r->ctrl.api_id, expect_api);
		return -EPROTO;
	}

	if (resp_len) {
		*resp_len = rlen;
	}
	return 0;
}

static int dp_send_impl(erpc_wifi_msg_send_t *msg)
{
	if (!msg) {
		return -EINVAL;
	}

	const uint16_t req_len = (uint16_t)(DP_CTRL_SIZE +
			sizeof(((data_path_msg_t *)0)->u.lwip_send) + msg->buff_size);

	data_path_msg_t *m = (data_path_msg_t *)k_malloc(req_len);
	if (!m) {
		return -ENOMEM;
	}

	dp_fill_ctrl(m, DATA_PATH_API_ID_LWIP_SEND);
	m->u.lwip_send.s = msg->fd;
	m->u.lwip_send.size = msg->buff_size;
	m->u.lwip_send.flags = (int32_t)msg->flags;
	if (msg->buff_size && msg->buff) {
		memcpy(m->u.lwip_send.data, msg->buff, msg->buff_size);
	}

	uint8_t resp[DP_CTRL_SIZE + sizeof(((data_path_msg_t *)0)->u.lwip_send_ret)];
	int rc = dp_do_xact((const uint8_t *)m, req_len, resp, sizeof(resp), NULL,
			    DATA_PATH_API_RET_ID_LWIP_SEND);
	k_free(m);
	if (rc != 0) {
		return rc;
	}

	return ((data_path_msg_t *)resp)->u.lwip_send_ret.ret;
}

static int dp_sendto_impl(erpc_wifi_msg_sendto_t *msg)
{
	if (!msg) {
		return -EINVAL;
	}

	const uint16_t req_len = (uint16_t)(DP_CTRL_SIZE +
			sizeof(((data_path_msg_t *)0)->u.lwip_sendto) + msg->buff_size);

	data_path_msg_t *m = (data_path_msg_t *)k_malloc(req_len);
	if (!m) {
		return -ENOMEM;
	}

	uint32_t tolen = 0U;
	if (msg->addr_erpc_wifi) {
		tolen = msg->addr_erpc_wifi->sa_len;
		if (tolen > sizeof(m->u.lwip_sendto.to)) {
			tolen = sizeof(m->u.lwip_sendto.to);
		}
	}

	dp_fill_ctrl(m, DATA_PATH_API_ID_LWIP_SENDTO);
	m->u.lwip_sendto.s = msg->fd;
	m->u.lwip_sendto.size = msg->buff_size;
	m->u.lwip_sendto.count = 0U;
	m->u.lwip_sendto.flags = (int32_t)msg->flags;
	m->u.lwip_sendto.tolen = tolen;
	memset(m->u.lwip_sendto.to, 0, sizeof(m->u.lwip_sendto.to));
	if (tolen) {
		memcpy(m->u.lwip_sendto.to, msg->addr_erpc_wifi, tolen);
	}
	if (msg->buff_size && msg->buff) {
		memcpy(m->u.lwip_sendto.data, msg->buff, msg->buff_size);
	}

	uint8_t resp[DP_CTRL_SIZE + sizeof(((data_path_msg_t *)0)->u.lwip_sendto_ret)];
	int rc = dp_do_xact((const uint8_t *)m, req_len, resp, sizeof(resp), NULL,
			    DATA_PATH_API_RET_ID_LWIP_SENDTO);
	k_free(m);
	if (rc != 0) {
		return rc;
	}

	return ((data_path_msg_t *)resp)->u.lwip_sendto_ret.ret;
}

static int dp_recv_impl(erpc_wifi_msg_recv_t *msg)
{
	if (!msg) {
		return -EINVAL;
	}

	const uint16_t req_len = (uint16_t)(DP_CTRL_SIZE +
			sizeof(((data_path_msg_t *)0)->u.lwip_recv));
	const uint16_t resp_cap = (uint16_t)(DP_CTRL_SIZE +
			sizeof(((data_path_msg_t *)0)->u.lwip_recv_ret) + msg->buff_size);

	data_path_msg_t *m = (data_path_msg_t *)k_malloc(req_len);
	if (!m) {
		return -ENOMEM;
	}

	uint8_t *resp = (uint8_t *)k_malloc(resp_cap);
	if (!resp) {
		k_free(m);
		return -ENOMEM;
	}

	dp_fill_ctrl(m, DATA_PATH_API_ID_LWIP_RECV);
	m->u.lwip_recv.s = msg->fd;
	m->u.lwip_recv.size = msg->buff_size;
	m->u.lwip_recv.flags = (int32_t)msg->flags;

	uint16_t rlen = 0U;
	int rc = dp_do_xact((const uint8_t *)m, req_len, resp, resp_cap, &rlen,
			    DATA_PATH_API_RET_ID_LWIP_RECV);
	k_free(m);
	if (rc != 0) {
		k_free(resp);
		return rc;
	}

	data_path_msg_t *r = (data_path_msg_t *)resp;
	int ret = r->u.lwip_recv_ret.ret;
	if (ret > 0 && msg->buff) {
		size_t copy = ((size_t)ret <= msg->buff_size) ? (size_t)ret : msg->buff_size;
		/* Guard against a short/truncated response frame. */
		size_t avail = (rlen > (DP_CTRL_SIZE +
			sizeof(r->u.lwip_recv_ret))) ?
			(rlen - DP_CTRL_SIZE - sizeof(r->u.lwip_recv_ret)) : 0U;
		if (copy > avail) {
			copy = avail;
		}
		memcpy(msg->buff, r->u.lwip_recv_ret.data, copy);
		ret = (int)copy;
	}

	k_free(resp);
	return ret;
}

static int dp_recvfrom_impl(erpc_wifi_msg_recvfrom_t *msg)
{
	if (!msg) {
		return -EINVAL;
	}

	const uint16_t req_len = (uint16_t)(DP_CTRL_SIZE +
			sizeof(((data_path_msg_t *)0)->u.lwip_recvfrom));
	const uint16_t resp_cap = (uint16_t)(DP_CTRL_SIZE +
			sizeof(((data_path_msg_t *)0)->u.lwip_recvfrom_ret) + msg->buff_size);

	data_path_msg_t *m = (data_path_msg_t *)k_malloc(req_len);
	if (!m) {
		return -ENOMEM;
	}

	uint8_t *resp = (uint8_t *)k_malloc(resp_cap);
	if (!resp) {
		k_free(m);
		return -ENOMEM;
	}

	dp_fill_ctrl(m, DATA_PATH_API_ID_LWIP_RECVFROM);
	m->u.lwip_recvfrom.s = msg->fd;
	m->u.lwip_recvfrom.size = msg->buff_size;
	m->u.lwip_recvfrom.flags = (int32_t)msg->flags;

	uint16_t rlen = 0U;
	int rc = dp_do_xact((const uint8_t *)m, req_len, resp, resp_cap, &rlen,
			    DATA_PATH_API_RET_ID_LWIP_RECVFROM);
	k_free(m);
	if (rc != 0) {
		k_free(resp);
		return rc;
	}

	data_path_msg_t *r = (data_path_msg_t *)resp;
	int ret = r->u.lwip_recvfrom_ret.ret;
	if (ret > 0) {
		if (msg->src_addr && msg->src_addrlen) {
			uint32_t fl = r->u.lwip_recvfrom_ret.fromlen;
			if (fl > sizeof(r->u.lwip_recvfrom_ret.from)) {
				fl = sizeof(r->u.lwip_recvfrom_ret.from);
			}
			*msg->src_addrlen = fl;
			memcpy(msg->src_addr, r->u.lwip_recvfrom_ret.from, fl);
		}

		if (msg->buff) {
			size_t copy = ((size_t)ret <= msg->buff_size) ? (size_t)ret : msg->buff_size;
			size_t avail = (rlen > (DP_CTRL_SIZE +
				sizeof(r->u.lwip_recvfrom_ret))) ?
				(rlen - DP_CTRL_SIZE - sizeof(r->u.lwip_recvfrom_ret)) : 0U;
			if (copy > avail) {
				copy = avail;
			}
			memcpy(msg->buff, r->u.lwip_recvfrom_ret.data, copy);
			ret = (int)copy;
		}
	}

	k_free(resp);
	return ret;
}

int erpc_wifi_dp_send_process(void *data)
{
	return dp_send_impl((erpc_wifi_msg_send_t *)data);
}

int erpc_wifi_dp_sendto_process(void *data)
{
	return dp_sendto_impl((erpc_wifi_msg_sendto_t *)data);
}

int erpc_wifi_dp_recv_process(void *data)
{
	return dp_recv_impl((erpc_wifi_msg_recv_t *)data);
}

int erpc_wifi_dp_recvfrom_process(void *data)
{
	return dp_recvfrom_impl((erpc_wifi_msg_recvfrom_t *)data);
}

int erpc_wifi_dp_send(int fd, const uint8_t *buff, size_t buff_size, uint32_t flags)
{
	struct dp_req req = {
		.type = DP_REQ_SEND,
		.u.send = {
			.fd = fd,
			.flags = flags,
			.buff = (uint8_t *)buff,
			.buff_size = buff_size,
		},
	};

	return dp_submit_req(&req);
}

int erpc_wifi_dp_recv(int fd, uint8_t *buff, size_t buff_size, uint32_t flags)
{
	struct dp_req req = {
		.type = DP_REQ_RECV,
		.u.recv = {
			.fd = fd,
			.flags = flags,
			.buff = buff,
			.buff_size = buff_size,
		},
	};

	return dp_submit_req(&req);
}

int erpc_wifi_dp_sendto(int fd, const uint8_t *buff, size_t buff_size, uint32_t flags,
			 const struct ra_erpc_sockaddr *addr)
{
	struct dp_req req = {
		.type = DP_REQ_SENDTO,
		.u.sendto = {
			.fd = fd,
			.flags = flags,
			.buff = (uint8_t *)buff,
			.buff_size = buff_size,
			.addr_erpc_wifi = (struct ra_erpc_sockaddr *)addr,
		},
	};

	return dp_submit_req(&req);
}

int erpc_wifi_dp_recvfrom(int fd, uint8_t *buff, size_t buff_size, uint32_t flags,
		       ra_erpc_sockaddr *src_addr, uint32_t *src_addrlen)
{
	struct dp_req req = {
		.type = DP_REQ_RECVFROM,
		.u.recvfrom = {
			.fd = fd,
			.flags = flags,
			.buff = buff,
			.buff_size = buff_size,
			.src_addr = src_addr,
			.src_addrlen = src_addrlen,
		},
	};

	return dp_submit_req(&req);
}

int erpc_wifi_dp_handlers_init(void)
{
	erpc_wifi_register_cmd_handler(ERPC_WIFI_DP_SEND_CMD, erpc_wifi_dp_send_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_DP_RECV_CMD, erpc_wifi_dp_recv_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_DP_SENDTO_CMD, erpc_wifi_dp_sendto_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_DP_RECVFROM_CMD, erpc_wifi_dp_recvfrom_process);

	LOG_INF("Data path handlers initialized");
	return 0;
}
