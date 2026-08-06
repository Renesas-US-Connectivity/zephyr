#include "erpc_wifi_cmd_process_handlers.h"
#include "erpc_wifi_cmd.h"
#include "erpc_wifi.h"
#include "service/c_wifi_host_to_ra_client.h"

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include "zephyr/net/socket.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

LOG_MODULE_REGISTER(erpc_wifi_cmd_socket_process, 4);

#ifndef PMGR_CONSTRAINT_SLEEP_PROHIBITED
#define PMGR_CONSTRAINT_SLEEP_PROHIBITED   (1U << 0)
#endif
#ifndef PMGR_CONSTRAINT_POWER_RAM
#define PMGR_CONSTRAINT_POWER_RAM (1U << 2)
#endif

#define DT_DRV_COMPAT renesas_erpc_wifi_spi

#define GPIO_WAKEUP_NODE_SOCKET DT_ALIAS(wakeup_gpio)
#define GPIO_WAKEUP_PORT_SOCKET DT_GPIO_CTLR(GPIO_WAKEUP_NODE_SOCKET, gpios)
#define GPIO_WAKEUP_PIN_SOCKET DT_GPIO_PIN(GPIO_WAKEUP_NODE_SOCKET, gpios)
#define WAKEUP_PULSE_DURATION_MS 5

static const struct device *g_gpio_wakeup_dev = DEVICE_DT_GET(GPIO_WAKEUP_PORT_SOCKET);
static struct gpio_dt_spec n_int_gpio = GPIO_DT_SPEC_GET(DT_DRV_INST(0), int_gpios);
static struct gpio_callback n_int_cb;

static atomic_t g_erpc_tx_blocked;
static atomic_t g_erpc_tx_block_timeout_ms;
static struct k_sem sem_if_wkup;

static int g_sockets_created;

static int erpc_wifi_socket_evtent_msg_process(int *fd);

static void n_int_iface_active_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	switch (erpc_wifi_get_wkup_source()) {
	case ERPC_WIFI_WKUP_MODULE:
		if (0 == erpc_wifi_send_cmd(ERPC_WIFI_WAIT_RA_AWAKE_CMD, &(int *) { ERPC_PMGR_JOB_ID_RECV } , sizeof(int), 0)) {
			gpio_remove_callback(dev, cb);
//			k_sem_reset(&sem_if_wkup);
			erpc_wifi_set_wkup_source(ERPC_WIFI_WKUP_MODULE_IN_PROGRESS);
		}
		break;
	case ERPC_WIFI_WKUP_HOST:
		gpio_remove_callback(dev, cb);
		erpc_wifi_set_wkup_source(ERPC_WIFI_WKUP_HOST_IN_PROGRESS);
		k_sem_give(&sem_if_wkup);
		break;
	default:
		break;
	}
}

/*
 * Optional wake trigger (Host -> RA6W1). Application can override this symbol
 * to pulse a GPIO connected to RA6W1 wake input.
 */
__weak void erpc_wifi_gpio_trigger_wakeup(void)
{
	LOG_WRN("trigger wakeup");

	gpio_pin_set(g_gpio_wakeup_dev, GPIO_WAKEUP_PIN_SOCKET, 1);
	k_msleep(WAKEUP_PULSE_DURATION_MS);
	gpio_pin_set(g_gpio_wakeup_dev, GPIO_WAKEUP_PIN_SOCKET, 0);
	k_msleep(WAKEUP_PULSE_DURATION_MS);
	gpio_pin_set(g_gpio_wakeup_dev, GPIO_WAKEUP_PIN_SOCKET, 1);
}

void erpc_wifi_socket_tx_block_set(bool enable, uint32_t timeout_ms)
{
	atomic_set(&g_erpc_tx_blocked, enable ? 1 : 0);
	atomic_set(&g_erpc_tx_block_timeout_ms, (atomic_val_t)timeout_ms);

	if (enable) {
		/* Sleep is enabled so update the wake up source to be from the module */
		LOG_ERR("set wakeup from module on sleep enable");
		erpc_wifi_set_wkup_source(ERPC_WIFI_WKUP_MODULE);
	} else {
		erpc_wifi_set_wkup_source(ERPC_WIFI_WKUP_UNKNOWN);
	}
}

static int post_awake_procedure(uint32_t job_id)
{
	int ret = 0;

//	ra6w1_pmgr_add_sleep_constraint(PMGR_CONSTRAINT_SLEEP_PROHIBITED);
//	ra6w1_pmgr_add_sleep_constraint(PMGR_CONSTRAINT_POWER_RAM);

//		k_sleep(K_MSEC(300));

	/* Make sure to have correct socket event handler to check if there
	 * is data to read on other socket operation*/
	erpc_wifi_unregister_cmd_handler(EPRC_WIFI_GET_SOCKET_EVT_CMD);
	erpc_wifi_register_cmd_handler(EPRC_WIFI_GET_SOCKET_EVT_CMD, erpc_wifi_socket_evtent_msg_process);

	/* Check is sleep is enabled and if so re-apply PS settings after some timeout */
	erpc_wifi_send_cmd(ERPC_WIFI_IS_PS_ENABLED, NULL , 0, 0);

//	ret = ra6w1_pmgr_dpm_rcv_ready_set(job_id);
//	if (!ret) {
//
//		ra6w1_pmgr_dpm_wakeup_done(job_id);
//
//		switch (job_id) {
//		case ERPC_PMGR_JOB_ID_SEND:
//			ra6w1_pmgr_dpm_job_name_set("ERPC_TCP_SEND");
//			break;
//		case ERPC_PMGR_JOB_ID_RECV:
//			ra6w1_pmgr_dpm_job_name_set("ERPC_TCP_RECV");
//			break;
//		default:
//			break;
//		}
//
//		ra6w1_pmgr_add_sleep_constraint(PMGR_CONSTRAINT_SLEEP_PROHIBITED);
//		ra6w1_pmgr_add_sleep_constraint(PMGR_CONSTRAINT_POWER_RAM);
//
////		k_sleep(K_MSEC(300));
//
//		/* Make sure to have correct socket event handler to check if there
//		 * is data to read on other socket operation*/
//		erpc_wifi_unregister_cmd_handler(EPRC_WIFI_GET_SOCKET_EVT_CMD);
//		erpc_wifi_register_cmd_handler(EPRC_WIFI_GET_SOCKET_EVT_CMD, erpc_wifi_socket_evtent_msg_process);
//	}

	return ret;
}

static int start_wakeup_procedure(void)
{
	int ret = -1;
	uint32_t trigger_cnt = 0;
	uint32_t trigger_dalay = 100;
	uint32_t total_delay = 0;

	/* Host will trigger the wake up so update the wake up source */
	erpc_wifi_set_wkup_source(ERPC_WIFI_WKUP_HOST);

	k_sem_reset(&sem_if_wkup);
	/* Remove then re-add to prevent double-registration (callback already added at init) */
	gpio_remove_callback(n_int_gpio.port, &n_int_cb);
	gpio_add_callback(n_int_gpio.port, &n_int_cb);

	do {
		erpc_wifi_gpio_trigger_wakeup();
		if (k_sem_take(&sem_if_wkup, K_MSEC(trigger_dalay)) == 0) {
			ret = 0;
			k_sleep(K_MSEC(400 - total_delay));
			break;
		}

		/* If rising edge interrupt didn't fire (e.g. pin already high or floating), check level directly */
		extern int erpc_wifi_transport_slave_ready(void);
		if (erpc_wifi_transport_slave_ready() == 1) {
			ret = 0;
			k_sleep(K_MSEC(300)); /* Wait for RTM restoration and eRPC server startup to finish */
			break;
		}

		total_delay += trigger_dalay;
	} while (++trigger_cnt < 3);

	if (ret < 0) {
		return -EIO;
	}

//	k_sleep(K_MSEC(300));

	/* Right after the wake up there can be CRC errors from the module.
	 * This will try to make sure module is fully awake and functional.
	 * And Yes, We should check if `ra6w1_pmgr_dpm_is_wakeup` returns true */
//	int dpm_is_enabled = ra6w1_pmgr_dpm_is_enabled();
//	int max_check = 3;
//
//	while (dpm_is_enabled != 1 && --max_check != 0) {
//		k_sleep(K_MSEC(100));
//		dpm_is_enabled = ra6w1_pmgr_dpm_is_enabled();
//	}
//
//	ret = (max_check == 0) ? -1 : 0;

	return ret;
}

static int wait_ra_awake(uint32_t job_id, erpc_wifi_cmd_t cmd)
{
	int ret = 0;
	static bool in_progress = false;

	if (in_progress) {
		ret = -EINPROGRESS;
		goto done;
	}

	in_progress = true;

	if (atomic_get(&g_erpc_tx_blocked) == 0) {
		goto done;
	}

	switch (erpc_wifi_get_wkup_source()) {
	case ERPC_WIFI_WKUP_UNKNOWN:
		LOG_DBG("+++ Unknown PS state. Check if PS should be enabled");
		erpc_wifi_send_cmd(ERPC_WIFI_ACTIVITY_CB_CMD, &(bool) { true } , sizeof(bool), 0);

		erpc_wifi_send_cmd(ERPC_WIFI_IS_PS_ENABLED, NULL , 0, 0);
		atomic_set(&g_erpc_tx_blocked, 0);
		goto done;
	/* The module is already active, lets just handle the wakeup done stuff */
	case ERPC_WIFI_WKUP_MODULE_IN_PROGRESS:
		k_sleep(K_MSEC(300));
	case ERPC_WIFI_WKUP_HOST_IN_PROGRESS:
		break;
	default:
		ret = start_wakeup_procedure();
		break;
	}

	if (ret) {
		LOG_ERR("++++ failed to wake up the device: ret: %d, source: %s", ret, erpc_wifi_cmd_to_str(cmd));
		erpc_wifi_send_cmd(ERPC_WIFI_ACTIVITY_CB_CMD, &(bool) { true } , sizeof(bool), 0);
		erpc_wifi_set_wkup_source(ERPC_WIFI_WKUP_MODULE);
		atomic_set(&g_erpc_tx_blocked, 1);
		ret = -EAGAIN;
		goto done;
	}

	/* Now that module is active We'll have a lot of calls from other procedures
	 *  so lets suppress that and enable back if needed when sleep re-enable happens */
	atomic_set(&g_erpc_tx_blocked, 0);

//	k_sleep(K_MSEC(300));

	ret = post_awake_procedure(job_id);
	if (ret) {
		erpc_wifi_set_wkup_source(ERPC_WIFI_WKUP_MODULE);
		atomic_set(&g_erpc_tx_blocked, 1);
		ret = -EAGAIN;
	}

	LOG_WRN("+++ Wake Up cmd ret: %d source: %s", ret, erpc_wifi_cmd_to_str(cmd));

done:
	in_progress = false;

	return ret;
}

static int erpc_wifi_ret_to_errno(WIFIReturnCode_t erpc_err)
{
	int ret = -EFAULT;

	switch (erpc_err) {
	case eWiFiSuccess:
		ret = 0;
		break;
	case eWiFiFailure:
		ret = -EIO;
		break;
	case eWiFiNotSupported:
		ret = -ENODEV;
		break;
	case eWiFiTimeout:
		ret = -ETIMEDOUT;
		break;
	default:
		LOG_WRN("eprc err: %d", erpc_err);
		break;
	}

	return ret;
}

static int erpc_wifi_bind_msg_process(erpc_wifi_msg_bind_t *d)
{
	if (wait_ra_awake(ERPC_PMGR_JOB_ID_SEND, ERPC_WIFI_BIND_CMD) != 0) {
		return -EAGAIN;
	}

	int ret = ra6w1_bind(d->fd, &d->addr_erpc_wifi, d->addr_erpc_wifi.sa_len);

//	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);

	return ret;
}

static int erpc_wifi_connect_msg_process(erpc_wifi_msg_connect_t *d)
{
	if (wait_ra_awake(ERPC_PMGR_JOB_ID_SEND, ERPC_WIFI_CONNECT_CMD) != 0) {
		return -EAGAIN;
	}

	int ret = ra6w1_connect(d->fd, &d->addr_erpc_wifi, d->addr_erpc_wifi.sa_len);

//	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);

	return ret;
}

static int erpc_wifi_listen_msg_process(erpc_wifi_msg_listen_t *d)
{
	if (wait_ra_awake(ERPC_PMGR_JOB_ID_SEND, ERPC_WIFI_LISTEN_CMD) != 0) {
		return -EAGAIN;
	}

	int ret = ra6w1_listen(d->fd, d->backlog);

//	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);

	return ret;
}

static int erpc_wifi_accept_msg_process(erpc_wifi_msg_accept_t *d)
{
	if (wait_ra_awake(ERPC_PMGR_JOB_ID_SEND, ERPC_WIFI_ACCEPT_CMD) != 0) {
		return -EAGAIN;
	}

	int ret = ra6w1_accept(d->fd, &d->addr_erpc_wifi, d->addrlen);

//	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);

	return ret;
}

static int erpc_wifi_check_socket_evt(int fd)
{
	uint32_t ev = get_socket_events(fd);

	if (ev == 0xFFFFFFFFU || ev == (uint32_t)-1) {
		return -EAGAIN;
	}

	if (ev < 0) {
		return -EAGAIN;
	}

	if (ev & SOCKET_EVENT_ERR) {;
//		zsock_close(fd);
		erpc_wifi_send_cmd(ERPC_WIFI_SOCKET_CLOSE_CMD, fd, sizeof(fd), 0);
		LOG_ERR("fd: %d socket err ECONNRESET", fd);
		return -ECONNRESET;
	}

	if (ev & SOCKET_EVENT_CLOSE) {
//		zsock_close(fd);
		erpc_wifi_send_cmd(ERPC_WIFI_SOCKET_CLOSE_CMD, fd, sizeof(fd), 0);
		LOG_ERR("fd: %d socket err ENOTCONN", fd);
		return -ENOTCONN;
	}

	return 0;
}

static int erpc_wifi_recvfrom_msg_process(erpc_wifi_msg_recvfrom_t *d)
{

	int ret = wait_ra_awake(ERPC_PMGR_JOB_ID_RECV, ERPC_WIFI_RECVFROM_CMD);

	if (ret < 0) {
		return ret;
	}

	ret = erpc_wifi_check_socket_evt(d->base.fd);

	if (ret < 0) {
		return ret;
	}

	if (d->src_addr) {
		ret =  ra6w1_recvfrom(d->base.fd, d->base.buff,d->base.buff_size, d->base.flags,
				d->src_addr, d->src_addrlen);
	} else {
		ret =  ra6w1_recv(d->base.fd, d->base.buff, d->base.buff_size, d->base.flags);
	}

	if (d->base.buff_size_proc) {
		*d->base.buff_size_proc -= d->base.buff_size;
	}

//	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_RECV);

	return ret;
}

static int erpc_wifi_sendto_msg_process(erpc_wifi_msg_sendto_t *d)
{
	int ret = wait_ra_awake(ERPC_PMGR_JOB_ID_SEND, ERPC_WIFI_SENDTO_CMD);

	if (ret < 0) {
		return ret;
	}

	ret = erpc_wifi_check_socket_evt(d->base.fd);

	if (ret < 0) {
		return ret;
	}

	if (d->addr_erpc_wifi) {
		ret = ra6w1_sendto(d->base.fd, d->base.buff, d->base.buff_size, d->base.flags,
				d->addr_erpc_wifi, d->addr_erpc_wifi->sa_len);
	} else {
		ret = ra6w1_send(d->base.fd, d->base.buff, d->base.buff_size, d->base.flags);
	}

	if (d->base.buff_size_proc) {
		*d->base.buff_size_proc -= d->base.buff_size;
	}

//	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);

	return ret;
}

static int erpc_wifi_socket_create_msg_process(erpc_wifi_msg_socket_create_t *d)
{
	if (!d) return -EINVAL;

	if (wait_ra_awake(ERPC_PMGR_JOB_ID_SEND, ERPC_WIFI_SOCKET_CREATE_CMD) != 0) {
		return -EAGAIN;
	}

	int ret = ra6w1_socket(d->domain, d->type, d->protocol);
	if (ret >= 0) {
		g_sockets_created++;
	}

	LOG_ERR("++++ create socket: g_sockets_created: %d", g_sockets_created);

//	if (ret < 0) {
//		erpc_wifi_send_cmd(ERPC_WIFI_AP_RESET_CMD, NULL , 0, 0);
//	}

//	erpc_wifi_pmgr_ram_release(0);

	return ret;
}

static int erpc_wifi_socket_close_msg_process(erpc_wifi_mqt_socket_close_t *d)
{
	if (!d) return -EINVAL;
	if (d->fd < 0) return -EBADFD;

	if (wait_ra_awake(ERPC_PMGR_JOB_ID_SEND, ERPC_WIFI_SOCKET_CLOSE_CMD) != 0) {
		return -EAGAIN;
	}

	int ret = ra6w1_close(d->fd);

	if (!ret && (d->socket_free_cb)) {
		d->socket_free_cb(d->fd);
		d->fd = -1;
		*d->flags = 0;
		g_sockets_created--;
	}

	LOG_ERR("++++ close socket: g_sockets_created: %d", g_sockets_created);

	if (d->lock) {
		k_mutex_unlock(d->lock);
	}

//	erpc_wifi_pmgr_ram_release(ERPC_PMGR_JOB_ID_SEND);

	return ret;
}

static int erpc_wifi_socket_getopt_msg_process(erpc_wifi_msg_sockgetopt_t *d)
{
	if (wait_ra_awake(ERPC_PMGR_JOB_ID_SEND, ERPC_WIFI_SOCKET_GETOPT_CMD) != 0) {
		return -EAGAIN;
	}

	int ret =ra6w1_getsockopt(d->fd, d->level, d->optname, d->optval, d->optlen);

	//erpc_wifi_pmgr_ram_release(0);

	return ret;
}

static int erpc_wifi_socket_setopt_msg_process(erpc_wifi_msg_socksetopt_t *d)
{
	if (wait_ra_awake(ERPC_PMGR_JOB_ID_SEND, ERPC_WIFI_SOCKET_SETOPT_CMD) != 0) {
		return -EAGAIN;
	}

	int ret = ra6w1_setsockopt(d->fd, d->level, d->optname, d->optval, d->optlen);

//	erpc_wifi_pmgr_ram_release(0);

	return ret;
}

static int erpc_wifi_socket_evtent_msg_process(int *fd)
{
	if (!fd) return -EINVAL;

	/* not wait_ra_awake procedure here.
	 * We should call for socket events when device is awake only */

	return get_socket_events(*fd);
}

static int erpc_wifi_socket_dns_addr_info_msg_process(erpc_wifi_dns_addrinfo_t *fd)
{
	if (!fd) return -EINVAL;

	if (wait_ra_awake(ERPC_PMGR_JOB_ID_RECV, EPRC_WIFI_GET_DNS_ADDR_INFO_CMD) != 0) {
		return -EAGAIN;
	}

	WIFIReturnCode_t server_status = ra6w1_dns_getaddrinfo(fd->node, fd->result,
			fd->max_dns_addreses, fd->actual_count);

	return erpc_wifi_ret_to_errno(server_status);
}

static int erpc_wifi_wait_ra_awake_msg_process(int *job_id)
{
	if (!job_id) return -EINVAL;

	return wait_ra_awake((uint32_t) *job_id, ERPC_WIFI_WAIT_RA_AWAKE_CMD);
}

static int erpc_wifi_activity_cb_msg_process(void *d)
{
	if (!d) return -EINVAL;

	bool enable = *(bool *) d;

	if (enable) {
		/* Callback may already be registered from init; remove first to avoid double-add error */
		gpio_remove_callback(n_int_gpio.port, &n_int_cb);
		gpio_add_callback(n_int_gpio.port, &n_int_cb);
	} else {
		gpio_remove_callback(n_int_gpio.port, &n_int_cb);
	}

	return 0;
}

int erpc_wifi_cmd_socket_handlers_init(void)
{
	erpc_wifi_register_cmd_handler(ERPC_WIFI_BIND_CMD, erpc_wifi_bind_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_CONNECT_CMD, erpc_wifi_connect_msg_process);

	erpc_wifi_register_cmd_handler(ERPC_WIFI_LISTEN_CMD, erpc_wifi_listen_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_ACCEPT_CMD, erpc_wifi_accept_msg_process);

	erpc_wifi_register_cmd_handler(ERPC_WIFI_RECVFROM_CMD, erpc_wifi_recvfrom_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_SENDTO_CMD, erpc_wifi_sendto_msg_process);

	erpc_wifi_register_cmd_handler(ERPC_WIFI_SOCKET_CREATE_CMD, erpc_wifi_socket_create_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_SOCKET_CLOSE_CMD, erpc_wifi_socket_close_msg_process);

	erpc_wifi_register_cmd_handler(ERPC_WIFI_SOCKET_GETOPT_CMD, erpc_wifi_socket_getopt_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_SOCKET_SETOPT_CMD, erpc_wifi_socket_setopt_msg_process);

	erpc_wifi_register_cmd_handler(EPRC_WIFI_GET_SOCKET_EVT_CMD, erpc_wifi_socket_evtent_msg_process);
	erpc_wifi_register_cmd_handler(EPRC_WIFI_GET_DNS_ADDR_INFO_CMD, erpc_wifi_socket_dns_addr_info_msg_process);

	erpc_wifi_register_cmd_handler(ERPC_WIFI_WAIT_RA_AWAKE_CMD, erpc_wifi_wait_ra_awake_msg_process);

	erpc_wifi_register_cmd_handler(ERPC_WIFI_ACTIVITY_CB_CMD, erpc_wifi_activity_cb_msg_process);


	k_sem_init(&sem_if_wkup, 0, 1);

	if (gpio_pin_configure_dt(&n_int_gpio, GPIO_INPUT) != 0) {
		return -1;
	}

	if (gpio_pin_interrupt_configure_dt(&n_int_gpio, GPIO_INT_EDGE_TO_ACTIVE) != 0) {
		return -1;
	}

	gpio_init_callback(&n_int_cb, n_int_iface_active_cb, BIT(n_int_gpio.pin));

	erpc_wifi_set_wkup_source(ERPC_WIFI_WKUP_UNKNOWN);
	gpio_add_callback(n_int_gpio.port, &n_int_cb);

	return 0;
}
