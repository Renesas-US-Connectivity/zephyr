#include "erpc_wifi_cmd_process_handlers.h"
#include "erpc_wifi_cmd.h"
#include "erpc_wifi.h"
#include "service/c_wifi_host_to_ra_client.h"

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include "zephyr/net/socket.h"

LOG_MODULE_REGISTER(erpc_wifi_cmd_ap_process, CONFIG_WIFI_LOG_LEVEL);

extern void erpc_wifi_lock(void);
extern void erpc_wifi_unlock(void);

#define DT_DRV_COMPAT renesas_erpc_wifi_spi

static struct gpio_dt_spec n_int_gpio = GPIO_DT_SPEC_GET(DT_DRV_INST(0), int_gpios);
static struct gpio_callback n_int_cb;
static struct k_sem sem_if_enabled;

static int erpc_wifi_ap_connect_msg_process(struct erpc_wifi_data *d)
{
	if (!d) return -EINVAL;

	erpc_wifi_lock();
	int ret = WIFI_ConnectAP(&d->drv_nwk_params);
	erpc_wifi_unlock();
	return ret;
}

static int erpc_wifi_ap_disconnect_msg_process(void *d)
{
	erpc_wifi_lock();
	int ret = WIFI_Disconnect();
	erpc_wifi_unlock();
	return ret;
}

static int erpc_wifi_ap_scan_msg_process(erpc_wifi_scan_t *d)
{
	int ret = 0;

	if (!d) return -EINVAL;

	erpc_wifi_lock();
	if (eWiFiSuccess != WIFI_Scan(d->scan_results, d->max_bss_cnt)) {
		ret = -1;
	}
	erpc_wifi_unlock();

	return ret;
}

static int erpc_wifi_ap_getrssi_msg_process(erpc_wifi_get_rssi_t *d)
{
	if (!d || !d->rssi) return -EINVAL;

	int8_t rssi = -127;
	erpc_wifi_lock();
	int ret = WIFI_GetRSSI(&rssi);
	erpc_wifi_unlock();

	if (!ret) {
		*d->rssi = rssi;
	}

	return ret;
}

static int erpc_wifi_ap_get_connection_info_msg_process(erpc_wifi_get_connection_info_t *d)
{
	if (!d) return -EINVAL;

	erpc_wifi_lock();
	int ret = WIFI_GetConnectionInfo(d->connection_info);
	erpc_wifi_unlock();
	return ret;
}

static void n_int_iface_active_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	gpio_remove_callback(dev, cb);

	k_sem_give(&sem_if_enabled);
}

static int erpc_wifi_acquire_reset_pin(void)
{
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	struct gpio_dt_spec wifi_reset = GPIO_DT_SPEC_GET(DT_DRV_INST(0), reset_gpios);

	/* Set wifi_reset as output and activate reset */
	int err = gpio_pin_configure_dt(&wifi_reset, GPIO_OUTPUT_ACTIVE);
	if (err) {
		LOG_ERR("Error %d: failed to configure wifi_reset %s pin %d", err,
			wifi_reset.port->name, wifi_reset.pin);
		return err;
	}
#endif

	return 0;
}

static int erpc_wifi_release_reset_pin(void)
{
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	struct gpio_dt_spec wifi_reset = GPIO_DT_SPEC_GET(DT_DRV_INST(0), reset_gpios);

	/* Release the device from reset */
	int err = gpio_pin_configure_dt(&wifi_reset, GPIO_OUTPUT_INACTIVE);
	if (err) {
		return err;
	}
#endif

	return 0;
}

static int erpc_wifi_reset(void)
{
	k_timeout_t timeout = K_NO_WAIT;
	int err = 0;

	err = erpc_wifi_acquire_reset_pin();
	if (err) {
		return err;
	}

#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	k_sleep(K_MSEC(DT_INST_PROP_OR(0, reset_assert_duration_ms, 0)));
#endif

	/* Release the device from reset */
	err = erpc_wifi_release_reset_pin();
	if (err) {
		return err;
	}

	/* We can either wait a fixed amount of time for the RA6Wx device to
	   finish booting or we can wait for it to send us the reset complete
	   eveent. At present, the SPI interface does not support sending the
	   reset complete event so we have to just wait for boot to complete. */
#if DT_INST_NODE_HAS_PROP(0, boot_duration_ms)
	k_sleep(K_MSEC(DT_INST_PROP_OR(0, boot_duration_ms, 0)));
#else
	/*
	   While we are waiting for this sempahore the erpc_wifi_server_thread
	   is running and calling erpc_server_poll to check for any incoming
	   message. When a valid message is received the ra_erpc_server_event_handler
	   function is called. The eRPC middleware sets the nestingDetection flag
	   to true just before calling the hanlder and sets it to false when execution
	   of the handler is complete and it has returned.

	   When the handler receives the eDeviceReset event it gives the
	   sem_if_ready sempaphore. This causes the thread running the handler
	   to immediately suspend and the RTOS starts running this init function
	   once again. It continues through this init function and starts running
	   main. In main the application calls a driver function, however this call
	   fails as when it calls the associated eRPC function a nesting error occurs
	   as the ra_erpc_server_event_handler has not yet had time to finish and so
	   the nestingDetection flag is still true...

	   According to the Zephyr documentation, the system thread running this init
	   function has the highest priority and therefore we can simply increase the
	   priority of the erpc_wifi_server_thread to resolve this issue:
	   https://docs.zephyrproject.org/latest/kernel/services/threads/system_threads.html
	*/
#ifdef CONFIG_WIFI_ERPC_WIFI_RESET_TIMEOUT
	timeout = K_MSEC(CONFIG_WIFI_ERPC_WIFI_RESET_TIMEOUT);
#endif
	err = k_sem_take(&sem_if_enabled, timeout);
	if (err) {
		return err;
	}
#endif

	return 0;
}

static int erpc_wifi_ap_reset(erpc_wifi_get_connection_info_t *d)
{
	return erpc_wifi_reset();
}

static int erpc_wifi_get_driver_ver_msg_process(void *d)
{
	erpc_wifi_driver_version_t *drv_ver = (erpc_wifi_driver_version_t *)d;

	if (!d) return -EINVAL;

	erpc_wifi_lock();
	fw_version_get_driver_ver(drv_ver->fw_version_driver, drv_ver->fw_ver_drv_len);
	erpc_wifi_unlock();

	return 0;
}

static int erpc_wifi_get_server_evt_msg_process(void *d)
{
	if (!d) return -EINVAL;

	erpc_wifi_server_evt_t *event = (erpc_wifi_server_evt_t *)d;

	erpc_wifi_lock();
	/* Skip if module is sleeping to prevent blocking the mutex for 60+ seconds. */
	extern bool erpc_wifi_ps_is_enabled(void);
	extern int erpc_wifi_transport_slave_ready(void);
	if (erpc_wifi_ps_is_enabled() && !erpc_wifi_transport_slave_ready()) {
		erpc_wifi_unlock();
		return -EAGAIN;
	}
	erpc_get_server_event(event->event);
	erpc_wifi_unlock();

	return 0;
}

int erpc_wifi_cmd_ap_handlers_init(void)
{
	k_sem_init(&sem_if_enabled, 0, 1);

	gpio_init_callback(&n_int_cb, n_int_iface_active_cb, BIT(n_int_gpio.pin));
	int ret = gpio_add_callback(n_int_gpio.port, &n_int_cb);
	if (ret < 0) {
		return -EIO;
	}

	erpc_wifi_register_cmd_handler(ERPC_WIFI_AP_CONNECT_CMD, erpc_wifi_ap_connect_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_AP_DISCONNECT_CMD, erpc_wifi_ap_disconnect_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_AP_SCAN_CMD, erpc_wifi_ap_scan_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_AP_GET_RSSI_CMD, erpc_wifi_ap_getrssi_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_AP_GET_CONNECTION_INFO_CMD, erpc_wifi_ap_get_connection_info_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_AP_RESET_CMD, erpc_wifi_ap_reset);

	erpc_wifi_register_cmd_handler(ERPC_WIFI_GET_DRIVER_VER_CMD, erpc_wifi_get_driver_ver_msg_process);

	erpc_wifi_register_cmd_handler(EPRC_WIFI_GET_SERVER_EVT_CMD, erpc_wifi_get_server_evt_msg_process);

	return 0;
}
