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

LOG_MODULE_REGISTER(erpc_wifi_cmd_ps_process, CONFIG_WIFI_LOG_LEVEL);

/* Defined in erpc_wifi_cmd_socket_process_handlers.c */
extern void erpc_wifi_socket_tx_block_set(bool enable, uint32_t timeout_ms);

#define TIMEOUT_MS_COEF 1000

#define DT_DRV_COMPAT renesas_erpc_wifi_spi

#define GPIO_WAKEUP_NODE DT_ALIAS(wakeup_gpio)
#define GPIO_WAKEUP_PORT DT_GPIO_CTLR(GPIO_WAKEUP_NODE, gpios)
#define GPIO_WAKEUP_PIN DT_GPIO_PIN(GPIO_WAKEUP_NODE, gpios)
#define GPIO_WAKEUP_FLAGS DT_GPIO_FLAGS(GPIO_WAKEUP_NODE, gpios)

static const struct device *g_gpio_wakeup_dev = DEVICE_DT_GET(GPIO_WAKEUP_PORT);

/* Delayable work: after TIMEOUT, allow RA6W1 to sleep */
static struct k_work_delayable g_ps_enable_work;


/* ---------------- Wi-Fi Power Save mapping ----------------
 *
 * Customer expectation (Zephyr net_mgmt):
 *  - LISTEN_INTERVAL / WAKEUP_MODE / EXIT_STRATEGY / TIMEOUT are "parameters"
 *  - STATE (enable/disable) actually turns low-power behavior on/off
 *
 * IMPORTANT (this project):
 *  - Host owns enabling/disabling DPM sleep via RA6W1 PMGR sleep constraint.
 *  - RA6W1 wifi_ps_apply() only configures parameters (PTIM etc). It does not force sleep.
 *
 * TIMEOUT semantics:
 *  - We interpret params.timeout_ms as "delay before allowing RA6W1 to enter DPM sleep"
 *    after STATE is enabled (common app behavior: finish work window, then sleep).
 */

struct erpc_ps_cache {
	uint32_t listen_interval;
	uint32_t wakeup_mode;
	uint32_t exit_strategy;
	uint32_t timeout_ms;

	bool li_set;
	bool wm_set;
	bool ex_set;
	bool tmo_set;

	bool enabled;          /* last STATE */
	bool allow_sleep_sent; /* whether we have removed the sleep constraint */
};

static struct erpc_ps_cache g_ps;
static erpc_wifi_wkup_src_t wkup_src = ERPC_WIFI_WKUP_UNKNOWN;

/* RA6W1 PMGR constraint bit masks (rm_pmgr_w_api.h) */
#ifndef PMGR_CONSTRAINT_SLEEP_PROHIBITED
#define PMGR_CONSTRAINT_SLEEP_PROHIBITED   (1U << 0)
#endif
#ifndef PMGR_CONSTRAINT_POWER_RAM
#define PMGR_CONSTRAINT_POWER_RAM (1U << 2)
#endif

static int ps_set_state(bool enable)
{
	int ret = 0;

	g_ps.enabled = enable;

	if (g_ps.enabled) {
		uint32_t delay = (g_ps.tmo_set) ? g_ps.timeout_ms : 0U;

		k_work_reschedule(&g_ps_enable_work, K_MSEC(delay));
		erpc_wifi_socket_tx_block_set(false, 0U);
		LOG_INF("PS STATE ENABLE requested (delay=%u ms)", delay);
	} else {
		k_work_cancel_delayable(&g_ps_enable_work);

		erpc_wifi_send_cmd(ERPC_WIFI_WAIT_RA_AWAKE_CMD, &(int) { /*ERPC_PMGR_JOB_ID_SEND*/ 0}, sizeof(int), 0);
		erpc_wifi_send_cmd(ERPC_WIFI_PS_SLEEP_ENABLE_CMD, &(bool) { false }, sizeof(bool), 0);

		LOG_INF("PS STATE DISABLED");
	}

	return ret;
}

static int erpc_wifi_ap_get_ps_set_param_msg_process(erpc_wifi_ps_t *d)
{
	int ret = -EINVAL;;

	if (!d) return ret;

	switch (d->params->type) {
	default:
		return -ENOTSUP;
	case WIFI_PS_PARAM_STATE:
		return ps_set_state(d->params->enabled);

	case WIFI_PS_PARAM_LISTEN_INTERVAL:
		g_ps.listen_interval = (uint32_t)d->params->listen_interval;
		g_ps.li_set = true;
		ret = ra6w1_wifi_ps_set_param(RA_WIFI_PS_PARAM_LISTEN_INTERVAL, g_ps.listen_interval);
		break;
	case WIFI_PS_PARAM_WAKEUP_MODE:
		g_ps.wakeup_mode = (uint32_t)d->params->wakeup_mode;
		g_ps.wm_set = true;
		ret = ra6w1_wifi_ps_set_param(RA_WIFI_PS_PARAM_WAKEUP_MODE, g_ps.wakeup_mode);
		break;
	case WIFI_PS_PARAM_EXIT_STRATEGY: {
		uint32_t ra_exit;

		/* Zephyr → RA enum translation
		 * Zephyr: CUSTOM=0, EVERY=1
		 * RA:     EVERY=0,  CUSTOM=1
		 */
		if (d->params->exit_strategy == WIFI_PS_EXIT_CUSTOM_ALGO) {
			ra_exit = RA_WIFI_PS_EXIT_CUSTOM_ALGO;
		} else {
			ra_exit = RA_WIFI_PS_EXIT_EVERY_TIM;
		}

		g_ps.exit_strategy = ra_exit;
		g_ps.ex_set = true;
		ret = ra6w1_wifi_ps_set_param(RA_WIFI_PS_PARAM_EXIT_STRATEGY, ra_exit);
	}
	break;
	case WIFI_PS_PARAM_TIMEOUT:
		g_ps.timeout_ms = (uint32_t)d->params->timeout_ms;
		g_ps.tmo_set = true;

		/* Forward to RA – RA PMGR policy decides post-TX behavior */
		ret = ra6w1_wifi_ps_set_param(RA_WIFI_PS_PARAM_TIMEOUT_MS, g_ps.timeout_ms + TIMEOUT_MS_COEF);
		break;
	}

	if (!ret) {
		ret = ra6w1_wifi_ps_apply();
	}

	return ret;
}

static void ps_set_param_reload(void)
{
	int ret = -1; // If one fails everyone will fail.

	if (g_ps.li_set) {
		ret = ra6w1_wifi_ps_set_param(RA_WIFI_PS_PARAM_LISTEN_INTERVAL, g_ps.listen_interval);
	}

	if (g_ps.wm_set && !ret) {
		ret = ra6w1_wifi_ps_set_param(RA_WIFI_PS_PARAM_WAKEUP_MODE, g_ps.wakeup_mode);
	}

	if (g_ps.ex_set && !ret) {
		ret = ra6w1_wifi_ps_set_param(RA_WIFI_PS_PARAM_EXIT_STRATEGY, g_ps.exit_strategy);
	}

	if (g_ps.tmo_set && !ret) {
		ret = ra6w1_wifi_ps_set_param(RA_WIFI_PS_PARAM_TIMEOUT_MS, g_ps.timeout_ms + TIMEOUT_MS_COEF);
	}

	if (!ret) {
		ra6w1_wifi_ps_apply();
	}
}

static int erpc_wifi_ap_get_ps_get_param_msg_process(erpc_wifi_ps_t *d)
{
	uint32_t ra_exit;
	int ret = -EINVAL;;

	if (!d) return ret;

	ret = ra6w1_wifi_ps_get_param(RA_WIFI_PS_PARAM_LISTEN_INTERVAL, &g_ps.listen_interval);
	if (ret) {
		return -EFAULT;
	}

	d->params->listen_interval = (unsigned short) g_ps.listen_interval;

	ret = ra6w1_wifi_ps_get_param(RA_WIFI_PS_PARAM_WAKEUP_MODE, &g_ps.wakeup_mode);
	if (ret) {
		return -EFAULT;
	}

	d->params->wakeup_mode = (enum wifi_ps_wakeup_mode) g_ps.wakeup_mode;

	ret = ra6w1_wifi_ps_get_param(RA_WIFI_PS_PARAM_EXIT_STRATEGY, &ra_exit);
	if (ret) {
		return -EFAULT;
	}

	/* Zephyr → RA enum translation
	 * Zephyr: CUSTOM=0, EVERY=1
	 * RA:     EVERY=0,  CUSTOM=1
	 */
	switch (ra_exit) {
	case RA_WIFI_PS_EXIT_CUSTOM_ALGO:
		g_ps.exit_strategy = WIFI_PS_EXIT_CUSTOM_ALGO;
		break;
	case RA_WIFI_PS_EXIT_EVERY_TIM:
		g_ps.exit_strategy = RA_WIFI_PS_EXIT_EVERY_TIM;
		break;
	default:
		g_ps.exit_strategy = ra_exit;
		break;
	}

	d->params->exit_strategy = (enum wifi_ps_exit_strategy) g_ps.exit_strategy;

	ret = ra6w1_wifi_ps_get_param(RA_WIFI_PS_PARAM_TIMEOUT_MS, &g_ps.timeout_ms);
	if (ret) {
		return -EFAULT;
	}

	g_ps.timeout_ms -= TIMEOUT_MS_COEF;

	d->params->timeout_ms = g_ps.timeout_ms;

	d->params->enabled = (enum wifi_ps) g_ps.enabled;

	return 0;
}

static int erpc_wifi_socket_evtent_msg_in_sleep_mode_process(int *fd)
{
//	k_sleep(K_USEC(100));

	return -ENODATA;
}

static void ps_allow_sleep_work(struct k_work *work)
{
	ARG_UNUSED(work);

	extern bool erpc_wifi_is_socket_op_active(void);
	if (erpc_wifi_cmd_is_processing() || erpc_wifi_is_socket_op_active()) {
		k_work_reschedule(&g_ps_enable_work, K_MSEC(1000));
	} else {
		erpc_wifi_unregister_cmd_handler(EPRC_WIFI_GET_SOCKET_EVT_CMD);
		erpc_wifi_register_cmd_handler(EPRC_WIFI_GET_SOCKET_EVT_CMD, erpc_wifi_socket_evtent_msg_in_sleep_mode_process);

		erpc_wifi_send_cmd(ERPC_WIFI_ACTIVITY_CB_CMD, &(bool) { true }, sizeof(bool), -1);
		int ret = erpc_wifi_send_cmd(ERPC_WIFI_PS_SLEEP_ENABLE_CMD, &(bool) { true }, sizeof(bool), -1);
		LOG_DBG("allow sleep call ret: %d", ret);
	}
}

static int erpc_wifi_ap_get_ps_sleep_enable_msg_process(void *d)
{
	int32_t rc = 0;

	if (!d) return -EINVAL;

	bool enable = *(bool *) d;

	g_ps.allow_sleep_sent = enable;
	uint32_t gate_ms = g_ps.tmo_set ? (g_ps.timeout_ms) : 3000U;

	if (enable) {
		rc = ra6w1_pmgr_remove_sleep_constraint(PMGR_CONSTRAINT_SLEEP_PROHIBITED);
		ra6w1_pmgr_remove_sleep_constraint(PMGR_CONSTRAINT_POWER_RAM);

		k_sleep(K_MSEC(300));

		/* Stop server event monitor polling so the module can become idle and enter DPM */
		extern volatile bool g_dpm_awake;
		g_dpm_awake = false;

		erpc_wifi_socket_tx_block_set(true, gate_ms);

	} else {
		erpc_wifi_socket_tx_block_set(false, 0);
		rc = ra6w1_pmgr_add_sleep_constraint(PMGR_CONSTRAINT_SLEEP_PROHIBITED);
		ra6w1_pmgr_add_sleep_constraint(PMGR_CONSTRAINT_POWER_RAM);
		ra6w1_wifi_ps_apply();
	}

	LOG_INF("PS enable rc: %d", rc);

	return 0;
}

static int erpc_wifi_is_ps_enabled_msg_process(erpc_wifi_ps_t *d)
{
	if (g_ps.enabled) {
		uint32_t delay = (g_ps.tmo_set && g_ps.timeout_ms > 0U) ? g_ps.timeout_ms : 5000U;
		k_work_reschedule(&g_ps_enable_work, K_MSEC(delay));
		/* Re-register sleep constraint after wakeup so next remove succeeds */
		(void)ra6w1_pmgr_add_sleep_constraint(PMGR_CONSTRAINT_SLEEP_PROHIBITED);
	}

	return g_ps.enabled;
}

void erpc_wifi_set_wkup_source(erpc_wifi_wkup_src_t src)
{
	wkup_src = src;
}

erpc_wifi_wkup_src_t erpc_wifi_get_wkup_source(void)
{
	return wkup_src;
}

void erpc_wifi_ps_set_enabled(bool enabled)
{
	g_ps.enabled = enabled;
}

void erpc_wifi_ps_sync_timeout(uint32_t timeout_ms, bool tmo_set)
{
	g_ps.timeout_ms = timeout_ms;
	g_ps.tmo_set = tmo_set;
}

void erpc_wifi_ps_reschedule_sleep(uint32_t delay_ms)
{
	if (g_ps.enabled) {
		k_work_reschedule(&g_ps_enable_work, K_MSEC(delay_ms));
	}
}

void erpc_wifi_ps_cancel_sleep(void)
{
	k_work_cancel_delayable(&g_ps_enable_work);
}

int erpc_wifi_cmd_ps_handlers_init(void)
{
	if (!device_is_ready(g_gpio_wakeup_dev)) {
		LOG_ERR("RRQ GPIO controller not ready: %s", g_gpio_wakeup_dev->name);
		return -1;
	}

	gpio_pin_configure(g_gpio_wakeup_dev, GPIO_WAKEUP_PIN,
					GPIO_OUTPUT_ACTIVE | GPIO_WAKEUP_FLAGS);

	erpc_wifi_register_cmd_handler(ERPC_WIFI_PS_SET_PARAM_CMD, (erpc_wifi_msg_handler_t)erpc_wifi_ap_get_ps_set_param_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_PS_GET_PARAM_CMD, (erpc_wifi_msg_handler_t)erpc_wifi_ap_get_ps_get_param_msg_process);
	erpc_wifi_register_cmd_handler(ERPC_WIFI_PS_SLEEP_ENABLE_CMD, (erpc_wifi_msg_handler_t)erpc_wifi_ap_get_ps_sleep_enable_msg_process);

	erpc_wifi_register_cmd_handler(ERPC_WIFI_IS_PS_ENABLED, (erpc_wifi_msg_handler_t)erpc_wifi_is_ps_enabled_msg_process);

	k_work_init_delayable(&g_ps_enable_work, ps_allow_sleep_work);

	return 0;
}
