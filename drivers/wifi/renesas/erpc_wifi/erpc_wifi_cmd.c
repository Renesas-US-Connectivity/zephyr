#include "erpc_wifi_cmd.h"

#include "erpc_wifi.h"
#include <c_wifi_host_to_ra_client.h>
#include <zephyr/net/socket.h>

LOG_MODULE_REGISTER(erpc_wifi_cmd, CONFIG_WIFI_LOG_LEVEL);

#define ERPC_WIFI_MSG_MAX 10

#define MSG_TASK_STACK_SIZE 3200
K_THREAD_STACK_DEFINE(msg_task_stack, MSG_TASK_STACK_SIZE);

static struct k_thread msgq_thread;
static k_tid_t msgq_task_tid;
static struct k_timer process_wdg;
static bool is_cmd_processing;

extern void erpc_wifi_lock(void);
extern void erpc_wifi_unlock(void);

K_MSGQ_DEFINE(cmd_msg_queue, sizeof(erpc_wifi_msg_data_t), ERPC_WIFI_MSG_MAX, 4);

static erpc_wifi_msg_prosess_t erpc_wifi_socket_handlers[] = {
		{ ERPC_WIFI_BIND_CMD, NULL, NULL },
		{ ERPC_WIFI_CONNECT_CMD, NULL, NULL },
		{ ERPC_WIFI_LISTEN_CMD, NULL, NULL },
		{ ERPC_WIFI_ACCEPT_CMD, NULL, NULL },

		{ ERPC_WIFI_RECV_CMD, NULL, NULL },
		{ ERPC_WIFI_RECVFROM_CMD, NULL, NULL },
		{ ERPC_WIFI_SEND_CMD, NULL, NULL },
		{ ERPC_WIFI_SENDTO_CMD, NULL, NULL },

		{ ERPC_WIFI_SOCKET_CREATE_CMD, NULL, NULL },
		{ ERPC_WIFI_SOCKET_CLOSE_CMD, NULL, NULL },

		{ ERPC_WIFI_SOCKET_GETOPT_CMD, NULL, NULL },
		{ ERPC_WIFI_SOCKET_SETOPT_CMD, NULL, NULL },

		{ EPRC_WIFI_GET_SOCKET_EVT_CMD, NULL, NULL },
		{ EPRC_WIFI_GET_SERVER_EVT_CMD, NULL, NULL },
		{ EPRC_WIFI_GET_DNS_ADDR_INFO_CMD, NULL, NULL },
		{ ERPC_WIFI_WAIT_RA_AWAKE_CMD, NULL, NULL },

		{ ERPC_WIFI_AP_CONNECT_CMD, NULL, NULL },
		{ ERPC_WIFI_AP_DISCONNECT_CMD, NULL, NULL },
		{ ERPC_WIFI_AP_SCAN_CMD, NULL, NULL },
		{ ERPC_WIFI_AP_GET_RSSI_CMD, NULL, NULL },
		{ ERPC_WIFI_AP_GET_CONNECTION_INFO_CMD, NULL, NULL },
		{ ERPC_WIFI_AP_RESET_CMD, NULL, NULL },
		{ ERPC_WIFI_GET_DRIVER_VER_CMD, NULL, NULL },

		{ ERPC_WIFI_PS_SET_PARAM_CMD, NULL, NULL },
		{ ERPC_WIFI_PS_GET_PARAM_CMD, NULL, NULL },
		{ ERPC_WIFI_PS_SLEEP_ENABLE_CMD, NULL, NULL },

		{ ERPC_WIFI_IS_PS_ENABLED, NULL, NULL },
		{ ERPC_WIFI_ACTIVITY_CB_CMD, NULL, NULL },

		{ ERPC_WIFI_TEST_CMD, NULL, NULL },
};

char *erpc_wifi_cmd_to_str(erpc_wifi_cmd_t cmd)
{
	static char str_buf[30];
	char *cmd_str = NULL;

	memset(str_buf, 0, sizeof(str_buf));

	switch (cmd) {
	case ERPC_WIFI_BIND_CMD:
		cmd_str = "bind";
		break;
	case ERPC_WIFI_CONNECT_CMD:
		cmd_str = "connect";
		break;
	case ERPC_WIFI_LISTEN_CMD:
		cmd_str = "listen";
		break;
	case ERPC_WIFI_ACCEPT_CMD:
		cmd_str = "accept";
		break;
	case ERPC_WIFI_RECV_CMD:
		cmd_str = "recv";
		break;
	case ERPC_WIFI_RECVFROM_CMD:
		cmd_str = "recvfrom";
		break;
	case ERPC_WIFI_SEND_CMD:
		cmd_str = "send";
		break;
	case ERPC_WIFI_SENDTO_CMD:
		cmd_str = "sendto";
		break;
	case ERPC_WIFI_SOCKET_CREATE_CMD:
		cmd_str = "socket create";
		break;
	case ERPC_WIFI_SOCKET_CLOSE_CMD:
		cmd_str = "socket close";
		break;
	case ERPC_WIFI_PS_SET_PARAM_CMD:
		cmd_str = "ps set param";
		break;
	case ERPC_WIFI_PS_GET_PARAM_CMD:
		cmd_str = "ps get param";
		break;
	case ERPC_WIFI_PS_SLEEP_ENABLE_CMD:
		cmd_str = "ps sleep enable";
		break;
	case EPRC_WIFI_GET_SERVER_EVT_CMD:
		cmd_str = "get server evt";
		break;

	case ERPC_WIFI_TEST_CMD:
		cmd_str = "TEST";
		break;

	/* Those are too noisy. Restrict by default */
	case EPRC_WIFI_GET_SOCKET_EVT_CMD:
		cmd_str = "get socket evt";
		break;
//		return NULL;
	case ERPC_WIFI_WAIT_RA_AWAKE_CMD:
		cmd_str = "wait ra awake";
		break;
	case ERPC_WIFI_IS_PS_ENABLED:
		cmd_str = "is ps enabled";
		break;
//		return NULL;
	default:
		snprintf(str_buf, sizeof(str_buf), "unknown (%d)", cmd);
		break;
	}

	if (cmd_str) {
		snprintf(str_buf, sizeof(str_buf), "%s (%d)", cmd_str, cmd);
	}

	return str_buf;
}

static void process_wgd_tout(struct k_timer *timer)
{
	bool *is_processing = (bool *) k_timer_user_data_get(timer);

	if (is_processing) {
		*is_processing = false;
	}

	LOG_ERR("message cmd wdg tout");
}

static int update_process_wdg(struct k_timer *t)
{
	bool *is_processing = (bool *) k_timer_user_data_get(t);

	if (is_processing) {
		*is_processing = true;
		k_timer_start(t, K_MSEC(5000), K_NO_WAIT);
	}

	return 0;
}

static bool is_erpc_wifi_cmd_handler_registered(erpc_wifi_cmd_t cmd)
{
	return erpc_wifi_socket_handlers[cmd].h != NULL;
}

static int alloc_cmd_msg_data(erpc_wifi_msg_data_t *msg, void *data, size_t size)
{
	if (!msg) {
		return -EINVAL;
	}

	/* There is nothing to copy */
	if (!data) {
		return 0;
	}

	msg->data = k_calloc(1, size);
	if (!msg->data) {
		LOG_ERR("Failed to allocate cmd: %d msg data", msg->cmd);
		return -ENOMEM;
	}

	memcpy(msg->data, data, size);

	return 0;
}

int erpc_wifi_send_cmd(erpc_wifi_cmd_t cmd, void *data, size_t size, int tout)
{
	static uint64_t id;
	int ret = 0;
	int cmd_ret = 0;
	k_timeout_t timeout = K_NO_WAIT;
	erpc_wifi_msg_data_t msg = { .cmd = cmd, .sem = NULL, .cmd_ret = &cmd_ret, .id = id };

	if (cmd > ERPC_WIFI_LAST_CMD) {
		return -ERANGE;
	}

	if (!is_erpc_wifi_cmd_handler_registered(cmd)) {
		return -ENOENT;
	}

	ret = alloc_cmd_msg_data(&msg, data, size);
	if (ret < 0) {
		return ret;
	}

	if (tout != 0) {
		timeout = (tout == -1) ? K_FOREVER : K_MSEC(tout);

		if ((msg.sem = k_malloc(sizeof(*msg.sem))) == NULL) {
			return -ENOMEM;
		}

		k_sem_init(msg.sem, 0, 1);
	}

	id++;

	if (k_msgq_put(&cmd_msg_queue, &msg, K_NO_WAIT) ==  0) {

		char *cmd_str = erpc_wifi_cmd_to_str(msg.cmd);
		if (cmd_str) {
			LOG_DBG(">>> (id: %llu) send for process: %s", msg.id, cmd_str);
		}

		if (msg.sem && (ret = k_sem_take(msg.sem, timeout)) != 0) {
			ret = -ETIMEDOUT;
		} else {
			ret = *msg.cmd_ret;
		}

	} else {
		LOG_ERR("Failed to enqueue message cmd: %d", cmd);
		ret = -EAGAIN;

		if (msg.data) {
			k_free(msg.data);
		}

		if (msg.sem) {
			k_free(msg.sem);
		}
	}

	if (ret == 0) {
		ret = cmd_ret;
	}

//	k_sleep(K_MSEC(20));

	return ret;
}

static void erpc_wifi_msg_handler_task(void *arg1, void *arg2, void *arg3)
{
	struct k_msgq *msgq = (struct k_msgq *) arg1;
	struct k_timer *wdg_tout = (struct k_timer *) arg2;

	erpc_wifi_msg_data_t msg = { 0 };
	int32_t ret = 0;

	if (!msgq) {
		LOG_ERR("msgq NULL");
		return;
	}

	while (1) {
		k_msgq_get(msgq, (erpc_wifi_msg_data_t *) &msg, K_FOREVER);

		switch (msg.cmd) {
		case EPRC_WIFI_GET_SOCKET_EVT_CMD:
		case EPRC_WIFI_GET_SERVER_EVT_CMD:
		case ERPC_WIFI_WAIT_RA_AWAKE_CMD:
		case ERPC_WIFI_PS_SET_PARAM_CMD:
		case ERPC_WIFI_PS_GET_PARAM_CMD:
		case ERPC_WIFI_PS_SLEEP_ENABLE_CMD:
		case ERPC_WIFI_IS_PS_ENABLED:
			break;
		default:
			update_process_wdg(wdg_tout);
			break;
		}

		erpc_wifi_lock();
		if (erpc_wifi_socket_handlers[msg.cmd].h) {
			ret = erpc_wifi_socket_handlers[msg.cmd].h(msg.data);
		} else {
			ret = -ENOTSUP;
		}
		erpc_wifi_unlock();

		if (msg.data) {
			k_free(msg.data);
			msg.data = NULL;
		}

		if (msg.cmd_ret) {
			*msg.cmd_ret = ret;
		}

		if (msg.sem) {
			k_sem_give(msg.sem);
			k_free(msg.sem);
		}

		char *cmd_str = erpc_wifi_cmd_to_str(msg.cmd);
		if (cmd_str) {
			LOG_DBG("<<< (id: %llu) processed: %s: ret: %d", msg.id, cmd_str, ret);
		}
	}
}

bool erpc_wifi_cmd_is_processing(void)
{
//	uint32_t cnt = k_msgq_num_free_get(&cmd_msg_queue);
//	LOG_ERR("is processing: %d", is_cmd_processing);

	return /*!(cnt == ERPC_WIFI_MSG_MAX) && */ is_cmd_processing;
}

static int erpc_wifi_test_msg_process(int *i)
{
	if (!i) return -EINVAL;

	int data = *(int *) i;
	int ret = 1000 + rand() % 100;

	LOG_ERR("test message processed: %d, ret: %d", data, ret);

	k_sleep(K_MSEC(100));

	return ret;
}

int erpc_wifi_register_cmd_handler(erpc_wifi_cmd_t cmd, erpc_wifi_msg_handler_t h)
{
	if (cmd >= ERPC_WIFI_LAST_CMD) {
		LOG_ERR("cmd : %d out of range", cmd);
		return -ERANGE;
	}

	if (erpc_wifi_socket_handlers[cmd].h) {
		return 0;
	}

	erpc_wifi_socket_handlers[cmd].h = h;
	LOG_DBG("handler for cmd: %d registered: %p", cmd, h);

	return 0;
}

int erpc_wifi_unregister_cmd_handler(erpc_wifi_cmd_t cmd)
{
	if (cmd >= ERPC_WIFI_LAST_CMD) {
		LOG_ERR("cmd : %d out of range", cmd);
		return -ERANGE;
	}

	erpc_wifi_socket_handlers[cmd].h = NULL;

	return 0;
}

int erpc_wifi_cmd_init(void)
{
	k_timer_init(&process_wdg, process_wgd_tout, 0);
	k_timer_user_data_set(&process_wdg, &is_cmd_processing);

//	erpc_wifi_register_cmd_handler(ERPC_WIFI_TEST_CMD, erpc_wifi_test_msg_process);

	msgq_task_tid = k_thread_create(&msgq_thread, msg_task_stack, K_THREAD_STACK_SIZEOF(msg_task_stack),
			erpc_wifi_msg_handler_task,
			(struct k_msgq *) &cmd_msg_queue, &process_wdg, NULL,
			K_PRIO_PREEMPT(10), 0, K_NO_WAIT);

//	erpc_wifi_send_cmd(ERPC_WIFI_TEST_CMD, &(int) { 30000 }, sizeof(int), -1);
//
//	for (int i = 0; i < 5; i++) {
//		int ret = erpc_wifi_send_cmd(ERPC_WIFI_TEST_CMD, &(int) { i + 10000 }, sizeof(int), 50 * (1 + i));
//		LOG_DBG("test ret: %d", ret);
//	}
//
//	for (int i = 0; i < 3; i++) {
//		erpc_wifi_send_cmd(ERPC_WIFI_TEST_CMD, &(int) { i + 20000 }, sizeof(int), 0);
//	}

	return 0;
}

