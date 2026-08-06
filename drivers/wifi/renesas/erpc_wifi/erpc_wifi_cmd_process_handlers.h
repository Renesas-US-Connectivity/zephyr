#pragma once

#include <stdint.h>

// Event flags that match what you'll return
#define SOCKET_EVENT_RX    0x01 // POLLIN - Data available to read
#define SOCKET_EVENT_TX    0x02 // POLLOUT - Ready to write
#define SOCKET_EVENT_ERR   0x04 // POLLERR - Error condition
#define SOCKET_EVENT_CLOSE 0x08 // POLLHUP - Connection closed

/* PMGR DPM job ids used by RA6W1 socket shim */
#ifndef ERPC_PMGR_JOB_ID_SEND
#define ERPC_PMGR_JOB_ID_SEND (1U)
#endif
#ifndef ERPC_PMGR_JOB_ID_RECV
#define ERPC_PMGR_JOB_ID_RECV (2U)
#endif

int erpc_wifi_cmd_socket_handlers_init(void);
int erpc_wifi_cmd_ap_handlers_init(void);
int erpc_wifi_cmd_ps_handlers_init(void);
void erpc_wifi_ps_reschedule_sleep(uint32_t delay_ms);
void erpc_wifi_ps_cancel_sleep(void);
