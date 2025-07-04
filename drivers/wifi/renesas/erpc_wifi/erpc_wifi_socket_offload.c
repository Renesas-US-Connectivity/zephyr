/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(erpc_wifi_socket_offload, CONFIG_WIFI_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <zephyr/posix/fcntl.h>

#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_offload.h>
#include <zephyr/net/socket_offload.h>

#include "sockets_internal.h"

#include "erpc_wifi.h"
#include "c_wifi_host_to_ra_client.h"
#include "c_wifi_ra_to_host_client.h"

#include <zephyr/net/socket.h>

// TODO - can these come from wifi_common file in service?
#define ERPC_WIFI_AF_INET		2
#define ERPC_WIFI_AF_INET6		10

#define ERPC_WIFI_MAX_SOCKETS 4

struct erpc_wifi_socket {
	int fd;
	bool in_use;
};

static struct erpc_wifi_socket sockets[ERPC_WIFI_MAX_SOCKETS];
static const struct socket_op_vtable erpc_wifi_socket_fd_op_vtable;

static int erpc_wifi_socket_family_to_posix(uint8_t family_erpc_wifi, int *family)
{
	switch (family_erpc_wifi) {
	case ERPC_WIFI_AF_INET:
		*family = AF_INET;
		break;
	case ERPC_WIFI_AF_INET6:
		*family = AF_INET6;
		break;
	default:
		return -EAFNOSUPPORT;
		break;
	}

	return 0;
}

static int erpc_wifi_socket_family_from_posix(int family, uint8_t *family_erpc_wifi)
{
	switch (family) {
	case AF_INET:
		*family_erpc_wifi = ERPC_WIFI_AF_INET;
		break;
	case AF_INET6:
		*family_erpc_wifi = ERPC_WIFI_AF_INET6;
		break;
	default:
		// TODO - family supplied by sendto function is not valid, figure out why?
		*family_erpc_wifi = ERPC_WIFI_AF_INET;
//		return -EAFNOSUPPORT;
		break;
	}

	return 0;
}

static int erpc_wifi_socket_addr_from_posix(const struct sockaddr *addr,
				struct ra_erpc_sockaddr *addr_erpc_wifi)
{
	int err;

	err = erpc_wifi_socket_family_from_posix(addr->sa_family, &addr_erpc_wifi->sa_family);
	if (err) {
		LOG_ERR("unsupported family: %d", addr->sa_family);
		// TODO - better error code
		return -1;
	}

	// TODO - use MIN macro to find best length to copy...?
	memcpy(addr_erpc_wifi->sa_data, addr->data, sizeof(addr->data));

	// TODO - must be a better way to get this...?
	addr_erpc_wifi->sa_len = sizeof(addr->data);

	return err;
}

static int erpc_wifi_socket_addr_to_posix(struct sockaddr *addr,
				struct ra_erpc_sockaddr *addr_erpc_wifi)
{
	int err;

	err = erpc_wifi_socket_family_to_posix(addr_erpc_wifi->sa_family, (int *)&addr->sa_family);
	if (err) {
		LOG_ERR("unsupported family: %d", addr_erpc_wifi->sa_family);
		// TODO - better error code
		return -1;
	}

	// TODO - use MIN macro to find best length to copy...?
	memcpy(addr->data, addr_erpc_wifi->sa_data, sizeof(addr->data));

	return err;
}

static struct erpc_wifi_socket *erpc_wifi_socket_allocate(int fd)
{
	struct erpc_wifi_socket *socket = NULL;

	for (int i = 0; i < ERPC_WIFI_MAX_SOCKETS; i++) {
		if (sockets[i].in_use == false) {
			sockets[i].fd = fd;
			sockets[i].in_use = true;
			socket = &sockets[i];
			break;
		}
	}

	return socket;
}

static void erpc_wifi_socket_free(int fd)
{
	for (int i = 0; i < ERPC_WIFI_MAX_SOCKETS; i++) {
		if (sockets[i].fd == fd) {
			sockets[i].in_use = false;
			break;
		}
	}
}

static ssize_t erpc_wifi_socket_read(void *obj, void *buf, size_t sz)
{
	LOG_DBG("erpc_wifi_socket_read");

	// TODO - better error code
	return -1;
}

static ssize_t erpc_wifi_socket_write(void *obj, const void *buf, size_t sz)
{
	LOG_DBG("erpc_wifi_socket_write");

	// TODO - better error code
	return -1;
}

static int erpc_wifi_socket_close(void *obj)
{
	int ret;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("erpc_wifi_socket_close");

	erpc_wifi_socket_free(sock->fd);

	ret = ra6w1_close(sock->fd);

	LOG_DBG("ra6w1_close: %d", ret);

	return ret;
}

static int erpc_wifi_socket_ioctl(void *obj, unsigned int request, va_list args)
{
	LOG_DBG("request: %d", request);

	// TODO - better error code
	return -1;
}

static int erpc_wifi_socket_shutdown(void *obj, int how)
{
	LOG_DBG("how: %d", how);

	// TODO - better error code
	return -1;
}

static int erpc_wifi_socket_bind(void *obj, const struct sockaddr *addr, socklen_t addrlen)
{
	int ret;
	struct ra_erpc_sockaddr addr_erpc_wifi;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("fd: %d", sock->fd);

	ret = erpc_wifi_socket_addr_from_posix(addr, &addr_erpc_wifi);
	if (ret) {
		// TODO - better error code
		return -1;
	}

	// TODO - pass addrlen instead?
	ret = ra6w1_bind(sock->fd, &addr_erpc_wifi, sizeof(struct ra_erpc_sockaddr));

	LOG_DBG("ra6w1_bind: %d", ret);

	return ret;
}

static int erpc_wifi_socket_connect(void *obj, const struct sockaddr *addr,
		       socklen_t addrlen)
{
	int ret;
	struct ra_erpc_sockaddr addr_erpc_wifi;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("sa_family: %d", addr->sa_family);
	LOG_DBG("addrlen: %d", addrlen);
	LOG_DBG("fd: %d", sock->fd);

	if (addr->sa_family == AF_INET) {
		char addr_str[NET_IPV4_ADDR_LEN];
		struct sockaddr_in * s_addr;

		s_addr = net_sin(addr);

		net_addr_ntop(addr->sa_family, &s_addr->sin_addr, addr_str, sizeof(addr_str));

		LOG_DBG("sin: addr: %s port: %d", addr_str, ntohs(s_addr->sin_port));
	}

	ret = erpc_wifi_socket_addr_from_posix(addr, &addr_erpc_wifi);
	if (ret) {
		// TODO - better error code
		return -1;
	}

	ret = ra6w1_connect(sock->fd, &addr_erpc_wifi, sizeof(struct ra_erpc_sockaddr));

	LOG_DBG("ra6w1_connect: %d", ret);

	return ret;
}

static int erpc_wifi_socket_listen(void *obj, int backlog)
{
	int ret;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("fd: %d", sock->fd);
	LOG_DBG("backlog: %d", backlog);

	ret = ra6w1_listen(sock->fd, backlog);

	LOG_DBG("ra6w1_listen: %d", ret);

	return ret;
}

static int erpc_wifi_socket_accept(void *obj, struct sockaddr *addr, socklen_t *addrlen)
{
	int fd;
	int conn_fd;
	struct ra_erpc_sockaddr addr_erpc_wifi;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;
    struct erpc_wifi_socket *socket;

	LOG_DBG("fd: %d", sock->fd);

    fd = zvfs_reserve_fd();

    /* Accept returns file descriptor for new connected socket */
	conn_fd = ra6w1_accept(sock->fd, &addr_erpc_wifi, addrlen);
    LOG_DBG("ra6w1_accept: %d", conn_fd);

	if (conn_fd < 0) {
        zvfs_free_fd(fd);
		return conn_fd;
	}

	int ret = erpc_wifi_socket_addr_to_posix(addr, &addr_erpc_wifi);
    if (ret < 0) {
        zvfs_free_fd(fd);
        LOG_DBG("erpc_wifi_socket_addr_to_posix error: %d", ret);
		return ret;
	}

    /* Keep track of the new fd returned by accept */
	socket = erpc_wifi_socket_allocate(conn_fd);

    if (socket == NULL){
		zvfs_free_fd(fd);
		// TODO - better error code
		return -1;
	}

    /* Associate zephyr file descriptor with the socket obj and offload table
       ensures subsequent socket operations using zephyr fd will use the
	   appropriate RA6W1 fd */
	zvfs_finalize_typed_fd(fd, socket,
			    (const struct fd_op_vtable *)&erpc_wifi_socket_fd_op_vtable,
			    ZVFS_MODE_IFSOCK);
            
    return fd;
}

static ssize_t erpc_wifi_socket_sendto(void *obj, const void *buf, size_t len, int flags,
			  const struct sockaddr *dest_addr, socklen_t addrlen)
{
	int ret;
	struct ra_erpc_sockaddr addr_erpc_wifi;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("len: %d", len);
	LOG_DBG("fd: %d", sock->fd);

	ret = erpc_wifi_socket_addr_from_posix(dest_addr, &addr_erpc_wifi);
	if (ret) {
		// TODO - better error code
		return -1;
	}

    // TODO this is handling IPV4, need to extend for IPV6
    ret = ra6w1_sendto(sock->fd, buf, len, flags, &addr_erpc_wifi, sizeof(ra_erpc_sockaddr));

	LOG_DBG("ra6w1_sendto: %d", ret);

	return ret;
}

static ssize_t erpc_wifi_socket_recvfrom(void *obj, void *buf, size_t max_len, int flags,
			    struct sockaddr *src_addr, socklen_t *addrlen)
{
	int ret;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;
	
	LOG_DBG("fd: %d", sock->fd);
	LOG_DBG("max_len: %d", max_len);
	LOG_DBG("src_addr: %x", (uint32_t)src_addr);
	LOG_DBG("addrlen: %x", (uint32_t)addrlen);

	if (src_addr) {
		ret = ra6w1_recvfrom(sock->fd, buf, max_len, flags, (ra_erpc_sockaddr *)src_addr, addrlen);

		LOG_DBG("ra6w1_recvfrom: %d", ret);
	
		if (src_addr) {
			LOG_DBG("family: %d", src_addr->sa_family);
			LOG_DBG("address: %d.%d.%d.%d", src_addr->data[2], src_addr->data[3], src_addr->data[4], src_addr->data[5]);
			LOG_DBG("port: %d", ((struct sockaddr_in *)src_addr)->sin_port);

			if (addrlen) {	
				LOG_DBG("addrlen: %d", *addrlen);
			}

			// TODO - Temporary fix because incorrect address type is returned, probably
			// due to a problem in the service (wifi.erpc) definition of this function...
			//src_addr->sa_family = ERPC_WIFI_AF_INET;
			//erpc_wifi_socket_family_to_posix(src_addr->sa_family, &src_addr->sa_family);

			src_addr->sa_family = AF_INET;
		}
	} else {
		ret = ra6w1_recv(sock->fd, buf, max_len, flags);

		LOG_DBG("ra6w1_recvfrom: %d", ret);
	}

	return ret;
}

static int erpc_wifi_socket_getsockopt(void *obj, int level, int optname,
			  void *optval, socklen_t *optlen)
{	
    int ret;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("level: %d", level);
    LOG_DBG("optname: %d", optname);

	ret = ra6w1_getsockopt(sock->fd, level, optname, optval, optlen);

    /*
		TODO map option defines in zephyr to RA6W1 e.g. erpc_wifi_socket_option_to_posix
    	SOL_SOCKET = 0xfff on RA6W1
		SO_REUSEADDR = 4 on RA6W1
	*/

	LOG_DBG("ra6w1_getsockopt: %d", ret);

	return ret;
}

static int erpc_wifi_socket_setsockopt(void *obj, int level, int optname,
			  const void *optval, socklen_t optlen)
{	
    int ret;
	struct erpc_wifi_socket *sock = (struct erpc_wifi_socket *)obj;

	LOG_DBG("level: %d", level);
    LOG_DBG("optname: %d", level);

    /*
		TODO map option defines in zephyr to RA6W1 e.g. erpc_wifi_socket_option_to_posix
    	SOL_SOCKET = 0xfff on RA6W1
		SO_REUSEADDR = 4 on RA6W1
	*/

	ret = ra6w1_setsockopt(sock->fd, level, optname, (uint32_t *)optval, optlen);

	LOG_DBG("ra6w1_setsockopt: %d", ret);

	return ret;
}

static ssize_t erpc_wifi_socket_sendmsg(void *obj, const struct msghdr *msg, int flags)
{
	LOG_DBG("erpc_wifi_socket_sendmsg");

	// TODO - better error code
	return -1;
}

static ssize_t erpc_wifi_socket_recvmsg(void *obj, struct msghdr *msg, int flags)
{
	LOG_DBG("erpc_wifi_socket_recvmsg");

	// TODO - better error code
	return -1;
}

static int erpc_wifi_socket_getpeername(void *obj, struct sockaddr *addr,
			   socklen_t *addrlen)
{
	LOG_DBG("erpc_wifi_socket_getpeername");

	// TODO - better error code
	return -1;
}

static int erpc_wifi_socket_getsockname(void *obj, struct sockaddr *addr,
			   socklen_t *addrlen)
{
	LOG_DBG("erpc_wifi_socket_getsockname");

	// TODO - better error code
	return -1;
}

static const struct socket_op_vtable erpc_wifi_socket_fd_op_vtable = {
	.fd_vtable = {
		.read = erpc_wifi_socket_read,
		.write = erpc_wifi_socket_write,
		.close = erpc_wifi_socket_close,
		.ioctl = erpc_wifi_socket_ioctl,
	},

	.shutdown = erpc_wifi_socket_shutdown,
	.bind = erpc_wifi_socket_bind,
	.connect = erpc_wifi_socket_connect,
	.listen = erpc_wifi_socket_listen,
	.accept = erpc_wifi_socket_accept,
	.sendto = erpc_wifi_socket_sendto,
	.recvfrom = erpc_wifi_socket_recvfrom,
	.getsockopt = erpc_wifi_socket_getsockopt,
	.setsockopt = erpc_wifi_socket_setsockopt,
	.sendmsg = erpc_wifi_socket_sendmsg,
	.recvmsg = erpc_wifi_socket_recvmsg,
	.getpeername = erpc_wifi_socket_getpeername,
	.getsockname = erpc_wifi_socket_getsockname,
    .setsockopt = erpc_wifi_socket_setsockopt,
};

static int erpc_wifi_socket_create(int family, int type, int proto)
{
	int err;
	int sock;
	uint8_t family_erpc_wifi;
	int fd = zvfs_reserve_fd();
	struct erpc_wifi_socket *socket = NULL;

	LOG_DBG("erpc_wifi_socket_create");
	LOG_DBG("family: %d", family);
	LOG_DBG("type: %d", type);
	LOG_DBG("proto: %d", proto);

	/* Map Zephyr socket.h family to RA RPC's */
	err = erpc_wifi_socket_family_from_posix(family, &family_erpc_wifi);
	if (err) {
		LOG_ERR("unsupported family: %d", family);
		// TODO - better error code
		return -1;
	}

	sock = ra6w1_socket(family_erpc_wifi, type, proto);

	LOG_DBG("ra6w1_socket: %d", sock);

	if (sock < 0) {
		zvfs_free_fd(fd);
		// TODO - better error code
		return -1;
	}

	socket = erpc_wifi_socket_allocate(sock);

	if (socket == NULL){
		zvfs_free_fd(fd);
		// TODO - better error code
		return -1;
	}

	zvfs_finalize_typed_fd(fd, socket,
			    (const struct fd_op_vtable *)&erpc_wifi_socket_fd_op_vtable,
			    ZVFS_MODE_IFSOCK);

	return fd;
}

static bool erpc_wifi_socket_is_supported(int family, int type, int proto)
{
	if (family != AF_INET &&
	    family != AF_INET6) {
		return false;
	}

	if (type != SOCK_DGRAM &&
	    type != SOCK_STREAM) {
		return false;
	}

	if (proto != IPPROTO_TCP &&
	    proto != IPPROTO_UDP) {
		return false;
	}

	return true;
}

int erpc_wifi_socket_offload_init(struct net_if *iface)
{
	memset(sockets, 0, sizeof(sockets));

	net_if_socket_offload_set(iface, erpc_wifi_socket_create);

	return 0;
}

#ifdef CONFIG_NET_SOCKETS_OFFLOAD
NET_SOCKET_OFFLOAD_REGISTER(erpc_wifi, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY,
				AF_UNSPEC, erpc_wifi_socket_is_supported, erpc_wifi_socket_create);
#endif
