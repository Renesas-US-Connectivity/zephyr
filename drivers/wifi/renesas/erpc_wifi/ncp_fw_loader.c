/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_erpc_wifi_spi

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(ncp_fw_loader, CONFIG_WIFI_LOG_LEVEL);

#include "ncp_fw_loader.h"
#include "ncp_fw_loader_binaries_v2.h"
#include "crc32.h"

#define SPI_OP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB) // Example: 8-bit word, MSB first
#define BOOT_PROTOCOL_SIZE (12)  // preamble[2], spi_mode, header crc, length[4], image_crc[4] = 12 byte

static unsigned char _secure_binary[] = {0};

#define SECURE_BINARY (_secure_binary)
#define BINARY (__sdk_bsp_system_loaders_uartboot_RRQ61000_dubug_uartboot_bin)

static char boot_preamble[BOOT_PROTOCOL_SIZE] = { 
	0x70, 0x50, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x60,
	0x00, 0x40, 0x08, 0x00
};

static void swap_data(char mode, char *body, int length, char *target)
{
    unsigned short *pshort ,*pshort_target;
    unsigned int *pword, *pword_target;
    int i;

    if (mode == 16) {
        pshort = (unsigned short*)body;
        pshort_target = (unsigned short*)target;
        for (i = 0; i < length / 2; i++) {
            pshort_target[i] = ((pshort[i] & 0xFF00) >> 8) | ((pshort[i] & 0xff) << 8);
        }
    } else if (mode == 32) {
        pword = (unsigned int *)body;
        pword_target = (unsigned int*)target;
        for (i = 0; i < length / 4; i++) {
            pword_target[i] = ((pword[i]&0xff000000) >> 24) | ((pword[i] & 0x00ff0000) >> 8) | ((pword[i] & 0x0000ff00) << 8) | ((pword[i] & 0x000000ff) << 24);
        }
    }
}

static int ncp_fw_loader_stage1(const struct spi_dt_spec *spi_slave_spec,
	const struct gpio_dt_spec *spi_start,
	const struct gpio_dt_spec *wifi_reset,
	int mode, bool secure)
{
    int err = 0, i;
	char *pbody = NULL, *pshort_body = NULL, *pword_body = NULL;
	unsigned char crc = 0xff;
	unsigned int fsize;
	unsigned int image_crc32;
	unsigned int payload_length;
	
	if (!gpio_is_ready_dt(wifi_reset)) {
		LOG_ERR("Error: failed to configure wifi_reset %s pin %d", wifi_reset->port->name,
			wifi_reset->pin);
		return -EIO;
	}

	/* Configure the pin as an input, with  */
	err = gpio_pin_configure_dt(spi_start, (GPIO_INPUT));
	if (err != 0) {
		LOG_ERR("Failed to configure SPI start pin");
		return -EIO;
	}

	LOG_INF("SPI device ready: %s", spi_slave_spec->bus->name);

	/* Fill the required fields in the boot preamble */
	if (secure == true) {
    	pbody = SECURE_BINARY;
        fsize = sizeof(SECURE_BINARY);
    } else {
    	pbody = BINARY;
        fsize = sizeof(BINARY);
    }

    image_crc32 = crc32(pbody, fsize);
    memcpy(&boot_preamble[6], &image_crc32, sizeof(unsigned int));
	LOG_INF("Firmware image size : %d bytes", fsize);
	LOG_INF("Firmware image crc32: 0x%08x", image_crc32);

    boot_preamble[10] = mode;
    if (mode == 16){
        pshort_body = malloc(fsize);
        swap_data(16, pbody, fsize, pshort_body);
    } else if (mode == 32){
        pword_body = malloc(fsize);
        swap_data(32, pbody, fsize, pword_body);
    }

	payload_length = fsize;
    memcpy(&boot_preamble[2], &payload_length, sizeof(unsigned int));

	for (i = 0; i < 11; i++)
		crc ^= boot_preamble[i];

    boot_preamble[11] = crc;

	/* Print the boot preamble */
    for (i = 0; i < BOOT_PROTOCOL_SIZE; i++) {
		LOG_INF("[%02x] ", (unsigned char)boot_preamble[i]);
	}

	/* Reset the NCP device via reset pin */
	LOG_INF("RSTn pin ready: %s pin %d", wifi_reset->port->name, wifi_reset->pin);

	/* Set wifi_reset as output and activate reset */
	err = gpio_pin_configure_dt(wifi_reset, GPIO_OUTPUT_ACTIVE);
	if (err) {
		LOG_ERR("Error %d: failed to configure wifi_reset %s pin %d", err,
			wifi_reset->port->name, wifi_reset->pin);
		return -EIO;
	}

	LOG_INF("RSTn pin set to active for: %d ms",
			DT_INST_PROP_OR(0, reset_assert_duration_ms, 0));
	k_sleep(K_MSEC(DT_INST_PROP_OR(0, reset_assert_duration_ms, 0)));

	/* Release the device from reset */
	err = gpio_pin_configure_dt(wifi_reset, GPIO_OUTPUT_INACTIVE);
	if (err) {
		return -EIO;
	}

	LOG_INF("RSTn pin set to inactive");

	/* Wait until start high */
	if (nfl_check_spi_start(spi_start, 1000) != 0) {
		LOG_ERR("Start time out \n");
		return -ETIMEDOUT;
	}

	/* Send the boot preamble */
    err = nfl_send(spi_slave_spec, boot_preamble, BOOT_PROTOCOL_SIZE);
    if (err) {
        LOG_ERR("Failed to send boot preamble");
        return -EIO;
    }

	LOG_INF("SUCCESS: Sending boot preamble to device over SPI");
	k_sleep(K_MSEC(10));

	/* 3. Perform the SPI read operation */
	if (nfl_read_ack(spi_slave_spec) != NFL_ACK) {
		LOG_ERR("Firmware payload not acknowledged by NCP");
		return -EBUSY;
	}

	k_sleep(K_MSEC(1));

	/* Prepare the transmit buffer */
	char *payload = NULL;
	if (mode == 8) {
		payload = pbody;
	} else if (mode == 16) {
		payload = pshort_body;
	} else if (mode == 32) {
		payload = pword_body;
	}

#define MAX_CHUNK_SIZE 1024
	int t_size = 0;
	while (t_size < fsize) {
		int chunk_size = (fsize - t_size) > MAX_CHUNK_SIZE ? MAX_CHUNK_SIZE : (fsize - t_size);

		/* Send the firmware payload in chunks */
		err = nfl_send(spi_slave_spec, payload + t_size, chunk_size);
		if (err) {
			LOG_ERR("Failed to send firmware payload chunk at offset %d", t_size);
			return -EIO;
		}

		LOG_INF("Sent chunk at offset %d, size %d, total %d", t_size, chunk_size, t_size + chunk_size);
		t_size += chunk_size;
	}

	LOG_INF("SUCCESS: Sending firmware payload to device over SPI");
	if (mode == 16) {
		free(pshort_body);
	} else if (mode == 32) {
		free(pword_body);
	}

	if (nfl_read_ack(spi_slave_spec) != NFL_ACK) {
		LOG_ERR("Firmware payload not acknowledged by NCP");
		return -EBUSY;
	}

	LOG_INF("NCP firmware loader completed");
	LOG_INF("=============================");

	/* Fw test */
	LOG_INF("TEST: Starting NCP firmware test : sending boot preamble again");

		/* Wait until start high */
	if (nfl_check_spi_start(spi_start, 5000) != 0) {
		LOG_ERR("Start time out \n");
		return -ETIMEDOUT;
	}

	/* Wait until start high */
	LOG_INF("TEST: NCP responded, proceeding with firmware load");

	/* Send the boot preamble */
    err = nfl_send(spi_slave_spec, boot_preamble, BOOT_PROTOCOL_SIZE);
    if (err) {
        LOG_ERR("Failed to send boot preamble");
        return -EIO;
    }

	LOG_INF("TEST: SUCCESS: Sending boot preamble to device over SPI");
	k_sleep(K_MSEC(5));

	if (nfl_read_ack(spi_slave_spec) != NFL_ACK) {
		LOG_ERR("Firmware payload not acknowledged by NCP");
		return -EBUSY;
	}

    return 0;
}


/*
 * a complete flow for transmission handling (including in/out data) is as follows:
 *
 * <= <STX> <SOH> (ver1) (ver2)
 * => <SOH>
 * => (type) (len1) (len2)
 * call HOP_INIT
 * <= <ACK> / <NAK>
 * if len > 0
 *      => (data...)
 *      call HOP_DATA
 *      <= <ACK> / <NAK>
 *      <= (crc1) (crc2)
 *      => <ACK> / <NAK>
 * call HOP_EXEC
 * <= <ACK> / <NAK>
 * call HOP_SEND_LEN
 * if len > 0
 *      <= (len1) (len2)
 *      => <ACK> / <NAK>
 *      call HOP_SEND_DATA
 *      <= (data...)
 *      => (crc1) (crc2)
 *      <= <ACK> / <NAK>
 *
 * If NAK has been sent at some step, next steps shouldn't be performed.
 */
static int ncp_fw_loader_stage2(const struct spi_dt_spec *spi_slave_spec,
	const struct gpio_dt_spec *spi_start,
	const struct gpio_dt_spec *wifi_reset)
{

	LOG_INF("====================================");
	LOG_INF("Starting NCP firmware loader stage 2");
	LOG_INF("====================================");

	/* Wait until start high */
	if (nfl_check_spi_start(spi_start, 5000) != 0) {
		LOG_ERR("Start time out \n");
		return -ETIMEDOUT;
	}

	/* Erase QSPI flash at address=0x000000, size=0x800000 */
	protocol_cmd_erase_qspi(spi_slave_spec, 0x000000, 0x800000);

   /* We can either wait a fixed amount of time for the RA6Wx device to
	* finish booting or we can wait for it to send us the reset complete
	event. At present, the SPI interface does not support sending the
	reset complete event so we have to just wait for boot to complete. */
#if DT_INST_NODE_HAS_PROP(0, boot_duration_ms)
	LOG_INF("Waiting %d ms for device boot", DT_INST_PROP_OR(0, boot_duration_ms, 0));
	k_sleep(K_MSEC(DT_INST_PROP_OR(0, boot_duration_ms, 0)));
#else
	/*
		* Default wait time for RA6W1x device to boot up
		* and be ready to accept NCP firmware.
		*/
	k_sleep(K_MSEC(200));
#endif /* DT_INST_NODE_HAS_PROP(0, boot_duration_ms) */

	return 0;
}

const struct spi_dt_spec spi_slave_spec = SPI_DT_SPEC_GET(DT_NODELABEL(qciot_rrq61051evz), SPI_OP, 0);
const struct gpio_dt_spec spi_start = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(0), int_gpios, {0});
const struct gpio_dt_spec wifi_reset = GPIO_DT_SPEC_GET(DT_DRV_INST(0), reset_gpios);

int ncp_fw_loader(int mode, bool secure)
{
	int err;

	//spi_slave_spec = SPI_DT_SPEC_GET(DT_NODELABEL(qciot_rrq61051evz), SPI_OP, 0);
	//spi_start = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(0), int_gpios, {0});
	//wifi_reset = GPIO_DT_SPEC_GET(DT_DRV_INST(0), reset_gpios);

	LOG_INF("Starting NCP firmware loader");
	if (!spi_is_ready_dt(&spi_slave_spec)) {
		LOG_ERR("SPI device not ready");
		return -EIO;
	}

	if (!gpio_is_ready_dt(&spi_start)) {
		LOG_ERR("SPI start GPIO device not ready");
		return -EIO;
	}

	if (!gpio_is_ready_dt(&wifi_reset)) {
		LOG_ERR("Error: failed to configure wifi_reset %s pin %d", wifi_reset.port->name,
			wifi_reset.pin);
		return -EIO;
	}

	err = ncp_fw_loader_stage1(&spi_slave_spec, &spi_start, &wifi_reset, mode, secure);
	if (err) {
		LOG_ERR("NCP firmware loader stage 1 failed with error: %d", err);
		return -EBUSY;
	}

	err = ncp_fw_loader_stage2(&spi_slave_spec, &spi_start, &wifi_reset);
	if (err) {
		LOG_ERR("NCP firmware loader stage 2 failed with error: %d", err);
		return -EBUSY;
	}

	return 0;
}