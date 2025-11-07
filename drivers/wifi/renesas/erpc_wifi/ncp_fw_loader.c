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

LOG_MODULE_REGISTER(renesas_erpc_wifi_spi, CONFIG_WIFI_LOG_LEVEL);

#include "ncp_fw_loader.h"
#include "ncp_fw_loader_binaries.h"
#include "crc32.h"

#define SPI_OP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB) // Example: 8-bit word, MSB first
#define BOOT_PROTOCOL_SIZE (12)  // preamble[2], spi_mode, header crc, length[4], image_crc[4] = 12 byte

static char boot_preamble[BOOT_PROTOCOL_SIZE] = { 
	0x70, 0x50, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x60,
	0x00, 0x40, 0x08, 0x00
};

static struct spi_buf tx_buf = {0};
static struct spi_buf_set tx_bufs = {
	.buffers = &tx_buf,
	.count = 1,
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

bool ncp_fw_loader(int mode, bool secure)
{
    int err = 0, i;
	char *pbody = NULL, *pshort_body = NULL, *pword_body = NULL;
	unsigned char crc = 0xff, pin = 0; //mode = 8,
	unsigned int fsize;
	unsigned int image_crc32;
	unsigned int payload_length;
	const struct spi_dt_spec spi_slave_spec =
			SPI_DT_SPEC_GET(DT_NODELABEL(qciot_rrq61051evz), SPI_OP, 0);
	const struct gpio_dt_spec spi_start =
			GPIO_DT_SPEC_GET_OR(DT_DRV_INST(0), int_gpios, {0});

	LOG_INF("Starting NCP firmware loader");
	if (!spi_is_ready_dt(&spi_slave_spec)) {
		LOG_ERR("SPI device not ready");
		return -EIO;
	}

	if (!gpio_is_ready_dt(&spi_start)) {
		LOG_ERR("SPI start GPIO device not ready");
		return -EIO;
	}

	/* Configure the pin as an input, with  */
	err = gpio_pin_configure_dt(&spi_start, (GPIO_INPUT));
	if (err != 0) {
		LOG_ERR("Failed to configure SPI start pin");
		return -EIO;
	}

	LOG_INF("SPI device ready: %s", spi_slave_spec.bus->name);

	/* Fill the required fields in the boot preamble */
	if (secure == true) {
    	pbody = secure_binary;
        fsize = sizeof(secure_binary);
    } else {
    	pbody = binary;
        fsize = sizeof(binary);
    }

    image_crc32 = crc32(pbody, fsize);
    memcpy(&boot_preamble[6], &image_crc32, sizeof(unsigned int));

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
    for (i = 0; i < BOOT_PROTOCOL_SIZE; i++)
        LOG_INF("[%02x] ", (unsigned char)boot_preamble[i]);

	/* Reset the NCP device via reset pin */
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	struct gpio_dt_spec wifi_reset = GPIO_DT_SPEC_GET(DT_DRV_INST(0), reset_gpios);

	if (!gpio_is_ready_dt(&wifi_reset)) {
		LOG_ERR("Error: failed to configure wifi_reset %s pin %d", wifi_reset.port->name,
			wifi_reset.pin);
		return -EIO;
	}

	LOG_INF("RSTn pin ready: %s pin %d", wifi_reset.port->name, wifi_reset.pin);

	/* Set wifi_reset as output and activate reset */
	err = gpio_pin_configure_dt(&wifi_reset, GPIO_OUTPUT_ACTIVE);
	if (err) {
		LOG_ERR("Error %d: failed to configure wifi_reset %s pin %d", err,
			wifi_reset.port->name, wifi_reset.pin);
		return err;
	}

	LOG_INF("RSTn pin set to active for: %d ms",
			DT_INST_PROP_OR(0, reset_assert_duration_ms, 0));
	k_sleep(K_MSEC(DT_INST_PROP_OR(0, reset_assert_duration_ms, 0)));

	/* Release the device from reset */
	err = gpio_pin_configure_dt(&wifi_reset, GPIO_OUTPUT_INACTIVE);
	if (err) {
		return err;
	}

	LOG_INF("RSTn pin set to inactive");
#endif /* DT_INST_NODE_HAS_PROP(0, reset_gpios) */

    /* Wait until start high */
    for (i = 0; i < 50; i++) {
    	int spi_start_input =  gpio_pin_get_dt(&spi_start);
    	if (spi_start_input == 1) {
			LOG_INF("NCP responded, proceeding with firmware load");
    		break;
    	} else {
    		k_sleep(K_MSEC(100));
		}
    }

    if (i == 50) {
    	LOG_ERR("Start time out \n");
    	return -ETIMEDOUT;
    }

	/* Prepare the transmit buffer */
	tx_buf.buf = boot_preamble;
	tx_buf.len = BOOT_PROTOCOL_SIZE;

	/* Send the boot preamble */
    err = spi_write_dt(&spi_slave_spec, &tx_bufs);
    if (err) {
        LOG_ERR("Failed to send boot preamble");
        return err;
    }

	LOG_INF("SUCCESS: Sending boot preamble to device over SPI");
	k_sleep(K_MSEC(10));

    uint8_t rx_buffer_ack[4]; // Buffer to store received data

    // 1. Prepare the receive buffer set
    struct spi_buf rx_spi_buf[1];
    rx_spi_buf[0].buf = rx_buffer_ack;
    rx_spi_buf[0].len = sizeof(rx_buffer_ack);

    const struct spi_buf_set rx_bufs = {
        .buffers = rx_spi_buf,
        .count = 1
    };

	/* 3. Perform the SPI read operation */
    err = spi_read_dt(&spi_slave_spec, &rx_bufs);
    if (err == 0) {
        LOG_INF("SPI read successful. Received data: ");
        for (i = 0; i < sizeof(rx_buffer_ack); i++) {
            LOG_INF("0x%02X ", rx_buffer_ack[i]);
        }
    } else {
        LOG_ERR("SPI read failed with error: %d\n", err);
    }

	if (*((int *)&rx_buffer_ack) == 0x00022000) {
    	LOG_INF("ACK\n");
    } else if (*((int *)&rx_buffer_ack) == 0x20000002) {
    	LOG_INF("NACK\n");
    } else {
    	LOG_INF("Unknown error \n");
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
		tx_buf.buf = payload + t_size;
		tx_buf.len = chunk_size;

		/* Send the firmware payload in chunks */
		err = spi_write_dt(&spi_slave_spec, &tx_bufs);
		if (err) {
			LOG_ERR("Failed to send firmware payload chunk at offset %d", t_size);
			return err;
		}

		LOG_INF("Sent chunk at offset %d, size %d, total %d", t_size, chunk_size, t_size + chunk_size);
		t_size += chunk_size;
	}

	/* Send the firmware payload in chunks */
	err = spi_write_dt(&spi_slave_spec, &tx_bufs);
	if (err) {
		LOG_ERR("Failed to send firmware payload chunk %d", i);
		return err;
	}


	LOG_INF("SUCCESS: Sending firmware payload to device over SPI");
	if (mode == 16) {
		free(pshort_body);
	} else if (mode == 32) {
		free(pword_body);
	}

	k_sleep(K_MSEC(5));

	memset(rx_buffer_ack, 0, sizeof(rx_buffer_ack));
	i = 0;
	while (i < 10) {
		err = spi_read_dt(&spi_slave_spec, &rx_bufs);
		if (err == 0) {
			LOG_INF("SPI read[%d] successful. Received data: ", i);
			for (int j = 0; j < sizeof(rx_buffer_ack); j++) {
				LOG_INF("0x%02X ", rx_buffer_ack[j]);
			}
		} else {
			LOG_ERR("SPI read[%d] failed with error: %d\n", i, err);
		}

		if (*((int *)&rx_buffer_ack) == 0x00022000) {
			LOG_INF("ACK\n");
			break;
		} else if (*((int *)&rx_buffer_ack) == 0x20000002) {
			LOG_INF("NACK\n");
			break;
		}

		i++;
		k_sleep(K_MSEC(10));
	}

	if (i == 10) {
		LOG_INF("Ack time out \n");
		return -ETIMEDOUT;
	}

	LOG_INF("NCP firmware loader completed");

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

    return err;
}
