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

#include "ncp_fw_loader.h"
#include "crc16.h"

LOG_MODULE_REGISTER(ncp_fw_loader_spi, CONFIG_WIFI_LOG_LEVEL);

static struct spi_buf tx_buf = {0};
static struct spi_buf_set tx_bufs = {
	.buffers = &tx_buf,
	.count = 1,
};

extern struct gpio_dt_spec spi_start;

int nfl_send(const struct spi_dt_spec *spec, const void *buf, size_t len)
{
    int err = 0;

	/* Prepare the transmit buffer */
	tx_buf.buf = buf;
	tx_buf.len = len;

	/* Send the boot preamble */
    err = spi_write_dt(spec, &tx_bufs);
    if (err) {
        LOG_ERR("SPI write failed with error: %d", err);
        return err;
    }

    return err;
}

int nfl_read(const struct spi_dt_spec *spec, void *buf, size_t len)
{
    int err = 0;

    // 1. Prepare the receive buffer set
    struct spi_buf rx_spi_buf[1];
    rx_spi_buf[0].buf = buf;
    rx_spi_buf[0].len = len;

    const struct spi_buf_set rx_bufs = {
        .buffers = rx_spi_buf,
        .count = 1
    };

    /* 3. Perform the SPI read operation */
    err = spi_read_dt(spec, &rx_bufs);
    if (err) {
        LOG_ERR("SPI read failed with error: %d", err);
    }

    return err;
}

#define PTROT_1_ACK (0x00022000)
#define PTROT_1_NACK (0x20000002)

nfl_ack_t nfl_read_ack(const struct spi_dt_spec *spec)
{
    int i = 0;
    uint8_t rx_buffer_ack[4]; // Buffer to store received data

    memset(rx_buffer_ack, 0, sizeof(rx_buffer_ack));
	while (i < 10) {
		nfl_read(spec, rx_buffer_ack, sizeof(rx_buffer_ack));
		if (*((int *)&rx_buffer_ack) == PTROT_1_ACK) {
            LOG_INF("SPI read[%d] ACK: ", i);
			return NFL_ACK;
		} else if (*((int *)&rx_buffer_ack) == PTROT_1_NACK) {
            LOG_INF("SPI read[%d] NACK: ", i);
			return NFL_NACK;
		}

		k_sleep(K_MSEC(10));
        i++;
	}

	if (i == 10) {
		LOG_INF("Ack time out \n");
	}

    return NFL_UNKNOWN;
}

#define SPI_START_TIMEOUT_INTERVAL_MS 10
int nfl_check_spi_start(const struct gpio_dt_spec *spec, int timeout_ms)
{
    int err = 0, i;
    int max_attempts = timeout_ms / SPI_START_TIMEOUT_INTERVAL_MS;

    /* Wait until start high */
    for (i = 0; i < max_attempts; i++) {
        int spi_start_input =  gpio_pin_get_dt(spec);
        if (spi_start_input == 1) {
            LOG_INF("NCP: SPI_START Asserted");
            return 0;
        } else {
            k_sleep(K_MSEC(SPI_START_TIMEOUT_INTERVAL_MS));
        }
    }

    if (i == max_attempts) {
        LOG_ERR("Start time out \n");
        err = -ETIMEDOUT;
    }

    return err;
}

/********************* second stage protocols *********************/
#define PROT2_TIMEOUT_INTERVAL_MS 10

static int spi_write_char(const struct spi_dt_spec *spec, uint8_t c)
{
    uint8_t buf[1] = {c};

    return nfl_send(spec, buf, sizeof(buf));
}

static int wait_for_ack(const struct spi_dt_spec *spec, size_t timeout_ms)
{
    int i = 0;
    uint8_t rx_buffer[1]; // Buffer to store received data
    int max_attempts = timeout_ms / PROT2_TIMEOUT_INTERVAL_MS;

    LOG_INF("PROT2: Waiting for ACK with timeout %d ms", timeout_ms);
    memset(rx_buffer, 0, sizeof(rx_buffer));
	while (i < max_attempts) {
		nfl_read(spec, rx_buffer, sizeof(rx_buffer));
        if (rx_buffer[0] == ACK) {
            LOG_INF("PROT2: ACK received");
            return 0;
        } else if (rx_buffer[0] == NAK) {
            LOG_INF("PROT2: NAK received");
            return ERR_PROT_CMD_REJECTED;
        } else if (rx_buffer[0]) {
            LOG_INF("PROT2: Invalid ACK response");
            return ERR_PROT_INVALID_RESPONSE;
        }

		k_sleep(K_MSEC(PROT2_TIMEOUT_INTERVAL_MS));
        i++;
	}

	if (i == max_attempts) {
		LOG_INF("PROT2: ACK time out");
	}

    return ERR_PROT_NO_RESPONSE;
}

static int send_cmd_header(const struct spi_dt_spec *spec, uint8_t type, uint16_t len)
{
    int err;
    uint8_t buf[4];

    buf[0] = SOH;
    buf[1] = type;
    buf[2] = (uint8_t) len;
    buf[3] = (uint8_t) (len >> 8);

    // if (serial_write(buf, sizeof(buf)) < 0) {
    //         return ERR_PROT_TRANSMISSION_ERROR;
    // }
    err = nfl_send(spec, buf, sizeof(buf));
    if (err) {
        LOG_ERR("PROT2: Failed to send command header");
        return -EIO;
    }

    return wait_for_ack(spec, 300);
}

static int send_cmd_data(const struct spi_dt_spec *spec,
    const struct write_buf *wb, int cnt)
{
    int i, ret = 0;
    uint16_t crc, crc_r;

    crc16_init(&crc);

    for (i = 0; i < cnt; i++) {
        const struct write_buf *b = &wb[i];

        crc16_update(&crc, b->buf, b->len);

        // ret = serial_write(b->buf, b->len);
        ret = nfl_send(spec, b->buf, b->len);
        if (ret) {
            LOG_ERR("PROT2: Failed to send data chunk %d", i);
            goto done;
        }
    }

    LOG_INF("PROT2: Data sent, waiting for ACK");
    ret = wait_for_ack(spec, EXECUTION_TIMEOUT);
    if (ret < 0) {
        goto done;
    }

    /* don't care if read is successfull, CRC won't simply match on error */
    // crc_r = serial_read_char(30);
    // crc_r |= serial_read_char(30) << 8;
    uint8_t crc_buf[2];
    LOG_INF("PROT2: Reading CRC16 from NCP");

    /* Wait until start high */
	if (nfl_check_spi_start(&spi_start, 1000) != 0) {
		LOG_ERR("Start time out \n");
		return -ETIMEDOUT;
	}

    LOG_INF("PROT2: SPI read CRC16");
    ret = nfl_read(spec, crc_buf, sizeof(crc_buf));
    if (ret) {
        LOG_ERR("PROT2: Failed to read CRC16");
        goto done;
    }

    crc_r = crc_buf[0];
    crc_r |= crc_buf[1] << 8;

    if (crc_r != crc) {
            // (void) serial_write_char(NAK); // return value does not matter here
            LOG_ERR("PROT2: CRC16 mismatch (calculated: 0x%04X, received: 0x%04X)", crc, crc_r);
            spi_write_char(spec, NAK);
            ret = ERR_PROT_CRC_MISMATCH;
    } else {
        LOG_INF("PROT2: CRC16 match (0x%04X) - SEND ACK", crc);
        // ret = serial_write_char(ACK);
        ret = spi_write_char(spec, ACK);
        if (ret) {
            /* overwrite ret since serial_write_char returns value from write() call */
            ret = ERR_PROT_TRANSMISSION_ERROR;
        } else {
            LOG_INF("PROT2: CRC HOST-ACK sent");
            ret = 0; // indicate success
        }
    }

done:
    return ret;
}

int protocol_cmd_erase_qspi(const struct spi_dt_spec *spec, uint32_t address, size_t size)
{
    uint8_t header_buf[8];
    struct write_buf wb[1];
    int err;

    LOG_INF("==========================");
    LOG_INF("Running QSPI erase command");
	LOG_INF("==========================");

    err = send_cmd_header(spec, CMD_ERASE_QSPI, sizeof(header_buf));
    if (err < 0) {
        LOG_ERR("PROT2: Failed to send command header: No ACK");
        return err;
    }

    LOG_INF("PROT2: QSPI erase address: 0x%06X, size: 0x%06X", address, size);
    header_buf[0] = (uint8_t) (address);
    header_buf[1] = (uint8_t) (address >> 8);
    header_buf[2] = (uint8_t) (address >> 16);
    header_buf[3] = (uint8_t) (address >> 24);
    header_buf[4] = (uint8_t) (size);
    header_buf[5] = (uint8_t) (size >> 8);
    header_buf[6] = (uint8_t) (size >> 16);
    header_buf[7] = (uint8_t) (size >> 24);

    wb[0].buf = header_buf;
    wb[0].len = sizeof(header_buf);

    LOG_INF("PROT2: Sending QSPI erase command data");
    err = send_cmd_data(spec, wb, 1);
    if (err) {
        LOG_ERR("PROT2: send_cmd_data failed with error: %d", err);
        return err;
    }

    /* Wait until start high - avoid busy polling */
	if (nfl_check_spi_start(&spi_start, size) != 0) {
		LOG_ERR("Start time out \n");
		return -ETIMEDOUT;
	}

    /* each sector erase time could be taken 200ms in max */
    err = wait_for_ack(spec, 100);
    LOG_INF("PROT2: QSPI erase command completed with status: %d", err);

    return err;
}