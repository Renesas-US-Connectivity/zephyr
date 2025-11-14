#ifndef NCP_FW_LOADER_H_
#define NCP_FW_LOADER_H_

#ifdef __cplusplus

extern "C" {
#endif

typedef enum {
    NFL_ACK,
    NFL_NACK,
    NFL_UNKNOWN
} nfl_ack_t;

int ncp_fw_loader(int mode, bool secure);

int nfl_send(const struct spi_dt_spec *spec, const void *buf, size_t len);
int nfl_read(const struct spi_dt_spec *spec, void *buf, size_t len);
nfl_ack_t nfl_read_ack(const struct spi_dt_spec *spec);
int nfl_check_spi_start(const struct gpio_dt_spec *spi_start, int timeout_ms);



/********** Second stage flsh load protocol ************/
#define ERR_PROT_NO_RESPONSE            -100    /**< timeout waiting for response */
#define ERR_PROT_CMD_REJECTED           -101    /**< NAK received when waiting for ACK */
#define ERR_PROT_INVALID_RESPONSE       -102    /**< invalid data received when waiting for ACK */
#define ERR_PROT_CRC_MISMATCH           -103    /**< CRC16 mismatch */
#define ERR_PROT_CHECKSUM_MISMATCH      -104    /**< checksum mismatch while uploading 2nd stage bootloader */
#define ERR_PROT_BOOT_LOADER_REJECTED   -105    /**< 2nd stage bootloader rejected */
#define ERR_PROT_UNKNOWN_RESPONSE       -106    /**< invalid announcement message received */
#define ERR_PROT_TRANSMISSION_ERROR     -107    /**< failed to transmit data */
#define ERR_PROT_COMMAND_ERROR          -108    /**< error executing command */
#define ERR_PROT_UNSUPPORTED_VERSION    -110    /**< unsupported version of bootloader detected */

#define ERR_PROG_QSPI_WRITE             -300    /**< QSPI write error */

#define CMD_ERASE_QSPI             0x04
#define CMD_DIRECT_WRITE_TO_QSPI   0x12

/* Typical timeout for command execution */
#define FLASH_ERASE_MASK (0x0FFF)
#define EXECUTION_TIMEOUT (5000)

/**
 * \brief Start of Header
 *
 * Sent as the beginning of each command. Second byte of the 'Hello message'.
 */
#define SOH '\x01'

/**
 * \brief Start of Text
 *
 * Sent at the beginning 'Hello message'.
 */
#define STX '\x02'

/**
 * \brief Acknowledge
 *
 * Sent by uartboot or the host application when the received command/data is valid.
 */
#define ACK '\x06'

/**
 * \brief Negative Acknowledge
 *
 * Sent by uartboot or the host application when the received command/data is not valid.
 */
#define NAK '\x15'

struct write_buf {
    const void *buf;
    size_t len;
};

int prog_erase_qspi(const struct spi_dt_spec *spec, uint32_t address, size_t size);
int prog_write_to_qspi(const struct spi_dt_spec *spec,
                       uint32_t flash_address, const uint8_t *buf, uint32_t size);
/******************** END *********************/

#ifdef __cplusplus
}
#endif

#endif /* NCP_FW_LOADER_H_ */