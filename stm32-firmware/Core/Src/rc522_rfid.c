/******************************************************************************
 * @file    rc522_rfid.c
 * @brief   Driver implementation for RC522 RFID module using SPI communication.
 ******************************************************************************/

#include "rc522_rfid.h"
#include "string.h"
#include <stdio.h>

/******************************************************************************
 * @brief  Static pointers to SPI and GPIO configuration
 ******************************************************************************/
static SPI_HandleTypeDef *spi = NULL;
static GPIO_TypeDef *cs_port = NULL;
static uint16_t cs_pin = 0;
static GPIO_TypeDef *reset_port = NULL;
static uint16_t reset_pin = 0;

/******************************************************************************
 * @function rfid_configure
 * @brief    Configure SPI and GPIO pins for RC522
 * @param    s         SPI handle
 * @param    cs_po     GPIO port for chip select
 * @param    cs_pi     GPIO pin for chip select
 * @param    reset_po  GPIO port for reset
 * @param    reset_pi  GPIO pin for reset
 ******************************************************************************/
void rfid_configure(SPI_HandleTypeDef *s, GPIO_TypeDef *cs_po, uint16_t cs_pi, GPIO_TypeDef *reset_po, uint16_t reset_pi) {
	spi = s;
	cs_port = cs_po;
	cs_pin = cs_pi;
	reset_port = reset_po;
	reset_pin = reset_pi;
}

/******************************************************************************
 * @function rfid_init
 * @brief    Initializes the RC522 RFID module with default configurations.
 ******************************************************************************/
void rfid_init() {
    rfid_cs_write(GPIO_PIN_SET);
    rfid_reset();

    rfid_write_register(REG_T_MODE, 0x8D);  // TAuto=1, TPrescaler_Hi=0x8 (upper 4 bits of 12-bit TPrescaler)
    rfid_write_register(REG_T_PRESCALER, 0xA9);  // TPrescaler_Lo=0xA9, full TPrescaler=0x8A9=2217
    // Timer frequency ftimer = 13.56 MHz / (2 × 2217 + 1) ≈ 3057.2 Hz (if TPrescalEven = 0, default)

    rfid_set_gain(0xff);  // Set maximum gain
    rfid_write_register(REG_RX_MODE, 0x00);     // RX CRC is disabled
    rfid_write_register(REG_MOD_WIDTH, 0x26);

    rfid_write_register(REG_TX_MODE, 0x80);		// TX CRC is enabled
    rfid_write_register(REG_T_RELOAD_H, 0x03);  // Timer reload value high byte
    rfid_write_register(REG_T_RELOAD_L, 0xE8);  // Timer reload value low byte (Total reload = 0x03E8 = 1000)

    rfid_write_register(REG_TX_ASK, 0x40);
    rfid_write_register(REG_MODE, 0x3D);

    rfid_antenna_on();
}


/******************************************************************************
 * @function rfid_self_test
 * @brief    Runs the built-in self test routine and prints results via UART.
 ******************************************************************************/
void rfid_self_test() {
	rfid_reset();
	rfid_set_bit_mask(REG_FIFO_LEVEL, 0x80);
	for (int i = 0; i < 25; i++) {
		rfid_write_register(REG_FIFO_DATA, 0x00);
	}

	rfid_write_register(REG_COMMAND, COM_MEM);
	rfid_set_bit_mask(REG_FIFO_LEVEL, 0x80);
	rfid_write_register(REG_AUTO_TEST, 0x09);
	rfid_write_register(REG_FIFO_DATA, 0x00);
	rfid_write_register(REG_COMMAND, CMD_CALCCRC);
	HAL_Delay(100);

	int n = rfid_read_register(REG_FIFO_LEVEL);
	printf("Test result (%d bytes):\r\n", n);

	for (int i = 0; i < n; ++i) {
		printf("%02x ", rfid_read_register(REG_FIFO_DATA));
	}
	printf("\n\r");
}

/******************************************************************************
 * @function rfid_set_bit_mask
 * @brief    Sets specified bits in a register.
 * @param    reg    Register address
 * @param    mask   Bit mask
 ******************************************************************************/
void rfid_set_bit_mask(uint8_t reg, uint8_t mask) {
	rfid_write_register(reg, rfid_read_register(reg) | mask);
}

/******************************************************************************
 * @function rfid_clear_bit_mask
 * @brief    Clears specified bits in a register.
 * @param    reg    Register address
 * @param    mask   Bit mask
 ******************************************************************************/
void rfid_clear_bit_mask(uint8_t reg, uint8_t mask) {
	rfid_write_register(reg, rfid_read_register(reg) & (~mask));
}

/******************************************************************************
 * @function rfid_antenna_on
 * @brief    Enables the RFID antenna.
 ******************************************************************************/
void rfid_antenna_on(void) {
	uint8_t temp = rfid_read_register(REG_TX_CONTROL);
	if (!(temp & 0x03)) {
		rfid_set_bit_mask(REG_TX_CONTROL, temp | 0x03);
	}
}

/******************************************************************************
 * @function rfid_cs_write
 * @brief    Controls the chip select pin.
 * @param    val    GPIO_PIN_SET or GPIO_PIN_RESET
 ******************************************************************************/
void rfid_cs_write(uint8_t val) {
	HAL_GPIO_WritePin(cs_port, cs_pin, val);
}

/******************************************************************************
 * @function rfid_read_register
 * @brief    Reads one byte from a register.
 * @param    addr   Register address
 * @return   Byte read from the register
 ******************************************************************************/
uint8_t rfid_read_register(uint8_t addr) {
	uint8_t val = 0x00;
	rfid_cs_write(GPIO_PIN_RESET);
	addr = (addr << 1) | 0x80;
	HAL_SPI_Transmit(spi, &addr, 1, 1000);
	uint8_t dummy = 0x00;
	HAL_SPI_TransmitReceive(spi, &dummy, &val, 1, 1000);
	rfid_cs_write(GPIO_PIN_SET);
	return val;
}

/******************************************************************************
 * @function rfid_read_register_many
 * @brief    Reads multiple bytes from a register.
 * @param    addr       Register address
 * @param    count      Number of bytes to read
 * @param    tab        Pointer to data buffer
 * @param    rx_align   Bit alignment
 ******************************************************************************/
void rfid_read_register_many(uint8_t addr, uint8_t count, uint8_t *tab, uint8_t rx_align) {
	if (count == 0) return;
	uint8_t index = 0;
	if (rx_align != 0) {
		uint8_t mask = (0xFF << rx_align) & 0xFF;
		uint8_t value = rfid_read_register(addr);
		tab[0] = (tab[0] & ~mask) | (value & mask);
		index++;
	}
	for (; index < count; ++index) {
		tab[index] = rfid_read_register(addr);
	}
}

/******************************************************************************
 * @function rfid_write_register
 * @brief    Writes one byte to a register.
 * @param    addr   Register address
 * @param    val    Value to write
 ******************************************************************************/
void rfid_write_register(uint8_t addr, uint8_t val) {
	rfid_cs_write(GPIO_PIN_RESET);
	addr = (addr << 1) & 0x7E;
	HAL_SPI_Transmit(spi, &addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(spi, &val, 1, HAL_MAX_DELAY);
	rfid_cs_write(GPIO_PIN_SET);
}

/******************************************************************************
 * @function rfid_reset
 * @brief    Performs a soft reset on the RFID module.
 ******************************************************************************/
void rfid_reset() {
	rfid_write_register(REG_COMMAND, COM_SOFT_RESET);
	uint8_t count = 0;
	do {
		HAL_Delay(50);
	} while ((rfid_read_register(REG_COMMAND) & (1 << 4)) && (++count) < 3);
}

/******************************************************************************
 * @function rfid_set_gain
 * @brief    Sets the receiver gain of the RC522.
 * @param    mask   Gain value (0x00 to 0x07 shifted left by 4)
 ******************************************************************************/
void rfid_set_gain(uint8_t mask) {
	rfid_clear_bit_mask(REG_RECEIVER_GAIN, (0x07 << 4));
	rfid_set_bit_mask(REG_RECEIVER_GAIN, mask & (0x07 << 4));
}

/******************************************************************************
 * @function rfid_get_gain
 * @brief    Gets the current receiver gain setting.
 * @return   Current gain (bits 4-6 of REG_RECEIVER_GAIN)
 ******************************************************************************/
uint8_t rfid_get_gain() {
	return rfid_read_register(REG_RECEIVER_GAIN) & (0x07 << 4);
}

/******************************************************************************
 * @function rfid_is_new_card
 * @brief    Checks if a new card is present.
 * @return   true if a card is detected, false otherwise.
 ******************************************************************************/
bool rfid_is_new_card() {
	uint8_t tag_buffer[2];
	uint8_t buffer_size = 2;

	rfid_write_register(REG_TX_MODE, 0x00);
	rfid_write_register(REG_RX_MODE, 0x00);

	rfid_status_t status = rfid_reqa(tag_buffer, &buffer_size);
	return (status == MI_OK || status == MI_COLLISION);
}

/******************************************************************************
 * @function rfid_reqa
 * @brief    Sends a REQA command to detect cards.
 * @param    response        Pointer to response buffer
 * @param    response_size   Pointer to response buffer size
 * @return   RFID status code
 ******************************************************************************/
rfid_status_t rfid_reqa(uint8_t *response, uint8_t *response_size) {
	return rfid_reqa_or_wupa(MIF_REQA, response, response_size);
}

/******************************************************************************
 * @function rfid_reqa_or_wupa
 * @brief    Sends REQA or WUPA command to detect cards.
 * @param    command         MIF_REQA or MIF_WUPA
 * @param    response        Pointer to response buffer
 * @param    response_size   Pointer to response size
 * @return   RFID status code
 ******************************************************************************/
rfid_status_t rfid_reqa_or_wupa(uint8_t command, uint8_t *response, uint8_t *response_size) {
	if (response == NULL || *response_size < 2)
		return MI_NO_ROOM;

	rfid_clear_bit_mask(REG_COLL, 0x80);
	uint8_t valid_bits = 7;

	rfid_status_t status = rfid_transcive_data(&command, 1, response, response_size, &valid_bits, 0, false);
	if (status != MI_OK || *response_size != 2 || valid_bits != 0)
		return MI_ERR;

	return MI_OK;
}

/******************************************************************************
 * @function rfid_transcive_data
 * @brief    Handles data transmission to/from the card.
 * @param    send_data       Data to send
 * @param    send_len        Length of data to send
 * @param    back_data       Buffer to receive response
 * @param    back_len        In: max length, Out: actual received
 * @param    valid_bits      Valid bits in last byte
 * @param    rx_align        Alignment of first received bit
 * @param    check_CRC       Whether to check CRC in response
 * @return   RFID status code
 ******************************************************************************/
rfid_status_t rfid_transcive_data(uint8_t *send_data, uint8_t send_len, uint8_t *back_data,
                                  uint8_t *back_len, uint8_t *valid_bits,
                                  uint8_t rx_align, bool check_CRC) {
	return rfid_to_card(CMD_TRANSCEIVE, 0x30, send_data, send_len, back_data, back_len, valid_bits, rx_align, check_CRC);
}

/******************************************************************************
 * @function rfid_to_card
 * @brief    Sends a command and data to the card and gets the response.
 * @param    command       Command to execute
 * @param    waitIRq       Bitmask for wait IRQ
 * @param    send_data     Data to send
 * @param    send_len      Length of data
 * @param    back_data     Output buffer
 * @param    back_len      Input: buffer size, Output: received size
 * @param    valid_bits    In/Out number of valid bits in last byte
 * @param    rx_align      Bit alignment
 * @param    check_CRC     Whether to check CRC
 * @return   RFID status code
 ******************************************************************************/
rfid_status_t rfid_to_card(uint8_t command,
uint8_t waitIRq,
uint8_t *send_data,
uint8_t send_len,
uint8_t *back_data,
uint8_t *back_len,
uint8_t *valid_bits,
uint8_t rx_align,
bool check_CRC
) {
	// Prepare values for BitFramingReg
	uint8_t txLastBits = valid_bits ? *valid_bits : 0;
	uint8_t bitFraming = (rx_align << 4) + txLastBits;  // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	rfid_write_register(REG_COMMAND, CMD_IDLE);     	// Stop any active command.
	rfid_write_register(REG_COMM_IRQ, 0x7F); 			// Clear all seven interrupt request bits
	rfid_write_register(REG_FIFO_LEVEL, 0x80); 			// FlushBuffer = 1, FIFO initialization
	for (int i = 0; i < send_len; ++i) {
		rfid_write_register(REG_FIFO_DATA, send_data[i]);
	}
	rfid_write_register(REG_BIT_FRAMING, bitFraming);        	// Bit adjustments
	rfid_write_register(REG_COMMAND, command);            		// Execute the command
	if (command == CMD_TRANSCEIVE) {
		rfid_set_bit_mask(REG_BIT_FRAMING, 0x80); 				// StartSend=1, transmission of data starts
	}

	uint16_t i;
	for (i = 2000; i > 0; i--) {
		uint8_t n = rfid_read_register(REG_COMM_IRQ); 	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) { 								// One of the interrupts that signal success has been set.
			break;
		}

		if (n & 0x01) {            						// Timer interrupt - nothing received in 25ms
			return MI_TIMEOUT;
		}
	}

	// 35.7ms and nothing happened. Communication with the MFRC522 might be down.

	if (i == 0) {
		return MI_TIMEOUT;
	}

	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = rfid_read_register(
	REG_ERROR); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {     // BufferOvfl ParityErr ProtocolErr
		return MI_ERR;
	}

	uint8_t _validBits = 0;

	// If the caller wants data back, get it from the MFRC522.
	if (back_data && back_len) {
		uint8_t n = rfid_read_register(REG_FIFO_LEVEL); // Number of uint8_ts in the FIFO

		if (n > *back_len) {
			return MI_NO_ROOM;
		}
		*back_len = n;                            // Number of uint8_ts returned
		//rfid_read_register(FIFODataReg, n, back_data, rx_align);	// Get received data from FIFO

		rfid_read_register_many(REG_FIFO_DATA, n, back_data, rx_align);

		_validBits = rfid_read_register(REG_CONTROL) & 0x07; // RxLastBits[2:0] indicates the number of valid bits in the last received uint8_t. If this value is 000b, the whole uint8_t is valid.
		if (valid_bits) {
			*valid_bits = _validBits;
		}
	}

	// Tell about collisions
	if (errorRegValue & 0x08) {        // CollErr
		return MI_COLLISION;
	}

	// Perform CRC_A validation if requested.
	if (back_data && back_len && check_CRC) {

		// In this case a MIFARE Classic NAK is not OK.
		if (*back_len == 1 && _validBits == 4) {
			return MI_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last uint8_t must be received.
		if (*back_len < 2 || _validBits != 0) {
			return MI_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];

		rfid_status_t status = rfid_calc_crc(&back_data[0], *back_len - 2, &controlBuffer[0]);
		// rfid_status_t = PCD_CalculateCRC(&back_data[0], *back_len - 2, &controlBuffer[0]);
		if (status != MI_OK) {
			return status;
		}
		if ((back_data[*back_len - 2] != controlBuffer[0]) || (back_data[*back_len - 1] != controlBuffer[1])) {
			return MI_CRC_WRONG;
		}
	}

	return MI_OK;
}

/******************************************************************************
 * @function rfid_calc_crc
 * @brief    Calculates CRC using the RC522 hardware engine.
 * @param    tab      Pointer to input data buffer
 * @param    len      Number of bytes in the input buffer
 * @param    out      Pointer to output buffer (2 bytes) to store CRC result
 * @return   RFID status code (MI_OK if CRC successful, MI_TIMEOUT if failed)
 *
 * @details
 * This function sends a buffer of data to the RC522’s FIFO and starts the
 * CRC calculation. It waits until the CRC interrupt flag is set, then reads
 * the resulting CRC (LSB and MSB) into the output buffer.
 ******************************************************************************/
rfid_status_t rfid_calc_crc(uint8_t *tab, uint8_t len, uint8_t *out) {

	rfid_clear_bit_mask(REG_DIV_IRQ, 0x04);                	// CRCIrq = 0
	rfid_set_bit_mask(REG_FIFO_LEVEL, 0x80);            	// Clear the FIFO pointer
	rfid_write_register(REG_COMMAND, CMD_IDLE);      		// Stop any active command.

	for (int i = 0; i < len; i++) {
		rfid_write_register(REG_FIFO_DATA, tab[i]);
	}
	rfid_write_register(REG_COMMAND, CMD_CALCCRC);

	uint8_t i = 0xFF;
	uint8_t n;
	do {
		n = rfid_read_register(REG_DIV_IRQ);
		i--;
	} while ((i != 0) && !(n & 0x04));           			//CRCIrq = 1

	if (i == 0) {
		return MI_TIMEOUT;
	}

	out[0] = rfid_read_register(REG_CRC_RESULT_L);
	out[1] = rfid_read_register(REG_CRC_RESULT_M);

	return MI_OK;

}

/******************************************************************************
 * @function rfid_anticoll
 * @brief    Performs anti-collision detection to retrieve the UID of a card.
 * @param    uid      Pointer to a buffer (at least 4 bytes) to store the UID
 * @return   RFID status code (MI_OK if UID is successfully read, MI_ERR on error)
 *
 * @details
 * This function sends the anti-collision command to the card, receives the UID,
 * and performs a checksum (XOR of first 4 bytes) to verify UID integrity.
 * If the checksum matches the fifth byte, UID is valid and copied to the buffer.
 ******************************************************************************/

rfid_status_t rfid_anticoll(uint8_t *uid) {
    rfid_status_t status;
    uint8_t buffer[9];
    uint8_t bufferSize = sizeof(buffer);
    uint8_t serNumCheck = 0;

    rfid_write_register(REG_BIT_FRAMING, 0x00); // StartSend = 0, TxLastBits = 0

    buffer[0] = PICC_ANTICOLL;
    buffer[1] = 0x20;

    status = rfid_transcive_data(buffer, 2, buffer, &bufferSize, NULL, 0, false);

    if (status == MI_OK) {
        if (bufferSize == 5) {
            for (int i = 0; i < 4; i++) {
                uid[i] = buffer[i];
                serNumCheck ^= buffer[i];
            }

            if (serNumCheck != buffer[4]) {
                return MI_ERR;
            }
        } else {
            return MI_ERR;
        }
    }

    return status;
}
