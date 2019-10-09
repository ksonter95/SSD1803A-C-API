/* ========================================================================== */
/**@file src/drivers/ssd_i2c.c
 *
 *  I2C interface to the MPU.
 *
 * ==========================================================================\n
 * Project:		SSD1803A-C-API
 * System:		Raspberry Pi
 * Created:		13/09/2019 12:42:11 PM ksonter \n
 * Copyright (c) 2019, Kieran Sonter
 * ==========================================================================\n
 * Description: \n
 * This driver is only used internally within the SSD1803A library and it
 * provides an interface to any I2C pigpio functions formatted to be understood
 * by the SSD1803A display MPU.  As such, it provides the following functions:
 * 		- i2c_init(): Initialises the I2C hardware drivers.
 * 		- i2c_deinit(): Deinitialises the I2C hardware drivers.
 * 		- i2c_write(): Conducts an I2C write operation.
 * 		- i2c_read(): Conducts an I2C read operation.
 * 
 * For any further information on these functions, see the function comments
 * shown below.
 * ========================================================================== */

/* === Includes ============================================================= */
#include "ssd_i2c.h"

#include <pigpio.h>
#include <stdio.h>

#include "ssd1803a.h"
#include "ssd_pigpio.h"

/* === Defines ============================================================== */
/* I2C configuration */
#define ADDRESS_BASE                    0x3CU
#define I2C_FLAGS                       0x00

/* Control bytes */
#define CONTROL_BYTE_DATA               0x40
#define CONTROL_BYTE_COMMAND_FURTHER    0x80
#define CONTROL_BYTE_COMMAND_LAST       0x00

/* === Enumerations ========================================================= */

/* === Structures =========================================================== */

/* === Typedefs ============================================================= */

/* === Global Variables ===================================================== */
int16_t m_I2cHandle = (int16_t) STATUS_INVALID_HANDLE;

/* === Function Prototypes ================================================== */

/* ========================================================================== */
/* ========================================================================== */
/* ========================================================================== */

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Open the I2C interface to the MPU.
 *
 * @param [in]		bus I2C bus number on the RPi that is connected to the I2C
 * bus on the display MPU.
 * @param [in]		sa0 Least significant bit of the I2C address at which the
 * display MPU can be found.
 *
 * @retval STATUS_OK If the I2C hardware was successfully initialised.
 * @retval STATUS_NOT_INITIALISED If the pigpio library has not been previously
 * initialised with a call to pigpio_init().
 * @retval STATUS_NO_HANDLE If there is no available handle to assign to the
 * display MPU I2C bus.
 * @retval STATUS_FAILED_OPEN If I2C interface failed to be opened.
 * @retval STATUS_ALREADY_OPEN If this function (i2c_init()) had already been
 * called and successfully opened an interface to the display MPU.
 * @retval STATUS_INVALID_BUS, STATUS_INVALID_ADDRESS, and STATUS_INVALID_FLAGS
 * are theoretically possible return values from the function i2cOpen, however
 * they should never occur due to the restrictions on the data types i2c_bus_t
 * and sa0_bit_t.
 * ========================================================================== */
status_t i2c_init(
		i2c_bus_t bus,
		sa0_bit_t sa0) {

	/* pigpio library is not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Interface to MPU has already been opened */
	if (m_I2cHandle != STATUS_INVALID_HANDLE) {
		LOG_TO_STDERR();
		return STATUS_ALREADY_OPEN;
	}

	/* Open the I2C interface */
	status_t ret = i2cOpen(bus, ADDRESS_BASE | sa0, I2C_FLAGS);

	/* Opened the I2C interface */
	if (ret >= STATUS_OK) {
		m_I2cHandle = (int16_t)ret;
		ret = STATUS_OK;
	} else {
		LOG_TO_STDERR();
	}
	return ret;

}

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Close the I2C interface to the MPU.
 *
 * retval STATUS_OK If the I2C hardware was successfully deinitialised.
 * retval STATUS_INVALID_HANDLE If the I2C hardware was not successfully
 * initialised before calling this function.
 * ========================================================================== */
status_t i2c_deinit(void) {

	status_t ret = STATUS_OK;

	/* I2C interface has not yet been opened */
	if (m_I2cHandle == STATUS_INVALID_HANDLE) {
		LOG_TO_STDERR();
		return STATUS_INVALID_HANDLE;
	}
	
	ret = i2cClose((uint16_t)m_I2cHandle);
	m_I2cHandle = STATUS_INVALID_HANDLE;
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Constructs the bytes that comprise the I2C transaction according to
 *				structure shown below.
 *		-# Write the data to the display MPU over the I2C interface.
 *
 * The transaction order is shown below:
 * S Addr Wr [A] Control #1 [A] Command [A] Control #2 [A]   Data [A]    P
 *              |                       |                 |             |
 *              +------- n times -------+                 +-- m times --+
 *				  Control/Command words				         Data bytes
 * Where:
 * 		- S: Start bit
 * 		- P: Stop bit
 * 		- A: Accept bit
 * 		- Wr: Write bit (= 0)
 * 		- Addr: 7-bit I2C address of the display MPU.
 * 		- [...]: Data transmitted from the display MPU.
 *      - Control: Byte used to communicate the meaning of the next byte.  The
 * 				byte structure is shown below.
 * 			+-----------------------------------------------+
 * 			| C0  | D/C |  0  |  0  |  0  |  0  |  0  |  0  |
 * 			+-----------------------------------------------+
 * 			Where:
 * 				- C0: Continuation bit. For C0 = 1, more Control/Command words
 * 						will proceed after the next command byte.  For C0 = 0,
 *						the next bytes will only be data bytes.
 *				- D/C: Data/Command bit.  For D/C = 1, the next byte is to be
 *						interpretted as a data byte.  For D/C = 0, the next byte
 *						is to be interpretted as a command byte.
 *			Specifically:
 *				- Control #1: 0b10000000
 *				- Control #2: 0b01000000
 *		- Command: Byte used to change the setting of the display MPU.
 *		- Data: Data to be written to DDRAM/CGRAM/SEGRAM.
 *
 * @param [in]		commands: Array of command bytes to populate the Command
 * portion of the I2C transaction shown above.
 * @param [in]		commandsLen: Number of command words to be included in the
 * I2C transaction.  This is the value of "n" in the above transaction order
 * diagram.
 * @param [in]		data: Array of data bytes to populate the Data portion of
 * the I2C transaction shown above.
 * @param [in]		dataLen: Number of data bytes to be included in the I2C
 * transaction.  This is the value of "m" in the above transaction order
 * diagram.
 * 
 * @pre commands must point to a section of memory that has been previously
 * allocated and is commandsLen bytes long.
 * @pre data must point to a section of memory that has been previously
 * allocated and is dataLen bytes long.
 *
 * @retval STATUS_OK If the commands and data were successfully transmitted to
 * the display MPU.
 * @retval STATUS_INVALID_HANDLE If the I2C hardware was not successfully
 * initialised before calling this function.
 * @retval STATUS_INVALID_PARAM If the number of bytes to be written to the
 * display MPU is <= 0 (commandsLen + dataLen <= 0).
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * ========================================================================== */
status_t i2c_write(
		uint8_t *commands,
		uint8_t commandsLen,
		uint8_t *data,
		uint8_t dataLen) {

	/* I2C interface has not yet been opened */
	if (m_I2cHandle == STATUS_INVALID_HANDLE) {
		LOG_TO_STDERR();
		return STATUS_INVALID_HANDLE;
	}

	/* Initialise the message */
	uint16_t length;
	length = (uint16_t)(2 * commandsLen + (dataLen > 0 ? 1 + dataLen : 0));
	uint8_t message[length];

	/* Add the control/command byte words */
	for (uint8_t i = 0; i < commandsLen; i++) {
		message[2 * i] = CONTROL_BYTE_COMMAND_FURTHER;
		message[2 * i + 1] = commands[i];
	}

	/* Add the data control byte */
	if (dataLen > 0) {
		message[2 * commandsLen] = CONTROL_BYTE_DATA;
	}

	/* Add the data */
	for (uint16_t i = 0; i < dataLen; i++) {
		message[2 * commandsLen + 1 + i] = data[i];
	}

	/* Transmit the message to the MPU */
	status_t ret = (status_t) i2cWriteDevice(
			(uint16_t)m_I2cHandle,
			(char *)message,
			length
	);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Read data from the DDRAM/CGRAM/SEGRAM.
 *
 * The transaction order is shown below:
 * S Addr Wr [A] Control #1 [A] P S Addr Rd [A]    [Data]     [A] P
 * 											   |             |
 * 											   +-- n times --+
 * Where:
 * 		- S: Start bit
 * 		- P: Stop bit
 * 		- A: Accept bit
 * 		- Wr: Write bit (= 0)
 * 		- Rd: Read bit (= 1)
 * 		- Addr: 7-bit I2C address of the display MPU.
 * 		- [...]: Data transmitted from the display MPU.
 *      - Control: Byte used to communicate the meaning of the next byte.  The
 * 				byte structure is shown below.
 * 			+-----------------------------------------------+
 * 			| C0  | D/C |  0  |  0  |  0  |  0  |  0  |  0  |
 * 			+-----------------------------------------------+
 * 			Where:
 * 				- C0: Continuation bit. For read, C0 will always be 0.
 *				- D/C: Data/Command bit.  For D/C = 1, the next byte(s) to be
 *						received should be the busy flag/address/part ID.  For
 *						For D/C = 0, the next byte(s) to be received should be
 *						data stored in the display MPU RAM (DDRAM/CGRAM/SEGRAM,
 *						depending which RAM is being currently used).
 *			Specifically:
 *				- Control #1: 0b01000000 (readRam == 1), 0b00000000 (readRam ==
 *						0)
 *		- Command: Byte used to change the setting of the display MPU.
 *		- Data: Data to be read from DDRAM/CGRAM/SEGRAM or busy flag/address/
 *				part ID.
 *
 * If readRam == true, then the first byte of data will be a dummy byte, and as
 * such has no meaning.  If readRam == false, then the format of the returned
 * data will be as shown below:
 * 			+-----------------------------------------------+
 * data0:	| BF  | AC6 | AC5 | AC4 | AC3 | AC2 | AC1 | AC0 |
 * 			+-----------------------------------------------+
 * 			+-----------------------------------------------+
 * data1:	| BF  | ID6 | ID5 | ID4 | ID3 | ID2 | ID1 | ID0 |
 * 			+-----------------------------------------------+
 * Where:
 * 		- BF: Busy flag.  BF = 1 indicates that the display MPU is still busy
 * 				processing a previous I2C command.
 * 		- AC6:AC0: Address counter.  Next address to be written to/read from.
 * 		- ID6:ID0: Part ID.  Will be 0x1A.
 *
 * @param [in]		readRam Flag indicating whether the read operation is to
 * return the data in the display MPU RAM (readRam == true) or the busy flag/
 * address counter value/part ID.
 * @param [out]		data Pointer to the memory address where the data read from
 * the display MPU is to be stored.
 * @param [in]		dataLen Number of data bytes to be read from the display
 * MPU.
 * 
 * @pre data must point to a section of memory that has been previously
 * allocated and is dataLen bytes long.
 * @pre If readRam == false, then dataLen == 2.
 * @pre dataLen > 0
 *
 * @retval STATUS_OK If the data was successfully received from the display MPU.
 * @retval STATUS_INVALID_HANDLE If the I2C hardware was not successfully
 * initialised before calling this function.
 * @retval STATUS_INVALID_PARAM If the number of bytes to be read from the
 * display MPU is <= 0 (dataLen <= 0).
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_FAILED_READ If the message failed to be successfully read from
 * the display MPU.
 * ========================================================================== */
status_t i2c_read(
		bool readRam,
		uint8_t *data,
		uint8_t dataLen) {
	
	/* Transmit the control byte */
	int32_t ret = i2cWriteByte(
			(uint16_t)m_I2cHandle,
			readRam ? CONTROL_BYTE_DATA : CONTROL_BYTE_COMMAND_LAST
	);
	if (ret < STATUS_OK) {
		LOG_TO_STDERR();
		return (status_t)ret;
	}

	/* Receive the data */
	ret = i2cReadDevice(
			(uint16_t)m_I2cHandle,
			(char *)data,
			dataLen
	);
	if (ret < STATUS_OK) {
		LOG_TO_STDERR();
	}
	return (ret < STATUS_OK) ? (status_t)ret : STATUS_OK;

}
