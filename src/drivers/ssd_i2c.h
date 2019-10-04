/* ========================================================================== */
/**@file src/drivers/ssd_i2c.h
 *
 *  I2C interface to the MPU.
 *
 * ==========================================================================\n
 * Project:		SSD1803A-C-API
 * System:		Raspberry Pi
 * Created:		13/09/2019 12:42:14 PM ksonter \n
 * Copyright (c) 2019, Kieran Sonter
 * ========================================================================== */
#ifndef SRC_DRIVERS_SSD_I2C_H_
#define SRC_DRIVERS_SSD_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

/* === Includes ============================================================= */
#include <stdint.h>
#include <stdbool.h>

#include <ssd1803a.h>

/* === Defines ============================================================== */
#define READ_RAM						true
#define READ_AC_ID						false

#define LENGTH_AC_ID					(0x02)

/* === Enumerations ========================================================= */

/* === Structures =========================================================== */

/* === Typedefs ============================================================= */

/* === Global Variables ===================================================== */

/* === Function Prototypes ================================================== */

/* ========================================================================== */
/**@brief Initialises the I2C hardware drivers.
 * ========================================================================== */
status_t i2c_init(
		i2c_bus_t bus,
		sa0_bit_t sa0
);

/* ========================================================================== */
/**@brief Deinitialises the I2C hardware drivers.
 * ========================================================================== */
status_t i2c_deinit(void);

/* ========================================================================== */
/**@brief Conducts an I2C write operation.
 * ========================================================================== */
status_t i2c_write(
		uint8_t *commands,
		uint8_t commandsLen,
		uint8_t *data,
		uint8_t dataLen
);

/* ========================================================================== */
/**@brief Conducts an I2C read operation.
 * ========================================================================== */
status_t i2c_read(
		bool readRam,
		uint8_t *data,
		uint8_t dataLen
);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // SRC_DRIVERS_SSD_I2C_H_
