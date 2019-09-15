/* ========================================================================== */
/**@file src/ssd1803a.h
 *
 *  API for the SSD1803A Display MPU.
 *
 * ==========================================================================\n
 * Project:		SSD1803A-C-API
 * System:		Raspberry Pi
 * Created:		13/09/2019 16:33:24 PM ksonter \n
 * Copyright (c) 2019, Kieran Sonter
 * ========================================================================== */
#ifndef SRC_SSD1803A_H_
#define SRC_SSD1803A_H_

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

/* === Includes ============================================================= */
#include <stdbool.h>

/* === Defines ============================================================== */
/* I2C Bus */
#define I2C_BUS_0                       0
#define I2C_BUS_1                       1

/* I2C Address LSB(it) */
#define SA0_ADDRESS_0x3C                0
#define SA0_ADDRESS_0x3D                1
#define SA0_LOW                         0
#define SA0_HIGH                        1

/* === Enumerations ========================================================= */
enum StatusCodes {
	STATUS_FAILED_WRITE					= -0x52, // -82
	STATUS_INVALID_PARAM				= -0x51, // -81
	STATUS_INVALID_FLAGS                = -0x4D, // -77
	STATUS_INVALID_ADDRESS              = -0x4B, // -75
	STATUS_INVALID_BUS                  = -0x4A, // -74
	STATUS_FAILED_OPEN                  = -0x47, // -71
	STATUS_INVALID_HANDLE               = -0x19, // -25
	STATUS_NO_HANDLE                    = -0x18, // -24
	STATUS_NOT_INITIALISED              = -0x01, // -1
	STATUS_OK,
	STATUS_ALREADY_OPEN,
};

/* === Structures =========================================================== */

/* === Typedefs ============================================================= */
typedef enum StatusCodes                status_t;
typedef bool							i2c_bus_t;
typedef bool							sa0_bit_t;

/* === Global Variables ===================================================== */

/* === Function Prototypes ================================================== */
/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // SRC_SSD1803A_H_
