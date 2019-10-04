/* ========================================================================== */
/**@file src/drivers/ssd_pigpio.h
 *
 *  Interface to the pigpio.
 *
 * ==========================================================================\n
 * Project:		SSD1803A-C-API
 * System:		Raspberry Pi
 * Created:		02/10/2019 08:33:09 AM ksonter \n
 * Copyright (c) 2019, Kieran Sonter
 * ========================================================================== */
#ifndef SRC_DRIVERS_SSD_PIGPIO_H_
#define SRC_DRIVERS_SSD_PIGPIO_H_

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

/* === Includes ============================================================= */
#include <ssd1803a.h>

/* === Defines ============================================================== */

/* === Enumerations ========================================================= */

/* === Structures =========================================================== */

/* === Typedefs ============================================================= */

/* === Global Variables ===================================================== */

/* === Function Prototypes ================================================== */

/* ========================================================================== */
/**@brief Initialises the pigpio interface.
 * ========================================================================== */
status_t pigpio_init(void);

/* ========================================================================== */
/**@brief Deinitialises the pigpio interface.
 * ========================================================================== */
void pigpio_deinit(void);

/* ========================================================================== */
/**@brief Returns whether or not the pigpio interface is initialised.
 * ========================================================================== */
bool pigpio_is_initialised(void);

/* ========================================================================== */
/**@brief Sleep for a certain amount of time.
 * ========================================================================== */
status_t pigpio_sleep(double seconds);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // SRC_DRIVERS_SSD_PIGPIO_H_