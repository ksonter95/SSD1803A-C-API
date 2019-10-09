/* ========================================================================== */
/**@file src/drivers/ssd_pigpio.c
 *
 *  Interface to the pigpio.
 *
 * ==========================================================================\n
 * Project:		SSD1803A-C-API
 * System:		Raspberry Pi
 * Created:		02/10/2019 08:33:51 AM ksonter \n
 * Copyright (c) 2019, Kieran Sonter
 * ==========================================================================\n
 * Description: \n
 * This driver is only used internally within the SSD1803A library and it
 * provides an interface to any generic pigpio functions.  As such, it provides
 * the following functions:
 * 		- pigpio_init(): Initialises the pigpio library.
 * 		- pigpio_deinit(): Deinitialises the pigpio library.
 * 		- pigpio_is_initialised(): Returns whether or not the pipgio has
 * 				previously been initialised.
 * 		- pigpio_sleep(): Pauses programme execution for the prescribed amount
 * 				of time.
 * 
 * For any further information on these functions, see the function comments
 * shown below.
 * ========================================================================== */

/* === Includes ============================================================= */
#include "ssd_pigpio.h"

#include <pigpio.h>
#include <stdbool.h>
#include <stdio.h>

/* === Defines ============================================================== */

/* === Enumerations ========================================================= */

/* === Structures =========================================================== */

/* === Typedefs ============================================================= */

/* === Global Variables ===================================================== */
bool m_PigpioInitialised				= false;

/* === Function Prototypes ================================================== */

/* ========================================================================== */
/* ========================================================================== */
/* ========================================================================== */

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Initialise the pigpio library.
 *
 * @retval STATUS_OK If the interface to the pigpio was successfully
 * initialised.
 * @retval STATUS_NOT_INITIALISED If the pigpio library failed to be
 * successfully initialised.
 * ========================================================================== */
status_t pigpio_init(void) {

	if (gpioInitialise() < STATUS_OK) {
		m_PigpioInitialised = false;
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	} else {
		m_PigpioInitialised = true;
		return STATUS_OK;
	}

}

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Deinitialise the pigpio library.
 * ========================================================================== */
void pigpio_deinit(void) {

	m_PigpioInitialised = false;
	gpioTerminate();

}

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Determine whether the pigpio interface is initialised.
 *
 * @retval true If the pigpio interface is initialised.
 * @retval false If the pigpio interface is not initialised.
 * ========================================================================== */
bool pigpio_is_initialised(void) {

	return m_PigpioInitialised;

}

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Sleep for the specified number of seconds.
 *
 * @param [in]		seconds Number of seconds to sleep.
 * 
 * @retval STATUS_OK If the sleep was successful.
 * @retval STATUS_NOT_INITIALISED If the pigpio library was not previously
 * successfully initialised.
 * ========================================================================== */
status_t pigpio_sleep(double seconds) {

	/* pigpio library not initialised */
	if (!m_PigpioInitialised) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Sleep */
	time_sleep(seconds);
	return STATUS_OK;

}