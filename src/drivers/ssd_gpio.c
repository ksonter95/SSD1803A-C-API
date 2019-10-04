/* ========================================================================== */
/**@file src/drivers/ssd_gpio.c
 *
 *  GPIO interface to the MPU.
 *
 * ==========================================================================\n
 * Project:		SSD1803A-C-API
 * System:		Raspberry Pi
 * Created:		01/10/2019 15:25:59 PM ksonter \n
 * Copyright (c) 2019, Kieran Sonter
 * ==========================================================================\n
 * Description: \n
 * TODO: Enter description.
 * ========================================================================== */

/* === Includes ============================================================= */
#include "ssd_gpio.h"

#include <pigpio.h>
#include <stdbool.h>

#include "ssd_pigpio.h"

/* === Defines ============================================================== */


/* === Enumerations ========================================================= */

/* === Structures =========================================================== */

/* === Typedefs ============================================================= */

/* === Global Variables ===================================================== */
active_level_t m_ActiveLevel			= ACTIVE_HIGH;
int16_t m_Pin							= STATUS_INVALID_GPIO;
bool m_HardwareResetEnabled				= false;

/* === Function Prototypes ================================================== */

/* ========================================================================== */
/* ========================================================================== */
/* ========================================================================== */

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Initialise the RPi pin connected (in some way) to the RESET pin on
 *				the SSD1803A such that:
 *			- Mode of the pin is an output, and
 *			- Level of the pin is the inactive level of the pin.
 *
 * @param [in]		pin GPIO pin number on the RPi to which the RESET pin on
 * the SSD1803A is connected (in some way).
 * @param [in]		activeLevel GPIO level at which the SSD1803A will be active.
 *
 * @retval STATUS_OK If the GPIO hardware was successfully initialised.
 * @retval STATUS_NOT_INITIALISED If the pigpio library has not been previously
 * initialised with a call to pigpio_init().
 * @retval STATUS_INVALID_GPIO If the specified GPIO to which the RESET pin on
 * the SSD1803A is connected (in some way) is not a valid RPi GPIO pin number.
 * @retval STATUS_INVALID_MODE and STATUS_INVALID_LEVEL are theoretically
 * possible return values from the function gpioSetMode/gpioWrite, however
 * they should never occur due to the restrictions on the data type
 * active_level_t.
 * ========================================================================== */
status_t gpio_init(
	uint8_t pin,
	active_level_t activeLevel) {

	/* Check whether hardware reset is utilised */
	m_HardwareResetEnabled = (pin != NO_HARDWARE_RESET_PIN);
	if (pin == NO_HARDWARE_RESET_PIN) {
		return STATUS_OK;
	}

	/* pigpio library is not initialised */
	if (!pigpio_is_initialised()) {
		return STATUS_NOT_INITIALISED;
	}

	/* Initialise the SSD1803A enable pin on the RPi */
	status_t ret = gpioSetMode(pin, PI_OUTPUT);
	if (ret != STATUS_OK) {
		return ret;
	}
	m_Pin = pin;

	/* Hold the SSD1803A enable pin in the reset position */
	m_ActiveLevel = activeLevel;
	return gpioWrite((uint32_t)m_Pin, !m_ActiveLevel);

}

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Initialise the RPi pin connected (in some way) to the RESET pin on
 *				the SSD1803A such that:
 *			- Mode of the pin is an input, and
 *			- Pull of the pin is to the inactive level of the pin.
 *
 * @retval STATUS_OK If the GPIO hardware was successfully deinitialised.
 * @retval STATUS_NOT_INITIALISED If the GPIO hardware had not been previously
 * initialised with a call to gpio_init().
 * @retval STATUS_INVALID_GPIO, STATUS_INVALID_MODE, STATUS_INVALID_LEVEL, and
 * STATUS_INVALID_PUD are theoretically possible return values from the function
 * gpioSetMode/gpioSetPullUpDown, however they should never occur due to the
 * restrictions on the data type active_level_t.
 * ========================================================================== */
status_t gpio_deinit(void) {

	/* Hardware reset is not enabled */
	if (!m_HardwareResetEnabled) {
		return STATUS_OK;
	}

	/* GPIO has not been previously initialised */
	if (m_Pin == STATUS_INVALID_GPIO) {
		return STATUS_NOT_INITIALISED;
	}

	/* Deinitialise the SSD1803A enable pin on the RPi */
	status_t ret = gpioSetMode((uint8_t)m_Pin, PI_INPUT);
	if (ret != STATUS_OK) {
		return ret;
	}
	return gpioSetPullUpDown((uint32_t)m_Pin, !m_ActiveLevel);

}

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Enable the RPi pin connected (in some way) to the RESET pin on the
 *				SSD1803A such that:
 *			- Level of the pin is the active level of the pin.
 *
 * @retval STATUS_OK If the GPIO hardware was successfully deinitialised.
 * @retval STATUS_NOT_INITIALISED If the GPIO hardware had not been previously
 * initialised with a call to gpio_init().
 * @retval STATUS_INVALID_GPIO and STATUS_INVALID_LEVEL are theoretically
 * possible return values from the function gpioWrite, however they should never
 * occur due to the restrictions on the data type active_level_t.
 * ========================================================================== */
status_t gpio_enable_display(void) {

	/* Hardware reset is not enabled */
	if (!m_HardwareResetEnabled) {
		return STATUS_OK;
	}

	/* GPIO has not been previously initialised */
	if (m_Pin == STATUS_INVALID_GPIO) {
		return STATUS_NOT_INITIALISED;
	}

	/* Enable the SSD1803A */
	return gpioWrite((uint32_t)m_Pin, m_ActiveLevel);

}

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Disable the RPi pin connected (in some way) to the RESET pin on the
 *				SSD1803A such that:
 *			- Level of the pin is the inactive level of the pin.
 *
 * @retval STATUS_OK If the GPIO hardware was successfully deinitialised.
 * @retval STATUS_NOT_INITIALISED If the GPIO hardware had not been previously
 * initialised with a call to gpio_init().
 * @retval STATUS_INVALID_GPIO and STATUS_INVALID_LEVEL are theoretically
 * possible return values from the function gpioWrite, however they should never
 * occur due to the restrictions on the data type active_level_t.
 * ========================================================================== */
status_t gpio_disable_display(void) {

	/* Hardware reset is not enabled */
	if (!m_HardwareResetEnabled) {
		return STATUS_OK;
	}

	/* GPIO has not been previously initialised */
	if (m_Pin == STATUS_INVALID_GPIO) {
		return STATUS_NOT_INITIALISED;
	}

	/* Enable the SSD1803A */
	return gpioWrite((uint32_t)m_Pin, !m_ActiveLevel);

}