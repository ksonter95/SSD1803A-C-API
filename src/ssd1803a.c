/* ========================================================================== */
/**@file src/ssd1803a.c
 *
 *  API for the SSD1803A Display MPU.
 *
 * ==========================================================================\n
 * Project:		SSD1803A-C-API
 * System:		Raspberry Pi
 * Created:		13/09/2019 16:45:36 PM ksonter \n
 * Copyright (c) 2019, Kieran Sonter
 * ==========================================================================\n
 * Description: \n
 * TODO: Enter description.
 * ========================================================================== */

/* === Includes ============================================================= */
#include "ssd1803a.h"

#include <stdio.h>
#include <string.h>

#include "ssd_gpio.h"
#include "ssd_i2c.h"
#include "ssd_pigpio.h"

/* === Defines ============================================================== */
#define NW_MASK							(0x01)
#define N_MASK							(0x08)

#define UD_MASK							(0x0C)
#define DH_MASK							(0x01)

#define BS0_MASK						(0x08)
#define BS1_MASK						(0x02)

#define CH_MASK							(0x03)
#define CL_MASK							(0x0F)

#define BF_MASK							(0x80)
#define AC_MASK							(0x7F)
#define ID_MASK							(0x7F)

#define DH_SHIFT						(0x02)
#define CH_SHIFT						(0x04)
#define SYMBOL_ID_SHIFT					(0x03)

#define IS_0							(0x00)		// IS = 0
#define IS_1							(0x01)		// IS = 1

#define RE_0							(0x00)		// RE = 0
#define RE_1							(0x02)		// RE = 1

#define DH_0							(0x00)		// DH = 0
#define DH_1							(0x04)		// DH = 1

#define CH_0							(0x00)		// C5-4 = 00
#define CL_0							(0x00)		// C3-0 = 0000

#define RAB_0							(0x00)		// Rab2-0 = 000

#define DDRAM_SIZE						(0x50)
#define CGRAM_SIZE						(0x40)
#define SEGRAM_SIZE						(0x10)

#define SYMBOL_SIZE						(0x08)
#define MAX_SYMBOLS						(0x08)

#define MIN_SYMBOL_ID					(0x00)
#define MAX_SYMBOL_ID					(0x07)

#define MIN_CGRAM						(0x00)
#define MAX_CGRAM						(0x3F)

#define MIN_SEGRAM						(0x00)
#define MAX_SEGRAM						(0x0F)

#define MIN_DDRAM						(0x00)

#define MIN_DDRAM_SINGLE_LINE_LINE1		(0x00)
#define MAX_DDRAM_SINGLE_LINE_LINE1		(0x4F)

#define MIN_DDRAM_DOUBLE_LINE_LINE1		(0x00)
#define MAX_DDRAM_DOUBLE_LINE_LINE1		(0x27)
#define MIN_DDRAM_DOUBLE_LINE_LINE2		(0x40)
#define MAX_DDRAM_DOUBLE_LINE_LINE2		(0x67)

#define MIN_DDRAM_TRIPLE_LINE_LINE1		(0x00)
#define MAX_DDRAM_TRIPLE_LINE_LINE1		(0x13)
#define MIN_DDRAM_TRIPLE_LINE_LINE2		(0x20)
#define MAX_DDRAM_TRIPLE_LINE_LINE2		(0x33)
#define MIN_DDRAM_TRIPLE_LINE_LINE3		(0x40)
#define MAX_DDRAM_TRIPLE_LINE_LINE3		(0x53)

#define MIN_DDRAM_QUADRUPLE_LINE_LINE1	(0x00)
#define MAX_DDRAM_QUADRUPLE_LINE_LINE1	(0x13)
#define MIN_DDRAM_QUADRUPLE_LINE_LINE2	(0x20)
#define MAX_DDRAM_QUADRUPLE_LINE_LINE2	(0x33)
#define MIN_DDRAM_QUADRUPLE_LINE_LINE3	(0x40)
#define MAX_DDRAM_QUADRUPLE_LINE_LINE3	(0x53)
#define MIN_DDRAM_QUADRUPLE_LINE_LINE4	(0x60)
#define MAX_DDRAM_QUADRUPLE_LINE_LINE4	(0x73)

#define MIN_CONTRAST					(0x00)
#define MAX_CONTRAST					(0x3F)

#define MIN_BRIGHTNESS					(0x00)
#define MAX_BRIGHTNESS					(0x07)

#define MIN_SCROLL						(0x00)
#define MAX_SCROLL						(0x30)

#define COMMAND_CLEAR_DISPLAY			(0x01)
#define COMMAND_RETURN_HOME				(0x02)
#define COMMAND_POWER_DOWN_MODE			(0x02)
#define COMMAND_ENTRY_MODE_SET			(0x04)
#define COMMAND_DISPLAY_ON_OFF_CONTROL	(0x08)
#define COMMAND_EXTENDED_FUNCTION_SET	(0x08)
#define COMMAND_CURSOR_SHIFT			(0x10)
#define COMMAND_DISPLAY_SHIFT			(0x18)
#define COMMAND_HEIGHT_BIAS_SHIFT		(0x10)
#define COMMAND_INTERNAL_OSC_FREQUENCY	(0x10)
#define COMMAND_SHIFT_SCROLL_ENABLE		(0x10)
#define COMMAND_FUNCTION_SET			(0x30)
#define COMMAND_SET_CG_SEG_RAM_ADDRESS	(0x40)
#define COMMAND_POWER_ICON_CONTRAST		(0x50)
#define COMMAND_FOLLOWER_CONTROL		(0x60)
#define COMMAND_CONTRAST_SET			(0x70)
#define COMMAND_SET_DDRAM_ADDRESS		(0x80)
#define COMMAND_SET_SCROLL_QUANTITY		(0x80)
#define COMMAND_TEMPERATURE_COEFF		(0x76)
#define COMMAND_ROM_SELECTION			(0x72)

/* === Enumerations ========================================================= */

/* === Structures =========================================================== */

/* === Typedefs ============================================================= */

/* === Global Variables ===================================================== */
/* Register bit cache */
uint8_t m_PD	= POWER_SAVING_MODE_DISABLED;		// PD = 0

uint8_t m_S		= CURSOR_MOVING;					// S = 0
uint8_t m_ID	= NORMAL_WRITE_DIRECTION;			// I/D = 1
uint8_t m_BDS	= NORMAL_HORIZONTAL_DIRECTION;		// BDS = 1
uint8_t m_BDC	= NORMAL_VERTICAL_DIRECTION;		// BDC = 0

uint8_t m_B		= CURSOR_BLINK_DISABLED;			// B = 0
uint8_t m_C		= CURSOR_DISABLED;					// C = 0
uint8_t m_D		= DISPLAY_DISABLED;					// D = 0

uint8_t m_NW	= DISPLAY_FOUR_LINES & NW_MASK;		// NW = 1
uint8_t m_BW	= CURSOR_SHADOW_ENABLED;			// B/W = 0
uint8_t m_FW	= FONT_WIDTH_FIVE_DOTS;				// FW = 0

uint8_t m_DHd	= DOT_SCROLL_ENABLED;				// DH' = 0
uint8_t m_BS1	= BIAS_1_5 & BS1_MASK;				// BS1 = 0
uint8_t m_UDx	= DOUBLE_HEIGHT_DISABLED & UD_MASK; // UD2-1 = 11

uint8_t m_Fx	= BLINK_FREQUENCY_2700mHz;			// F2-0 = 011
uint8_t m_BS0	= BIAS_1_5 & BS0_MASK;				// BS0 = 0

uint8_t m_DHSx	= SHIFT_SCROLL_ALL_LINES_ENABLED;	// DS4-1 = HS4-1 = 1

uint8_t m_IS	= IS_0;								// IS = 0
uint8_t m_RE	= RE_0;								// RE = 0
uint8_t m_DH	= DH_0;								// DH = 0
uint8_t m_N		= DISPLAY_FOUR_LINES & N_MASK;		// N = 1
uint8_t m_REV	= FONT_BLACK;						// REV = 0
uint8_t m_BE	= BLINK_DISABLED;					// BE = 0

uint8_t m_Bon	= BOOSTER_AND_REGULATOR_EXTERNAL;	// Bon = 0
uint8_t m_Ion	= ICON_DISPLAY_DISABLED;			// Ion = 0

uint8_t m_Rabx	= 0x02;								// Rab2-0 = 010
uint8_t m_Don	= DIVIDER_CIRCUIT_EXTERNAL;			// Don = 0

uint8_t m_Cx	= 0x20;								// C5-0 = 100000
uint8_t m_Ch	= 0x02;								// C5-4 = 10
uint8_t m_Cl	= 0x00;								// C3-0 = 0000

uint8_t m_AC	= MIN_DDRAM;						// AC6-0 = 0000000
uint8_t m_SQx	= MIN_SCROLL;						// SQ5-0 = 000000

uint8_t m_TCx	= NEG_0_05_PERCENT_PER_DEG_C;		// TC2-0 = 010

uint8_t m_ROMx	= ROM_A;							// ROM2-1 = 00

bool m_Symbols[MAX_SYMBOLS] = {0};

/* === Function Prototypes ================================================== */

/* ========================================================================== */
/* ========================================================================== */
/* ========================================================================== */

/* ========================================================================== */
/**@brief Initialises the SSD1803A display MPU
 * ========================================================================== */
status_t ssd_init(
		uint8_t rpiResetPin,
		active_level_t resetPinActiveLevel,
		i2c_bus_t rpiI2CBus,
		sa0_bit_t sa0,
		uint8_t internalDividerBias,
		uint8_t boosterAndRegulatorCircuit,
		uint8_t dividerCircuit,
		uint8_t temperatureCoefficient,
		uint8_t rom) {

	/* Set the internal divider bias */
	if ((internalDividerBias == BIAS_1_4) ||
			(internalDividerBias == BIAS_1_5) ||
			(internalDividerBias == BIAS_1_6) ||
			(internalDividerBias == BIAS_1_7)) {
		m_BS0 = internalDividerBias & BS0_MASK;
		m_BS1 = internalDividerBias & BS1_MASK;
	} else if (internalDividerBias != DEFAULT) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Set the booster and regulator circuit */
	if ((boosterAndRegulatorCircuit == BOOSTER_AND_REGULATOR_EXTERNAL) ||
			(boosterAndRegulatorCircuit == BOOSTER_AND_REGULATOR_INTERNAL)) {
		m_Bon = boosterAndRegulatorCircuit;
	} else {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Set the divider and regulator circuit */
	if ((dividerCircuit == DIVIDER_CIRCUIT_EXTERNAL) ||
			(dividerCircuit == DIVIDER_CIRCUIT_INTERNAL)) {
		m_Don = dividerCircuit;
	} else {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Check the temperature coefficient */
	if ((temperatureCoefficient == NEG_0_05_PERCENT_PER_DEG_C) ||
			(temperatureCoefficient == NEG_0_10_PERCENT_PER_DEG_C) ||
			(temperatureCoefficient == NEG_0_15_PERCENT_PER_DEG_C)) {
		m_TCx = temperatureCoefficient;
	} else if (temperatureCoefficient != DEFAULT) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Set the ROM */
	if ((rom == ROM_A) || (rom == ROM_B) || (rom == ROM_C)) {
		m_ROMx = rom;
	} else if (rom != DEFAULT) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Set the DDRAM address counter */
	m_AC = MIN_DDRAM;

	/* Initialise the pigpio library */
	status_t ret = pigpio_init();
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}

	/* Initialise and enable the GPIO hardware */
	ret = gpio_init(rpiResetPin, resetPinActiveLevel);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}
	ret = gpio_enable_display();
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		gpio_deinit();
		return ret;
	}

	/* Initialise the I2C */
	ret = i2c_init(rpiI2CBus, sa0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		gpio_disable_display();
		gpio_deinit();
		return ret;
	}

	/* Update the MPU hardware settings */
	uint8_t commands[10];
	commands[0] = COMMAND_CLEAR_DISPLAY;
	commands[1] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	commands[2] = COMMAND_FUNCTION_SET | m_N | m_BE | RE_1 | m_REV;
	commands[3] = COMMAND_HEIGHT_BIAS_SHIFT | m_UDx | m_BS1 | m_DHd;
	commands[4] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_1;
	commands[5] = COMMAND_INTERNAL_OSC_FREQUENCY | m_BS0 | m_Fx;
	commands[6] = COMMAND_CONTRAST_SET | m_Cl;
	commands[7] = COMMAND_POWER_ICON_CONTRAST | m_Ion | m_Bon | m_Ch;
	commands[8] = COMMAND_FOLLOWER_CONTROL | m_Don | m_Rabx;
	commands[9] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	ret = i2c_write(commands, 10, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		i2c_deinit();
		gpio_disable_display();
		gpio_deinit();
		return ret;
	}

	/* Update the MPU temperature coefficient settings */
	commands[0] = COMMAND_TEMPERATURE_COEFF;
	uint8_t data = m_TCx;
	ret = i2c_write(commands, 1, &data, 1);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		i2c_deinit();
		gpio_disable_display();
		gpio_deinit();
		return ret;
	}

	/* Update the MPU ROM settings */
	commands[0] = COMMAND_ROM_SELECTION;
	data = m_ROMx;
	ret = i2c_write(commands, 1, &data, 1);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		i2c_deinit();
		gpio_disable_display();
		gpio_deinit();
		return ret;
	}

	/* Reset the RE and IS to default state */
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	ret = i2c_write(commands, 1, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		i2c_deinit();
		gpio_disable_display();
		gpio_deinit();
	}
	return ret;

}

status_t ssd_deinit(void) {

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Disable/deinitialise the I2C, GPIO and pigpio library */
	i2c_deinit();
	gpio_disable_display();
	gpio_deinit();
	pigpio_deinit();
	return STATUS_OK;

}

status_t ssd_display_configure(
		uint8_t writeDirection,
		uint8_t numberOfLines,
		uint8_t blinkEnable,
		uint8_t shiftOrScroll,
		uint8_t shiftOrScrollLinesEnabled,
		uint8_t horizontalDisplayDirection,
		uint8_t verticalDisplayDirection) {

	/* Set the write direction */
	if ((writeDirection == NORMAL_WRITE_DIRECTION) ||
			(writeDirection == REVERSE_WRITE_DIRECTION)) {
		m_ID = writeDirection;
	} else if (writeDirection != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Set the number of lines */
	if ((numberOfLines == DISPLAY_ONE_LINE) ||
			(numberOfLines == DISPLAY_TWO_LINES) ||
			(numberOfLines == DISPLAY_THREE_LINES) ||
			(numberOfLines == DISPLAY_FOUR_LINES)) {
		m_N = numberOfLines & N_MASK;
		m_NW = numberOfLines & NW_MASK;
	} else if (numberOfLines != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Set the CGRAM/SEGRAM character blink mode */
	if ((blinkEnable == BLINK_ENABLED) || (blinkEnable == BLINK_DISABLED)) {
		m_BE = blinkEnable;
	} else if (blinkEnable != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Set the character movement */
	if ((shiftOrScroll == DISPLAY_SHIFT_ENABLED) ||
			(shiftOrScroll == DOT_SCROLL_ENABLED)) {
		m_DHd = shiftOrScroll;
	} else if (shiftOrScroll != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Set the lines where the shift/scroll is enabled */
	if (shiftOrScrollLinesEnabled <= SHIFT_SCROLL_LINES_MAX) {
		m_DHSx = shiftOrScrollLinesEnabled;
	} else if (shiftOrScrollLinesEnabled != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Set the horizontal display direction */
	if ((horizontalDisplayDirection == NORMAL_HORIZONTAL_DIRECTION) ||
			(horizontalDisplayDirection == REVERSE_HORIZONTAL_DIRECTION)) {
		m_BDS = horizontalDisplayDirection;
	} else if (horizontalDisplayDirection != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Set the vertical display direction */
	if ((verticalDisplayDirection == NORMAL_VERTICAL_DIRECTION) ||
			(verticalDisplayDirection == REVERSE_VERTICAL_DIRECTION)) {
		m_BDC = verticalDisplayDirection;
	} else if (verticalDisplayDirection != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Update the MPU display settings */
	uint8_t commands[10];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	commands[1] = COMMAND_ENTRY_MODE_SET | m_ID | m_S;
	commands[2] = COMMAND_FUNCTION_SET | m_N | m_BE | RE_1 | m_REV;
	commands[3] = COMMAND_EXTENDED_FUNCTION_SET | m_FW | m_BW | m_NW;
	commands[4] = COMMAND_ENTRY_MODE_SET | m_BDC | m_BDS;
	commands[5] = COMMAND_HEIGHT_BIAS_SHIFT | m_UDx | m_BS1 | m_DHd;
	commands[6] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_1;
	commands[7] = COMMAND_FUNCTION_SET | m_N | m_BE | RE_1 | m_REV;
	commands[8] = COMMAND_SHIFT_SCROLL_ENABLE | m_DHSx;
	commands[9] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	status_t ret = i2c_write(commands, 10, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_display_enable(void) {

	/* Set the power down mode and display state */
	m_PD = POWER_SAVING_MODE_DISABLED;
	m_D = DISPLAY_ENABLED;

	/* Leave sleep mode */
	uint8_t commands[5];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_BE | RE_1 | m_REV;
	commands[1] = COMMAND_POWER_DOWN_MODE | m_PD;
	commands[2] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	status_t ret = i2c_write(commands, 3, NULL, 0);
	if (ret == STATUS_INVALID_HANDLE) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	} else if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}
	pigpio_sleep(0.001);

	/* Turn the display on */
	commands[0] = COMMAND_DISPLAY_ON_OFF_CONTROL | m_D | m_C | m_B;
	ret = i2c_write(commands, 1, NULL, 0);
	if (ret == STATUS_INVALID_HANDLE) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	} else if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}
	pigpio_sleep(0.001);

	/* Increase the contrast and gain */
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_1;
	commands[1] = COMMAND_CONTRAST_SET | m_Cl;
	commands[2] = COMMAND_POWER_ICON_CONTRAST | m_Ion | m_Bon | m_Ch;
	commands[3] = COMMAND_FOLLOWER_CONTROL | m_Don | m_Rabx;
	commands[4] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	ret = i2c_write(commands, 5, NULL, 0);
	if (ret == STATUS_INVALID_HANDLE) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	} else if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	pigpio_sleep(0.025);
	return ret;

}

status_t ssd_display_disable(uint8_t lowPowerMode) {

	/* Set the power down mode */
	if ((lowPowerMode == POWER_SAVING_MODE_DISABLED) ||
			(lowPowerMode == POWER_SAVING_MODE_ENABLED)) {
		m_PD = lowPowerMode;
	}

	/* Set the display state */
	m_D = DISPLAY_DISABLED;

	/* Drop the contrast and gain */
	uint8_t commands[5];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_1;
	commands[1] = COMMAND_POWER_ICON_CONTRAST | m_Ion | m_Bon | CH_0;
	commands[2] = COMMAND_CONTRAST_SET | CL_0;
	commands[3] = COMMAND_FOLLOWER_CONTROL | m_Don | RAB_0;
	commands[4] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	status_t ret = i2c_write(commands, 5, NULL, 0);
	if (ret == STATUS_INVALID_HANDLE) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	} else if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}
	pigpio_sleep(0.025);

	/* Turn the display off */
	commands[0] = COMMAND_DISPLAY_ON_OFF_CONTROL | m_D | m_C | m_B;
	ret = i2c_write(commands, 1, NULL, 0);
	if (ret == STATUS_INVALID_HANDLE) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	} else if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}
	pigpio_sleep(0.001);

	/* Enter sleep mode */
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_BE | RE_1 | m_REV;
	commands[1] = COMMAND_POWER_DOWN_MODE | m_PD;
	commands[2] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	ret = i2c_write(commands, 3, NULL, 0);
	if (ret == STATUS_INVALID_HANDLE) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	} else if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}
	pigpio_sleep(0.001);

	return STATUS_OK;

}

status_t ssd_cursor_configure(
		uint8_t cursorBlink,
		uint8_t cursorBlinkRate,
		uint8_t cursorShadow,
		uint8_t cursorStationary) {

	/* Set the cursor blink mode */
	if ((cursorBlink == CURSOR_BLINK_DISABLED) ||
			(cursorBlink == CURSOR_BLINK_ENABLED)) {
		m_B = cursorBlink;
	} else if (cursorBlink != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Set the cursor blink rate */
	if (cursorBlinkRate <= BLINK_FREQUENCY_MAX) {
		m_Fx = cursorBlinkRate;
	} else if (cursorBlinkRate != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Set the cursor shadow */
	if ((cursorShadow == CURSOR_SHADOW_DISABLED) ||
			(cursorShadow == CURSOR_SHADOW_ENABLED)) {
		m_BW = cursorShadow;
	} else if (cursorShadow != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Set the cursor movement */
	if ((cursorStationary == CURSOR_MOVING) ||
			(cursorStationary == CURSOR_STATIONARY)) {
		m_S = cursorStationary;
	} else if (cursorStationary != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Update the MPU cursor settings */
	uint8_t commands[7];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_1;
	commands[1] = COMMAND_INTERNAL_OSC_FREQUENCY | m_BS0 | m_Fx;
	commands[2] = COMMAND_FUNCTION_SET | m_N | m_BE | RE_1 | m_REV;
	commands[3] = COMMAND_EXTENDED_FUNCTION_SET | m_FW | m_BW | m_NW;
	commands[4] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	commands[5] = COMMAND_DISPLAY_ON_OFF_CONTROL | m_D | m_C | m_B;
	commands[6] = COMMAND_ENTRY_MODE_SET | m_ID | m_S;
	status_t ret = i2c_write(commands, 7, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_cursor_enable(void) {

	/* Set the cursor state */
	m_C = CURSOR_ENABLED;

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Enable the cursor */
	uint8_t commands[2];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	commands[1] = COMMAND_DISPLAY_ON_OFF_CONTROL | m_D | m_C | m_B;
	status_t ret = i2c_write(commands, 2, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_cursor_disable(void) {

	/* Set the cursor state */
	m_C = CURSOR_DISABLED;

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Enable the cursor */
	uint8_t commands[2];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	commands[1] = COMMAND_DISPLAY_ON_OFF_CONTROL | m_D | m_C | m_B;
	status_t ret = i2c_write(commands, 2, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_set_font(
		uint8_t fontHeight,
		uint8_t fontWidth,
		uint8_t fontColour) {

	/* Set the font height */
	if ((fontHeight == DOUBLE_HEIGHT_DISABLED) ||
			(fontHeight == SINGLE_SINGLE_DOUBLE) ||
			(fontHeight == SINGLE_DOUBLE_SINGLE) ||
			(fontHeight == DOUBLE_DOUBLE) ||
			(fontHeight == DOUBLE_SINGLE_SINGLE)) {
		m_DH = fontHeight & DH_MASK;
		m_UDx = fontHeight & UD_MASK;
	} else if (fontHeight != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Set the font width */
	if ((fontWidth == FONT_WIDTH_FIVE_DOTS) ||
			(fontWidth == FONT_WIDTH_SIX_DOTS)) {
		m_FW = fontWidth;
	} else if (fontWidth != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Set the font colour */
	if ((fontColour == FONT_BLACK) || (fontColour == FONT_WHITE)) {
		m_REV = fontColour;
	} else if (fontWidth != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Update the MPU font settings */
	uint8_t commands[5];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	commands[1] = COMMAND_FUNCTION_SET | m_N | m_BE | RE_1 | m_REV;
	commands[2] = COMMAND_HEIGHT_BIAS_SHIFT | m_UDx | m_BS1 | m_DHd;
	commands[3] = COMMAND_EXTENDED_FUNCTION_SET | m_FW | m_BW | m_NW;
	commands[4] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	status_t ret = i2c_write(commands, 5, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_set_brightness(uint8_t displayBrightness) {

	/* Set the display brightness */
	if (displayBrightness <= MAX_BRIGHTNESS) {
		m_Rabx = displayBrightness;
	} else if (displayBrightness != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Update the brightness */
	uint8_t commands[3];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_1;
	commands[1] = COMMAND_FOLLOWER_CONTROL | m_Don | m_Rabx;
	commands[2] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	status_t ret = i2c_write(commands, 3, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_increment_brightness(void) {

	/* Set the display brightness */
	m_Rabx = (uint8_t)(m_Rabx - (m_Rabx != MIN_BRIGHTNESS));

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Update the brightness */
	uint8_t commands[3];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_1;
	commands[1] = COMMAND_FOLLOWER_CONTROL | m_Don | m_Rabx;
	commands[2] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	status_t ret = i2c_write(commands, 3, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_decrement_brightness(void) {

	/* Set the display brightness */
	m_Rabx = (uint8_t)(m_Rabx + (m_Rabx != MAX_BRIGHTNESS));

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Update the brightness */
	uint8_t commands[3];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_1;
	commands[1] = COMMAND_FOLLOWER_CONTROL | m_Don | m_Rabx;
	commands[2] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	status_t ret = i2c_write(commands, 3, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_set_contrast(uint8_t displayContrast) {

	/* Set the display contrast */
	if (displayContrast <= MAX_CONTRAST) {
		m_Cx = displayContrast;
		m_Ch = (m_Cx >> CH_SHIFT) & CH_MASK;
		m_Cl = m_Cx & CL_MASK;
	} else if (displayContrast != UNCHANGED) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Update the contrast */
	uint8_t commands[4];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_1;
	commands[1] = COMMAND_POWER_ICON_CONTRAST | m_Ion | m_Bon | m_Ch;
	commands[2] = COMMAND_CONTRAST_SET | m_Cl;
	commands[3] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	status_t ret = i2c_write(commands, 4, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_increment_contrast(void) {

	/* Set the display contrast */
	m_Cx = (uint8_t)(m_Cx + (m_Cx != MAX_CONTRAST));
	m_Ch = (m_Cx >> CH_SHIFT) & CH_MASK;
	m_Cl = m_Cx & CL_MASK;

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Update the brightness */
	uint8_t commands[4];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_1;
	commands[1] = COMMAND_POWER_ICON_CONTRAST | m_Ion | m_Bon | m_Ch;
	commands[2] = COMMAND_CONTRAST_SET | m_Cl;
	commands[3] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	status_t ret = i2c_write(commands, 4, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_decrement_contrast(void) {

	/* Set the display contrast */
	m_Cx = (uint8_t)(m_Cx - (m_Cx != MIN_CONTRAST));
	m_Ch = (m_Cx >> CH_SHIFT) & CH_MASK;
	m_Cl = m_Cx & CL_MASK;

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	/* Update the brightness */
	uint8_t commands[4];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_1;
	commands[1] = COMMAND_POWER_ICON_CONTRAST | m_Ion | m_Bon | m_Ch;
	commands[2] = COMMAND_CONTRAST_SET | m_Cl;
	commands[3] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	status_t ret = i2c_write(commands, 4, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_clear_display(void) {

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t commands[1];
	commands[0] = COMMAND_CLEAR_DISPLAY;
	status_t ret = i2c_write(commands, 1, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_move_cursor_line1(void) {

	/* Set the DDRAM counter */
	m_AC = MIN_DDRAM;

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t commands[2];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	commands[1] = COMMAND_SET_DDRAM_ADDRESS | m_AC;
	status_t ret = i2c_write(commands, 2, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_move_cursor_line2(void) {

	/* Set the DDRAM counter */
	uint8_t lines = (m_N | m_NW);
	if (lines == DISPLAY_ONE_LINE) {
		LOG_TO_STDERR();
		return STATUS_INVALID_ADDRESS;
	} else if (lines == DISPLAY_TWO_LINES) {
		m_AC = MIN_DDRAM_DOUBLE_LINE_LINE2;
	} else if (lines == DISPLAY_THREE_LINES) {
		m_AC = MIN_DDRAM_TRIPLE_LINE_LINE2;
	} else if (lines == DISPLAY_FOUR_LINES) {
		m_AC = MIN_DDRAM_QUADRUPLE_LINE_LINE2;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t commands[2];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	commands[1] = COMMAND_SET_DDRAM_ADDRESS | m_AC;
	status_t ret = i2c_write(commands, 2, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_move_cursor_line3(void) {

	/* Set the DDRAM counter */
	uint8_t lines = (m_N | m_NW);
	if ((lines == DISPLAY_ONE_LINE) || (lines == DISPLAY_TWO_LINES)) {
		LOG_TO_STDERR();
		return STATUS_INVALID_ADDRESS;
	} else if (lines == DISPLAY_THREE_LINES) {
		m_AC = MIN_DDRAM_TRIPLE_LINE_LINE3;
	} else if (lines == DISPLAY_FOUR_LINES) {
		m_AC = MIN_DDRAM_QUADRUPLE_LINE_LINE3;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t commands[2];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	commands[1] = COMMAND_SET_DDRAM_ADDRESS | m_AC;
	status_t ret = i2c_write(commands, 2, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_move_cursor_line4(void) {

	/* Set the DDRAM counter */
	uint8_t lines = (m_N | m_NW);
	if ((lines == DISPLAY_ONE_LINE) ||
			(lines == DISPLAY_TWO_LINES) ||
			(lines == DISPLAY_THREE_LINES)) {
		LOG_TO_STDERR();
		return STATUS_INVALID_ADDRESS;
	} else if (lines == DISPLAY_FOUR_LINES) {
		m_AC = MIN_DDRAM_QUADRUPLE_LINE_LINE4;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t commands[2];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	commands[1] = COMMAND_SET_DDRAM_ADDRESS | m_AC;
	status_t ret = i2c_write(commands, 2, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_move_cursor_left(void) {

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t commands[2];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	commands[1] = COMMAND_CURSOR_SHIFT | SHIFT_SCROLL_LEFT;
	status_t ret = i2c_write(commands, 2, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_move_cursor_right(void) {

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t commands[2];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	commands[1] = COMMAND_CURSOR_SHIFT | SHIFT_SCROLL_RIGHT;
	status_t ret = i2c_write(commands, 2, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_move_cursor(uint8_t ddramAddress) {

	/* Set the DDRAM counter */
	uint8_t lines = (m_N | m_NW);
	if (lines == DISPLAY_ONE_LINE) {
		if (ddramAddress > MAX_DDRAM_SINGLE_LINE_LINE1) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		}
	} else if (lines == DISPLAY_TWO_LINES) {
		if ((ddramAddress > MAX_DDRAM_DOUBLE_LINE_LINE1) &&
				(ddramAddress < MIN_DDRAM_DOUBLE_LINE_LINE2)) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		} else if (ddramAddress > MAX_DDRAM_DOUBLE_LINE_LINE2) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		}
	} else if (lines == DISPLAY_THREE_LINES) {
		if ((ddramAddress > MAX_DDRAM_TRIPLE_LINE_LINE1) &&
				(ddramAddress < MIN_DDRAM_TRIPLE_LINE_LINE2)) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		} else if ((ddramAddress > MAX_DDRAM_TRIPLE_LINE_LINE2) &&
				(ddramAddress < MIN_DDRAM_TRIPLE_LINE_LINE3)) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		} else if (ddramAddress > MAX_DDRAM_TRIPLE_LINE_LINE3) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		}
	} else if (lines == DISPLAY_FOUR_LINES) {
		if ((ddramAddress > MAX_DDRAM_QUADRUPLE_LINE_LINE1) &&
				(ddramAddress < MIN_DDRAM_QUADRUPLE_LINE_LINE2)) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		} else if ((ddramAddress > MAX_DDRAM_QUADRUPLE_LINE_LINE2) &&
				(ddramAddress < MIN_DDRAM_QUADRUPLE_LINE_LINE3)) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		} else if ((ddramAddress > MAX_DDRAM_QUADRUPLE_LINE_LINE3) &&
				(ddramAddress < MIN_DDRAM_QUADRUPLE_LINE_LINE4)) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		} else if (ddramAddress > MAX_DDRAM_QUADRUPLE_LINE_LINE4) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		}
	}
	m_AC = ddramAddress;

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t commands[2];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	commands[1] = COMMAND_SET_DDRAM_ADDRESS | m_AC;
	status_t ret = i2c_write(commands, 2, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_create_symbol(
		uint8_t *symbolData,
		uint8_t symbolID) {

	/* Set the CGRAM address */
	uint8_t address;
	if (symbolID > MAX_SYMBOL_ID) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	} else {
		address = (uint8_t)(symbolID << SYMBOL_ID_SHIFT);
		m_Symbols[symbolID] = true;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t commands[3];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	commands[1] = COMMAND_ENTRY_MODE_SET | NORMAL_WRITE_DIRECTION | m_S;
	commands[2] = COMMAND_SET_CG_SEG_RAM_ADDRESS | address;
	status_t ret = i2c_write(commands, 3, symbolData, SYMBOL_SIZE);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}

	commands[0] = COMMAND_ENTRY_MODE_SET | m_ID | m_S;
	commands[1] = COMMAND_SET_DDRAM_ADDRESS | m_AC;
	ret = i2c_write(commands, 2, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_write_symbol(uint8_t symbolID) {

	/* Validate the symbol */
	if (symbolID > MAX_SYMBOL_ID) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	} else if (!m_Symbols[symbolID]) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	status_t ret = i2c_write(NULL, 0, &symbolID, 1);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}

	ret = ssd_get_address_counter(&m_AC);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_write_text(char *text) {

	/* Validate the text to be written */
	size_t length = strlen(text);
	if (length > DDRAM_SIZE) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	status_t ret = i2c_write(NULL, 0, (uint8_t *)text, (uint8_t)length);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}

	ret = ssd_get_address_counter(&m_AC);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_shift_display(
		uint8_t shiftAmount,
		double shiftSpeed,
		uint8_t shiftDirection) {

	/* Verify the shift amount */
	if (shiftAmount == 0) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}
	
	/* Verify the shift direction */
	if ((shiftDirection != SHIFT_SCROLL_LEFT) &&
			(shiftDirection != SHIFT_SCROLL_RIGHT)) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Verify that a shift can occur */
	if (m_DHd == DOT_SCROLL_ENABLED) {
		LOG_TO_STDERR();
		return STATUS_INVALID_MODE;
	} else if (m_DHSx == SHIFT_SCROLL_ALL_LINES_DISABLED) {
		LOG_TO_STDERR();
		return STATUS_INVALID_MODE;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t commands[shiftAmount];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	status_t ret = i2c_write(commands, 1, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}
	if (shiftSpeed == SHIFT_SCROLL_IMMEDIATE) {
		for (uint8_t i = 0; i < shiftAmount; i++) {
			commands[i] = COMMAND_DISPLAY_SHIFT | shiftDirection;
		}
		ret = i2c_write(commands, shiftAmount, NULL, 0);
		if (ret != STATUS_OK) {
			LOG_TO_STDERR();
		}
		return ret;
	} else {
		for (uint8_t i = 0; i < shiftAmount; i++) {
			commands[0] = COMMAND_DISPLAY_SHIFT | shiftDirection;
			ret = i2c_write(commands, 1, NULL, 0);
			if (ret != STATUS_OK) {
				LOG_TO_STDERR();
				return ret;
			}
			pigpio_sleep(shiftSpeed);
		}
	}

	return STATUS_OK;

}

status_t ssd_scroll_display(
		uint8_t scrollAmount,
		double scrollSpeed,
		uint8_t scrollDirection) {

	/* Verify scroll direction */
	if ((scrollDirection != SHIFT_SCROLL_LEFT) &&
			(scrollDirection != SHIFT_SCROLL_RIGHT)) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Verify scroll amount */
	if ((scrollDirection == SHIFT_SCROLL_LEFT) &&
			(scrollAmount + m_SQx > MAX_SCROLL)) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	} else if ((scrollDirection == SHIFT_SCROLL_RIGHT) &&
			((uint16_t)m_SQx - scrollAmount < MIN_SCROLL)) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Verify that a shift can occur */
	if (m_DHd == DISPLAY_SHIFT_ENABLED) {
		LOG_TO_STDERR();
		return STATUS_INVALID_MODE;
	} else if (m_DHSx == SHIFT_SCROLL_ALL_LINES_DISABLED) {
		LOG_TO_STDERR();
		return STATUS_INVALID_MODE;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t commands[3];
	if (scrollAmount == MIN_SCROLL) {
		m_SQx = scrollAmount;
		commands[0] = COMMAND_FUNCTION_SET | m_N | m_BE | RE_1 | m_REV;
		commands[1] = COMMAND_SET_SCROLL_QUANTITY | m_SQx;
		commands[2] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
		return i2c_write(commands, 3, NULL, 0);
	} else if (scrollSpeed == SHIFT_SCROLL_IMMEDIATE) {
		if (scrollDirection == SHIFT_SCROLL_LEFT) {
			m_SQx = (uint8_t)(m_SQx + scrollAmount);
		} else {
			m_SQx = (uint8_t)(m_SQx + scrollAmount);
		}
		commands[0] = COMMAND_FUNCTION_SET | m_N | m_BE | RE_1 | m_REV;
		commands[1] = COMMAND_SET_SCROLL_QUANTITY | m_SQx;
		commands[2] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
		status_t ret = i2c_write(commands, 3, NULL, 0);
		if (ret != STATUS_OK) {
			LOG_TO_STDERR();
		}
		return ret;
	}

	commands[0] = COMMAND_FUNCTION_SET | m_N | m_BE | RE_1 | m_REV;
	status_t ret = i2c_write(commands, 1, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}

	for (uint8_t i = 0; i < scrollAmount; i++) {
		if (scrollDirection == SHIFT_SCROLL_LEFT) {
			m_SQx = (uint8_t)(m_SQx + scrollAmount);
		} else {
			m_SQx = (uint8_t)(m_SQx + scrollAmount);
		}
		commands[0] = COMMAND_SET_SCROLL_QUANTITY | m_SQx;
		status_t ret = i2c_write(commands, 1, NULL, 0);
		if (ret != STATUS_OK) {
			LOG_TO_STDERR();
			return ret;
		}
		pigpio_sleep(scrollSpeed);
	}

	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	ret = i2c_write(commands, 1, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_get_busy_flag(uint8_t *busyFlag) {

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t data[2];
	status_t ret = i2c_read(READ_AC_ID, data, LENGTH_AC_ID);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}
	*busyFlag = data[1] & BF_MASK;
	return STATUS_OK;

}

status_t ssd_get_address_counter(uint8_t *currentAddress) {

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t data[2];
	status_t ret = i2c_read(READ_AC_ID, data, LENGTH_AC_ID);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}
	*currentAddress = data[0] & AC_MASK;
	return STATUS_OK;

}

status_t ssd_get_part_id(uint8_t *partID) {

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t data[2];
	status_t ret = i2c_read(READ_AC_ID, data, LENGTH_AC_ID);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}
	*partID = data[1] & ID_MASK;
	return STATUS_OK;

}

status_t ssd_read_from_DDRAM(
		uint8_t address,
		uint8_t *data,
		uint16_t dataLength) {

	/* Set the DDRAM counter */
	uint8_t lines = (m_N | m_NW);
	if (address == UNCHANGED) {
		// Do not update the address counter
	} else if (lines == DISPLAY_ONE_LINE) {
		if (address > MAX_DDRAM_SINGLE_LINE_LINE1) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		}
	} else if (lines == DISPLAY_TWO_LINES) {
		if ((address > MAX_DDRAM_DOUBLE_LINE_LINE1) &&
				(address < MIN_DDRAM_DOUBLE_LINE_LINE2)) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		} else if (address > MAX_DDRAM_DOUBLE_LINE_LINE2) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		}
	} else if (lines == DISPLAY_THREE_LINES) {
		if ((address > MAX_DDRAM_TRIPLE_LINE_LINE1) &&
				(address < MIN_DDRAM_TRIPLE_LINE_LINE2)) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		} else if ((address > MAX_DDRAM_TRIPLE_LINE_LINE2) &&
				(address < MIN_DDRAM_TRIPLE_LINE_LINE3)) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		} else if (address > MAX_DDRAM_TRIPLE_LINE_LINE3) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		}
	} else if (lines == DISPLAY_FOUR_LINES) {
		if ((address > MAX_DDRAM_QUADRUPLE_LINE_LINE1) &&
				(address < MIN_DDRAM_QUADRUPLE_LINE_LINE2)) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		} else if ((address > MAX_DDRAM_QUADRUPLE_LINE_LINE2) &&
				(address < MIN_DDRAM_QUADRUPLE_LINE_LINE3)) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		} else if ((address > MAX_DDRAM_QUADRUPLE_LINE_LINE3) &&
				(address < MIN_DDRAM_QUADRUPLE_LINE_LINE4)) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		} else if (address > MAX_DDRAM_QUADRUPLE_LINE_LINE4) {
			LOG_TO_STDERR();
			return STATUS_INVALID_PARAM;
		}
	}

	/* Validate the data length */
	if ((dataLength == MIN_DDRAM) || (dataLength > DDRAM_SIZE)) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t commands[1];
	uint8_t dataTemp[dataLength + 1];
	commands[0] = COMMAND_SET_DDRAM_ADDRESS | address;
	status_t ret = i2c_write(commands, 1, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}

	ret = i2c_read(READ_RAM, dataTemp, (uint8_t)(dataLength + 1));
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}

	memcpy(data, dataTemp + 1, dataLength);
	commands[0] = COMMAND_SET_DDRAM_ADDRESS | m_AC;
	ret = i2c_write(commands, 1, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_read_from_CGRAM(
		uint8_t address,
		uint8_t *data,
		uint16_t dataLength) {
	
	/* Validate address */
	if (address > MAX_CGRAM) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Validate data length */
	if ((dataLength == MIN_CGRAM) || (dataLength > CGRAM_SIZE)) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t commands[2];
	uint8_t dataTemp[dataLength + 1];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	commands[1] = COMMAND_SET_CG_SEG_RAM_ADDRESS | address;
	status_t ret = i2c_write(commands, 2, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}

	ret = i2c_read(READ_RAM, dataTemp, (uint8_t)(dataLength + 1));
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}

	memcpy(data, dataTemp, dataLength);
	commands[0] = COMMAND_SET_DDRAM_ADDRESS | m_AC;
	ret = i2c_write(commands, 1, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_read_from_SEGRAM(
		uint8_t address,
		uint8_t *data,
		uint16_t dataLength) {
	
	/* Validate address */
	if (address > MAX_SEGRAM) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* Validate data length */
	if ((dataLength == MIN_SEGRAM) || (dataLength > SEGRAM_SIZE)) {
		LOG_TO_STDERR();
		return STATUS_INVALID_PARAM;
	}

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	uint8_t commands[2];
	uint8_t dataTemp[dataLength + 1];
	commands[0] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_1;
	commands[1] = COMMAND_SET_CG_SEG_RAM_ADDRESS | address;
	status_t ret = i2c_write(commands, 2, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}

	ret = i2c_read(READ_RAM, dataTemp, (uint8_t)(dataLength + 1));
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
		return ret;
	}

	memcpy(data, dataTemp, dataLength);
	commands[0] = COMMAND_SET_DDRAM_ADDRESS | m_AC;
	commands[1] = COMMAND_FUNCTION_SET | m_N | m_DH | RE_0 | IS_0;
	ret = i2c_write(commands, 2, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}

status_t ssd_write_command_raw(uint8_t command) {

	/* pigpio library not initialised */
	if (!pigpio_is_initialised()) {
		LOG_TO_STDERR();
		return STATUS_NOT_INITIALISED;
	}

	status_t ret = i2c_write(&command, 1, NULL, 0);
	if (ret != STATUS_OK) {
		LOG_TO_STDERR();
	}
	return ret;

}