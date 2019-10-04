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
#include <stdint.h>

/* === Defines ============================================================== */
/* I2C bus */
#define I2C_BUS_0                       0
#define I2C_BUS_1                       1

/* I2C address LSB(it) */
#define SA0_ADDRESS_0x3C                0
#define SA0_ADDRESS_0x3D                1
#define SA0_LOW                         0
#define SA0_HIGH                        1

/* SSD1803A reset pin */
#define NO_HARDWARE_RESET_PIN			(0xFF)

/* GPIO active level */
#define ACTIVE_LOW						false
#define ACTIVE_HIGH						true

/* Symbol identifiers */
#define SYMBOL1_ID						(0x00)
#define SYMBOL2_ID						(0x01)
#define SYMBOL3_ID						(0x02)
#define SYMBOL4_ID						(0x03)
#define SYMBOL5_ID						(0x04)
#define SYMBOL6_ID						(0x05)
#define SYMBOL7_ID						(0x06)
#define SYMBOL8_ID						(0x07)

/* Shift speed */
#define SHIFT_SCROLL_IMMEDIATE			(0x00)

/* Command bit defines */
#define DEFAULT							(0xFF)
#define UNCHANGED						(0xFF)

#define POWER_SAVING_MODE_DISABLED		(0x00)		// PD = 0
#define POWER_SAVING_MODE_ENABLED		(0x01)		// PD = 1

#define CURSOR_MOVING					(0x00)		// S = 0
#define CURSOR_STATIONARY				(0x01)		// S = 1

#define REVERSE_WRITE_DIRECTION			(0x00)		// I/D = 0
#define NORMAL_WRITE_DIRECTION			(0x02)		// I/D = 1

#define REVERSE_HORIZONTAL_DIRECTION	(0x00)		// BDS = 0
#define NORMAL_HORIZONTAL_DIRECTION		(0x01)		// BDS = 1

#define NORMAL_VERTICAL_DIRECTION		(0x00)		// BDC = 0
#define REVERSE_VERTICAL_DIRECTION		(0x02)		// BDC = 1

#define CURSOR_BLINK_DISABLED			(0x00)		// B = 0
#define CURSOR_BLINK_ENABLED			(0x01)		// B = 1

#define CURSOR_DISABLED					(0x00)		// C = 0
#define CURSOR_ENABLED					(0x02)		// C = 1

#define DISPLAY_DISABLED				(0x00)		// D = 0
#define DISPLAY_ENABLED					(0x04)		// D = 1

#define DISPLAY_ONE_LINE				(0x00)		// NW = 0, N = 0
#define DISPLAY_THREE_LINES				(0x01)		// NW = 1, N = 0
#define DISPLAY_TWO_LINES				(0x08)		// NW = 0, N = 1
#define DISPLAY_FOUR_LINES				(0x09)		// NW = 1, N = 1

#define CURSOR_SHADOW_ENABLED			(0x00)		// B/W = 0
#define CURSOR_SHADOW_DISABLED			(0x02)		// B/W = 1

#define FONT_WIDTH_FIVE_DOTS			(0x00)		// FW = 0
#define FONT_WIDTH_SIX_DOTS				(0x04)		// FW = 1

#define SHIFT_SCROLL_LEFT				(0x00)		// R/L = 0
#define SHIFT_SCROLL_RIGHT				(0x04)		// R/L = 1

#define CURSOR_SHIFT					(0x00)		// S/C = 0
#define DISPLAY_SHIFT					(0x08)		// S/C = 1

#define DOT_SCROLL_ENABLED				(0x00)		// DH' = 0
#define DISPLAY_SHIFT_ENABLED			(0x01)		// DH' = 1

#define BIAS_1_4						(0x08)		// BS1 = 0, BS0 = 1
#define BIAS_1_5						(0x00)		// BS1 = 0, BS0 = 0
#define BIAS_1_6						(0x0A)		// BS1 = 1, BS0 = 1
#define BIAS_1_7						(0x02)		// BS1 = 1, BS0 = 0

#define DOUBLE_HEIGHT_DISABLED			(0x0C)		// DH = 0, UD2 = 1, UD1 = 1
#define SINGLE_SINGLE_DOUBLE			(0x01)		// DH = 1, UD2 = 0, UD1 = 0
#define SINGLE_DOUBLE_SINGLE			(0x05)		// DH = 1, UD2 = 0, UD1 = 1
#define DOUBLE_DOUBLE					(0x09)		// DH = 1, UD2 = 1, UD1 = 0
#define DOUBLE_SINGLE_SINGLE			(0x0D)		// DH = 1, UD2 = 1, UD1 = 1

#define BLINK_FREQUENCY_2100mHz			(0x00)		// F2 = 0, F1 = 0, F0 = 0
#define BLINK_FREQUENCY_2300mHz			(0x01)		// F2 = 0, F1 = 0, F0 = 1
#define BLINK_FREQUENCY_2500mHz			(0x02)		// F2 = 0, F1 = 1, F0 = 0
#define BLINK_FREQUENCY_2700mHz			(0x03)		// F2 = 0, F1 = 1, F0 = 1
#define BLINK_FREQUENCY_2900mHz			(0x04)		// F2 = 1, F1 = 0, F0 = 0
#define BLINK_FREQUENCY_3100mHz			(0x05)		// F2 = 1. F1 = 0, F0 = 1
#define BLINK_FREQUENCY_3200mHz			(0x06)		// F2 = 1, F1 = 1, F0 = 0
#define BLINK_FREQUENCY_3400mHz			(0x07)		// F2 = 1, F1 = 1, F0 = 1
#define BLINK_FREQUENCY_MAX				(0x07)

#define SHIFT_SCROLL_ALL_LINES_DISABLED	(0x00)		// DSx = HSx = 0 for x = 1-4
#define SHIFT_SCROLL_LINE1_ENABLED		(0x01)		// DS1 = HS1 = 1
#define SHIFT_SCROLL_LINE2_ENABLED		(0x02)		// DS2 = HS2 = 1
#define SHIFT_SCROLL_LINE3_ENABLED		(0x04)		// DS3 = HS3 = 1
#define SHIFT_SCROLL_LINE4_ENABLED		(0x08)		// DS4 = HS4 = 1
#define SHIFT_SCROLL_ALL_LINES_ENABLED	(0x0F)		// DSx = HSx = 1 for x = 1-4
#define SHIFT_SCROLL_LINES_MAX			(0x0F)

#define FONT_BLACK						(0x00)		// REV = 0
#define FONT_WHITE						(0x01)		// REV = 1

#define BLINK_DISABLED					(0x00)		// BE = 0
#define BLINK_ENABLED					(0x04)		// BE = 1

#define BOOSTER_AND_REGULATOR_EXTERNAL	(0x00)		// Bon = 0
#define BOOSTER_AND_REGULATOR_INTERNAL	(0x04)		// Bon = 1

#define ICON_DISPLAY_DISABLED			(0x00)		// Ion = 0
#define ICON_DISPLAY_ENABLED			(0x08)		// Ion = 1

#define DIVIDER_CIRCUIT_EXTERNAL		(0x00)		// Don = 0
#define DIVIDER_CIRCUIT_INTERNAL		(0x08)		// Don = 1

#define MPU_READY						(0x00)		// BF = 0
#define MPU_BUSY						(0x80)		// BF = 1

#define NEG_0_05_PERCENT_PER_DEG_C		(0x02)		// TC2 = 0, TC1 = 1, TC0 = 0
#define NEG_0_10_PERCENT_PER_DEG_C		(0x04)		// TC2 = 1, TC1 = 0, TC0 = 0
#define NEG_0_15_PERCENT_PER_DEG_C		(0x06)		// TC2 = 1, TC1 = 1, TC0 = 0
#define NEG_0_20_PERCENT_PER_DEG_C		(0x07)		// TC2 = 1, TC1 = 1, TC0 = 1

#define ROM_A							(0x00)		// ROM2 = 0, ROM1 = 0
#define ROM_B							(0x04)		// ROM2 = 0, ROM1 = 1
#define ROM_C							(0x08)		// ROM2 = 1, ROM1 = 0

/* === Enumerations ========================================================= */
enum StatusCodes {
	STATUS_FAILED_READ					= -0x53,	// -83
	STATUS_FAILED_WRITE					= -0x52,	// -82
	STATUS_INVALID_PARAM				= -0x51,	// -81
	STATUS_INVALID_FLAGS                = -0x4D,	// -77
	STATUS_INVALID_ADDRESS              = -0x4B,	// -75
	STATUS_INVALID_BUS                  = -0x4A,	// -74
	STATUS_FAILED_OPEN                  = -0x47,	// -71
	STATUS_INVALID_HANDLE               = -0x19,	// -25
	STATUS_NO_HANDLE                    = -0x18,	// -24
	STATUS_INVALID_PUD					= -0x06,	// -6
	STATUS_INVALID_LEVEL				= -0x05,	// -5
	STATUS_INVALID_MODE					= -0x04,	// -4
	STATUS_INVALID_GPIO					= -0x03,	// -3
	STATUS_NOT_INITIALISED              = -0x01,	// -1
	STATUS_OK,
	STATUS_ALREADY_OPEN,
};

/* === Structures =========================================================== */

/* === Typedefs ============================================================= */
typedef enum StatusCodes                status_t;
typedef bool							i2c_bus_t;
typedef bool							sa0_bit_t;
typedef bool							active_level_t;

/* === Global Variables ===================================================== */

/* === Function Prototypes ================================================== */

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Initialise the GPIO interface that allows for hardware control over
 *				the enabling/disabling of the display.
 * 		-# Initialise the I2C communication interface to the SSD1803A MPU by
 * 				using the specified Raspberry Pi I2C bus (rpiI2CBus) and slave
 * 				address LSBit (sa0).
 *		-# Update the following register bits to the specified values:
 *				- BS0/BS1: Bias setting of the internal divider.
 *				- Bon: Type of booster and regulator circuitry.
 *				- Don: Type of divider circuitry.
 *				- TC2/TC1/TC0: Temperature compensation circuitry coefficient.
 *				- ROM1/ROM2: ROM used for character generation.
 *
 * @param [in]		rpiResetPin GPIO pin number on the RPi to which the RESET
 * pin on the SSD1803A is connected (in some way).  Valid values for this
 * parameter vary depending on the RPi hardware version, however in general the
 * following should apply:
 * 		- 0-53: Valid RPi GPIO numbers.
 * 		- NO_HARDWARE_RESET_PIN: No hardware reset pin is to be used.  Specify
 * 				this value if the RESET pin on the SSD1803A is tied directly to
 * 				Vcc.
 * @param [in]		resetPinActiveLevel GPIO level at which the SSD1803A will be
 * active.  Valid values are:
 *		- ACTIVE_LOW: The SSD1803A is in normal operating mode when rpiResetPin
 *				is low and in a reset state when rpiResetPin is high.
 *		- ACTIVE_HIGH: The SSD1803A is in normal operating mode when rpiResetPin
 *				is high and in a reset state when rpiResetPin is low.
 * @param [in]		rpiI2CBus I2C bus number on the RPi that is connected to the
 * I2C bus on the display MPU.  Valid values are:
 * 		- I2C_BUS_0
 * 		- I2C_BUS_1
 * @param [in]		sa0 Least significant bit of the I2C address at which the
 * display MPU can be found.  Valid values are:
 * 		- SA0_LOW
 * 		- SA0_HIGH
 * @param [in]		internalDividerBias Bias of the internal divider.  Valid
 * values are:
 * 		- BIAS_1_4: the bias of the internal divider is set to 1/4.
 * 		- BIAS_1_5: the bias of the internal divider is set to 1/5.
 * 		- BIAS_1_6: the bias of the internal divider is set to 1/6.
 * 		- BIAS_1_7: the bias of the internal divider is set to 1/7.
 * 		- DEFAULT: the bias of the internal divider is set to the power on reset
 * 				value (1/5).
 * @param [in]		boosterAndRegulatorCircuit Type of booster and regulator
 * circuitry.  Valid values are:
 * 		- BOOSTER_AND_REGULATOR_EXTERNAL: an external voltage will be applied to
 * 				the pin V0.
 *		- BOOSTER_AND_REGULATOR_INTERNAL: the internal booster and regulator
 *				will be used to power pin V0.
 * @param [in]		dividerCircuit Type of divider circuitry.  Valid values are:
 *		- DIVIDER_CIRCUIT_EXTERNAL: an external resistor divider will be
 *				connected between pin V0, V1, V2, V3, V4, and VSS.
 *		- DIVIDER_CIRCUIT_INTERNAL: the internal resistor divider network will
 *				be connected the pins V0, V1, V2, V3, V4, and VSS.
 * @param [in]		temperatureCoefficient Coefficient of the temperature
 * compensation circuitry.  Valid values are:
 *		- NEG_0_05_PERCENT_PER_DEG_C: the temperature coefficient is set to
 *				-0.05%/degC.
 *		- NEG_0_10_PERCENT_PER_DEG_C: the temperature coefficient is set to
 *				-0.10%/degC.
 *		- NEG_0_15_PERCENT_PER_DEG_C: the temperature coefficient is set to
 *				-0.15%/degC.
 *		- NEG_0_15_PERCENT_PER_DEG_C: the temperature coefficient is set to
 *				-0.20%/degC.
 * @param [in]		rom ROM used for character generation.  Valid values are:
 * 		- ROM_A: CGROM is set to ROM A.
 *		- ROM_B: CGROM is set to ROM B.
 *		- ROM_C: CGROM is set to ROM C.
 *
 * @retval STATUS_OK If the SSD1803A was successfully initialised.
 * @retval STATUS_INVALID_PARAM If one of the input parameters was set to an
 * invalid value.
 * @retval STATUS_ALREADY_OPEN If this function (ssd_init()) had already been
 * called and successfully opened an I2C interface to the display MPU.
 * @retval STATUS_NOT_INITIALISED If the pigpio library failed to be
 * successfully initialised.
 * @retval STATUS_NO_HANDLE If there is no available handle to assign to the
 * display MPU I2C bus.
 * @retval STATUS_FAILED_OPEN If I2C interface failed to be opened.
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU over I2C.
 * @retval STATUS_INVALID_HANDLE, STATUS_INVALID_BUS, STATUS_INVALID_ADDRESS,
 * and STATUS_INVALID_FLAGS are theoretically possible return values however
 * they should never occur.
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
		uint8_t rom
);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_deinit(void);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_display_configure(
		uint8_t writeDirection,
		uint8_t numberOfLines,
		uint8_t blinkEnable,
		uint8_t shiftOrScroll,
		uint8_t shiftOrScrollLinesEnabled,
		uint8_t horizontalDisplayDirection,
		uint8_t verticalDisplayDirection
);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_display_enable(void);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_display_disable(uint8_t lowPowerMode);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_cursor_configure(
		uint8_t cursorBlink,
		uint8_t cursorBlinkRate,
		uint8_t cursorShadow,
		uint8_t cursorStationary
);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_cursor_enable(void);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_cursor_disable(void);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_set_font(
		uint8_t fontHeight,
		uint8_t fontWidth,
		uint8_t fontColour
); // TODO: Enable DH bit if height needs

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_set_brightness(uint8_t displayBrightness);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_increment_brightness(void);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_decrement_brightness(void);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_set_contrast(uint8_t displayConstrast);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_increment_contrast(void);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_decrement_contrast(void);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_clear_display(void);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_move_cursor_line1(void);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_move_cursor_line2(void);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_move_cursor_line3(void);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_move_cursor_line4(void);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_move_cursor_left(void);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_move_cursor_right(void);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_move_cursor(uint8_t ddramAddress);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_create_symbol(
		uint8_t *symbolData,
		uint8_t symbolID
);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_write_symbol(uint8_t symbolID);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_write_text(char *text);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_shift_display(
		uint8_t shiftAmount,
		double shiftSpeed,
		uint8_t shiftDirection
);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_scroll_display(
		uint8_t scrollAmount,
		double scrollSpeed, // TODO: 0 equals immediately scroll scrollAmount
		uint8_t scrollDirection
);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_get_busy_flag(uint8_t *busyFlag);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_get_address_counter(uint8_t *currentAddress);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_get_part_id(uint8_t *partID);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_read_from_DDRAM(
		uint8_t address,
		uint8_t *data,
		uint16_t dataLength
);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_read_from_CGRAM(
		uint8_t address,
		uint8_t *data,
		uint16_t dataLength
);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_read_from_SEGRAM(
		uint8_t address,
		uint8_t *data,
		uint16_t dataLength
);

/* ========================================================================== */
/**@brief TODO: Insert brief.
 * ========================================================================== */
status_t ssd_write_command_raw(uint8_t command);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // SRC_SSD1803A_H_
