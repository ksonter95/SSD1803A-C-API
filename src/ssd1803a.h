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
 * ==========================================================================\n
 * Description: \n
 * This library provides an API to intuitively and efficiently interact with the
 * SSD1803A display MPU from a Raspberry Pi.  As such, it provides the following
 * functions:
 * 		- ssd_init(): Initialises the SSD1803A display MPU.
 * 		- ssd_deinit(): Deinitialises the SSD1803A display MPU.
 * 		- ssd_display_configure(): Configures the display related settings of
 * 				the SSD1803A.
 * 		- ssd_display_enable(): Enables the display of the SSD1803A.
 * 		- ssd_display_disable(): Disables the display of the SSD1803A.
 * 		- ssd_cursor_configure(): Configures the cursor related settings of the
 * 				SSD1803A.
 * 		- ssd_cursor_enable(): Enables the cursor of the SSD1803A.
 * 		- ssd_cursor_disable(): Disables the cursor of the SSD1803A.
 * 		- ssd_set_font(): Configures the font related settings of the SSD1803A.
 * 		- ssd_set_brightness(): Sets the brightness of the SSD1803A LCD.
 * 		- ssd_increment_brightness(): Increments the brightness of the SSD1803A
 * 				LCD by one.
 * 		- ssd_decrement_brightness(): Decrements the brightness of the SSD1803A
 * 				LCD by one.
 * 		- ssd_set_contrast(): Sets the contrast of the SSD1803A LCD.
 * 		- ssd_increment_contrast(): Increments the contrast of the SSD1803A LCD
 * 				by one.
 * 		- ssd_decrement_contrast(): Decrements the contrast of the SSD1803A LCD
 * 				by one.
 * 		- ssd_clear_display(): Clears the display of the SSD1803A.
 * 		- ssd_move_cursor_line1(): Moves the cursor to the beginning of the
 * 				first line.
 * 		- ssd_move_cursor_line2(): Moves the cursor to the beginning of the
 * 				second line.
 * 		- ssd_move_cursor_line3(): Moves the cursor to the beginning of the
 * 				third line.
 * 		- ssd_move_cursor_line4(): Moves the cursor to the beginning of the
 * 				fourth line.
 * 		- ssd_move_cursor_right(): Moves the cursor one position to the right.
 * 		- ssd_move_cursor_left(): Moves the cursor one position to the left.
 * 		- ssd_move_cursor(): Moves the cursor to the specified line and
 * 				position.
 * 		- ssd_create_symbol(): Creates and saves a custom symbol to CGRAM.
 * 		- ssd_write_symbol(): Writes the custom symbol to the SSD1803A LCD.
 * 		- ssd_write_text(): Writes the specified text to the SSD1803A LCD.
 * 		- ssd_shift_display(): Shifts all enabled lines.
 * 		- ssd_scroll_display(): Scrolls all enabled lines.
 * 		- ssd_get_busy_flag(): Obtains the busy flag.
 * 		- ssd_get_address_counter(): Obtains the current address counter.
 * 		- ssd_get_part_id(): Obtains the part ID of the SSD1803A.
 * 		- ssd_read_from_DDRAM(): Reads data from the specified DDRAM
 * 				address(es).
 * 		- ssd_read_from_CGRAM(): Reads data from the specified CGRAM
 * 				address(es).
 * 		- ssd_read_from_SEGRAM(): Reads data from the specified SEGRAM
 * 				address(es).
 * 		- ssd_write_command_raw(): Writes a raw command over I2C to the SSD1803A
 * 				display MPU.
 * 
 * For any further information on these functions, see the function comments
 * shown below.
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
#define SSD_DEBUG
#ifdef SSD_DEBUG
#define LOG_TO_STDERR()	{ \
		fprintf(stderr, "%s:%d - %s()\n", __FILE__, __LINE__, __func__); \
		fflush(stderr); \
}
#else
#define LOG_TO_STDERR()
#endif // SSD_DEBUG

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

/* Shift/scroll reset */
#define SHIFT_SCROLL_RESET				(0x00)

/* Shift speed */
#define SHIFT_SCROLL_IMMEDIATE			(0x00)

/* Symbol identifiers */
#define SYMBOL1_ID						(0x00)
#define SYMBOL2_ID						(0x01)
#define SYMBOL3_ID						(0x02)
#define SYMBOL4_ID						(0x03)
#define SYMBOL5_ID						(0x04)
#define SYMBOL6_ID						(0x05)
#define SYMBOL7_ID						(0x06)
#define SYMBOL8_ID						(0x07)

/* Line identifiers */
#define LINE_1							(0x00)
#define LINE_2							(0x01)
#define LINE_3							(0x02)
#define LINE_4							(0x03)

/* Position identifiers */
#define POSITION_1						(0x00)
#define POSITION_2						(0x01)
#define POSITION_3						(0x02)
#define POSITION_4						(0x03)
#define POSITION_5						(0x04)
#define POSITION_6						(0x05)
#define POSITION_7						(0x06)
#define POSITION_8						(0x07)
#define POSITION_9						(0x08)
#define POSITION_10						(0x09)
#define POSITION_11						(0x0A)
#define POSITION_12						(0x0B)
#define POSITION_13						(0x0C)
#define POSITION_14						(0x0D)
#define POSITION_15						(0x0E)
#define POSITION_16						(0x0F)
#define POSITION_17						(0x10)
#define POSITION_18						(0x11)
#define POSITION_19						(0x12)
#define POSITION_20						(0x13)
#define POSITION_21						(0x14)
#define POSITION_22						(0x15)
#define POSITION_23						(0x16)
#define POSITION_24						(0x17)
#define POSITION_25						(0x18)
#define POSITION_26						(0x19)
#define POSITION_27						(0x1A)
#define POSITION_28						(0x1B)
#define POSITION_29						(0x1C)
#define POSITION_30						(0x1D)
#define POSITION_31						(0x1E)
#define POSITION_32						(0x1F)
#define POSITION_33						(0x20)
#define POSITION_34						(0x21)
#define POSITION_35						(0x22)
#define POSITION_36						(0x23)
#define POSITION_37						(0x24)
#define POSITION_38						(0x25)
#define POSITION_39						(0x26)
#define POSITION_40						(0x27)
#define POSITION_41						(0x28)
#define POSITION_42						(0x29)
#define POSITION_43						(0x2A)
#define POSITION_44						(0x2B)
#define POSITION_45						(0x2C)
#define POSITION_46						(0x2D)
#define POSITION_47						(0x2E)
#define POSITION_48						(0x2F)
#define POSITION_49						(0x30)
#define POSITION_50						(0x31)
#define POSITION_51						(0x32)
#define POSITION_52						(0x33)
#define POSITION_53						(0x34)
#define POSITION_54						(0x35)
#define POSITION_55						(0x36)
#define POSITION_56						(0x37)
#define POSITION_57						(0x38)
#define POSITION_58						(0x39)
#define POSITION_59						(0x3A)
#define POSITION_60						(0x3B)
#define POSITION_61						(0x3C)
#define POSITION_62						(0x3D)
#define POSITION_63						(0x3E)
#define POSITION_64						(0x3F)
#define POSITION_65						(0x40)
#define POSITION_66						(0x41)
#define POSITION_67						(0x42)
#define POSITION_68						(0x43)
#define POSITION_69						(0x44)
#define POSITION_70						(0x45)
#define POSITION_71						(0x46)
#define POSITION_72						(0x47)
#define POSITION_73						(0x48)
#define POSITION_74						(0x49)
#define POSITION_75						(0x4A)
#define POSITION_76						(0x4B)
#define POSITION_77						(0x4C)
#define POSITION_78						(0x4D)
#define POSITION_79						(0x4E)
#define POSITION_80						(0x4F)

/* Command bit defines */
#define DEFAULT							(0xFF)
#define UNCHANGED						(0xFF)

#define POWER_SAVING_MODE_DISABLED		(0x00)	// PD = 0
#define POWER_SAVING_MODE_ENABLED		(0x01)	// PD = 1

#define CURSOR_MOVING					(0x00)	// S = 0
#define CURSOR_STATIONARY				(0x01)	// S = 1

#define REVERSE_WRITE_DIRECTION			(0x00)	// I/D = 0
#define NORMAL_WRITE_DIRECTION			(0x02)	// I/D = 1

#define REVERSE_HORIZONTAL_DIRECTION	(0x00)	// BDS = 0
#define NORMAL_HORIZONTAL_DIRECTION		(0x01)	// BDS = 1

#define NORMAL_VERTICAL_DIRECTION		(0x00)	// BDC = 0
#define REVERSE_VERTICAL_DIRECTION		(0x02)	// BDC = 1

#define CURSOR_BLINK_DISABLED			(0x00)	// B = 0
#define CURSOR_BLINK_ENABLED			(0x01)	// B = 1

#define CURSOR_DISABLED					(0x00)	// C = 0
#define CURSOR_ENABLED					(0x02)	// C = 1

#define DISPLAY_DISABLED				(0x00)	// D = 0
#define DISPLAY_ENABLED					(0x04)	// D = 1

#define DISPLAY_ONE_LINE				(0x00)	// NW = 0, N = 0
#define DISPLAY_THREE_LINES				(0x01)	// NW = 1, N = 0
#define DISPLAY_TWO_LINES				(0x08)	// NW = 0, N = 1
#define DISPLAY_FOUR_LINES				(0x09)	// NW = 1, N = 1

#define CURSOR_SHADOW_ENABLED			(0x00)	// B/W = 0
#define CURSOR_SHADOW_DISABLED			(0x02)	// B/W = 1

#define FONT_WIDTH_FIVE_DOTS			(0x00)	// FW = 0
#define FONT_WIDTH_SIX_DOTS				(0x04)	// FW = 1

#define SHIFT_SCROLL_LEFT				(0x00)	// R/L = 0
#define SHIFT_SCROLL_RIGHT				(0x04)	// R/L = 1

#define CURSOR_SHIFT					(0x00)	// S/C = 0
#define DISPLAY_SHIFT					(0x08)	// S/C = 1

#define DOT_SCROLL_ENABLED				(0x00)	// DH' = 0
#define DISPLAY_SHIFT_ENABLED			(0x01)	// DH' = 1

#define BIAS_1_4						(0x08)	// BS1 = 0, BS0 = 1
#define BIAS_1_5						(0x00)	// BS1 = 0, BS0 = 0
#define BIAS_1_6						(0x0A)	// BS1 = 1, BS0 = 1
#define BIAS_1_7						(0x02)	// BS1 = 1, BS0 = 0

#define DOUBLE_HEIGHT_DISABLED			(0x0C)	// DH = 0, UD2 = 1, UD1 = 1
#define SINGLE_SINGLE_DOUBLE			(0x01)	// DH = 1, UD2 = 0, UD1 = 0
#define SINGLE_DOUBLE_SINGLE			(0x05)	// DH = 1, UD2 = 0, UD1 = 1
#define DOUBLE_DOUBLE					(0x09)	// DH = 1, UD2 = 1, UD1 = 0
#define DOUBLE_SINGLE_SINGLE			(0x0D)	// DH = 1, UD2 = 1, UD1 = 1

#define BLINK_FREQUENCY_2100mHz			(0x00)	// F2 = 0, F1 = 0, F0 = 0
#define BLINK_FREQUENCY_2300mHz			(0x01)	// F2 = 0, F1 = 0, F0 = 1
#define BLINK_FREQUENCY_2500mHz			(0x02)	// F2 = 0, F1 = 1, F0 = 0
#define BLINK_FREQUENCY_2700mHz			(0x03)	// F2 = 0, F1 = 1, F0 = 1
#define BLINK_FREQUENCY_2900mHz			(0x04)	// F2 = 1, F1 = 0, F0 = 0
#define BLINK_FREQUENCY_3100mHz			(0x05)	// F2 = 1. F1 = 0, F0 = 1
#define BLINK_FREQUENCY_3200mHz			(0x06)	// F2 = 1, F1 = 1, F0 = 0
#define BLINK_FREQUENCY_3400mHz			(0x07)	// F2 = 1, F1 = 1, F0 = 1

#define SHIFT_SCROLL_ALL_LINES_DISABLED	(0x00)	// DSx = HSx = 0 for x = 1-4
#define SHIFT_SCROLL_LINE_1_ENABLED		(0x01)	// DS1 = HS1 = 1
#define SHIFT_SCROLL_LINE_2_ENABLED		(0x02)	// DS2 = HS2 = 1
#define SHIFT_SCROLL_LINE_1_2_ENABLED	(0x03)	// DSx = HSx = 1 for x = 1-2
#define SHIFT_SCROLL_LINE_3_ENABLED		(0x04)	// DS3 = HS3 = 1
#define SHIFT_SCROLL_LINE_1_3_ENABLED	(0x05)	// DSx = HSx = 1 for x = 1,3
#define SHIFT_SCROLL_LINE_2_3_ENABLED	(0x06)	// DSx = HSx = 1 for x = 2-3
#define SHIFT_SCROLL_LINE_1_2_3_ENABLED	(0x07)	// DSx = HSx = 1 for x = 1-3
#define SHIFT_SCROLL_LINE_4_ENABLED		(0x08)	// DS4 = HS4 = 1
#define SHIFT_SCROLL_LINE_1_4_ENABLED	(0x09)	// DSx = HSx = 1 for x = 1,4
#define SHIFT_SCROLL_LINE_2_4_ENABLED	(0x0A)	// DSx = HSx = 1 for x = 2,4
#define SHIFT_SCROLL_LINE_1_2_4_ENABLED	(0x0B)	// DSx = HSx = 1 for x = 1-2,4
#define SHIFT_SCROLL_LINE_3_4_ENABLED	(0x0C)	// DSx = HSx = 1 for x = 3-4
#define SHIFT_SCROLL_LINE_1_3_4_ENABLED	(0x0D)	// DSx = HSx = 1 for x = 1,3-4
#define SHIFT_SCROLL_LINE_2_3_4_ENABLED	(0x0E)	// DSx = HSx = 1 for x = 2-4
#define SHIFT_SCROLL_ALL_LINES_ENABLED	(0x0F)	// DSx = HSx = 1 for x = 1-4

#define FONT_BLACK						(0x00)	// REV = 0
#define FONT_WHITE						(0x01)	// REV = 1

#define BLINK_DISABLED					(0x00)	// BE = 0
#define BLINK_ENABLED					(0x04)	// BE = 1

#define CONTRAST_0						(0x00)	// C5-0 = 000000
#define CONTRAST_1						(0x01)	// C5-0 = 000001
#define CONTRAST_2						(0x02)	// C5-0 = 000010
#define CONTRAST_3						(0x03)	// C5-0 = 000011
#define CONTRAST_4						(0x04)	// C5-0 = 000100
#define CONTRAST_5						(0x05)	// C5-0 = 000101
#define CONTRAST_6						(0x06)	// C5-0 = 000110
#define CONTRAST_7						(0x07)	// C5-0 = 000111
#define CONTRAST_8						(0x08)	// C5-0 = 001000
#define CONTRAST_9						(0x09)	// C5-0 = 001001
#define CONTRAST_10						(0x0A)	// C5-0 = 001010
#define CONTRAST_11						(0x0B)	// C5-0 = 001011
#define CONTRAST_12						(0x0C)	// C5-0 = 001100
#define CONTRAST_13						(0x0D)	// C5-0 = 001101
#define CONTRAST_14						(0x0E)	// C5-0 = 001110
#define CONTRAST_15						(0x0F)	// C5-0 = 001111
#define CONTRAST_16						(0x10)	// C5-0 = 010000
#define CONTRAST_17						(0x11)	// C5-0 = 010001
#define CONTRAST_18						(0x12)	// C5-0 = 010010
#define CONTRAST_19						(0x13)	// C5-0 = 010011
#define CONTRAST_20						(0x14)	// C5-0 = 010100
#define CONTRAST_21						(0x15)	// C5-0 = 010101
#define CONTRAST_22						(0x16)	// C5-0 = 010110
#define CONTRAST_23						(0x17)	// C5-0 = 010111
#define CONTRAST_24						(0x18)	// C5-0 = 011000
#define CONTRAST_25						(0x19)	// C5-0 = 011001
#define CONTRAST_26						(0x1A)	// C5-0 = 011010
#define CONTRAST_27						(0x1B)	// C5-0 = 011011
#define CONTRAST_28						(0x1C)	// C5-0 = 011100
#define CONTRAST_29						(0x1D)	// C5-0 = 011101
#define CONTRAST_30						(0x1E)	// C5-0 = 011110
#define CONTRAST_31						(0x1F)	// C5-0 = 011111
#define CONTRAST_32						(0x20)	// C5-0 = 100000
#define CONTRAST_33						(0x21)	// C5-0 = 100001
#define CONTRAST_34						(0x22)	// C5-0 = 100010
#define CONTRAST_35						(0x23)	// C5-0 = 100011
#define CONTRAST_36						(0x24)	// C5-0 = 100100
#define CONTRAST_37						(0x25)	// C5-0 = 100101
#define CONTRAST_38						(0x26)	// C5-0 = 100110
#define CONTRAST_39						(0x27)	// C5-0 = 100111
#define CONTRAST_40						(0x28)	// C5-0 = 101000
#define CONTRAST_41						(0x29)	// C5-0 = 101001
#define CONTRAST_42						(0x2A)	// C5-0 = 101010
#define CONTRAST_43						(0x2B)	// C5-0 = 101011
#define CONTRAST_44						(0x2C)	// C5-0 = 101100
#define CONTRAST_45						(0x2D)	// C5-0 = 101101
#define CONTRAST_46						(0x2E)	// C5-0 = 101110
#define CONTRAST_47						(0x2F)	// C5-0 = 101111
#define CONTRAST_48						(0x30)	// C5-0 = 110000
#define CONTRAST_49						(0x31)	// C5-0 = 110001
#define CONTRAST_50						(0x32)	// C5-0 = 110010
#define CONTRAST_51						(0x33)	// C5-0 = 110011
#define CONTRAST_52						(0x34)	// C5-0 = 110100
#define CONTRAST_53						(0x35)	// C5-0 = 110101
#define CONTRAST_54						(0x36)	// C5-0 = 110110
#define CONTRAST_55						(0x37)	// C5-0 = 110111
#define CONTRAST_56						(0x38)	// C5-0 = 111000
#define CONTRAST_57						(0x39)	// C5-0 = 111001
#define CONTRAST_58						(0x3A)	// C5-0 = 111010
#define CONTRAST_59						(0x3B)	// C5-0 = 111011
#define CONTRAST_60						(0x3C)	// C5-0 = 111100
#define CONTRAST_61						(0x3D)	// C5-0 = 111101
#define CONTRAST_62						(0x3E)	// C5-0 = 111110
#define CONTRAST_63						(0x3F)	// C5-0 = 111111

#define BOOSTER_AND_REGULATOR_EXTERNAL	(0x00)	// Bon = 0
#define BOOSTER_AND_REGULATOR_INTERNAL	(0x04)	// Bon = 1

#define ICON_DISPLAY_DISABLED			(0x00)	// Ion = 0
#define ICON_DISPLAY_ENABLED			(0x08)	// Ion = 1

#define BRIGHTNESS_0					(0x00)	// Rab2-0 = 000
#define BRIGHTNESS_1					(0x01)	// Rab2-0 = 001
#define BRIGHTNESS_2					(0x02)	// Rab2-0 = 010
#define BRIGHTNESS_3					(0x03)	// Rab2-0 = 011
#define BRIGHTNESS_4					(0x04)	// Rab2-0 = 100
#define BRIGHTNESS_5					(0x05)	// Rab2-0 = 101
#define BRIGHTNESS_6					(0x06)	// Rab2-0 = 110
#define BRIGHTNESS_7					(0x07)	// Rab2-0 = 111

#define DIVIDER_CIRCUIT_EXTERNAL		(0x00)	// Don = 0
#define DIVIDER_CIRCUIT_INTERNAL		(0x08)	// Don = 1

#define MPU_READY						(0x00)	// BF = 0
#define MPU_BUSY						(0x80)	// BF = 1

#define NEG_0_05_PERCENT_PER_DEG_C		(0x02)	// TC2 = 0, TC1 = 1, TC0 = 0
#define NEG_0_10_PERCENT_PER_DEG_C		(0x04)	// TC2 = 1, TC1 = 0, TC0 = 0
#define NEG_0_15_PERCENT_PER_DEG_C		(0x06)	// TC2 = 1, TC1 = 1, TC0 = 0
#define NEG_0_20_PERCENT_PER_DEG_C		(0x07)	// TC2 = 1, TC1 = 1, TC0 = 1

#define ROM_A							(0x00)	// ROM2 = 0, ROM1 = 0
#define ROM_B							(0x04)	// ROM2 = 0, ROM1 = 1
#define ROM_C							(0x08)	// ROM2 = 1, ROM1 = 0

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
	STATUS_REACHED_MAX_MIN,
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
 *		-# Set the bias of the internal divider.
 *		-# Set the type of booster and regulator circuitry.
 *		-# Set the type of divider circuitry.
 *		-# Set the temperature compensation circuitry coefficient.
 *		-# Set the ROM used for character generation.
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
 *		- DEFAULT: ONLY valid if rpiResetPin = NO_HARDWARE_RESET_PIN.
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
 *		- DEFAULT: the temperature coefficient is set to the power on reset
 *				value (-0.05%/degC).
 * @param [in]		rom ROM used for character generation.  Valid values are:
 * 		- ROM_A: CGROM is set to ROM A.
 *		- ROM_B: CGROM is set to ROM B.
 *		- ROM_C: CGROM is set to ROM C.
 *		- DEFAULT: CGROM is set to the power on reset value (ROM A).
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
/**
 * @details This function implements the following objectives:
 * 		-# Deinitialise the I2C communication interface to the SSD1803A MPU.
 * 		-# Hardware disable the SSD1803A display MPU by setting the RPi reset
 * 				pin level to the reset level.
 *		-# Deinitialise the GPIO interface that allows for hardware control over
 *				the enabling/disabling of the display.
 *
 * It is strongly recommended (although not strictly required) to call
 * ssd_display_disable() before calling this function.
 * 
 * @retval STATUS_OK If the SSD1803A display MPU was successfully deinitialised.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * ========================================================================== */
status_t ssd_deinit(void);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Set the direction which the text characters and symbols will be
 *				written to the SSD1803A LCD.
 *		-# Set the number of lines of text that the SSD1803A LCD is configured
 *				to show.
 *		-# Enable/disable the blinking of the symbols created with
 *				ssd_create_symbol().
 *		-# Set the character and symbol movement mode (shift/scroll).
 *		-# Set the lines on which the character and symbol movement is enabled.
 *		-# Set the horizontal display direction of the SSD1803A LCD.
 *		-# Set the vertical display direction of the SSD1803A LCD.
 *
 * @param [in]		writeDirection Direction which the text and symbols will be
 * written to the SSD1803A LCD.  Valid values are:
 * 		- NORMAL_WRITE_DIRECTION: A text string will be written to the SSD1803A
 * 				from left to right.
 * 		- REVERSE_WRITE_DIRECTION: A text string will be written to the SSD1803A
 * 				from right to left.
 * 		- UNCHANGED: The previous write direction will be set as the write
 * 				direction.
 * @param [in]		numberOfLines Number of lines on the SSD1803A LCD that are
 * enabled on which to display text and symbols.  It should be noted that this
 * does not mean that there is the specified number of distinct lines available
 * to write text on.  For instance, if it is set to display four lines
 * (DISPLAY_FOUR_LINES) but the font height is set to two lines of double height
 * text (fontHeight = DOUBLE_DOUBLE in ssd_set_font()), then only two lines of
 * text will be able to be visible on the SSD1803A LCD, not four.  In this case,
 * if four lines of distinct text are desired, then the possibility for double
 * font must be disabled (fontHeight = DOUBLE_HEIGHT_DISABLED in
 * ssd_set_font()).  Valid values are:
 * 		- DISPLAY_ONE_LINE: Only the bottom line of the SSD1803A LCD is enabled.
 * 		- DISPLAY_TWO_LINES: Only the bottom two lines of the SSD1803A LCD are
 * 				enabled.
 * 		- DISPLAY_THREE_LINES: Only the bottom three lines of the SSD1803A LCD
 * 				are enabled.
 * 		- DISPLAY_FOUR_LINES: All of the lines of the SSD1803A LCD are enabled.
 * 		- UNCHANGED: The previous number of enabled lines will be set as the
 * 				number of lines.
 * @param [in]		blinkEnable Enables/disables the blinking of custom symbols
 * created with a call to ssd_create_symbol().  See the function comments of
 * ssd_create_symbol() for an overview of how to control which individual dots
 * of the custom symbol blink when the blinking of custom symbols is enabled.
 * Valid values are:
 * 		- BLINK_ENABLED: The bits enabled to blink in the customs symbols blink
 * 				at the same frequency as the cursor.
 * 		- BLINK_DISABLED: The blinking of the custom symbols is disabled.
 * 		- UNCHANGED: The previous blinking state of the custom symbols is used
 * 				to enable/disable the blinking of the custom symbols.
 * @param [in]		shiftOrScroll Sets the type of movement of characters and
 * symbols on the lines enabled by shiftOrScrollLinesEnabled.  Valid values are:
 * 		- DISPLAY_SHIFT_ENABLED: The characters and symbols can be moved across
 * 				the SSD1803A LCD by means of a display shift
 * 				(ssd_shift_display()).
 * 		- DOT_SCROLL_ENABLED: The characters and symbols can be moves across the
 * 				SSD1803A LCD one dot at a time by means of a dot scroll
 * 				(ssd_scroll_display()).
 * 		- UNCHANGED: The previous type of character and symbol movement is used
 * 				to set the type of character and symbol movement.
 * @param [in]		shiftOrScrollLinesEnabled Sets the lines on which the
 * characters and symbols can be moved across the SSD1803A LCD.  Valid values
 * are:
 * 		- SHIFT_SCROLL_ALL_LINES_DISABLED: None of the lines on the SSD1803A LCD
 * 				have character or symbol movement enabled.
 * 		- SHIFT_SCROLL_LINE_1_ENABLED: Only the first line on the SSD1803A LCD
 * 				has character and symbol movement enabled.
 * 		- SHIFT_SCROLL_LINE_1_2_ENABLED: Only the first two lines on the
 * 				SSD1803A LCD have character and symbol movement enabled.
 * 		- etc
 * 		- SHIFT_SCROLL_ALL_LINES_ENABLED: All of the lines on the SSD1803A LCD
 * 				have character and symbol movement enabled.
 * 		- UNCHANGED: The previously enabled lines are used to set the lines on
 * 				which characters and symbols can be moved across the SSD1803A
 * 				LCD.
 * @param [in]		horizontalDisplayDirection Sets the horizontal display
 * direction of the characters and symbols on the SSD1803A LCD.  Valid values
 * are:
 * 		- NORMAL_HORIZONTAL_DIRECTION: The horizontal display direction is in
 * 				the normal orientation.
 * 		- REVERSE_HORIZONTAL_DIRECTION: The horizontal display direction is in
 * 				the reverse orientation.  This view is best described as what
 * 				would be seen if a mirror was positioned parallel with the
 * 				vertical axis of the SSD1803A LCD.
 * 		- UNCHANGED: The previous horizontal display direction will be used to
 * 				set the horizontal display direction.
 * @param [in]		verticalDisplayDirection Sets the vertical display direction
 * of the characters and symbols on the SSD1803A LCD.  Valid values are:
 * 		- NORMAL_VERTICAL_DIRECTION: The vertical display direction is in the
 * 				normal orientation.
 * 		- REVERSE_VERTICAL_DIRECTION: The vertical display direction is in the
 * 				reverse orientation.  This view is best described as what would
 * 				be seen if a mirror was positioned parallel with the horizontal
 * 				axis of the SSD1803A LCD.
 * 		- UNCHANGED: The previous vertical display direction will be used to set
 * 				the vertical display direction.
 * 
 * @retval STATUS_OK If the display settings were successfully configured.
 * @retval STATUS_INVALID_PARAM If a parameter value was outside of the valid
 * range listed above.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE is a theoretically possible return value,
 * however it should never occur.
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
/**
 * @details This function implements the following objectives:
 *		-# Leave sleep mode if required.
 *		-# Turn on the display of the SSD1803A.
 *		-# Increase the contrast and brightness to the previous levels.
 *
 * All of the display settings configured with ssd_display_configure() still
 * apply.
 *
 * @retval STATUS_OK If the display was successfully enabled.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_display_enable(void);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Drop the contrast and the brightness of the SSD1803A LCD.
 *		-# Turn off the display of the SSD1803A LCD.
 *		-# Enter sleep mode if required.
 *
 * It is strongly recommended (although not strictly necessary) to execute this
 * function before executing ssd_deinit().
 * All of the display settings configured with ssd_display_configure() still
 * apply.
 *
 * @param [in]		lowPowerMode Defines whether the SSD1803A display MPU will
 * enter low power mode after the display is disabled.  Valid values are:
 * 		- POWER_SAVING_MODE_DISABLED: After the display is disabled, the
 * 				SSD1803A display MPU will not enter low power mode.
 * 		- POWER_SAVING_MODE_ENABLED: After the display is disabled, the SSD1803A
 * 				display MPU will enter low power mode.  It is recommended that
 * 				this is the setting that is used when deinitialising and
 * 				disabling the SSD1803A at the end of the programme.
 * 
 * @retval STATUS_OK If the display was successfully disabled.
 * @retval STATUS_INVALID_PARAM If a parameter value was outside of the valid
 * range listed above.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE is a theoretically possible return value,
 * however it should never occur.
 * ========================================================================== */
status_t ssd_display_disable(uint8_t lowPowerMode);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Enable/disable the cursor blink.
 *		-# Set the cursor blink rate.
 *		-# Set the cursor shadow.
 *		-# Set the cursor behaviour after a character is written to the SSD1803A
 *				LCD.
 *
 * @param [in]		cursorBlink Defines whether or not the cursor will blink
 * when it is enabled with a call to ssd_cursor_enable().  Valid values are:
 * 		- CURSOR_BLINK_DISABLED: When the cursor is enabled, it will not blink.
 * 				If the cursor shadow is not enabled (cursorShadow =
 * 				CURSOR_SHADOW_DISABLED), then even after the cursor is enabled,
 * 				it will still appear as if the cursor has not been enabled.
 * 		- CURSOR_BLINK_ENABLED: When the cursor is enabled, it will blink.
 * 		- UNCHANGED: The previous cursor blink state will be used to set the
 * 				cursor blink state.
 * @param [in]		cursorBlinkRate: Defines the frequency at which the cursor
 * will blink when it is enabled.  Valid values are:
 * 		- BLINK_FREQUENCY_2100mHz: The frequency of the cursor blink will be set
 * 				to 2.1Hz.
 * 		- BLINK_FREQUENCY_2300mHz: The frequency of the cursor blink will be set
 * 				to 2.3Hz.
 * 		- BLINK_FREQUENCY_2500mHz: The frequency of the cursor blink will be set
 * 				to 2.5Hz.
 * 		- BLINK_FREQUENCY_2700mHz: The frequency of the cursor blink will be set
 * 				to 2.7Hz.
 * 		- BLINK_FREQUENCY_2900mHz: The frequency of the cursor blink will be set
 * 				to 2.9Hz.
 * 		- BLINK_FREQUENCY_3100mHz: The frequency of the cursor blink will be set
 * 				to 3.1Hz.
 * 		- BLINK_FREQUENCY_3200mHz: The frequency of the cursor blink will be set
 * 				to 3.2Hz.
 * 		- BLINK_FREQUENCY_3400mHz: The frequency of the cursor blink will be set
 * 				to 3.4Hz.
 * 		- UNCHANGED: The previous cursor blink rate will be used to set the
 * 				cursor blink rate.
 * @param [in]		cursorShadow: Defines whether the presence of the cursor
 * will always be noted by a single line beneath the text character or symbol
 * when the cursor is enabled with a call to ssd_cursor_enable().  Valid values
 * are:
 * 		- CURSOR_SHADOW_DISABLED: No single line beneath the text character or
 * 				symbol will denote the current cursor location once the cursor
 * 				is enabled.  If the cursor is set to not blink (cursorBlink =
 * 				CURSOR_BLINK_DISABLED), then even after the cursor is enabled,
 * 				it will still appear as if the cursor has not been enabled.
 * 		- CURSOR_SHADOW_ENABLED: A single line beneath a text character or
 * 				symbol will denote the current cursor location when the cursor
 * 				is enabled.
 * 		- UNCHANGED: The previous configuration of the cursor shadow will be
 * 				used to set the cursor shadow.
 * @param [in]		cursorStationary Defines whether the cursor will move after
 * a character or symbol is written to the SSD1803A LCD.  Valid values are:
 * 		- CURSOR_STATIONARY: After a character or symbol is written to the
 * 				SSD1803A LCD, then instead of the cursor shifting to the next
 * 				position, the display will instead shift.  This gives the
 * 				appearance of a stationary cursor.
 * 		- CURSOR_MOVING: After a character or symbol is written to the SSD1803A
 * 				LCD, then the cursor will shift to the next position.
 * 		- UNCHANGED: The last cursor movement configuration will be used to set
 * 				the cursor movement after a character or symbol is written to
 * 				the SSD1803A LCD.
 * 
 * @retval STATUS_OK If the cursor settings were successfully configured.
 * @retval STATUS_INVALID_PARAM If a parameter value was outside of the valid
 * range listed above.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE is a theoretically possible return value,
 * however it should never occur.
 * ========================================================================== */
status_t ssd_cursor_configure(
		uint8_t cursorBlink,
		uint8_t cursorBlinkRate,
		uint8_t cursorShadow,
		uint8_t cursorStationary
);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Enable the cursor (ensure its visibility on the SSD1803A LCD).
 *
 * All of the cursor settings configured with ssd_cursor_configure() still
 * apply.
 *
 * @retval STATUS_OK If the cursor was successfully enabled.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_cursor_enable(void);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Disable the cursor (remove its visibility from the SSD1803A LCD).
 *
 * All of the cursor settings configured with ssd_cursor_configure() still
 * apply.
 *
 * @retval STATUS_OK If the cursor was successfully disabled.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_cursor_disable(void);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Set the font height for the enabled lines of the SSD1803A LCD.
 *		-# Set the font width for the text and symbols to be displayed on the
 *				SSD1803A LCD.
 *		-# Set the font colour for the text and symbols to be displayed on the
 *				SSD1803A LCD.
 *
 * @param [in]		fontHeight Height of the text across the enabled lines of
 * the SSD1803A.  Valid values are:
 * 		- DOUBLE_HEIGHT_DISABLED: The text on each line of the SSD1803A LCD only
 * 				spans a single line.
 * 		- SINGLE_SINGLE_DOUBLE: The text on the first and second lines of the
 * 				SSD1803A LCD only span one line, however the text on the third
 * 				line of the SSD1803A LCD spans two lines.  If the display is
 * 				only configured for displaying three lines of text
 * 				(numberOfLines = DISPLAY_THREE_LINES in
 * 				ssd_display_configure()), then only the top half of the third
 * 				line of text will be displayed.  Therefore it is advised not to
 * 				use this value of fontHeight for such a configuration.
 * 		- SINGLE_DOUBLE_SINGLE: The text on the first and third lines of the
 * 				SSD1803A LCD only span one line, however the text on the second
 * 				line of the SSD1803A LCD spans two lines.  If the display is
 * 				only configured for displaying two lines of text
 * 				(numberOfLines = DISPLAY_TWO_LINES in ssd_display_configure()),
 * 				then only the top half of the second line of text will be
 * 				displayed.  Therefore it is advised not to use this value of
 * 				fontHeight for such a configuration.
 * 		- DOUBLE_SINGLE_SINGLE: The text on the second and third lines of the
 * 				SSD1803A LCD only span one line, however the text on the first
 * 				line of the SSD1803A LCD spans two lines.  If the display is
 * 				only configured for displaying one lines of text
 * 				(numberOfLines = DISPLAY_ONE_LINE in ssd_display_configure()),
 * 				then only the top half of the first line of text will be
 * 				displayed.  Therefore it is advised not to use this value of
 * 				fontHeight for such a configuration.
 * 		- DOUBLE_DOUBLE: The text on both the first and second lines of the
 * 				SSD1803A LCD will span two lines.  If the display is only
 * 				configured for displaying one line of text (numberOfLines =
 * 				DISPLAY_ONE_LINE in ssd_display_configure()), then only the top
 * 				half of the first line of text will be displayed.  If the
 * 				display is only configured for displaying three lines of text
 * 				(numberOfLines = DISPLAY_THREE_LINES in
 * 				ssd_display_configure()), then only the top half of the second
 * 				line of text will be displayed.  Therefore it is advised not to
 * 				use this value of fontHeight for such configurations.
 * 		- UNCHANGED: The previous font height configuration will be used to set
 * 				the font height.
 * @param [in]		fontWidth Width of the text and symbols displayed on the
 * LCD1803A LCD.  Valid values are:
 * 		- FONT_WIDTH_FIVE_DOTS: The width of each text character and symbol will
 * 				span five dots.
 * 		- FONT_WIDTH_SIX_DOTS: The width of each text character and symbol will
 * 				span six dots.
 * 		- UNCHANGED: The previous font width configuration will be used to set
 * 				the font width.
 * @param [in]		fontColour Colour of the text and symbols displayed on the
 * LCD1803A LCD.  Valid values are:
 * 		- FONT_BLACK: The colour of each text character and symbol will be black
 * 				while the background of each text character and symbol will be
 * 				white.
 * 		- FONT_WHITE: The colour of each text character and symbol will be white
 * 				while the background of each text character and symbol will be
 * 				black.
 * 		- UNCHANGED: The previous font colour configuration will be used to set
 * 				the font colour.
 * 
 * @retval STATUS_OK If the font for the text and symbols displayed on the
 * SSD1803A LCD was successfully configured.
 * @retval STATUS_INVALID_PARAM If a parameter value was outside of the valid
 * range listed above.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE is a theoretically possible return value,
 * however it should never occur.
 * ========================================================================== */
status_t ssd_set_font(
		uint8_t fontHeight,
		uint8_t fontWidth,
		uint8_t fontColour
);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Set the brightness of the SSD1803A LCD to the specified level.
 *
 * There are eight brightness levels for the SSD1803A LCD.
 * 
 * @param [in]		displayBrightness Brightness level to which the SSD1803A LCD
 * will be set.  Valid values are:
 * 		- BRIGHTNESS_0 <= displayBrightness <= BRIGHTNESS_7
 * 
 * @retval STATUS_OK If the brightness of the SSD1803A LCD was successfully set
 * to the specified value.
 * @retval STATUS_INVALID_PARAM If a parameter value was outside of the valid
 * range listed above.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE is a theoretically possible return value,
 * however it should never occur.
 * ========================================================================== */
status_t ssd_set_brightness(uint8_t displayBrightness);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Increment the brightness of the SSD1803A LCD by one level.
 *
 * There are eight brightness levels for the SSD1803A LCD.
 *
 * @retval STATUS_OK If the brightness of the SSD1803A LCD was successfully
 * incremented by one level.
 * @retval STATUS_REACHED_MAX_MIN If the SSD1803A LCD brightness was already at
 * the maximum value.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_increment_brightness(void);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Decrement the brightness of the SSD1803A LCD by one level.
 *
 * There are eight brightness levels for the SSD1803A LCD.
 *
 * @retval STATUS_OK If the brightness of the SSD1803A LCD was successfully
 * decremented by one level.
 * @retval STATUS_REACHED_MAX_MIN If the SSD1803A LCD brightness was already at
 * the minimum value.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_decrement_brightness(void);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Set the contrast of the SSD1803A LCD to the specified level.
 *
 * There are 64 contrast levels for the SSD1803A LCD.
 * 
 * @param [in]		displayContrast Contrast level to which the SSD1803A LCD
 * will be set.  Valid values are:
 * 		- CONTRAST_0 <= displayContrast <= CONTRAST_63
 * 
 * @retval STATUS_OK If the contrast of the SSD1803A LCD was successfully set to
 * the specified value.
 * @retval STATUS_INVALID_PARAM If a parameter value was outside of the valid
 * range listed above.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE is a theoretically possible return value,
 * however it should never occur.
 * ========================================================================== */
status_t ssd_set_contrast(uint8_t displayConstrast);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Increment the contrast of the SSD1803A LCD by one level.
 *
 * There are 64 contrast levels for the SSD1803A LCD.
 *
 * @retval STATUS_OK If the contrast of the SSD1803A LCD was successfully
 * incremented by one level.
 * @retval STATUS_REACHED_MAX_MIN If the SSD1803A LCD contrast was already at
 * the maximum value.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_increment_contrast(void);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Decrement the contrast of the SSD1803A LCD by one level.
 *
 * There are 64 contrast levels for the SSD1803A LCD.
 *
 * @retval STATUS_OK If the contrast of the SSD1803A LCD was successfully
 * decremented by one level.
 * @retval STATUS_REACHED_MAX_MIN If the SSD1803A LCD contrast was already at
 * the minimum value.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_decrement_contrast(void);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Clear all text and from the SSD1803A LCD.
 *		-# Reset the cursor to the start of first line.
 *		-# Reset any display shift or dot scroll quantity.
 *
 * @retval STATUS_OK If the SSD1803A LCD was successfully cleared of text and
 * symbols.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_clear_display(void);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Move the cursor to the start of the first line.
 *
 * @retval STATUS_OK If the cursor was successfully moved to the start of the
 * first line.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_move_cursor_line1(void);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Move the cursor to the start of the second line.
 *
 * @retval STATUS_OK If the cursor was successfully moved to the start of the
 * second line.
 * @retval STATUS_INVALID_MODE If the display is configured for only one line
 * (numberOfLines = DISPLAY_ONE_LINE ssd_display_configure()).
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_move_cursor_line2(void);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Move the cursor to the start of the third line.
 *
 * @retval STATUS_OK If the cursor was successfully moved to the start of the
 * third line.
 * @retval STATUS_INVALID_MODE If the display is configured for only one or two
 * lines (numberOfLines = DISPLAY_ONE_LINE or DISPLAY_TWO_LINES in
 * ssd_display_configure()).
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_move_cursor_line3(void);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Move the cursor to the start of the fourth line.
 *
 * @retval STATUS_OK If the cursor was successfully moved to the start of the
 * fourth line.
 * @retval STATUS_INVALID_MODE If the display is configured for only one, two,
 * or three lines (numberOfLines = DISPLAY_ONE_LINE, DISPLAY_TWO_LINES, or
 * DISPLAY_THREE_LINES in ssd_display_configure()).
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_move_cursor_line4(void);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Move the cursor one position to the left.
 *
 * When the cursor is at the start of a line, the next call to this function
 * will put the cursor at the end of the previous line.  When the cursor is at
 * the start of the first line, the next call to this function will put the
 * cursor at the end of the last line.
 *
 * @retval STATUS_OK If the cursor was successfully moved to the left.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_move_cursor_left(void);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Move the cursor one position to the right.
 *
 * When the cursor is at the end of a line, the next call to this function will
 * put the cursor at the start of the next line.  When the cursor is at the end
 * of the last line, the next call to this function will put the cursor at the
 * start of the first line.
 *
 * @retval STATUS_OK If the cursor was successfully moved to the right.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_move_cursor_right(void);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Move the cursor to the specified location.
 *
 * @param [in]		line Line to which the cursor is to be moved.  Possible
 * values and their validity are:
 * 		- LINE_1: Always valid
 * 		- LINE_2: Only valid if the display has been configured with two or more
 * 				lines (numberOfLines = DISPLAY_TWO_LINES, DISPLAY_THREE_LINES,
 * 				or DISPLAY_FOUR_LINES in ssd_display_configure()).
 * 		- LINE_3: Only valid if the display has been configured with three or
 * 				more lines (numberOfLines = DISPLAY_THREE_LINES or
 * 				DISPLAY_FOUR_LINES in ssd_display_configure()).
 * 		- LINE_4: Only valid if the display has been configured with four lines
 * 				(numberOfLines = DISPLAY_FOUR_LINES in ssd_display_configure()).
 * @param [in]		position Position in the specified line to which the cursor
 * is to be moved.  Possible values and their validity are:
 * 		- POSITION_1 <= position <= POSITION_20: Always valid.
 * 		- POSITION_21 <= position <= POSITION_40: Only valid if the display has
 * 				been configured with one or two lines (numberOfLines = 
 * 				DISPLAY_ONE_LINE or DISPLAY_TWO_LINES in
 * 				ssd_display_configure()).
 * 		- POSITION_41 <= position <= POSITION_80: Only valid if the display has
 * 				been configured with one line (numberOfLines = DISPLAY_ONE_LINE
 * 				in ssd_display_configure()).
 * It should be noted that only POSITION_1 to POSITION_20 are normally visible
 * on the LCD.  In single or double line mode (numberOfLines = 
 * DISPLAY_ONE_LINE or DISPLAY_TWO_LINES in ssd_display_configure()), the
 * other positions can be accessed by scrolling (ssd_scroll_display()) or
 * shifting (ssd_shift_display()) the display.
 * 
 * @retval STATUS_OK If the cursor was successfully moved to the specified line
 * and position.
 * @retval STATUS_INVALID_PARAM If a parameter value was outside of the valid
 * range listed above.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE is a theoretically possible return value,
 * however it should never occur.
 * ========================================================================== */
status_t ssd_move_cursor(
		uint8_t line,
		uint8_t position
);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Set the address counter to the CGRAM address corresponding to the
 *				specified symbol ID.
 *		-# Write the symbol data to the CGRAM.
 *		-# Reset the address counter to the previous DDRAM address.
 *
 * For a five dot font width (fontWidth = FONT_WIDTH_FIVE_DOTS in
 * ssd_set_font()), the format of symbolData is as follows.
 * 
 *               | Bit7  Bit6  Bit5  Bit4  Bit3  Bit2  Bit1  Bit0
 * --------------+------------------------------------------------
 * symbolData[0] | B01   B00    x    D04   D03   D02   D01   D00
 * symbolData[1] | B11   B10    x    D14   D13   D12   D11   D10
 * symbolData[2] | B21   B20    x    D24   D23   D22   D21   D20
 * symbolData[3] | B31   B30    x    D34   D33   D32   D31   D30
 * symbolData[4] | B41   B40    x    D44   D43   D42   D41   D40
 * symbolData[5] | B51   B50    x    D54   D53   D52   D51   D50
 * symbolData[6] | B61   B60    x    D64   D63   D62   D61   D60
 * symbolData[7] | B71   B70    x    D74   D73   D72   D71   D70
 *
 * For a six dot font width (fontWidth = FONT_WIDTH_SIX_DOTS in ssd_set_font()),
 * the format of symbolData is as follows.
 * 
 *               | Bit7  Bit6  Bit5  Bit4  Bit3  Bit2  Bit1  Bit0
 * --------------+------------------------------------------------
 * symbolData[0] | B01   B00   D05   D04   D03   D02   D01   D00
 * symbolData[1] | B11   B10   D15   D14   D13   D12   D11   D10
 * symbolData[2] | B21   B20   D25   D24   D23   D22   D21   D20
 * symbolData[3] | B31   B30   D35   D34   D33   D32   D31   D30
 * symbolData[4] | B41   B40   D45   D44   D43   D42   D41   D40
 * symbolData[5] | B51   B50   D55   D54   D53   D52   D51   D50
 * symbolData[6] | B61   B60   D65   D64   D63   D62   D61   D60
 * symbolData[7] | B71   B70   D75   D74   D73   D72   D71   D70
 * 
 * Where:
 * 		- Bx1 and Bx0 set the blink mode of the enabled bits of the xth row of
 * 				the symbol according to the following table:
 * 
 * 			Bx1  Bx0 | Blink function
 *          ---------+------------------------------------------------------
 * 			 0    0  | Blinking disabled.
 * 			 0    1  | Only Dx4 (5-dot width) / Dx5 (6-dot width) will blink.
 * 			 1    0  | Blinking of Dx4-Dx0 (5-dot width) / Dx5-Dx0 (6-dot
 * 					 | width) enabled.
 * 			 1    1  | Blinking of Dx4-Dx0 (5-dot width) / Dx5-Dx0 (6-dot
 * 					 | width) enabled.
 * 
 * 		- Dxy enables/disables the dot found at row x, column y.
 * 
 * By way of an example, if the letter A (shown below) was to be saved as a
 * custom symbol with blinking of all enabled bits, then the following code
 * snippet should be used:
 *	uint8_t symbolData = {
 *			0xCE, // B01 = B00 = 1, D04 = 0, D03 = D02 = D01 = 1, D00 = 0
 *			0xD1, // B11 = B10 = 1, D14 = 1, D13 = D02 = D11 = 0, D10 = 1
 *			0xD1, // B21 = B20 = 1, D24 = 1, D23 = D02 = D21 = 0, D20 = 1
 *			0xDF, // B31 = B30 = 1, D34 = D33 = D32 = D31 = D30 = 1
 *			0xD1, // B41 = B40 = 1, D44 = 1, D43 = D42 = D41 = 0, D40 = 1
 *			0xD1, // B51 = B50 = 1, D54 = 1, D53 = D52 = D51 = 0, D50 = 1
 *			0xD1, // B61 = B60 = 1, D64 = 1, D63 = D62 = D61 = 0, D60 = 1
 *			0xC0, // B71 = B70 = 1, D74 = D73 = D72 = D71 = D70 = 0
 *	};
 *	status_t ret = ssd_create_symbol(symbolData, SYMBOL1_ID);
 * 
 * 				  x x x 
 *				x       x
 *				x       x
 *				x x x x x
 *				x       x
 *				x       x
 *				x       x
 * 
 * @require symbolData to point to eight bytes of previously allocated memory.
 * 
 * @param [in]		symbolData Pointer to the memory where the symbol data is
 * stored.
 * @param [in]		symbolID Identification number of a custom symbol to be
 * created.  Valid values are:
 * 		- symbolID = SYMBOL1_ID, ..., SYMBOL8_ID.
 * 
 * @retval STATUS_OK If the custom symbol was successfully written to the LCD.
 * @retval STATUS_INVALID_PARAM If a parameter value was outside of the valid
 * range listed above.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE is a theoretically possible return value,
 * however it should never occur.
 * ========================================================================== */
status_t ssd_create_symbol(
		uint8_t *symbolData,
		uint8_t symbolID
);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Write the symbol identified by symbolID to the LCD.
 *
 * @param [in]		symbolID Identification number of a custom symbol that was
 * previously created with a call to ssd_create_symbol().  Valid values are:
 * 		- symbolID = SYMBOL1_ID, ..., SYMBOL8_ID (assuming the same symbolID
 * 				was passed to the function ssd_create_symbol() and the function
 * 				successfully returned).
 * 
 * @retval STATUS_OK If the custom symbol was successfully written to the LCD.
 * @retval STATUS_INVALID_PARAM If a parameter value was outside of the valid
 * range listed above.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE is a theoretically possible return value,
 * however it should never occur.
 * ========================================================================== */
status_t ssd_write_symbol(uint8_t symbolID);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Write the text string to the LCD.
 *
 * @param [in]		text String of text to be written to the LCD.  Valid values
 * are:
 * 		- length(text) < 0x50.
 * 
 * @retval STATUS_OK If the text string was successfully written to the LCD.
 * @retval STATUS_INVALID_PARAM If a parameter value was outside of the valid
 * range listed above.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE is a theoretically possible return value,
 * however it should never occur.
 * ========================================================================== */
status_t ssd_write_text(char *text);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Shift the display the desired amount at the desired speed in the
 *				desired direction.
 *
 * @param [in]		shiftAmount Number of positions by which to shift the
 * display.  Valid values are:
 * 		- SHIFT_SCROLL_RESET: Display is reset to the original position (i.e.
 * 				with no shift).
 * 		- shiftAmount > 0: Display is shifted in the specified direction by this
 * 				amount.
 * @param [in]		shiftSpeed: Number of seconds between each consecutive
 * display shift.  Valid values are:
 * 		- SHIFT_SCROLL_IMMEDIATE: The display is immediately shifted shiftAmount
 *				positions.
 *		- DEFAULT: Filler value to be used when shiftAmount is set to
 *				SHIFT_SCROLL_RESET.
 * 		- scrollSpeed > 0.0
 * @param [in]		shiftDirection Direction that the display will be shifted.
 * Valid values are:
 * 		- SHIFT_SCROLL_LEFT: The display will be shifted to the left.
 * 		- SHIFT_SCROLL_RIGHT: The display will be shifted to the right.
 * 		- DEFAULT: Filler value only valid if shiftAmount is set to
 * 				SHIFT_SCROLL_RESET.
 * 
 * @retval STATUS_OK If the display was successfully shifted the specified
 * amount.
 * @retval STATUS_INVALID_MODE If shift/scroll mode of the SSD1803A display MPU
 * is set to scroll mode (shiftOrScroll = DOT_SCROLL_ENABLED in
 * ssd_display_configure()) or none of the lines of the SSD1803A LCD are enabled
 * for shifting (shiftOrScrollLinesEnabled = SHIFT_SCROLL_ALL_LINES_DISABLED in
 * ssd_display_configure()).
 * @retval STATUS_INVALID_PARAM If a parameter value was outside of the valid
 * range listed above.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE is a theoretically possible return value,
 * however it should never occur.
 * ========================================================================== */
status_t ssd_shift_display(
		uint8_t shiftAmount,
		double shiftSpeed,
		uint8_t shiftDirection
);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Scroll the display from the start position to the end position at the
 *				specified rate.
 *
 * @param [in]		startPosition Number of dots at which to start the scroll.
 * Valid values are:
 * 		- 0x00 <= startPosition <= 0x30: Starting scroll quantity is set to the
 * 				start position.
 * 		- UNCHANGED: Starting scroll quantity remains at the current scroll
 * 				quantity.
 * It ALWAYS must be the case that startPosition != endPosition.
 * @param [in]		endPosition Number of dots at which to end the scroll.
 * Valid values are:
 * 		- 0x00 <= endPosition <= 0x30: Ending scroll quantity is set to the
 *				end position.
 * It always must be the case that startPosition != endPosition.
 * @param [in]		scrollSpeed: Number of seconds between each consecutive dot
 * scroll.  Valid values are:
 * 		- SHIFT_SCROLL_IMMEDIATE: The display is immediately scrolled to the end
 * 				position.
 * 		- scrollSpeed > 0.0
 * 
 * @retval STATUS_OK If the display was successfully scrolled from startPosition
 * to endPosition.
 * @retval STATUS_INVALID_MODE If shift/scroll mode of the SSD1803A display MPU
 * is set to shift mode (shiftOrScroll = DISPLAY_SHIFT_ENABLED in
 * ssd_display_configure()) or none of the lines of the SSD1803A LCD are enabled
 * for scrolling (shiftOrScrollLinesEnabled = SHIFT_SCROLL_ALL_LINES_DISABLED in
 * ssd_display_configure()).
 * @retval STATUS_INVALID_PARAM If a parameter value was outside of the valid
 * range listed above.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE is a theoretically possible return value,
 * however it should never occur.
 * ========================================================================== */
status_t ssd_scroll_display(
		uint8_t startPosition,
		uint8_t endPosition,
		double scrollSpeed
);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Read the current status of the SSD1803A from the SSD1803A display
 *				MPU.
 *
 * @require busyFlag points to one byte of previously allocated memory.
 * 
 * @param [out]		busyFlag Pointer to the memory address where the current
 * status of the SSD1803A read from the SSD1803A is to be stored.  This value
 * will either be MPU_READY or MPU_BUSY.
 * 
 * @retval STATUS_OK If the current status of the SSD1803A was successfully read
 * from the SSD1803A.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_FAILED_READ If the message failed to be successfully read from
 * the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_get_busy_flag(uint8_t *busyFlag);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Read the current value in the address counter of the SSD1803A
 *				from the SSD1803A display MPU.
 *
 * @require currentAddress points to one byte of previously allocated memory.
 * 
 * @param [out]		currentAddress Pointer to the memory address where the
 * current address counter value read from the SSD1803A is to be stored.
 * 
 * @retval STATUS_OK If the current address counter value was successfully read
 * from the SSD1803A.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_FAILED_READ If the message failed to be successfully read from
 * the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_get_address_counter(uint8_t *currentAddress);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Read the SSD1803A part ID from the SSD1803A display MPU.  This
 *				should always be 0x1A.
 *
 * @require partID points to one byte of previously allocated memory.
 * 
 * @param [out]		partID Pointer to the memory address where the part ID read
 * from the SSD1803A is to be stored.
 * 
 * @retval STATUS_OK If the part ID was successfully read from the SSD1803A.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_FAILED_READ If the message failed to be successfully read from
 * the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_get_part_id(uint8_t *partID);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Update the address counter to be the specified DDRAM address.
 *		-# Read the specified amount of data from the DDRAM.
 *		-# Reset the address counter to be the previous DDRAM address.
 *
 * @note The address counter will always be incremented after one byte of DDRAM
 * data is read.  Therefore if the starting address is 0x00, then after the
 * first byte of data is ready, the next bytes of data will be read from 0x01
 * and onwards (if multiple data bytes are required).
 * 
 * @require data points to dataLength of previously allocated memory.
 *
 * @param [in]		address Starting DDRAM address that will be read.  Valid
 * values are:
 * 		- For single line display (numberOfLines = DISPLAY_ONE_LINE in
 * 				ssd_display_configure()): 0x00 <= address <= 0x4F.
 * 		- For double line display (numberOfLines = DISPLAY_TWO_LINES in
 * 				ssd_display_configure()): 0x00 <= address <= 0x27 and
 * 				0x40 <= address <= 0x67.
 * 		- For triple line display (numberOfLines = DISPLAY_THREE_LINES in
 * 				ssd_display_configure()): 0x00 <= address <= 0x13,
 * 				0x20 <= address <= 0x33, and 0x40 <= address <= 0x53.
 * 		- For quadruple line display (numberOfLines = DISPLAY_FOUR_LINES in
 * 				ssd_display_configure()): 0x00 <= address <= 0x13,
 * 				0x20 <= address <= 0x33, 0x40 <= address <= 0x53, and
 * 				0x60 <= address <= 0x73.
 * @param [out]		data Pointer to the memory location where the data read from
 * the DDRAM will be stored.
 * @param [in]		dataLength Number of bytes to be read from the DDRAM.  Valid
 * values are:
 * 		- 0x00 < dataLength <= 0x80
 * 
 * @retval STATUS_OK If dataLength bytes of data were successfully read from the
 * specified DDRAM address.
 * @retval STATUS_INVALID_PARAM If a parameter value was outside of the valid
 * range listed above.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_FAILED_READ If the message failed to be successfully read from
 * the display MPU.
 * @retval STATUS_INVALID_HANDLE is a theoretically possible return value,
 * however it should never occur.
 * ========================================================================== */
status_t ssd_read_from_DDRAM(
		uint8_t address,
		uint8_t *data,
		uint16_t dataLength
);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Update the address counter to be the specified CGRAM address.
 *		-# Read the specified amount of data from the CGRAM.
 *		-# Reset the address counter to be the previous DDRAM address.
 *
 * @note The address counter will always be incremented after one byte of CGRAM
 * data is read.  Therefore if the starting address is 0x00, then after the
 * first byte of data is ready, the next bytes of data will be read from 0x01
 * and onwards (if multiple data bytes are required).
 * 
 * @require data points to dataLength of previously allocated memory.
 *
 * @param [in]		address Starting CGRAM address that will be read.  Valid
 * values are:
 * 		- 0x00 <= address <= 0x3F
 * @param [out]		data Pointer to the memory location where the data read from
 * the CGRAM will be stored.
 * @param [in]		dataLength Number of bytes to be read from the CGRAM.  Valid
 * values are:
 * 		- 0x00 < dataLength <= 0x40
 * 
 * @retval STATUS_OK If dataLength bytes of data were successfully read from the
 * specified CGRAM address.
 * @retval STATUS_INVALID_PARAM If a parameter value was outside of the valid
 * range listed above.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_FAILED_READ If the message failed to be successfully read from
 * the display MPU.
 * @retval STATUS_INVALID_HANDLE is a theoretically possible return value,
 * however it should never occur.
 * ========================================================================== */
status_t ssd_read_from_CGRAM(
		uint8_t address,
		uint8_t *data,
		uint16_t dataLength
);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Update the address counter to be the specified SEGRAM address.
 *		-# Read the specified amount of data from the SEGRAM.
 *		-# Reset the address counter to be the previous DDRAM address.
 *
 * @note The address counter will always be incremented after one byte of SEGRAM
 * data is read.  Therefore if the starting address is 0x00, then after the
 * first byte of data is ready, the next bytes of data will be read from 0x01
 * and onwards (if multiple data bytes are required).
 * 
 * @require data points to dataLength of previously allocated memory.
 *
 * @param [in]		address Starting SEGRAM address that will be read.  Valid
 * values are:
 * 		- 0x00 <= address <= 0x0F
 * @param [out]		data Pointer to the memory location where the data read from
 * the SEGRAM will be stored.
 * @param [in]		dataLength Number of bytes to be read from the SEGRAM.
 * Valid values are:
 * 		- 0x00 < dataLength <= 0x10
 * 
 * @retval STATUS_OK If dataLength bytes of data were successfully read from the
 * specified SEGRAM address.
 * @retval STATUS_INVALID_PARAM If a parameter value was outside of the valid
 * range listed above.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_FAILED_READ If the message failed to be successfully read from
 * the display MPU.
 * @retval STATUS_INVALID_HANDLE is a theoretically possible return value,
 * however it should never occur.
 * ========================================================================== */
status_t ssd_read_from_SEGRAM(
		uint8_t address,
		uint8_t *data,
		uint16_t dataLength
);

/* ========================================================================== */
/**
 * @details This function implements the following objectives:
 *		-# Transmit a raw command from the SSD1803A instruction set to the
 *				SSD1803A display MPU.  The commands that can be sent using this
 *				function are shown below.  An overview of these commands can be
 *				found on pages 38-50 of the SSD1803A datasheet
 *		(https://www.lcd-module.de/fileadmin/eng/pdf/zubehoer/ssd1803a_2_0.pdf).
 *			- Clear display
 *			- Return home
 *			- Power down mode
 *			- Entry mode set
 *			- Display on/off control
 *			- Extended function set
 *			- Cursor or display shift
 *			- Double height (4-lines)/bias/display-dot shift
 *			- Internal OSC frequency
 *			- Shift enable
 *			- Scroll enable
 *			- Function set
 *			- Set CGRAM address
 *			- Set SEGRAM address
 *			- Power/icon control/contrast set
 *			- Follower control
 *			- Contrast set
 *			- Set DDRAM address
 *			- Set scroll quantity
 *			- Temperature coefficient control
 *			- ROM selection
 *
 * @note Please do not use this function, as it will alter the effectiveness of
 * the other API functions.  Every setting (except for setting the Ion bit) can
 * be adjusted through the previous API functions.  It has been included only
 * for completeness.
 * 
 * @param [in]		command Raw command byte to be transmitted to the SSD1803A
 * display MPU.
 * 
 * @retval STATUS_OK If the raw command byte was successfully transmitted to the
 * SSD1803A display MPU.
 * @retval STATUS_NOT_INITIALISED If the SSD1803A has not been previously
 * initialised with a call to ssd_init().
 * @retval STATUS_FAILED_WRITE If the message failed to be successfully written
 * to the display MPU.
 * @retval STATUS_INVALID_HANDLE and STATUS_INVALID_PARAM are theoretically
 * possible return values, however they should never occur.
 * ========================================================================== */
status_t ssd_write_command_raw(uint8_t command);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // SRC_SSD1803A_H_
