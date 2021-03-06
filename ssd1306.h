/**
 * @file ssd1306.h
 * @brief the header file for the ssd1306 controller simple source.
 * @note This source ONLY supports 8bit SPI control mode & remember to set the CONFIG_SPI_INTEGRATED_CS macro to fit the applications available HW.
 */

#ifndef SSD1306_H_
#define SSD1306_H_

#include "stdint.h"
#include "stdbool.h"

/** @brief Length of display in pixels.*/
#define DISPLAY_PIXEL_LENGTH		(128U)
/** @brief Log base 2 of length of display in pixels.*/
#define DISPLAY_PIXEL_LENGTH_LOG2	(7U)
/** @brief Height of display in pixels.*/
#define DISPLAY_PIXEL_HEIGHT		(32U)
/** @brief Inline assembly used for nop.*/
#define SSD1306_NOP()				__nop()
/** @brief Inline assembly used for putting device in low power mode when performing blocking operations.*/
#define SSD1306_SLEEP()				__halt()
/** @brief Inline assembly used for putting device in ultra low power mode when performing blocking operations.*/
#define SSD1306_DEEP_SLEEP()		__stop()
/** @brief Configuration macro - sets initial contrast of display*/
#define CONFIG_INIT_CONTRAST		(0x8F)
/** @brief Configuration macro - set to 1 if an assembly acceleration file is included to improve drawing speed of internal RAM*/
#define CONFIG_ASM_ACC				(1)
/** @brief Configuration macro - set to 1 if the DTC is being used to perform SPI transactions*/
#define CONFIG_USE_DTC				(1)
/** @brief Configuration macro:
 * 0 - perform NOP while waiting for SPI blocking transmissions to complete
 * 1 - Enter SLEEP mode while waiting for SPI blocking transmissions to complete
 * 2 - Enter DEEP_SLEEP mode while waiting for SPI blocking transmissions to complete (Not currently supported)
 */
#define CONFIG_STATE_WHILE_WAIT		(1)

/**
 * @brief Complete initialisation of the SSD1306 driver module for a given display.
 * @param spi_raw_write - function pointer for NON-BLOCKING SPI write function.
 * @param reset_gpio - pointer to register holding reset gpio.
 * @param reset_mask - bit mask used to toggle reset gpio (1 marks the bit of the register corresponding to pin level)
 * @param dc_gpio - pointer to register holding data/control gpio.
 * @param dc_mask - bit mask used to toggle data/control gpio (1 marks the bit of the register corresponding to pin level)
 * @param cs_gpio - pointer to register holding chip select gpio - NULL if unused.
 * @param cs_mask - bit mask used to toggle chip select gpio (1 marks the bit of the register corresponding to pin level) - 0 if unused.
 */
void Ssd1306_init(void(*spi_raw_write)(uint8_t * const tx_buf, uint16_t tx_num), uint8_t * reset_gpio, const uint8_t reset_mask, uint8_t * dc_gpio, const uint8_t dc_mask, uint8_t * cs_gpio, const uint8_t cs_mask);

/** @brief Turns on and starts displaying the gddram of the SSD1306*/
void Ssd1306_display_on(void);

/** @brief Send display and SSD1306 into sleep mode*/
void Ssd1306_display_off(void);

/**
 * @brief Function used to write raw data to internal display ram.
 * @param raw_data - pointer to raw 8 bit aray of data.
 * @param row - the row in the internal ram to start writing.
 * @param col - the column in the internal ram to start writing.
 * @param len - length of data to write.
 */
void Ssd1306_write_raw_to_ram(uint8_t const * raw_data, const uint16_t row, const uint16_t col, const uint16_t len);

/**
 * @brief Function used to write a string to internal display ram.
 * @param str - pointer to the string to write.
 * @param row - the row in the internal ram to start writing.
 * @param col - the column in the internal ram to start writing.
 * @param len - length of string to write.
 */
void Ssd1306_write_str_to_ram(char const * str, const uint16_t row, const uint16_t col, const uint16_t len);

/**
 * @brief Draws a line in the display RAM (top left to bottom right --> 0,0 --> max,max).
 * @details Implementation ported from pseudo code on wikipedia: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm#Algorithm_for_integer_arithmetic
 * @param x0 - initial x.
 * @param y0 - initial y.
 * @param x1 - final x.
 * @param y1 - final y.
 */
void Ssd1306_write_line_to_ram(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/**
 * @brief Draws two parallel lines in the display RAM (top left to bottom right --> 0,0 --> max,max).
 * @details Modified Implementation of the ported pseudo code on wikipedia: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm#Algorithm_for_integer_arithmetic
 * @param x0 - initial x.
 * @param y0 - initial y.
 * @param x1 - final x.
 * @param y1 - final y.
 * @param offset - distance between the line described and the one parallel to it.
 */
void Ssd1306_write_parallel_lines_to_ram(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t offset);

/**
 * @brief draws a rectangle in the display ram - provided with the cordinates describing the rectangles top line + height.
 * @param x0 - initial x of top line.
 * @param y0 - initial y of top line.
 * @param width - width of the box (x direction).
 * @param height - height of the box (y direction).
 */
void Ssd1306_write_rectangle_to_ram(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height);

/** @brief Performs a blocking refresh of the display*/
void Ssd1306_blocking_refresh(void);

/** @brief indicates whether the SPI is busy on the display*/
bool Ssd1306_is_busy(void);

/**
 * @brief Marks a region on the display to be marked for update
 * @param page_start - starting row of dirty area
 * @param page_end - last row of dirty area
 * @param col_start - starting column of dirty area
 * @param col_end - last column of dirty area
 * @return true if successfully added - false if dirty mark buffer is full.
 */
bool Ssd1306_dirty_mark(uint8_t const page_start, uint8_t const page_end, const uint16_t col_start, const uint16_t col_end);

/** @brief dirty marks entire display. */
void Ssd1306_dirty_mark_all(void);

/** @brief User MUST call this function inside the SPI tx complete ISR.*/
void Ssd1306_spi_tx_done_callback(void);

#if CONFIG_USE_DTC == 1
/**
 * @brief Function to setup DTC API.
 * @param dtc_enable - function pointer API which enables the DTC to accept activation source.
 * @param dtc_disable - function pointer API which disables the DTC from accepting activation source.
 * @param dtc_update_source_and_size - function pointer API which allows an update of the DTC source address and size of the transfer.
 */
void Ssd1306_setup_dtc_api(void(*dtc_enable)(void), void(*dtc_update_source_and_size)(uint16_t src_addr, uint16_t siz_of_xfer));
#endif

#endif /* SSD1306_H_ */
