/**
 * @file ssd1306.c
 * @brief the source file for the ssd1306 controller simple source.
 */

#include "ssd1306.h"

/** @brief Used in future algorithms for bounds checking*/
#define DISPLAY_TOTAL_NUM_PIXELS	(DISPLAY_PIXEL_LENGTH * DISPLAY_PIXEL_HEIGHT)
/** @brief Used to define array size for display ram*/
#define DISPLAY_TOTAL_RAM_BYTES		(DISPLAY_TOTAL_NUM_PIXELS/8U)

#if DISPLAY_PIXEL_HEIGHT == 32
static uint8_t init_buf[] = {
		0xAE, /* Display OFF*/
		0xD5, /* Display frame frequency = 370/1*66*31 = ~180Hz*/
		0x80,
		0xA8, /* Display MUX Ratio = 32 (32bit)*/
		0x1F,
		0xD3, /* Display Offset = 0*/
		0x00,
		0x40, /* Display Start Line = beginning*/
		0x8D, /* Enable Charge Pump*/
		0x14,
		0xA1, /* Display Segment Map Col 127 = Seg 0*/
		0xC8, /* Display Scan from COM[N-1] to COM0*/
		0xDA, /* Display Set Sequential COM pin configuration (32bit)*/
		0x02,
		0x81, /* Display Set Contrast*/
		CONFIG_INIT_CONTRAST,
		0xD9, /* PreCharge Period = PH2 = 15DCLK, PH1 = 1DCLK*/
		0xF1,
		0xDB, /* Display VCOMH Deselect level (0.65 x VCC)*/
		0x40,
		0x22, /* Display Page Start & End Address (0 & 3 Respectively) (32bit)*/
		0x00,
		0x03,
		0x20, /* Display Horizontal Addressing Mode*/
		0x00
};
#elif DISPLAY_PIXEL_HEIGHT == 64
static uint8_t init_buf[] = {
		0xAE, /* Display OFF*/
		0xD5, /* Display frame frequency = 370/1*66*31 = ~180Hz*/
		0x80,
		0xA8, /* Display MUX Ratio = 64 (64bit)*/
		0x3F,
		0xD3, /* Display Offset = 0*/
		0x00,
		0x40, /* Display Start Line = beginning*/
		0x8D, /* Enable Charge Pump*/
		0x14,
		0xA1, /* Display Segment Map Col 127 = Seg 0*/
		0xC8, /* Display Scan from COM[N-1] to COM0*/
		0xDA, /* Display Set Sequential COM pin configuration (64bit)*/
		0x12,
		0x81, /* Display Set Contrast*/
		CONFIG_INIT_CONTRAST,
		0xD9, /* PreCharge Period = PH2 = 15DCLK, PH1 = 1DCLK*/
		0xF1,
		0xDB, /* Display VCOMH Deselect level (0.65 x VCC)*/
		0x40,
		0x22, /* Display Page Start & End Address (0 & 7 Respectively) (64bit)*/
		0x00,
		0x07,
		0x20, /* Display Horizontal Addressing Mode*/
		0x00
};
#else
#error "Please select a support pixel height (32 or 64)"
#endif

/** @brief structure used to contain and control the SSD1306 library runtime data*/
typedef struct SSD1306_t
{
	uint8_t disp_region[DISPLAY_TOTAL_RAM_BYTES];					/** Display RAM*/
	bool tx_done;				/** Indicates SPI transmission status*/
	void(*spi_raw_write)(uint8_t * const tx_buf, uint16_t tx_num);	/** Function pointer for SPI write (non-blcoking)*/
	uint8_t * reset_gpio;		/** pointer to register containing reset GPIO */
	uint8_t reset_mask;			/** bit mask used to toggle reset gpio (1 marks the bit of the register corresponding to pin level)*/
	uint8_t * dc_gpio;			/** pointer to register containing Data/Control GPIO*/
	uint8_t dc_mask;			/** bit mask used to toggle data/control gpio (1 marks the bit of the register corresponding to pin level)*/
	uint8_t * cs_gpio;			/** pointer to register containing Chip Select GPIO*/
	uint8_t cs_mask;			/** bit mask used to toggle chip select gpio (1 marks the bit of the register corresponding to pin level)*/
	bool disp_ram_changed;		/** flag inidicating that the display ram has been changed through the API and the screen should be updated*/
}ssd1306_t;

ssd1306_t ssd1306_ctl; /** instance of ssd1306 control structure*/

/* 768 byte ASCII table (could remove first 192 bytes (32 INCOMPLETE entries)).
 * Reason not to is it reduces the need for any computation on data being input to the lookup.
 */
static const uint8_t Font5x7[128][6] = {
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* (space)  */
		{0x00, 0x00, 0x5F, 0x00, 0x00, 0x00}, /* !        */
		{0x00, 0x07, 0x00, 0x07, 0x00, 0x00}, /* "        */
		{0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00}, /* #        */
		{0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00}, /* $        */
		{0x23, 0x13, 0x08, 0x64, 0x62, 0x00}, /* %        */
		{0x36, 0x49, 0x55, 0x22, 0x50, 0x00}, /* &        */
		{0x00, 0x05, 0x03, 0x00, 0x00, 0x00}, /* '        */
		{0x00, 0x1C, 0x22, 0x41, 0x00, 0x00}, /* (        */
		{0x00, 0x41, 0x22, 0x1C, 0x00, 0x00}, /* )        */
		{0x08, 0x2A, 0x1C, 0x2A, 0x08, 0x00}, /* *        */
		{0x08, 0x08, 0x3E, 0x08, 0x08, 0x00}, /* +        */
		{0x00, 0x50, 0x30, 0x00, 0x00, 0x00}, /* ,        */
		{0x08, 0x08, 0x08, 0x08, 0x08, 0x00}, /* -        */
		{0x00, 0x60, 0x60, 0x00, 0x00, 0x00}, /* .        */
		{0x20, 0x10, 0x08, 0x04, 0x02, 0x00}, /* /        */
		{0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00}, /* 0        */
		{0x00, 0x42, 0x7F, 0x40, 0x00, 0x00}, /* 1        */
		{0x42, 0x61, 0x51, 0x49, 0x46, 0x00}, /* 2        */
		{0x21, 0x41, 0x45, 0x4B, 0x31, 0x00}, /* 3        */
		{0x18, 0x14, 0x12, 0x7F, 0x10, 0x00}, /* 4        */
		{0x27, 0x45, 0x45, 0x45, 0x39, 0x00}, /* 5        */
		{0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00}, /* 6        */
		{0x01, 0x71, 0x09, 0x05, 0x03, 0x00}, /* 7        */
		{0x36, 0x49, 0x49, 0x49, 0x36, 0x00}, /* 8        */
		{0x06, 0x49, 0x49, 0x29, 0x1E, 0x00}, /* 9        */
		{0x00, 0x36, 0x36, 0x00, 0x00, 0x00}, /* :        */
		{0x00, 0x56, 0x36, 0x00, 0x00, 0x00}, /* ;        */
		{0x00, 0x08, 0x14, 0x22, 0x41, 0x00}, /* <        */
		{0x14, 0x14, 0x14, 0x14, 0x14, 0x00}, /* =        */
		{0x41, 0x22, 0x14, 0x08, 0x00, 0x00}, /* >        */
		{0x02, 0x01, 0x51, 0x09, 0x06, 0x00}, /* ?        */
		{0x32, 0x49, 0x79, 0x41, 0x3E, 0x00}, /* @        */
		{0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00}, /* A        */
		{0x7F, 0x49, 0x49, 0x49, 0x36, 0x00}, /* B        */
		{0x3E, 0x41, 0x41, 0x41, 0x22, 0x00}, /* C        */
		{0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00}, /* D        */
		{0x7F, 0x49, 0x49, 0x49, 0x41, 0x00}, /* E        */
		{0x7F, 0x09, 0x09, 0x01, 0x01, 0x00}, /* F        */
		{0x3E, 0x41, 0x41, 0x51, 0x32, 0x00}, /* G        */
		{0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00}, /* H        */
		{0x00, 0x41, 0x7F, 0x41, 0x00, 0x00}, /* I        */
		{0x20, 0x40, 0x41, 0x3F, 0x01, 0x00}, /* J        */
		{0x7F, 0x08, 0x14, 0x22, 0x41, 0x00}, /* K        */
		{0x7F, 0x40, 0x40, 0x40, 0x40, 0x00}, /* L        */
		{0x7F, 0x02, 0x04, 0x02, 0x7F, 0x00}, /* M        */
		{0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00}, /* N        */
		{0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00}, /* O        */
		{0x7F, 0x09, 0x09, 0x09, 0x06, 0x00}, /* P        */
		{0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00}, /* Q        */
		{0x7F, 0x09, 0x19, 0x29, 0x46, 0x00}, /* R        */
		{0x46, 0x49, 0x49, 0x49, 0x31, 0x00}, /* S        */
		{0x01, 0x01, 0x7F, 0x01, 0x01, 0x00}, /* T        */
		{0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00}, /* U        */
		{0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00}, /* V        */
		{0x7F, 0x20, 0x18, 0x20, 0x7F, 0x00}, /* W        */
		{0x63, 0x14, 0x08, 0x14, 0x63, 0x00}, /* X        */
		{0x03, 0x04, 0x78, 0x04, 0x03, 0x00}, /* Y        */
		{0x61, 0x51, 0x49, 0x45, 0x43, 0x00}, /* Z        */
		{0x00, 0x00, 0x7F, 0x41, 0x41, 0x00}, /* [        */
		{0x02, 0x04, 0x08, 0x10, 0x20, 0x00}, /* "\"      */
		{0x41, 0x41, 0x7F, 0x00, 0x00, 0x00}, /* ]        */
		{0x04, 0x02, 0x01, 0x02, 0x04, 0x00}, /* ^        */
		{0x40, 0x40, 0x40, 0x40, 0x40, 0x00}, /* _        */
		{0x00, 0x01, 0x02, 0x04, 0x00, 0x00}, /* `        */
		{0x20, 0x54, 0x54, 0x54, 0x78, 0x00}, /* a        */
		{0x7F, 0x48, 0x44, 0x44, 0x38, 0x00}, /* b        */
		{0x38, 0x44, 0x44, 0x44, 0x20, 0x00}, /* c        */
		{0x38, 0x44, 0x44, 0x48, 0x7F, 0x00}, /* d        */
		{0x38, 0x54, 0x54, 0x54, 0x18, 0x00}, /* e        */
		{0x08, 0x7E, 0x09, 0x01, 0x02, 0x00}, /* f        */
		{0x08, 0x14, 0x54, 0x54, 0x3C, 0x00}, /* g        */
		{0x7F, 0x08, 0x04, 0x04, 0x78, 0x00}, /* h        */
		{0x00, 0x44, 0x7D, 0x40, 0x00, 0x00}, /* i        */
		{0x20, 0x40, 0x44, 0x3D, 0x00, 0x00}, /* j        */
		{0x00, 0x7F, 0x10, 0x28, 0x44, 0x00}, /* k        */
		{0x00, 0x41, 0x7F, 0x40, 0x00, 0x00}, /* l        */
		{0x7C, 0x04, 0x18, 0x04, 0x78, 0x00}, /* m        */
		{0x7C, 0x08, 0x04, 0x04, 0x78, 0x00}, /* n        */
		{0x38, 0x44, 0x44, 0x44, 0x38, 0x00}, /* o        */
		{0x7C, 0x14, 0x14, 0x14, 0x08, 0x00}, /* p        */
		{0x08, 0x14, 0x14, 0x18, 0x7C, 0x00}, /* q        */
		{0x7C, 0x08, 0x04, 0x04, 0x08, 0x00}, /* r        */
		{0x48, 0x54, 0x54, 0x54, 0x20, 0x00}, /* s        */
		{0x04, 0x3F, 0x44, 0x40, 0x20, 0x00}, /* t        */
		{0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00}, /* u        */
		{0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00}, /* v        */
		{0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00}, /* w        */
		{0x44, 0x28, 0x10, 0x28, 0x44, 0x00}, /* x        */
		{0x0C, 0x50, 0x50, 0x50, 0x3C, 0x00}, /* y        */
		{0x44, 0x64, 0x54, 0x4C, 0x44, 0x00}, /* z        */
		{0x00, 0x08, 0x36, 0x41, 0x00, 0x00}, /* {        */
		{0x00, 0x00, 0x7F, 0x00, 0x00, 0x00}, /* |        */
		{0x00, 0x41, 0x36, 0x08, 0x00, 0x00}, /* }        */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}, /* INCOMPLETE */
};

#if CONFIG_ASM_ACC == 1
/**
 * @brief Sets a pixel at position x and y, where (0,0) is top left and (x-max, y-max) is bottom right.
 * @param x - x position of pixel
 * @param y - y position of pixel
 */
extern void Set_pixel(uint16_t x, uint16_t y);

/**
 * @brief Clears a pixel at position x and y, where (0,0) is top left and (x-max, y-max) is bottom right.
 * @param x - x position of pixel
 * @param y - y position of pixel
 */
extern void Clear_pixel(uint16_t x, uint16_t y);
#else
/**
 * @brief Sets a pixel at position x and y, where (0,0) is top left and (x-max, y-max) is bottom right.
 * @param x - x position of pixel
 * @param y - y position of pixel
 */
static inline void Set_pixel(uint16_t x, uint16_t y)
{
	/* Take y/8 (y >> 3) gives us the y offset in rows...
	 * Multiply the offset in rows by the number of bytes in a row and this gives us the byte offset in the 1d buffer*/
	const uint16_t y_row = (y >> 3U) << DISPLAY_PIXEL_LENGTH_LOG2;
	const uint8_t y_bit = y & 7U; /* Lower 3 bits contain pixel offset within byte*/
	ssd1306_ctl.disp_region[x+y_row] |= 0x01 << y_bit;
}
/* END OF FUNCTION*/

/**
 * @brief Clears a pixel at position x and y, where (0,0) is top left and (x-max, y-max) is bottom right.
 * @param x - x position of pixel
 * @param y - y position of pixel
 */
static inline void Clear_pixel(uint16_t x, uint16_t y)
{
	/* Take y/8 (y >> 3) gives us the y offset in rows...
	 * Multiply the offset in rows by the number of bytes in a row and this gives us the byte offset in the 1d buffer*/
	const uint16_t y_row = (y >> 3U) << DISPLAY_PIXEL_LENGTH_LOG2;
	const uint8_t y_bit = y & 7U; /* Lower 3 bits contain pixel offset within byte*/
	ssd1306_ctl.disp_region[x+y_row] &= ~(0x01 << y_bit);
}
/* END OF FUNCTION*/
#endif

/**
 * @brief Forms parts of the bresenham line drawing algorithm.
 * @param x0 - initial x.
 * @param y0 - initial y.
 * @param x1 - final x.
 * @param y1 - final y.
 * @param dx - x1-x0.
 * @param dy - y1-y0.
 */
static void Plot_line_low(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, int16_t dx, int16_t dy)
{
	int16_t yi = 1;

    if(dy < 0)
    {
        yi = -1;
        dy = -dy;
    }

    int16_t D = (2*dy) - dx;
    int16_t y = y0;

    for (uint16_t x = x0; x < x1; ++x)
    {
        Set_pixel(x, (uint16_t)(y));
        if(D > 0)
        {
            y = y + yi;
            D = D + (2*(dy - (int16_t)(dx)));
        }
        else
        {
            D = D + 2*dy;
        }
    }
}
/* END OF FUNCTION*/

/**
 * @brief Forms parts of the bresenham line drawing algorithm.
 * @param x0 - initial x.
 * @param y0 - initial y.
 * @param x1 - final x.
 * @param y1 - final y.
 * @param dx - x1-x0.
 * @param dy - y1-y0.
 */
static void Plot_line_high(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, int16_t dx, int16_t dy)
{
	int16_t xi = 1;

    if(dx < 0)
    {
        xi = -1;
        dx = -dx;
    }

    int16_t D = (2*dx) - dy;
    int16_t x = x0;

    for (uint16_t y = (int16_t)(y0); y < (int16_t)(y1); ++y)
    {
        Set_pixel((uint16_t)(x), y);
        if(D > 0)
        {
            x = x + xi;
            D = D + (2*(dx - dy));
        }
        else
        {
            D = D + 2*dx;
        }
    }
}
/* END OF FUNCTION*/

/**
 * @brief Forms parts of the modified bresenham line drawing algorithm to draw two parallel lines.
 * @param x0 - initial x.
 * @param y0 - initial y.
 * @param x1 - final x.
 * @param y1 - final y.
 * @param dx - x1-x0.
 * @param dy - y1-y0.
 * @param offset - distance between described line and parallel one.
 */
static void Plot_parallel_lines_low(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, int16_t dx, int16_t dy, uint16_t offset)
{
	int16_t yi = 1;

    if(dy < 0)
    {
        yi = -1;
        dy = -dy;
    }

    int16_t D = (2*dy) - dx;
    int16_t y = y0;

    for (uint16_t x = x0; x < x1; ++x)
    {
        Set_pixel(x, (uint16_t)(y));
        Set_pixel(x, (uint16_t)(y)+offset);
        if(D > 0)
        {
            y = y + yi;
            D = D + (2*(dy - (int16_t)(dx)));
        }
        else
        {
            D = D + 2*dy;
        }
    }
}
/* END OF FUNCTION*/

/**
 * @brief Forms parts of the modified bresenham line drawing algorithm to draw two parallel lines.
 * @param x0 - initial x.
 * @param y0 - initial y.
 * @param x1 - final x.
 * @param y1 - final y.
 * @param dx - x1-x0.
 * @param dy - y1-y0.
 * @param offset - distance between described line and parallel one.
 */
static void Plot_parallel_lines_high(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, int16_t dx, int16_t dy, uint16_t offset)
{
	int16_t xi = 1;

    if(dx < 0)
    {
        xi = -1;
        dx = -dx;
    }

    int16_t D = (2*dx) - dy;
    int16_t x = x0;

    for (uint16_t y = y0; y < y1; ++y)
    {
        Set_pixel((uint16_t)(x), y);
        Set_pixel((uint16_t)(x)+offset, y);
        if(D > 0)
        {
            x = x + xi;
            D = D + (2*(dx - dy));
        }
        else
        {
            D = D + 2*dx;
        }
    }
}
/* END OF FUNCTION*/

/**
 * @brief Writes a raw ssd1306 command byte.
 * @param value
 */
static void Ssd1306_write_cmd(uint8_t value)
{
	/* Ensure SPI isn't busy */
	while(!ssd1306_ctl.tx_done)
	{
		SSD1306_NOP();
	}

	ssd1306_ctl.tx_done = false;

	/* Assert control signals - de-asserted in the spi callback*/
#if CONFIG_SPI_INTEGRATED_CS == 0
	*ssd1306_ctl.cs_gpio &= ~ssd1306_ctl.cs_mask;
#endif
	*ssd1306_ctl.dc_gpio &= ~ssd1306_ctl.dc_mask;

	/* Start the write sequence*/
	ssd1306_ctl.spi_raw_write(&value, 1U);

	/* Wait for write to complete*/
	while(!ssd1306_ctl.tx_done)
	{
		SSD1306_NOP();
	}
}
/* END OF FUNCTION*/
/** @brief Writes the initialisation buffer of the ssd1306. */
static void Ssd1306_write_init(void)
{
	/* Ensure SPI isn't busy */
	while(!ssd1306_ctl.tx_done)
	{
		SSD1306_NOP();
	}

	ssd1306_ctl.tx_done = false;

	/* Assert control signals - de-asserted in the spi callback*/
#if CONFIG_SPI_INTEGRATED_CS == 0
	*ssd1306_ctl.cs_gpio &= ~ssd1306_ctl.cs_mask;
#endif
	*ssd1306_ctl.dc_gpio &= ~ssd1306_ctl.dc_mask;

	/* Start the write sequence*/
	ssd1306_ctl.spi_raw_write(init_buf, sizeof(init_buf));

	/* Wait for write to complete*/
	while(!ssd1306_ctl.tx_done)
	{
		SSD1306_NOP();
	}
}
/* END OF FUNCTION*/

/** @brief Turns on and starts displaying the gddram of the SSD1306*/
static void Ssd1306_display_on(void)
{
	Ssd1306_write_cmd(0xA4); /* Turn on the display*/
	Ssd1306_write_cmd(0xA6); /* Display Normal Mode*/
	Ssd1306_write_cmd(0xAF); /* Display ON in Normal Mode*/
}
/* END OF FUNCTION*/

void Ssd1306_init(void(*spi_raw_write)(uint8_t * const tx_buf, uint16_t tx_num), uint8_t * reset_gpio, const uint8_t reset_mask, uint8_t * dc_gpio, const uint8_t dc_mask, uint8_t * cs_gpio, const uint8_t cs_mask)
{
	ssd1306_ctl.tx_done = true;
	for(uint16_t i = 0U; i < DISPLAY_TOTAL_RAM_BYTES; ++i)
	{
		ssd1306_ctl.disp_region[i] = 0x0U;
	}
	ssd1306_ctl.spi_raw_write = spi_raw_write;
	ssd1306_ctl.reset_gpio = reset_gpio;
	ssd1306_ctl.dc_gpio = dc_gpio;
	ssd1306_ctl.cs_gpio = cs_gpio;
	ssd1306_ctl.reset_mask = reset_mask;
	ssd1306_ctl.dc_mask = dc_mask;
	ssd1306_ctl.cs_mask = cs_mask;
	ssd1306_ctl.disp_ram_changed = true; /* Enable initial write to display*/

	/* Reset device*/
	*ssd1306_ctl.reset_gpio &= ~ssd1306_ctl.reset_mask;
	SSD1306_NOP();
	*ssd1306_ctl.reset_gpio |= ssd1306_ctl.reset_mask;

	Ssd1306_write_init(); /* Init the display*/
	Ssd1306_blocking_refresh(); /* Update GDDRAM*/
	Ssd1306_display_on(); /* Turn on the display*/
}
/* END OF FUNCTION*/

void Ssd1306_write_raw_to_ram(uint8_t const * raw_data, const uint16_t loc, const uint16_t len)
{
	/* Write the data into the display buffer - while checking the written data is an actual change to the buffer*/
	uint16_t change_cnt = 0U;

	for(uint16_t i = 0U; i < len; ++i)
	{
		if(ssd1306_ctl.disp_region[loc+i] != raw_data[i])
		{
			change_cnt += 1U;
			ssd1306_ctl.disp_region[loc+i] = raw_data[i];
		}
	}

	ssd1306_ctl.disp_ram_changed = change_cnt > 0U;
}
/* END OF FUNCTION*/

void Ssd1306_write_str_to_ram(char const * str, const uint16_t loc, const uint16_t len)
{
	/* Write the data into the display buffer - while checking the written data is an actual change to the buffer*/
	uint16_t change_cnt = 0U;

	for(uint16_t i = 0; i < len; ++i)
	{
		const uint16_t offset = i * 6;

		for(uint16_t j = 0; j < 6; ++j)
		{
			/* place the string characters through the lookup table*/
			uint8_t * const cur_dat = &ssd1306_ctl.disp_region[loc + offset + j];
			const uint8_t new_dat = Font5x7[*(str+i)][j];
			if(*cur_dat != new_dat)
			{
				change_cnt += 1U;
				*cur_dat = new_dat;
			}
		}
	}

	ssd1306_ctl.disp_ram_changed = change_cnt > 0U;
}
/* END OF FUNCTION*/

void Ssd1306_write_line_to_ram(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	uint16_t dy = (y0>y1) ? y0 - y1 : y1 - y0;
	uint16_t dx = (x0>x1) ? x0 - x1 : x1 - x0;
	bool rev_inputs = (y0>y1) || (x0>x1);

    if(dy < dx)
    {
        if(rev_inputs)
        {
            Plot_line_low(x1, y1, x0, y0, dx, dy);
        }
        else
        {
        	Plot_line_low(x0, y0, x1, y1, dx, dy);
        }
    }
    else
    {
        if(rev_inputs)
        {
        	Plot_line_high(x1, y1, x0, y0, dx, dy);
        }
        else
        {
        	Plot_line_high(x0, y0, x1, y1, dx, dy);
        }
    }

    ssd1306_ctl.disp_ram_changed = true;
}
/* END OF FUNCTION*/

void Ssd1306_write_parallel_lines_to_ram(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t offset)
{
	uint16_t dy = (y0>y1) ? y0 - y1 : y1 - y0;
	uint16_t dx = (x0>x1) ? x0 - x1 : x1 - x0;
	bool rev_inputs = (y0>y1) || (x0>x1);

    if(dy < dx)
    {
        if(rev_inputs)
        {
            Plot_parallel_lines_low(x1, y1, x0, y0, dx, dy, offset);
        }
        else
        {
        	Plot_parallel_lines_low(x0, y0, x1, y1, dx, dy, offset);
        }
    }
    else
    {
        if(rev_inputs)
        {
        	Plot_parallel_lines_high(x1, y1, x0, y0, dx, dy, offset);
        }
        else
        {
        	Plot_parallel_lines_high(x0, y0, x1, y1, dx, dy, offset);
        }
    }

    ssd1306_ctl.disp_ram_changed = true;
}
/* END OF FUNCTION*/

void Ssd1306_write_rectangle_to_ram(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height)
{
	Ssd1306_write_parallel_lines_to_ram(x0,y0,x0+width,y0, height);
	Ssd1306_write_parallel_lines_to_ram(x0,y0,x0,y0+height, width);
}
/* END OF FUNCTION*/

void Ssd1306_blocking_refresh(void)
{
	/* Only update if display ram has changed*/
	if(ssd1306_ctl.disp_ram_changed)
	{
		ssd1306_ctl.disp_ram_changed = false;

		/* Ensure SPI is free*/
		while(!ssd1306_ctl.tx_done)
		{
			SSD1306_NOP();
		}

		ssd1306_ctl.tx_done = false;

		/* Assert signals*/
#if CONFIG_SPI_INTEGRATED_CS == 0
		*ssd1306_ctl.cs_gpio &= ~ssd1306_ctl.cs_mask;
#endif
		*ssd1306_ctl.dc_gpio |= ssd1306_ctl.dc_mask;

		/* Write to GDDRAM*/
		ssd1306_ctl.spi_raw_write(ssd1306_ctl.disp_region, DISPLAY_TOTAL_RAM_BYTES);

		/* Wait for write to complete*/
		while(!ssd1306_ctl.tx_done)
		{
			SSD1306_NOP();
		}
	}
}
/* END OF FUNCTION*/

void Ssd1306_non_blocking_refresh(void)
{
	/* Only update if display ram has changed*/
	if(ssd1306_ctl.disp_ram_changed)
	{
		ssd1306_ctl.disp_ram_changed = false;

		/* Ensure SPI is free*/
		while(!ssd1306_ctl.tx_done)
		{
			SSD1306_NOP();
		}

		ssd1306_ctl.tx_done = false;

		/* Assert signals*/
#if CONFIG_SPI_INTEGRATED_CS == 0
		*ssd1306_ctl.cs_gpio &= ~ssd1306_ctl.cs_mask;
#endif
		*ssd1306_ctl.dc_gpio |= ssd1306_ctl.dc_mask;

		/* Write to GDDRAM*/
		ssd1306_ctl.spi_raw_write(ssd1306_ctl.disp_region, DISPLAY_TOTAL_RAM_BYTES);
	}
}
/* END OF FUNCTION*/

bool Ssd1306_is_busy(void)
{
	return !ssd1306_ctl.tx_done; /* Device is busy is tx is not done (false)*/
}
/* END OF FUNCTION*/

void Ssd1306_spi_tx_done_callback(void)
{
	/* Set tx to done (no longer busy) and de-assert control signals*/
	ssd1306_ctl.tx_done = true;
#if CONFIG_SPI_INTEGRATED_CS == 0
	*ssd1306_ctl.cs_gpio |= ssd1306_ctl.cs_mask;
#endif
	*ssd1306_ctl.dc_gpio |= ssd1306_ctl.dc_mask;
}
/* END OF FUNCTION*/
