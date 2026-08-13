#ifndef __OLED_H
#define __OLED_H

#include "stm32f10x.h"

/**
 * @file    OLED.h
 * @brief   SSD1306 OLED display driver (128x64, I2C) with framebuffer.
 * @note    Device 0x3C (write address 0x78). Requires soft I2C (i2c.h).
 *          Coordinates: x 0..127 (left to right), y 0..7 (pages,
 *          8 pixels each). 6x8 font -> 21 chars per line, 8 lines.
 *
 *          Framebuffer: ShowChar/Clear/Fill only touch a 1 KB RAM image
 *          (microseconds).  OLED_FlushPage() streams one page over I2C;
 *          the scheduler spreads pages across frames so the main loop
 *          never blocks for a full-screen write.
 */

/* ---- I2C address ---- */
#define OLED_ADDR       0x3C        /* 7-bit I2C address */
#define OLED_ADDR_WRITE (OLED_ADDR << 1)    /* write address = 0x78 */

/* ---- I2C control byte ---- */
/**
 * @name  I2C control bytes
 * @note  First byte of each transfer selects the stream type:
 *        0x00 = following bytes are commands, 0x40 = data (GDDRAM).
 *        Unlike MPU6050, SSD1306 has no register address space.
 */
#define OLED_CTRL_CMD   0x00        /* next bytes are commands */
#define OLED_CTRL_DATA  0x40        /* next bytes are display data */

/* ---- panel size ---- */
#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_PAGES      8           /* 64 px / 8 px per page */

/* ---- public API ---- */

/** @brief Initialize the display (charge pump, addressing, display on). */
void OLED_Init(void);
/** @brief Clear the framebuffer (all pixels off). RAM-only, no I2C. */
void OLED_Clear(void);
/** @brief Fill the framebuffer (all pixels on; test pattern). RAM-only. */
void OLED_Fill(void);

/** @brief Push one framebuffer page (128 bytes) to the panel over I2C.
 *  @param page: page 0..7; blocks ~10-50 ms depending on I2C speed. */
void OLED_FlushPage(uint8_t page);
/** @brief Push all 8 pages (one-time init display, blocking). */
void OLED_FlushAll(void);

/** @brief Stamp one character into the framebuffer at (x, y). RAM-only. */
void OLED_ShowChar(uint8_t x, uint8_t y, char ch);
/** @brief Display a string with automatic line wrap. */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);
/** @brief Display a signed integer right-aligned to len digits. */
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len);
/** @brief Display a float with fixed integer and decimal widths. */
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t intLen, uint8_t decLen);

/** @brief Set GDDRAM write position (page, column). */
void OLED_SetCursor(uint8_t page, uint8_t col);

#endif /* __OLED_H */
