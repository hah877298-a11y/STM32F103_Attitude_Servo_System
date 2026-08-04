#include "OLED.h"
#include "i2c.h"

/**
 * @file    OLED.c
 * @brief   SSD1306 OLED display driver (128x64, I2C).
 * @note    GDDRAM: 8 pages x 128 columns; one byte = one 8-pixel column
 *          (bit0 = top). First byte of each transfer is the control byte:
 *          0x00 = command, 0x40 = data. Device address 0x3C (write 0x78).
 */

/** 6x8 ASCII font (0x20..0x7E), 6 bytes per character.
 *  One byte = one column, LSB = top pixel; index = (ch - 0x20) * 6. */
static const uint8_t Font6x8[][6] =
{
    /* 0x20 ' ' */ { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    /* 0x21 '!' */ { 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00 },
    /* 0x22 '"' */ { 0x00, 0x07, 0x00, 0x07, 0x00, 0x00 },
    /* 0x23 '#' */ { 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00 },
    /* 0x24 '$' */ { 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00 },
    /* 0x25 '%' */ { 0x23, 0x13, 0x08, 0x64, 0x62, 0x00 },
    /* 0x26 '&' */ { 0x36, 0x49, 0x55, 0x22, 0x50, 0x00 },
    /* 0x27 ''' */ { 0x00, 0x05, 0x03, 0x00, 0x00, 0x00 },
    /* 0x28 '(' */ { 0x00, 0x1C, 0x22, 0x41, 0x00, 0x00 },
    /* 0x29 ')' */ { 0x00, 0x41, 0x22, 0x1C, 0x00, 0x00 },
    /* 0x2A '*' */ { 0x08, 0x2A, 0x1C, 0x2A, 0x08, 0x00 },
    /* 0x2B '+' */ { 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00 },
    /* 0x2C ',' */ { 0x00, 0x50, 0x30, 0x00, 0x00, 0x00 },
    /* 0x2D '-' */ { 0x08, 0x08, 0x08, 0x08, 0x08, 0x00 },
    /* 0x2E '.' */ { 0x00, 0x60, 0x60, 0x00, 0x00, 0x00 },
    /* 0x2F '/' */ { 0x20, 0x10, 0x08, 0x04, 0x02, 0x00 },

    /* 0x30 '0' */ { 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00 },
    /* 0x31 '1' */ { 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00 },
    /* 0x32 '2' */ { 0x42, 0x61, 0x51, 0x49, 0x46, 0x00 },
    /* 0x33 '3' */ { 0x21, 0x41, 0x45, 0x4B, 0x31, 0x00 },
    /* 0x34 '4' */ { 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00 },
    /* 0x35 '5' */ { 0x27, 0x45, 0x45, 0x45, 0x39, 0x00 },
    /* 0x36 '6' */ { 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00 },
    /* 0x37 '7' */ { 0x01, 0x71, 0x09, 0x05, 0x03, 0x00 },
    /* 0x38 '8' */ { 0x36, 0x49, 0x49, 0x49, 0x36, 0x00 },
    /* 0x39 '9' */ { 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00 },
    /* 0x3A ':' */ { 0x00, 0x36, 0x36, 0x00, 0x00, 0x00 },
    /* 0x3B ';' */ { 0x00, 0x56, 0x36, 0x00, 0x00, 0x00 },
    /* 0x3C '<' */ { 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 },
    /* 0x3D '=' */ { 0x14, 0x14, 0x14, 0x14, 0x14, 0x00 },
    /* 0x3E '>' */ { 0x00, 0x41, 0x22, 0x14, 0x08, 0x00 },
    /* 0x3F '?' */ { 0x02, 0x01, 0x51, 0x09, 0x06, 0x00 },

    /* 0x40 '@' */ { 0x32, 0x49, 0x79, 0x41, 0x3E, 0x00 },
    /* 0x41 'A' */ { 0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00 },
    /* 0x42 'B' */ { 0x7F, 0x49, 0x49, 0x49, 0x36, 0x00 },
    /* 0x43 'C' */ { 0x3E, 0x41, 0x41, 0x41, 0x22, 0x00 },
    /* 0x44 'D' */ { 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00 },
    /* 0x45 'E' */ { 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00 },
    /* 0x46 'F' */ { 0x7F, 0x09, 0x09, 0x01, 0x01, 0x00 },
    /* 0x47 'G' */ { 0x3E, 0x41, 0x41, 0x51, 0x32, 0x00 },
    /* 0x48 'H' */ { 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00 },
    /* 0x49 'I' */ { 0x00, 0x41, 0x7F, 0x41, 0x00, 0x00 },
    /* 0x4A 'J' */ { 0x20, 0x40, 0x41, 0x3F, 0x01, 0x00 },
    /* 0x4B 'K' */ { 0x7F, 0x08, 0x14, 0x22, 0x41, 0x00 },
    /* 0x4C 'L' */ { 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00 },
    /* 0x4D 'M' */ { 0x7F, 0x02, 0x04, 0x02, 0x7F, 0x00 },
    /* 0x4E 'N' */ { 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00 },
    /* 0x4F 'O' */ { 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00 },

    /* 0x50 'P' */ { 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00 },
    /* 0x51 'Q' */ { 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00 },
    /* 0x52 'R' */ { 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00 },
    /* 0x53 'S' */ { 0x46, 0x49, 0x49, 0x49, 0x31, 0x00 },
    /* 0x54 'T' */ { 0x01, 0x01, 0x7F, 0x01, 0x01, 0x00 },
    /* 0x55 'U' */ { 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00 },
    /* 0x56 'V' */ { 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00 },
    /* 0x57 'W' */ { 0x7F, 0x20, 0x18, 0x20, 0x7F, 0x00 },
    /* 0x58 'X' */ { 0x63, 0x14, 0x08, 0x14, 0x63, 0x00 },
    /* 0x59 'Y' */ { 0x03, 0x04, 0x78, 0x04, 0x03, 0x00 },
    /* 0x5A 'Z' */ { 0x61, 0x51, 0x49, 0x45, 0x43, 0x00 },
    /* 0x5B '[' */ { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00 },
    /* 0x5C '\' */ { 0x02, 0x04, 0x08, 0x10, 0x20, 0x00 },
    /* 0x5D ']' */ { 0x41, 0x41, 0x7F, 0x00, 0x00, 0x00 },
    /* 0x5E '^' */ { 0x04, 0x02, 0x01, 0x02, 0x04, 0x00 },
    /* 0x5F '_' */ { 0x40, 0x40, 0x40, 0x40, 0x40, 0x00 },

    /* 0x60 '`' */ { 0x00, 0x01, 0x02, 0x04, 0x00, 0x00 },
    /* 0x61 'a' */ { 0x20, 0x54, 0x54, 0x54, 0x78, 0x00 },
    /* 0x62 'b' */ { 0x7F, 0x48, 0x44, 0x44, 0x38, 0x00 },
    /* 0x63 'c' */ { 0x38, 0x44, 0x44, 0x44, 0x20, 0x00 },
    /* 0x64 'd' */ { 0x38, 0x44, 0x44, 0x48, 0x7F, 0x00 },
    /* 0x65 'e' */ { 0x38, 0x54, 0x54, 0x54, 0x18, 0x00 },
    /* 0x66 'f' */ { 0x08, 0x7E, 0x09, 0x01, 0x02, 0x00 },
    /* 0x67 'g' */ { 0x08, 0x54, 0x54, 0x54, 0x3C, 0x00 },
    /* 0x68 'h' */ { 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00 },
    /* 0x69 'i' */ { 0x00, 0x44, 0x7D, 0x40, 0x00, 0x00 },
    /* 0x6A 'j' */ { 0x20, 0x40, 0x44, 0x3D, 0x00, 0x00 },
    /* 0x6B 'k' */ { 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 },
    /* 0x6C 'l' */ { 0x00, 0x41, 0x7F, 0x40, 0x00, 0x00 },
    /* 0x6D 'm' */ { 0x7C, 0x04, 0x18, 0x04, 0x78, 0x00 },
    /* 0x6E 'n' */ { 0x7C, 0x08, 0x04, 0x04, 0x78, 0x00 },
    /* 0x6F 'o' */ { 0x38, 0x44, 0x44, 0x44, 0x38, 0x00 },

    /* 0x70 'p' */ { 0x7C, 0x14, 0x14, 0x14, 0x08, 0x00 },
    /* 0x71 'q' */ { 0x08, 0x14, 0x14, 0x18, 0x7C, 0x00 },
    /* 0x72 'r' */ { 0x7C, 0x08, 0x04, 0x04, 0x08, 0x00 },
    /* 0x73 's' */ { 0x48, 0x54, 0x54, 0x54, 0x20, 0x00 },
    /* 0x74 't' */ { 0x04, 0x3F, 0x44, 0x40, 0x20, 0x00 },
    /* 0x75 'u' */ { 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00 },
    /* 0x76 'v' */ { 0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00 },
    /* 0x77 'w' */ { 0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00 },
    /* 0x78 'x' */ { 0x44, 0x28, 0x10, 0x28, 0x44, 0x00 },
    /* 0x79 'y' */ { 0x0C, 0x50, 0x50, 0x50, 0x3C, 0x00 },
    /* 0x7A 'z' */ { 0x44, 0x64, 0x54, 0x4C, 0x44, 0x00 },
    /* 0x7B '{' */ { 0x00, 0x08, 0x36, 0x41, 0x00, 0x00 },
    /* 0x7C '|' */ { 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00 },
    /* 0x7D '}' */ { 0x00, 0x41, 0x36, 0x08, 0x00, 0x00 },
    /* 0x7E '~' */ { 0x08, 0x04, 0x08, 0x10, 0x08, 0x00 },
};

/**
 * @brief  Send one command byte to the controller.
 * @param  cmd: command byte
 * @note   Transfer: [S][0x78 addr+W][0x00 command][cmd][P]
 */
static void OLED_WriteCmd(uint8_t cmd)
{
    I2C_Start();
    I2C_SendByte(OLED_ADDR_WRITE);   /* 0x78: addr + write */
    I2C_WaitAck();
    I2C_SendByte(OLED_CTRL_CMD);     /* 0x00: control byte = command */
    I2C_WaitAck();
    I2C_SendByte(cmd);
    I2C_WaitAck();
    I2C_Stop();
}

/**
 * @brief  Send multiple data bytes in a single I2C transaction.
 * @param  buf: data buffer
 * @param  len: byte count
 * @note   Transfer: [S][0x78][0x40][data...][P]
 */
static void OLED_WriteMultiData(const uint8_t *buf, uint8_t len)
{
    uint8_t i;

    I2C_Start();
    I2C_SendByte(OLED_ADDR_WRITE);
    I2C_WaitAck();
    I2C_SendByte(OLED_CTRL_DATA);    /* 0x40: control byte = data */
    I2C_WaitAck();

    for (i = 0; i < len; i++)
    {
        I2C_SendByte(buf[i]);
        I2C_WaitAck();
    }

    I2C_Stop();
}

/**
 * @brief  Set GDDRAM write position; the column pointer auto-increments
 *         after each written byte.
 * @param  page: page 0..7 (screen row = page * 8)
 * @param  col:  column 0..127
 */
void OLED_SetCursor(uint8_t page, uint8_t col)
{
    OLED_WriteCmd(0xB0 | (page & 0x07));        /* page address 0xB0..0xB7 */
    OLED_WriteCmd(0x00 | (col & 0x0F));          /* column low nibble */
    OLED_WriteCmd(0x10 | ((col >> 4) & 0x0F));   /* column high nibble */
}

/**
 * @brief  Initialize SSD1306: clock, multiplex, segment remap, charge
 *         pump, display on. Sequence per datasheet.
 */
void OLED_Init(void)
{
    /* ~300 ms power-on delay: VDD must be stable before I2C commands */
    {
        volatile uint32_t i;
        for (i = 0; i < 800000; i++) __NOP();
    }

    OLED_WriteCmd(0xAE);    /* display off during configuration */

    OLED_WriteCmd(0x20);    /* addressing mode */
    OLED_WriteCmd(0x02);    /*   page addressing */

    OLED_WriteCmd(0xA1);    /* segment remap (column 127 -> SEG0) */
    OLED_WriteCmd(0xC8);    /* COM scan direction reversed */
                            /* => (0,0) at top-left */

    OLED_WriteCmd(0x40);    /* display start line 0 */

    OLED_WriteCmd(0x81);    /* contrast */
    OLED_WriteCmd(0xCF);    /*   value 0xCF */

    OLED_WriteCmd(0xA6);    /* normal display mode */

    OLED_WriteCmd(0xA8);    /* multiplex ratio */
    OLED_WriteCmd(0x3F);    /*   64 rows */

    OLED_WriteCmd(0xD3);    /* display vertical offset */
    OLED_WriteCmd(0x00);    /*   none */

    OLED_WriteCmd(0xD5);    /* clock divide / oscillator */
    OLED_WriteCmd(0x80);    /*   default */

    OLED_WriteCmd(0xD9);    /* precharge period */
    OLED_WriteCmd(0xF1);    /*   recommended */

    OLED_WriteCmd(0xDA);    /* COM pins hardware config */
    OLED_WriteCmd(0x12);    /*   128x64 recommended */

    OLED_WriteCmd(0xDB);    /* VCOMH deselect level */
    OLED_WriteCmd(0x40);    /*   ~0.77 x VCC */

    OLED_WriteCmd(0x8D);    /* charge pump */
    OLED_WriteCmd(0x14);    /*   enable (required at 3.3 V) */

    OLED_WriteCmd(0xA4);    /* resume to RAM content */

    OLED_WriteCmd(0xAF);    /* display on */
}

/** @brief Clear the whole GDDRAM (all pixels off). */
void OLED_Clear(void)
{
    uint8_t page, col;

    for (page = 0; page < OLED_PAGES; page++)
    {
        OLED_SetCursor(page, 0);

        I2C_Start();
        I2C_SendByte(OLED_ADDR_WRITE);
        I2C_WaitAck();
        I2C_SendByte(OLED_CTRL_DATA);
        I2C_WaitAck();
        for (col = 0; col < OLED_WIDTH; col++)
        {
            I2C_SendByte(0x00);
            I2C_WaitAck();
        }
        I2C_Stop();
    }

    OLED_SetCursor(0, 0);
}

/** @brief Fill the whole GDDRAM with 0xFF (all pixels on; test pattern). */
void OLED_Fill(void)
{
    uint8_t page, col;

    for (page = 0; page < OLED_PAGES; page++)
    {
        OLED_SetCursor(page, 0);

        I2C_Start();
        I2C_SendByte(OLED_ADDR_WRITE);
        I2C_WaitAck();
        I2C_SendByte(OLED_CTRL_DATA);
        I2C_WaitAck();
        for (col = 0; col < OLED_WIDTH; col++)
        {
            I2C_SendByte(0xFF);
            I2C_WaitAck();
        }
        I2C_Stop();
    }

    OLED_SetCursor(0, 0);
}

/**
 * @brief  Display one ASCII character.
 * @param  x: column 0..127
 * @param  y: page 0..7 (8 pixels high)
 * @param  ch: character to display; chars outside 0x20..0x7E shown as space
 */
void OLED_ShowChar(uint8_t x, uint8_t y, char ch)
{
    if (ch < ' ' || ch > '~')
        ch = ' ';

    OLED_SetCursor(y, x);

    /* 6 bytes per glyph; font index = (ch - 0x20) */
    OLED_WriteMultiData(Font6x8[ch - 0x20], 6);
}

/**
 * @brief  Display a NUL-terminated string with automatic line wrap.
 * @param  x: start column
 * @param  y: start page
 * @param  str: string to display
 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str)
{
    uint8_t col = x;

    while (*str != '\0')
    {
        if (col > OLED_WIDTH - 6)   /* wrap: glyph is 6 columns wide */
        {
            col = 0;
            y++;
        }

        if (y >= OLED_PAGES)        /* past bottom of screen */
            break;

        OLED_ShowChar(col, y, *str);

        col += 6;
        str++;
    }
}

/**
 * @brief  Display a signed integer, right-aligned to len digits
 *         (leading spaces; minus sign included in the width).
 * @param  x:   start column
 * @param  y:   page
 * @param  num: value to display
 * @param  len: total digit width
 */
void OLED_ShowNum(uint8_t x, uint8_t y, int32_t num, uint8_t len)
{
    char buf[12];   /* up to 11 digits + sign, plus NUL */
    uint8_t i;

    if (num < 0)
    {
        buf[len] = '\0';
        num = -num;

        for (i = len - 1; ; i--)
        {
            buf[i] = '0' + (num % 10);
            num /= 10;

            if (num == 0)
                break;
            if (i == 0) break;
        }

        while (i > 1) { buf[--i] = ' '; }

        buf[0] = '-';
    }
    else
    {
        buf[len] = '\0';
        for (i = len - 1; ; i--)
        {
            buf[i] = '0' + (num % 10);
            num /= 10;
            if (num == 0)
                break;
            if (i == 0) break;
        }
        while (i > 0) { buf[--i] = ' '; }
    }

    OLED_ShowString(x, y, buf);
}

/**
 * @brief  Display a float with fixed integer and decimal widths.
 * @param  x:      start column
 * @param  y:      page
 * @param  num:    value to display
 * @param  intLen: integer digit width
 * @param  decLen: decimal digit count
 */
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t intLen, uint8_t decLen)
{
    uint32_t intPart, decPart;
    uint8_t negative = 0;

    if (num < 0)
    {
        negative = 1;
        num = -num;
    }

    intPart = (uint32_t)num;
    float frac = num - (float)intPart;

    /* scale fraction: 0.14, decLen=2 -> 14 */
    {
        uint8_t i;
        for (i = 0; i < decLen; i++)
            frac *= 10;
    }
    decPart = (uint32_t)(frac + 0.5f);                    /* round */

    /* build string: [sign][pad..int][.][dec] */
    uint8_t buf[12];
    uint8_t pos = 0;

    /* --- integer part (right-aligned, leading spaces) --- */
    {
        uint8_t intBuf[6];
        uint8_t intCount = 0;

        if (intPart == 0)
        {
            intBuf[intCount++] = 0;
        }
        else
        {
            while (intPart > 0)
            {
                intBuf[intCount++] = intPart % 10;
                intPart /= 10;
            }
        }

        while (intCount < intLen)
            buf[pos++] = ' ';

        if (negative && pos > 0)
        {
            buf[pos - 1] = '-';
        }
        else if (negative)
        {
            buf[pos++] = '-';
        }

        /* digits were stored LSB first: write back-to-front */
        {
            int8_t j;
            for (j = intCount - 1; j >= 0; j--)
                buf[pos++] = '0' + intBuf[j];
        }
    }

    buf[pos++] = '.';

    /* --- fractional part (zero-padded) --- */
    {
        uint8_t decBuf[6];
        uint8_t decCount = 0;

        if (decPart == 0)
        {
            decBuf[decCount++] = 0;
        }
        else
        {
            while (decPart > 0)
            {
                decBuf[decCount++] = decPart % 10;
                decPart /= 10;
            }
        }

        while (decCount < decLen)
            decBuf[decCount++] = 0;

        /* write back-to-front */
        {
            int8_t j;
            for (j = decCount - 1; j >= 0; j--)
                buf[pos++] = '0' + decBuf[j];
        }
    }

    buf[pos] = '\0';

    OLED_ShowString(x, y, (const char *)buf);
}
