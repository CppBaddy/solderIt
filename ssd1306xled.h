/*
 * SSD1306xLED - Drivers for SSD1306 controlled dot matrix OLED/PLED 128x64 displays
 *
 * @file: ssd1306xled.h
 * @created: 2014-08-12
 * @author: Neven Boyanov
 *
 * Source code available at: https://bitbucket.org/tinusaur/ssd1306xled
 *
 */

#ifndef SSD1306XLED_H
#define SSD1306XLED_H

// ----------------------------------------------------------------------------

// ---------------------	// Vcc,	Pin 1 on SSD1306 Board
// ---------------------	// GND,	Pin 2 on SSD1306 Board
#ifndef SSD1306_SCL
#define SSD1306_SCL		PB2	// SCL,	Pin 3 on SSD1306 Board
#endif
#ifndef SSD1306_SDA
#define SSD1306_SDA		PB0	// SDA,	Pin 4 on SSD1306 Board
#endif
#ifndef SSD1306_SA
#define SSD1306_SA		0x78	// Slave address
#endif

// ----------------------------------------------------------------------------

void ssd1306_init(void);
void ssd1306_xfer_start(void);
void ssd1306_xfer_stop(void);
void ssd1306_send_byte(uint8_t byte);
void ssd1306_send_command(uint8_t command);
void ssd1306_send_data_start(void);
void ssd1306_send_data_stop(void);
void ssd1306_setpos(uint8_t x, uint8_t y);
void ssd1306_fillscreen(uint8_t fill_Data);
void ssd1306_char_f6x8(uint8_t x, uint8_t y, const char* p);
void ssd1306_char_f8x16(uint8_t x, uint8_t y,const char* p);
void ssd1306_char_f16x16(uint8_t x, uint8_t y, uint8_t N);
void ssd1306_draw_bmp(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t bitmap[]);

void ssd1306_char_f16x32(uint8_t x, uint8_t y,const char* p);

void ssd1306_bar(uint8_t x, uint8_t y, uint8_t v);
void ssd1306_dbar(uint8_t x, uint8_t y, uint8_t v);

// ----------------------------------------------------------------------------

#endif
