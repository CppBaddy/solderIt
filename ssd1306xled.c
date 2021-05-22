/*
 * SSD1306xLED - Drivers for SSD1306 controlled dot matrix OLED/PLED 128x64 displays
 *
 * @file: ssd1306xled.c
 * @created: 2014-08-12
 * @author: Neven Boyanov
 *
 * Source code available at: https://bitbucket.org/tinusaur/ssd1306xled
 *
 */

// ----------------------------------------------------------------------------

#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>

#include <avr/pgmspace.h>

#include "ssd1306xled.h"
#include "font6x8.h"
#include "font8X16.h"
//#include "font16x16cn.h"

// ----------------------------------------------------------------------------

#define DIGITAL_WRITE_HIGH(PORT) (PORTB |= _BV(PORT))
#define DIGITAL_WRITE_LOW(PORT)  (PORTB &= ~(_BV(PORT)))

// ----------------------------------------------------------------------------

// Some code based on "IIC_wtihout_ACK" by http://www.14blog.com/archives/1358

const uint8_t ssd1306_init_sequence [] PROGMEM = {      // Initialization Sequence
        0xAE,                   // Display OFF (sleep mode)
        0x20, 0b00,             // Set Memory Addressing Mode
                                        // 00=Horizontal Addressing Mode; 01=Vertical Addressing Mode;
                                        // 10=Page Addressing Mode (RESET); 11=Invalid
        0xB0,                   // Set Page Start Address for Page Addressing Mode, 0-7
        0xC8,                   // Set COM Output Scan Direction
        0x00,                   // ---set low column address
        0x10,                   // ---set high column address
        0x40,                   // --set start line address
        0x81, 0x7F,             // Set contrast control register
        0xA1,                   // Set Segment Re-map. A0=address mapped; A1=address 127 mapped.
        0xA6,                   // Set display mode. A6=Normal; A7=Inverse
        0xA8, 0x3F,             // Set multiplex ratio(1 to 64)
        0xA4,                   // Output RAM to Display
                                        // 0xA4=Output follows RAM content; 0xA5,Output ignores RAM content
        0xD3, 0x00,             // Set display offset. 00 = no offset
        0xD5,                   // --set display clock divide ratio/oscillator frequency
        0xF0,                   // --set divide ratio
        0xD9, 0x22,             // Set pre-charge period
        0xDA, 0x12,             // Set com pins hardware configuration
        0xDB,                   // --set vcomh
        0x20,                   // 0x20,0.77xVcc
        0x8D, 0x14,             // Set DC-DC enable
        0xAF                    // Display ON in normal mode

};

void ssd1306_init(void)
{
	DDRB  |= _BV(SSD1306_SDA) | _BV(SSD1306_SCL); // Set port as output
	PORTB |= _BV(SSD1306_SDA) | _BV(SSD1306_SCL); //set to high state

	for (uint8_t i = 0; i < sizeof (ssd1306_init_sequence); ++i)
	{
	    ssd1306_send_command(pgm_read_byte(&ssd1306_init_sequence[i]));
	}
}

void ssd1306_xfer_start(void)
{
	DIGITAL_WRITE_HIGH(SSD1306_SCL);	// Set to HIGH
	DIGITAL_WRITE_HIGH(SSD1306_SDA);	// Set to HIGH
	DIGITAL_WRITE_LOW(SSD1306_SDA);		// Set to LOW
	DIGITAL_WRITE_LOW(SSD1306_SCL);		// Set to LOW
}

void ssd1306_xfer_stop(void)
{
	DIGITAL_WRITE_LOW(SSD1306_SCL);		// Set to LOW
	DIGITAL_WRITE_LOW(SSD1306_SDA);		// Set to LOW
	DIGITAL_WRITE_HIGH(SSD1306_SCL);	// Set to HIGH
	DIGITAL_WRITE_HIGH(SSD1306_SDA);	// Set to HIGH
}

void ssd1306_send_byte(uint8_t byte)
{
	for(uint8_t i=0; i<8; ++i)
	{
		((byte << i) & 0x80) ? DIGITAL_WRITE_HIGH(SSD1306_SDA) : DIGITAL_WRITE_LOW(SSD1306_SDA);

		DIGITAL_WRITE_HIGH(SSD1306_SCL);
		DIGITAL_WRITE_LOW(SSD1306_SCL);
	}
	DIGITAL_WRITE_HIGH(SSD1306_SDA);
	DIGITAL_WRITE_HIGH(SSD1306_SCL);
	DIGITAL_WRITE_LOW(SSD1306_SCL);
}

void ssd1306_send_command(uint8_t command)
{
	ssd1306_xfer_start();
	ssd1306_send_byte(SSD1306_SA);  // Slave address, SA0=0
	ssd1306_send_byte(0x00);	// write command
	ssd1306_send_byte(command);
	ssd1306_xfer_stop();
}

void ssd1306_send_data_start(void)
{
	ssd1306_xfer_start();
	ssd1306_send_byte(SSD1306_SA);
	ssd1306_send_byte(0x40);	//write data
}

void ssd1306_send_data_stop(void)
{
	ssd1306_xfer_stop();
}

void ssd1306_setpos(uint8_t x, uint8_t y)
{
	ssd1306_xfer_start();
	ssd1306_send_byte(SSD1306_SA);  //Slave address,SA0=0
	ssd1306_send_byte(0x00);	//write command

	ssd1306_send_byte(0xb0 + y);
	ssd1306_send_byte( ( (x & 0xf0) >> 4) | 0x10); // |0x10
	ssd1306_send_byte( (x & 0x0f) | 0x01); // |0x01

	ssd1306_xfer_stop();
}

void ssd1306_fillscreen(uint8_t fill_Data)
{
	for(uint8_t m=0; m<8; ++m)
	{
		ssd1306_send_command(0xb0 | m);	//page0-page7
		ssd1306_send_command(0x00);		//low column start address
		ssd1306_send_command(0x10);		//high column start address

		ssd1306_send_data_start();

		for(uint8_t n=0; n<128; ++n)
		{
			ssd1306_send_byte(fill_Data);
		}

		ssd1306_send_data_stop();
	}
}

void ssd1306_bar(uint8_t x, uint8_t y, uint8_t v)
{
	ssd1306_setpos(x, y);
	ssd1306_send_data_start();

	v >>= 1;

	uint8_t i=0;
	for(; i<v; ++i)
	{
		ssd1306_send_byte(7);
    }

	for(; i<128; ++i)
	{
		ssd1306_send_byte(0);
    }

	ssd1306_send_data_stop();
}

void ssd1306_dbar(uint8_t x, uint8_t y, uint8_t v)
{
	ssd1306_setpos(x, y);
	ssd1306_send_data_start();

	v >>= 2;

	uint8_t i=0;

	for(; i<(64-v); ++i)
	{
		ssd1306_send_byte(0);
    }

	for(; i<(64+v); ++i)
	{
		ssd1306_send_byte(7);
    }

	for(; i<128; ++i)
	{
		ssd1306_send_byte(0);
    }

	ssd1306_send_data_stop();
}

void ssd1306_char_f6x8(uint8_t x, uint8_t y, const char* p)
{
	for(; *p; ++p)
	{
		if(x > 126)
		{
			x=0;
			++y;
		}

		ssd1306_setpos(x,y);
		ssd1306_send_data_start();

		uint8_t c = *p - 32;

		for(uint8_t i=0; i<6; ++i)
		{
			ssd1306_send_byte( pgm_read_byte(&ssd1306xled_font6x8[c*6 + i]) );
		}
		ssd1306_send_data_stop();
		x += 6;
	}
}

void ssd1306_char_f8x16(uint8_t x, uint8_t y, const char* p)
{
	for(uint8_t i=0; *p; ++p)
	{
		uint8_t c = *p - 32;

		if (x>120)
		{
			x=0;
			y++;
		}

		ssd1306_setpos(x,y);
		ssd1306_send_data_start();

		for(i=0; i<8; ++i)
		{
			ssd1306_send_byte(pgm_read_byte(&ssd1306xled_font8X16[c*16+i]));
		}

		ssd1306_send_data_stop();

		ssd1306_setpos(x, y+1);
		ssd1306_send_data_start();

		for(i=0; i<8; ++i)
		{
			ssd1306_send_byte(pgm_read_byte(&ssd1306xled_font8X16[c*16+i+8]));
		}

		ssd1306_send_data_stop();

		x+=8;
	}
}

void ssd1306_char_f16x32(uint8_t x, uint8_t y,const char* p)
{
	for(uint8_t i=0; *p; ++p)
	{
		uint8_t c = *p - 32;

		if (x>120)
		{
			x=0;
			y++;
		}

		for(uint8_t k=0; k<2; ++k)
		{
            ssd1306_setpos(x, y+k);
            ssd1306_send_data_start();

            for(i=0; i<8; ++i)
            {
                uint8_t b = pgm_read_byte( &ssd1306xled_font8X16[c*16+i] );

                uint8_t h = 0;
                for(uint8_t z=0; z<4; ++z)
                {
                    if(b & (1 << (z+(k*4))))
                    {
                        h |= (3 << (z*2));
                    }
                }

                ssd1306_send_byte(h);
                ssd1306_send_byte(h);
            }

            ssd1306_send_data_stop();
		}

		for(uint8_t k=0; k<2; ++k)
		{
            ssd1306_setpos(x, y+k+2);
            ssd1306_send_data_start();

            for(i=0; i<8; ++i)
            {
                uint8_t b = pgm_read_byte( &ssd1306xled_font8X16[c*16+i+8] );

                uint8_t h = 0;
                for(uint8_t z=0; z<4; ++z)
                {
                    if(b & (1 << (z+(k*4))))
                    {
                        h |= (3 << (z*2));
                    }
                }

                ssd1306_send_byte(h);
                ssd1306_send_byte(h);
            }

            ssd1306_send_data_stop();
		}

		x+=16;
	}
}
/*
void ssd1306_char_f16x16(uint8_t x, uint8_t y, uint8_t N)
{
	uint8_t wm=0;
	unsigned int adder = 32 * N;
	ssd1306_setpos(x , y);
	ssd1306_send_data_start();
	for(wm = 0; wm < 16; wm++)
	{
		//ssd1306_send_byte(pgm_read_byte(&ssd1306xled_font8X16cn[adder]));
		adder += 1;
	}
	ssd1306_send_data_stop();
	ssd1306_setpos(x,y + 1);
	ssd1306_send_data_start();
	for(wm = 0;wm < 16;wm++)
	{
		//ssd1306_send_byte(pgm_read_byte(&ssd1306xled_font8X16cn[adder]));
		adder += 1;
	}
	ssd1306_send_data_stop();
}
*/

void ssd1306_draw_bmp(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t bitmap[])
{
	unsigned int j = 0;
	uint8_t x,y;

	if(y1%8==0)
	y=y1/8;
	else
	y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		ssd1306_setpos(x0,y);
		ssd1306_send_data_start();
		for(x=x0;x<x1;x++)
		{
			ssd1306_send_byte(pgm_read_byte(&bitmap[j++]));
		}
		ssd1306_send_data_stop();
	}
}

// ----------------------------------------------------------------------------
