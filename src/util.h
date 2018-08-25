#ifndef _VVC_UTIL_H
#define _VVC_UTIL_H

#include "global.h"

// General utility methods.
void setup_clocks(void);

// I2C control functions.
// TODO: Make a cpp class for this and the display.
void i2c_initialize(I2C_TypeDef *I2Cx,
                    uint32_t timing_value);
void i2c_set_addr(I2C_TypeDef *I2Cx,
                         uint8_t addr);
void i2c_start(I2C_TypeDef *I2Cx);
void i2c_stop(I2C_TypeDef *I2Cx);
void i2c_set_num_bytes(I2C_TypeDef *I2Cx,
                              uint8_t nbytes);
void i2c_write_byte(I2C_TypeDef *I2Cx,
                           uint8_t dat);
uint8_t i2c_read_byte(I2C_TypeDef *I2Cx);
void i2c_write_command(I2C_TypeDef *I2Cx,
                       uint8_t cmd);
void i2c_write_data_byte(I2C_TypeDef *I2Cx,
                         uint8_t dat);
void i2c_stream_bytes(I2C_TypeDef *I2Cx,
                      volatile uint8_t* buf, int len);

// SSD1306 display methods.
void ssd1306_start_sequence(I2C_TypeDef *I2Cx);

// Methods for writing to the 1KB OLED framebuffer.
// These don't actually write through to the screen.
void oled_draw_h_line(int x, int y, int w, unsigned char color);
void oled_draw_v_line(int x, int y, int h, unsigned char color);
void oled_draw_rect(int x, int y, int w, int h,
                    int outline, unsigned char color);
void oled_write_pixel(int x, int y, unsigned char color);
void oled_draw_letter(int x, int y, uint32_t w0, uint32_t w1, unsigned char color, char size);
void oled_draw_letter_c(int x, int y, char c, unsigned char color, char size);
void oled_draw_letter_i(int x, int y, int ic, unsigned char color, char size);
void oled_draw_text(int x, int y, char* cc, unsigned char color, char size);

#endif
