#include "util.h"

/*
 * Setup the core system clock.
 */
void setup_clocks(void) {
  #ifdef VVC_F1
    // Set 2 wait states in flash and enable the prefetch buffer.
    FLASH->ACR &= ~(FLASH_ACR_LATENCY);
    FLASH->ACR |=  (FLASH_ACR_LATENCY_2 |
                    FLASH_ACR_PRFTBE);
    // Enable the HSE oscillator.
    // (This will be an infinite loop if your board doesn't have
    //  an HSE oscillator, but most cheap F103C8 boards seem to have one.)
    RCC->CR    |=  (RCC_CR_HSEON);
    while (!(RCC->CR & RCC_CR_HSERDY)) {};
    // Set the HSE oscillator as the system clock source.
    RCC->CFGR  &= ~(RCC_CFGR_SW);
    RCC->CFGR  |=  (RCC_CFGR_SW_HSE);
    // Set the PLL multiplication factor to 9, for 8*9=72MHz.
    RCC->CFGR  &= ~(RCC_CFGR_PLLMULL);
    RCC->CFGR  |=  (RCC_CFGR_PLLMULL9);
    // Enable the PLL.
    RCC->CR    |=  (RCC_CR_PLLON);
    while (!(RCC->CR & RCC_CR_PLLRDY)) {};
    // Set the PLL as the system clock source.
    RCC->CFGR  &= ~(RCC_CFGR_SW);
    RCC->CFGR  |=  (RCC_CFGR_SW_PLL);
    // The core clock is now 72MHz.
    core_clock_hz = 72000000;
  #elif VVC_F3
    // TODO.
    core_clock_hz = 8000000;
  #endif
}

/*
 * Initialize the I2C peripheral with a given timing value.
 */
void i2c_initialize(I2C_TypeDef *I2Cx,
                    uint32_t timing_value) {
  #ifdef VVC_F1
    // Perform a software reset.
    I2Cx->CR1     |=  (I2C_CR1_SWRST);
    I2Cx->CR1     &= ~(I2C_CR1_SWRST);
    // Disable the peripheral.
    I2Cx->CR1     &= ~(I2C_CR1_PE);
    // The F1 series uses a different I2C peripheral than
    // most other STM32s, so the timing parameter doesn't apply.
    // But these don't have to be precise to work, so
    // set the 'frequency' bits to...I dunno, 36MHz?
    I2Cx->CR2     &= ~(I2C_CR2_FREQ);
    I2Cx->CR2     |=  (36 << I2C_CR2_FREQ_Pos);
    // Set the CCR bits to...uh...TODO: calculte an actual value.
    I2Cx->CCR     &= ~(I2C_CCR_CCR);
    I2Cx->CCR     |=  (I2C_CCR_FS | 0x24);
    // Enable the peripheral.
    I2Cx->CR1     |=  (I2C_CR1_PE);
  #elif  VVC_F3
    // First, disable the peripheral.
    I2Cx->CR1     &= ~(I2C_CR1_PE);
    // Clear some 'CR1' bits.
    I2Cx->CR1     &= ~( I2C_CR1_DNF    |
                        I2C_CR1_ANFOFF |
                        I2C_CR1_SMBHEN |
                        I2C_CR1_SMBDEN );
    // Clear some 'CR2' bits.
    I2Cx->CR2     &= ~( I2C_CR2_RD_WRN  |
                        I2C_CR2_NACK    |
                        I2C_CR2_RELOAD  |
                        I2C_CR2_AUTOEND );
    // Clear all 'ICR' flags.
    I2Cx->ICR     |=  ( I2C_ICR_ADDRCF   |
                        I2C_ICR_NACKCF   |
                        I2C_ICR_STOPCF   |
                        I2C_ICR_BERRCF   |
                        I2C_ICR_ARLOCF   |
                        I2C_ICR_OVRCF    |
                        I2C_ICR_PECCF    |
                        I2C_ICR_TIMOUTCF |
                        I2C_ICR_ALERTCF  );
    // Configure I2C timing.
    // Reset all but the reserved bits.
    I2Cx->TIMINGR &=  (0x0F000000);
    // set the given timing value.
    I2Cx->TIMINGR |=  (timing_value);
    // Enable the peripheral.
    I2Cx->CR1     |=  I2C_CR1_PE;
  #endif
}

/*
 * Set the I2C address of the device to communicate with.
 */
void i2c_set_addr(I2C_TypeDef *I2Cx,
                         uint8_t addr) {
  #ifdef VVC_F1
    // Generate a start condition to set the chip as a host.
    i2c_start(I2Cx);
    // Wait for the peripheral to update its role.
    while (!(I2Cx->SR2 & I2C_SR2_MSL)) {};
    // Set the device address; 7-bits followed by an R/W bit.
    // Currently, we only ever write so just use 0 for R/W.
    I2Cx->DR   =  (addr);
    // Wait for address to match.
    while (!(I2Cx->SR1 & I2C_SR1_ADDR)) {};
    // Read SR2 to clear the ADDR flag.
    (void) I2Cx->SR2;
    // (Ready to send data.)
  #elif  VVC_F3
    // Set the device address.
    I2Cx->CR2 &= ~(I2C_CR2_SADD);
    I2Cx->CR2 |=  (addr << I2C_CR2_SADD_Pos);
    // (Start condition still needed before transmitting.)
  #endif
}

/*
 * Send a 'start' condition on the I2C bus.
 */
void i2c_start(I2C_TypeDef *I2Cx) {
  // Send 'Start' condition, and wait for acknowledge.
  #ifdef VVC_F1
    I2Cx->CR1 |=  (I2C_CR1_START);
    while (!(I2Cx->SR1 & I2C_SR1_SB)) {};
  #elif  VVC_F3
    I2Cx->CR2 |=  (I2C_CR2_START);
    while ((I2Cx->CR2 & I2C_CR2_START)) {};
  #endif
}

/*
 * Send a 'stop' condition on the I2C bus.
 */
void i2c_stop(I2C_TypeDef *I2Cx) {
  #ifdef VVC_F1
    // Send 'Stop' condition, and wait for acknowledge.
    I2Cx->CR1 |=  (I2C_CR1_STOP);
    while (I2Cx->SR2 & I2C_SR2_MSL) {};
  #elif  VVC_F3
    // Send 'Stop' condition, and wait for acknowledge.
    I2Cx->CR2 |=  (I2C_CR2_STOP);
    while ((I2Cx->CR2 & I2C_CR2_STOP)) {}
    // Reset the ICR ('Interrupt Clear Register') event flag.
    I2Cx->ICR |=  (I2C_ICR_STOPCF);
    while ((I2Cx->ICR & I2C_ICR_STOPCF)) {}
  #endif
}

/*
 * Set the number of bytes to send/receive over the I2C bus.
 */
void i2c_set_num_bytes(I2C_TypeDef *I2Cx,
                              uint8_t nbytes) {
  #ifdef VVC_F1
    // (Not applicable)
  #elif  VVC_F3
    // Set number of bytes to process in the next transmission.
    I2Cx->CR2 &= ~(I2C_CR2_NBYTES);
    I2Cx->CR2 |=  (nbytes << I2C_CR2_NBYTES_Pos);
  #endif
}

/*
 * Write a byte of data on the I2C bus.
 */
void i2c_write_byte(I2C_TypeDef *I2Cx,
                    uint8_t dat) {
  #ifdef VVC_F1
    // Transmit a byte of data, and wait for it to send.
    I2Cx->DR   = (I2Cx->DR & 0xFF00) |  dat;
    while (!(I2Cx->SR1 & I2C_SR1_TXE)) {};
  #elif  VVC_F3
    // Transmit a byte of data, and wait for it to send.
    I2Cx->TXDR = (I2Cx->TXDR & 0xFFFFFF00) | dat;
    while (!(I2Cx->ISR & (I2C_ISR_TXIS |
                          I2C_ISR_TC |
                          I2C_ISR_TCR))) {};
  #endif
}

/*
 * Read a byte of data on the I2C bus.
 */
uint8_t i2c_read_byte(I2C_TypeDef *I2Cx) {
  #ifdef VVC_F1
    // TODO
    return 0x00;
  #elif  VVC_F3
    // Wait for a byte of data to be available, then read it.
    while (!(I2Cx->ISR & I2C_ISR_RXNE)) {}
    return (I2Cx->RXDR & 0xFF);
  #endif
}

/*
 * Write a single command byte over I2C.
 */
void i2c_write_command(I2C_TypeDef *I2Cx,
                       uint8_t cmd) {
  // On the I2C bus, the first byte of the transmission
  // indicates D/C; 0x00 for 'Command', 0x40 for 'Data'.
  #ifdef VVC_F1
    i2c_set_addr(I2Cx, 0x78);
    i2c_write_byte(I2Cx, 0x00);
    i2c_write_byte(I2Cx, cmd);
    i2c_stop(I2Cx);
  #elif  VVC_F3
    i2c_set_num_bytes(I2Cx, 2);
    i2c_start(I2Cx);
    i2c_write_byte(I2Cx, 0x00);
    i2c_write_byte(I2Cx, cmd);
    i2c_stop(I2Cx);
  #endif
}

/*
 * Write a single data byte over I2C.
 */
void i2c_write_data_byte(I2C_TypeDef *I2Cx,
                         uint8_t dat) {
  // With the SSD1306, the first byte of the transmission
  // indicates D/C; 0x00 for 'Command', 0x40 for 'Data'.
  #ifdef VVC_F1
    i2c_set_addr(I2Cx, 0x78);
    i2c_write_byte(I2Cx, 0x40);
    i2c_write_byte(I2Cx, dat);
    i2c_stop(I2Cx);
  #elif  VVC_F3
    i2c_set_num_bytes(I2Cx, 2);
    i2c_start(I2Cx);
    i2c_write_byte(I2Cx, 0x40);
    i2c_write_byte(I2Cx, dat);
    i2c_stop(I2Cx);
  #endif
}

/* Stream the given number of bytes in one transmission. */
void i2c_stream_bytes(I2C_TypeDef *I2Cx,
                      volatile uint8_t* buf, int len) {
#ifdef VVC_F1
  // The F1 series have a simpler 'transmit' process
  // for longer frames; you just send all of the bytes.
  int nb_i = 0;
  i2c_set_addr(I2Cx, 0x78);
  i2c_write_byte(I2Cx, 0x40);
  for (nb_i = 0; nb_i < len; ++nb_i) {
    i2c_write_byte(I2Cx, buf[nb_i]);
  }
  i2c_stop(I2Cx);
#elif  VVC_F3
  // The more recent chips have an internal counter
  // to keep track of how many bytes they send/receive,
  // so we need to manage that.
  volatile int pos = 0;
  int nb_i = 0;
  // Set the 'RELOAD' flag
  I2Cx->CR2 |= I2C_CR2_RELOAD;
  // (We also need one byte to say this is a data transmission)
  i2c_set_num_bytes(I2Cx, 1);
  i2c_start(I2Cx);
  i2c_write_byte(I2Cx, 0x40);
  // The 'NBYTES' setting is only 8 bits wide, so
  // transmit 255 bytes at a time.
  while ((len - pos) > 255) {
    i2c_set_num_bytes(I2Cx, 255);
    for (nb_i = 0; nb_i < 255; ++nb_i) {
      i2c_write_byte(I2Cx, buf[pos]);
      pos += 1;
    }
  }
  // Stream the remaining bytes.
  if (pos < len) {
    int remaining_bytes = len - pos;
    i2c_set_num_bytes(I2Cx, remaining_bytes);
    for (nb_i = 0; nb_i < remaining_bytes; ++nb_i) {
      i2c_write_byte(I2Cx, buf[pos]);
      ++pos;
    }
  }
  // Send 'stop' condition and un-set the 'RELOAD' flag.
  i2c_stop(I2Cx);
  I2Cx->CR2 |= I2C_CR2_RELOAD;
#endif
}

/*
 * Send a series of startup commands over I2C for an
 * SSD1306 OLED display.
 * TODO: Support resolutions other than 128x64
 * TODO: Make macro definitions for these command values.
 */
void ssd1306_start_sequence(I2C_TypeDef *I2Cx) {
  // Display clock division
  i2c_write_command(I2Cx, 0xD5);
  i2c_write_command(I2Cx, 0x80);
  // Set multiplex
  i2c_write_command(I2Cx, 0xA8);
  i2c_write_command(I2Cx, 0x3F);
  // Set display offset ('start column')
  i2c_write_command(I2Cx, 0xD3);
  i2c_write_command(I2Cx, 0x00);
  // Set start line (0b01000000 | line)
  i2c_write_command(I2Cx, 0x40);
  // Set internal charge pump (on)
  i2c_write_command(I2Cx, 0x8D);
  i2c_write_command(I2Cx, 0x14);
  // Set memory mode
  i2c_write_command(I2Cx, 0x20);
  i2c_write_command(I2Cx, 0x00);
  // Set 'SEGREMAP'
  i2c_write_command(I2Cx, 0xA1);
  // Set column scan (descending)
  i2c_write_command(I2Cx, 0xC8);
  // Set 'COMPINS'
  i2c_write_command(I2Cx, 0xDA);
  i2c_write_command(I2Cx, 0x12);
  // Set contrast
  i2c_write_command(I2Cx, 0x81);
  i2c_write_command(I2Cx, 0xCF);
  // Set precharge
  i2c_write_command(I2Cx, 0xD9);
  i2c_write_command(I2Cx, 0xF1);
  // Set VCOM detect
  i2c_write_command(I2Cx, 0xDB);
  i2c_write_command(I2Cx, 0x40);
  // Set output to follow RAM content
  i2c_write_command(I2Cx, 0xA4);
  // Normal display mode
  i2c_write_command(I2Cx, 0xA6);
  // Display on
  i2c_write_command(I2Cx, 0xAF);
}

/*
 * Draw a horizontal line.
 * First, calculate the Y bitmask and byte offset, then just go from x->x.
 */
void oled_draw_h_line(int x, int y,
                      int w, unsigned char color) {
  int y_page_offset  = y / 8;
  y_page_offset     *= 128;
  int bit_to_set     = 0x01 << (y & 0x07);
  if (!color) {
    bit_to_set = ~bit_to_set;
  }
  int x_pos;
  for (x_pos = x; x_pos < (x+w); ++x_pos) {
    if (color) {
      oled_fb[x_pos + y_page_offset] |= bit_to_set;
    }
    else {
      oled_fb[x_pos + y_page_offset] &= bit_to_set;
    }
  }
}

/*
 * Draw a veritcal line.
 */
void oled_draw_v_line(int x, int y,
                             int h, unsigned char color) {
  int y_page_offset;
  int bit_to_set;
  int y_pos;
  for (y_pos = y; y_pos < (y+h); ++y_pos) {
    y_page_offset = y_pos/8;
    y_page_offset *= 128;
    bit_to_set = 0x01 << (y_pos & 0x07);
    if (color) {
      oled_fb[x + y_page_offset] |= bit_to_set;
    }
    else {
      bit_to_set = ~bit_to_set;
      oled_fb[x + y_page_offset] &= bit_to_set;
    }
  }
}

/*
 * Draw a rectangle on the display.
 * I guess just pick the longer dimension, and either draw
 * horizontal or vertical lines.
 * Notable args:
 *   - outline: If <=0, fill the rectangle with 'color'.
 *     If >0, draw an outline inside the dimensions of N pixels.
 *   - color: If 0, clear drawn bits. If not 0, set drawn bits.
 */
void oled_draw_rect(int x, int y, int w, int h,
                    int outline, unsigned char color) {
  if (outline > 0) {
    // Draw an outline.
    int o_pos;
    // Top.
    for (o_pos = y; o_pos < (y+outline); ++o_pos) {
      oled_draw_h_line(x, o_pos, w, color);
    }
    // Bottom.
    for (o_pos = (y+h-1); o_pos > (y+h-1-outline); --o_pos) {
      oled_draw_h_line(x, o_pos, w, color);
    }
    // Left.
    for (o_pos = x; o_pos < (x+outline); ++o_pos) {
      oled_draw_v_line(o_pos, y, h, color);
    }
    // Right.
    for (o_pos = (x+w-1); o_pos > (x+w-1-outline); --o_pos) {
      oled_draw_v_line(o_pos, y, h, color);
    }
  }
  else {
    // Draw a filled rectangle.
    if (w > h) {
      // Draw fewer horizontal lines than vertical ones.
      int y_pos;
      for (y_pos = y; y_pos < (y+h); ++y_pos) {
        oled_draw_h_line(x, y_pos, w, color);
      }
    }
    else {
      // Draw fewer (or ==) vertical lines than horizontal ones.
      int x_pos;
      for (x_pos = x; x_pos < (x+w); ++x_pos) {
        oled_draw_v_line(x_pos, y, h, color);
      }
    }
  }
}

/*
 * Write a pixel in the current OLED framebuffer.
 * Note that the positioning is a bit odd; each byte is a VERTICAL column
 * of 8 pixels, but each successive byte increments the row position by 1.
 * This means that the buffer is 8x 128-byte pages stacked on top of one
 * another. To set an (x, y) pixel, we |= one position in one byte.
 *   Byte offset = x + ((y / 8) * 128)
 *   Bit offset  = (y & 0x07)
 * 'color' indicates whether to set or unset the pixel. 0 means 'unset.'
 */
void oled_write_pixel(int x, int y, unsigned char color) {
  int y_page = y / 8;
  int byte_to_mod = x + (y_page * 128);
  int bit_to_set = 0x01 << (y & 0x07);
  if (color) {
    oled_fb[byte_to_mod] |= bit_to_set;
  }
  else {
    bit_to_set = ~bit_to_set;
    oled_fb[byte_to_mod] &= bit_to_set;
  }
}

/*
 * Draw a letter to the framebuffer, using the 'font' defined
 * in 'src/global.h'. That comes in the form of 48 bits,
 * so a different function is used to draw a character.
 */
void oled_draw_letter(int x, int y,
                      uint32_t w0, uint32_t w1,
                      unsigned char color, char size) {
  // TODO: Make this more efficient than drawing
  // pixels one-by-one.
  int w_iter = 0;
  int cur_x = x;
  int cur_y = y;
  uint32_t aw0 = w0;
  uint32_t aw1 = w1;
  if (!color) {
    aw0 = ~aw0;
    aw1 = ~aw1;
  }
  int px_incr = 1;
  int line_h = 8;
  unsigned char t_col = 0x00;
  int cx = cur_x;
  int cy = cur_y;
  if (size == 'L') {
    px_incr = 2;
    line_h = 16;
  }
  for (w_iter = 31; w_iter >= 0; --w_iter) {
    t_col = !(!(aw0 & (1 << w_iter)));
    for (cx = cur_x; cx < cur_x + px_incr; ++cx) {
      for (cy = cur_y; cy < cur_y + px_incr; ++cy) {
        oled_write_pixel(cx, cy, t_col);
      }
    }
    cur_y += px_incr;
    if (cur_y == y+line_h) {
      cur_y = y;
      cur_x += px_incr;
    }
  }
  for (w_iter = 15; w_iter >= 0; --w_iter) {
    t_col = !(!(aw1 & (1 << w_iter)));
    for (cx = cur_x; cx < cur_x + px_incr; ++cx) {
      for (cy = cur_y; cy < cur_y + px_incr; ++cy) {
        oled_write_pixel(cx, cy, t_col);
      }
    }
    cur_y += px_incr;
    if (cur_y == y+line_h) {
      cur_y = y;
      cur_x += px_incr;
    }
  }
}

/*
 * Draw a single character to the OLED display.
 * This method translates between a character and the
 * font data defined in 'src/global.h'
 */
void oled_draw_letter_c(int x, int y, char c, unsigned char color, char size) {
  unsigned int w0 = 0x00;
  unsigned int w1 = 0x00;
  if (c == 'A') {
    w0 = OLED_CH_A0;
    w1 = OLED_CH_A1B1 >> 16;
  }
  else if (c == 'B') {
    w0 = OLED_CH_B0;
    w1 = OLED_CH_A1B1 & 0x0000FFFF;
  }
  else if (c == 'C') {
    w0 = OLED_CH_C0;
    w1 = OLED_CH_C1D1 >> 16;
  }
  else if (c == 'D') {
    w0 = OLED_CH_D0;
    w1 = OLED_CH_C1D1 & 0x0000FFFF;
  }
  else if (c == 'E') {
    w0 = OLED_CH_E0;
    w1 = OLED_CH_E1F1 >> 16;
  }
  else if (c == 'F') {
    w0 = OLED_CH_F0;
    w1 = OLED_CH_E1F1 & 0x0000FFFF;
  }
  else if (c == 'G') {
    w0 = OLED_CH_G0;
    w1 = OLED_CH_G1H1 >> 16;
  }
  else if (c == 'H') {
    w0 = OLED_CH_H0;
    w1 = OLED_CH_G1H1 & 0x0000FFFF;
  }
  else if (c == 'I') {
    w0 = OLED_CH_I0;
    w1 = OLED_CH_I1J1 >> 16;
  }
  else if (c == 'J') {
    w0 = OLED_CH_J0;
    w1 = OLED_CH_I1J1 & 0x0000FFFF;
  }
  else if (c == 'K') {
    w0 = OLED_CH_K0;
    w1 = OLED_CH_K1L1 >> 16;
  }
  else if (c == 'L') {
    w0 = OLED_CH_L0;
    w1 = OLED_CH_K1L1 & 0x0000FFFF;
  }
  else if (c == 'M') {
    w0 = OLED_CH_M0;
    w1 = OLED_CH_M1N1 >> 16;
  }
  else if (c == 'N') {
    w0 = OLED_CH_N0;
    w1 = OLED_CH_M1N1 & 0x0000FFFF;
  }
  else if (c == 'O') {
    w0 = OLED_CH_O0;
    w1 = OLED_CH_O1P1 >> 16;
  }
  else if (c == 'P') {
    w0 = OLED_CH_P0;
    w1 = OLED_CH_O1P1 & 0x0000FFFF;
  }
  else if (c == 'Q') {
    w0 = OLED_CH_Q0;
    w1 = OLED_CH_Q1R1 >> 16;
  }
  else if (c == 'R') {
    w0 = OLED_CH_R0;
    w1 = OLED_CH_Q1R1 & 0x0000FFFF;
  }
  else if (c == 'S') {
    w0 = OLED_CH_S0;
    w1 = OLED_CH_S1T1 >> 16;
  }
  else if (c == 'T') {
    w0 = OLED_CH_T0;
    w1 = OLED_CH_S1T1 & 0x0000FFFF;
  }
  else if (c == 'U') {
    w0 = OLED_CH_U0;
    w1 = OLED_CH_U1V1 >> 16;
  }
  else if (c == 'V') {
    w0 = OLED_CH_V0;
    w1 = OLED_CH_U1V1 & 0x0000FFFF;
  }
  else if (c == 'W') {
    w0 = OLED_CH_W0;
    w1 = OLED_CH_W1X1 >> 16;
  }
  else if (c == 'X') {
    w0 = OLED_CH_X0;
    w1 = OLED_CH_W1X1 & 0x0000FFFF;
  }
  else if (c == 'Y') {
    w0 = OLED_CH_Y0;
    w1 = OLED_CH_Y1Z1 >> 16;
  }
  else if (c == 'Z') {
    w0 = OLED_CH_Z0;
    w1 = OLED_CH_Y1Z1 & 0x0000FFFF;
  }
  else if (c == 'a') {
    w0 = OLED_CH_a0;
    w1 = OLED_CH_a1b1 >> 16;
  }
  else if (c == 'b') {
    w0 = OLED_CH_b0;
    w1 = OLED_CH_a1b1 & 0x0000FFFF;
  }
  else if (c == 'c') {
    w0 = OLED_CH_c0;
    w1 = OLED_CH_c1d1 >> 16;
  }
  else if (c == 'd') {
    w0 = OLED_CH_d0;
    w1 = OLED_CH_c1d1 & 0x0000FFFF;
  }
  else if (c == 'e') {
    w0 = OLED_CH_e0;
    w1 = OLED_CH_e1f1 >> 16;
  }
  else if (c == 'f') {
    w0 = OLED_CH_f0;
    w1 = OLED_CH_e1f1 & 0x0000FFFF;
  }
  else if (c == 'g') {
    w0 = OLED_CH_g0;
    w1 = OLED_CH_g1h1 >> 16;
  }
  else if (c == 'h') {
    w0 = OLED_CH_h0;
    w1 = OLED_CH_g1h1 & 0x0000FFFF;
  }
  else if (c == 'i') {
    w0 = OLED_CH_i0;
    w1 = OLED_CH_i1j1 >> 16;
  }
  else if (c == 'j') {
    w0 = OLED_CH_j0;
    w1 = OLED_CH_i1j1 & 0x0000FFFF;
  }
  else if (c == 'k') {
    w0 = OLED_CH_k0;
    w1 = OLED_CH_k1l1 >> 16;
  }
  else if (c == 'l') {
    w0 = OLED_CH_l0;
    w1 = OLED_CH_k1l1 & 0x0000FFFF;
  }
  else if (c == 'm') {
    w0 = OLED_CH_m0;
    w1 = OLED_CH_m1n1 >> 16;
  }
  else if (c == 'n') {
    w0 = OLED_CH_n0;
    w1 = OLED_CH_m1n1 & 0x0000FFFF;
  }
  else if (c == 'o') {
    w0 = OLED_CH_o0;
    w1 = OLED_CH_o1p1 >> 16;
  }
  else if (c == 'p') {
    w0 = OLED_CH_p0;
    w1 = OLED_CH_o1p1 & 0x0000FFFF;
  }
  else if (c == 'q') {
    w0 = OLED_CH_q0;
    w1 = OLED_CH_q1r1 >> 16;
  }
  else if (c == 'r') {
    w0 = OLED_CH_r0;
    w1 = OLED_CH_q1r1 & 0x0000FFFF;
  }
  else if (c == 's') {
    w0 = OLED_CH_s0;
    w1 = OLED_CH_s1t1 >> 16;
  }
  else if (c == 't') {
    w0 = OLED_CH_t0;
    w1 = OLED_CH_s1t1 & 0x0000FFFF;
  }
  else if (c == 'u') {
    w0 = OLED_CH_u0;
    w1 = OLED_CH_u1v1 >> 16;
  }
  else if (c == 'v') {
    w0 = OLED_CH_v0;
    w1 = OLED_CH_u1v1 & 0x0000FFFF;
  }
  else if (c == 'w') {
    w0 = OLED_CH_w0;
    w1 = OLED_CH_w1x1 >> 16;
  }
  else if (c == 'x') {
    w0 = OLED_CH_x0;
    w1 = OLED_CH_w1x1 & 0x0000FFFF;
  }
  else if (c == 'y') {
    w0 = OLED_CH_y0;
    w1 = OLED_CH_y1z1 >> 16;
  }
  else if (c == 'z') {
    w0 = OLED_CH_z0;
    w1 = OLED_CH_y1z1 & 0x0000FFFF;
  }
  else if (c == '0') {
    w0 = OLED_CH_00;
    w1 = OLED_CH_0111 >> 16;
  }
  else if (c == '1') {
    w0 = OLED_CH_10;
    w1 = OLED_CH_0111 & 0x0000FFFF;
  }
  else if (c == '2') {
    w0 = OLED_CH_20;
    w1 = OLED_CH_2131 >> 16;
  }
  else if (c == '3') {
    w0 = OLED_CH_30;
    w1 = OLED_CH_2131 & 0x0000FFFF;
  }
  else if (c == '4') {
    w0 = OLED_CH_40;
    w1 = OLED_CH_4151 >> 16;
  }
  else if (c == '5') {
    w0 = OLED_CH_50;
    w1 = OLED_CH_4151 & 0x0000FFFF;
  }
  else if (c == '6') {
    w0 = OLED_CH_60;
    w1 = OLED_CH_6171 >> 16;
  }
  else if (c == '7') {
    w0 = OLED_CH_70;
    w1 = OLED_CH_6171 & 0x0000FFFF;
  }
  else if (c == '8') {
    w0 = OLED_CH_80;
    w1 = OLED_CH_8191 >> 16;
  }
  else if (c == '9') {
    w0 = OLED_CH_90;
    w1 = OLED_CH_8191 & 0x0000FFFF;
  }
  else if (c == ':') {
    w0 = OLED_CH_col0;
    w1 = OLED_CH_col1per1 >> 16;
  }
  else if (c == '.') {
    w0 = OLED_CH_per0;
    w1 = OLED_CH_col1per1 & 0x0000FFFF;
  }
  else if (c == '!') {
    w0 = OLED_CH_exc0;
    w1 = OLED_CH_exc1fws1 >> 16;
  }
  else if (c == '/') {
    w0 = OLED_CH_fws0;
    w1 = OLED_CH_exc1fws1 & 0x0000FFFF;
  }
  else if (c == '-') {
    w0 = OLED_CH_hyp0;
    w1 = OLED_CH_hyp1pls1 >> 16;
  }
  else if (c == '+') {
    w0 = OLED_CH_pls0;
    w1 = OLED_CH_hyp1pls1 & 0x0000FFFF;
  }
  else if (c == '<') {
    w0 = OLED_CH_lct0;
    w1 = OLED_CH_lct1rct1 >> 16;
  }
  else if (c == '>') {
    w0 = OLED_CH_rct0;
    w1 = OLED_CH_lct1rct1 & 0x0000FFFF;
  }
  oled_draw_letter(x, y, w0, w1, color, size);
}

/*
 * Draw an integer to the display.
 * Just take mod values at decreasing powers of 10 and print
 * the digits one-by-one. Assumes the int is <= 9,999,999,999
 */
void oled_draw_letter_i(int x, int y, int ic,
                        unsigned char color, char size) {
  int magnitude = 1000000000;
  int cur_x = x;
  int first_found = 0;
  int proc_val = ic;
  if (proc_val < 0) {
    proc_val = proc_val * -1;
    oled_draw_letter_c(cur_x, y, '-', color, size);
    if (size == 'S') {
      cur_x += 6;
    }
    else if (size == 'L') {
      cur_x += 12;
    }
  }
  for (magnitude = 1000000000; magnitude > 0; magnitude = magnitude / 10) {
    int m_val = proc_val / magnitude;
    proc_val -= (m_val * magnitude);
    if (m_val > 0 || first_found || magnitude == 1) {
      first_found = 1;
      char mc = ' ';
      if (m_val == 0) {
        mc = '0';
      }
      else if (m_val == 1) {
        mc = '1';
      }
      else if (m_val == 2) {
        mc = '2';
      }
      else if (m_val == 3) {
        mc = '3';
      }
      else if (m_val == 4) {
        mc = '4';
      }
      else if (m_val == 5) {
        mc = '5';
      }
      else if (m_val == 6) {
        mc = '6';
      }
      else if (m_val == 7) {
        mc = '7';
      }
      else if (m_val == 8) {
        mc = '8';
      }
      else if (m_val == 9) {
        mc = '9';
      }
      oled_draw_letter_c(cur_x, y, mc, color, size);
      if (size == 'S') {
        cur_x += 6;
      }
      else if (size == 'L') {
        cur_x += 12;
      }
      if (cur_x >= 128) { return; }
    }
  }
}

/*
 * Draw a string of text.
 * Careful; this assumes the text is a C-string ending in '\0'.
 */
void oled_draw_text(int x, int y, const char* cc,
                    unsigned char color, const char size) {
  int i = 0;
  int offset = 0;
  while (cc[i] != '\0') {
    oled_draw_letter_c(x + offset, y, cc[i], color, size);
    if (size == 'S') {
      offset += 6;
    }
    else if (size == 'L') {
      offset += 12;
    }
    ++i;
  }
}
