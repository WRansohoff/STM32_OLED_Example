#include "peripherals.h"

pGPIO::pGPIO() { status = 0; }

pGPIO::pGPIO(GPIO_TypeDef* pin_bank, uint8_t pin_num) {
  bank   = pin_bank;
  pin    = pin_num;
  status = 0;
}

#if    defined(VVC_F3)

pGPIO::pGPIO(GPIO_TypeDef* pin_bank, uint8_t pin_num,
             uint8_t mode, uint8_t type,
             uint8_t speed, uint8_t pupdr) {
  bank           =   pin_bank;
  pin            =   pin_num;
  bank->MODER   &= ~(0x3 << (pin * 2));
  bank->MODER   |=  (mode << (pin * 2));
  bank->OTYPER  &= ~(0x1 << pin);
  bank->OTYPER  |=  (type << pin);
  bank->OSPEEDR &= ~(0x3 << (pin * 2));
  bank->OSPEEDR |=  (speed << (pin * 2));
  bank->PUPDR   &= ~(0x3 << (pin * 2));
  bank->PUPDR   |=  (pupdr << (pin * 2));
  status         =   1;
}

#elif  VVC_F1

pGPIO::pGPIO(GPIO_TypeDef* pin_bank, uint8_t pin_num,
             uint8_t mode, uint8_t cnf) {
  bank   = pin_bank;
  pin    = pin_num;
  if (pin < 8) {
    bank->CRL &= ~(0x0F << (pin * 4));
    bank->CRL |=  (mode << (pin * 4));
    bank->CRL |=  (cnf << ((pin * 4) + 2));
  }
  else {
    bank->CRH &= ~(0x0F << ((pin - 8) * 4));
    bank->CRH |=  (mode << ((pin - 8) * 4));
    bank->CRH |=  (cnf << (((pin - 8) * 4) + 2));
  }
  status = 1;
}

#endif

#if defined(VVC_F3)

/*
 * Set the 'alternate function' of a GPIO pin.
 * Check the datasheet for AF pin mappings.
 */
void pGPIO::set_af(int gpio_af) {
  if (status == 0) { return; }
  if (pin < 8) {
    bank->AFR[0] &= ~(0xF     << (pin * 4));
    bank->AFR[0] |=  (gpio_af << (pin * 4));
  }
  else {
    bank->AFR[1] &= ~(0xF     << ((pin - 8) * 4));
    bank->AFR[1] |=  (gpio_af << ((pin - 8) * 4));
  }
}

#endif

pLED::pLED() : pGPIO() {}

#if    defined(VVC_F3)

pLED::pLED(GPIO_TypeDef* pin_bank, int pin_num) :
      pGPIO(pin_bank, pin_num, 0x01, 0x00, 0x00, 0x00) {}

#elif  VVC_F1

pLED::pLED(GPIO_TypeDef* pin_bank, int pin_num) :
      pGPIO(pin_bank, pin_num, 0x02, 0x00) {}

#endif

void pLED::on() {
  bank->ODR |=  (1 << pin);
}

void pLED::off() {
  bank->ODR &= ~(1 << pin);
}

void pLED::toggle() {
  bank->ODR ^=  (1 << pin);
}

pI2C::pI2C() { status = 0; }

pI2C::pI2C(I2C_TypeDef* i2c_regs,
           GPIO_TypeDef* scl_bank, int scl_pin,
           GPIO_TypeDef* sda_bank, int sda_pin
           #if defined(VVC_F3)
           , int gpio_af
           #endif
           ) {
  I2Cx = i2c_regs;
  // Initialize the SCL and SDA pins.
  #if defined(VVC_F3)
    scl = pGPIO(scl_bank, scl_pin, 0x2, 0x1, 0x0, 0x1);
    sda = pGPIO(sda_bank, sda_pin, 0x2, 0x1, 0x0, 0x1);
    scl.set_af(gpio_af);
    sda.set_af(gpio_af);
  #elif  VVC_F1
    scl = pGPIO(scl_bank, scl_pin, 0x2, 0x3);
    sda = pGPIO(sda_bank, sda_pin, 0x2, 0x3);
  #endif
  status = 1;
}

pI2C::pI2C(I2C_TypeDef* i2c_regs,
           pGPIO scl_gpio, pGPIO sda_gpio) {
  I2Cx   = i2c_regs;
  scl    = scl_gpio;
  sda    = sda_gpio;
  status = 1;
}

#if   defined(VVC_F3)

/*
 * Initialize the I2C interface for chips with the
 * newer of ST's I2C peripherals.
 */
void pI2C::init(uint32_t timing_register) {
  if (status == 0) { return; }
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
  I2Cx->TIMINGR |=  (timing_register);
  // Enable the peripheral.
  I2Cx->CR1     |=  I2C_CR1_PE;
  status = 2;
}

/*
 * Set the address of the device to communicate with.
 * TODO: Add a 'read/write' option.
 */
void pI2C::set_addr(uint8_t addr) {
  // Set the device address.
  I2Cx->CR2 &= ~(I2C_CR2_SADD);
  I2Cx->CR2 |=  (addr << I2C_CR2_SADD_Pos);
}

/*
 * Set the number of bytes to transmit or receive.
 */
void pI2C::set_num_bytes(uint8_t nbytes) {
  // Set number of bytes to process in the next transmission.
  I2Cx->CR2 &= ~(I2C_CR2_NBYTES);
  I2Cx->CR2 |=  (nbytes << I2C_CR2_NBYTES_Pos);
}

/*
 * Send a 'start' condition on the bus.
 */
void pI2C::start() {
  I2Cx->CR2 |=  (I2C_CR2_START);
  while ((I2Cx->CR2 & I2C_CR2_START)) {};
}

/*
 * Send a 'stop' condition on the bus.
 */
void pI2C::stop() {
  // Send 'Stop' condition, and wait for acknowledge.
  I2Cx->CR2 |=  (I2C_CR2_STOP);
  while ((I2Cx->CR2 & I2C_CR2_STOP)) {}
  // Reset the ICR ('Interrupt Clear Register') event flag.
  I2Cx->ICR |=  (I2C_ICR_STOPCF);
  while ((I2Cx->ICR & I2C_ICR_STOPCF)) {}
  // Ensure that the 'RELOAD' flag is un-set.
  I2Cx->CR2 &= ~(I2C_CR2_RELOAD);
}

#elif VVC_F1

/*
 * Initialize the I2C peripheral for chips using
 * the older of ST's I2C peripherals.
 */
void pI2C::init(uint8_t freq_bits, uint8_t ccr_bits) {
  if (status == 0) { return; }
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
  I2Cx->CR2     |=  (freq_bits << I2C_CR2_FREQ_Pos);
  // Set the CCR bits to...uh...TODO: calculte an actual value.
  I2Cx->CCR     &= ~(I2C_CCR_CCR);
  I2Cx->CCR     |=  (I2C_CCR_FS | ccr_bits);
  // Enable the peripheral.
  I2Cx->CR1     |=  (I2C_CR1_PE);
  status = 2;
}

/*
 * Send a 'start' condition and send the device address;
 * in the older F1 peripheral, the 'start' condition is
 * sent first and implicitly sets the peripheral to
 * 'master' mode, so that needs to be done before the
 * address is set.
 * The 'rw' byte should be either 0 for a write or 1 for a read.
 */
void pI2C::start_with_addr(uint8_t addr, uint8_t rw) {
  // Generate a start condition to set the chip as a host.
  I2Cx->CR1 |=  (I2C_CR1_START);
  while (!(I2Cx->SR1 & I2C_SR1_SB)) {};
  // Wait for the peripheral to update its role.
  while (!(I2Cx->SR2 & I2C_SR2_MSL)) {};
  // Set the device address; 7-bits followed by an R/W bit.
  // Currently, we only ever write so just use 0 for R/W.
  I2Cx->DR   =  (addr);
  // Wait for address to match.
  while (!(I2Cx->SR1 & I2C_SR1_ADDR)) {};
  // Read SR2 to clear the ADDR flag.
  (void) I2Cx->SR2;
}

/*
 * Send a 'stop' condition.
 */
void pI2C::stop() {
  // Send 'Stop' condition, and wait for acknowledge.
  I2Cx->CR1 |=  (I2C_CR1_STOP);
  while (I2Cx->SR2 & I2C_SR2_MSL) {};
}

#endif

/*
 * (These methods describe behavior common to all
 *  supported I2C peripherals.)
 */
/*
 * Transmit a single byte.
 */
void pI2C::write_byte(uint8_t wb) {
  #ifdef VVC_F1
    // Transmit a byte of data, and wait for it to send.
    I2Cx->DR   = (I2Cx->DR & 0xFF00) |  wb;
    while (!(I2Cx->SR1 & I2C_SR1_TXE)) {};
  #elif  VVC_F3
    // Transmit a byte of data, and wait for it to send.
    I2Cx->TXDR = (I2Cx->TXDR & 0xFFFFFF00) | wb;
    while (!(I2Cx->ISR & (I2C_ISR_TXIS |
                          I2C_ISR_TC |
                          I2C_ISR_TCR))) {};
  #endif
}

/*
 * Stream a whole bunch of bytes from a buffer.
 * Note: This does not handle setting the device address
 *       or required start/stop conditions.
 */
void pI2C::write_bytes(volatile uint8_t* buf, int len) {
  #if    defined(VVC_F3)
    // The more recent chips have an internal counter to keep
    // track of how many bytes they send/receive, and it's
    // limited to 255 bytes at once, so we need to manage that.
    volatile int pos = 0;
    int nb_i = 0;
    // Ensure that the 'RELOAD' flag is set.
    I2Cx->CR2 |= I2C_CR2_RELOAD;
    // Send 255 bytes at a time, until there's <255B left.
    while ((len - pos) > 255) {
      set_num_bytes(255);
      for (nb_i = 0; nb_i < 255; ++nb_i) {
        write_byte(buf[pos]);
        pos += 1;
      }
    }
    // Stream the remaining bytes.
    if (pos < len) {
      int remaining_bytes = len - pos;
      set_num_bytes(remaining_bytes);
      for (nb_i = 0; nb_i < remaining_bytes; ++nb_i) {
        write_byte(buf[pos]);
        ++pos;
      }
    }
  #elif  VVC_F1
    // The F1 series have a simpler 'transmit' process
    // for longer frames; you just send all of the bytes.
    int nb_i = 0;
    for (nb_i = 0; nb_i < len; ++nb_i) {
      write_byte(buf[nb_i]);
    }
  #endif
}

/*
 * Read a byte of data from the peripheral.
 */
uint8_t pI2C::read_byte() {
  #ifdef VVC_F1
    // TODO
    return 0x00;
  #elif  VVC_F3
    // Wait for a byte of data to be available, then read it.
    while (!(I2Cx->ISR & I2C_ISR_RXNE)) {}
    return (I2Cx->RXDR & 0xFF);
  #endif
}

pSSD1306::pSSD1306() { status = 0; }

pSSD1306::pSSD1306(pI2C* i2c_interface,
                   int display_w, int display_h) {
}

/*
 * Write a 'command byte' to the display.
 * Sending 0x00 as a first byte indicates a command.
 */
void pSSD1306::write_command_byte(uint8_t cmd) {
  #if    defined(VVC_F3)
    i2c->set_addr(0x78);
    i2c->set_num_bytes(2);
    i2c->start();
    i2c->write_byte(0x00);
    i2c->write_byte(cmd);
    i2c->stop();
  #elif  VVC_F1
    i2c->start_with_addr(0x78, 0);
    i2c->write_byte(0x00);
    i2c->write_byte(cmd);
    i2c->stop();
  #endif
}

/*
 * Write a 'data byte' to the display.
 * Sending 0x40 as a first byte indicates that
 * display data will follow.
 */
void pSSD1306::write_data_byte(uint8_t dat) {
  #if    defined(VVC_F3)
    i2c->set_addr(0x78);
    i2c->set_num_bytes(2);
    i2c->start();
    i2c->write_byte(0x40);
    i2c->write_byte(dat);
    i2c->stop();
  #elif  VVC_F1
    i2c->start_with_addr(0x78, 0);
    i2c->write_byte(0x40);
    i2c->write_byte(dat);
    i2c->stop();
  #endif
}

pSSD1306_128x64::pSSD1306_128x64() : pSSD1306() {}

pSSD1306_128x64::pSSD1306_128x64(pI2C* i2c_interface) :
                 pSSD1306(i2c_interface, 128, 64) {
  // Initialize the framebuffer to 0's.
  int fb_i;
  for (fb_i = 0; fb_i < SSD1306_128x64_FB_SIZE; ++fb_i) {
    framebuffer[fb_i] = 0x00;
  }
}

/*
 * Initialize a 128x64-pixel SSD1306 display.
 */
void pSSD1306_128x64::init_display() {
  // Display clock division
  write_command_byte(0xD5);
  write_command_byte(0x80);
  // Set multiplex
  write_command_byte(0xA8);
  write_command_byte(0x3F);
  // Set display offset ('start column')
  write_command_byte(0xD3);
  write_command_byte(0x00);
  // Set start line (0b01000000 | line)
  write_command_byte(0x40);
  // Set internal charge pump (on)
  write_command_byte(0x8D);
  write_command_byte(0x14);
  // Set memory mode
  write_command_byte(0x20);
  write_command_byte(0x00);
  // Set 'SEGREMAP'
  write_command_byte(0xA1);
  // Set column scan (descending)
  write_command_byte(0xC8);
  // Set 'COMPINS'
  write_command_byte(0xDA);
  write_command_byte(0x12);
  // Set contrast
  write_command_byte(0x81);
  write_command_byte(0xCF);
  // Set precharge
  write_command_byte(0xD9);
  write_command_byte(0xF1);
  // Set VCOM detect
  write_command_byte(0xDB);
  write_command_byte(0x40);
  // Set output to follow RAM content
  write_command_byte(0xA4);
  // Normal display mode
  write_command_byte(0xA6);
  // Display on
  write_command_byte(0xAF);
}

/*
 * Draw the entire framebuffer to a 128x64-pixel SSD1306 display.
 */
void pSSD1306_128x64::stream_fb() {
  #if    defined(VVC_F3)
    // Set the 'RELOAD' flag.
    i2c->I2Cx->CR2 |= I2C_CR2_RELOAD;
    // Set the device address and send a 'start' condition.
    i2c->set_addr(0x78);
    i2c->start();
    // Send one byte to set a 'data' transmission.
    i2c->write_byte(0x40);
    // Send the rest of the framebuffer.
    i2c->write_bytes(framebuffer, SSD1306_128x64_FB_SIZE);
    // Send a 'stop' condition.
    i2c->stop();
  #elif  VVC_F1
    // Start with the display's address.
    i2c->start_with_addr(0x78, 0);
    // Set a 'data' transmission.
    i2c->write_byte(0x40);
    // Stream the framebuffer.
    i2c->write_bytes(framebuffer, SSD1306_128x64_FB_SIZE);
    // Send a 'stop' condition.
    i2c->stop();
  #endif
}
